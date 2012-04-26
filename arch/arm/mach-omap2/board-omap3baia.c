/*
 * linux/arch/arm/mach-omap2/board-omap3baia.c
 *
 * Copyright (C) 2010 Bticino S.p.A.
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/at24.h>
#include <linux/usb/otg.h>

#include <linux/wl12xx.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>

#include <linux/backlight.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/mcspi.h>
#include <plat/display.h>
#include <plat/omap-pm.h>
#include <linux/moduleparam.h>

#include <linux/i2c/tda9885.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "hsmmc.h"
#include "board-omap3baia.h"


static u8 omap3_baia_version;

/* For testing purpose with different hardware */
static unsigned int baia_lcd = 0;
module_param(baia_lcd, uint, S_IRUGO);
MODULE_PARM_DESC(baia_lcd, "LCD: 0:AMPIRE (default), 1:AUO");

/* BOARD: reading revision */
u8 get_omap3_baia_rev(void)
{
	return omap3_baia_version;
}
EXPORT_SYMBOL(get_omap3_baia_rev);

static void __init omap3_baia_get_revision(void)
{
	/* TODO */
}

/* DEVICE: ADS7846 touchpad */
static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_BAIA_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");
	gpio_direction_input(OMAP3_BAIA_TS_GPIO);
	gpio_set_debounce(OMAP3_BAIA_TS_GPIO, 310);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_BAIA_TS_GPIO);
}

static struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 450,
	.pressure_max		= 20 * 1024,
	.debounce_max		= 20,
	.debounce_tol		= 30,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 1500,
	.wakeup			= true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3_baia_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_BAIA_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

/* DEVICE: BACKLIGHT */
static struct gpio_led gpio_leds[] = {
	[OMAP3_BAIA_LED_BACKLIGHT] = {
		.name                   = "omap3baia::backlight",
		/* normally not visible (board underside) */
		.default_trigger        = "default-on",
		.gpio                   = -EINVAL,      /* gets replaced */
		.active_low             = false,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds           = gpio_leds,
	.num_leds       = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name   = "leds-gpio",
	.id     = -1,
	.dev    = {
		.platform_data  = &gpio_led_info,
	},
};

/* DEVICE: Ethernet Micrel ks8851_mll support through GPMC */
#define OMAP3BAIA_KS8851_MLL_DATA 0x2c000000
#define OMAP3BAIA_KS8851_MLL_CMD  0x2c000002
#define OMAP3BAIA_KS8851_MLL_SIZE       1024

static u64 baia_ks8851_mll_dmamask = 0xffffffff;

static struct resource baia_ks8851_mll_resources[] = {
	[0] =	{
		.start          = OMAP3BAIA_KS8851_MLL_DATA,
		.end            = OMAP3BAIA_KS8851_MLL_DATA + 0x1,
		.flags          = IORESOURCE_MEM,
	},
	[1] =	{
		.start          = OMAP3BAIA_KS8851_MLL_CMD,
		.end            = OMAP3BAIA_KS8851_MLL_CMD + 0x1,
		.flags          = IORESOURCE_MEM,
	},
	[2] =	{
		.start	= OMAP_GPIO_IRQ(OMAP3_BAIA_KS8851_MLL_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(OMAP3_BAIA_KS8851_MLL_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct platform_device platform_baia_ks8851_mll = {
	.name           = "ks8851_mll",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(baia_ks8851_mll_resources),
	.resource       = &baia_ks8851_mll_resources[0],
	.dev = {
		.dma_mask       = &baia_ks8851_mll_dmamask,
	},
};

static void omap3baia_init_ks8851_mll(void)
{
	struct clk *l3ck;
	unsigned int rate;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpio_request(OMAP3_BAIA_KS8851_MLL_GPIO_IRQ, "kS8851_MLL irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for KS8851_MLL IRQ\n",
			OMAP3_BAIA_KS8851_MLL_GPIO_IRQ);
		return;
	}
	gpio_direction_input(OMAP3_BAIA_KS8851_MLL_GPIO_IRQ);

	if (gpio_request(OMAP3_BAIA_KS8851_MLL_RESET, "kS8851_MLL reset") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for KS8851_MLL IRQ\n",
			OMAP3_BAIA_KS8851_MLL_RESET);
		return;
	}
	gpio_direction_output(OMAP3_BAIA_KS8851_MLL_RESET,1);
	platform_device_register(&platform_baia_ks8851_mll);
}
EXPORT_SYMBOL(omap3baia_init_ks8851_mll);

/* BOARD: LCD Panel control signals */
static void __init omap3_baia_display_init(void)
{
	int r;

	r = gpio_request(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8,
			 "lcd_panel_envideo_1v8");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_envideo_1v8\n");
		goto err;
	}
	gpio_direction_output(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8, 1);
	return;

err:
	gpio_free(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8);
}

#if defined(CONFIG_PANEL_CPT_CLAA102NA0DCW) || defined(CONFIG_PANEL_AUO_B101EW05)

#define TWL_LED_LEDEN           0x00
#define TWL_PWMA_PWMAON         0x00
#define TWL_PWMA_PWMAOFF        0x01

static int omap3_baia_set_bl_intensity_lvds(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;

	printk("%s-%d\n", __func__, __LINE__);

	if (level > dssdev->max_backlight_level)
		level = dssdev->max_backlight_level;

	c = ((125 * (100 - level)) / 100);
	switch (baia_lcd) {
	case 0:
	printk("%s-%d\n", __func__, __LINE__);
		/* CPT_CLAA102NA0DCW compatible regulations */
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMA_PWMAON);
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x40, TWL_PWMA_PWMAOFF);
		twl_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_LEDEN);
		break;
	case 1:
	printk("%s-%d\n", __func__, __LINE__);
		twl_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_LEDEN);
	        twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMA_PWMAON);
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x02, TWL_PWMA_PWMAOFF);
		/* AUO_B101EW05 regulations */
		break;
	}
	return 0;
}
#endif

static int omap3_baia_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8, 1);
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
	return 0;
}

static void omap3_baia_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8, 0);
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

#ifdef CONFIG_PANEL_CPT_CLAA102NA0DCW
/* LVDS */
static struct omap_dss_device omap3_baia_lcd_device_claa = {
	.name			= "lcd",
	.driver_name		= "claa102na0dcw",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.max_backlight_level	= 100,
	.platform_enable	= omap3_baia_enable_lcd,
	.platform_disable	= omap3_baia_disable_lcd,
	.set_backlight		= omap3_baia_set_bl_intensity_lvds,
};
#endif
#ifdef CONFIG_PANEL_AUO_B101EW05
/* LVDS */
static struct omap_dss_device omap3_baia_lcd_device_auo = {
	.name			= "lcd",
	.driver_name		= "b101ew05",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= omap3_baia_enable_lcd,
	.platform_disable	= omap3_baia_disable_lcd,
	.set_backlight		= omap3_baia_set_bl_intensity_lvds,
};
#endif

static int omap3_baia_enable_tv(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 400000);
	return 0;
}

static void omap3_baia_disable_tv(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

static struct omap_dss_device omap3_baia_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= omap3_baia_enable_tv,
	.platform_disable	= omap3_baia_disable_tv,
};

static struct omap_dss_device omap3_baia_lcd_device;

static struct omap_dss_device *omap3_baia_dss_devices[] = {
	&omap3_baia_tv_device,
	&omap3_baia_lcd_device,
};

static struct omap_dss_board_info omap3_baia_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3_baia_dss_devices),
	.devices	= omap3_baia_dss_devices,
	.default_device	= &omap3_baia_tv_device,
};

static struct platform_device omap3_baia_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3_baia_dss_data,
	},
};

static struct regulator_consumer_supply omap3baia_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply omap3baia_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct regulator_consumer_supply omap3baia_vmmc2_supply = {
	.supply			= "vmmc",
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data omap3baia_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3baia_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data omap3baia_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap3baia_vmmc2_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data omap3baia_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3baia_vsim_supply,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.name		= "SD_card",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL, /* gpio 0 : tps65930 */
		.gpio_wp	= OMAP3_BAIA_SDCARD_WR_PR,
	},
        {
		.name		= "eMMC",
                .mmc            = 2,
                .caps           = MMC_CAP_4_BIT_DATA,
                .gpio_cd        = -EINVAL,
                .gpio_wp        = -EINVAL,
                .nonremovable   = true,
        },
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
		.name		= "wl1271",
		.mmc		= 3, /*VERIFICA ALIM, diversa posizione su mmc rispetto s schedsl */
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
	},
#endif
	{}	/* Terminator */
};

static int gpio_video_dec_noff, gpio_video_dec_nres;

static int omap3_baia_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	printk("%s-%d gpio=%d\n", __func__, __LINE__, gpio);

	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
//	omap_mux_init_gpio(129, OMAP_PIN_INPUT);
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	omap3baia_vmmc1_supply.dev = mmc[0].dev;
	omap3baia_vsim_supply.dev = mmc[0].dev;
	omap3baia_vmmc2_supply.dev = mmc[1].dev;

	/*
	 * Most GPIOs are for USB OTG.  Some are mostly sent to
	 * the P2 connector; notably LEDA for the LCD backlight.
	 */

	/* gpio + 1 == Audio Codec Reset */
	gpio_request(gpio + 1, "aud-cod #reset");
	gpio_direction_output(gpio + 1, 1);

	/* gpio + 7 == PAL decoder Enable */
	gpio_video_dec_noff = gpio + 7;
	gpio_request(gpio_video_dec_noff, "vid-dec #off");
	gpio_direction_output(gpio_video_dec_noff, 1); // TODO now is ALWAYS ON

	/* gpio + 15 == PAL decoder Reset */
	gpio_video_dec_nres = gpio + 15;
	printk("%s-%d gpio_video_dec_nres=%d\n", __func__, __LINE__, gpio_video_dec_nres);
	gpio_request(gpio_video_dec_nres, "vid-dec #reset"); 
	// TODO now is always on
	gpio_direction_output(gpio_video_dec_nres, 0);
	mdelay(10);
	gpio_direction_output(gpio_video_dec_nres, 1);
	mdelay(10);

	printk("%s-%d TVP: NRES=%d,NOFF=%d\n", __func__, __LINE__, gpio_get_value(gpio_video_dec_nres), gpio_get_value(gpio_video_dec_noff));

	platform_device_register(&leds_gpio);
	return 0;
}

static struct twl4030_gpio_platform_data omap3baia_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3_baia_twl_gpio_setup,
};

static struct twl4030_usb_data omap3baia_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_F1),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3baia_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 1,
	.cols		= 1,
	.rep		= 1,
};

static struct twl4030_madc_platform_data omap3baia_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data omap3baia_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data omap3baia_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap3baia_audio_data,
};

static struct regulator_consumer_supply omap3baia_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &omap3_baia_dss_device.dev,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data omap3baia_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3baia_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply omap3baia_vpll2_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static struct regulator_init_data omap3baia_vpll2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3baia_vpll2_supply,
};

/* VIO_1V8 is required for diff modules: ads7846 on SPI, pull-ups, etc... */
static struct regulator_consumer_supply omap3baia_vio_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
	REGULATOR_SUPPLY("vio_1v8", NULL),
};

static struct regulator_init_data omap3baia_vio = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(omap3baia_vio_supply),
	.consumer_supplies      = omap3baia_vio_supply,
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA

static struct regulator_consumer_supply omap3baia_vmmc3_supply = {
	.supply		= "vmmc",
	.dev_name	= "mmci-omap-hs.2",
};

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data omap3baia_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &omap3baia_vmmc3_supply,
};

static struct fixed_voltage_config omap3baia_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.80V */
	.gpio			= OMAP3_BAIA_WLAN_PMENA_GPIO,
	.startup_delay		= 200000, /* 70ms */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &omap3baia_vmmc3,
};

static struct platform_device omap3baia_wlan_regulator = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &omap3baia_vwlan,
	},
};

struct wl12xx_platform_data omap3baia_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(OMAP3_BAIA_WLAN_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38, /* 38.4 MHz */
};
#endif

static struct twl4030_platform_data omap3_baia_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &omap3baia_kp_data,
	.madc		= &omap3baia_madc_data,
	.usb		= &omap3baia_usb_data,
	.gpio		= &omap3baia_gpio_data,
	.codec		= &omap3baia_codec_data,
	.vdac		= &omap3baia_vdac,
	.vpll2		= &omap3baia_vpll2,
	.vio		= &omap3baia_vio,
	.vmmc1          = &omap3baia_vmmc1,
	.vmmc2          = &omap3baia_vmmc2,
	.vsim           = &omap3baia_vsim,
};

static struct at24_platform_data eeprom_info = {
        .byte_len       = (256*1024) / 8,
        .page_size      = 64,
        .flags          = AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata omap3_baia_twl_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3_baia_twldata,
	},
};

static struct tda9885_platform_data omap3baia_tda9885_platform_data = {
	.switching_mode = 0xf2,
	.adjust_mode = 0xd0,
	.data_mode = 0x0b,
	.power = OMAP3_BAIA_ABIL_DEM_VIDEO1V8,
};

static struct i2c_board_info __initdata omap3_baia_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c256", 0x53),
		.platform_data  = &eeprom_info,
	},
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
	{
		I2C_BOARD_INFO("tda9885", 0x43),
		.platform_data  = &omap3baia_tda9885_platform_data,
	},
};

static int __init omap3_baia_i2c_init(void)
{
	/*
	 * REVISIT: These entries can be set in omap3baia_twl_data
	 * after a merge with MFD tree
	 */
//	omap3_baia_twldata.vmmc1 = &omap3baia_vmmc1;
//	omap3_baia_twldata.vsim = &omap3baia_vsim;

	omap_register_i2c_bus(1, 2600, omap3_baia_twl_i2c_boardinfo,
			ARRAY_SIZE(omap3_baia_twl_i2c_boardinfo));
	/* Bus 2 is used for Camera/Sensor interface */
	omap_register_i2c_bus(2, 100, omap3_baia_i2c_boardinfo,
			ARRAY_SIZE(omap3_baia_i2c_boardinfo));
	return 0;
}



static struct omap_board_config_kernel omap3_baia_config[] __initdata = {
};

static void __init omap3_baia_init_irq(void)
{
	omap_board_config = omap3_baia_config;
	omap_board_config_size = ARRAY_SIZE(omap3_baia_config);
	omap2_init_common_infrastructure();
	if (cpu_is_omap3630())
		omap2_init_common_devices(mt46h32m32lf6_sdrc_params, NULL);
	omap_init_irq();
}

static struct platform_device *omap3_baia_devices[] __initdata = {
	&omap3_baia_dss_device,
};

/* USB HOST */

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	/* PHY reset GPIO will be runtime programmed based on EVM version */
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

/* MUX */

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux omap36x_board_mux[] __initdata = {
#ifdef CONFIG_KEYBOARD_TWL4030
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
			OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
			OMAP_PIN_OFF_WAKEUPENABLE),
#endif
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	OMAP3_MUX(CAM_D9, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
			OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
			OMAP_PIN_OFF_WAKEUPENABLE), /* gpio_108 */
#endif
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT0, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT1, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT3, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT4, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT6, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_CLKOUT2, OMAP_MUX_MODE0 | OMAP_PIN_OFF_NONE),

	/* MCSPI4 */
	OMAP3_MUX(MCBSP1_DR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),

	/* Video demodulator for SCS video in path */
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

#ifdef CONFIG_OMAP_MCBSP
	/* For MCBSP3 */
	OMAP3_MUX(MCBSP1_CLKX, OMAP_MUX_MODE2 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(MCBSP1_FSX, OMAP_MUX_MODE2 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(MCBSP3_DX, OMAP_MUX_MODE0 | OMAP_PIN_OFF_NONE),
#endif
	/* MMC2 SDIO pin muxes for eMMC */
	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_clk */
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_cmd */
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat0 */
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat1 */
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat2 */
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat3 */
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat4 */
	OMAP3_MUX(SDMMC2_DAT5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat5 */
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat6 */
	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), /* sdmmc2_dat7 */

	/* TODO: check Bluetooth
	 * UART1_TX (GPIO_148)
	 * UART1_RTS (GPIO_149)
	 * UART1_CTS(GPIO_150)
	 * UART1_RX (GPIO_151)
	 * tutti in mode0
	 * enable: GPIO6 del TPS65930
	 * dovrebbe essere conteggiato come GPIO_198
	 */
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WLAN IRQ - GPIO 70 */
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

	/* WLAN POWER ENABLE - GPIO 184 */
	OMAP3_MUX(I2C3_SCL, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* MMC3 SDIO pin muxes for WL12xx */
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP), /* sdmmc3_clk */
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP), /* sdmmc3_cmd */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP), /* sdmmc3_dat3 */
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP), /* sdmmc3_dat0 */
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP), /* sdmmc3_dat1 */
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP), /* sdmmc3_dat2 */
#endif

	/* For UART2 */
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1 | OMAP_PIN_OFF_NONE), /* uart2_tx */
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT), /* uart2_rx */

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define omap36x_board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void __init omap3_baia_init(void)
{
	omap3_baia_get_revision();

	if (cpu_is_omap3630()) {
		omap3_mux_init(omap36x_board_mux, OMAP_PACKAGE_CUS);
	} else {
		printk(KERN_ERR "WRONG CPU .. STOPPING\n");
		for (;;);
	}

	printk(KERN_INFO "baia_lcd=%d\n", baia_lcd);
	switch (baia_lcd) {
	case 0:
		printk(KERN_INFO "LCD CLAA/CPT/AMPIRE OVERRIDING\n");
		memcpy(&omap3_baia_lcd_device,&omap3_baia_lcd_device_claa, sizeof(struct omap_dss_device));
		break;
	case 1:
	default:
		printk(KERN_INFO "LCD AUO OVERRIDING\n");
		memcpy(&omap3_baia_lcd_device,&omap3_baia_lcd_device_auo, sizeof(struct omap_dss_device));
		break;
	}

	platform_add_devices(omap3_baia_devices, ARRAY_SIZE(omap3_baia_devices));

	spi_register_board_info(omap3_baia_spi_board_info,
				ARRAY_SIZE(omap3_baia_spi_board_info));

	omap_serial_init();

	/*
	 * Enable USB
	 */

	/* setup EHCI phy reset config */
	omap_mux_init_gpio(65, OMAP_PIN_INPUT_PULLUP);
	usb_ehci_init(&ehci_pdata);

	usb_musb_init(&musb_board_data);

	ads7846_dev_init();
	omap3baia_init_ks8851_mll();
	omap3_baia_display_init();

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&omap3baia_wlan_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&omap3baia_wlan_regulator);
#endif

	/*
	 * TODO CHECK:
	 * ds1803 must be checked after the LCD is powered up ?
	 */
	omap3_baia_i2c_init();
}

MACHINE_START(OMAP3_BAIA, "OMAP3 BAIA")
	/* Maintainer: Raffaele Recalcati : Bticino S.p.A. */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_baia_init_irq,
	.init_machine	= omap3_baia_init,
	.timer		= &omap_timer,
MACHINE_END
