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
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>

#include <linux/spi/spi.h>
#include <linux/spi/tsc2005.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/twl4030-madc.h>
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
#include <linux/memory.h>
#include <linux/ks8851_mll.h>

#include <linux/spi/zl38005.h>
#include <sound/tpa2016d2-plat.h>
#include <sound/tlv320aic3x.h>
#include <linux/mcp453x.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "hsmmc.h"
#include "board-omap3baia.h"

static struct ks8851_mll_platform_data platform_baia_ks8851_mll_pdata;
static struct twl4030_madc_platform_data omap3baia_madc_data;

void baia_get_mac_addr(struct memory_accessor *mem_acc, void *context)
{
	char *mac_addr = platform_baia_ks8851_mll_pdata.mac_addr;
	off_t offset = (off_t)context;
	/* Read MAC addr from EEPROM */
	if (mem_acc->read(mem_acc, mac_addr, offset, ETH_ALEN) == ETH_ALEN)
		pr_info("Read MAC addr from EEPROM: %pM\n", mac_addr);
}

/* For testing purpose with different hardware */
static unsigned int baia_gpio_debug = 1;
module_param(baia_gpio_debug, uint, S_IRUGO);
MODULE_PARM_DESC(baia_gpio_debug, "0:disabled, 1:enabled (default)");

/* For testing purpose with different hardware */
static unsigned int baia_lcd;
module_param(baia_lcd, uint, S_IRUGO);
MODULE_PARM_DESC(baia_lcd, "LCD: 0:AMPIRE (default), 1:AUO");

/* Getting hardware version from u-boot */
static unsigned int baia_dig_board_rev;
module_param(baia_dig_board_rev, uint, S_IRUGO);
MODULE_PARM_DESC(baia_dig_board_rev, "digital board revision");

/* Getting hardware version from u-boot */
static unsigned int baia_con_board_rev;
module_param(baia_con_board_rev, uint, S_IRUGO);
MODULE_PARM_DESC(baia_con_board_rev, "connector board revision");

#define U7503A_U7504A_HWversion

static void baia_get_baia_rev(struct work_struct *work);
static DECLARE_DELAYED_WORK(late_init_work, baia_get_baia_rev);

/* Reading Baia board revision from tps65951 madc in2 */
static void baia_get_baia_rev(struct work_struct *work)
{
	int ret;

	while (1) {
		ret = twl4030_get_madc_conversion(2);
		if (ret >= 0)
			break;
		msleep(500);
	}
	system_serial_low = (ret + 5) / 10;
}

/* DEVICE: TSC2005 touchpad */
static struct tsc2005_platform_data tsc2005_pdata = {
	.ts_pressure_max        = 2048,
	.ts_pressure_fudge      = 2,
	.ts_x_max               = 4096,
	.ts_x_fudge             = 4,
	.ts_y_max               = 4096,
	.ts_y_fudge             = 7,
	.ts_x_plate_ohm         = 700,
	.esd_timeout_ms         = 8000,
};

static void baia_tsc2005_set_reset(bool enable)
{
	gpio_set_value(OMAP3_BAIA_TS_RESET, enable);
}

static void tsc2005_dev_init(void)
{
	if (gpio_request(OMAP3_BAIA_TS_NPENIRQ, "TSC2005 pendown") < 0)
		printk(KERN_ERR "can't get tsc2005 pen down GPIO\n");
	gpio_direction_input(OMAP3_BAIA_TS_NPENIRQ);
	gpio_set_debounce(OMAP3_BAIA_TS_NPENIRQ, 310);

	if (gpio_request(OMAP3_BAIA_TS_RESET, "TSC2005 reset") < 0)
		printk(KERN_ERR "can't get tsc2005 reset\n");
	gpio_direction_output(OMAP3_BAIA_TS_RESET, 1);

	tsc2005_pdata.set_reset = baia_tsc2005_set_reset;
}

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct zl38005_platform_data zl_config[] = {
	[0] = {
		.minor = 0,
		.reset_gpio = OMAP3_BAIA_RES_ZL1_1V8,
	},
	[1] = {
		.minor = 1,
		.reset_gpio = OMAP3_BAIA_RES_ZL2_1V8,
	}
};

static struct spi_board_info omap3_baia_spi_board_info[] = {
	[0] = {
		.modalias		= "tsc2005",
		.bus_num		= 1,
		.chip_select		= 0,
		.irq			= OMAP_GPIO_IRQ(OMAP3_BAIA_TS_NPENIRQ),
		.max_speed_hz		= 6000000,
		.controller_data	= &tsc2005_mcspi_config,
		.platform_data		= &tsc2005_pdata,
	},
	[1] = {
		.modalias		= "zl38005",
		.bus_num		= 4,
		.chip_select_gpio	= OMAP3_BAIA_NCS5A_ZL1_CS,
		.chip_select		= 0,
		.max_speed_hz		= 2 * 1000 * 1000,
		.mode			= SPI_MODE_0,
		.platform_data          = &zl_config[0],
	},
	[2] = {
		.modalias		= "zl38005",
		.bus_num		= 4,
		.chip_select_gpio	= OMAP3_BAIA_NCS4A_ZL2_CS,
		.chip_select		= 1,
		.max_speed_hz		= 2 * 1000 * 1000,
		.mode			= SPI_MODE_0,
		.platform_data          = &zl_config[1],
	},
};

/* DEVICE: BACKLIGHT */
static struct gpio_led gpio_leds[] = {
	[OMAP3_BAIA_VLED_EN_1V8] = {
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
		.platform_data  = &platform_baia_ks8851_mll_pdata,
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

	if (gpio_request(OMAP3_BAIA_KS8851_MLL_GPIO_IRQ,
			 "kS8851_MLL irq") < 0) {
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
	gpio_direction_output(OMAP3_BAIA_KS8851_MLL_RESET, 0);
	mdelay(10);
	gpio_direction_output(OMAP3_BAIA_KS8851_MLL_RESET, 1);

	/* The mac address is read from the at24 eeprom automatically */

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

	msleep(200);

	r = gpio_request(OMAP3_BAIA_SHTD_VIDEO_1V8,
			 "lcd_panel_shtd_video_1v8");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_shtd_video_1v8\n");
		goto err;
	}
	gpio_direction_output(OMAP3_BAIA_SHTD_VIDEO_1V8, 1);

	r = gpio_request(OMAP3_BAIA_EN_BL,
			 "lcd_panel_en_bl");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_en_bl\n");
		goto err;
	}
	gpio_direction_output(OMAP3_BAIA_EN_BL, 1);

	/* backlight */
	r = gpio_request(OMAP3_BAIA_VLED_EN_1V8,
			 "lcd_panel_vled_en_1v8");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_vled_en_1v8\n");
		goto err;
	}
	gpio_direction_output(OMAP3_BAIA_VLED_EN_1V8, 1);

	return;

err:
	gpio_free(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8);
}

#if defined(CONFIG_PANEL_CPT_CLAA102NA0DCW) || \
	    defined(CONFIG_PANEL_AUO_B101EW05)

#define TWL_LED_LEDEN           0x00
#define TWL_PWMA_PWMAON         0x00
#define TWL_PWMA_PWMAOFF        0x01

static int omap3_baia_set_bl_intensity_lvds(struct omap_dss_device *dssdev,
					    int level)
{
	unsigned char c;

	if (level > dssdev->max_backlight_level)
		level = dssdev->max_backlight_level;

	c = ((125 * (100 - level)) / 100);
	switch (baia_lcd) {
	case 0:
		/* CPT_CLAA102NA0DCW compatible regulations */
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMA_PWMAON);
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x40, TWL_PWMA_PWMAOFF);
		twl_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_LEDEN);
		break;
	case 1:
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

	gpio_set_value(OMAP3_BAIA_EN_BL, 1);
	return 0;
}

static void omap3_baia_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3_BAIA_EN_BL, 0);
	gpio_set_value(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8, 0);
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

#ifdef CONFIG_PANEL_CPT_CLAA102NA0DCW
/* LVDS */
static struct omap_dss_device omap3_baia_lcd_device_claa = {
	.name			= "lcd",
	.driver_name		= "claa102na0dcw",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 18,
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
	&omap3_baia_lcd_device,
	&omap3_baia_tv_device,
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
		.caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
		.nonremovable   = true,
	},
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
		.name		= "wl1271",
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
	},
#endif
	{}	/* Terminator */
};

static int omap3_baia_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) and is equal to 192
	 * and to OMAP3_BAIA_TPS_OFFSET and to
	 * OMAP3_BAIA_TPS_SD_CARD_DET
	 */
	mmc[0].gpio_cd = OMAP3_BAIA_TPS_SD_CARD_DET;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	omap3baia_vmmc1_supply.dev = mmc[0].dev;
	omap3baia_vsim_supply.dev = mmc[0].dev;
	omap3baia_vmmc2_supply.dev = mmc[1].dev;

	/* See omap3_baia_gpio_keys for the following keys
	 * OMAP3_BAIA_TPS_PULS1,
	 * OMAP3_BAIA_TPS_PULS2,
	 * OMAP3_BAIA_TPS_PULS3,
	 * OMAP3_BAIA_TPS_PULS4,
	 */

	platform_device_register(&leds_gpio);
	return 0;
}

static struct twl4030_gpio_platform_data omap3baia_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3_baia_twl_gpio_setup,
	.mmc_cd		= 1,
	.debounce	= (BIT(7) |		/* key 1 -- gpio + 7  */
			  (BIT(5) << 8) |	/* key 2 -- gpio + 13 */
			   BIT(2) |		/* key 3 -- gpio + 2  */
			   BIT(1)),		/* key 4 -- gpio + 1  */
};

static struct twl4030_usb_data omap3baia_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_madc_platform_data omap3baia_madc_data = {
	.irq_line       = 1,
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
	/* tlv320aic3x digital supplies */
	REGULATOR_SUPPLY("IOVDD", "2-0018"),
	REGULATOR_SUPPLY("DVDD", "2-0018"),
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

static struct platform_device madc_hwmon = {
	.name = "twl4030_madc_hwmon",
	.id = -1,
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
	.madc           = &omap3baia_madc_data,
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

/*
wp_set: set/unset the at24 eeprom write protect
*/
void wp_set(int enable)
{
	gpio_set_value(OMAP3_BAIA_EEPROM_WP, enable);
}

static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup		= baia_get_mac_addr,
	.context	= (void *)0x19e, /* where it gets the mac-address */
	.wpset		= wp_set,
	.wppol		= 0,
};

static struct i2c_board_info __initdata omap3_baia_twl_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3_baia_twldata,
	},
};

static struct tpa2016d2_platform_data omap3baia_tpa2016d2_platform_data = {
	.shutdown_gpio = OMAP3_BAIA_EN_AMPLI,
};

static struct tda9885_platform_data omap3baia_tda9885_platform_data = {
	.switching_mode = 0xf2,
	.adjust_mode = 0xd0,
	.data_mode = 0x0b,
	.power = OMAP3_BAIA_ABIL_DEM_VIDEO1V8,
};

static struct aic3x_pdata omap3baia_aic3x_data = {
	.gpio_reset = OMAP3_BAIA_RES_O_1V8,
};

static struct mcp453x_platform_data omap3baia_mcp453x_2e_data = {
	.volatile_wiper0 = 0x5b,
	.volatile_tcon = 0xf,
};

static struct mcp453x_platform_data omap3baia_mcp453x_2f_data = {
	.volatile_wiper0 = 0x78,
	.volatile_tcon = 0xf,
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
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
		.platform_data  = &omap3baia_aic3x_data,
	},
	{
		I2C_BOARD_INFO("tda9885", 0x43),
		.platform_data  = &omap3baia_tda9885_platform_data,
	},
	{
		I2C_BOARD_INFO("tpa2016d2", 0x58),
		.platform_data  = &omap3baia_tpa2016d2_platform_data,
	},
	{
		I2C_BOARD_INFO("mcp453x", 0x2e),
		.platform_data  = &omap3baia_mcp453x_2e_data,
	},
	{
		I2C_BOARD_INFO("mcp453x", 0x2f),
		.platform_data  = &omap3baia_mcp453x_2f_data,
	},
};

static int __init omap3_baia_i2c_init(void)
{
	/*
	 * REVISIT: These entries can be set in omap3baia_twl_data
	 * after a merge with MFD tree
	 */
#if 0
	omap3_baia_twldata.vmmc1 = &omap3baia_vmmc1;
	omap3_baia_twldata.vsim = &omap3baia_vsim;
#endif

	omap_register_i2c_bus(1, 2600, omap3_baia_twl_i2c_boardinfo,
			ARRAY_SIZE(omap3_baia_twl_i2c_boardinfo));
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

static struct platform_device omap3_baia_cy7c65630 = {
	.name		= "cypress_cy7c65",
	.id		= -1,
};

static struct platform_device *omap3_baia_devices[] __initdata = {
	&omap3_baia_dss_device,
	&madc_hwmon,
	&omap3_baia_cy7c65630,
};

/* USB HOST */

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	/* PHY reset GPIO will be runtime programmed based on board version */
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = OMAP3_BAIA_EHCIPHYRESET,
	.reset_gpio_port[2]  = -EINVAL
};

/* MUX */

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux omap36x_board_mux[] __initdata = {
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
	/* For MCBSP2 */
	/* PIN 'MCBSP2_DX/GPIO_119' is connected to NET 'IIS3_DATAO' */
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	/* PIN 'MCBSP2_DR/GPIO_118' is connected to NET 'IIS3_DATAI' */
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* PIN 'MCBSP2_CLKX/GPIO_117' is connected to NET 'IIS3_CLK_B2' */
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	/* PIN 'MCBSP2_FSX/GPIO_116' is connected to NET 'IIS3_FS' */
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),

	/* For MCBSP3 */
	OMAP3_MUX(MCBSP1_CLKX, OMAP_MUX_MODE2 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(MCBSP1_FSX, OMAP_MUX_MODE2 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(MCBSP3_DX, OMAP_MUX_MODE0 | OMAP_PIN_OFF_NONE),
#endif
	/* MMC2 SDIO pin muxes for eMMC */
	/* sdmmc2_clk */
	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_cmd */
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat0 */
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat1 */
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat2 */
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat3 */
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat4 */
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat5 */
	OMAP3_MUX(SDMMC2_DAT5, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat6 */
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc2_dat7 */
	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* PAL DEC  POWER DOWN */
	/* etk_d9 : gpio_23 */
	OMAP3_MUX(ETK_D9, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

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
	/* sdmmc3_clk */
	OMAP3_MUX(ETK_CLK, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc3_cmd */
	OMAP3_MUX(ETK_CTL, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc3_dat3 */
	OMAP3_MUX(ETK_D3, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc3_dat0 */
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc3_dat1 */
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
	/* sdmmc3_dat2 */
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE2 | OMAP_PIN_INPUT_PULLUP),
#endif

	/* For UART2 */
	/* uart2_tx */
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE1 | OMAP_PIN_OFF_NONE),
	/* uart2_rx */
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),

	/* AT24 EEPROM WP - GPIO 72 */
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* TLV320AIC3104 reset - GPIO 71 : OMAP3_BAIA_RES_O_1V8 */
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* OMAP3_BAIA_EN_AMPLI - GPIO 110 */
	OMAP3_MUX(CAM_D11, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* OMAP3_BAIA_ABIL_SOURCE_IP1V8 - GPIO 59 */
	OMAP3_MUX(GPMC_CLK, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* OMAP3_BAIA_SHTD_VIDEO_1V8 - GPIO 61 */
	OMAP3_MUX(GPMC_NBE1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* OMAP3_BAIA_VLED_EN_1V8 - GPIO 185 */
	OMAP3_MUX(I2C3_SDA, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* OMAP3_BAIA_TS_RESET - GPIO 126 */
	OMAP3_MUX(CAM_STROBE, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* I2S: CLK to TLV320AIC3104 */
	/* already set by x-loader */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE2 | OMAP_PIN_INPUT),
	OMAP3_MUX(GPMC_NCS5, OMAP_MUX_MODE2 | OMAP_PIN_INPUT),
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE2 | OMAP_PIN_INPUT),

	/* Audio path: GPIO_160, GPIO_109 */
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(CAM_D10, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* KSZ8851_MLL reset */
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

	/* Factory Default Reset: GPIO_15 (drv) GPIO_164 (read) */
	OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(ETK_D1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

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

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)

#define OMAP3_BAIA_GPIO_DEBOUNCE_TIMEOUT 10

static struct gpio_keys_button omap3_baia_gpio_keys[] = {
	{
		.desc			= "Factory Default Reset",
		.type			= EV_SW,
		.code			= SW_FRONT_PROXIMITY,
		.gpio			= OMAP3_BAIA_FACTORY_RESET_READ,
		.active_low		= 1,
		.debounce_interval	= OMAP3_BAIA_GPIO_DEBOUNCE_TIMEOUT,
	},
	{
		.desc			= "key 1",
		.type			= EV_KEY,
		.code			= KEY_1,
		.gpio			= OMAP3_BAIA_TPS_PULS1,
		.active_low		= 1,
		.debounce_interval	= OMAP3_BAIA_GPIO_DEBOUNCE_TIMEOUT,
	},
	{
		.desc			= "key 2",
		.type			= EV_KEY,
		.code			= KEY_2,
		.gpio			= OMAP3_BAIA_TPS_PULS2,
		.active_low		= 1,
		.debounce_interval	= OMAP3_BAIA_GPIO_DEBOUNCE_TIMEOUT,
	},
	{
		.desc			= "key 3",
		.type			= EV_KEY,
		.code			= KEY_3,
		.gpio			= OMAP3_BAIA_TPS_PULS3,
		.active_low		= 1,
		.debounce_interval	= OMAP3_BAIA_GPIO_DEBOUNCE_TIMEOUT,
	},
	{
		.desc			= "key 4",
		.type			= EV_KEY,
		.code			= KEY_4,
		.gpio			= OMAP3_BAIA_TPS_PULS4,
		.active_low		= 1,
		.debounce_interval	= OMAP3_BAIA_GPIO_DEBOUNCE_TIMEOUT,
	},
};

static struct gpio_keys_platform_data omap3_baia_gpio_keys_data = {
	.buttons	= omap3_baia_gpio_keys,
	.nbuttons	= ARRAY_SIZE(omap3_baia_gpio_keys),
};

static struct platform_device omap3_baia_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &omap3_baia_gpio_keys_data,
	},
};

static void __init omap3_baia_add_gpio_keys(void)
{
	int ret = gpio_request(OMAP3_BAIA_FACTORY_RESET_DRV,
			       "FACTORY reset enable");
	if (ret)
		printk(KERN_ERR "unable to request FACTORY reset enable!\n");
	gpio_direction_output(OMAP3_BAIA_FACTORY_RESET_DRV, 1);

	platform_device_register(&omap3_baia_gpio_keys_device);
}
#else
static void __init omap3_baia_add_gpio_keys(void)
{
}
#endif /* CONFIG_KEYBOARD_GPIO || CONFIG_KEYBOARD_GPIO_MODULE */

static void __init omap3_baia_init(void)
{
	int ret;

	if (cpu_is_omap3630()) {
		omap3_mux_init(omap36x_board_mux, OMAP_PACKAGE_CUS);
	} else {
		printk(KERN_ERR "WRONG CPU .. STOPPING\n");
		for (;;)
			schedule();
	}

	switch (baia_lcd) {
	case 0:
		printk(KERN_INFO "LCD CLAA/CPT/AMPIRE OVERRIDING\n");
		memcpy(&omap3_baia_lcd_device, &omap3_baia_lcd_device_claa,
		       sizeof(struct omap_dss_device));
		break;
	case 1:
	default:
		printk(KERN_INFO "LCD AUO OVERRIDING\n");
		memcpy(&omap3_baia_lcd_device, &omap3_baia_lcd_device_auo,
		       sizeof(struct omap_dss_device));
		break;
	}

	platform_add_devices(omap3_baia_devices,
			     ARRAY_SIZE(omap3_baia_devices));

	spi_register_board_info(omap3_baia_spi_board_info,
				ARRAY_SIZE(omap3_baia_spi_board_info));

	omap_serial_init();


	/*
	 * Enable USB
	 */

	/* setup EHCI phy reset config */
	omap_mux_init_gpio(OMAP3_BAIA_EHCIPHYRESET, OMAP_PIN_INPUT_PULLUP);
	usb_ehci_init(&ehci_pdata);

	usb_nop_xceiv_register(0);
	usb_musb_init(&musb_board_data);

	tsc2005_dev_init();

	ret = gpio_request(OMAP3_BAIA_EEPROM_WP, "AT24 WP enable");
	if (ret)
		printk(KERN_ERR "unable to request AT24 WP enable!\n");
	gpio_direction_output(OMAP3_BAIA_EEPROM_WP, 1);

	omap3_baia_i2c_init();
	omap3baia_init_ks8851_mll();
	omap3_baia_display_init();
	omap3_baia_add_gpio_keys();

	if (baia_gpio_debug) {
		/* Not nice for production release */
		gpio_export(OMAP3_BAIA_VLED_EN_1V8, 0);
		gpio_export(OMAP3_BAIA_LCD_PANEL_ENVIDEO_1V8, 0);
		gpio_export(OMAP3_BAIA_SHTD_VIDEO_1V8, 0);
		gpio_export(OMAP3_BAIA_EN_BL, 0);
		gpio_export(OMAP3_BAIA_NPDEC_PWRDN, 0);
		gpio_export(OMAP3_BAIA_TS_RESET, 0);
		gpio_export(OMAP3_BAIA_FACTORY_RESET_DRV, 0);
/*
		This commands BLOCK the kernel startup
		gpio_export(OMAP3_BAIA_TPS_PDEC_RES, 0);
		gpio_export(OMAP3_BAIA_TPS_PULS4, 0);
		gpio_export(OMAP3_BAIA_TPS_PULS3, 0);
		gpio_export(OMAP3_BAIA_TPS_PULS1, 0);
		gpio_export(OMAP3_BAIA_TPS_PULS2, 0);
*/
	}

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&omap3baia_wlan_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&omap3baia_wlan_regulator);
#endif

	schedule_delayed_work(&late_init_work, (msecs_to_jiffies(1*10)));
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
