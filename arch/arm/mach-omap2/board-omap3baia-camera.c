/*
 * OMAP3BAIA: Driver for TVP5150 input
 *
 * Copyright (C) 2011 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>

#include <media/tvp5150.h>
#include <linux/i2c/tda9885.h>

#include <../drivers/media/video/isp/isp.h>

#include "mux.h"
#include "devices.h"
#include "board-omap3baia.h"

/* mux id to enable/disable signal routing to different peripherals */
enum omap3baia_cam_mux {
	MUX_EN_TVP5150 = 0,
	MUX_INVALID,
};

/**
 * @brief omap3baia_set_mux - Sets mux to enable/disable signal routing to
 *                             different peripherals present on new EVM board
 *
 * @param mux_id - enum, mux id to enable/disable
 * @param value - enum, ENABLE_MUX for enabling and DISABLE_MUX for disabling
 *
 */
static void omap3baia_set_mux(enum omap3baia_cam_mux mux_id)
{
	printk("%s-%d TODO set mux\n", __func__, __LINE__);
}

static struct tda9885_platform_data tda9885_defaults = {
	.switching_mode = 0xf2,
	.adjust_mode = 0xd0,
	.data_mode = 0x0b,
//	.power = ABIL_DEM_VIDEO,
};

/* TVP5150: Video Decoder */

/*
TODO : not present nothing in
include/media/tvp5150.h

static int omap3baia_tvp515x_s_power(struct v4l2_subdev *subdev, u32 on)
{
//	omap3baia_set_mux(MUX_EN_TVP5146);
	printk("%s-%d TODO POWERON\n", __func__, __LINE__);

	return 0;
}

static struct tvp515x_platform_data omap3baia_tvp515x_platform_data = {
	.s_power		= omap3baia_tvp515x_s_power,
};
*/

#define TVP515X_I2C_BUS_NUM		2

static struct i2c_board_info omap3baia_camera_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tvp5150", 0x5d),
//		.platform_data	= &omap3baia_tvp515x_platform_data,
	},
	{
		I2C_BOARD_INFO("tda9885", 0x43),
//		.platform_data	= &omap3baia_tvp515x_platform_data,
	},
};

static struct isp_subdev_i2c_board_info omap3baia_tvp515x_subdevs[] = {
	{
		.board_info	= &omap3baia_camera_i2c_devices[0],
		.i2c_adapter_id	= TVP515X_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};

static struct isp_v4l2_subdevs_group omap3baia_camera_subdevs[] = {
	{
		.subdevs	= omap3baia_tvp515x_subdevs,
		.interface	= ISP_INTERFACE_PARALLEL,
		.bus		= {
			.parallel	= {
				.width			= 8,
				.data_lane_shift	= 1,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 1,
				.fldmode		= 1,
				.bridge			= 0,
				.is_bt656		= 1,
			},
		},
	},
	{ NULL, 0 },
};

static struct isp_platform_data omap3baia_isp_platform_data = {
	.subdevs = omap3baia_camera_subdevs,
};

static int __init omap3baia_cam_init(void)
{
	int ret = 0;

	printk("%s-%d TODO  11 res poweron\n", __func__, __LINE__);

	omap3_init_camera(&omap3baia_isp_platform_data);

	printk(KERN_INFO "omap3baia camera init done successfully...\n");
	return 0;

}

static void __exit omap3baia_cam_exit(void)
{
	printk("%s-%d TODO res poweroff\n", __func__, __LINE__);
}

module_init(omap3baia_cam_init);
module_exit(omap3baia_cam_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("omap3baia Camera Module");
MODULE_LICENSE("GPL");
