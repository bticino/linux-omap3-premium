/*
 * LCD panel driver for CPT CLAA102NA0DCW
 *
 * Copyright (C) 2011 Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/display.h>

struct cmo_data {
	struct backlight_device *bl;
};

static struct omap_video_timings cmo_timings = {
	.x_res = 1024,
	.y_res = 600,

	.pixel_clock	= 45000,

	.hsw		= 2,
	.hfp		= 172,
	.hbp		= 2,

	.vsw		= 2,
	.vfp		= 21,
	.vbp		= 2,
};

static int cmo_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;

	if (!dssdev->set_backlight)
		return -EINVAL;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}

static int cmo_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}


static const struct backlight_ops cmo_bl_ops = {
	.get_brightness = cmo_bl_get_brightness,
	.update_status  = cmo_bl_update_status,
};

static int cmo_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct cmo_data *sd;
	int r;

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = cmo_timings;

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);

	printk("%s-%d\n", __func__, __LINE__);
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;

	bl = backlight_device_register("claa102", &dssdev->dev, dssdev,
			&cmo_bl_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		kfree(sd);
		return r;
	}
	sd->bl = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = cmo_bl_update_status(bl);
	if (r < 0)
		dev_err(&dssdev->dev, "failed to set lcd brightness\n");

	return 0;
}

static void cmo_panel_remove(struct omap_dss_device *dssdev)
{
	struct cmo_data *sd = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bl = sd->bl;

	printk("%s-%d\n", __func__, __LINE__);
	bl->props.power = FB_BLANK_POWERDOWN;
	cmo_bl_update_status(bl);
	backlight_device_unregister(bl);

	kfree(sd);
}

static int cmo_power_on(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void cmo_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int cmo_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	r = cmo_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static void cmo_panel_disable(struct omap_dss_device *dssdev)
{
	cmo_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int cmo_panel_suspend(struct omap_dss_device *dssdev)
{
	cmo_panel_disable(dssdev);
	return 0;
}

static int cmo_panel_resume(struct omap_dss_device *dssdev)
{
	int r;
	r = cmo_power_on(dssdev);
	return cmo_panel_enable(dssdev);
}

static struct omap_dss_driver cmo_driver = {
	.probe		= cmo_panel_probe,
	.remove		= cmo_panel_remove,

	.enable		= cmo_panel_enable,
	.disable	= cmo_panel_disable,
	.suspend	= cmo_panel_suspend,
	.resume		= cmo_panel_resume,

	.driver         = {
		.name   = "claa102na0dcw",
		.owner  = THIS_MODULE,
	},
};

static int __init cmo_panel_drv_init(void)
{
	return omap_dss_register_driver(&cmo_driver);
}

static void __exit cmo_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&cmo_driver);
}

module_init(cmo_panel_drv_init);
module_exit(cmo_panel_drv_exit);
MODULE_DESCRIPTION("CHUNGHWA PICTURE TUBES claa102na0dcw LCD panel");
MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_LICENSE("GPL");
