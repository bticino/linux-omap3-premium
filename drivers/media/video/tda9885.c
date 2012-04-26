/*
 * Driver for the TDA9885 chip
 *
 * Copyright (c) 2010 Rodolfo Giometti <giometti@linux.it>
 * Copyright (c) 2011 Bticino S.p.A. <raffaele.recalcati@bticino.it>
 * The former driver of Rodolfo Giometti has been converted
 * to a subdev one.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <linux/i2c/tda9885.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <linux/gpio.h>

#define DRIVER_VERSION	"2.0.0"

#define AFCWIN (1<<7)

static int debug; /* insmod parameter */
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_LICENSE("GPL");

struct tda9885 {
	struct v4l2_subdev sd;
	const struct tda9885_platform_data *pdata;
	int status;
};

static inline struct tda9885 *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tda9885, sd);
}

static int tda9885_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	v4l2_dbg(1, debug, sd, "%s : ctrl->id = %d\n", __func__, ctrl->id);
	return 0;
}

static int tda9885_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tda9885 *t = to_state(sd);
	int ret;
	u8 status;
	u8 buf[] = {
		0, t->pdata->switching_mode, t->pdata->adjust_mode, t->pdata->data_mode,
	};

	v4l2_dbg(1, debug, sd, "Switching ON the demodulator\n");

	/*
	 * This chip is very simple, just write first the base address
	 * and then all registers settings.
	 */
	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	ret = (ret == ARRAY_SIZE(buf)) ? 0 : ret;
	v4l2_dbg(1, debug, sd, "Reading status byte\n");
	ret = i2c_master_recv(client, &status, 1);
	if (unlikely(ret != 1))
		dev_err(&client->dev, "wanted %d bytes, got %d\n",
			1, ret);
	v4l2_dbg(1, debug, sd, "Status byte 0x%02X\n", status);
	ret = 0;

	switch (status & AFCWIN) {
	case 1:
		*std = V4L2_STD_PAL;
	break;
	case 0:
	default:
	break;
	}

	return 0;
}

static int tda9885_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tda9885 *t = to_state(sd);
	int ret;
	u8 buf[] = {
		0, t->pdata->switching_mode, t->pdata->adjust_mode, t->pdata->data_mode,
	};

	switch (enable) {
	case 0:
	{
		v4l2_dbg(1, debug, sd, "Switching OFF the demodulator\n");
		/* Power Down */
		gpio_set_value(t->pdata->power, 0);
		ret = 0;
		break;
	}
	case 1:
	{
		v4l2_dbg(1, debug, sd, "Switching ON the demodulator\n");
		/* Power up */
		//gpio_set_value(t->pdata->power, 1);
		gpio_set_value(t->pdata->power, 0); /* Alwasy ON */

		/*
		 * Little delay for power up
		 * datasheet: time constant (R × C) for network
		 * without i2c bus is 1.2 usec
		*/
		mdelay(1);

		/*
		 * This chip is very simple, just write first the base address
		 * and then all registers settings.
		 */
		ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
		ret = (ret == ARRAY_SIZE(buf)) ? 0 : ret;

		v4l2_dbg(1, debug, sd, "Reading status byte\n");
		ret = i2c_master_recv(client, buf, 1);
		if (unlikely(ret != 1))
			dev_err(&client->dev, "wanted %d bytes, got %d\n",
				1, ret);
		v4l2_dbg(1, debug, sd, "Status byte 0x%02X\n", buf[0]);
		ret = 0;
		break;
	}
	default:
		return -ENODEV;
		break;
	}

	return ret;
}

static int tda9885_s_power(struct v4l2_subdev *sd, int power)
{
	printk("%s-%d\n", __func__, __LINE__);
	return tda9885_s_stream(sd, power);
}

static int tda9885_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	int rev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_TDA9885,
					  0);
}

static const struct v4l2_subdev_video_ops tda9885_video_ops = {
	.s_stream = tda9885_s_stream,
	.querystd = tda9885_querystd,
};

static const struct v4l2_subdev_core_ops tda9885_core_ops = {
	.g_ctrl = tda9885_g_ctrl,
	.s_power = tda9885_s_power,
	.g_chip_ident = tda9885_g_chip_ident,
};

static const struct v4l2_subdev_ops tda9885_ops = {
	.core = &tda9885_core_ops,
	.video = &tda9885_video_ops
};

/*
 * I2C init/probing/exit functions
 */
static int tda9885_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tda9885 *data;
	struct v4l2_subdev *sd;
	int err = 0;
	struct tda9885 *t;
	int ret;
	u8 buf[4];

	v4l_info(client, "chip found @ 1x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	if (!client->dev.platform_data) {
		v4l2_err(client, "No platform data!!\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct tda9885), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	} else {
		/* Copy board specific information here */
		data->pdata = client->dev.platform_data;
		i2c_set_clientdata(client, data);
	}

	printk("%s-%d QUICK fix waiting for Media Controller\n", __func__, __LINE__);
	/* Register with V4L2 layer as slave device */
	sd = &data->sd;
	v4l2_i2c_subdev_init(sd, client, &tda9885_ops);
	v4l2_dbg(1, debug, sd, "default switching mode is 0x%02x\n",
		data->pdata->switching_mode);
	printk("switching_mode=%d\n", data->pdata->switching_mode);
	v4l2_dbg(1, debug, sd, "default adjust mode is 0x%02x\n",
		data->pdata->adjust_mode);
	printk("switching_mode=%d\n", data->pdata->adjust_mode);
	v4l2_dbg(1, debug, sd, "default data mode is 0x%02x\n",
		data->pdata->data_mode);
	printk("switching_mode=%d\n", data->pdata->data_mode);
	v4l2_dbg(1, debug, sd, "power gpio is %d\n",
		data->pdata->power);
	v4l2_info(sd, "%s decoder driver registered (ver. %s)\n", sd->name, DRIVER_VERSION);

	buf[0] = 0;
	buf[1] = data->pdata->switching_mode;
	buf[2] = data->pdata->adjust_mode;
	buf[3] = data->pdata->data_mode;

	/*
	 * This chip is very simple, just write first the base address
	 * and then all registers settings.
	 */
	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	ret = (ret == ARRAY_SIZE(buf)) ? 0 : ret;

	v4l2_dbg(1, debug, sd, "Reading status byte\n");
	ret = i2c_master_recv(client, buf, 1);
	if (unlikely(ret != 1))
		dev_err(&client->dev, "wanted %d bytes, got %d\n",
			1, ret);
	v4l2_dbg(1, debug, sd, "Status byte 0x%02X\n", buf[0]);
	printk("TDAAAAA Status byte 0x%02X\n", buf[0]);
	ret = 0;

	printk("TDA9885 ALWAYS ON\n");
	/* Power Down */
	gpio_set_value(data->pdata->power, 0); /* Alway ON */

	return 0;

exit:
	return err;
}

static int tda9885_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tda9885 *t = to_state(sd);

	v4l2_device_unregister_subdev(sd);

	gpio_set_value(t->pdata->power, 0);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id tda9885_id[] = {
	{ "tda9885", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tda9885_id);

static struct i2c_driver tda9885_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tda9885",
	},
	.probe		= tda9885_probe,
	.remove		= tda9885_remove,
	.id_table	= tda9885_id,
};

static __init int init_tda9885(void)
{
	return i2c_add_driver(&tda9885_driver);
}

static __exit void exit_tda9885(void)
{
	i2c_del_driver(&tda9885_driver);
}

module_init(init_tda9885);
module_exit(exit_tda9885);

/* Module information */
MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("TDA9885 IF-PPL demodulator driver");
MODULE_LICENSE("GPLv2");
MODULE_VERSION(DRIVER_VERSION);

