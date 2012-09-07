/*
 * ALSA SoC Texas Instruments TPA2016D2 Class-D stereo amplifier driver
 *
 * Copyright (C) Bticino S.p.A.
 *
 * Author: Raffaele Recalcati <raffaele.recalcati@bticino.it>
 *
 * Derived from TPA6130a2 driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <sound/tpa2016d2-plat.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <linux/delay.h>

#include "tpa2016d2.h"

static struct i2c_client *tpa2016d2_client;

/* This struct is used to save the context */
struct tpa2016d2_data {
	struct mutex mutex;
	unsigned char regs[TPA2016D2_CACHEREGNUM];
	int shutdown_gpio;
	unsigned char shutdown_state;
};

static int tpa2016d2_i2c_read(int reg)
{
	struct tpa2016d2_data *data;
	int val;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);

	/* If shutdown, return the cached value */
	if (!data->shutdown_state) {
		val = i2c_smbus_read_byte_data(tpa2016d2_client, reg);
		if (val < 0)
			dev_err(&tpa2016d2_client->dev, "Read failed\n");
		else
			data->regs[reg] = val;
	} else {
		val = data->regs[reg];
	}

	return val;
}

static int tpa2016d2_i2c_write(int reg, u8 value)
{
	struct tpa2016d2_data *data;
	int val = 0;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);
	if (!data->shutdown_state) {
		val = i2c_smbus_write_byte_data(tpa2016d2_client, reg, value);
		if (val < 0) {
			dev_err(&tpa2016d2_client->dev, "Write failed\n");
			return val;
		}
	}

	/* Either in shutdown or not, we save the context */
	data->regs[reg] = value;

	return val;
}

static u8 tpa2016d2_read(int reg)
{
	struct tpa2016d2_data *data;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);

	return data->regs[reg];
}

static int tpa2016d2_initialize(void)
{
	struct tpa2016d2_data *data;
	int i, ret = 0;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);

	for (i = 1; i < TPA2016D2_CACHEREGNUM; i++) {
		ret = tpa2016d2_i2c_write(i, data->regs[i]);
		if (ret < 0)
			break;
	}
	for (i = 1; i < TPA2016D2_CACHEREGNUM; i++) {
		if (tpa2016d2_i2c_read(i) != data->regs[i])
			printk("%s-%d ERROR RE_READING\n", __func__, __LINE__);
	}

	return ret;
}

int tpa2016d2_shutdown(int shutdown)
{
	struct	tpa2016d2_data *data;
	u8	val;
	int	ret = 0;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);

	mutex_lock(&data->mutex);
	if (!shutdown && data->shutdown_state) {
		/* Oerative State */
		if (data->shutdown_gpio >= 0) {
			gpio_set_value(data->shutdown_gpio, 1);
			mdelay(1);
		}

		data->shutdown_state = 0;
		ret = tpa2016d2_initialize();
		if (ret < 0) {
			dev_err(&tpa2016d2_client->dev,
				"Failed to initialize chip\n");
			if (data->shutdown_gpio >= 0)
				gpio_set_value(data->shutdown_gpio, 0);
			data->shutdown_state = 0;
			goto exit;
		}

		/* Clear SWS */
		val = tpa2016d2_read(TPA2016D2_REG_CONTROL);
		val &= ~TPA2016D2_SWS;
		tpa2016d2_i2c_write(TPA2016D2_REG_CONTROL, val);
	} else if (shutdown && !data->shutdown_state) {
		/* set SWS */
		val = tpa2016d2_read(TPA2016D2_REG_CONTROL);
		val |= TPA2016D2_SWS;
		tpa2016d2_i2c_write(TPA2016D2_REG_CONTROL, val);

		/* Power off */
		if (data->shutdown_gpio >= 0)
			gpio_set_value(data->shutdown_gpio, 0);

		data->shutdown_state = 1;
	}

exit:
	mutex_unlock(&data->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(tpa2016d2_shutdown);

static int tpa2016d2_get_reg(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tpa2016d2_data *data;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);

	mutex_lock(&data->mutex);

	ucontrol->value.integer.value[0] =
		(tpa2016d2_read(reg) >> shift) & mask;

	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
	}

	mutex_unlock(&data->mutex);
	return 0;
}

static int tpa2016d2_put_reg(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tpa2016d2_data *data;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	unsigned int val_reg;

	BUG_ON(tpa2016d2_client == NULL);
	data = i2c_get_clientdata(tpa2016d2_client);

	if (invert)
		val = max - val;

	mutex_lock(&data->mutex);

	val_reg = tpa2016d2_read(reg);
	if (((val_reg >> shift) & mask) == val) {
		mutex_unlock(&data->mutex);
		return 0;
	}

	val_reg &= ~(mask << shift);
	val_reg |= val << shift;
	tpa2016d2_i2c_write(reg, val_reg);

	mutex_unlock(&data->mutex);

	return 1;
}

/*
 * Enable or disable channel (left or right)
 * The bit number for mute and amplifier are the same per channel:
 * bit 6: Right channel
 * bit 7: Left channel
 * in both registers.
 */
static void tpa2016d2_channel_enable(u8 channel, int enable)
{
	u8	val;

	if (enable) {
		/* Enable channel */
		/* Enable amplifier */
		val = tpa2016d2_read(TPA2016D2_REG_CONTROL);
		val |= channel;
		tpa2016d2_i2c_write(TPA2016D2_REG_CONTROL, val);

	} else {
		/* Disable channel */
		/* Mute channel */
		val = tpa2016d2_read(TPA2016D2_REG_CONTROL);
		val |= channel;
		tpa2016d2_i2c_write(TPA2016D2_REG_CONTROL, val);
	}
}

static int tpa2016d2_left_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		tpa2016d2_channel_enable(TPA2016D2_SPK_EN_L, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tpa2016d2_channel_enable(TPA2016D2_SPK_EN_L, 0);
		break;
	}
	return 0;
}

static int tpa2016d2_right_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		tpa2016d2_channel_enable(TPA2016D2_SPK_EN_R, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tpa2016d2_channel_enable(TPA2016D2_SPK_EN_L, 0);
		break;
	}
	return 0;
}

static int tpa2016d2_startup_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = tpa2016d2_shutdown(0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = tpa2016d2_shutdown(1);
		break;
	}
	return ret;
}

/* 0dB to 30db in 3dB steps (compression not enabled) */
static const DECLARE_TLV_DB_SCALE(fixed_ampli_tlv, 0, 100, 3000);

/* Linear scale between 0.1067ms to 6.722ms: 0.1067ms every step */
static const unsigned int atk_time_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 63, TLV_DB_LINEAR_ITEM(1067, 6722),
};

/* Linear scale between 0.0137s to 0.8631ms: 0.0137s every step */
static const unsigned int rel_hold_time_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 63, TLV_DB_LINEAR_ITEM(137,8631),
};

static const char *agc_enable[] =
    { "agc disabled", "agc 2:1", "agc 4:1", "agc 8:1"};

static const struct soc_enum agc_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(agc_enable), agc_enable);

/* 18dB to 30db in 1dB steps */
static const DECLARE_TLV_DB_SCALE(agc_ampli, 1800, 100, 3000);

static const struct snd_kcontrol_new tpa2016d2_controls[] = {
	SOC_SINGLE_EXT("TPA2016D2 Noise Gate",
		       TPA2016D2_REG_CONTROL, TPA2016D2_NG_EN, 1, 0,
		       tpa2016d2_get_reg, tpa2016d2_put_reg),
	SOC_SINGLE_EXT("TPA2016D2 Left Mute",
		       TPA2016D2_REG_CONTROL, TPA2016D2_SPK_EN_L, 1, 0,
		       tpa2016d2_get_reg, tpa2016d2_put_reg),

	SOC_SINGLE_EXT("TPA2016D2 Right Mute",
		   TPA2016D2_REG_CONTROL, TPA2016D2_SPK_EN_R, 1, 0,
		       tpa2016d2_get_reg, tpa2016d2_put_reg),
	SOC_SINGLE_EXT("TPA2016D2 Compression",
		   TPA2016D2_REG_AGC_CONTROL, TPA2016D2_OUT_LIM_EN, 1, 1,
		   tpa2016d2_get_reg, tpa2016d2_put_reg),
	SOC_SINGLE_EXT_TLV("TPA2016D2 Gain",
			   TPA2016D2_REG_AGC_FIXED_GAIN, 0, 0x1e, 0,
			   tpa2016d2_get_reg, tpa2016d2_put_reg,
			   fixed_ampli_tlv),
	SOC_SINGLE_EXT_TLV("TPA2016D2 ATK Time",
		       TPA2016D2_REG_AGC_ATTACH, 0, 0x3f, 0,
		       tpa2016d2_get_reg, tpa2016d2_put_reg,
		       atk_time_tlv),
	SOC_SINGLE_EXT_TLV("TPA2016D2 REL Time",
		       TPA2016D2_REG_AGC_RELEASE, 0, 0x3f, 0,
		       tpa2016d2_get_reg, tpa2016d2_put_reg,
		       rel_hold_time_tlv),
	SOC_SINGLE_EXT_TLV("TPA2016D2 HOLD Time",
			   TPA2016D2_REG_AGC_HOLD, 0, 0x3f, 0,
			   tpa2016d2_get_reg, tpa2016d2_put_reg,
			   rel_hold_time_tlv),
//	SOC_ENUM_EXT("AGC enable", agc_enum,
//			   tpa2016d2_get_agc, tpa2016d2_put_agc),
	SOC_SINGLE_EXT_TLV("AGC Gain", TPA2016D2_REG_AGC_CONTROL_1, 4, 12, 0,
			   tpa2016d2_get_reg, tpa2016d2_put_reg,
			   agc_ampli),
};

static const struct snd_soc_dapm_widget tpa2016d2_dapm_widgets[] = {
	SND_SOC_DAPM_PGA_E("TPA2016D2 Left", SND_SOC_NOPM,
			0, 0, NULL, 0, tpa2016d2_left_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("TPA2016D2 Right", SND_SOC_NOPM,
			0, 0, NULL, 0, tpa2016d2_right_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("TPA2016D2 Enable", SND_SOC_NOPM,
			0, 0, tpa2016d2_startup_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("TPA2016D2 Speaker Left"),
	SND_SOC_DAPM_OUTPUT("TPA2016D2 Speaker Right"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"TPA2016D2 Speaker Left", NULL, "TPA2016D2 Left"},
	{"TPA2016D2 Speaker Right", NULL, "TPA2016D2 Right"},
	{"TPA2016D2 Speaker Left", NULL, "TPA2016D2 Enable"},
	{"TPA2016D2 Speaker Right", NULL, "TPA2016D2 Enable"},
};

int tpa2016d2_add_controls(struct snd_soc_codec *codec)
{
	struct	tpa2016d2_data *data;

	if (tpa2016d2_client == NULL)
		return -ENODEV;
	data = i2c_get_clientdata(tpa2016d2_client);

	snd_soc_dapm_new_controls(codec, tpa2016d2_dapm_widgets,
				ARRAY_SIZE(tpa2016d2_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	return snd_soc_add_controls(codec, tpa2016d2_controls,
						ARRAY_SIZE(tpa2016d2_controls));

}
EXPORT_SYMBOL_GPL(tpa2016d2_add_controls);

static int __devinit tpa2016d2_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct device *dev;
	struct tpa2016d2_data *data;
	struct tpa2016d2_platform_data *pdata;
	int ret;

	dev = &client->dev;

	if (client->dev.platform_data == NULL) {
		dev_err(dev, "Platform data not set\n");
		dump_stack();
		return -ENODEV;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Can not allocate memory\n");
		return -ENOMEM;
	}

	tpa2016d2_client = client;

	i2c_set_clientdata(tpa2016d2_client, data);

	pdata = client->dev.platform_data;
	data->shutdown_gpio = pdata->shutdown_gpio;

	mutex_init(&data->mutex);

	/* Set default register values */
	data->regs[TPA2016D2_REG_CONTROL] = TPA2016D2_NG_EN;
	data->regs[TPA2016D2_REG_AGC_ATTACH] = 0;
	data->regs[TPA2016D2_REG_AGC_RELEASE] = 0;
	data->regs[TPA2016D2_REG_AGC_HOLD] = 0;
	data->regs[TPA2016D2_REG_AGC_FIXED_GAIN] = 0;
	data->regs[TPA2016D2_REG_AGC_CONTROL] = 0x7f;
	data->regs[TPA2016D2_REG_AGC_CONTROL_1] = 0xc0;

	if (data->shutdown_gpio >= 0) {
		ret = gpio_request(data->shutdown_gpio, "tpa2016d2 enable");
		if (ret < 0) {
			dev_err(dev, "Failed to request shutdown GPIO (%d)\n",
				data->shutdown_gpio);
			goto err_gpio;
		}
		gpio_direction_output(data->shutdown_gpio, 0);
		gpio_export(data->shutdown_gpio, 0);
		data->shutdown_state = 1;
	}

	return 0;
err_gpio:
	kfree(data);
	i2c_set_clientdata(tpa2016d2_client, NULL);
	tpa2016d2_client = NULL;

	return ret;
}

static int __devexit tpa2016d2_remove(struct i2c_client *client)
{
	struct tpa2016d2_data *data = i2c_get_clientdata(client);

	tpa2016d2_shutdown(1);

	if (data->shutdown_gpio >= 0)
		gpio_free(data->shutdown_gpio);

	kfree(data);
	tpa2016d2_client = NULL;

	return 0;
}

static const struct i2c_device_id tpa2016d2_id[] = {
	{ "tpa2016d2", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tpa2016d2_id);

static struct i2c_driver tpa2016d2_i2c_driver = {
	.driver = {
		.name = "tpa2016d2",
		.owner = THIS_MODULE,
	},
	.probe = tpa2016d2_probe,
	.remove = __devexit_p(tpa2016d2_remove),
	.id_table = tpa2016d2_id,
};

static int __init tpa2016d2_init(void)
{
	return i2c_add_driver(&tpa2016d2_i2c_driver);
}

static void __exit tpa2016d2_exit(void)
{
	i2c_del_driver(&tpa2016d2_i2c_driver);
}

MODULE_AUTHOR("Raffaele Recalcati");
MODULE_DESCRIPTION("TPA2016D2 Stereo Class-D AGC amplifier driver");
MODULE_LICENSE("GPL");

module_init(tpa2016d2_init);
module_exit(tpa2016d2_exit);
