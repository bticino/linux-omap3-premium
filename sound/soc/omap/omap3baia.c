/*
 * omap3baia.c  --  SoC audio for Bticino Baia
 *
 * Copyright (C) 2012 Bticino
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
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <plat/mcbsp.h>

#include <asm/mach-types.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/tlv320aic3x.h"
#include "../codecs/zl38005.h"
#include "../codecs/tpa2016d2.h"

#define CODEC_SYS_FREQ 13000000

static int omap3baia_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	snd_pcm_hw_constraint_minmax(runtime,
				     SNDRV_PCM_HW_PARAM_CHANNELS, 2, 2);
	return 0;
}

static int omap3baia_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int err;

	/* Set codec DAI configuration */
	err = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	/* Set cpu DAI configuration */
	err = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (err < 0)
		return err;

	/* Set the codec system clock for DAC and ADC */
	return snd_soc_dai_set_sysclk(codec_dai, 0, CODEC_SYS_FREQ,
				      SND_SOC_CLOCK_IN);
}

static struct snd_soc_ops omap3baia_ops = {
	.startup = omap3baia_startup,
	.hw_params = omap3baia_hw_params,
};

static const struct snd_soc_dapm_widget aic31_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("DMic", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"DMic Rate 64", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "DMic"},
};

static int omap3baia_aic31_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int err;

	/* Set up NC codec pins */
	snd_soc_dapm_nc_pin(codec, "MIC3L");
	snd_soc_dapm_nc_pin(codec, "MIC3R");
//	snd_soc_dapm_nc_pin(codec, "LINE1R");

	snd_soc_dapm_new_controls(codec, aic31_dapm_widgets,
				  ARRAY_SIZE(aic31_dapm_widgets));

	tpa2016d2_add_controls(codec);

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3baia_dai[] = {
	{
		.name = "TLV320AIC31",
		.stream_name = "AIC31",
		.cpu_dai_name = "omap-mcbsp-dai.3",
		.codec_dai_name = "tlv320aic3x-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "tlv320aic3x-codec.2-0018",
		.init = omap3baia_aic31_init,
		.ops = &omap3baia_ops,
	},
};

/* Audio card */
static struct snd_soc_card omap3baia_sound_card = {
	.name = "omap3_baia_aic31",
	.dai_link = omap3baia_dai,
	.num_links = ARRAY_SIZE(omap3baia_dai),
};

static struct platform_device *omap3baia_snd_device;
static struct clk *sys_out2_ck, *sys_out2, *sys_ck;

static int __init omap3baia_soc_init(void)
{
	int err;

	if (!machine_is_omap3_baia())
		return -ENODEV;

	omap3baia_snd_device = platform_device_alloc("soc-audio", 1);
	if (!omap3baia_snd_device) {
		err = -ENOMEM;
		goto err1;
	}

	platform_set_drvdata(omap3baia_snd_device, &omap3baia_sound_card);

	err = platform_device_add(omap3baia_snd_device);
	if (err)
		goto err2;

        /* Enable clock for aic34 */
        sys_out2_ck = clk_get(NULL, "clkout2_src_ck");
        if (IS_ERR(sys_out2_ck)) {
                printk(KERN_ERR "unable to request clkout2_src_ck clock!\n");
		goto err2;
	}

        sys_out2 = clk_get(&omap3baia_snd_device->dev, "sys_clkout2");
        if (IS_ERR(sys_out2)) {
                printk(KERN_ERR "unable to request sys_clkout2 clock!\n");
		goto err3;
	}

        sys_ck = clk_get(&omap3baia_snd_device->dev, "sys_ck");
        if (IS_ERR(sys_ck)) {
                printk(KERN_ERR "unable to request sys_ck clock!\n");
		goto err4;
	}

	clk_set_parent(sys_out2_ck, sys_ck);
	clk_enable(sys_out2);
	err = clk_set_rate(sys_out2, CODEC_SYS_FREQ);
	if (err) {
		printk(KERN_ERR "Could not find matching rate for clock\n");
		goto err5;
	}

	return 0;
err5:
	clk_put(sys_ck);
err4:
	clk_put(sys_out2);
err3:
	clk_put(sys_out2_ck);
err2:
	platform_device_put(omap3baia_snd_device);
err1:
	return err;
}

static void __exit omap3baia_soc_exit(void)
{
	platform_device_unregister(omap3baia_snd_device);
	clk_disable(sys_out2);
	clk_put(sys_out2);
}

module_init(omap3baia_soc_init);
module_exit(omap3baia_soc_exit);

MODULE_AUTHOR("Raffaele Recalcati");
MODULE_DESCRIPTION("ALSA SoC Bticino");
MODULE_LICENSE("GPL");
