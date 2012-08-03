/*
 * omap3baia.c  --  SoC audio for OMAP3 Baia board
 *
 * Author: Raffaele Recalcati <raffaele.recalcati@bticino.it>
 *
 * Derived from omap3beagle
 * Author: Steve Sakoman <steve@sakoman.com>
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
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../../../arch/arm/mach-omap2/board-omap3baia.h"

static int omap3baia_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt;
	int ret;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 4: /* Four channel TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_B |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	default:
		return -EINVAL;
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT,
				     256 * params_rate(params),
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 8);
	if (ret < 0) {
		pr_err(KERN_ERR "can't set SRG clock divider\n");
		return ret;
	}

	return 0;
}

static int tps_to_tsh_spk_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value(OMAP3_BAIA_ABIL_SOURCE_IP1V8, 1);
	else
		gpio_set_value(OMAP3_BAIA_ABIL_SOURCE_IP1V8, 0);

	return 0;
}

static const struct snd_soc_dapm_widget twl_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("TSH512 in", tps_to_tsh_spk_event),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/*
	 * From tps65951 output (ihf left and right)
	 * to TSH512 inputs
	 */
	{"TSH512 in", NULL, "HandsfreeL PGA"},
	{"TSH512 in", NULL, "HandsfreeR PGA"},
};

static int omap3baia_twl_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	/* TPS65951 not connected pins */
	snd_soc_dapm_nc_pin(codec, "HSMIC");
	snd_soc_dapm_nc_pin(codec, "HSOL");
	snd_soc_dapm_nc_pin(codec, "HSOR");

	snd_soc_dapm_new_controls(codec, twl_dapm_widgets,
				  ARRAY_SIZE(twl_dapm_widgets));

	ret = gpio_request(OMAP3_BAIA_ABIL_SOURCE_IP1V8,
			   "ABIL_SOURCE_IP enable");
	if (ret < 0)
		printk(KERN_ERR "can't get ABIL_SOURCE_IP enable\n");
	gpio_direction_output(OMAP3_BAIA_ABIL_SOURCE_IP1V8, 0);
	gpio_export(OMAP3_BAIA_ABIL_SOURCE_IP1V8, 0);

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}
static struct snd_soc_ops omap3baia_ops = {
	.hw_params = omap3baia_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap3baia_dai = {
	.name = "TWL4030",
	.stream_name = "TWL4030",
	.cpu_dai_name = "omap-mcbsp-dai.1",
	.platform_name = "omap-pcm-audio",
	.codec_dai_name = "twl4030-hifi",
	.codec_name = "twl4030-codec",
	.init = omap3baia_twl_init,
	.ops = &omap3baia_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_omap3baia = {
	.name = "omap3baia",
	.owner = THIS_MODULE,
	.dai_link = &omap3baia_dai,
	.num_links = 1,
};

static struct platform_device *omap3baia_snd_device;

static int __init omap3baia_soc_init(void)
{
	int ret;

	if (!machine_is_omap3_baia())
		return -ENODEV;
	pr_info("OMAP3 Baia SoC init\n");

	omap3baia_snd_device = platform_device_alloc("soc-audio", -1);
	if (!omap3baia_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(omap3baia_snd_device, &snd_soc_omap3baia);

	ret = platform_device_add(omap3baia_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(omap3baia_snd_device);

	return ret;
}

static void __exit omap3baia_soc_exit(void)
{
	platform_device_unregister(omap3baia_snd_device);
}

module_init(omap3baia_soc_init);
module_exit(omap3baia_soc_exit);

MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_DESCRIPTION("ALSA SoC OMAP3 Baia");
MODULE_LICENSE("GPL");
