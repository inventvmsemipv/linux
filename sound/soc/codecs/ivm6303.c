/*
 * ivm6303.c -- ivm6303 ALSA SoC Audio Driver
 *
 * Copyright 2022 Inventvm
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/asound.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>
#include <linux/pm_wakeup.h>
#include <linux/kernel.h>

/* Base Region */
#define IVM6303_SYSTEM_CTRL		0x00

/* Base Region */
#define IVM6303_SYSTEM_CTRL		0x00
#define IVM6303_SOFTWARE_RESET		0x01
#define IVM6303_ENABLES_SETTINGS(n)	(0x14 + (n))
/* ENABLES_SETTINGS_1 */
# define PLL_EN				BIT(3)
/* ENABLES_SETTINGS_5 */
# define SPK_EN				BIT(0)
# define SPK_MUTE			BIT(1)

#define IVM6303_TDM_APPLY_CONF		0x30
# define DO_APPLY_CONF			BIT(0)
# define TDM_RESYNC			BIT(7)
#define IVM6303_TDM_SETTINGS(n)		(0x30 + (n))
/* TDM_SETTINGS_1 */
# define TDM_BCLK_POLARITY		BIT(4)
# define TDM_FSYN_POLARITY		BIT(5)
# define TDM_DELAY_MODE			BIT(6)
# define TDM_SETTINGS_MASK (TDM_BCLK_POLARITY|TDM_FSYN_POLARITY|TDM_DELAY_MODE)
/* IVM6303_TDM_SETTINGS(3) */
# define I_SLOT_SIZE_SHIFT		6
# define I_SLOT_SIZE_MASK		(0x3 << I_SLOT_SIZE_SHIFT)
/* IVM6303_TDM_SETTINGS(4) */
# define O_SLOT_SIZE_SHIFT		6
# define O_SLOT_SIZE_MASK		(0x3 << O_SLOT_SIZE_SHIFT)
# define O_SLOT_CHAN_SHIFT		0
# define O_SLOT_CHAN_MASK		(0x1f << O_SLOT_CHAN_SHIFT)
/* TDM_SETTINGS_5 .. 9 */
# define I_SLOT_SIZE_SHIFT		6
# define I_SLOT_SIZE_MASK		(0x3 << I_SLOT_SIZE_SHIFT)
# define I_SLOT_CHAN_SHIFT		0
# define I_SLOT_CHAN_MASK		(0x1f << I_SLOT_CHAN_SHIFT)
/* TDM_SETTINGS_10 .. 16 */
# define O_DL_SHIFT			6
# define O_DL_MASK			(0x3 << O_DL_SHIFT)
# define I_DL_SHIFT			6
# define I_DL_MASK			(0x3 << I_DL_SHIFT)

#define IVM6303_PAGE_SELECTION		0xfe

#define IVM6303_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define IVM6303_I2S_DAI 0
#define IVM6303_TDM_DAI 1

struct ivm6303_platform_data {
};

struct ivm6303_priv {
	struct i2c_client	*i2c_client;
	struct regmap		*regmap;
};

static int playback_mode_control_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int playback_mode_control_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	return -EPERM;
}

static const struct snd_kcontrol_new playback_mode_control[] = {
	SOC_SINGLE_EXT("Playback mode",
		       0, 0, 1, 0, playback_mode_control_get,
		       playback_mode_control_put),
};

int tdm_in_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c, int e)
{
	pr_debug("%s, event %d, stream %s\n", __func__, e, w->sname);
	return 0;
}

int i2s_in_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c, int e)
{
	pr_debug("%s, event %d, stream %s\n", __func__, e, w->sname);
	return 0;
}

int tdm_out_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c, int e)
{
	pr_debug("%s, event %d, stream %s\n", __func__, e, w->sname);
	return 0;
}

int i2s_out_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c, int e)
{
	pr_debug("%s, event %d, stream %s\n", __func__, e, w->sname);
	return 0;
}

int playback_mode_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c,
			int e)
{
	pr_debug("%s: event %d, stream %s\n", __func__, e, w->sname);
	return 0;
}

static const struct snd_soc_dapm_widget ivm6303_dapm_widgets[] = {
	/* PLL */
	SND_SOC_DAPM_SUPPLY("PLL", IVM6303_ENABLES_SETTINGS(1), 3, 0, NULL, 0),
	/* Analog Output */
	SND_SOC_DAPM_OUTPUT("SPK"),
	/* TDM INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN_E("AIF TDM IN", "TDM Playback", 0,
			      SND_SOC_NOPM, 0, 0,
			      tdm_in_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	/* I2S INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN_E("AIF I2S IN", "I2S Playback", 0,
			      SND_SOC_NOPM, 0, 0,
			      i2s_in_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("CLASS-D", IVM6303_ENABLES_SETTINGS(5), 0, 0,
			   playback_mode_control, 1,
			   playback_mode_event,
			   SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("AIF CH1-2 I2S OUT", "I2S Capture", 0,
			       SND_SOC_NOPM, 0, 0,
			       i2s_out_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("AIF CH1-2 TDM OUT", "TDM Capture", 0,
			       SND_SOC_NOPM, 0, 0,
			       tdm_out_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("AIF CH3-4 TDM OUT", "TDM Capture", 0,
			       SND_SOC_NOPM, 0, 0,
			       tdm_out_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route ivm6303_dapm_routes[] = {
	/* sink | control | source */
	{"AIF TDM IN", NULL, "PLL" },
	{"AIF I2S IN", NULL, "PLL" },
	{"CLASS-D", NULL, "AIF TDM IN"},
	{"CLASS-D", NULL, "AIF I2S IN"},
	{"SPK", NULL, "CLASS-D"},
	{"AIF CH1-2 I2S OUT", NULL, "PLL"},
	{"AIF CH1-2 TDM OUT", NULL, "PLL"},
	{"AIF CH3-4 TDM OUT", NULL, "PLL"},
};

static int ivm6303_component_probe(struct snd_soc_component *component)
{
	return 0;
}

static void ivm6303_component_remove(struct snd_soc_component *component)
{
}

static struct snd_soc_component_driver soc_component_dev_ivm6303 = {
	.probe		= ivm6303_component_probe,
	.remove		= ivm6303_component_remove,
	.controls	= NULL,
	.num_controls	= 0,
	.dapm_widgets	= ivm6303_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ivm6303_dapm_widgets),
	.dapm_routes	= ivm6303_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ivm6303_dapm_routes),
};

static const struct regmap_range_cfg ivm6303_range_cfg[] = {
	{
		.range_min = IVM6303_SYSTEM_CTRL,
		/* From 0x100 to 0x1ff: undocumented test registers */
		.range_max = 0x1ff,
		.selector_reg = IVM6303_PAGE_SELECTION,
		.selector_mask = 1,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 256,
	}
};

static bool ivm6303_readable_register(struct device *dev, unsigned int reg)
{
	return (reg <= 0x1ff);
}

static bool ivm6303_writeable_register(struct device *dev,
				       unsigned int reg)
{
	return (reg <= 0x1ff);
}

static bool ivm6303_volatile_register(struct device *dev, unsigned int reg)
{
	return reg <= 0xff;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.ranges = ivm6303_range_cfg,
	.num_ranges = ARRAY_SIZE(ivm6303_range_cfg),
	.max_register = 0x1ff,
	.readable_reg = ivm6303_readable_register,
	.writeable_reg = ivm6303_writeable_register,
	.volatile_reg = ivm6303_volatile_register,

	.cache_type = REGCACHE_NONE,
};

static int ivm6303_dummy_hw_params(struct snd_pcm_substream *ss,
				   struct snd_pcm_hw_params *hwp,
				   struct snd_soc_dai *dai)
{
	return 0;
}

static int ivm6303_dummy_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

const struct snd_soc_dai_ops ivm6303_i2s_dai_ops = {
	.hw_params	= ivm6303_dummy_hw_params,
	.set_fmt	= ivm6303_dummy_set_fmt,
};

const struct snd_soc_dai_ops ivm6303_tdm_dai_ops = {
	.hw_params	= ivm6303_dummy_hw_params,
	.set_fmt	= ivm6303_dummy_set_fmt,
};

static struct snd_soc_dai_driver ivm6303_dais[] = {
	{
		.name = "ivm6303-i2s",
		.id = IVM6303_I2S_DAI,
		.playback = {
			.stream_name = "I2S Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = IVM6303_FORMATS,
		},
		.capture = {
			.stream_name = "I2S Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = IVM6303_FORMATS,
		},
		.ops = &ivm6303_i2s_dai_ops,
		.symmetric_rate = 1,
	},
	{
		.name = "ivm6303-tdm",
		.id = IVM6303_TDM_DAI,
		.playback = {
			.stream_name = "TDM Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = IVM6303_FORMATS,
		},
		.capture = {
			.stream_name = "TDM Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_KNOT,
			.formats = IVM6303_FORMATS,
		},
		.ops = &ivm6303_tdm_dai_ops,
		.symmetric_rate = 1,
	},
};

static int ivm6303_probe(struct i2c_client *client)
{
	struct ivm6303_priv  *priv;
	int ret = 0;

	dev_notice(&client->dev, "%s: IVM6303 driver\n", __func__);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"%s: no support for i2c read/write byte data\n",
			__func__);
		return -EIO;
	}
	priv = devm_kzalloc(&client->dev, sizeof(struct ivm6303_priv),
			    GFP_KERNEL);
	if (priv == NULL) {
		ret = -ENOMEM;
		goto end;
	}

	priv->i2c_client = client;

	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "regmap init failed\n");
	}

	i2c_set_clientdata(client, priv);
	ret = devm_snd_soc_register_component(&client->dev,
					      &soc_component_dev_ivm6303,
					      ivm6303_dais,
					      ARRAY_SIZE(ivm6303_dais));
end:
	return ret;
}


static void ivm6303_remove(struct i2c_client *client)
{
}

static const struct of_device_id ivm6303_match_table[] = {
	{ .compatible = "inventvm,ivm6303-amp", },
	{}
};

static const struct i2c_device_id ivm6303_id[] = {
	{ "ivm6303-amp", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ivm6303_id);

static struct i2c_driver ivm6303_i2c_driver = {
	.driver = {
		.name		= "ivm6303-amp",
		.owner		= THIS_MODULE,
		.of_match_table = ivm6303_match_table,
	},
	.probe		= ivm6303_probe,
	.remove		= ivm6303_remove,
	.id_table	= ivm6303_id,
};

module_i2c_driver(ivm6303_i2c_driver);

MODULE_DESCRIPTION("ASoC IVM6303 driver");
MODULE_AUTHOR("Davide Ciminaghi, ciminaghi@gnudd.com");
MODULE_LICENSE("GPL");
