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
#include <linux/firmware.h>
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
#define IVM6303_HW_REV			0xff

#define IVM6303_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define IVM6303_I2S_DAI 0
#define IVM6303_TDM_DAI 1

enum ivm_tdm_size {
	IVM_TDM_SIZE_16 = 1,
	IVM_TDM_SIZE_20 = 0,
	IVM_TDM_SIZE_24 = 2,
	IVM_TDM_SIZE_32 = 3,
};

struct ivm6303_platform_data {
};

struct ivm6303_priv {
	struct i2c_client	*i2c_client;
	struct regmap		*regmap;
	const struct firmware	*fw;
	u8			hw_rev;
	/* Total number of stream slots */
	int			slots;
	int			slot_width;
	int			i2s;
	int			fsync_edge;
	int			delay;
	int			inverted_fsync;
	int			inverted_bclk;
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

static int check_hw_rev(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;
	unsigned int rev;

	ret = regmap_read(priv->regmap, IVM6303_HW_REV, &rev);
	dev_info(component->dev, "ivm6303 rev %2x\n", rev);
	switch(rev) {
	case 0xf9:
	case 0xfa:
		priv->hw_rev = rev;
		break;
	default:
		dev_err(component->dev, "ivm6303, unknown hw rev");
		ret = -EINVAL;
	}
	return ret;
}

#define ADDR_INVALID 0xffffffff
#define VAL_INVALID 0xffffffff

static inline int is_addr(u16 w)
{
	return (w & 0xf000) == 0x1000;
}

static inline int is_val(u16 w)
{
	return (w & 0xf000) == 0x0000;
}

static inline int is_file_end(u16 w)
{
	return (w & 0xf000) == 0xf000;
}

static inline unsigned int to_addr(u16 w)
{
	return w & ~0xf000;
}

static inline unsigned int to_val(u16 w)
{
	return w & ~0xf000;
}

#define MAX_FW_FILENAME_LEN 256

static int load_fw(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	int ret, i;
	int eof_record;
	u16 *w;
	unsigned int addr = ADDR_INVALID, val = VAL_INVALID, pg = 0;
	static char fw_file_name[MAX_FW_FILENAME_LEN];

	snprintf(fw_file_name, sizeof(fw_file_name), "ivm6303-param-%2x.bin",
		 priv->hw_rev);
	ret = request_firmware(&priv->fw, fw_file_name, component->dev);
	if (ret < 0) {
		dev_err(component->dev, "cannot load firmware");
		return ret;
	}
	fw = priv->fw;
	for (w = (u16 *)fw->data, i =0, eof_record = -1;
	     i < (fw->size / 2) && !eof_record; w++, i++) {
		if (is_file_end(*w)) {
			eof_record = i;
			break;
		}
		if (is_addr(*w))
			addr = to_addr(*w);
		if (is_val(*w))
			val = to_val(*w);
		if (addr != ADDR_INVALID && val != VAL_INVALID) {
			if (addr == 254) {
				pg = val ? 0x100 : 0x000;
				addr = ADDR_INVALID;
				val = VAL_INVALID;
				continue;
			}

			/* Write register */
			ret = regmap_write(priv->regmap, addr | pg, val);
			if (ret < 0) {
				dev_err(component->dev,
					"error writing register");
				break;
			}
			addr = ADDR_INVALID;
			val = VAL_INVALID;
		}
	}
	dev_info(component->dev, "firmware loaded");
	if (eof_record < 0)
		dev_warn(component->dev, "no end of file firmware record");
	return ret;
}

static void unload_fw(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	release_firmware(priv->fw);
}

static int ivm6303_component_probe(struct snd_soc_component *component)
{
	int ret;

	ret = check_hw_rev(component);
	if (ret < 0)
		return ret;
	return load_fw(component);
}

static void ivm6303_component_remove(struct snd_soc_component *component)
{
	unload_fw(component);
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

static int _set_sam_size(struct snd_soc_component *component,
			 unsigned int stream,
			 unsigned int samsize)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0, i;

	switch (stream) {
	case SNDRV_PCM_STREAM_CAPTURE:
		/* TDM_CHxO_DL */
		for (i = 0; i < 4; i++) {
			ret = regmap_update_bits(priv->regmap,
						 IVM6303_TDM_SETTINGS(10 + 2*i),
						 O_DL_MASK,
						 samsize << O_DL_SHIFT);
			if (ret < 0)
				break;
		}
		break;
	case SNDRV_PCM_STREAM_PLAYBACK:
		for (i = 0; i < 5; i++) {
			/* TDM_CHxI_DL */
			ret = regmap_update_bits(priv->regmap,
						 IVM6303_TDM_SETTINGS(5 + i),
						 I_DL_MASK,
						 samsize << I_DL_SHIFT);
			if (ret < 0)
				break;
		}
		break;
	default:
		dev_err(component->dev, "%s: invalid stream\n", __func__);
		ret = -EINVAL;
	}
	return ret;
}

static int _setup_pll(struct snd_soc_component *component, unsigned int bclk)
{
	int ret;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned long osr, rate;
	/* See register 0x32 */
	u8 tdm_fsyn_sr, tdm_bclk_osr;

	/* Disable PLL */
	regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(1),
			   PLL_EN, 0);
	switch (bclk) {
	case 12288000:
		/* 12.288MHz */
		regmap_write(priv->regmap, 0x80, 0x10);
		regmap_write(priv->regmap, 0x81, 0x20);
		regmap_write(priv->regmap, 0x82, 0x40);
		break;
	case 6144000:
		/* 6.144MHz */
		regmap_write(priv->regmap, 0x80, 0x10);
		regmap_write(priv->regmap, 0x81, 0x20);
		regmap_write(priv->regmap, 0x82, 0x20);
		break;
	case 3072000:
		/* 3.072MHz */
		regmap_write(priv->regmap, 0x80, 0x10);
		regmap_write(priv->regmap, 0x81, 0x60);
		regmap_write(priv->regmap, 0x82, 0x30);
		break;
	case 1536000:
		/* 1.536MHz: FIXME: IS THIS CORRECT ? */
		regmap_write(priv->regmap, 0x80, 0x20);
		regmap_write(priv->regmap, 0x81, 0x60);
		regmap_write(priv->regmap, 0x82, 0x30);
		break;
	default:
		dev_err(component->dev, "unsupported bclk\n");
		return -EINVAL;
	}
	/* Re-enable PLL */
	ret = regmap_update_bits(priv->regmap,
				 IVM6303_ENABLES_SETTINGS(1), PLL_EN, PLL_EN);
	if (ret < 0)
		return ret;

	/*
	 * Also setup TDM registers (FIXME: this should actually belong
	 *  to another method ...)
	 */
	osr = priv->slots * priv->slot_width;
	rate = bclk / osr;

	switch(rate) {
	case 16000:
		tdm_fsyn_sr = 0x10;
		break;
	case 48000:
		tdm_fsyn_sr = 0x20;
		break;
	case 96000:
		tdm_fsyn_sr = 0x30;
		break;
	case 192000:
		tdm_fsyn_sr = 0x40;
		break;
	default:
		dev_err(component->dev, "unsupported rate %lu\n", rate);
		break;
	}

	switch (osr) {
	case 64:
		tdm_bclk_osr = 1;
		break;
	case 96:
		tdm_bclk_osr = 2;
		break;
	case 128:
		tdm_bclk_osr = 3;
		break;
	case 192:
		tdm_bclk_osr = 4;
		break;
	case 256:
		tdm_bclk_osr = 5;
		break;
	case 288:
		tdm_bclk_osr = 6;
		break;
	case 384:
		tdm_bclk_osr = 7;
		break;
	case 512:
		tdm_bclk_osr = 8;
		dev_err(component->dev,
			"unsupported oversample rate %lu\n", rate);
		break;
	}
	return regmap_update_bits(priv->regmap, 0x32, 0x7f,
				  tdm_fsyn_sr | tdm_bclk_osr);
}

static int ivm6303_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *codec_dai)
{
	struct snd_soc_component *component = codec_dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int bps = params_width(params);
	unsigned int pbps = params_physical_width(params);
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params), minch, maxch;
	unsigned int samsize, bclk;
	int ret;

	dev_dbg(component->dev,
		"%s(): entry , bps : %u , rate : %u, channels : %u\n",
		__func__, bps, rate, channels);

	if (bps != 16 && bps != 24 && bps != 32) {
		dev_err(component->dev, "invalid bits per sample %u\n", bps);
		return -EINVAL;
	}

	if (bps > priv->slot_width) {
		dev_err(component->dev, "Requested slotsize is too big\n");
		return -EINVAL;
	}

	if (pbps < 16 || pbps > 32) {
		dev_err(component->dev, "invalid phy size %u\n", pbps);
		return -EINVAL;
	}

	if (rate != 48000 && rate != 96000) {
		dev_err(component->dev, "invalid rate %u\n", rate);
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		minch = codec_dai->driver->playback.channels_min;
		maxch = codec_dai->driver->playback.channels_max;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		minch = codec_dai->driver->capture.channels_min;
		maxch = codec_dai->driver->capture.channels_max;
	}

	if (channels > maxch || channels < minch) {
		dev_err(component->dev, "invalid chan num %u for stream\n",
			channels);
		return -EINVAL;
	}

	bclk = rate;

	switch (bps) {
	case 16:
		samsize = IVM_TDM_SIZE_16;
		break;
	case 24:
	case 32:
		samsize = IVM_TDM_SIZE_24;
		break;
	default:
		/* NEVER REACHED */
		return -EINVAL;
	}

	bclk *= priv->slots * priv->slot_width;
	dev_dbg(component->dev, "bclk = %uHz\n", bclk);

	/* Set PLL given bclk */
	ret = _setup_pll(component, bclk);
	if (ret < 0)
		return ret;

	/* Set samples and slots sizes */
	return _set_sam_size(component, substream->stream, samsize);
}

static int _set_protocol(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	priv->i2s = priv->delay = priv->inverted_fsync =
		priv->fsync_edge = priv->inverted_bclk = 0;

	if ((dai->id == IVM6303_I2S_DAI) &&
	    ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)) {
		dev_err(component->dev, "Non I2S format requested for I2S dai");
		return -EINVAL;
	}
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		priv->inverted_bclk = 1;
		/* FALLTHROUGH */
	case SND_SOC_DAIFMT_NB_IF:
		priv->inverted_fsync = 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		priv->inverted_bclk = 1;
		break;
	default:
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		priv->i2s = 1;
		priv->delay = 1;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		priv->delay = 1;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		dev_err(component->dev, "unsupported protocol %d\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	/*
	 * Truth table
	 *
	 * i2s  |   inverted_fsync  | fsync_edge
	 * -----+-------------------+------------
	 *  0   |         0         |      1
	 *  0   |         1         |      0
	 *  1   |         0         |      0
	 *  1   |         1         |      1
	 */
	priv->fsync_edge = (priv->i2s == priv->inverted_fsync);

	return 0;
}

static int ivm6303_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	int ret;
	struct snd_soc_component *component = dai->component;

	dev_dbg(component->dev, "%s(): fmt = 0x%08x\n", __func__, fmt);

	/* Master/slave configuration */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(component->dev,
			"%s: codec can only be slave\n", __func__);
		return -EINVAL;
	}
	/* Clock gating */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_CONT: /* continuous clock */
		dev_dbg(component->dev, "%s: IF0 Clock is continuous.\n",
			__func__);
		break;
	case SND_SOC_DAIFMT_GATED: /* clock is gated */
		dev_dbg(component->dev, "%s: IF0 Clock is gated.\n",
			__func__);
		break;
	default:
		dev_err(component->dev,
			"%s: ERROR: Unsupported clock mask (0x%x)!\n",
			__func__, fmt & SND_SOC_DAIFMT_CLOCK_MASK);
		return -EINVAL;
	}
	ret= _set_protocol(dai, fmt);
	if (ret < 0)
		return ret;
	return 0;
}

static int ivm6303_set_tdm_slot(struct snd_soc_dai *dai,
				unsigned int tx_mask,
				unsigned int rx_mask,
				int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int w, i, ch;
	int stat;

	if (dai->id != IVM6303_TDM_DAI) {
		dev_err(component->dev,
			"set_tdm_slot called for a non tdm dai");
		return -ENOTSUPP;
	}
	if (!slots) {
		/* Disable TDM, error for the moment */
		dev_err(component->dev,
			"This is a TDM DAI, you can't disable TDM\n");
		return -EINVAL;
	}
	if (slots > 31) {
		dev_err(component->dev, "Max 31 slots supported\n");
		return -EINVAL;
	}
	if (hweight32(tx_mask) > 4) {
		dev_err(component->dev, "Max 4 tx slots supported\n");
		return -EINVAL;
	}
	if (hweight32(rx_mask) > 1) {
		dev_err(component->dev, "Max 1 rx slots supported\n");
		return -EINVAL;
	}
	switch (slot_width) {
	case 16:
		w = 1 << I_SLOT_SIZE_SHIFT;
		break;
	case 20:
		w = 0 << I_SLOT_SIZE_SHIFT;
		break;
	case 24:
		w = 2 << I_SLOT_SIZE_SHIFT;
		break;
	case 32:
		w = 3 << I_SLOT_SIZE_SHIFT;
		break;
	default:
		dev_err(component->dev,
			"%s: Unsupported slot width 0x%x\n",
			__func__, slot_width);
		return -EINVAL;
	}
	priv->slots = slots;
	priv->slot_width = slot_width;

	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(3),
				  I_SLOT_SIZE_MASK << I_SLOT_SIZE_SHIFT, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing input slot size\n");
		return stat;
	}
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(4),
				  O_SLOT_SIZE_MASK << O_SLOT_SIZE_SHIFT, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing output slot size\n");
		return stat;
	}
	ch = 0;
	while ((i = ffs(tx_mask))) {
		/* Tx slot i is active and assigned to channel ch */
		/* i ranges from 1 to 31, 0 means not assigned */
		stat = regmap_write(priv->regmap,
				    IVM6303_TDM_SETTINGS(0xb) + (ch << 1),
				    i);
		if (stat < 0) {
			dev_err(component->dev, "error setting up tx slot\n");
			return stat;
		}
		tx_mask &= ~(1 << (i - 1));
		ch++;
	}
	ch = 0;
	while ((i = ffs(rx_mask))) {
		/* Rx slot i is active and assigned to channel ch */
		/* i ranges from 1 to 31, 0 means not assigned */
		stat = regmap_write(priv->regmap,
				    IVM6303_TDM_SETTINGS(0x5) + (ch << 1),
				    i);
		if (stat < 0) {
			dev_err(component->dev, "error setting up tx slot\n");
			return stat;
		}
		rx_mask &= ~(1 << (i - 1));
		ch++;
	}
	return 0;
}

static int ivm6303_set_channel_map(struct snd_soc_dai *dai,
				   unsigned int tx_num,
				   unsigned int *tx_slot,
				   unsigned int rx_num,
				   unsigned int *rx_slot)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int i, v, r;
	int stat;

	if (tx_num > 4) {
		dev_err(component->dev, "Invalid number of tx channels");
		return -EINVAL;
	}
	for (i = 0; i < tx_num; i++) {
		v = tx_slot[i] + 1;
		r = IVM6303_TDM_SETTINGS(0xb) + (i << 1);
		stat = regmap_write(priv->regmap, r, v);
		if (stat < 0) {
			dev_err(component->dev, "Error writing register %u\n",
				r);
			return stat;
		}
	}
	if (rx_num > 1) {
		dev_err(component->dev, "Invalid number of rx channels");
		return -EINVAL;
	}
	v = rx_slot[0] + 1;
	r = IVM6303_TDM_SETTINGS(0x5);
	stat = regmap_write(priv->regmap, r, v);
	if (stat < 0)
		dev_err(component->dev, "Error writing register %u\n", r);
	return stat;
}

static int ivm6303_get_channel_map(struct snd_soc_dai *dai,
				   unsigned int *tx_num,
				   unsigned int *tx_slot,
				   unsigned int *rx_num,
				   unsigned int *rx_slot)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int stat, ch;
	unsigned int v, r;

	/* tx -> output channels (1-4) */
	/*
	 * For each channel. Note that 4 output channels are available,
	 * and we assume that all active channels are consecutive:
	 * for instance the sequence 1,2,4 is __not__ allowed. You __must__
	 * have 1,2,3
	 */
	for (ch = 0, *tx_num = 0; ch < 4; ch++) {
		/*
		 * Each channel can be assigned 2 slots, we just consider
		 * the first one. So we read registers 0x3b, 0x3d, ....
		 */
		r = IVM6303_TDM_SETTINGS(0xb) + (ch << 1);
		stat = regmap_read(priv->regmap, r, &v);
		if (stat < 0) {
			dev_err(component->dev,
				"Error reading register %u\n", r);
			return stat;
		}
		/* v is equal to <slsz>|<slot # for this channel + 1 */
		v &= 0x1f;
		if (!v)
			break;
		*tx_slot++ = v - 1;
		(*tx_num)++;
	}
	/* We only consider channel 1, which is the one played back */
	r = IVM6303_TDM_SETTINGS(0x5);
	*rx_num = 0;
	stat = regmap_read(priv->regmap, r, &v);
	if (stat < 0) {
		dev_err(component->dev,
			"Error reading register %u\n", r);
		return stat;
	}
	v &= 0x1f;
	if (v) {
		*rx_slot++ = v - 1;
		(*rx_num)++;
	}
	return stat;
}

const struct snd_soc_dai_ops ivm6303_i2s_dai_ops = {
	.hw_params	= ivm6303_hw_params,
	.set_fmt	= ivm6303_set_fmt,
};

const struct snd_soc_dai_ops ivm6303_tdm_dai_ops = {
	.hw_params	= ivm6303_hw_params,
	.set_fmt	= ivm6303_set_fmt,
	.set_tdm_slot   = ivm6303_set_tdm_slot,
	.set_channel_map = ivm6303_set_channel_map,
	.get_channel_map = ivm6303_get_channel_map,
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
