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
#include <linux/workqueue.h>
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
/* ENABLES_SETTINGS_2 */
# define TDM_EN				BIT(0)
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

#define IVM6303_PLL_SETTINGS(x)		((x) + 0x80)
#define PLL_POST_DIVIDER_MASK		0x0f
#define PLL_POST_DIVIDER_SHIFT		4
#define PLL_FEEDB_DIV_MSB_MASK		0x01
#define PLL_FEEDB_DIV_MSB_SHIFT		0
#define PLL_INPUT_DIVIDER_MASK		0x0f
#define PLL_INPUT_DIVIDER_SHIFT		4

#define IVM6303_PAGE_SELECTION		0xfe
#define IVM6303_HW_REV			0xff

#define IVM6303_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define IVM6303_RATES (SNDRV_PCM_RATE_16000 |			\
		       SNDRV_PCM_RATE_48000 |			\
		       SNDRV_PCM_RATE_96000)

#define IVM6303_I2S_DAI 0
#define IVM6303_TDM_DAI 1

enum ivm_tdm_size {
	IVM_TDM_SIZE_16 = 1,
	IVM_TDM_SIZE_20 = 0,
	IVM_TDM_SIZE_24 = 2,
	IVM_TDM_SIZE_32 = 3,
};

enum ivm6303_section_type {
	IVM6303_PROBE_WRITES = 1,
	IVM6303_PRE_PMU_WRITES,
	IVM6303_POST_PMD_WRITES,
	IVM6303_STREAM_START,
	IVM6303_STREAM_STOP,
	IVM6303_SPEAKER_MODE,
	IVM6303_RECEIVER_MODE,
	IVM6303_N_SECTIONS,
};

struct ivm6303_platform_data {
};

struct ivm6303_register {
	u16 addr;
	u16 val;
};

#define IVM6303_SECTION_MAX_REGISTERS 512

struct ivm6303_fw_section {
	struct ivm6303_register *regs;
	int nregs;
};

struct ivm6303_priv {
	struct delayed_work	tdm_apply_work;
	struct i2c_client	*i2c_client;
	struct regmap		*regmap;
	const struct firmware	*fw;
	u8			hw_rev;
	struct mutex		regmap_mutex;
	/* Total number of stream slots */
	int			slots;
	int			slot_width;
	int			i2s;
	int			fsync_edge;
	int			delay;
	int			inverted_fsync;
	int			inverted_bclk;
	enum ivm6303_section_type  playback_mode_fw_section;
	struct ivm6303_fw_section fw_sections[IVM6303_N_SECTIONS];
};

/*
 * Assumes regmap mutex taken
 */
static int _run_fw_section(struct snd_soc_component *component, int s)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct ivm6303_fw_section *section;
	struct ivm6303_register *r;
	int i, ret = 0;

	dev_dbg(component->dev, "running fw section %d\n", s);
	if (s < IVM6303_PROBE_WRITES || s >= IVM6303_N_SECTIONS) {
		dev_err(component->dev, "trying to run invalid section %d", s);
		return -EINVAL;
	}
	section = &priv->fw_sections[s];
	if (!section->regs) {
		dev_dbg(component->dev, "trying to run empty section %d", s);
		return ret;
	}
	for (i = 0; i < section->nregs; i++) {
		if (i >= IVM6303_SECTION_MAX_REGISTERS) {
			dev_err(component->dev, "%s, too many registers\n",
				__func__);
			ret = -ENOMEM;
			break;
		}
		r = &section->regs[i];
		ret = regmap_write(priv->regmap, r->addr, r->val);
		if (ret < 0) {
			dev_err(component->dev, "error writing to register %u",
				r->addr);
			break;
		}
	}
	return ret;
}

/*
 * Assumes regmap lock taken
 * Saves current status of bit IVM6303_ENABLES_SETTINGS(5).SPK_EN
 * and sets it to 0
 */
static int _save_and_switch_speaker_off(struct snd_soc_component *component,
					unsigned int *v)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regmap_read(priv->regmap, IVM6303_ENABLES_SETTINGS(5), v);
	if (ret < 0) {
		dev_err(component->dev, "error reading speaker status");
		return ret;
	}
	ret = regmap_write(priv->regmap, IVM6303_ENABLES_SETTINGS(5),
			   (*v) & ~SPK_EN);
	if (ret < 0)
		dev_err(component->dev, "error turning speaker off");
	return ret;
}

/*
 * Assumes regmap lock taken
 * Restore IVM6303_ENABLES_SETTINGS(5) register
 */
static void _restore_enables_status(struct snd_soc_component *component,
				    unsigned int v)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int stat;

	stat = regmap_write(priv->regmap, IVM6303_ENABLES_SETTINGS(5), v);
	if (stat < 0)
		dev_err(component->dev, "error restoring enables status");
}

static int playback_mode_control_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_dapm_kcontrol_component(kcontrol);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(c);
	unsigned int v;

	dev_dbg(c->dev, "%s called\n", __func__);
	switch(priv->playback_mode_fw_section) {
	case IVM6303_SPEAKER_MODE:
		v = 1;
		break;
	case IVM6303_RECEIVER_MODE:
		v = 0;
		break;
	default:
		return -EIO;
	}
	ucontrol->value.integer.value[0] = v;
	return 0;
}

static int playback_mode_control_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_dapm_kcontrol_component(kcontrol);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(c);
	int ret;
	unsigned int spkstat;

	dev_dbg(c->dev, "%s called (%ld)\n", __func__,
		ucontrol->value.integer.value[0]);
	switch(ucontrol->value.integer.value[0]) {
	case 0:
		priv->playback_mode_fw_section = IVM6303_RECEIVER_MODE;
		break;
	case 1:
		priv->playback_mode_fw_section = IVM6303_SPEAKER_MODE;
		break;
	default:
		return -EINVAL;
	}
	mutex_lock(&priv->regmap_mutex);
	ret = _save_and_switch_speaker_off(c, &spkstat);
	if (!ret)
		ret = _run_fw_section(c, priv->playback_mode_fw_section);
	_restore_enables_status(c, spkstat);
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

static const struct snd_kcontrol_new playback_mode_control[] = {
	SOC_SINGLE_EXT("Playback mode",
		       0, 0, 1, 0, playback_mode_control_get,
		       playback_mode_control_put),
};

int playback_mode_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c,
			int e)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;

	pr_debug("%s: event %d, stream %s\n", __func__, e, w->sname);

	mutex_lock(&priv->regmap_mutex);
	switch(e) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Run playback mode (speaker or receiver) fw section ... */
		ret = _run_fw_section(component,
				      priv->playback_mode_fw_section);
		if (ret < 0)
			break;
		/* And finally the PRE_PMU section */
		ret = _run_fw_section(component, IVM6303_PRE_PMU_WRITES);
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	default:
		dev_err(component->dev, "%s: unexpected event %d\n",
			__func__, e);
	}
	mutex_unlock(&priv->regmap_mutex);
	if (ret < 0)
		dev_err(component->dev, "%s: error in event handling",
			__func__);
	return 0;
}

static const struct snd_soc_dapm_widget ivm6303_dapm_widgets[] = {
	/* PLL */
	SND_SOC_DAPM_SUPPLY("PLL", IVM6303_ENABLES_SETTINGS(1), 3, 0, NULL, 0),
	/* Analog Output */
	SND_SOC_DAPM_OUTPUT("SPK"),
	/* TDM INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN("AIF TDM IN", "TDM Playback", 0,
			    IVM6303_ENABLES_SETTINGS(2), 0, 0),
	/* I2S INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN("AIF I2S IN", "I2S Playback", 0,
			    IVM6303_ENABLES_SETTINGS(2), 0, 0),
	SND_SOC_DAPM_PGA_E("CLASS-D", IVM6303_ENABLES_SETTINGS(5), 0, 0,
			   playback_mode_control, 1,
			   playback_mode_event,
			   SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("AIF CH1-2 I2S OUT", "I2S Capture", 0,
			     IVM6303_ENABLES_SETTINGS(2), 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF CH1-2 TDM OUT", "TDM Capture", 0,
			     IVM6303_ENABLES_SETTINGS(2), 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF CH3-4 TDM OUT", "TDM Capture", 0,
			     IVM6303_ENABLES_SETTINGS(2), 0, 0),
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

	mutex_lock(&priv->regmap_mutex);
	ret = regmap_read(priv->regmap, IVM6303_HW_REV, &rev);
	if (ret < 0)
		goto err;
	switch(rev) {
	case 0xf9:
	case 0xfa:
		priv->hw_rev = rev;
		break;
	default:
		dev_err(component->dev, "ivm6303, unknown hw rev");
		ret = -EINVAL;
	}
err:
	mutex_unlock(&priv->regmap_mutex);
	if (!ret)
		dev_info(component->dev, "ivm6303 rev %2x\n", rev);
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

static inline int is_new_section(u16 w)
{
	return (w & 0x2000) == 0x2000;
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

static int alloc_fw_section(struct snd_soc_component *component,
			    enum ivm6303_section_type t)
{
	struct ivm6303_fw_section *s;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	if (t >= IVM6303_N_SECTIONS) {
		dev_err(component->dev, "invalid section type");
		return -EINVAL;
	}
	s = &priv->fw_sections[t];
	if (s->regs) {
		dev_err(component->dev, "section has already been filled");
		return -EBUSY;
	}
	s->regs = devm_kzalloc(component->dev,
			       IVM6303_SECTION_MAX_REGISTERS * sizeof(*s),
			       GFP_KERNEL);
	if (!s->regs) {
		dev_err(component->dev, "error allocating fw section");
		return -ENOMEM;
	}
	s->nregs = 0;
	return 0;
}

static void free_fw_section(struct snd_soc_component *component,
			    enum ivm6303_section_type t)
{
	struct ivm6303_fw_section *s;
	struct ivm6303_register *tmp;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	s = &priv->fw_sections[t];
	tmp = s->regs;
	/* Lock needed ? */
	s->nregs = 0;
	s->regs = NULL;
	barrier();
	devm_kfree(component->dev, tmp);
}

static int load_fw(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	int ret, i;
	int eof_record;
	u16 *w;
	unsigned int addr = ADDR_INVALID, val = VAL_INVALID, pg = 0,
	    section = IVM6303_PROBE_WRITES, reg_index = 0;
	static char fw_file_name[MAX_FW_FILENAME_LEN];

	snprintf(fw_file_name, sizeof(fw_file_name), "ivm6303-param-%2x.bin",
		 priv->hw_rev);
	ret = request_firmware(&priv->fw, fw_file_name, component->dev);
	if (ret < 0) {
		dev_err(component->dev, "cannot load firmware");
		return ret;
	}
	fw = priv->fw;
	ret = alloc_fw_section(component, IVM6303_PROBE_WRITES);
	if (ret < 0)
		return ret;
	dev_dbg(component->dev, "firmware size = %d\n", fw->size);
	for (w = (u16 *)fw->data, i =0, eof_record = -1;
	     i < (fw->size / 2) && eof_record < 0; w++, i++) {
		if (is_file_end(*w)) {
			eof_record = i;
			break;
		}
		if (is_addr(*w))
			addr = to_addr(*w);
		if (is_val(*w))
			val = to_val(*w);
		if (is_new_section(*w)) {
			section = to_val(*w);
			dev_dbg(component->dev, "firmware: new section %u\n",
				section);
			if (section > IVM6303_PROBE_WRITES) {
				ret = alloc_fw_section(component, section);
				dev_dbg(component->dev, "alloc new section\n");
				if (ret < 0)
					return ret;
				/* Start new section from scratch */
				addr = ADDR_INVALID;
				val = VAL_INVALID;
				reg_index = 0;
			}
		}
		if (addr != ADDR_INVALID && val != VAL_INVALID) {
			struct ivm6303_register *r;

			if (addr == 254) {
				pg = val ? 0x100 : 0x000;
				addr = ADDR_INVALID;
				val = VAL_INVALID;
				continue;
			}

			r = &priv->fw_sections[section].regs[reg_index++];
			r->addr = addr | pg;
			r->val = val;
			priv->fw_sections[section].nregs++;
			addr = ADDR_INVALID;
			val = VAL_INVALID;
			if (reg_index >= IVM6303_SECTION_MAX_REGISTERS) {
				dev_err(component->dev,
					"too many registers in section %d",
					section);
				return -ENOMEM;
			}
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
	int i;

	for (i = IVM6303_PROBE_WRITES; i < IVM6303_N_SECTIONS; i++)
		free_fw_section(component, i);
	release_firmware(priv->fw);
}

static void tdm_apply_handler(struct work_struct * work)
{
	struct ivm6303_priv *priv = container_of(work, struct ivm6303_priv,
						 tdm_apply_work.work);
	int ret;
	unsigned int v = 0;

	mutex_lock(&priv->regmap_mutex);
	ret = regmap_update_bits(priv->regmap, IVM6303_TDM_APPLY_CONF,
				 DO_APPLY_CONF, DO_APPLY_CONF);
	if (ret < 0)
		pr_err("%s: error setting tdm config\n", __func__);
	udelay(1000);
	if (priv->delay)
		v |= TDM_DELAY_MODE;
	if (priv->fsync_edge)
		v |= TDM_FSYN_POLARITY;
	if (priv->inverted_bclk)
		v |= TDM_BCLK_POLARITY;
	ret = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(1),
				 TDM_SETTINGS_MASK, v);
	mutex_unlock(&priv->regmap_mutex);
	if (ret < 0)
		pr_err("%s: error setting tdm config\n", __func__);
}

static int ivm6303_component_probe(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	WARN_ON(!priv);
	if (!priv)
		return -ENODEV;
	ret = check_hw_rev(component);
	if (ret < 0)
		return ret;
	ret = load_fw(component);
	if (ret < 0)
		return ret;
	INIT_DELAYED_WORK(&priv->tdm_apply_work, tdm_apply_handler);
	snd_soc_component_init_regmap(component, priv->regmap);
	mutex_lock(&priv->regmap_mutex);
	ret = _run_fw_section(component, IVM6303_PROBE_WRITES);
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

static void ivm6303_component_remove(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	unload_fw(component);
	cancel_delayed_work_sync(&priv->tdm_apply_work);
}

static struct snd_kcontrol_new ivm6303_ctrls[] = {
	SOC_SINGLE("Data1: enable T", IVM6303_TDM_SETTINGS(0x15), 0, 1, 0),
	SOC_SINGLE("Data1: enable Vbat", IVM6303_TDM_SETTINGS(0x15), 1, 1, 0),
	SOC_SINGLE("Data1: enable Vbatout",
		   IVM6303_TDM_SETTINGS(0x15), 2, 1, 0),
	SOC_SINGLE("Data1: enable Vboost", IVM6303_TDM_SETTINGS(0x15), 3, 1, 0),
	SOC_SINGLE("Data1: enable Vol", IVM6303_TDM_SETTINGS(0x15), 4, 1, 0),
	SOC_SINGLE("Data1: enable Hdrc Gain", IVM6303_TDM_SETTINGS(0x15), 5, 1,
		   0),
	SOC_SINGLE("Data1: enable Single pole", IVM6303_TDM_SETTINGS(0x15), 6,
		   1, 0),
	SOC_SINGLE("Data1: enable Vbatout Bstlev",
		   IVM6303_TDM_SETTINGS(0x15), 7, 1, 0),
	SOC_SINGLE("Data1: enable Ldrc Gain", IVM6303_TDM_SETTINGS(0x14), 0,
		   1, 0),
};

static struct snd_soc_component_driver soc_component_dev_ivm6303 = {
	.probe		= ivm6303_component_probe,
	.remove		= ivm6303_component_remove,
	.controls	= ivm6303_ctrls,
	.num_controls	= ARRAY_SIZE(ivm6303_ctrls),
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
	.disable_locking = 1,
};

/* Assumes regmap mutex taken */
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

/*
 * PLL input divider table
 *
 * Frequencies:
 * 16 * 1 => 16
 * 16 * 3 => 48
 * 16 * 6 => 96
 *
 * For each frequency
 * 2 channels
 * 4 channels
 * 8 channels
 * 16 channels
 *
 * Always 32 bits per slot
 */
enum chan_index {
	CH2 = 0,
	CH4 = 1,
	CH8 = 2,
	CH16 = 3,
	MAX_CHANNELS,
};

static inline int chan_to_index(int ch)
{
	/* ch must be a power of 2 */
	if (ch & (ch - 1))
		return -EINVAL;
	return ilog2(ch) - 1;
}

#define RATEK(r) ((r)/16000)

enum fsyn_rates {
	RATE_16K = RATEK(16000),
	RATE_48K = RATEK(48000),
	RATE_96K = RATEK(96000),
	MAX_FSYN_RATES
};

static const unsigned int ivm6303_fsyn[MAX_FSYN_RATES] = {
	[RATE_16K] = 0x10,
	[RATE_48K] = 0x20,
	[RATE_96K] = 0x30,
};

#define OSR(r)  ((r) / 32)

#define OSR_64  OSR(64)
#define OSR_96  OSR(96)
#define OSR_128 OSR(128)
#define OSR_192 OSR(192)
#define OSR_256 OSR(256)
#define OSR_288 OSR(288)
#define OSR_384 OSR(384)
#define OSR_512 OSR(512)

static const unsigned int ivm6303_osr[] = {
	[OSR_64] =  1,
	[OSR_96] =  2,
	[OSR_128] = 3,
	[OSR_192] = 4,
	[OSR_256] = 5,
	[OSR_288] = 6,
	[OSR_384] = 7,
	[OSR_512] = 8,
};

static const unsigned int
ivm6303_pll_input_div[MAX_FSYN_RATES][MAX_CHANNELS] = {
	[RATE_16K] = {
		[CH2] = 1,
		[CH4] = 1,
		[CH8] = 2,
		[CH16] = 4,
	},
	[RATE_48K] = {
		[CH2] = 1,
		[CH4] = 2,
		[CH8] = 4,
		[CH16] = 8,
	},
	[RATE_96K] = {
		[CH2] = 2,
		[CH4] = 4,
		[CH8] = 8,
	},
};

#define VCO_FREQ 98304000UL
#define PLL_POST_DIVIDER 2

/* Assumes regmap mutex taken */
static int _disable_pll(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int v;
	int ret, out;

	ret = regmap_read(priv->regmap, IVM6303_ENABLES_SETTINGS(1), &v);
	if (ret < 0) {
		dev_err(component->dev, "error reading pll enable bit");
		return ret;
	}

	out = v & PLL_EN;

	if (!out)
		/* Already disabled */
		return out;
	ret = regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(1),
				 PLL_EN, 0);
	if (ret < 0)
		dev_err(component->dev, "error writing pll enable bit");
	return ret < 0 ? ret : out;
}

/* Assumes regmap mutex taken */
static int _restore_pll(struct snd_soc_component *component, int en)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	ret = regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(1),
				 PLL_EN, en);
	if (ret < 0)
		dev_err(component->dev, "error restoring pll enable bit");
	return ret;
}

/* Assumes regmap mutex taken */
static int _setup_pll(struct snd_soc_component *component, unsigned int bclk)
{
	int ret, ch_index, pll_status;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned long osr, rate, ratek, refclk, pll_input_divider,
		pll_feedback_divider;
	u8 tdm_fsyn_sr, tdm_bclk_osr, shift, mask, v;

	osr = priv->slots * priv->slot_width;
	rate = bclk / osr;
	ratek = RATEK(rate);
	if (ratek < RATE_16K || ratek >= MAX_FSYN_RATES) {
		dev_err(component->dev, "unsupported fsyn rate %lu\n", rate);
		return -EINVAL;
	}
	ch_index = chan_to_index(priv->slots);
	if (ch_index < CH2 || ch_index > CH16) {
		dev_err(component->dev, "unsupported number of channels");
		return -EINVAL;
	}
	pll_input_divider = ivm6303_pll_input_div[ratek][ch_index];
	if (!pll_input_divider) {
		dev_err(component->dev, "unsupported channels/rate combo");
		return -EINVAL;
	}
	dev_dbg(component->dev, "pll_input_divider = %lu\n", pll_input_divider);
	refclk = bclk / pll_input_divider;
	dev_dbg(component->dev, "refclk = %lu", refclk);
	pll_feedback_divider = VCO_FREQ / refclk;
	dev_dbg(component->dev, "feedback divider = %lu", pll_feedback_divider);

	/* Disable PLL */
	ret = _disable_pll(component);
	if (ret < 0) {
		dev_err(component->dev, "error disabling pll");
		goto err;
	}
	pll_status = ret;
	/* Write post divider */
	shift = PLL_POST_DIVIDER_SHIFT;
	mask = PLL_POST_DIVIDER_MASK;
	ret = regmap_update_bits(priv->regmap, IVM6303_PLL_SETTINGS(0),
				 mask << shift, (PLL_POST_DIVIDER/2) << shift);
	if (ret < 0) {
		dev_err(component->dev, "error writing post divider");
		goto err;
	}
	/* Write feedback divider msb */
	shift = PLL_FEEDB_DIV_MSB_SHIFT;
	mask = PLL_FEEDB_DIV_MSB_MASK;
	v = (pll_feedback_divider & 0x100) >> 8;
	ret = regmap_update_bits(priv->regmap, IVM6303_PLL_SETTINGS(0),
				 mask << shift, v << shift);
	if (ret < 0) {
		dev_err(component->dev, "error writing feedback div. msb");
		goto err;
	}
	/* Write feedback divider lsb */
	v = pll_feedback_divider & 0xff;
	ret = regmap_write(priv->regmap, IVM6303_PLL_SETTINGS(1), v);
	if (ret < 0) {
		dev_err(component->dev, "error writing feedback div.");
		return ret;
	}
	/* Write input divider */
	shift = PLL_INPUT_DIVIDER_SHIFT;
	mask = PLL_INPUT_DIVIDER_MASK;
	ret = regmap_update_bits(priv->regmap, IVM6303_PLL_SETTINGS(2),
				 mask << shift, pll_input_divider);
	if (ret < 0) {
		dev_err(component->dev,"error writing input divider");
		goto err;
	}

	/* Restore original pll status */
	ret = _restore_pll(component, pll_status);
	if (ret < 0) {
		dev_err(component->dev, "error restoring pll status");
		goto err;
	}

	tdm_fsyn_sr = ivm6303_fsyn[ratek];

	if ((OSR(osr) < OSR_64) || (OSR(osr) > OSR_512)) {
		dev_err(component->dev, "invalid osr %lu", osr);
		ret = -EINVAL;
		goto err;
	}

	tdm_bclk_osr = ivm6303_osr[OSR(osr)];
	/*
	 * Table is sparse, 0 indicates osr is not supported
	 */
	if (!tdm_bclk_osr) {
		dev_err(component->dev, "unsupported osr %lu", osr);
		ret = -EINVAL;
		goto err;
	}

	ret = regmap_update_bits(priv->regmap, 0x32, 0x7f,
				 tdm_fsyn_sr | tdm_bclk_osr);
err:
	return ret;
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

	if (rate < 16000 && rate > 96000) {
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

	mutex_lock(&priv->regmap_mutex);

	/* Set PLL given bclk */
	ret = _setup_pll(component, bclk);
	if (ret < 0)
		goto err;

	/* Set samples and slots sizes */
	ret = _set_sam_size(component, substream->stream, samsize);
err:
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

/* Doesn't actually write to registers */
static int set_protocol(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	priv->i2s = priv->delay = priv->inverted_fsync =
		priv->fsync_edge = priv->inverted_bclk = 0;

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
	ret= set_protocol(dai, fmt);
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

	mutex_lock(&priv->regmap_mutex);
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(3),
				  I_SLOT_SIZE_MASK << I_SLOT_SIZE_SHIFT, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing input slot size\n");
		goto err;
	}
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(4),
				  O_SLOT_SIZE_MASK << O_SLOT_SIZE_SHIFT, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing output slot size\n");
		goto err;
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
			goto err;
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
			goto err;
		}
		rx_mask &= ~(1 << (i - 1));
		ch++;
	}
err:
	mutex_unlock(&priv->regmap_mutex);
	return stat;
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
	mutex_lock(&priv->regmap_mutex);
	for (i = 0; i < tx_num; i++) {
		v = tx_slot[i] + 1;
		r = IVM6303_TDM_SETTINGS(0xb) + (i << 1);
		stat = regmap_write(priv->regmap, r, v);
		if (stat < 0) {
			dev_err(component->dev, "Error writing register %u\n",
				r);
			goto err;
		}
	}
	if (rx_num > 1) {
		dev_err(component->dev, "Invalid number of rx channels");
		stat = -EINVAL;
		goto err;
	}
	v = rx_slot[0] + 1;
	r = IVM6303_TDM_SETTINGS(0x5);
	stat = regmap_write(priv->regmap, r, v);
	if (stat < 0)
		dev_err(component->dev, "Error writing register %u\n", r);
err:
	mutex_unlock(&priv->regmap_mutex);
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
	mutex_lock(&priv->regmap_mutex);
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
			goto err;
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
		goto err;
	}
	v &= 0x1f;
	if (v) {
		*rx_slot++ = v - 1;
		(*rx_num)++;
	}
err:
	mutex_unlock(&priv->regmap_mutex);
	return stat;
}

static int ivm6303_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	if (cmd == SNDRV_PCM_TRIGGER_START) {
		dev_dbg(component->dev, "%s, start trigger cmd\n", __func__);
		return schedule_delayed_work(&priv->tdm_apply_work, HZ/100);
	}
	return 0;
}

static int ivm6303_dai_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		/* Ignore mute on capture */
		return 0;

	dev_dbg(component->dev, "%s(): mute = %d\n", __func__, mute);
	mutex_lock(&priv->regmap_mutex);
	ret = regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(5),
				 SPK_MUTE, mute ? SPK_MUTE : 0);
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

static int ivm6303_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	int ret;

	/* Setup for mono playback and stereo capture (Vs/Is) */
	ret = ivm6303_set_tdm_slot(dai, 0x3, 0x1, 2, 32);
	if (ret < 0) {
		dev_err(component->dev,
			"Cannot setup slots for I2S dai operation");
		return ret;
	}
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(component->dev, "Non I2S format requested for I2S dai");
		return -EINVAL;
	}
	return ivm6303_set_fmt(dai, fmt);
}

const struct snd_soc_dai_ops ivm6303_i2s_dai_ops = {
	.hw_params	= ivm6303_hw_params,
	.set_fmt	= ivm6303_i2s_set_fmt,
	.trigger	= ivm6303_dai_trigger,
	.mute_stream	= ivm6303_dai_mute,
};

const struct snd_soc_dai_ops ivm6303_tdm_dai_ops = {
	.hw_params	= ivm6303_hw_params,
	.set_fmt	= ivm6303_set_fmt,
	.set_tdm_slot   = ivm6303_set_tdm_slot,
	.set_channel_map = ivm6303_set_channel_map,
	.get_channel_map = ivm6303_get_channel_map,
	.trigger	= ivm6303_dai_trigger,
	.mute_stream	= ivm6303_dai_mute,
};

static struct snd_soc_dai_driver ivm6303_dais[] = {
	{
		.name = "ivm6303-i2s",
		.id = IVM6303_I2S_DAI,
		.playback = {
			.stream_name = "I2S Playback",
			.channels_min = 1,
			.channels_max = 1,
			.rates = IVM6303_RATES,
			.formats = IVM6303_FORMATS,
		},
		.capture = {
			.stream_name = "I2S Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = IVM6303_RATES,
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
			.rates = IVM6303_RATES,
			.formats = IVM6303_FORMATS,
		},
		.capture = {
			.stream_name = "TDM Capture",
			.channels_min = 1,
			.channels_max = 16,
			.rates = IVM6303_RATES,
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
	mutex_init(&priv->regmap_mutex);

	priv->i2c_client = client;

	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "regmap init failed\n");
	}
	priv->playback_mode_fw_section = IVM6303_SPEAKER_MODE;

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
