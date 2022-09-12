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
#define DEBUG 1
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
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/pm_runtime.h>
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
# define REF_EN				BIT(0)
# define REF_LDO4V_EN			BIT(1)
/* Charge pump enable */
# define CP_EN				BIT(4)
# define IVM6303_REFS_MASK		(REF_EN|REF_LDO4V_EN|CP_EN)

# define PLL_EN				BIT(3)
# define PLL_CLKMUX_EN			BIT(2)
/* ENABLES_SETTINGS_2 */
# define TDM_EN				BIT(0)
/* Boost enable */
# define BST_EN				BIT(5)
/* ENABLES_SETTINGS_5 */
# define SPK_EN				BIT(0)
# define SPK_MUTE			BIT(1)

#define IVM6303_IRQ_STATUS(n)		(0x03 + (((n) - 1) << 1))
/* Interrupt Status 1 */
# define IRQ_CLK_MON_FAULT		BIT(0)
# define IRQ_PWROK_FAULT		BIT(1)
# define IRQ_TSD_LEV2_FAULT		BIT(2)
# define IRQ_TSD_LEV1_FAULT		BIT(3)
# define IRQ_TEMP_DIG_CHANGE		BIT(4)
# define IRQ_CLD_OCP_FAULT		BIT(5)
# define IRQ_PLL_LOCK_OK		BIT(6)
# define IRQ_PLL_LOCK_FAULT		BIT(7)
# define IRQ_FAULTS1_MASK (IRQ_CLK_MON_FAULT|IRQ_PWROK_FAULT|		\
			   IRQ_TSD_LEV2_FAULT|IRQ_TSD_LEV1_FAULT|	\
			   IRQ_CLD_OCP_FAULT|IRQ_PLL_LOCK_FAULT)

/* Interrupt status 2 */
# define IRQ_TDM_I_FAULT		BIT(0)
# define IRQ_TDM_FIFO_I_FAULT		BIT(1)
# define IRQ_TDM_O_FAULT		BIT(2)
# define IRQ_TDM_FIFO_O_FAULT		BIT(3)
# define IRQ_CLASSD_NG_OFF		BIT(4)
# define IRQ_CLASSD_NG_ON		BIT(5)
# define IRQ_BST_OVP_FAULT		BIT(6)
# define IRQ_BST_OCP_FAULT		BIT(7)
# define IRQ_FAULTS2_MASK (IRQ_TDM_I_FAULT|IRQ_TDM_FIFO_I_FAULT| \
			   IRQ_TDM_O_FAULT|IRQ_TDM_FIFO_O_FAULT| \
			   IRQ_BST_OVP_FAULT|IRQ_BST_OCP_FAULT)

#define IVM6303_IRQ_MASK(n)		(0x04 + (((n) - 1) << 1))

#define IVM6303_STATUS(n)		(0x09 + ((n) - 1))
# define PLL_LOCK_OK			BIT(7)

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

#define IVM6303_FORCE_INTFB		0x110

#define IVM6303_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define IVM6303_RATES (SNDRV_PCM_RATE_16000 |			\
		       SNDRV_PCM_RATE_48000 |			\
		       SNDRV_PCM_RATE_96000)

#define IVM6303_I2S_DAI 0
#define IVM6303_TDM_DAI 1

#define PLL_LOCKED_POLL_PERIOD (5 * ((HZ)/1000))
#define MAX_PLL_LOCKED_POLL_ATTS (100)

enum ivm_tdm_size {
	IVM_TDM_SIZE_16 = 1,
	IVM_TDM_SIZE_20 = 0,
	IVM_TDM_SIZE_24 = 2,
	IVM_TDM_SIZE_32 = 3,
};

enum ivm6303_section_type {
	IVM6303_NO_SECTION = 0,
	IVM6303_PROBE_WRITES = 1,
	IVM6303_PRE_PMU_WRITES,
	IVM6303_POST_PMD_WRITES,
	IVM6303_STREAM_START,
	IVM6303_STREAM_STOP,
	IVM6303_SPEAKER_MODE,
	IVM6303_RECEIVER_MODE,
	IVM6303_BIAS_OFF_TO_STANDBY,
	IVM6303_BIAS_STANDBY_TO_OFF,
	IVM6303_N_SECTIONS,
};

enum ivm6303_irq {
	/*
	 * clk_mon_fault, pwrok_fault, tsd_lev2_fault, tsd_lev1_fault,
	 * clk_ocp_fault, pll_lock_fault
	 */
	IVM6303_IRQ_FAULTS1 = 0,
	/*
	 * tdm_i_fault, tdm_fifo_i_fault, tdm_o_fault, tdm_fifo_o_fault,
	 * tdm_bst_ovp_fault, tdm_bst_ocp_fault
	 */
	IVM6303_IRQ_FAULTS2,
	IVM6303_IRQ_PLL_LOCK_OK,
};

struct ivm6303_platform_data {
};

struct ivm6303_register {
	u16 addr;
	u16 val;
	/* Delay after register write in usecs */
	unsigned int delay_us;
};

#define IVM6303_SECTION_MAX_REGISTERS 512

struct ivm6303_fw_section {
	int can_be_aborted;
	struct ivm6303_register *regs;
	int nsteps;
};

struct ivm6303_priv {
	struct workqueue_struct	*wq;
	struct delayed_work	pll_locked_work;
	struct work_struct	fw_exec_work;
	struct completion	fw_section_completion;
	struct i2c_client	*i2c_client;
	struct regmap		*regmap;
	const struct firmware	*fw;
	u8			hw_rev;
	struct mutex		regmap_mutex;
	/* Total number of stream slots */
	int			slots;
	int			slot_width;
	int			pll_locked_poll_attempts;
	/* tdm_settings_1 register */
	int			tdm_settings_1;
	/* PLL settings */
	unsigned long		pll_feedback_divider;
	unsigned long		pll_input_divider;
	struct regmap_irq_chip_data *irq_data;
	enum ivm6303_section_type  playback_mode_fw_section;
	struct ivm6303_fw_section fw_sections[IVM6303_N_SECTIONS];
	atomic_t		running_section;
	int			tdm_apply_needed;
};

/*
 * Assumes regmap mutex taken
 */
static int _run_fw_section(struct ivm6303_priv *priv,
			   struct ivm6303_fw_section *section)
{
	struct device *dev = &priv->i2c_client->dev;
	struct ivm6303_register *r;
	int i, ret = 0;

	dev_dbg(dev, "%s entered, section = %ld\n", __func__,
		section - priv->fw_sections);
	for (i = 0; i < section->nsteps; i++) {
		if (i >= IVM6303_SECTION_MAX_REGISTERS) {
			dev_err(dev, "%s, too many registers\n", __func__);
			ret = -ENOMEM;
			break;
		}
		r = &section->regs[i];
		ret = regmap_write(priv->regmap, r->addr, r->val);
		if (ret < 0) {
			dev_err(dev, "error writing to register %u", r->addr);
			break;
		}
		if (r->delay_us) {
			dev_dbg(dev, "delaying %u usecs\n", r->delay_us);
			if (r->delay_us > 1000)
				mdelay(r->delay_us / 1000);
			udelay(r->delay_us % 1000);
		}
	}
	dev_dbg(dev, "leaving %s, ret = %d\n", __func__, ret);
	return ret;
}

/*
 * Assumes regmap mutex taken
 */
static void fw_exec_handler(struct work_struct *work)
{
	struct ivm6303_priv *priv = container_of(work, struct ivm6303_priv,
						 fw_exec_work);
	int s = atomic_read(&priv->running_section);
	struct device *dev = &priv->i2c_client->dev;
	struct ivm6303_fw_section *section;

	if (s < 0) {
		dev_dbg(dev, "%s: section index is invalid\n", __func__);
		goto end;
	}
	section = &priv->fw_sections[s];
	if (!section->nsteps) {
		dev_dbg(dev, "trying to run empty section %d", s);
		goto end;
	}
	mutex_lock(&priv->regmap_mutex);
	_run_fw_section(priv, section);
	mutex_unlock(&priv->regmap_mutex);
end:
	complete(&priv->fw_section_completion);
}

static int run_fw_section(struct snd_soc_component *component, int s)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s entered, s = %d\n", __func__, s);
	if (atomic_cmpxchg(&priv->running_section, IVM6303_NO_SECTION, s)) {
		struct ivm6303_fw_section *section = &priv->fw_sections[s];

		if (section->can_be_aborted) {
			dev_dbg(dev, "%s: section can be aborted\n", __func__);
			cancel_work_sync(&priv->fw_exec_work);
		} else {
			dev_dbg(dev, "%s: waiting for completion\n", __func__);
			wait_for_completion(&priv->fw_section_completion);
		}
		atomic_set(&priv->running_section, s);
	}
	reinit_completion(&priv->fw_section_completion);
	/* Start */
	dev_dbg(dev, "%s: queueing work\n", __func__);
	return queue_work(priv->wq, &priv->fw_exec_work);
}

/* Assumes regmap lock tocken */
static int _run_fw_section_sync(struct snd_soc_component *component, int s)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct ivm6303_fw_section *section;

	if (s < 0 || s >= IVM6303_N_SECTIONS)
		return -EINVAL;
	section = &priv->fw_sections[s];
	return _run_fw_section(priv, section);
}

/* Runs a firmware section synchronously */
static int run_fw_section_sync(struct snd_soc_component *component, int s)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&priv->regmap_mutex);
	ret = _run_fw_section_sync(component, s);
	mutex_unlock(&priv->regmap_mutex);
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
	/* We have to turn off the output before mode changing */
	mutex_lock(&priv->regmap_mutex);
	ret = _save_and_switch_speaker_off(c, &spkstat);
	if (ret)
		goto err;
	ret = _run_fw_section_sync(c, priv->playback_mode_fw_section);
	_restore_enables_status(c, spkstat);
err:
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

static const struct snd_kcontrol_new playback_mode_control[] = {
	SOC_SINGLE_EXT("Speaker mode",
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

	switch(e) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Run playback mode (speaker or receiver) fw section ... */
		ret = run_fw_section(component,
				      priv->playback_mode_fw_section);
		if (ret < 0)
			break;
		/* And finally the PRE_PMU section */
		ret = run_fw_section(component, IVM6303_PRE_PMU_WRITES);
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	default:
		dev_err(component->dev, "%s: unexpected event %d\n",
			__func__, e);
	}
	if (ret < 0)
		dev_err(component->dev, "%s: error in event handling",
			__func__);
	return 0;
}

static const struct snd_soc_dapm_widget ivm6303_dapm_widgets[] = {
	/* PLL */
	SND_SOC_DAPM_SUPPLY("PLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	/* Analog Output */
	SND_SOC_DAPM_OUTPUT("SPK"),
	/* TDM INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN("AIF TDM IN", "TDM Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	/* I2S INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN("AIF I2S IN", "I2S Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA_E("CLASS-D", SND_SOC_NOPM, 0, 0,
			   playback_mode_control, 1,
			   playback_mode_event,
			   SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("AIF CH1-2 I2S OUT", "I2S Capture", 0,
			     SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF CH1-2 TDM OUT", "TDM Capture", 0,
			     SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF CH3-4 TDM OUT", "TDM Capture", 0,
			     SND_SOC_NOPM, 0, 0),
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
	return (w & 0xf000) == 0x2000;
}

static inline int is_mdelay(u16 w)
{
	return (w & 0xf000) == 0x3000;
}

static inline int is_udelay(u16 w)
{
	return (w & 0xf000) == 0x4000;
}

static inline unsigned int to_addr(u16 w)
{
	return w & ~0xf000;
}

static inline unsigned int to_val(u16 w)
{
	return w & ~0xf000;
}

/* w contains a usecs delay. This function returns a delay in usecs */
static inline unsigned int to_udelay(u16 w)
{
	return w & ~0xf000;
}

/* w contains a msecs delay. This function returns a delay in usecs */
static inline unsigned int to_mdelay(u16 w)
{
	return (w & ~0xf000) * 1000U;
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
	if (s->nsteps) {
		dev_err(component->dev, "section has already been filled");
		return -EBUSY;
	}
	if (!s->regs) {
		s->regs = devm_kzalloc(component->dev,
				       IVM6303_SECTION_MAX_REGISTERS *
				       sizeof(*s),
				       GFP_KERNEL);
		if (!s->regs) {
			dev_err(component->dev, "error allocating fw section");
			return -ENOMEM;
		}
	}
	s->nsteps = 0;
	/* Temporary: only bias off (which takes a long time) can be aborted */
	if (t == IVM6303_BIAS_STANDBY_TO_OFF)
		s->can_be_aborted = 1;
	return 0;
}

static void free_fw_section(struct snd_soc_component *component,
			    enum ivm6303_section_type t)
{
	struct ivm6303_fw_section *s;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	s = &priv->fw_sections[t];
	s->nsteps = 0;
}

static int load_fw(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	const struct firmware *fw;
	int ret, i;
	int eof_record;
	u16 *w;
	unsigned int addr = ADDR_INVALID, val = VAL_INVALID, pg = 0,
		section = IVM6303_PROBE_WRITES, reg_index = 0, delay_us = 0;
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
		if (is_mdelay(*w))
			delay_us += to_mdelay(*w);
		if (is_udelay(*w))
			delay_us += to_udelay(*w);
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
				delay_us = 0;
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
			r->delay_us = delay_us;
			priv->fw_sections[section].nsteps++;
			addr = ADDR_INVALID;
			val = VAL_INVALID;
			delay_us = 0;
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

/* Assumes regmap lock taken */
static int _poll_pll_locked(struct ivm6303_priv *priv)
{
	int ret;
	unsigned int v;

	ret = regmap_read(priv->regmap, IVM6303_STATUS(1), &v);
	if (ret < 0)
		goto err;
	ret = v & PLL_LOCK_OK ? 0 : -EIO;
err:
	return ret;
}

/* Assumes regmap mutex taken */
static void _do_tdm_apply(struct ivm6303_priv *priv)
{
	int stat;
	struct device *dev = &priv->i2c_client->dev;

	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_APPLY_CONF,
				  DO_APPLY_CONF, DO_APPLY_CONF);
	if (stat < 0)
		dev_err(dev, "%s: could not apply TDM conf", __func__);
}

static int _set_references_enable(struct ivm6303_priv *priv, int en)
{
	return regmap_update_bits(priv->regmap,
				  IVM6303_ENABLES_SETTINGS(1),
				  IVM6303_REFS_MASK,
				  en ? IVM6303_REFS_MASK : 0);
}

static void _turn_speaker_on(struct ivm6303_priv *priv)
{
	static const u8 force_intfb_vals[] = { 0x70, 0x60, };
	static const u8 leave_intfb_vals[] = { 0x00, 0x00, };
	int stat;

	/* Force internal feedback */
	stat = regmap_bulk_write(priv->regmap, IVM6303_FORCE_INTFB,
				 force_intfb_vals,
				 ARRAY_SIZE(force_intfb_vals));
	if (stat < 0)
		pr_err("Error forcing internal feedback\n");
	/* Turn on speaker */
	stat = regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(5),
				  SPK_EN, SPK_EN);
	if (stat < 0)
		pr_err("Error enabling speaker\n");
	/* Boost enable */
	stat = regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(1),
				  BST_EN, BST_EN);
	if (stat < 0)
		pr_err("Error enabling boost\n");
	/* Finally leave internal feedback */
	stat = regmap_bulk_write(priv->regmap, IVM6303_FORCE_INTFB,
				 leave_intfb_vals,
				 ARRAY_SIZE(leave_intfb_vals));
	if (stat < 0)
		pr_err("Error leaving internal feedback\n");
}

/* Assumes regmap mutex taken */
static void _pll_locked_handler(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;

	if (priv->tdm_apply_needed) {
		dev_dbg(dev, "%s: doing tdm apply\n", __func__);
		_do_tdm_apply(priv);
		priv->tdm_apply_needed = 0;
	}
	_set_references_enable(priv, 1);
	_turn_speaker_on(priv);
}

static void pll_locked_handler(struct work_struct * work)
{
	struct ivm6303_priv *priv = container_of(work, struct ivm6303_priv,
						 pll_locked_work.work);
	struct device *dev = &priv->i2c_client->dev;

	pr_debug("%s called\n", __func__);
	mutex_lock(&priv->regmap_mutex);
	if (_poll_pll_locked(priv) < 0) {
		mutex_unlock(&priv->regmap_mutex);
		dev_dbg(dev, "pll not locked\n");
		if (priv->pll_locked_poll_attempts++ >=
		    MAX_PLL_LOCKED_POLL_ATTS) {
			dev_err(dev, "pll lock timeout\n");
			return;
		}
		queue_delayed_work(priv->wq, &priv->pll_locked_work,
				   PLL_LOCKED_POLL_PERIOD);
		return;
	}
	_pll_locked_handler(priv);
	mutex_unlock(&priv->regmap_mutex);
}

static irqreturn_t ivm6303_pll_lock_ok_handler(int irq, void *_priv)
{
	struct ivm6303_priv *priv = _priv;
	struct device *dev = &priv->i2c_client->dev;
	int stat;

	/* Threaded IRQ, we can sleep */
	mutex_lock(&priv->regmap_mutex);
	_pll_locked_handler(priv);
	stat = regmap_update_bits(priv->regmap, IVM6303_IRQ_STATUS(1),
				  IRQ_PLL_LOCK_OK, IRQ_PLL_LOCK_OK);
	mutex_unlock(&priv->regmap_mutex);
	if (stat < 0)
		dev_err(dev, "error clearing irq pll lock ok interrupt\n");
	return IRQ_HANDLED;
}

/* Assumes regmap mutex taken */
static void _try_tdm_apply(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;

	priv->tdm_apply_needed = !_poll_pll_locked(priv);

	if (priv->tdm_apply_needed) {
		dev_dbg(dev, "%s: PLL not locked\n", __func__);
		return;
	}
	dev_dbg(dev, "PLL locked, applying TDM conf\n");
	_do_tdm_apply(priv);
}

static unsigned int ivm6303_component_read(struct snd_soc_component *component,
					   unsigned int reg)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int stat;
	unsigned int v = 0;

	dev_dbg(component->dev, "%s, reading %u", __func__, reg);
	mutex_lock(&priv->regmap_mutex);
	stat = regmap_read(priv->regmap, reg, &v);
	mutex_unlock(&priv->regmap_mutex);
	if (stat < 0)
		dev_err(component->dev,
			"%s, error in regmap_read (%d)\n", __func__, stat);
	return v;
}

static int ivm6303_component_write(struct snd_soc_component *component,
				   unsigned int reg, unsigned int val)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	dev_dbg(component->dev, "%s, writing to reg %u", __func__, reg);
	mutex_lock(&priv->regmap_mutex);
	ret = regmap_write(priv->regmap, reg, val);
	mutex_unlock(&priv->regmap_mutex);
	if (ret < 0)
		dev_err(component->dev,
			"%s, error in regmap_write (%d)\n", __func__, ret);
	return ret;
}

/* Assumes regmap mutex taken */
static int _set_tdm_enable(struct ivm6303_priv *priv, int en)
{
	return regmap_update_bits(priv->regmap,
				  IVM6303_ENABLES_SETTINGS(2),
				  TDM_EN, en ? TDM_EN : 0);
}

static int set_tdm_enable(struct ivm6303_priv *priv, int en)
{
	int ret;

	mutex_lock(&priv->regmap_mutex);
	ret = _set_tdm_enable(priv, en);
	mutex_unlock(&priv->regmap_mutex);
	if (ret < 0)
		dev_err(&priv->i2c_client->dev,
			"%s, error setting tdm en (%d)\n", __func__, ret);
	return ret;
}

static int ivm6303_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct device *dev = &priv->i2c_client->dev;
	const enum snd_soc_bias_level prev_level =
		snd_soc_component_get_bias_level(component);
	int ret = 0;

	dev_dbg(dev, "%s: level = %d, prev_level = %d\n", __func__, level,
		prev_level);
	switch(level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (prev_level == SND_SOC_BIAS_OFF)
			run_fw_section(component, IVM6303_BIAS_OFF_TO_STANDBY);
		if (prev_level == SND_SOC_BIAS_PREPARE) {
			/* Disable TDM */
			ret = set_tdm_enable(priv, 0);
			if (ret < 0)
				dev_err(dev, "%s: error disabling TDM\n",
					__func__);
			/* Forget about current value of tdm_settings_1 */
			priv->tdm_settings_1 = -1;
			/* Forget about current PLL settings */
			priv->pll_input_divider = 0;
			priv->pll_feedback_divider = 0;
			/* Disable references */
			ret = _set_references_enable(priv, 0);
			if (ret < 0)
				dev_err(dev, "%s: error disabling references\n",
					__func__);
		}
		break;
	case SND_SOC_BIAS_OFF:
		if (prev_level == SND_SOC_BIAS_STANDBY)
			run_fw_section(component, IVM6303_BIAS_STANDBY_TO_OFF);
		break;
	}
	return ret;
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
	priv->wq = create_singlethread_workqueue("ivm6303-wq");
	if (!priv->wq) {
		unload_fw(component);
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&priv->pll_locked_work, pll_locked_handler);
	INIT_WORK(&priv->fw_exec_work, fw_exec_handler);
	ret = run_fw_section_sync(component, IVM6303_PROBE_WRITES);
	return ret;
}

static void ivm6303_component_remove(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	cancel_delayed_work_sync(&priv->pll_locked_work);
	flush_workqueue(priv->wq);
	unload_fw(component);
}

static const char *ch3_output_mux_texts[] = {
	"Data1",
	"eq2_datao",
	"Vbatout + bst_lvl",
};

static unsigned int ch3_output_mux_values[] = {
	0, 1, 2,
};

static SOC_VALUE_ENUM_SINGLE_DECL(ch3_output_mux_enum,
				  IVM6303_TDM_SETTINGS(0x13),
				  /* shift */
				  4,
				  /* mask */
				  0x3,
				  ch3_output_mux_texts,
				  ch3_output_mux_values);

static const struct snd_kcontrol_new ch3_output_mux =
	SOC_DAPM_ENUM("ch3 output mux", ch3_output_mux_enum);

static const char *ch4_output_mux_texts[] = {
	"Data2",
	"et_datao",
	"Vol + DRCg",
};

static unsigned int ch4_output_mux_values[] = {
	0, 1, 2,
};

static SOC_VALUE_ENUM_SINGLE_DECL(ch4_output_mux_enum,
				  IVM6303_TDM_SETTINGS(0x13),
				  /* shift */
				  6,
				  /* mask */
				  0x3,
				  ch4_output_mux_texts,
				  ch4_output_mux_values);

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
	SOC_SINGLE("Data2: enable T", IVM6303_TDM_SETTINGS(0x17), 0, 1, 0),
	SOC_SINGLE("Data2: enable Vbat", IVM6303_TDM_SETTINGS(0x17), 1, 1, 0),
	SOC_SINGLE("Data2: enable Vbatout",
		   IVM6303_TDM_SETTINGS(0x17), 2, 1, 0),
	SOC_SINGLE("Data2: enable Vboost", IVM6303_TDM_SETTINGS(0x17), 3, 1, 0),
	SOC_SINGLE("Data2: enable Vol", IVM6303_TDM_SETTINGS(0x17), 4, 1, 0),
	SOC_SINGLE("Data2: enable Hdrc Gain", IVM6303_TDM_SETTINGS(0x17), 5, 1,
		   0),
	SOC_SINGLE("Data2: enable Single pole", IVM6303_TDM_SETTINGS(0x17), 6,
		   1, 0),
	SOC_SINGLE("Data2: enable Vbatout Bstlev",
		   IVM6303_TDM_SETTINGS(0x17), 7, 1, 0),
	SOC_SINGLE("Data2: enable Ldrc Gain", IVM6303_TDM_SETTINGS(0x16), 0,
		   1, 0),
	SOC_ENUM("ch3 output mux", ch3_output_mux_enum),
	SOC_ENUM("ch4 output mux", ch4_output_mux_enum),
};

static struct snd_soc_component_driver soc_component_dev_ivm6303 = {
	.probe		= ivm6303_component_probe,
	.set_bias_level = ivm6303_set_bias_level,
	.remove		= ivm6303_component_remove,
	.read		= ivm6303_component_read,
	.write		= ivm6303_component_write,
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
	int ret = 0, i, need_tdm_apply = 0;

	switch (stream) {
	case SNDRV_PCM_STREAM_CAPTURE:
		/* TDM_CHxO_DL */
		for (i = 0; i < 4; i++, need_tdm_apply++) {
			dev_dbg(component->dev,
				"%s: writing reg %2x, mask = %2x, val = %2x",
				__func__, IVM6303_TDM_SETTINGS(11 + 2*i),
				O_DL_MASK, samsize << O_DL_SHIFT);
			ret = regmap_update_bits(priv->regmap,
						 IVM6303_TDM_SETTINGS(11 + 2*i),
						 O_DL_MASK,
						 samsize << O_DL_SHIFT);
			if (ret < 0)
				break;
		}
		break;
	case SNDRV_PCM_STREAM_PLAYBACK:
		for (i = 0; i < 5; i++, need_tdm_apply++) {
			/* TDM_CHxI_DL */
			dev_dbg(component->dev,
				"%s: writing reg %2x, mask = %2x, val = %2x",
				__func__, IVM6303_TDM_SETTINGS(5 + i),
				I_DL_MASK, samsize << I_DL_SHIFT);
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
	if (need_tdm_apply)
		_try_tdm_apply(priv);
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
static int _set_pll_enable(struct ivm6303_priv *priv, int en)
{
	return regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(1),
				  PLL_EN|PLL_CLKMUX_EN,
				  en ? PLL_EN|PLL_CLKMUX_EN : 0);
}

/* Assumes regmap mutex taken */
static int _setup_pll(struct snd_soc_component *component, unsigned int bclk)
{
	int ret, ch_index;
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

	if ((pll_feedback_divider == priv->pll_feedback_divider) &&
	    (pll_input_divider == priv->pll_input_divider)) {
		dev_dbg(component->dev, "not updating pll settings\n");
		goto pll_done;
	}
	/* Start updating PLL settings by disabling PLL */
	ret = _set_pll_enable(priv, 0);
	if (ret < 0) {
		dev_err(component->dev, "error disabling pll");
		goto err;
	}
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

	/*
	 * Store current pll settings to avoid rewriting them later on
	 * if not changed
	 */
	priv->pll_input_divider = pll_input_divider;
	priv->pll_feedback_divider = pll_feedback_divider;

	/* Finally enable PLL */
	ret = _set_pll_enable(priv, 1);
	if (ret < 0) {
		dev_err(component->dev, "error restoring pll status");
		goto err;
	}

pll_done:
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

	ret = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(2), 0x7f,
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

/* Assumes regmap mutex taken */
static int _set_protocol(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int i2s, delay, fsync_edge, inverted_bclk, inverted_fsync, ret;
	unsigned int v = 0;

	i2s = delay = inverted_fsync = fsync_edge = inverted_bclk = 0;

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		inverted_bclk = 1;
		fallthrough;
	case SND_SOC_DAIFMT_NB_IF:
		inverted_fsync = 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		inverted_bclk = 1;
		break;
	default:
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s = 1;
		delay = 1;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		delay = 1;
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
	fsync_edge = (i2s == inverted_fsync);

	if (delay)
		v |= TDM_DELAY_MODE;
	if (fsync_edge)
		v |= TDM_FSYN_POLARITY;
	if (inverted_bclk)
		v |= TDM_BCLK_POLARITY;

	if (v == priv->tdm_settings_1) {
		dev_dbg(component->dev, "tdm settings 1 already setup\n");
		return 0;
	}

	ret = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(1),
				 TDM_SETTINGS_MASK, v);
	if (ret < 0) {
		dev_err(component->dev, "error writing to TDM_SETTINGS(1)");
		return ret;
	}
	priv->tdm_settings_1 = v;
	/* Finally enable TDM */
	return _set_tdm_enable(priv, 1);
}

static int ivm6303_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	int ret;
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

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
	mutex_lock(&priv->regmap_mutex);
	ret = _set_protocol(dai, fmt);
	mutex_unlock(&priv->regmap_mutex);
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
	dev_dbg(component->dev, "%s: writing reg %2x, mask %2x, val %2x",
		__func__, IVM6303_TDM_SETTINGS(3), I_SLOT_SIZE_MASK, w);
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(3),
				  I_SLOT_SIZE_MASK, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing input slot size\n");
		goto err;
	}
	dev_dbg(component->dev, "%s: writing reg %2x, mask %2x, val %2x",
		__func__, IVM6303_TDM_SETTINGS(4), O_SLOT_SIZE_MASK, w);
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(4),
				  O_SLOT_SIZE_MASK, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing output slot size\n");
		goto err;
	}
	ch = 0;
	while ((i = ffs(tx_mask))) {
		/* Tx slot i is active and assigned to channel ch */
		/* i ranges from 1 to 31, 0 means not assigned */
		dev_dbg(component->dev, "%s: writing reg %2x, val %2x",
			__func__, IVM6303_TDM_SETTINGS(0xb) + (ch << 1), i);
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
		dev_dbg(component->dev, "%s: writing reg %2x, val %2x",
			__func__, IVM6303_TDM_SETTINGS(0x5) + (ch << 1), i);
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
	int ret = 0;

	switch(cmd) {
	case SNDRV_PCM_TRIGGER_START:
		dev_dbg(component->dev, "%s, start trigger cmd\n", __func__);
		if (priv->i2c_client->irq <= 0) {
			priv->pll_locked_poll_attempts = 0;
			ret = queue_delayed_work(priv->wq,
						 &priv->pll_locked_work,
						 PLL_LOCKED_POLL_PERIOD);
		}
		break;
	default:
		break;
	}
	return ret;
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
	dev_dbg(component->dev, "%s invoked\n", __func__);
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

static int ivm6303_tdm_dai_probe(struct snd_soc_dai *dai)
{
	return snd_soc_add_component_controls(dai->component, ivm6303_ctrls,
					      ARRAY_SIZE(ivm6303_ctrls));
}

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
		.probe = ivm6303_tdm_dai_probe,
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

static struct regmap_irq ivm6303_irqs[] = {
	/* Interrupt status 1 */
	[IVM6303_IRQ_FAULTS1] = {
		.mask = IRQ_FAULTS1_MASK,
	},
	[IVM6303_IRQ_PLL_LOCK_OK] = {
		.mask = IRQ_PLL_LOCK_OK,
	},
	/* Interrupt status 2 */
	[IVM6303_IRQ_FAULTS2] = {
		.reg_offset = 2,
		.mask = IRQ_FAULTS2_MASK,
	},
};

static struct regmap_irq_chip ivm6303_irq_chip = {
	.name = "ivm6303",
	.irqs = ivm6303_irqs,
	.num_irqs = ARRAY_SIZE(ivm6303_irqs),
	.num_regs = 6,
	.status_base = IVM6303_IRQ_STATUS(1),
	.mask_base = IVM6303_IRQ_MASK(1),
	.ack_base = IVM6303_IRQ_STATUS(1),
};

static int ivm6303_irq_chip_init(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	unsigned long flags = IRQF_TRIGGER_LOW|IRQF_ONESHOT;
	int ret = -EINVAL;

	if (!priv->regmap || priv->i2c_client->irq < 0) {
		dev_err(dev, "incorrect parameters\n");
		return -EINVAL;
	}

	ret = devm_regmap_add_irq_chip(dev,
				       priv->regmap, priv->i2c_client->irq,
				       flags, 0, &ivm6303_irq_chip,
				       &priv->irq_data);
	if (ret < 0)
		dev_err(dev, "error %d from devm_regmap_add_irq_chip\n", ret);
	return ret;
}

int ivm6303_setup_irqs(struct ivm6303_priv *priv)
{
	int irq, ret;
	struct device *dev = &priv->i2c_client->dev;

	irq = regmap_irq_get_virq(priv->irq_data, IVM6303_IRQ_PLL_LOCK_OK);
	if (irq < 0) {
		dev_err(dev, "regmap_irq_get_virq() returned %d\n", irq);
		return irq;
	}
	ret = devm_request_threaded_irq(dev, irq, NULL,
					ivm6303_pll_lock_ok_handler,
					IRQF_TRIGGER_LOW|IRQF_ONESHOT,
					"ivm6303-pll-lock-ok", priv);
	if (ret)
		dev_err(dev, "devm_request_threaded_irq() returned %d\n", ret);
	ret = regmap_update_bits(priv->regmap, IVM6303_IRQ_MASK(1),
				 IRQ_PLL_LOCK_OK, IRQ_PLL_LOCK_OK);
	return ret;
}

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
	priv->tdm_settings_1 = -1;

	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "regmap init failed\n");
	}
	priv->playback_mode_fw_section = IVM6303_SPEAKER_MODE;
	init_completion(&priv->fw_section_completion);

	if (priv->i2c_client->irq > 0) {
		if (ivm6303_irq_chip_init(priv) < 0)
			priv->i2c_client->irq = 0;
		if (priv->i2c_client->irq > 0)
			if (ivm6303_setup_irqs(priv) < 0)
				priv->i2c_client->irq = 0;
	}

	i2c_set_clientdata(client, priv);
	ret = devm_snd_soc_register_component(&client->dev,
					      &soc_component_dev_ivm6303,
					      ivm6303_dais,
					      ARRAY_SIZE(ivm6303_dais));

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_request_autosuspend(&client->dev);

end:
	return ret;
}


static void ivm6303_remove(struct i2c_client *client)
{
	pm_runtime_disable(&client->dev);
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
