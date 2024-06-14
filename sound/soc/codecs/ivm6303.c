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
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/completion.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/asound.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>
#include <linux/pm_wakeup.h>
#include <linux/kernel.h>

#include "ivm6303.h"

/* First supported fw abi */
#define OLDEST_FW_ABI			1

/* Base Region */
#define IVM6303_SYSTEM_CTRL		0x00
# define POWER				BIT(0)

#define IVM6303_SOFTWARE_RESET		0x01
# define RESET				BIT(0)

#define IVM6303_ENABLES_SETTINGS(n)	(0x14 + (n))
/* ENABLES_SETTINGS_1 */
# define REF_EN				BIT(0)
# define REF_LDO4V_EN			BIT(1)
/* Charge pump enable */
# define CP_EN				BIT(4)
# define IVM6303_REFS_MASK		(REF_EN|REF_LDO4V_EN|CP_EN)

# define EQ2_EN				BIT(2)
# define ENV_TRACK_EN			(BIT(4)|BIT(5))
# define BOP_EN				BIT(6)
# define VBAT_EN			BIT(7)
# define IVM6303_DSP_MASK		(ENV_TRACK_EN|BOP_EN|VBAT_EN|EQ2_EN)

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
# define CLK_MON_OK			BIT(0)

# define IVM6303_SEQUENCER_STATUS(n)	(0x1b + ((n) - 1))

#define IVM6303_MEAS_RANGE_START	0x20
#define IVM6303_VSENSE			0x21
#define IVM6303_MEAS_RANGE_END		0x2f

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

/* TDM_SETTINGS(27) */
# define TDM_MONO_MIX_SHIFT		4

#define IVM6303_VOLUME(n)		(0x61 + (n))
#define VOLUME_LSBS_MASK		0x3U

/* 0.125dB * 1000 */
#define IVM6303_VOLUME_HW_STEP		125
/* 0.25dB * 1000 */
#define IVM6303_VOLUME_CTRL_STEP	250

#define IVM6303_DEFAULT_MAX_VOLUME	752

/* ((752 (0xbc0) - 1) * 125) / 10 => 9387 */
#define IVM6303_DEFAULT_MIN_VOLUME				\
	(-(((IVM6303_DEFAULT_MAX_VOLUME - 1) * 125)/10))

/*
 * Control step is 0.25dB, which is double the actual step, so we have to
 * divide max by 2
 */
#define IVM6303_DEFAULT_MAX_CTRL_VOLUME \
	DIV_ROUND_UP(IVM6303_DEFAULT_MAX_VOLUME,2)

#define IVM6303_VOLUME_STATUS(x)	((x) + 0x6c)

#define IVM6303_PLL_SETTINGS(x)		((x) + 0x80)
#define PLL_POST_DIVIDER_MASK		0x0f
#define PLL_POST_DIVIDER_SHIFT		4
#define PLL_FEEDB_DIV_MSB_MASK		0x01
#define PLL_FEEDB_DIV_MSB_SHIFT		0
#define PLL_INPUT_DIVIDER_MASK		0x0f
#define PLL_INPUT_DIVIDER_SHIFT		4

#define IVM6303_PROTECTION_REG(x)	(0x9a + ((x) - 1))

#define IVM6303_VISENSE_SETTINGS(x)	(0xa0 + ((x) - 1))
/* IVM6303_VISENSE_SETTINGS(1) */
# define VIS_DIG_EN_V			BIT(0)
# define VIS_DIG_EN_I			BIT(1)
# define VIS_DIG_EN_MASK		(VIS_DIG_EN_V|VIS_DIG_EN_I)
# define VIS_DIG_EN_SHIFT		0

#define IVM6303_DIG_BOOST_STATUS(x)	(((x) - 1) + 0xcd)

#define IVM6303_SAR_SETTINGS(x)		(((x) - 1) + 0xd0)

#define IVM6303_CAL_SETTINGS(x)		((x - 1) + 0xe0)
/* IVM6303_CAL_SETTINGS(6) */
# define HW_OFFSET_CAL			BIT(0)

#define IVM6303_CLIPPING_RANGE_START	0xf0
#define IVM6303_CLIPPING_RANGE_END	0xf7

#define IVM6303_PAD_SETTINGS		0xfc

#define IVM6303_PAGE_SELECTION		0xfe
# define IVM6303_PAGE_MASK		0x03

#define IVM6303_HW_REV			0xff

#define IVM6303_IO_TEST_SETTINGS(x)	(((x) - 1) + 0x100)

#define IVM6303_DIG_TEST_SETTINGS(x)    \
	(((x) - 1) + (((x) <= 2) ? 0x103 : 0x106))

#define IVM6303_SEQ_SETTINGS		0x10d
# define SEQ_OTP_LOAD_DIS		BIT(0)

#define IVM6303_FORCE_INTFB		0x110

#define IVM6303_ANALOG_REG2_FORCE	0x112

#define IVM6303_ANALOG_REG3_FORCE	0x114
#define IVM6303_ANALOG_REG3		0x115
#define IVM6303_TEST_DIG1_FORCE		0x116
# define FORCE_SEQ_CAL_EN		BIT(4)

#define IVM6303_TEST_DIG1		0x117
# define SEQ_CAL_EN_M			BIT(4)

#define IVM6303_EQ_SETTINGS		0x140

#define IVM6303_EQ_APPLY		0x155

#define IVM6303_SP_RANGE_START		0x156
#define IVM6303_SP_READ_RANGE_START	IVM6303_SP_RANGE_START
#define IVM6303_SP_READ_RANGE_END	0x159
#define IVM6303_SP_RANGE_END		0x15f

#define IVM6303_DRC_RO_RANGE_START	0x177
#define IVM6303_DRC_RO_RANGE_END	0x17f

#define IVM6303_DTC_STATUS(x)		(((x) - 1) + 0x19a)

#define IVM6303_OTP_CONTROL(x)		(((x) - 1) + 0x1d0)

#define IVM6303_GAIN_000_OFFS_COMP_HI	0x1d2
/* Reg 0x1d3 is shared */
#define IVM6303_GAIN_000_OFFS_COMP_LO	0x1d3
#define IVM6303_GAIN_001_OFFS_COMP_HI	0x1d3
#define IVM6303_GAIN_001_OFFS_COMP_LO	0x1d4
#define IVM6303_GAIN_010_OFFS_COMP_HI	0x1d5
/* Reg 0x1d6 is shared */
#define IVM6303_GAIN_010_OFFS_COMP_LO	0x1d6
#define IVM6303_GAIN_011_OFFS_COMP_HI	0x1d6
#define IVM6303_GAIN_011_OFFS_COMP_LO	0x1d7
#define IVM6303_GAIN_100_OFFS_COMP_HI	0x1d8
/* Reg 0x1d9 is shared */
#define IVM6303_GAIN_100_OFFS_COMP_LO	0x1d9
#define IVM6303_GAIN_101_OFFS_COMP_HI	0x1d9
#define IVM6303_GAIN_101_OFFS_COMP_LO	0x1da

/* Otp registers, starting from 0 */
#define IVM6303_OTP(n)			((n)+0x1d2)

#define IVM6303_GAIN_OFFS_INTFB_COMP(n)	((n) + 0x1db)

#define IVM6303_TEST_DEVICE_INFO	0x1ff

#define IVM6303_LAST_REG		0x3ff

#define IVM6303_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |	\
			 SNDRV_PCM_FMTBIT_S24_LE |	\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define IVM6303_RATES (SNDRV_PCM_RATE_16000 |			\
		       SNDRV_PCM_RATE_48000 |			\
		       SNDRV_PCM_RATE_96000)

#define IVM6303_I2S_DAI 0
#define IVM6303_TDM_DAI 1

#define PLL_LOCKED_POLL_PERIOD ((HZ)/20)
#define MAX_PLL_LOCKED_POLL_ATTS (10)
#define MAX_CLK_MON_OK_ATTS (100)

#define VSIS_ON_WAIT_TIME ((HZ)/10)

enum ivm_tdm_size {
	IVM_TDM_SIZE_16 = 1,
	IVM_TDM_SIZE_20 = 0,
	IVM_TDM_SIZE_24 = 2,
	IVM_TDM_SIZE_32 = 3,
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

static int check_fw_abi(unsigned int abi)
{
	return abi >= OLDEST_FW_ABI ? 0 : -1;
}

static const char *ivm6303_fw_op_to_str(enum ivm6303_fw_op o)
{
	switch(o) {
	case IVM6303_REG_WRITE:
		return "wr";
	case IVM6303_MASK_SET:
		return "mset";
	case IVM6303_MASK_CLR:
		return "mclr";
	default:
		return "UNK";
	}
	/* NEVER REACHED */
	return "UNK";
}

/*
 * Assumes regmap mutex taken
 */
static int _do_regs_assign_seq(struct ivm6303_priv *priv,
			       int nsteps,
			       struct ivm6303_register *regs)
{
	struct device *dev = &priv->i2c_client->dev;
	struct ivm6303_register *r;
	int i, ret = 0;

	dev_dbg(dev, "%s starting sequence with %d steps\n", __func__, nsteps);
	for (i = 0, r = regs; i < nsteps; i++, r++) {
		if (i >= IVM6303_SECTION_MAX_REGISTERS) {
			dev_err(dev, "%s, too many registers\n", __func__);
			ret = -ENOMEM;
			break;
		}
		dev_dbg(dev, "%p, op = %s, addr = %3x, val = %2x\n",
			r, ivm6303_fw_op_to_str(r->op), r->addr, r->val);
		switch (r->op) {
		case IVM6303_REG_WRITE:
			ret = regmap_write(priv->regmap, r->addr, r->val);
			break;
		case IVM6303_MASK_SET:
			ret = regmap_update_bits(priv->regmap, r->addr,
						 r->val, r->val);
			break;
		case IVM6303_MASK_CLR:
			ret = regmap_update_bits(priv->regmap, r->addr,
						 r->val, ~r->val);
			break;
		}
		if (ret < 0) {
			dev_err(dev, "error: op %s, reg %u, val = %u",
				ivm6303_fw_op_to_str(r->op), r->addr, r->val);
			break;
		}
		if ((r->addr == IVM6303_SOFTWARE_RESET) &&
		    (r->val & RESET) &&
		    (r->op == IVM6303_REG_WRITE || r->op == IVM6303_MASK_SET))
			/* Doing reset, invalidate cache */
			regcache_drop_region(priv->regmap, 0, IVM6303_LAST_REG);
		if (r->delay_us) {
			if (r->delay_us < 20)
				udelay(r->delay_us);
			else if (r->delay_us <= 20000)
				usleep_range(r->delay_us, r->delay_us * 2);
			else
				msleep(r->delay_us / 1000);
		}
	}
	dev_dbg(dev, "%s: sequence done, ret = %d\n", __func__, ret);
	return ret;
}

static int _run_fw_section(struct ivm6303_priv *priv,
			   struct ivm6303_fw_section *section)
{
	return _do_regs_assign_seq(priv, section->nsteps, section->regs);
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
		dev_err(dev, "%s: section index is invalid\n", __func__);
		goto end;
	}
	dev_dbg(dev, "%s: running fw section %d", __func__, s);
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

	if (atomic_cmpxchg(&priv->running_section, IVM6303_NO_SECTION, s)) {
		struct ivm6303_fw_section *section = &priv->fw_sections[s];

		if (section->can_be_aborted)
			cancel_work_sync(&priv->fw_exec_work);
		else
			wait_for_completion(&priv->fw_section_completion);
		atomic_set(&priv->running_section, s);
	}
	reinit_completion(&priv->fw_section_completion);
	/* Start */
	return queue_work(priv->wq, &priv->fw_exec_work);
}

/* Assumes regmap lock tocken */
static int _run_fw_section_sync(struct snd_soc_component *component, int s)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct ivm6303_fw_section *section;

	if (s < 0 || s >= IVM6303_N_SECTIONS)
		return -EINVAL;
	dev_dbg(component->dev, "%s: running seq %d\n", __func__, s);
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

static int _get_vsis_en(struct ivm6303_priv *priv, unsigned int *v)
{
	struct device *dev = &priv->i2c_client->dev;
	int ret = regmap_read(priv->regmap, IVM6303_VISENSE_SETTINGS(1), v);

	if (ret < 0) {
		dev_err(dev, "error saving Vs/Is enable state\n");
		return ret;
	}
	*v &= VIS_DIG_EN_MASK;
	return ret;
}

/* Set/reset on bit(s) for Vs and/or Is */
static int _set_vsis_en(struct ivm6303_priv *priv, int mask, int v)
{
	struct device *dev = &priv->i2c_client->dev;
	int ret = regmap_update_bits(priv->regmap,
				     IVM6303_VISENSE_SETTINGS(1),
				     mask, v);
	if (ret < 0)
		dev_err(dev, "error setting Vs/Is enable bits\n");
	return ret;
}

static int _get_volume(struct ivm6303_priv *priv, unsigned int *out)
{
	struct device *dev = &priv->i2c_client->dev;
	unsigned int tmp;
	int ret = regmap_read(priv->regmap, IVM6303_VOLUME(0), out);

	if (ret)
		goto err;
	/* 8 MSBs */
	*out <<= 2U;
	ret = regmap_read(priv->regmap, IVM6303_VOLUME(1), &tmp);
	if (ret)
		goto err;
	/* 2 MSBs */
	*out |= (tmp & VOLUME_LSBS_MASK);
	return ret;

err:
	dev_err(dev, "error reading volume\n");
	return ret;
}

static int _set_volume(struct ivm6303_priv *priv, unsigned int v)
{
	struct device *dev = &priv->i2c_client->dev;
	int ret;

	if (v > priv->max_volume) {
		dev_dbg(dev, "attempt to set too high volume, saturating\n");
		v = priv->max_volume;
	}

	ret = regmap_update_bits(priv->regmap, IVM6303_VOLUME(1),
				 VOLUME_LSBS_MASK, v & VOLUME_LSBS_MASK);
	if (ret)
		goto err;
	ret = regmap_write(priv->regmap, IVM6303_VOLUME(0), v >> 2);
	if (ret)
		goto err;
	return ret;
err:
	dev_err(dev, "error setting volume\n");
	return ret;
}

static int _do_mute(struct ivm6303_priv *priv, int mute)
{
	int ret;
	struct device *dev = &priv->i2c_client->dev;

	if (mute) {
		if (test_bit(SPEAKER_ENABLED, &priv->flags)) {
			unsigned int curr_volume;

			ret = _get_volume(priv, &curr_volume);
			if (ret < 0)
				goto err;
			if (curr_volume) {
				priv->saved_volume = curr_volume;
				dev_dbg(dev, "Saved volume = %2x\n",
					priv->saved_volume);
			} else
				dev_dbg(dev, "Not saving 0 volume\n");
		}
		ret = _set_volume(priv, 0);
		dev_dbg(dev, "Set volume to 0\n");
	} else {
		dev_dbg(dev, "Setting volume to %3x\n", priv->saved_volume);
		ret = _set_volume(priv, priv->saved_volume);
		if (ret)
			dev_err(dev, "Error restoring saved volume\n");
	}
err:
	return ret;
}

static inline int needs_autocal(struct ivm6303_priv *priv)
{
	return priv->quirks->needs_autocal;
}

static inline int has_working_hw_autocal(struct ivm6303_priv *priv)
{
	return priv->quirks->has_working_hw_autocal;
}

static int _do_hw_autocal(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	struct ivm6303_fw_section *section =
		&priv->fw_sections[IVM6303_HW_AUTOCAL];
	int ret;

	dev_dbg(dev, "running autocal section\n");
	ret = _run_fw_section(priv, section);
	if (ret)
		dev_err(dev, "error running hw autocal fw section\n");
	return ret;
}

static void _set_speaker_enable(struct ivm6303_priv *priv, int en)
{
	struct ivm6303_fw_section *section = en ?
		&priv->fw_sections[IVM6303_STREAM_START]:
		&priv->fw_sections[IVM6303_STREAM_STOP];
	struct device *dev = &priv->i2c_client->dev;
	int stat;

	if (!en) {
		_do_mute(priv, 1);
		msleep(100);
	}

	if (en && !priv->autocal_done && needs_autocal(priv)) {
		if (!has_working_hw_autocal(priv)) {
			dev_err(dev, "AUTOCAL NEEDED, BUT NO HW AUTOCAL IS AVAILABLE AND DRIVER DOES NOT SUPPORT SW AUTOCAL ANYMORE\n");
		} else {
			dev_dbg(dev, "START HW AUTOCAL\n");
			stat = _do_hw_autocal(priv);
			if (!stat) {
				dev_dbg(dev, "HW AUTOCAL DONE\n");
				priv->autocal_done = 1;
			}
		}
	}

	dev_dbg(dev, "running section %s\n",
		en ? "stream_start" : "stream_stop");
	_run_fw_section(priv, section);

	if (en)
		_do_mute(priv, priv->muted);
}

static int playback_mode_control_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_dapm_kcontrol_component(kcontrol);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(c);
	unsigned int v;

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
	int ret, vsis_error;
	unsigned int v;

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
	if (test_bit(SPEAKER_ENABLED, &priv->flags)) {
		/* Save Vs/Is enable status, the off sequence could touch it */
		vsis_error = _get_vsis_en(priv, &v);
		if (vsis_error)
			dev_err(c->dev, "error saving vs/is enable state\n");
		_set_speaker_enable(priv, 0);
	}
	dev_dbg(c->dev, "%s: running section %d\n", __func__,
		priv->playback_mode_fw_section);
	ret = _run_fw_section_sync(c, priv->playback_mode_fw_section);
	if (test_bit(SPEAKER_ENABLED, &priv->flags)) {
		_set_speaker_enable(priv, 1);
		if (!vsis_error) {
			/* Restore Vs/Is enable status */
			ret = _set_vsis_en(priv, VIS_DIG_EN_MASK, v);
			if (ret)
				dev_err(c->dev,
					"error restoring vs/is enable state\n");
		}
	}
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

static const struct snd_kcontrol_new playback_mode_control[] = {
	SOC_SINGLE_EXT("Speaker mode",
		       0, 0, 1, 0, playback_mode_control_get,
		       playback_mode_control_put),
};

static int _set_dsp_enable(struct ivm6303_priv *priv, int en)
{
	return regmap_update_bits(priv->regmap,
				  IVM6303_ENABLES_SETTINGS(2),
				  IVM6303_DSP_MASK,
				  en ? IVM6303_DSP_MASK : 0);
}

static int start_pll_polling(struct ivm6303_priv *priv)
{
	int olds;

	olds = atomic_cmpxchg(&priv->clk_status, STOPPED, WAITING_FOR_PLL_LOCK);
	if (olds == WAITING_FOR_PLL_LOCK)
		/* Already waiting, do nothing */
		return 0;
	priv->pll_locked_poll_attempts = 0;
	return queue_delayed_work(priv->wq, &priv->pll_locked_work,
				  PLL_LOCKED_POLL_PERIOD);
}

static int playback_mode_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *c, int e)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;

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
	case SND_SOC_DAPM_POST_PMU:
		if (!priv->capture_only) {
			clear_bit(WAITING_FOR_SPEAKER_OFF, &priv->flags);
			set_bit(WAITING_FOR_SPEAKER_ON, &priv->flags);
		}
		if (priv->i2c_client->irq <= 0)
			ret = start_pll_polling(priv);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (!test_and_set_bit(WAITING_FOR_SPEAKER_OFF, &priv->flags))
			ret = queue_work(priv->wq,
					 &priv->speaker_deferred_work);
		atomic_set(&priv->clk_status, STOPPED);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = _set_dsp_enable(priv, 0);
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

static void vsis_enable_handler(struct work_struct * work)
{
	struct ivm6303_priv *priv = container_of(work, struct ivm6303_priv,
						 vsis_enable_work.work);
	struct device *dev = &priv->i2c_client->dev;
	int stat;

	if (!test_and_clear_bit(WAITING_FOR_VSIS_ON, &priv->flags))
		return;
	mutex_lock(&priv->regmap_mutex);
	/*
	 * Only enable Vs here, Is is needed (and actually works) only
	 * when playback is acive. The IS_DIG_EN_I bit is set when enabling
	 * the speaker
	 */
	stat = _set_vsis_en(priv, VIS_DIG_EN_V, VIS_DIG_EN_V);
	mutex_unlock(&priv->regmap_mutex);
	if (stat)
		dev_err(dev, "Error enabling Vs/Is");
}

static int adc_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *c,
		     int e)
{
	struct snd_soc_component *component =
		snd_soc_dapm_to_component(w->dapm);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0, on = 0, deferred;

	switch(e) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable Vs/Is */
		on = 1;
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (priv->i2c_client->irq <= 0)
			ret = start_pll_polling(priv);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		atomic_set(&priv->clk_status, STOPPED);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Disable Vs/Is */
		on = 0;
		break;
	default:
		dev_err(component->dev, "%s: unexpected event %d\n",
			__func__, e);
		ret = -EINVAL;
	}
	if (ret || (e == SND_SOC_DAPM_POST_PMU) || (e == SND_SOC_DAPM_PRE_PMD))
		return ret;
	/*
	 * If we're capture only, then we can avoid deferring Vs/Is enable
	 * Also, if speaker has already been turned on, we can enable Vs/Is
	 * immediately
	 */
	deferred = !priv->capture_only &&
		!test_bit(SPEAKER_ENABLED, &priv->flags);
	if (!on || !deferred) {
		clear_bit(WAITING_FOR_VSIS_ON, &priv->flags);
		mutex_lock(&priv->regmap_mutex);
		ret = _set_vsis_en(priv, VIS_DIG_EN_MASK, on ?
				   VIS_DIG_EN_MASK : 0);
		mutex_unlock(&priv->regmap_mutex);
	} else {
		/*
		 * Avoid turning Vs/Is on immediately if playback is possible.
		 * The reason for this is that turning the speaker on with
		 * Vs/Is enabled can trigger a pop. So wait and see if a
		 * playback stream arrives. If it doesn't within 100ms, enable
		 * Vs/Is. This is really UGLY !
		 */
		set_bit(WAITING_FOR_VSIS_ON, &priv->flags);
		queue_delayed_work(priv->wq, &priv->vsis_enable_work,
				   VSIS_ON_WAIT_TIME);
	}
	return ret;
}

static const char *tdm_mono_mix_ch1_texts[] = {
	"0 dB",
	"-6 dB",
	"-12 dB",
	"-18 dB",
};

static unsigned int tdm_mono_mix_ch1_values[] = {
	0, 1, 2, 3,
};

static const char *tdm_mono_mix_ch2_texts[] = {
	"0 dB",
	"-6 dB",
	"-12 dB",
	"-18 dB",
	"OFF"
};

static unsigned int tdm_mono_mix_ch2_values[] = {
	4, 5, 6, 7, 0,
};

static SOC_VALUE_ENUM_SINGLE_DECL(tdm_mono_mix_ch1_enum,
				  IVM6303_TDM_SETTINGS(28),
				  /* shift */
				  0,
				  /* mask */
				  0x3,
				  tdm_mono_mix_ch1_texts,
				  tdm_mono_mix_ch1_values);

static SOC_VALUE_ENUM_SINGLE_DECL(tdm_mono_mix_ch2_enum,
				  IVM6303_TDM_SETTINGS(28),
				  /* shift */
				  2,
				  /* mask */
				  0x7,
				  tdm_mono_mix_ch2_texts,
				  tdm_mono_mix_ch2_values);

static const struct snd_kcontrol_new tdm_mono_mix_controls[] = {
	SOC_DAPM_ENUM("tdm mono mix ch1 gain", tdm_mono_mix_ch1_enum),
	SOC_DAPM_ENUM("tdm mono mix ch2 gain", tdm_mono_mix_ch2_enum),
};

static const struct snd_soc_dapm_widget ivm6303_dapm_widgets[] = {
	/* Analog Output */
	SND_SOC_DAPM_OUTPUT("SPK"),
	/* Mono mixer (digital in -> class D stereo to mono) */
	SND_SOC_DAPM_MIXER("MONOMIX", SND_SOC_NOPM, 0, 0,
			   tdm_mono_mix_controls,
			   ARRAY_SIZE(tdm_mono_mix_controls)),
	/* TDM INPUT(s) (Playback) */
	SND_SOC_DAPM_AIF_IN("AIF TDM IN CH1", "TDM Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF TDM IN CH2", "TDM Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	/* I2S INPUT (Playback) */
	SND_SOC_DAPM_AIF_IN("AIF I2S IN", "I2S Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA_E("CLASS-D", SND_SOC_NOPM, 0, 0,
			   playback_mode_control, 1,
			   playback_mode_event,
			   SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMU|
			   SND_SOC_DAPM_PRE_PMD|SND_SOC_DAPM_POST_PMD),
	/* Analog input */
	SND_SOC_DAPM_INPUT("VSIS-IN"),
	SND_SOC_DAPM_ADC_E("VSIS-ADC", NULL, SND_SOC_NOPM, 0, 0,
			   adc_event,
			   SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMU|
			   SND_SOC_DAPM_PRE_PMD|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("AIF CH1-2 I2S OUT", "I2S Capture", 0,
			     SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF CH1-2 TDM OUT", "TDM Capture", 0,
			     SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF CH3-4 TDM OUT", "TDM Capture", 0,
			     SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route ivm6303_dapm_routes[] = {
	/* sink | control | source */
	{"MONOMIX", NULL, "AIF TDM IN CH1"},
	{"MONOMIX", NULL, "AIF TDM IN CH2"},
	{"CLASS-D", NULL, "MONOMIX"},
	{"CLASS-D", NULL, "AIF I2S IN"},
	{"SPK", NULL, "CLASS-D"},
	{"VSIS-ADC", NULL, "VSIS-IN"},
	{"VSIS-ADC", NULL, "VSIS-IN"},
	{"AIF CH1-2 I2S OUT", NULL, "VSIS-ADC"},
	{"AIF CH1-2 TDM OUT", NULL, "VSIS-ADC"},
	{"AIF CH3-4 TDM OUT", NULL, "VSIS-ADC"},
};

#define REV_BASE 0xf9

#define REV_OFFSET(r) ((r) - REV_BASE)

static const struct ivm6303_quirks quirks[] = {
	[REV_OFFSET(0xf9)] = {
		.needs_autocal = 1,
		.has_working_hw_autocal = 0,
	},
	[REV_OFFSET(0xfa)] = {
		.needs_autocal = 1,
		.has_working_hw_autocal = 0,
	},
	[REV_OFFSET(0xfc)] = {
		.needs_autocal = 1,
		.has_working_hw_autocal = 1,
	},
};

#define QUIRKS(a) &quirks[REV_OFFSET(a)]

/* Assumes regmap lock taken */
static inline int _do_reset(struct ivm6303_priv *priv)
{
	int ret;
	struct device *dev = &priv->i2c_client->dev;

	ret = regmap_update_bits(priv->regmap, IVM6303_SOFTWARE_RESET, RESET,
				 RESET);
	if (ret) {
		dev_err(dev, "Error resetting device\n");
		return ret;
	}
	regcache_drop_region(priv->regmap, 0, IVM6303_LAST_REG);
	return ret;
}

/* Assumes regmap lock taken */
static inline int _do_power(struct ivm6303_priv *priv, int up)
{
	dev_dbg(&priv->i2c_client->dev, "POWER -> %d\n", up);
	return regmap_update_bits(priv->regmap, IVM6303_SYSTEM_CTRL, POWER,
				  up ? POWER : 0);
}

static inline int _do_power_up(struct ivm6303_priv *priv)
{
	return _do_power(priv, 1);
}

static inline int _do_power_down(struct ivm6303_priv *priv)
{
	return _do_power(priv, 0);
}

static int _resync_power_state(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int on;

	on = snd_soc_component_get_bias_level(component) != SND_SOC_BIAS_OFF;
	return _do_power(priv, on);
}

/* Assumes regmap mutex taken */
static int _set_pll_enable_check(struct ivm6303_priv *priv, int en,
				 bool *changed)
{
	return regmap_update_bits_check(priv->regmap,
					IVM6303_ENABLES_SETTINGS(1),
					PLL_EN|PLL_CLKMUX_EN,
					en ? PLL_EN|PLL_CLKMUX_EN : 0,
					changed);
}

static int _set_pll_enable(struct ivm6303_priv *priv, int en)
{
	return regmap_update_bits(priv->regmap, IVM6303_ENABLES_SETTINGS(1),
				  PLL_EN|PLL_CLKMUX_EN,
				  en ? PLL_EN|PLL_CLKMUX_EN : 0);
}

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
	case 0xfc:
		priv->hw_rev = rev;
		priv->quirks = QUIRKS(rev);
		dev_dbg(component->dev, "needs_autocal = %d\n",
			priv->quirks->needs_autocal);
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

static int cope_with_untrimmed(struct snd_soc_component *component)
{
	int ret, i;
	unsigned int v = 0;
	/* Check registers: otp registers 15 and 16 (0x1e1, 0x1e2) */
	unsigned int otp_check_regs[] = { IVM6303_OTP(15), IVM6303_OTP(16) };
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	mutex_lock(&priv->regmap_mutex);

	ret = _do_reset(priv);
	if (ret < 0)
		goto err;

	ret = _do_power_up(priv);
	if (ret < 0)
		goto err;

	/* If all registers == 0xff -> part is untrimmed */
	for (i = 0; i < ARRAY_SIZE(otp_check_regs) && v != 0xff && !ret; i++)
		ret = regmap_read(priv->regmap, otp_check_regs[i], &v);

	if (ret < 0)
		goto err;
	if (v != 0xff) {
		/* Part is trimmed */
		ret = _do_power_down(priv);
		if (ret)
			dev_err(component->dev, "error powering down\n");
		mutex_unlock(&priv->regmap_mutex);
		priv->untrimmed = 0;
		return ret;
	}

	/* Part is untrimmed */
	dev_info(component->dev, "part is not trimmed\n");
	ret = _do_reset(priv);
	if (ret)
		goto err;
	/* Do not load otp registers */
	ret = regmap_update_bits(priv->regmap, IVM6303_SEQ_SETTINGS,
				 SEQ_OTP_LOAD_DIS, SEQ_OTP_LOAD_DIS);
	if (ret)
		goto err;
	ret = _do_power_up(priv);
	if (ret)
		goto err;
	ret = _run_fw_section(priv,
			      &priv->fw_sections[IVM6303_TRIMMING_DEFAULTS]);
	if (ret < 0)
		goto err;
	priv->untrimmed = 1;
	ret = _do_power_down(priv);
	if (ret)
		dev_err(component->dev, "error powering down\n");
	mutex_unlock(&priv->regmap_mutex);
	return ret;
err:
	/* We can ignore further errors here */
	_do_power_down(priv);
	mutex_unlock(&priv->regmap_mutex);
	dev_err(component->dev, "cope_with_untrimmed() error\n");
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

static inline int is_fwabi(u16 w)
{
	return (w & 0xf000) == 0x5000;
}

static inline int is_mask_set_addr(u16 w)
{
	return (w & 0xf000) == 0x6000;
}

static inline int is_mask_clr_addr(u16 w)
{
	return (w & 0xf000) == 0x7000;
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

static inline unsigned int to_fwabi(u16 w)
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
	unsigned int fw_abi;
	u16 *w;
	unsigned int addr = ADDR_INVALID, val = VAL_INVALID, pg = 0,
		section = IVM6303_PROBE_WRITES, reg_index = 0, delay_us = 0,
		op = WRITE;
	static char fw_file_name[MAX_FW_FILENAME_LEN];

	/*
	 * Firmware is searched first with hw rev and i2c address in its
	 * name (for loading different configuration for different devices,
	 * and then with hw revision only (same firmware for all installed
	 * devices)
	 */
	i = 0;
	do {
		switch(i++) {
		case 0:
			/*
			 * First attempt: try loading fw file specific for
			 * this device address and hw revision
			 */
			snprintf(fw_file_name, sizeof(fw_file_name),
				 "ivm6303-param-%2x-%2x.bin",
				 priv->hw_rev, priv->i2c_client->addr);
			break;
		case 1:
			/* Second attempt: only look at hw revision */
			snprintf(fw_file_name, sizeof(fw_file_name),
				 "ivm6303-param-%2x.bin", priv->hw_rev);
			break;
		default:
			/* UNREACHABLE */
			ret = -EINVAL;
			break;
		}
		ret = request_firmware(&priv->fw, fw_file_name, component->dev);
	} while(ret < 0 && i < 2);
	if (ret < 0) {
		dev_err(component->dev, "cannot load firmware");
		return ret;
	}
	fw = priv->fw;
	ret = alloc_fw_section(component, IVM6303_PROBE_WRITES);
	if (ret < 0)
		return ret;
	dev_dbg(component->dev, "firmware size = %ld\n", fw->size);
	for (w = (u16 *)fw->data, i =0, eof_record = -1, fw_abi = 0;
	     i < (fw->size / 2) && eof_record < 0; w++, i++) {
		if (is_fwabi(*w)) {
			fw_abi = to_fwabi(*w);
			dev_info(component->dev,
				 "found firmware abi 0x%08x\n", fw_abi);
		}
		if (is_file_end(*w)) {
			eof_record = i;
			break;
		}
		if (is_mdelay(*w))
			delay_us += to_mdelay(*w);
		if (is_udelay(*w))
			delay_us += to_udelay(*w);
		if (is_mask_set_addr(*w)) {
			addr = to_addr(*w);
			op = IVM6303_MASK_SET;
		}
		if (is_mask_clr_addr(*w)) {
			addr = to_addr(*w);
			op = IVM6303_MASK_CLR;
		}
		if (is_addr(*w)) {
			addr = to_addr(*w);
			op = IVM6303_REG_WRITE;
		}
		if (is_val(*w))
			val = to_val(*w);
		if (is_new_section(*w)) {
			section = to_val(*w);
			dev_dbg(component->dev, "new section %d\n", section);
			if (section > IVM6303_PROBE_WRITES) {
				ret = alloc_fw_section(component, section);
				if (ret < 0)
					return ret;
				/* Start new section from scratch */
				addr = ADDR_INVALID;
				val = VAL_INVALID;
				op = IVM6303_REG_WRITE;
				/*
				 * Sections always assume we start from page 0
				 */
				pg = 0;
				delay_us = 0;
				reg_index = 0;
			}
		}
		if (addr != ADDR_INVALID && val != VAL_INVALID) {
			struct ivm6303_register *r;

			if (addr == 254) {
				pg = (val & IVM6303_PAGE_MASK) << 8;
				addr = ADDR_INVALID;
				val = VAL_INVALID;
				continue;
			}

			r = &priv->fw_sections[section].regs[reg_index++];
			r->addr = addr | pg;
			r->val = val;
			r->delay_us = delay_us;
			r->op = op;
			dev_dbg(component->dev,
				"new op: %p, %s, addr = %2x, val = %3x\n", r,
				ivm6303_fw_op_to_str(r->op), r->addr, r->val);
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
	if (check_fw_abi(fw_abi)) {
		dev_err(component->dev,
			"firmware ABI 0x%08x is incompatible\n", fw_abi);
		return -EINVAL;
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

static void _post_tdm_apply_hack(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	unsigned int v;
	int stat;

	stat = regmap_read(priv->regmap, IVM6303_TDM_SETTINGS(1), &v);
	if (stat < 0)
		dev_err(dev, "error reading tdm settings 1\n");
	stat = regmap_write(priv->regmap, IVM6303_TDM_SETTINGS(1), 0x70);
	if (stat < 0)
		dev_err(dev, "error writing dummy tdm settings 1\n");
	usleep_range(1000, 5000);
	stat = regmap_write(priv->regmap, IVM6303_TDM_SETTINGS(1), v);
	if (stat < 0)
		dev_err(dev, "error writing final tdm settings 1\n");
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
	usleep_range(1000, 5000);
	_post_tdm_apply_hack(priv);
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
	int stat;
	struct device *dev = &priv->i2c_client->dev;
	unsigned int vsis_enabled;

	stat = _get_vsis_en(priv, &vsis_enabled);

	if (stat)
		dev_err(dev, "Error reading VsIs enable bits\n");

	_set_speaker_enable(priv, 1);
	if ((!stat && vsis_enabled) ||
	    test_and_clear_bit(WAITING_FOR_VSIS_ON, &priv->flags)) {
		stat = _set_vsis_en(priv, VIS_DIG_EN_MASK, VIS_DIG_EN_MASK);
		if (stat < 0)
			dev_err(dev, "Error writing VsIs enable bits\n");
	}
	set_bit(SPEAKER_ENABLED, &priv->flags);
}

static void _turn_speaker_off(struct ivm6303_priv *priv)
{
	int stat;
	unsigned int v;
	struct device *dev = &priv->i2c_client->dev;

	/*
	 * Save current state of vs/is enable bits, the speaker off sequence
	 * can touch them !
	 */
	stat = _get_vsis_en(priv, &v);
	if (stat)
		dev_err(dev, "error saving vs/is enable status\n");
	_set_speaker_enable(priv, 0);
	clear_bit(SPEAKER_ENABLED, &priv->flags);
	if (stat)
		return;
	/*
	 * Re-enable Vs only, if it was enabled beforehand.
	 * Is does not work when the speaker is off
	 */
	stat = _set_vsis_en(priv, VIS_DIG_EN_MASK, v & VIS_DIG_EN_V);
	if (stat)
		dev_err(dev, "error restoring is enable status\n");
}

/* Assumes regmap mutex taken */
static void _pll_locked_handler(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	unsigned int status;
	int olds, stat;

	regmap_read(priv->regmap, IVM6303_STATUS(1), &status);
	if (!(status & CLK_MON_OK)) {
		olds = atomic_cmpxchg(&priv->clk_status, WAITING_FOR_PLL_LOCK,
				      WAITING_FOR_CLKMON_OK);
		switch(olds) {
		case WAITING_FOR_PLL_LOCK:
			priv->clkmon_ok_attempts = 0;
			break;
		case WAITING_FOR_CLKMON_OK:
			if (priv->clkmon_ok_attempts++ >= MAX_CLK_MON_OK_ATTS) {
				dev_err(dev, "clk mon ok timeout\n");
				atomic_set(&priv->clk_status, ERROR);
				return;
			}
			break;
		case STOPPED:
			/*
			 * We got a stop while waiting for pll locked
			 * Just avoid rescheduling another poll
			 */
			return;
		default:
			dev_err(dev, "%s: unexpected clock state %d",
				__func__, olds);
			return;
		}
		/* Wait a little bit more */
		queue_delayed_work(priv->wq, &priv->pll_locked_work,
				   PLL_LOCKED_POLL_PERIOD);
		return;
	}
	/* PLL locked and clkmon OK, clock is running (BCLK + FSYN) */
	atomic_set(&priv->clk_status, RUNNING);
	if (priv->tdm_apply_needed) {
		_do_tdm_apply(priv);
		priv->tdm_apply_needed = 0;
	}
	_set_references_enable(priv, 1);
	if (test_and_clear_bit(WAITING_FOR_SPEAKER_ON, &priv->flags)) {
		stat = _set_dsp_enable(priv, 1);
		if (stat)
			dev_err(dev, "%s: error turning dsp on\n", __func__);
		_turn_speaker_on(priv);

	}
}

static void pll_locked_handler(struct work_struct * work)
{
	struct ivm6303_priv *priv = container_of(work, struct ivm6303_priv,
						 pll_locked_work.work);
	struct device *dev = &priv->i2c_client->dev;

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

static int _tdm_enabled(struct ivm6303_priv *priv)
{
	struct device *dev = &priv->i2c_client->dev;
	int stat, ret;
	unsigned int v;

	stat = regmap_read(priv->regmap, IVM6303_ENABLES_SETTINGS(2), &v);
	if (stat < 0) {
		dev_err(dev, "error checking for tdm enabled\n");
		/* Say it's enabled anyway */
		return 1;
	}
	ret = v & TDM_EN;
	return ret;
}

/* Assumes regmap mutex taken */
static void _try_tdm_apply(struct ivm6303_priv *priv)
{
	unsigned int clk_status;

	clk_status = atomic_read(&priv->clk_status);
	priv->tdm_apply_needed = (clk_status != RUNNING) || !_tdm_enabled(priv);

	if (priv->tdm_apply_needed)
		/* CLK_MON not ok or TDM not enabled */
		return;
	_do_tdm_apply(priv);
}

static void speaker_deferred_handler(struct work_struct * work)
{
	struct ivm6303_priv *priv = container_of(work, struct ivm6303_priv,
						 speaker_deferred_work);

	if (!test_and_clear_bit(WAITING_FOR_SPEAKER_OFF, &priv->flags))
		return;
	mutex_lock(&priv->regmap_mutex);
	_turn_speaker_off(priv);
	mutex_unlock(&priv->regmap_mutex);
}

static unsigned int ivm6303_component_read(struct snd_soc_component *component,
					   unsigned int reg)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int stat;
	unsigned int v = 0;

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

	switch(level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (prev_level == SND_SOC_BIAS_OFF) {
			/* Power up first */
			ret = _do_power(priv, 1);
			if (ret)
				dev_err(dev,
					"%s: error powering up\n", __func__);
			run_fw_section(component, IVM6303_BIAS_OFF_TO_STANDBY);
			/* Enable tdm */
			ret = set_tdm_enable(priv, 1);
			if (ret < 0)
				dev_err(component->dev, "Error enabling tdm\n");
			/* Enable pll */
			ret = _set_pll_enable(priv, 1);
			if (ret  < 0)
				dev_err(component->dev, "Error enabling pll\n");
		}
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
		if (prev_level == SND_SOC_BIAS_STANDBY) {
			run_fw_section(component, IVM6303_BIAS_STANDBY_TO_OFF);
			/* Disable pll when going to SND_SOC_BIAS_OFF */
			ret = _set_pll_enable(priv, 0);
			if (ret  < 0)
				dev_err(component->dev,
					"Error disabling pll\n");
			/* And turn power off power off */
			ret = _do_power(priv, 0);
			if (ret)
				dev_err(component->dev,
					"Error powering down\n");
		}
		break;
	}
	return ret;
}


/* Read only calibration values, format is fixed point 28 fractional bits */

static ssize_t _cal_show(struct device *dev,
			 unsigned int r,
			 char *buf,
			 unsigned int untrimmed_value)
{
	struct ivm6303_priv *priv = dev_get_drvdata(dev);
	int stat;
	unsigned int v;

	if (priv->untrimmed) {
		v = untrimmed_value;
		goto end;
	}
	stat = regmap_read(priv->regmap, r, &v);
	if (stat) {
		dev_err(dev, "Error reading register 0x%02x\n", r);
		return stat;
	}
end:
	return snprintf(buf, PAGE_SIZE, "0x%02x", v);
}

static ssize_t is_g1_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return _cal_show(dev, IVM6303_CAL_SETTINGS(9), buf, 0);
}

static ssize_t is_g2_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return _cal_show(dev, IVM6303_CAL_SETTINGS(10), buf, 128);
}

static ssize_t vs_g_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return _cal_show(dev, IVM6303_CAL_SETTINGS(11), buf, 128);
}

static DEVICE_ATTR_RO(is_g1);
static DEVICE_ATTR_RO(is_g2);
static DEVICE_ATTR_RO(vs_g);

static void _add_calibration_attrs(struct snd_soc_component *component)
{
	device_create_file(component->dev, &dev_attr_is_g1);
	device_create_file(component->dev, &dev_attr_is_g2);
	device_create_file(component->dev, &dev_attr_vs_g);
}

static void _remove_calibration_attrs(struct snd_soc_component *component)
{
	device_remove_file(component->dev, &dev_attr_vs_g);
	device_remove_file(component->dev, &dev_attr_is_g2);
	device_remove_file(component->dev, &dev_attr_is_g1);
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
		ret = -ENOMEM;
		goto err;
	}
	INIT_DELAYED_WORK(&priv->pll_locked_work, pll_locked_handler);
	INIT_WORK(&priv->fw_exec_work, fw_exec_handler);
	INIT_WORK(&priv->speaker_deferred_work, speaker_deferred_handler);
	INIT_DELAYED_WORK(&priv->vsis_enable_work, vsis_enable_handler);
	ret = cope_with_untrimmed(component);
	if (ret < 0)
		return ret;
	dev_dbg(component->dev, "%s: running probe section\n", __func__);
	ret = run_fw_section_sync(component, IVM6303_PROBE_WRITES);
	if (ret < 0)
		goto err;
	ivm6303_init_debugfs(component);
	mutex_lock(&priv->regmap_mutex);
	/* Power up */
	ret = _do_power_up(priv);
	/* Initialize volume */
	ret = _get_volume(priv, &priv->saved_volume);
	if (ret) {
		dev_err(component->dev, "error reading initial volume\n");
		mutex_unlock(&priv->regmap_mutex);
		goto err;
	}
	/* Volume initially set by firmware is the max allowed */
	priv->max_volume = priv->saved_volume;
	_add_calibration_attrs(component);
	/* Switch off power */
	ret = _resync_power_state(component);
	if (ret)
		dev_err(component->dev, "error syncing power state\n");
	mutex_unlock(&priv->regmap_mutex);
	return ret;

err:
	unload_fw(component);
	return ret;
}

static void ivm6303_component_remove(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	ivm6303_cleanup_debugfs(component);
	cancel_delayed_work_sync(&priv->pll_locked_work);
	cancel_delayed_work_sync(&priv->vsis_enable_work);
	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);
	_remove_calibration_attrs(component);
	priv->wq = NULL;
	priv->flags = priv->muted = priv->capture_only = priv->autocal_done = 0;
	priv->tdm_settings_1 = -1;
	atomic_set(&priv->clk_status, 0);
	unload_fw(component);
	regcache_drop_region(priv->regmap, 0, IVM6303_LAST_REG);
	_do_power_down(priv);
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

/*
 * Max/min volume depend on values set by firmware, so this has to be
 * reinitialized on probe
 */
static DECLARE_TLV_DB_SCALE(vol_scale, IVM6303_DEFAULT_MIN_VOLUME,
			    IVM6303_VOLUME_CTRL_STEP/10, 0);

static int ivm6303_info_volsw(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct device *dev = &priv->i2c_client->dev;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	/* 0 is mute */
	uinfo->value.integer.min = 1;
	/* Max value has to be divided by 2 because control step is 0.25 */
	uinfo->value.integer.max = DIV_ROUND_UP(priv->max_volume, 2);
	dev_dbg(dev, "%s, min = %ld, max = %ld\n", __func__,
		uinfo->value.integer.min, uinfo->value.integer.max);
	return 0;
}

static int ivm6303_get_volsw(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct device *dev = &priv->i2c_client->dev;
	int v = DIV_ROUND_UP(priv->saved_volume, 2);

	mutex_lock(&priv->regmap_mutex);
	v = (v >= 1) ? v : 1;
	dev_dbg(dev, "%s, vol = %d\n", __func__, v);
	ucontrol->value.integer.value[0] = v;
	mutex_unlock(&priv->regmap_mutex);
	return 0;
}

static int ivm6303_put_volsw(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	struct device *dev = &priv->i2c_client->dev;
	int ret, v = ucontrol->value.integer.value[0];

	mutex_lock(&priv->regmap_mutex);

	/* Convert to actual device value (step 0.125dB) */
	v *= 2;
	dev_dbg(dev, "%s, setting vol = %d\n", __func__, v);
	if (v < 1)
		v = 1;
	if (v > priv->max_volume)
		v = priv->max_volume;
	priv->saved_volume = v;
	if (test_bit(SPEAKER_ENABLED, &priv->flags)) {
		ret = _set_volume(priv, priv->saved_volume);
		if (ret)
			dev_err(dev, "error writing volume\n");
	}
	mutex_unlock(&priv->regmap_mutex);
	return ret;
}

#define VOL_ACCESS \
	SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE

static struct snd_kcontrol_new ivm6303_ctrls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Playback Volume Volume",
		.info = ivm6303_info_volsw,
		.access = VOL_ACCESS,
		.tlv.p = vol_scale,
		.get = ivm6303_get_volsw,
		.put = ivm6303_put_volsw,
	},
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
	SOC_ENUM("monomix ch1 gain", tdm_mono_mix_ch1_enum),
	SOC_ENUM("monomix ch2 gain", tdm_mono_mix_ch2_enum),
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
		/* From 0x100 to 0x3ff: undocumented test registers */
		.range_max = IVM6303_LAST_REG,
		.selector_reg = IVM6303_PAGE_SELECTION,
		.selector_mask = IVM6303_PAGE_MASK,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 256,
	}
};

static bool ivm6303_readable_register(struct device *dev, unsigned int reg)
{
	return (reg <= IVM6303_LAST_REG);
}

static bool ivm6303_writeable_register(struct device *dev,
				       unsigned int reg)
{
	if (reg >= IVM6303_MEAS_RANGE_START &&
	    reg <= IVM6303_MEAS_RANGE_END)
		return false;

	if (reg >= IVM6303_SP_READ_RANGE_START &&
	    reg <= IVM6303_SP_READ_RANGE_END)
		return false;

	if (reg >= IVM6303_DRC_RO_RANGE_START &&
	    reg <= IVM6303_DRC_RO_RANGE_END)
		return false;

	if (reg >= IVM6303_DTC_STATUS(1) &&
	    reg <= IVM6303_DTC_STATUS(6))
		return false;

	switch(reg) {
	case IVM6303_STATUS(1):
	case IVM6303_STATUS(2):
	case IVM6303_STATUS(3):
	case IVM6303_STATUS(4):
	case IVM6303_SEQUENCER_STATUS(1):
	case IVM6303_SEQUENCER_STATUS(2):
	case IVM6303_SEQUENCER_STATUS(3):
	case IVM6303_SEQUENCER_STATUS(4):
	case IVM6303_VOLUME_STATUS(0):
	case IVM6303_VOLUME_STATUS(1):
	case IVM6303_VOLUME_STATUS(2):
	case IVM6303_VOLUME_STATUS(3):
	case IVM6303_DIG_BOOST_STATUS(1):
	case IVM6303_DIG_BOOST_STATUS(2):
	case IVM6303_DIG_BOOST_STATUS(3):
	case IVM6303_HW_REV:
	case IVM6303_IO_TEST_SETTINGS(3):
	case IVM6303_TEST_DEVICE_INFO:
		return false;

	}
	return (reg <= IVM6303_LAST_REG);
}

static bool ivm6303_volatile_register(struct device *dev, unsigned int reg)
{
	if (reg >= IVM6303_MEAS_RANGE_START &&
	    reg <= IVM6303_MEAS_RANGE_END)
		return true;

	if (reg >= IVM6303_CLIPPING_RANGE_START &&
	    reg <= IVM6303_CLIPPING_RANGE_END)
		return true;

	if (reg >= IVM6303_SP_RANGE_START &&
	    reg <= IVM6303_SP_RANGE_END)
		return true;

	if (reg >= IVM6303_DRC_RO_RANGE_START &&
	    reg <= IVM6303_DRC_RO_RANGE_END)
		return true;

	if (reg >= IVM6303_DTC_STATUS(1) &&
	    reg <= IVM6303_DTC_STATUS(6))
		return true;

	switch(reg) {
	case IVM6303_IRQ_STATUS(1):
	case IVM6303_IRQ_STATUS(2):
	case IVM6303_IRQ_STATUS(3):
	case IVM6303_STATUS(1):
	case IVM6303_STATUS(2):
	case IVM6303_STATUS(3):
	case IVM6303_STATUS(4):
	case IVM6303_ENABLES_SETTINGS(5):
	case IVM6303_SEQUENCER_STATUS(1):
	case IVM6303_SEQUENCER_STATUS(2):
	case IVM6303_SEQUENCER_STATUS(3):
	case IVM6303_SEQUENCER_STATUS(4):
	case IVM6303_TDM_APPLY_CONF:
	case IVM6303_VOLUME_STATUS(0):
	case IVM6303_VOLUME_STATUS(1):
	case IVM6303_VOLUME_STATUS(2):
	case IVM6303_VOLUME_STATUS(3):
	case IVM6303_PROTECTION_REG(4):
	case IVM6303_DIG_BOOST_STATUS(1):
	case IVM6303_DIG_BOOST_STATUS(2):
	case IVM6303_DIG_BOOST_STATUS(3):
	case IVM6303_SAR_SETTINGS(11):
	case IVM6303_CAL_SETTINGS(6):
	case IVM6303_PAD_SETTINGS:
	case IVM6303_IO_TEST_SETTINGS(3):
	case IVM6303_DIG_TEST_SETTINGS(6):
	case IVM6303_EQ_SETTINGS:
	case IVM6303_EQ_APPLY:
	case IVM6303_OTP_CONTROL(1):
		return true;
	default:
		break;
	}
	return false;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.ranges = ivm6303_range_cfg,
	.num_ranges = ARRAY_SIZE(ivm6303_range_cfg),
	.max_register = IVM6303_LAST_REG,
	.readable_reg = ivm6303_readable_register,
	.writeable_reg = ivm6303_writeable_register,
	.volatile_reg = ivm6303_volatile_register,

	.cache_type = REGCACHE_RBTREE,
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
static int _setup_pll(struct snd_soc_component *component, unsigned int bclk)
{
	int ret, ch_index;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned long osr, rate, ratek, refclk, pll_input_divider,
		pll_feedback_divider;
	u8 tdm_fsyn_sr, tdm_bclk_osr, shift, mask, v;
	bool pll_was_enabled;

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
	    (pll_input_divider == priv->pll_input_divider))
		goto pll_done;
	/* Start updating PLL settings by disabling PLL, if it was enabled */
	ret = _set_pll_enable_check(priv, 0, &pll_was_enabled);
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

	/* Finally restore PLL state depending on current bias level */
	if (pll_was_enabled) {
		ret = _set_pll_enable(priv, 1);
		if (ret < 0) {
			dev_err(component->dev, "error restoring pll status");
			goto err2;
		}
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
	if (pll_was_enabled)
		_set_pll_enable(priv, 1);
err2:
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

	dev_dbg(component->dev, "%s: bps = %d, slot_width = %d\n",
		__func__, bps, priv->slot_width);

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

	if (rate < 16000 || rate > 96000) {
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

	ret = _do_power_up(priv);
	if (ret < 0)
		goto err;

	/* Set PLL given bclk */
	ret = _setup_pll(component, bclk);
	if (ret < 0)
		goto err;

	/* Set samples and slots sizes */
	ret = _set_sam_size(component, substream->stream, samsize);
err:
	_resync_power_state(component);
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

	if (v == priv->tdm_settings_1)
		return 0;

	ret = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(1),
				 TDM_SETTINGS_MASK, v);
	if (ret < 0) {
		dev_err(component->dev, "error writing to TDM_SETTINGS(1)");
		return ret;
	}
	priv->tdm_settings_1 = v;
	return 0;
}

static int ivm6303_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	int ret, stat;
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s entered, fmt = 0x%08x\n", __func__, fmt);

	/* Master/slave configuration */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(component->dev,
			"%s: codec can only be slave\n", __func__);
		return -EINVAL;
	}
	/* Clock gating */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_CONT: /* continuous clock */
		break;
	case SND_SOC_DAIFMT_GATED: /* clock is gated */
		break;
	default:
		dev_err(component->dev,
			"%s: ERROR: Unsupported clock mask (0x%x)!\n",
			__func__, fmt & SND_SOC_DAIFMT_CLOCK_MASK);
		return -EINVAL;
	}
	mutex_lock(&priv->regmap_mutex);
	ret = _do_power_up(priv);
	if (!ret)
		ret = _set_protocol(dai, fmt);
	stat = _resync_power_state(component);
	if (stat)
		dev_err(component->dev, "error in resync power state\n");
	mutex_unlock(&priv->regmap_mutex);
	dev_dbg(component->dev, "%s leaving, ret = %d\n", __func__, ret);
	return ret;
}

static int _assign_slot(struct ivm6303_priv *priv, bool tx, unsigned int ch,
			unsigned int slot)
{
	unsigned int base_en = tx ? IVM6303_TDM_SETTINGS(0xb) :
		IVM6303_TDM_SETTINGS(0x5);
	unsigned int ch_shift = tx ? 1 : 0;
	unsigned int r = base_en + (ch << ch_shift);
	unsigned int slots_mask = tx ? O_SLOT_CHAN_MASK : I_SLOT_CHAN_MASK;

	return regmap_update_bits(priv->regmap, r, slots_mask, slot);
}

static int _program_channels(struct ivm6303_priv *priv,
			     bool tx, unsigned int msk)
{
	struct device *dev = &priv->i2c_client->dev;
	unsigned int ch;
	unsigned int nch = tx ? 4 : 5, lowest_disabled_ch;
	const char *what = tx ? "tx" : "rx";
	int ret, i;

	lowest_disabled_ch = hweight32(msk) ? hweight32(msk) - 1 : 0;

	if (lowest_disabled_ch >= nch) {
		dev_err(dev, "requested more %s channels than available", what);
		return -EINVAL;
	}
	/* Disable unused channels first */
	for (ch = lowest_disabled_ch; ch < nch && !ret; ch++)
		ret = _assign_slot(priv, tx, ch, 0);
	if (ret)
		return ret;

	/*
	 * And finally program the other ones. Take channels sequentially and
	 * assign them to the enabled slots
	 */
	ch = 0;
	while ((i = ffs(msk))) {
		/* Slot i is active and assigned to channel ch */
		/* i ranges from 1 to 31, 0 means not assigned */
		ret = _assign_slot(priv, tx, ch, i);
		if (ret < 0) {
			dev_err(dev, "error setting up tx slot\n");
			break;
		}
		msk &= ~(1 << (i - 1));
		ch++;
	}
	return ret;
}

static int _program_tx_channels(struct ivm6303_priv *priv, unsigned int msk)
{
	return _program_channels(priv, true, msk);
}

static int _program_rx_channels(struct ivm6303_priv *priv, unsigned int msk)
{
	return _program_channels(priv, false, msk);
}

static int ivm6303_set_tdm_slot(struct snd_soc_dai *dai,
				/*
				 * HACK: tx mask is actually PLAYBACK mask
				 * tx mask is actually CAPTURE masks
				 */
				unsigned int playback_mask,
				unsigned int capture_mask,
				int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int w;
	int stat;
	unsigned int tx_mask = capture_mask, rx_mask = playback_mask;

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
	if (hweight32(rx_mask) > 2) {
		dev_err(component->dev, "Max 1 rx slots supported\n");
		return -EINVAL;
	}
	priv->capture_only = rx_mask == 0;
	if (priv->capture_only)
		dev_info(component->dev, "Configured for capture only\n");
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

	dev_dbg(component->dev, "slots number = %d, slot_width = %d\n",
		slots, slot_width);

	mutex_lock(&priv->regmap_mutex);
	stat = _do_power_up(priv);
	if (stat < 0) {
		dev_err(component->dev, "error powering up device\n");
		goto err;
	}
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(3),
				  I_SLOT_SIZE_MASK, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing input slot size\n");
		goto err;
	}
	stat = regmap_update_bits(priv->regmap, IVM6303_TDM_SETTINGS(4),
				  O_SLOT_SIZE_MASK, w);
	if (stat < 0) {
		dev_err(component->dev, "error writing output slot size\n");
		goto err;
	}
	stat = _program_tx_channels(priv, tx_mask);
	if (stat < 0)
		goto err;
	stat = _program_rx_channels(priv, rx_mask);
err:
	stat = _resync_power_state(component);
	if (stat < 0)
		dev_err(component->dev, "error resyncing power state\n");
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
	stat = _do_power_up(priv);
	if (stat) {
		dev_err(component->dev, "Error powering up device\n");
		goto err;
	}
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
	stat = _resync_power_state(component);
	if (stat < 0)
		dev_err(component->dev, "Error resyncing power state\n");
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
	stat = _do_power_up(priv);
	if (stat < 0) {
		dev_err(component->dev, "Error powering up device\n");
		goto err;
	}
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
	stat = _resync_power_state(component);
	if (stat)
		dev_err(component->dev, "error resyncing power state\n");
	mutex_unlock(&priv->regmap_mutex);
	return stat;
}

static int ivm6303_dai_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		/* Ignore mute on capture */
		return 0;

	mutex_lock(&priv->regmap_mutex);
	dev_dbg(component->dev, "%s, mute = %d\n", __func__, mute);
	ret = _do_power_up(priv);
	if (test_bit(SPEAKER_ENABLED, &priv->flags))
		ret = _do_mute(priv, mute);
	/* Mute status will be resynced on next speaker on */
	priv->muted = mute;
	ret = _resync_power_state(component);
	if (ret)
		dev_err(component->dev, "error resyncing power state\n");
	dev_dbg(component->dev, "%s returns\n", __func__);
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
	.mute_stream	= ivm6303_dai_mute,
};

const struct snd_soc_dai_ops ivm6303_tdm_dai_ops = {
	.hw_params	= ivm6303_hw_params,
	.set_fmt	= ivm6303_set_fmt,
	.set_tdm_slot   = ivm6303_set_tdm_slot,
	.set_channel_map = ivm6303_set_channel_map,
	.get_channel_map = ivm6303_get_channel_map,
	.mute_stream	= ivm6303_dai_mute,
};

static int ivm6303_tdm_dai_probe(struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	/* Initialize volume scale before adding control */
	/*
	 * Max volume set by firmware maps as 0dB, min volume corresponds to 1.
	 * Device volume step is 0.125dB, but control step is 0.25dB
	 * So: ((firmware_max_volume - 2)/2) * 0.25dB + min_volume_db = 0dB
	 * min_volume_db = -(1000 * ((firmware_max_volume - 2)/2) * 0.25dB)
	 * For instance: firmware_max_volume = 755 -> min_volume_db = -94375
	 * Units are 0.001dB here, while the kernel uses 0.01db. So we take
	 * a step of 0.25 dB and min_vol shall be an even multiple of the step
	 */
	int min_vol =((priv->max_volume - 1) * IVM6303_VOLUME_HW_STEP);

	rounddown(min_vol, IVM6303_VOLUME_CTRL_STEP);

	min_vol /= 10;

	vol_scale[SNDRV_CTL_TLVO_DB_SCALE_MIN] = -min_vol;

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
			.channels_max = 2,
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
	atomic_set(&priv->clk_status, STOPPED);

	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "regmap init failed\n");
		goto end;
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

#ifdef CONFIG_ACPI
static const struct acpi_device_id ivm6303_acpi_match[] = {
	{ "IVM6303", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ivm6303_acpi_match);
#endif

static struct i2c_driver ivm6303_i2c_driver = {
	.driver = {
		.name		= "ivm6303-amp",
		.owner		= THIS_MODULE,
		.of_match_table = ivm6303_match_table,
		.acpi_match_table = ACPI_PTR(ivm6303_acpi_match),
	},
	.probe		= ivm6303_probe,
	.remove		= ivm6303_remove,
	.id_table	= ivm6303_id,
};

module_i2c_driver(ivm6303_i2c_driver);

MODULE_DESCRIPTION("ASoC IVM6303 driver");
MODULE_AUTHOR("Davide Ciminaghi, ciminaghi@gnudd.com");
MODULE_LICENSE("GPL");
