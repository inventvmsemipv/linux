/*
 * ivm6303.h -- ivm6303 ALSA SoC Audio Driver, private header
 *
 * Copyright 2022 Inventvm
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __IVM6303_H__
#define __IVM6303_H__

#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>


#define IVM6303_SECTION_MAX_REGISTERS 512

struct ivm6303_register {
	u16 addr;
	u16 val;
	/* Delay after register write in usecs */
	unsigned int delay_us;
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

struct ivm6303_fw_section {
	int can_be_aborted;
	struct ivm6303_register *regs;
	int nsteps;
};

enum ivm6303_clk_status {
	ERROR = -1,
	STOPPED = 0,
	WAITING_FOR_PLL_LOCK,
	WAITING_FOR_CLKMON_OK,
	RUNNING,
};

struct ivm6303_priv {
	struct workqueue_struct	*wq;
	struct delayed_work	pll_locked_work;
	struct work_struct	speaker_deferred_work;
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
	int			clkmon_ok_attempts;
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
	int			autocal_done;
	atomic_t		clk_status;
#define WAITING_FOR_SPEAKER_OFF 1
#define WAITING_FOR_SPEAKER_ON 2
#define SPEAKER_ENABLED 3
#define DEFERRED_MUTE 4
	unsigned long		flags;
	unsigned int		saved_volume;
	int			muted;
};


#ifdef CONFIG_DEBUG_FS

void ivm6303_init_debugfs(struct snd_soc_component *component);
void ivm6303_cleanup_debugfs(struct snd_soc_component *component);

#else /* !CONFIG_DEBUG_FS */

static inline void ivm6303_init_debugfs(struct snd_soc_component *component)
{
}

static inline void ivm6303_cleanup_debugfs(struct snd_soc_component *component)
{
}

#endif /* !CONFIG_DEBUG_FS */

#endif /* __IVM6303_H__ */
