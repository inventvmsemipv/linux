/* SPDX-License-Identifier: GPL-2.0
 *
 * linux/sound/soc-configfs-card.h -- ASOC configfs based card
 *
 * Author:		Davide Ciminaghi <ciminaghi@gnudd.cm>
 */

#ifndef __LINUX_SND_SOC_CONFIGFS_CARD_H
#define __LINUX_SND_SOC_CONFIGFS_CARD_H

#include <sound/pcm.h>
#include <linux/configfs.h>

#define MAX_CODECS 4
#define MAX_CPUS   2

struct asoc_configfs_dai_link {
	char *component_dev_name;
	struct bus_type *component_bt;
	char *component_dai_name;
	unsigned long slot_width;
	unsigned long rx_mask;
	unsigned long tx_mask;
	unsigned long mclk_fs;
	struct config_group group;
};

struct asoc_configfs_soundcard {
	const char *name;
	struct config_group group;
	int format;
	unsigned long invert_fsyn;
	unsigned long invert_bclk;
	int cpu_bitclock_master;
	int cpu_frameclock_master;
	int ncpus;
	/* Total number of slots for TDM */
	unsigned long total_slots;
	struct asoc_configfs_dai_link cpus[MAX_CPUS];
	int ncodecs;
	struct asoc_configfs_dai_link codecs[MAX_CODECS];
	struct platform_device *pdev;
};

#endif /* __LINUX_SND_SOC_CONFIGFS_CARD_H */

