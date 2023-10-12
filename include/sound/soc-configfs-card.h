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

#define MAX_DAI_LINKS 4

struct asoc_configfs_dai_data {
	char *component_dev_name;
	struct bus_type *component_bt;
	char *component_dai_name;
	unsigned long slot_width;
	unsigned long rx_mask;
	unsigned long tx_mask;
	unsigned long mclk_fs;
	struct config_group group;
};

struct asoc_configfs_dai_link_data {
	const char *name;
	const char *stream_name;
	int format;
	unsigned long invert_fsyn;
	unsigned long invert_bclk;
	int cpu_bitclock_master;
	int cpu_frameclock_master;
	int ncpus;
	/* Total number of slots for TDM */
	unsigned long total_slots;
	struct asoc_configfs_dai_data cpus[MAX_CPUS];
	int ncodecs;
	struct asoc_configfs_dai_data codecs[MAX_CODECS];
};

struct asoc_configfs_soundcard {
	const char *name;
	struct platform_device *pdev;
	int ndai_links;
	struct asoc_configfs_dai_link_data dai_links[MAX_DAI_LINKS];
	struct config_group group;
};

#endif /* __LINUX_SND_SOC_CONFIGFS_CARD_H */

