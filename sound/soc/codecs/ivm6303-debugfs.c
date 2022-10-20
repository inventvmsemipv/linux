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
#include <linux/debugfs.h>
#include <sound/soc.h>
#include "ivm6303.h"

void ivm6303_init_debugfs(struct snd_soc_component *component)
{
	struct dentry *root = NULL;
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	root = debugfs_create_dir(dev_name(component->dev),
				  component->debugfs_root);
	debugfs_create_x8("hw_rev", 0444, root, &priv->hw_rev);
	debugfs_create_ulong("pll_feedback_divider", 0444, root,
			     &priv->pll_feedback_divider);
	debugfs_create_ulong("pll_input_divider", 0444, root,
			     &priv->pll_input_divider);
	debugfs_create_u32("slots", 0444, root, &priv->slots);
	debugfs_create_u32("slot_width", 0444, root, &priv->slot_width);
	debugfs_create_atomic_t("clk_status", 0444, root, &priv->clk_status);
}

void ivm6303_cleanup_debugfs(struct snd_soc_component *component)
{
	struct ivm6303_priv *priv = snd_soc_component_get_drvdata(component);

	debugfs_remove_recursive(priv->debugfs_root);
}
