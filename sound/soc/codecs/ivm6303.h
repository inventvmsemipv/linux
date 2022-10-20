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
