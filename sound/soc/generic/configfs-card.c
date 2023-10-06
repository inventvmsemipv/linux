// configfs based asoc card
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/configfs.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

struct asoc_configfs_soundcard {
	const char *name;
	struct config_group group;
	int format;
	int cpu_bitclock_master;
	int cpu_frameclock_master;
};

static inline struct asoc_configfs_soundcard *
to_asoc_configfs_soundcard(struct config_group *g)
{
	return container_of(g, struct asoc_configfs_soundcard, group);
}

static const char *dai_formats[] = {
	[SND_SOC_DAIFMT_I2S] = "i2s",
	[SND_SOC_DAIFMT_RIGHT_J] = "right_j",
	[SND_SOC_DAIFMT_LEFT_J] = "left_j",
	[SND_SOC_DAIFMT_DSP_A] = "dsp_a",
	[SND_SOC_DAIFMT_DSP_B] = "dsp_b",
	[SND_SOC_DAIFMT_AC97] = "ac97",
	[SND_SOC_DAIFMT_PDM] = "pdm",
};

static ssize_t asoc_card_format_store(struct config_item *item,
				      const char *page, size_t len)
{
	int i;
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));

	pr_debug("%s: written %s\n", __func__, page);
	for (i = SND_SOC_DAIFMT_I2S; i <= SND_SOC_DAIFMT_PDM; i++)
		if (!strncmp(page, dai_formats[i], strlen(dai_formats[i]))) {
			sc->format = i;
			return len;
		}
	return -EINVAL;
}

static ssize_t _set_bitclock_master(const char *page, size_t len,
				    int *what)
{
	*what = !strncmp(page, "cpu", 3) ? 1 : 0;
	if (*what)
		return len;
	if (strncmp(page, "codec", 5))
		return -EINVAL;
	return len;
}

static ssize_t asoc_card_bitclock_master_store(struct config_item *item,
					       const char *page, size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));

	pr_debug("%s: written %s\n", __func__, page);
	return _set_bitclock_master(page, len, &sc->cpu_bitclock_master);
}

static ssize_t asoc_card_frameclock_master_store(struct config_item *item,
						 const char *page, size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));

	pr_debug("%s: written %s\n", __func__, page);
	return _set_bitclock_master(page, len, &sc->cpu_frameclock_master);
}

CONFIGFS_ATTR_WO(asoc_card_, format);
CONFIGFS_ATTR_WO(asoc_card_, bitclock_master);
CONFIGFS_ATTR_WO(asoc_card_, frameclock_master);

static struct configfs_attribute *soundcard_root_attrs[] = {
	&asoc_card_attr_format,
	&asoc_card_attr_bitclock_master,
	&asoc_card_attr_frameclock_master,
	NULL,
};

static void single_soundcard_type_item_release(struct config_item *item)
{
	struct config_group *gr = to_config_group(item);
	struct asoc_configfs_soundcard *sc = to_asoc_configfs_soundcard(gr);

	pr_err("%s invoked, item = %p, group = %p, sc = %p\n", __func__, item,
	       gr, sc);
	kfree(sc);
}

static struct configfs_item_operations single_soundcard_type_item_ops = {
	.release = single_soundcard_type_item_release,
};

static const struct config_item_type single_soundcard_type = {
	.ct_item_ops = &single_soundcard_type_item_ops,
	.ct_attrs = soundcard_root_attrs,
	.ct_owner = THIS_MODULE,
};

static struct config_group *
asoc_soundcard_make_group(struct config_group *group, const char *name)
{
	struct asoc_configfs_soundcard *out = kzalloc(sizeof(*out), GFP_KERNEL);

	pr_err("%s: allocated soundcard %p\n", __func__, out);
	if (!out)
		return ERR_PTR(-ENOMEM);
	out->name = group->cg_item.ci_namebuf;
	config_group_init_type_name(&out->group, name, &single_soundcard_type);
	return &out->group;
}

static struct configfs_group_operations soundcard_type_group_ops = {
	.make_group = asoc_soundcard_make_group,
};

static struct config_group *soundcard_group;

static const struct config_item_type soundcard_type = {
	.ct_owner = THIS_MODULE,
	.ct_group_ops = &soundcard_type_group_ops,
};

static const struct config_item_type asoc_group_type = {
	.ct_owner = THIS_MODULE,
};

static struct configfs_subsystem asoc_configfs = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "asoc",
			.ci_type = &asoc_group_type,
		},
	},
};

static int __init configfs_card_init(void)
{
	int ret;
	struct config_group *root = &asoc_configfs.su_group;

	config_group_init(root);
	mutex_init(&asoc_configfs.su_mutex);

	ret = configfs_register_subsystem(&asoc_configfs);
	if (ret)
		return ret;

	soundcard_group = configfs_register_default_group(root,
							  "soundcard",
							  &soundcard_type);
	return PTR_ERR_OR_ZERO(soundcard_group);
}

static void __exit configfs_card_exit(void)
{
	configfs_unregister_default_group(soundcard_group);
	configfs_unregister_subsystem(&asoc_configfs);
}

module_init(configfs_card_init);
module_exit(configfs_card_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ASoC Configfs Sound Card");
MODULE_AUTHOR("Davide Ciminaghi <ciminaghi@gnudd.com>");
