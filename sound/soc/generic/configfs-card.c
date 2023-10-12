// configfs based asoc card
#define DEBUG 1
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/configfs.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/soc-configfs-card.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

static atomic_t asoc_sc_instance ;

static inline struct asoc_configfs_soundcard *
to_asoc_configfs_soundcard(struct config_group *g)
{
	return container_of(g, struct asoc_configfs_soundcard, group);
}

static inline struct asoc_configfs_dai_data *
to_asoc_configfs_dai_data(struct config_group *g)
{
	return container_of(g, struct asoc_configfs_dai_data, group);
}

struct btype {
	struct bus_type *bt;
	const char *name;
} bus_types[] = {
	{
		.bt = &platform_bus_type,
		.name = "platform",
	},
	{
		.bt = &i2c_bus_type,
		.name = "i2c",
	},
};

static ssize_t asoc_card_dai_data_comp_bustype_store(struct config_item *item,
						     const char *page,
						     size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));
	int i;

	for (i = 0; i < ARRAY_SIZE(bus_types); i++) {
		if (!strncmp(bus_types[i].name, page,
			     strnlen(bus_types[i].name, len))) {
			dd->component_bt = bus_types[i].bt;
			return len;
		}
	}
	return -EINVAL;
}

static ssize_t asoc_card_dai_data_comp_devname_store(struct config_item *item,
						     const char *page,
						     size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));

	if (dd->component_dev_name)
		/* Already assigned previously ? */
		kfree(dd->component_dev_name);
	dd->component_dev_name = kstrndup(page, PAGE_SIZE, GFP_KERNEL);
	dd->component_dev_name[strlen(dd->component_dev_name) - 1] = 0;
	pr_debug("dai_data->component_dev_name = %s\n", dd->component_dev_name);
	if (!dd->component_dev_name)
		return -ENOMEM;
	return len;
}

static ssize_t asoc_card_dai_data_comp_dainame_store(struct config_item *item,
						     const char *page,
						     size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));

	if (dd->component_dai_name)
		/* Already assigned previously ? */
		kfree(dd->component_dai_name);
	dd->component_dai_name = kstrndup(page, PAGE_SIZE, GFP_KERNEL);
	dd->component_dai_name[strlen(dd->component_dai_name) - 1] = 0;
	pr_debug("dai_data->component_dai_name = %s\n", dd->component_dai_name);
	if (!dd->component_dai_name)
		return -ENOMEM;
	return len;
}


static ssize_t asoc_card_dai_data_slot_width_store(struct config_item *item,
						   const char *page, size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));
	int ret;

	ret = kstrtoul(page, 10, &dd->slot_width);
	return ret < 0 ? ret : len;
}

static ssize_t asoc_card_dai_data_rx_mask_store(struct config_item *item,
						const char *page, size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));
	int ret;

	ret = kstrtoul(page, 16, &dd->rx_mask);
	return ret < 0 ? ret : len;
}

static ssize_t asoc_card_dai_data_tx_mask_store(struct config_item *item,
						const char *page, size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));
	int ret;

	ret = kstrtoul(page, 16, &dd->tx_mask);
	return ret < 0 ? ret : len;
}

static ssize_t asoc_card_dai_data_mclk_fs_store(struct config_item *item,
						const char *page, size_t len)
{
	struct asoc_configfs_dai_data *dd =
		to_asoc_configfs_dai_data(to_config_group(item));
	int ret;
	
	ret = kstrtoul(page, 10, &dd->mclk_fs);
	return ret < 0 ? ret : len;
}


CONFIGFS_ATTR_WO(asoc_card_dai_data_, comp_bustype);
CONFIGFS_ATTR_WO(asoc_card_dai_data_, comp_devname);
CONFIGFS_ATTR_WO(asoc_card_dai_data_, comp_dainame);
CONFIGFS_ATTR_WO(asoc_card_dai_data_, slot_width);
CONFIGFS_ATTR_WO(asoc_card_dai_data_, rx_mask);
CONFIGFS_ATTR_WO(asoc_card_dai_data_, tx_mask);
CONFIGFS_ATTR_WO(asoc_card_dai_data_, mclk_fs);

static struct configfs_attribute *asoc_card_dai_data_attrs[] = {
	&asoc_card_dai_data_attr_comp_bustype,
	&asoc_card_dai_data_attr_comp_devname,
	&asoc_card_dai_data_attr_comp_dainame,
	&asoc_card_dai_data_attr_slot_width,
	&asoc_card_dai_data_attr_rx_mask,
	&asoc_card_dai_data_attr_tx_mask,
	&asoc_card_dai_data_attr_mclk_fs,
	NULL,
};

static void dai_type_item_release(struct config_item *item)
{
	struct config_group *gr = to_config_group(item);
	struct asoc_configfs_dai_data *dd = to_asoc_configfs_dai_data(gr);

	pr_debug("%s invoked, item = %p, group = %p, dai_data = %p\n",
		 __func__, item, gr, dd);
	if (dd->component_dev_name)
		kfree(dd->component_dev_name);
}

static struct configfs_item_operations dai_data_type_item_ops = {
	.release = dai_type_item_release,
};

static const struct config_item_type dai_data_type = {
	.ct_item_ops = &dai_data_type_item_ops,
	.ct_owner = THIS_MODULE,
	.ct_attrs = asoc_card_dai_data_attrs
};

static struct config_group *_make_codec_dai_data(struct config_group *group,
						 const char *name)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(group);
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];
	struct asoc_configfs_dai_data *out;

	if (dld->ncodecs >= MAX_CODECS) {
		pr_err("max %d codec dai links allowed\n", MAX_CPUS);
		return ERR_PTR(-ENOMEM);
	}

	out = &dld->codecs[dld->ncodecs++];

	pr_debug("created codec dai %s, %p\n", name, out);

	config_group_init_type_name(&out->group, name, &dai_data_type);
	return &out->group;
}

static struct config_group *_make_cpu_dai_data(struct config_group *group,
					       const char *name)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(group);
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];
	struct asoc_configfs_dai_data *out;

	if (dld->ncpus >= MAX_CPUS) {
		pr_err("max %d cpu dais allowed\n", MAX_CPUS);
		return ERR_PTR(-ENOMEM);
	}

	out = &dld->cpus[dld->ncpus++];

	pr_debug("created cpu dai %s, %p\n", name, out);

	config_group_init_type_name(&out->group, name, &dai_data_type);
	return &out->group;
}

static struct config_group *
single_soundcard_dai_link_type_make_group(struct config_group *group,
					  const char *name)
{
	if (!strncmp(name, "codec", 5))
		return _make_codec_dai_data(group, name);
	if (!strncmp(name, "cpu", 3))
		return _make_cpu_dai_data(group, name);
	return ERR_PTR(-EINVAL);
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

static ssize_t asoc_card_dai_link_format_store(struct config_item *item,
					       const char *page, size_t len)
{
	int i;
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];

	sc->ndai_links = 1;

	pr_debug("%s: written %s\n", __func__, page);
	for (i = SND_SOC_DAIFMT_I2S; i <= SND_SOC_DAIFMT_PDM; i++)
		if (!strncmp(page, dai_formats[i], strlen(dai_formats[i]))) {
			dld->format = i;
			if (i == SND_SOC_DAIFMT_I2S)
				dld->total_slots = 2;
			return len;
		}
	return -EINVAL;
}

static ssize_t asoc_card_dai_link_invert_fsyn_store(struct config_item *item,
						    const char *page,
						    size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];
	int ret;

	pr_debug("%s: written %s\n", __func__, page);
	ret = kstrtoul(page, 10, &dld->invert_fsyn);
	return ret < 0 ? ret : len;
}

static ssize_t asoc_card_dai_link_invert_bclk_store(struct config_item *item,
						    const char *page,
						    size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];
	int ret;

	pr_debug("%s: written %s\n", __func__, page);
	ret = kstrtoul(page, 10, &dld->invert_bclk);
	return ret < 0 ? ret : len;
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

static ssize_t
asoc_card_dai_link_bitclock_master_store(struct config_item *item,
					 const char *page, size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];

	pr_debug("%s: written %s\n", __func__, page);
	return _set_bitclock_master(page, len, &dld->cpu_bitclock_master);
}

static ssize_t
asoc_card_dai_link_frameclock_master_store(struct config_item *item,
					   const char *page, size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];

	pr_debug("%s: written %s\n", __func__, page);
	return _set_bitclock_master(page, len, &dld->cpu_frameclock_master);
}

static void fixup_total_slots(struct asoc_configfs_dai_link_data *dld)
{
	int i;

	if (dld->total_slots)
		/* Already assigned (I2S) */
		return;

	for (i = 0; i < dld->ncodecs; i++) {
		int rxns, txns;

		rxns = hweight_long(dld->codecs[i].rx_mask);
		txns = hweight_long(dld->codecs[i].tx_mask);
		dld->total_slots += max(rxns, txns);
	}
	pr_debug("%s calculated %lu slots\n", __func__, dld->total_slots);
}

static ssize_t asoc_card_dai_link_command_store(struct config_item *item,
						const char *page, size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	struct asoc_configfs_dai_link_data *dld = &sc->dai_links[0];
	struct platform_device *pdev;

	if (strncmp(page, "start", 5)) {
		pr_err("say start to register and start up the board\n");
		return -EINVAL;
	}
	fixup_total_slots(dld);
	pdev = platform_device_register_resndata(NULL, "configfssc",
						 atomic_inc_return(&asoc_sc_instance),
						 NULL, 0,
						 sc, sizeof(*sc));
	if (IS_ERR(pdev)) {
		pr_err("configfs-card: error registering platform device\n");
		return PTR_ERR(pdev);
	}
	sc->pdev = pdev;
	dev_info(&pdev->dev, "registered (asoc configfs board %s)\n", sc->name);
	return len;
}

static ssize_t asoc_card_dai_link_total_slots_store(struct config_item *item,
						    const char *page,
						    size_t len)
{
	struct asoc_configfs_soundcard *sc =
		to_asoc_configfs_soundcard(to_config_group(item));
	int ret;

	ret = kstrtoul(page, 10, &sc->dai_links[0].total_slots);
	return ret < 0 ? ret : len;
}

CONFIGFS_ATTR_WO(asoc_card_dai_link_, format);
CONFIGFS_ATTR_WO(asoc_card_dai_link_, invert_fsyn);
CONFIGFS_ATTR_WO(asoc_card_dai_link_, invert_bclk);
CONFIGFS_ATTR_WO(asoc_card_dai_link_, bitclock_master);
CONFIGFS_ATTR_WO(asoc_card_dai_link_, frameclock_master);
CONFIGFS_ATTR_WO(asoc_card_dai_link_, command);
CONFIGFS_ATTR_WO(asoc_card_dai_link_, total_slots);

static struct configfs_attribute *soundcard_dai_link_attrs[] = {
	&asoc_card_dai_link_attr_format,
	&asoc_card_dai_link_attr_invert_fsyn,
	&asoc_card_dai_link_attr_invert_bclk,
	&asoc_card_dai_link_attr_bitclock_master,
	&asoc_card_dai_link_attr_frameclock_master,
	&asoc_card_dai_link_attr_command,
	&asoc_card_dai_link_attr_total_slots,
	NULL,
};

static
void single_soundcard_dai_link_type_item_release(struct config_item *item)
{
	struct config_group *gr = to_config_group(item);
	struct asoc_configfs_soundcard *sc = to_asoc_configfs_soundcard(gr);

	pr_err("%s invoked, item = %p, group = %p, sc = %p\n", __func__, item,
	       gr, sc);
	platform_device_unregister(sc->pdev);
	kfree(sc->name);
	kfree(sc);
}

static
struct configfs_item_operations single_soundcard_dai_link_type_item_ops = {
	.release = single_soundcard_dai_link_type_item_release,
};

static
struct configfs_group_operations single_soundcard_dai_link_type_group_ops = {
	.make_group = single_soundcard_dai_link_type_make_group,
};

static const struct config_item_type single_soundcard_dai_link_type = {
	.ct_item_ops = &single_soundcard_dai_link_type_item_ops,
	.ct_group_ops = &single_soundcard_dai_link_type_group_ops,
	.ct_attrs = soundcard_dai_link_attrs,
	.ct_owner = THIS_MODULE,
};

static struct config_group *
asoc_soundcard_make_group(struct config_group *group, const char *name)
{
	struct asoc_configfs_soundcard *out = kzalloc(sizeof(*out), GFP_KERNEL);
	struct asoc_configfs_dai_link_data *dld;

	pr_err("%s: allocated soundcard %p (%s)\n", __func__, out, name);
	if (!out)
		return ERR_PTR(-ENOMEM);
	dld = &out->dai_links[0];
	out->name = kstrndup(name, 32, GFP_KERNEL);
	out->ndai_links = 1;
	dld->name = "configurable-link";
	dld->stream_name = "configurable-link-stream";
	config_group_init_type_name(&out->group, name,
				    &single_soundcard_dai_link_type);
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
