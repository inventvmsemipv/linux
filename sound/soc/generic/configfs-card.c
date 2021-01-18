// configfs based asoc card
#include <linux/module.h>
#include <linux/configfs.h>

struct asoc_configfs_soundcard {
	const char *name;
	struct config_group group;
};

static struct config_group *soundcard_group;

static const struct config_item_type soundcard_type = {
	.ct_owner = THIS_MODULE,
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
