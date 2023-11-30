// SPDX-License-Identifier: GPL-2.0
//
// ASoC machine driver for configfs asoc soundcard
#define DEBUG 1
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-configfs-card.h>

struct configfs_sc_tdm_data {
	unsigned int rx_mask;
	unsigned int tx_mask;
	unsigned long total_slots;
	int slot_width;
};

struct configfs_sc_priv {
	struct snd_soc_dai_link dai_link;
	unsigned int mclk_fs;
	unsigned int fmt;
	struct configfs_sc_tdm_data *tdm_data;
};

static void setup_name_ofnode(struct snd_soc_dai_link_component *dlc,
			      struct asoc_configfs_dai_link *dl)
{
	struct device *dev = bus_find_device_by_name(dl->component_bt,
						     NULL,
						     dl->component_dev_name);

	if (!dev || !dev->of_node) {
		dlc->name = dl->component_dev_name;
		pr_debug("%s: assigning name %s\n", __func__, dlc->name);
		return;
	}
	pr_debug("%s: assigning of_node %p\n", __func__, dev->of_node);
	dlc->of_node = of_node_get(dev->of_node);
}

static int configfs_sc_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct configfs_sc_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec_dai, *cpu_dai;
	int i;

	dev_dbg(rtd->card->dev, "%s entered\n", __func__);

	cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	snd_soc_dai_set_fmt(cpu_dai, priv->fmt);

	for_each_rtd_codec_dais(rtd, i, codec_dai)
		snd_soc_dai_set_fmt(codec_dai, priv->fmt);

	return 0;
}

static int configfs_sc_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	dev_dbg(rtd->card->dev, "%s entered\n", __func__);
	return 0;
}

static void configfs_sc_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);

	dev_dbg(rtd->card->dev, "%s entered\n", __func__);
}

static const struct snd_soc_ops configfs_sc_card_ops = {
	.hw_params = configfs_sc_hw_params,
	.startup = configfs_sc_startup,
	.shutdown = configfs_sc_shutdown,
};

static int configfs_sc_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct configfs_sc_priv *priv;
	struct configfs_sc_tdm_data *tdata;
	int i, ret;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link);
	dev_dbg(rtd->card->dev, "%s entered\n", __func__);
	priv = snd_soc_card_get_drvdata(rtd->card);

	ret = snd_soc_dai_set_sysclk(asoc_rtd_to_cpu(rtd, 0), 0, 24576000/2,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	tdata = &priv->tdm_data[0];
	ret = snd_soc_dai_set_tdm_slot(asoc_rtd_to_cpu(rtd, 0), tdata->tx_mask,
				       tdata->rx_mask, tdata->total_slots,
				       tdata->slot_width);
	if (ret < 0 && ret != -ENOTSUPP)
		return ret;
	for (i = 0; i < rtd->dai_link->num_codecs; i++) {
		tdata = &priv->tdm_data[i + 1];
		ret = snd_soc_dai_set_tdm_slot(asoc_rtd_to_codec(rtd, i),
					       tdata->tx_mask, tdata->rx_mask,
					       tdata->total_slots,
					       tdata->slot_width);
		if (ret < 0 && ret != -ENOTSUPP)
			return ret;
	}
	return 0;
}

static int configfs_sc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct snd_soc_dai_link *link;
	struct configfs_sc_priv *priv;
	struct configfs_sc_tdm_data *tdata;
	struct asoc_configfs_soundcard *configfs_data;
	struct snd_soc_codec_conf *codec_confs = NULL;
	const char *stereo_prefixes[2] = {
		"Left", "Right",
	};
	const char *multi_prefixes[8] = {
		"Ch0", "Ch1", "Ch2", "Ch3", "Ch4", "Ch5", "Ch6", "Ch7",
	};
	const char **prefixes = NULL;
	int i, ret;

	dev_dbg(&pdev->dev, "%s entered\n", __func__);
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	configfs_data = dev_get_platdata(&pdev->dev);
	if (!configfs_data) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	card = devm_kzalloc(&pdev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->name = configfs_data->name;
	card->owner = THIS_MODULE;
	card->late_probe = configfs_sc_late_probe;

	link = &priv->dai_link;

	link->dai_fmt = configfs_data->format | SND_SOC_DAIFMT_NB_NF;
	if (!configfs_data->cpu_bitclock_master &&
	    !configfs_data->cpu_frameclock_master)
		link->dai_fmt |= SND_SOC_DAIFMT_CBM_CFM;
	else if (configfs_data->cpu_bitclock_master &&
		 !configfs_data->cpu_frameclock_master)
		link->dai_fmt |= SND_SOC_DAIFMT_CBS_CFM;
	else if (!configfs_data->cpu_bitclock_master &&
		 configfs_data->cpu_frameclock_master)
		link->dai_fmt |= SND_SOC_DAIFMT_CBM_CFS;
	else
		link->dai_fmt |= SND_SOC_DAIFMT_CBS_CFS;
	if (configfs_data->invert_fsyn && configfs_data->invert_bclk)
		link->dai_fmt |= SND_SOC_DAIFMT_IB_IF;
	if (configfs_data->invert_fsyn && !configfs_data->invert_bclk)
		link->dai_fmt |= SND_SOC_DAIFMT_NB_IF;
	if (!configfs_data->invert_fsyn && configfs_data->invert_bclk)
		link->dai_fmt |= SND_SOC_DAIFMT_IB_NF;

	link->name = "configurable-link";
	link->stream_name = link->name;

	link->num_cpus = configfs_data->ncpus;
	link->cpus = devm_kzalloc(&pdev->dev,
				  sizeof(link->cpus[0]) * link->num_cpus,
				  GFP_KERNEL);
	if (!link->cpus)
		return -ENOMEM;
	setup_name_ofnode(&link->cpus[0], &configfs_data->cpus[0]);
	link->cpus[0].dai_name = configfs_data->cpus[0].component_dai_name;
	dev_dbg(&pdev->dev, "link->cpus[0].name = %s\n", link->cpus[0].name);

	link->num_codecs = configfs_data->ncodecs;
	link->codecs = devm_kzalloc(&pdev->dev,
				    sizeof(link->codecs[0])*link->num_codecs,
				    GFP_KERNEL);
	if (!link->codecs)
		return -ENOMEM;
	if (link->num_codecs > 1) {
		int sz;

		if (link->num_codecs > ARRAY_SIZE(multi_prefixes)) {
			dev_err(&pdev->dev, "too many codecs");
			return -ENOMEM;
		}
		sz = link->num_codecs * sizeof(codec_confs[0]);
		codec_confs = devm_kzalloc(&pdev->dev, sz, GFP_KERNEL);
		if (link->num_codecs == 2)
			prefixes = stereo_prefixes;
		else
			prefixes = multi_prefixes;
		card->codec_conf = codec_confs;
		card->num_configs = link->num_codecs;
	}
	for (i = 0; i < link->num_codecs; i++) {
		struct asoc_configfs_dai_link *c = &configfs_data->codecs[i];

		setup_name_ofnode(&link->codecs[i], c);
		link->codecs[i].dai_name = c->component_dai_name;
		if (codec_confs) {
			if (link->codecs[i].name)
				codec_confs[i].dlc.name =
					link->codecs[i].name;
			else
				codec_confs[i].dlc.of_node =
					link->codecs[i].of_node;
			codec_confs[i].name_prefix = prefixes[i];
		}
	}

	link->num_platforms = 1;
	link->platforms = devm_kzalloc(&pdev->dev,
				       sizeof(link->platforms[0]) *
				       link->num_platforms,
				       GFP_KERNEL);
	if (!link->platforms)
		return -ENOMEM;
	card->dai_link = link;
	card->num_links = 1;
	card->dev = &pdev->dev;

	priv->mclk_fs = configfs_data->cpus[0].mclk_fs;

	link->ops = &configfs_sc_card_ops;

	priv->tdm_data = devm_kzalloc(&pdev->dev,
				      sizeof(priv->tdm_data[0]) *
				      link->num_codecs + link->num_cpus,
				      GFP_KERNEL);
	if (!priv->tdm_data)
		return -ENOMEM;
	/* Parse cpu tdm slots configuration */
	tdata = &priv->tdm_data[0];
	tdata->tx_mask = configfs_data->cpus[0].tx_mask;
	tdata->rx_mask = configfs_data->cpus[0].rx_mask;
	tdata->total_slots = configfs_data->total_slots;
	tdata->slot_width = configfs_data->cpus[0].slot_width;

	/* Parse codecs' tdm slots configurations */
	for (i = 0; i < configfs_data->ncodecs; i++) {
		tdata = &priv->tdm_data[i + 1];
		tdata->tx_mask = configfs_data->codecs[i].tx_mask;
		tdata->rx_mask = configfs_data->codecs[i].rx_mask;
		tdata->total_slots = configfs_data->total_slots;
		tdata->slot_width = configfs_data->codecs[i].slot_width;
	}

	link->platforms->name = link->cpus->name;
	link->platforms->of_node = link->cpus->of_node;

	/* Parse format */
	priv->fmt = link->dai_fmt;

	snd_soc_card_set_drvdata(card, priv);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	return ret;
}

static int configfs_sc_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id configfs_sc_driver_ids[] = {
	{ .name = "configfssc", },
	{},
};
MODULE_DEVICE_TABLE(platform, configfs_sc_driver_ids);

static struct platform_driver configfs_sc_driver = {
	.id_table = configfs_sc_driver_ids,
	.probe = configfs_sc_probe,
	.remove = configfs_sc_remove,
	.driver = {
		.name = "soundcard",
	},
};

module_platform_driver(configfs_sc_driver);

MODULE_DESCRIPTION("ALSA SoC Audio machine driver for configfs soundcards");
MODULE_LICENSE("GPL");
