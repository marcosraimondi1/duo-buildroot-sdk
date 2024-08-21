// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remoteproc driver for Sophgo CV1800b.
 *
 * Copyright (c) 2024 GP Orcullo <kinsamanka@gmail.com>
 */

#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>

#include "remoteproc_internal.h"

#define MBOX_NB_VQ			2

#define CV180X_CLK_OFFSET		0x24
#define CV180X_CLK_MASK			BIT(6)
#define CV180X_CLK_START		BIT(6)
#define CV180X_CLK_STOP			0

#define RESET_DELAY_US			10

static int cv180x_rproc_xtr_mbox_init(struct rproc *rproc);

struct cv180x_rproc_dcfg {
	u32 src_reg;
	u32 src_mask;
	u32 src_start;
	u32 src_stop;
};

struct cv180x_rproc {
	struct device			*dev;
	struct regmap			*regmap;
	struct rproc			*rproc;
	const struct cv180x_rproc_dcfg	*dcfg;
	struct mbox_client		cl;
	struct mbox_chan		*tx_ch;
	struct mbox_chan		*rx_ch;
	struct work_struct              rproc_work;
	struct workqueue_struct         *workqueue;
};

static const struct cv180x_rproc_dcfg cv180x_rproc_cfg = {
	.src_reg   = CV180X_CLK_OFFSET,
	.src_mask  = CV180X_CLK_MASK,
	.src_start = CV180X_CLK_START,
	.src_stop  = CV180X_CLK_STOP,
};

static int cv180x_rproc_start(struct rproc *rproc)
{
	struct cv180x_rproc *priv = rproc->priv;
	const struct cv180x_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	int ret;

	ret = cv180x_rproc_xtr_mbox_init(rproc);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
				 dcfg->src_mask, dcfg->src_stop);
	if (ret)
		goto err_start;

	udelay(RESET_DELAY_US);

	ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
				 dcfg->src_mask, dcfg->src_start);
	if (ret)
		goto err_start;

	return ret;

err_start:
	dev_err(dev, "Failed to start secondary core!\n");

	return ret;
}

static int cv180x_rproc_stop(struct rproc *rproc)
{
	struct cv180x_rproc *priv = rproc->priv;
	const struct cv180x_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = priv->dev;
	int ret;

	ret = regmap_update_bits(priv->regmap, dcfg->src_reg,
				 dcfg->src_mask, dcfg->src_stop);
	if (ret)
		dev_err(dev, "Failed to stop secondary core!\n");

	return ret;
}

static int cv180x_rproc_mem_alloc(struct rproc *rproc,
				  struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %p+%zx\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "Unable to map memory region: %pa+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	mem->va = va;

	return 0;
}

static int cv180x_rproc_mem_release(struct rproc *rproc,
				    struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static int cv180x_rproc_prepare(struct rproc *rproc)
{
	struct cv180x_rproc *priv = rproc->priv;
	struct device_node *np = priv->dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			of_node_put(it.node);
			dev_err(priv->dev, "unable to acquire memory-region %s\n", it.node->name);
			return -EINVAL;
		}

		dev_info(priv->dev, "Acquired memory-region: %s, base: %pa, size: %zx\n", it.node->name, &rmem->base, rmem->size);

		/* Register memory region */
		mem = rproc_mem_entry_init(priv->dev, NULL,
					   (dma_addr_t)rmem->base,
					   rmem->size, rmem->base,
					   cv180x_rproc_mem_alloc,
					   cv180x_rproc_mem_release,
					   it.node->name);

		if (mem) {
			rproc_coredump_add_segment(rproc, rmem->base, rmem->size);
		} else {
			of_node_put(it.node);
			return -ENOMEM;
		}

		rproc_add_carveout(rproc, mem);
	}

	return  0;
}

static int cv180x_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	if (rproc_elf_load_rsc_table(rproc, fw))
		dev_info(&rproc->dev, "No resource table in elf\n");

	return 0;
}

static void cv180x_rproc_kick(struct rproc *rproc, int vqid)
{
	struct cv180x_rproc *priv = rproc->priv;
	int err;

	if (WARN_ON(vqid >= MBOX_NB_VQ))
		return;

	if (!priv->tx_ch) {
		dev_err(priv->dev, "No initialized mbox tx channel\n");
		return;
	}

	err = mbox_send_message(priv->tx_ch, (void *)&vqid);
	if (err < 0)
		dev_err(priv->dev, "%s: failed (%d, err:%d)\n",
			__func__, vqid, err);
}

static void cv180x_rproc_vq_work(struct work_struct *work)
{
	struct cv180x_rproc *priv = container_of(work, struct cv180x_rproc,
						 rproc_work);
	struct rproc *rproc = priv->rproc;

	if (rproc_vq_interrupt(rproc, 0) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message was found in vqid 0\n");
}

static void cv180x_rproc_rx_callback(struct mbox_client *cl, void *msg)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct cv180x_rproc *priv = rproc->priv;

	queue_work(priv->workqueue, &priv->rproc_work);
}

static void cv180x_rproc_free_mbox(struct rproc *rproc)
{
	struct cv180x_rproc *priv = rproc->priv;

	if (priv->tx_ch) {
		mbox_free_channel(priv->tx_ch);
		priv->tx_ch = NULL;
	}

	if (priv->rx_ch) {
		mbox_free_channel(priv->rx_ch);
		priv->rx_ch = NULL;
	}
}

static int cv180x_rproc_xtr_mbox_init(struct rproc *rproc)
{
	struct cv180x_rproc *priv = rproc->priv;
	struct device *dev = priv->dev;
	struct mbox_client *cl;

	if (priv->tx_ch && priv->rx_ch)
		return 0;

	if (!of_get_property(dev->of_node, "mbox-names", NULL))
		return 0;

	cl = &priv->cl;
	cl->dev = dev;
	cl->tx_block = true;
	cl->tx_tout = 100;
	cl->knows_txdone = false;
	cl->rx_callback = cv180x_rproc_rx_callback;

	priv->tx_ch = mbox_request_channel_byname(cl, "tx");
	if (IS_ERR(priv->tx_ch))
		return dev_err_probe(cl->dev, PTR_ERR(priv->tx_ch),
				     "failed to request tx mailbox channel\n");

	priv->rx_ch = mbox_request_channel_byname(cl, "rx");
	if (IS_ERR(priv->rx_ch)) {
		mbox_free_channel(priv->tx_ch);
		return dev_err_probe(cl->dev, PTR_ERR(priv->rx_ch),
				     "failed to request rx mailbox channel\n");
	}

	return 0;
}

static const struct rproc_ops cv180x_rproc_ops = {
	.prepare	= cv180x_rproc_prepare,
	.start		= cv180x_rproc_start,
	.stop		= cv180x_rproc_stop,
	.kick		= cv180x_rproc_kick,
	.load		= rproc_elf_load_segments,
	.parse_fw	= cv180x_rproc_parse_fw,
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check	= rproc_elf_sanity_check,
	.get_boot_addr	= rproc_elf_get_boot_addr,
};

static int cv180x_rproc_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Probing remoteproc\n");
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cv180x_rproc *priv;
	struct rproc *rproc;
	struct regmap *regmap;
	struct regmap_config config = { .name = "cv180x-rproc" };
	const struct cv180x_rproc_dcfg *dcfg;
	const char *fw_name;
	int ret;

	regmap = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(regmap)) {
		dev_err(dev, "failed to find syscon\n");
		return PTR_ERR(regmap);
	}
	regmap_attach_dev(dev, regmap, &config);

	ret = of_property_read_string(np, "firmware-name", &fw_name);
	if (ret)
	{
		dev_err(dev, "failed to read firmware-name\n");
		fw_name = NULL;
	}

	rproc = rproc_alloc(dev, dev_name(dev), &cv180x_rproc_ops, fw_name,
			    sizeof(*priv));
	if (!rproc)
	{
		dev_err(dev, "failed to allocate remoteproc\n");
		return -ENOMEM;
	}

	dcfg = of_device_get_match_data(dev);
	if (!dcfg) {
		dev_err(dev, "failed to get device match data\n");
		ret = -EINVAL;
		goto err_put_rproc;
	}

	rproc->has_iommu = false;
	rproc->auto_boot = of_property_read_bool(np, "cv180x,auto-boot");

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->regmap = regmap;
	priv->dcfg = dcfg;
	priv->dev = dev;

	dev_set_drvdata(dev, rproc);

	priv->workqueue = create_workqueue(dev_name(dev));
	if (!priv->workqueue) {
		dev_err(dev, "cannot create workqueue\n");
		ret = -ENOMEM;
		goto err_put_rproc;
	}

	ret = cv180x_rproc_xtr_mbox_init(rproc);
	if (ret) {
		dev_err(dev, "failed on cv180x_rproc_xtr_mbox_init\n");
		goto err_put_wkq;
	}

	INIT_WORK(&priv->rproc_work, cv180x_rproc_vq_work);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err_put_mbox;
	}

	dev_info(dev, "remoteproc probed\n");
	return 0;

err_put_mbox:
	cv180x_rproc_free_mbox(rproc);
err_put_wkq:
	destroy_workqueue(priv->workqueue);
err_put_rproc:
	rproc_free(rproc);

	return ret;
}

static int cv180x_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct cv180x_rproc *priv = rproc->priv;

	rproc_del(rproc);
	cv180x_rproc_free_mbox(rproc);
	destroy_workqueue(priv->workqueue);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id cv180x_rproc_match[] = {
	{ .compatible = "sophgo,cv180x-rproc", .data = &cv180x_rproc_cfg },
	{},
};
MODULE_DEVICE_TABLE(of, cv180x_rproc_match);

static struct platform_driver cv180x_rproc_driver = {
	.probe	= cv180x_rproc_probe,
	.remove = cv180x_rproc_remove,
	.driver = {
		.name = "cv180x-rproc",
		.of_match_table = cv180x_rproc_match,
	},
};

module_platform_driver(cv180x_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sophgo Cvitek Remote Processor Control Driver");
MODULE_AUTHOR("GP Orcullo <kinsamanka@gmail.com>");

