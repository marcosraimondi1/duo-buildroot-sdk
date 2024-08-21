// SPDX-License-Identifier: GPL-2.0+
/*
 * Mailbox driver for Cevitek chips.
 *
 * Author: GP Orcullo <kinsamanka@gmail.com>
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define DRIVER_NAME			"cvitek-mbox"

#define NUM_SHARED_MB			2
#define NUM_DOORBELL			6
#define NUM_CHANS			(NUM_SHARED_MB + NUM_DOORBELL)

#define MAILBOX_ID_CPU1			1	/* c906B */
#define MAILBOX_ID_CPU2			2	/* c906L */

#define CVI_SPINLOCK_TIMEOUT		10000
#define CVI_SPINLOCK_WAIT_USEC		1
#define CVI_SPINLOCK_IDX		0
#define CVI_SPINLOCK_VAL 		(MAILBOX_ID_CPU1 << 4)

#define CPU_ENABLE(i)			(0x0  + ((i) << 2))
#define CPU_INT_CLEAR(i)		(0x10 + ((i) << 4))
#define CPU_INT_MASK(i)			(0x14 + ((i) << 4))
#define CPU_INT_REQUEST(i)		(0x18 + ((i) << 4))
#define MBOX_SET 			0x60
#define HW_MUTEX			(0xC0 + (CVI_SPINLOCK_IDX << 2))
#define MBOX_BUFFER(i,j)		(0x400 + ((i) << 3) + ((j) << 2))


struct cv180x_chan {
	struct cv180x_mbox	*mbox;
	int			idx;
	u8			mask;
};

struct cv180x_mbox {
	struct mbox_controller	controller;
	void __iomem		*base;
};

static int cv180x_get_hw_mutex(void *base, u8 mask)
{
	ktime_t timeout = ktime_add_us(ktime_get(), CVI_SPINLOCK_TIMEOUT);
	writeb(mask, base + HW_MUTEX);

	for (;;) {
		if (mask == readb(base + HW_MUTEX))
			return 0;

		if (ktime_compare(ktime_get(), timeout) > 0)
			return -ETIMEDOUT;

		udelay(CVI_SPINLOCK_WAIT_USEC);
		writeb(mask, base + HW_MUTEX);
	}

	return 0;
}

static inline void cv180x_release_hw_mutex(void *base, u8 mask)
{
	writeb(mask, base + HW_MUTEX);
}

static irqreturn_t cv180x_mbox_irq_handler(int irq, void *data)
{
	struct cv180x_mbox *mbox = data;
	struct device *dev = mbox->controller.dev;
	const int id = MAILBOX_ID_CPU1;
	u8 mask = CVI_SPINLOCK_VAL;
	u32 buf[NUM_CHANS];
	u32 reg, tmp;
	int n;

	if (cv180x_get_hw_mutex(mbox->base, mask)) {
		dev_err(dev, "Unable to secure hw mutex\n");
		return IRQ_HANDLED;
	}

	reg = readl(mbox->base + CPU_INT_REQUEST(id));

	if (!reg) {
		cv180x_release_hw_mutex(mbox->base, mask);
		dev_err(dev, "Spurious mbox interrupt\n");
		return IRQ_HANDLED;
	}

	for (n=0; n < NUM_CHANS; n++) {
		if (reg & BIT(n)) {
			buf[n] = -1;
			if (n < NUM_SHARED_MB)
				buf[n] = readl(mbox->base + MBOX_BUFFER(id,n));

			dev_dbg(dev, "Channel %d received 0x%08x", n, buf[n]);
		}
	}

	/* acknowledge request */
	writel(reg, mbox->base + CPU_INT_CLEAR(id));

	/* clear cpu request */
	tmp = ~reg & readl(mbox->base + CPU_ENABLE(id));
	writel(tmp, mbox->base + CPU_ENABLE(id));

	cv180x_release_hw_mutex(mbox->base, mask);

	for (n=0; n < NUM_CHANS; n++)
		if (reg & BIT(n))
			mbox_chan_received_data(&mbox->controller.chans[n], &buf[n]);

	return IRQ_HANDLED;
}

static int cv180x_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct cv180x_chan *mchan = (struct cv180x_chan *)chan->con_priv;
	struct cv180x_mbox *mbox = mchan->mbox;
	struct device *dev = mbox->controller.dev;
	const int id = MAILBOX_ID_CPU2;
	int idx = mchan->idx;
	u8 mask = mchan->mask;
	u32 val = 0;

	if (!data && idx < NUM_SHARED_MB)
	{
		dev_err(dev, "No data to send on channel %d\n", idx);
		return -EINVAL;
	}

	if (data)
		val = *(u32 *)data;

	dev_dbg(dev, "Sending 0x%08x on channel %d", val, idx);

	if (cv180x_get_hw_mutex(mbox->base, mask)) {
		dev_err(dev, "Unable to secure hw mutex\n");
		return -EBUSY;
	}

	if (idx < NUM_SHARED_MB)
		writel(val, mbox->base + MBOX_BUFFER(id, idx));

	/* clear & set request */
	writel(BIT(idx), mbox->base + CPU_INT_CLEAR(id));
	__set_bit(idx, mbox->base + CPU_ENABLE(id));
	/* trigger interrupt */
	writel(BIT(idx), mbox->base + MBOX_SET);

	cv180x_release_hw_mutex(mbox->base, mask);

	return 0;
}

static int cv180x_mbox_startup(struct mbox_chan *chan)
{
	struct cv180x_chan *mchan = (struct cv180x_chan *)chan->con_priv;
	struct cv180x_mbox *mbox = mchan->mbox;
	struct device *dev = mbox->controller.dev;
	const int id = MAILBOX_ID_CPU1;
	int idx = mchan->idx;
	u8 mask = mchan->mask;

	if (cv180x_get_hw_mutex(mbox->base, mask)) {
		dev_err(dev, "Unable to secure hw mutex\n");
		return -EBUSY;
	}

	__clear_bit(idx, mbox->base + CPU_INT_MASK(id));

	cv180x_release_hw_mutex(mbox->base, mask);

	return 0;
}

static void cv180x_mbox_shutdown(struct mbox_chan *chan)
{
	struct cv180x_chan *mchan = (struct cv180x_chan *)chan->con_priv;
	struct cv180x_mbox *mbox = mchan->mbox;
	struct device *dev = mbox->controller.dev;
	const int id = MAILBOX_ID_CPU1;
	int idx = mchan->idx;
	u8 mask = mchan->mask;

	if (cv180x_get_hw_mutex(mbox->base, mask)) {
		dev_err(dev, "Unable to secure hw mutex\n");
		return;
	}

	__set_bit(idx, mbox->base + CPU_INT_MASK(id));

	cv180x_release_hw_mutex(mbox->base, mask);
}

static bool cv180x_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct cv180x_chan *mchan = (struct cv180x_chan *)chan->con_priv;
	struct cv180x_mbox *mbox = mchan->mbox;
	const int id = MAILBOX_ID_CPU2;

	return !test_bit(mchan->idx, mbox->base + CPU_INT_REQUEST(id));
}

static const struct mbox_chan_ops cv180x_mbox_ops = {
	.send_data	= cv180x_mbox_send_data,
	.startup	= cv180x_mbox_startup,
	.shutdown	= cv180x_mbox_shutdown,
	.last_tx_done	= cv180x_mbox_last_tx_done,
};

static int cv180x_mbox_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Probing mailbox\n");
	struct cv180x_mbox *mbox;
	struct cv180x_chan *chans;
	int irq;
	int ret;
	int n;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
	{
		dev_err(&pdev->dev, "Cannot allocate mailbox\n");
		return -ENOMEM;
	}

	chans = devm_kcalloc(&pdev->dev, NUM_CHANS, sizeof(*chans), GFP_KERNEL);
	if (!chans)
	{
		dev_err(&pdev->dev, "Cannot allocate channels\n");
		return -ENOMEM;
	}

	mbox->controller.chans = devm_kcalloc(&pdev->dev, NUM_CHANS,
					      sizeof(*mbox->controller.chans),
					      GFP_KERNEL);
	if (!mbox->controller.chans)
	{
		dev_err(&pdev->dev, "Cannot allocate controller channels\n");
		return -ENOMEM;
	}

	mbox->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mbox->base))
	{
		dev_err(&pdev->dev, "Cannot remap resource\n");
		return PTR_ERR(mbox->base);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
	{
		dev_err(&pdev->dev, "Cannot get irq\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, cv180x_mbox_irq_handler, 0,
			       DRIVER_NAME, mbox);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot request irq\n");
		return ret;
	}

	/* mask all channels */
	writel(0xff, mbox->base + CPU_INT_MASK(MAILBOX_ID_CPU1));

	for (n=0; n < NUM_CHANS; ++n) {
		chans[n].idx = n;
		chans[n].mask = (n + 1) | CVI_SPINLOCK_VAL;
		chans[n].mbox = mbox;
		mbox->controller.chans[n].con_priv = &chans[n];
	}

	mbox->controller.dev		= &pdev->dev;
	mbox->controller.num_chans	= NUM_CHANS;
	mbox->controller.ops		= &cv180x_mbox_ops;
	mbox->controller.txdone_irq	= false;
	mbox->controller.txdone_poll	= true;
	mbox->controller.txpoll_period	= 5;

	ret = devm_mbox_controller_register(&pdev->dev, &mbox->controller);
	if (ret) {
		dev_err(&pdev->dev, "Could not register mailbox controller\n");
		return ret;
	}

	platform_set_drvdata(pdev, mbox);
	dev_info(&pdev->dev, "Mailbox controller registered\n");
	return ret;
}


static const struct of_device_id cv180x_mbox_match[] = {
	{ .compatible = "cvitek,mbox" },
	{ },
};

MODULE_DEVICE_TABLE(of, cv180x_mbox_match);

static struct platform_driver cv180x_mbox_driver = {
	.probe	= cv180x_mbox_probe,
	.driver	= {
		.name		= DRIVER_NAME,
		.of_match_table	= cv180x_mbox_match,
	},
};

module_platform_driver(cv180x_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sophgo cvitek mailbox driver");
MODULE_AUTHOR("GP Orcullo <kinsamanka@gmail.com>");
