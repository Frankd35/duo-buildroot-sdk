/**
 * License -- FIXME
*/

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

union cvi_mailbox_info_offset{
	u8 mbox_info;
	u32 reserved;
};

union cvi_mailbox_int_clr_offset{
	u8 mbox_int_clr;
	u32 reserved;
};
union cvi_mailbox_int_mask_offset{
	u8 mbox_int_mask;
	u32 reserved;
};
union cvi_mailbox_int_offset{
	u8 mbox_int;
	u32 reserved;
};
union cvi_mailbox_int_raw_offset{
	u8 mbox_int_raw;
	u32 reserved;
};

union mailbox_set{
	u8 mbox_set;
	u32 reserved;
};
union mailbox_status{
	u8 mbox_status;
	u32 reserved;
};

union cvi_mailbox_status{
	u8 mbox_status;
	u32 reserved;
};

/* register mapping refers to mailbox user guide*/
struct cpu_mbox_int {
	union cvi_mailbox_int_clr_offset  cpu_mbox_int_clr;
	union cvi_mailbox_int_mask_offset cpu_mbox_int_mask;
	union cvi_mailbox_int_offset      cpu_mbox_int_int;
	union cvi_mailbox_int_raw_offset  cpu_mbox_int_raw;
};

struct mailbox_set_register {
	union  cvi_mailbox_info_offset cpu_mbox_en[4];      //0x00, 0x04, 0x08, 0x0c
	struct cpu_mbox_int cpu_mbox_set[4];                //0x10~0x1C, 0x20~0x2C, 0x30~0x3C, 0x40~0x4C
	u32    reserved[4];                                 //0x50~0x5C
	union  mailbox_set mbox_set;                        //0x60
	union  mailbox_status mbox_status;                  //0x64
	u32    reserved2[2];                                //0x68~0x6C
	union  cvi_mailbox_status cpu_mbox_status[4];       //0x70
};

struct mailbox_done_register {
	union  cvi_mailbox_info_offset cpu_mbox_done_en[4];
	struct cpu_mbox_int cpu_mbox_done[4];
};

#define MAILBOX_MAX_CHAN		0x0008
#define MAILBOX_DONE_OFFSET     0x0002
#define MAILBOX_CONTEXT_SIZE	0x0040
#define MAILBOX_CONTEXT_OFFSET  0x0400

#define SEND_TO_CPU  1
#define RECEIVE_CPU  1 // c906B


struct cv1800b_mbox {
	struct mbox_controller mbox;
	void __iomem *mbox_base;

	struct mbox_chan chans[MAILBOX_MAX_CHAN];
	u64 *content[MAILBOX_MAX_CHAN];

};

static irqreturn_t cv1800b_mbox_isr(int irq, void *dev_id)
{
	size_t i;
	struct cv1800b_mbox *mb = (struct cv1800b_mbox *)dev_id;
	struct mailbox_done_register *mbox_done_reg = 
		(struct mailbox_done_register*)(mb->mbox_base + MAILBOX_DONE_OFFSET);
	
	printk("cv1800b_mbox_isr:%d\n", irq);

	for (i = 0; i < MAILBOX_MAX_CHAN; i++) {
		if (mb->content[i]) {
			printk("cv1800b_mbox_isr, content: %ld\n", i);
			// [TO OPEN] mbox_chan_received_data(&mb->chans[i], mb->content[i]);

			// for test
			*mb->content[i] = *mb->content[i]+0xff;

			mb->content[i] = NULL;
			//
			mbox_done_reg->cpu_mbox_done_en[RECEIVE_CPU].mbox_info |= (short)(1<<i);
			return IRQ_HANDLED;
		}
	}
	printk("cv1800b_mbox_isr: invalid interupt, idx not found\n");
	return IRQ_NONE;
}

static irqreturn_t cv1800b_mbox_irq(int irq, void *dev_id)
{
	struct cv1800b_mbox *mb = (struct cv1800b_mbox *)dev_id;
	struct mailbox_set_register *mbox_reg = mb->mbox_base;
	struct mailbox_done_register *mbox_done_reg = 
		(struct mailbox_done_register*)(mb->mbox_base + MAILBOX_DONE_OFFSET);
	u8 set, done, valid;
	int i;
	u64 *buf;

	printk("cv1800b_mbox_irq:%d\n",irq);

	set = mbox_reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_int.mbox_int;
	done = mbox_done_reg->cpu_mbox_done[RECEIVE_CPU].cpu_mbox_int_int.mbox_int;
	printk("set: %d, done: %d\n", set, done);

	if (set) {
		for (i = 0; i < MAILBOX_MAX_CHAN; i++) {
			valid = set & (1 << i);
			if (valid) {
				buf = (u64*)(mb->mbox_base + MAILBOX_CONTEXT_OFFSET) + i;
				printk("mailbox: %d, value: 0x%llX\n", i, *buf);
				mb->content[i] = buf;
				/* mailbox buffer context is send from rtos, clear mailbox interrupt */
				mbox_reg->cpu_mbox_set[RECEIVE_CPU].cpu_mbox_int_clr.mbox_int_clr = valid;
				// need to disable enable bit
				mbox_reg->cpu_mbox_en[RECEIVE_CPU].mbox_info &= ~valid;

				return IRQ_WAKE_THREAD;
			}
		}
		
	} else if (done) {
		for (i = 0; i < MAILBOX_MAX_CHAN; i++) {
			valid = done & (1 << i);
			if (done & (1 << i)) {
				printk("mailbox: %d tx done\n", i);
				// clr interrupt
				mbox_done_reg->cpu_mbox_done[RECEIVE_CPU].cpu_mbox_int_clr.mbox_int_clr = valid;
				// need to disable enable bit
				mbox_done_reg->cpu_mbox_done_en[RECEIVE_CPU].mbox_info &= ~valid;
				// callback for tx done here?
				return IRQ_HANDLED;
			}
		}
	}

	printk("cv1800b_mbox_irq: invalid interupt, idx not found\n");
	return IRQ_NONE;
}

static int cv1800b_mbox_send_data(struct mbox_chan *chan, void *data) {
	struct cv1800b_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct mailbox_set_register *mbox_reg = mbox->mbox_base;
	int idx = (int)chan->con_priv;
	u64 *buf = (u64*)(mbox->mbox_base + MAILBOX_CONTEXT_OFFSET) + idx;
	dev_info(chan->mbox->dev, "cv1800b_mbox_send_data, chan: %d, msg:%llX\n", idx, *((u64*)data));

	// write data
	*buf = *((u64*)data);
	// clear mailbox
	mbox_reg->cpu_mbox_set[SEND_TO_CPU].cpu_mbox_int_clr.mbox_int_clr = (1 << idx);
	// trigger mailbox valid to rtos
	mbox_reg->cpu_mbox_en[SEND_TO_CPU].mbox_info |= (1 << idx);
	mbox_reg->mbox_set.mbox_set = (1 << idx);
	return 0;
}


static const struct mbox_chan_ops cv1800b_mbox_chan_ops = {
	.send_data	= cv1800b_mbox_send_data,
	.startup	= NULL,
	.shutdown	= NULL,
};

static const struct of_device_id cv1800b_mbox_of_match[] = {
	{ .compatible = "cvitek,rtos_cmdqu", },
	{},
};
MODULE_DEVICE_TABLE(of, cv1800b_mbox_of_match);


static int cv1800b_mbox_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct cv1800b_mbox* mb;
	struct resource *res;
	int irq, idx, err;

	printk("cv1800b_mbox_probe\n");	//FIXME
	if (!pdev)
		return -ENODEV;

	mb = devm_kzalloc(dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	platform_set_drvdata(pdev, mb);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_info(dev,"res.start: %llX\n", res->start);
	if (!res)
		return -ENODEV;

	mb->mbox_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mb->mbox_base))
		return PTR_ERR(mb->mbox_base);

	mb->mbox.dev = dev;
	mb->mbox.num_chans = MAILBOX_MAX_CHAN;
	mb->mbox.chans = mb->chans;
	mb->mbox.ops = &cv1800b_mbox_chan_ops;
	mb->mbox.txdone_irq = true;

	irq = platform_get_irq_byname(pdev, "mailbox");
	dev_info(dev,"irq: %d\n", irq);
	err = devm_request_threaded_irq(dev, irq,
						cv1800b_mbox_irq,
						cv1800b_mbox_isr, IRQF_ONESHOT,
						dev_name(&pdev->dev), mb);
	if (err < 0) {
		dev_err(dev, "Failed to register irq %d\n", err);
		return err;
	}

	for (idx = 0; idx < MAILBOX_MAX_CHAN; idx++)
		mb->mbox.chans[idx].con_priv = (void*)idx;

	err = devm_mbox_controller_register(dev, &mb->mbox);
	if (err) {
		dev_err(dev, "Failed to register mailbox %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, mb);
	dev_info(dev, "Mailbox enabled\n");
	return 0;
}

static struct platform_driver cv1800b_mbox_driver = {
	.driver = {
		.name = "cv1800b-mbox",
		.of_match_table = cv1800b_mbox_of_match,
	},
	.probe	= cv1800b_mbox_probe,
};

static int __init cv1800b_mbox_init(void)
{
	printk("cv1800b_mbox_init -- 2023-11-13\n");
	return platform_driver_register(&cv1800b_mbox_driver);
}
core_initcall(cv1800b_mbox_init);

static void __exit cv1800b_mbox_exit(void)
{
	platform_driver_unregister(&cv1800b_mbox_driver);
}
module_exit(cv1800b_mbox_exit);

MODULE_DESCRIPTION("cv1800b mailbox driver");
MODULE_LICENSE("GPL v2");