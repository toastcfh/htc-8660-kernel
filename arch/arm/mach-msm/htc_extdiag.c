/* arch/arm/mach-msm/htc_extdiag.c - extended diag driver
 *
 * Copyright (C) 2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

//#define DEBUG

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#define RAMLOG_MAGIC1	0x1438aec9
#define RAMLOG_MAGIC2	0x9cea8341

#define EXTDIAG_IOCTL_PURGE 	0x01

struct htc_radio_ramlog
{
	u32 magic1;				//[MODEM] = HTC_RAMLOG_MAGIC1
	u32 magic2;				//[MODEM] = HTC_RAMLOG_MAGIC2
	u32 maskFileOffset;		//[MODEM] = HTC_RAMLOG_MASKFILE_OFFSET
	u32 maskFileSize;		//[APPS] file size of ”maskFileSetting.dat”
	u32 bufferOffset;		//[MODEM] = HTC_RAMLOG_BUFFER_OFFSET
	u32 bufferSize;			//[MODEM] = HTC_RAMLOG_BUFFER_SIZE
	u32 read_idx;			//[APPS] update read_idx after storing the log into files
	u32 write_idx;			//[MODEM] update write_idx after writing logs into RAM buffer
};

struct extdiag_dev {
	struct class *dev_class;
	struct platform_device *pdev;
	struct cdev cdev;
	struct mutex mutex;
	dev_t dev_number;

	int ref_count;
	u32 phys_size;
	phys_addr_t phys_base;
	void __iomem *mem_base;
	struct htc_radio_ramlog *ch;
};

static struct extdiag_dev *drv;

static int extdiag_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	mutex_lock(&drv->mutex);
	if (drv->ch->magic1!=RAMLOG_MAGIC1 ||
		drv->ch->magic2!=RAMLOG_MAGIC2) {
		ret = -ENODEV;
	} else if (drv->ref_count) {
		ret = -EBUSY;
	} else {
		file->private_data = drv;
		drv->ref_count++;
	}
	mutex_unlock(&drv->mutex);

	return ret;
}

static int extdiag_close(struct inode *inode, struct file *file)
{
	mutex_lock(&drv->mutex);
	drv->ref_count--;
	mutex_unlock(&drv->mutex);

	return 0;
}

static int extdiag_ioctl(struct inode *inode, struct file *file,
			   unsigned int iocmd, unsigned long ioarg)
{
	int ret = 0;

	mutex_lock(&drv->mutex);
	if (iocmd == EXTDIAG_IOCTL_PURGE) {
		drv->ch->read_idx = drv->ch->write_idx;
	}
	mutex_unlock(&drv->mutex);

	return ret;
}

static int extdiag_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	void *logbuf = drv->mem_base + drv->ch->bufferOffset;
	int head = drv->ch->write_idx, ret = 0;
	if (drv->ch->read_idx==head)
		return 0;

	if (!drv->ch->bufferSize)
		return -EINVAL;

	mutex_lock(&drv->mutex);

	if (drv->ch->read_idx > head) {
		*ppos = drv->ch->read_idx;
		ret = simple_read_from_buffer(buf, count, ppos, logbuf, drv->ch->bufferSize);
	}
	else if (drv->ch->read_idx < head) {
		*ppos = drv->ch->read_idx;
		ret = simple_read_from_buffer(buf, count, ppos, logbuf, head);
	}
	if (ret<0)
		goto exit;

	drv->ch->read_idx += ret;
	drv->ch->read_idx %= drv->ch->bufferSize;

exit:
	mutex_unlock(&drv->mutex);
	return ret;
}

static int extdiag_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	pr_debug("%s()\n", __func__);
	return -EPERM;
}

static ssize_t debug_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char buffer[1024];
	int len = 0, bufsz = sizeof(buffer);

	len += snprintf(buffer+len, bufsz, "(0x%08x 0x%08x)\n", drv->ch->magic1, drv->ch->magic2);
	len += snprintf(buffer+len, bufsz, "(  %8d   %8d)\n", drv->ch->maskFileOffset, drv->ch->maskFileSize);
	len += snprintf(buffer+len, bufsz, "(  %8d   %8d)\n", drv->ch->bufferOffset, drv->ch->bufferSize);
	len += snprintf(buffer+len, bufsz, "(  %8d   %8d)\n", drv->ch->read_idx, drv->ch->write_idx);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_fops = {
	.read = debug_read,
	.open = debug_open,
};

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = extdiag_read,
	.write = extdiag_write,
	.ioctl = extdiag_ioctl,
	.open = extdiag_open,
	.release = extdiag_close
};

static int __devinit htc_extdiag_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int err;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -1;
	}

	dev_dbg(dev, "res=%p start=0x%x end=0x%x, size=%x\n",
			res, res->start, res->end, res->end-res->start+1);

	drv = kzalloc(sizeof(struct extdiag_dev), GFP_KERNEL);
	drv->pdev = pdev;
	drv->phys_base = res->start;
	drv->phys_size = res->end - res->start + 1;
	drv->mem_base = ioremap(drv->phys_base, drv->phys_size);
	mutex_init(&drv->mutex);

	dev_dbg(dev, "Mapping %x -> 0x%x\n", drv->phys_base, (int)drv->mem_base);
	drv->ch = drv->mem_base;
	memset(drv->ch, 0, sizeof(struct htc_radio_ramlog));
	drv->ch->magic1 = RAMLOG_MAGIC2;
	drv->ch->magic2 = RAMLOG_MAGIC1;

	err = alloc_chrdev_region(&drv->dev_number, 0, 1, "extdiag");
	if (err<0) {
		dev_err(dev, "err = %d\n", err);
	}

	cdev_init(&drv->cdev, &fops);
	drv->cdev.owner = THIS_MODULE;

	err = cdev_add(&drv->cdev, drv->dev_number, 1);
	if (err) {
		dev_err(dev, "diagchar cdev registration failed !\n\n");
		return -1;
	}

	drv->dev_class = class_create(THIS_MODULE, "extdiag");
	if (IS_ERR(drv->dev_class)) {
		dev_err(dev, "Error creating diagchar class.\n");
		return -1;
	}
	device_create(drv->dev_class, NULL, drv->dev_number, (void *)drv, "extdiag");

#ifdef CONFIG_DEBUG_FS
	dent = debugfs_create_file("extdiag", 0444, NULL, NULL, &debug_fops);
	platform_set_drvdata(pdev, dent);
#endif

	dev_info(dev, "%s: OK\n", __func__);
	return 0;
}

static int __devexit htc_extdiag_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dentry *dent;

#ifdef CONFIG_DEBUG_FS
	dent = platform_get_drvdata(pdev);
	debugfs_remove(dent);
	platform_set_drvdata(pdev, NULL);
#endif

	cdev_del(&drv->cdev);

	device_destroy(drv->dev_class, drv->dev_number);
	class_destroy(drv->dev_class);

	unregister_chrdev_region(MAJOR(drv->dev_number), 1);

	iounmap(drv->mem_base);
	kzfree(drv);

	dev_info(dev, "%s: OK\n", __func__);
	return 0;
}

static struct platform_driver htc_extdiag_driver = {
	.probe		= htc_extdiag_probe,
	.remove		= __devexit_p(htc_extdiag_remove),
	.driver		= {
		.name = "htc_extdiag",
		.owner = THIS_MODULE,
	},
};

static int __init htc_extdiag_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&htc_extdiag_driver);
}

static void __exit htc_extdiag_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&htc_extdiag_driver);
}

module_init(htc_extdiag_init);
module_exit(htc_extdiag_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HTC Extended Diag driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:htc_extdiag");
