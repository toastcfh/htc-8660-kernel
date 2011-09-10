/* arch/arm/mach-msm/htc_simlock-8x60.c
 *
 * Copyright (C) 2011 HTC Corporation.
 * Author: Jon Tsai <jon_tsai@htc.com>
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


#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <mach/scm.h>

#define DEVICE_NAME "simlock"
#define SIMLOCK_MAJOR 230

#define SIMLOCK_GET_SET_CMD	0x2596
#define SIMLOCK_GET_SECURITY_LEVEL 0x2597

#define CODE_SIZE 32
static char code[CODE_SIZE];

static struct class *simlock_class;
static const struct file_operations simlock_fops;

struct msg_s {
	int size; 	/* msg size, return value */
	unsigned int unlock;
	char code[CODE_SIZE];
};

static int simlock_ioctl(struct inode *inode, struct file *file,
						unsigned int command, unsigned long arg)
{
	struct msg_s *msg_p = (struct msg_s *)arg;
	int size, ret;
	unsigned int unlock;

	switch (command) {
	case SIMLOCK_GET_SET_CMD:
		if (copy_from_user(&size, (void __user *)&msg_p->size, sizeof(int))) {
			printk(KERN_ERR "simlock_ioctl: copy_from_user error\n");
			return -EFAULT;
		}

		if (size > CODE_SIZE) {
			printk(KERN_ERR "simlock_ioctl: size error\n");
			return -EFAULT;
		}

		if (size > 0) {
			if (copy_from_user(&unlock, (void __user *)&msg_p->unlock, sizeof(unsigned int))) {
				printk(KERN_ERR "simlock_ioctl: copy_from_user error\n");
				return -EFAULT;
			}
			if (copy_from_user(code, (void __user *)&msg_p->code, size)) {
				printk(KERN_ERR "simlock_ioctl: copy_from_user error\n");
				return -EFAULT;
			}
			ret = secure_simlock_unlock(unlock, code);
		} else {
			ret = secure_read_simlock_mask();
		}
		if (copy_to_user(&msg_p->size, &ret, sizeof(int))) {
			printk(KERN_ERR "simlock_ioctl: copy_to_user error\n");
			return -EFAULT;
		}
		break;
	case SIMLOCK_GET_SECURITY_LEVEL:
		ret = secure_get_security_level();
		if (copy_to_user(&msg_p->size, &ret, sizeof(int))) {
			printk(KERN_ERR "simlock_ioctl: copy_to_user error\n");
			return -EFAULT;
		}
		break;
	default:
		printk(KERN_ERR "simlock_ioctl: command error\n");
		return -EFAULT;
	}
	return ret;
}

static const struct file_operations simlock_fops = {
	.ioctl  =   simlock_ioctl,
	.owner  =   THIS_MODULE,
};


static int __init simlock_init(void)
{
	int ret;

	ret = register_chrdev(SIMLOCK_MAJOR, DEVICE_NAME, &simlock_fops);
	if (ret < 0) {
		printk(KERN_ERR "simlock_init : register module fail\n");
		return ret;
	}

	simlock_class = class_create(THIS_MODULE, "simlock");
	device_create(simlock_class, NULL, MKDEV(SIMLOCK_MAJOR, 0), NULL, DEVICE_NAME);

	printk(KERN_INFO "simlock_init: register module ok\n");
	return 0;
}


static void  __exit simlock_exit(void)
{
	device_destroy(simlock_class, MKDEV(SIMLOCK_MAJOR, 0));
	class_unregister(simlock_class);
	class_destroy(simlock_class);
	unregister_chrdev(SIMLOCK_MAJOR, DEVICE_NAME);
	printk(KERN_INFO "simlock_exit: un-registered module ok\n");
}

module_init(simlock_init);
module_exit(simlock_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jon Tsai");

