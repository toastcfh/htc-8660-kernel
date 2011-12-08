/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/clkdev.h>
#include <mach/mdm.h>
#include <mach/restart.h>
#include <mach/subsystem_notif.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <linux/clk.h>
#include "mdm_ioctls.h"
#include "../msm_watchdog.h"
#include "../devices.h"
#include "../clock.h"
#include <mach/oem_rapi_client.h>
#include <mach/scm.h>
#include <linux/syscalls.h> /* sys_sync */

/* Modified by HTC according to radio guys' request */
#define CHARM_MODEM_TIMEOUT	6000
#define CHARM_HOLD_TIME		4000
#define CHARM_MODEM_ERROR_RESTART_WAITING	3000
#define CHARM_MEDEM_AVOID_SIGNAL_NOISE		0
/*-----------------------------------*/
#define CHARM_MODEM_DELTA	100

static void (*power_on_charm)(void);
static void (*power_down_charm)(void);
static void (*reset_charm)(void);	/* Added by HTC */
static void (*suspend_charm)(void);    /* Added by HTC */
static void (*resume_charm)(void);     /* Added by HTC */

static int charm_debug_on;
static int charm_status_irq;
static int charm_errfatal_irq;

#define CHARM_DBG(...)	do { if (charm_debug_on) \
					pr_info(__VA_ARGS__); \
			} while (0);

/* Added and Modified by HTC */
static unsigned AP2MDM_STATUS  = 0;
static unsigned AP2MDM_WAKEUP = 0;
static unsigned AP2MDM_ERRFATAL = 0;
/*static unsigned AP2MDM_SYNC = 0;*/
static unsigned AP2MDM_PMIC_RESET_N = 0;
static unsigned AP2MDM_KPDPWR_N = 0;
static unsigned AP2PMIC_TMPNI_CKEN = 0;

static unsigned MDM2AP_STATUS = 0;
static unsigned MDM2AP_WAKEUP = 0;
static unsigned MDM2AP_ERRFATAL = 0;
static unsigned MDM2AP_SYNC = 0;
static unsigned MDM2AP_VFR = 0;

static unsigned charm_MDM_error_flag = 0;

unsigned charm_get_MDM_error_flag(void)
{
	return charm_MDM_error_flag;
}

static struct delayed_work charm_status_work;
static struct delayed_work charm_fatal_work;

static char mdm9k_serial[9];
module_param_string(debug, mdm9k_serial, sizeof(mdm9k_serial), 0644);

void charm_panic_notify(void)
{
	CHARM_DBG("%s: setting AP2MDM_ERRFATAL high for a non graceful reset\n",
			 __func__);
	disable_irq_nosync(charm_errfatal_irq);
	disable_irq_nosync(charm_status_irq);
	gpio_set_value(AP2MDM_ERRFATAL, 1);
}

void charm_panic_wait_mdm_shutdown(void)
{
	int i;

	CHARM_DBG("%s: waiting MDM2AP_ERRFATAL high for MDM/Q6 shutdown\n", __func__);

	for (i = 0; i < CHARM_MODEM_TIMEOUT; i += CHARM_MODEM_DELTA) {
		pet_watchdog();
		mdelay(CHARM_MODEM_DELTA);
		if (gpio_get_value(MDM2AP_ERRFATAL) == 1)
			break;
	}

	if (i >= CHARM_MODEM_TIMEOUT) {
		pr_err("%s: MDM2AP_ERRFATAL never went high in %d(ms)!\n", __func__, CHARM_MODEM_TIMEOUT);
		gpio_direction_output(AP2MDM_PMIC_RESET_N, 0);
	}
}
/*------------------------------------------*/

static int charm_panic_prep(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	charm_panic_notify();	/* Modified by HTC */
	return NOTIFY_DONE;
}

static struct notifier_block charm_panic_blk = {
	.notifier_call  = charm_panic_prep,
};

static int successful_boot;


struct encrypt_ramdump_parm {
    int len;
    int rev[3];
};

struct encrypt_ramdump_tz_buf {
    unsigned char *dst;
    unsigned char *src;
    int len;
};

static int simlock_encrypt_ramdump(unsigned char *dst, unsigned char *src, int len)
{
    struct encrypt_ramdump_tz_buf buf;

	if (dst != NULL)
	    buf.dst = (unsigned char *)virt_to_phys(dst);
	else
		buf.dst = NULL;
	if (src != NULL)
	    buf.src = (unsigned char *)virt_to_phys(src);
	else
		buf.src = NULL;
    buf.len = len;

    return secure_access_item(0, ITEM_ENCRYPT_RAMDUMP, sizeof(buf), (char *)&buf);
}

struct encrypt_ramdump_parm enc_parm;

static long charm_modem_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{

	int ret = 0;

	if (_IOC_TYPE(cmd) != CHARM_CODE) {
		pr_err("%s: invalid ioctl code\n", __func__);
		return -EINVAL;
	}

	gpio_request(MDM2AP_STATUS, "MDM2AP_STATUS");
	gpio_direction_input(MDM2AP_STATUS);

	CHARM_DBG("%s: Entering ioctl cmd = %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case WAKE_CHARM:
		/* turn the charm on */
		if (power_on_charm)
			power_on_charm();
		break;

	case RESET_CHARM:
		/* put the charm back in reset */
		if (reset_charm)
			reset_charm();	/* Modified by HTC */
		break;

	case CHECK_FOR_BOOT:
		if (gpio_get_value(MDM2AP_STATUS) == 0) {
			put_user(1, (unsigned long __user *) arg);
			pr_info("%s: failure_boot = 0. \n", __func__);
		} else {
			put_user(0, (unsigned long __user *) arg);
			if (!successful_boot) {
				successful_boot = 1;
				pr_info("%s: sucessful_boot = 1. Monitoring \
					for mdm interrupts\n",
					__func__);
			}
		}
		break;

	case WAIT_FOR_BOOT:
		/* wait for status to be high */
		while (gpio_get_value(MDM2AP_STATUS) == 0)
			;
		break;

	case RELEASE_CHARM:		/* Added by HTC */
		/* release the charm KPDPWR pin */
		if (power_down_charm)
			power_down_charm();
		break;

	case GET_CIPHER_RAMDUMP_MODE:
		{
			int mode = 0;
			ret = secure_access_item(0, ITEM_CRYPTO_RAMDUMP, sizeof(int), (char *)&mode);
			if (copy_to_user((char *)arg, &mode, sizeof(int))) {
				printk(KERN_ERR "mdm_ioctl: copy_to_user error\n");
				return -EFAULT;
			}
		}
		break;
	case SET_ENCRYPT_BUFFER_PARM:
		if (copy_from_user((char *)&enc_parm, (void __user *)arg, sizeof(enc_parm))) {
				printk(KERN_ERR "mdm_ioctl: copy_from_user error\n");
				return -EFAULT;
		}
		ret = 0;
		if (enc_parm.len == 0) {
			ret = simlock_encrypt_ramdump(NULL, NULL, 0);
			if (ret) {
				printk(KERN_ERR "mdm_ioctl: secure_access_item error\n");
				return -EFAULT;
			}
		}
		break;
	case ENCRYPT_BUFFER:
		{
			unsigned char *buf, *enc;
			int len, len16;

			enc = (unsigned char *)arg;
			len = enc_parm.len;

			len16 = (len + 0xF) & ~0xF;
			if (len16 > (32 * 1024)) {
				printk(KERN_ERR "mdm_ioctl: buffer size error\n");
				return -EFAULT;
			}

			buf = kmalloc(len16, GFP_KERNEL);
			memset(buf, 0, len16);
			if (copy_from_user(buf, (void __user *)enc, len)) {
				printk(KERN_ERR "mdm_ioctl: copy_from_user error\n");
				kfree(buf);
				return -EFAULT;
			}

			ret = simlock_encrypt_ramdump(buf, buf, len16);
			if (ret) {
				printk(KERN_ERR "mdm_ioctl: secure_access_item error\n");
				kfree(buf);
				return -EFAULT;
			}

			if (copy_to_user(enc, buf, len)) {
				printk(KERN_ERR "mdm_ioctl: copy_to_user error\n");
				return -EFAULT;
			}
			kfree(buf);
		}
		break;




	default:
		ret = -EINVAL;
	}

	gpio_free(MDM2AP_STATUS);

	return ret;
}

static int charm_modem_open(struct inode *inode, struct file *file)
{

	CHARM_DBG("%s: successful_boot = 0\n", __func__);
	successful_boot = 0;
	return 0;
}

static const struct file_operations charm_modem_fops = {
	.owner		= THIS_MODULE,
	.open		= charm_modem_open,
	.unlocked_ioctl	= charm_modem_ioctl,
};

struct miscdevice charm_modem_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdm",
	.fops	= &charm_modem_fops
};

static void charm_dump_GPIO(char *name, unsigned int gpio)
{
	if (gpio == 0)
		return;

	pr_info("%s: %s = %d\n", __func__, name, gpio_get_value(gpio));
}

static void charm_dump_mdm_related_GPIO_status(void)
{
	charm_dump_GPIO("AP2MDM_STATUS", AP2MDM_STATUS);
	charm_dump_GPIO("AP2MDM_WAKEUP", AP2MDM_WAKEUP);
	charm_dump_GPIO("AP2MDM_ERRFATAL", AP2MDM_ERRFATAL);
	/*charm_dump_GPIO("AP2MDM_SYNC", AP2MDM_SYNC);*/
	charm_dump_GPIO("AP2MDM_PMIC_RESET_N", AP2MDM_PMIC_RESET_N);
	charm_dump_GPIO("AP2MDM_KPDPWR_N", AP2MDM_KPDPWR_N);
	charm_dump_GPIO("AP2PMIC_TMPNI_CKEN", AP2PMIC_TMPNI_CKEN);

	charm_dump_GPIO("MDM2AP_STATUS", MDM2AP_STATUS);
	charm_dump_GPIO("MDM2AP_WAKEUP", MDM2AP_WAKEUP);
	charm_dump_GPIO("MDM2AP_ERRFATAL", MDM2AP_ERRFATAL);
	charm_dump_GPIO("MDM2AP_SYNC", MDM2AP_SYNC);
	charm_dump_GPIO("MDM2AP_VFR", MDM2AP_VFR);
}

static void charm_status_fn(struct work_struct *work)
{
	int i;
	if (gpio_get_value(MDM2AP_STATUS) == 0) {	/* Modified by HTC to avoid signal noise */
		if (!charm_MDM_error_flag) {
			charm_MDM_error_flag = 1;
			pr_info("%s: Status went low!\n", __func__);
			charm_dump_mdm_related_GPIO_status();

			for (i = 0; i < CHARM_MODEM_ERROR_RESTART_WAITING; i += CHARM_MODEM_DELTA) {
				pet_watchdog();
				msleep(CHARM_MODEM_DELTA);
			}

			soc_restart(RESTART_MODE_MDM_DOG_BITE, "MDM DOG!");
		} else {
			pr_info("%s: Charm status went low comes after other MDM error signal!(Ignore it...)\n", __func__);
		}
	} else {
		pr_info("%s: Charm status went low seems a false alarm!\n", __func__);
	}
}

static void charm_fatal_fn(struct work_struct *work)
{
	int i;
	if (gpio_get_value(MDM2AP_ERRFATAL) == 1) {	/* Modified by HTC to avoid signal noise */
		if (!charm_MDM_error_flag) {
			charm_MDM_error_flag = 1;
			pr_info("%s: Got an error fatal!\n", __func__);
			charm_dump_mdm_related_GPIO_status();

			for (i = 0; i < CHARM_MODEM_ERROR_RESTART_WAITING; i += CHARM_MODEM_DELTA) {
				pet_watchdog();
				msleep(CHARM_MODEM_DELTA);
			}

			soc_restart(RESTART_MODE_MDM_FATAL, "MDM FATAL!");
		} else {
			pr_info("%s: Charm got errfatal interrupt comes after other MDM error signal!(Ignore it...)\n", __func__);
		}
	} else {
		pr_info("%s: Charm got errfatal interrupt seems a false alarm\n", __func__);
	}
}

static int mdm9k_serial_restart = 0;
static struct msm_rpc_client *rpc_client = NULL;
#define OEM_RAPI_CLIENT_EVENT_SSD_MDM_SERIAL_NUM 2003

#if defined(CONFIG_MACH_VERDI_LTE) || defined(CONFIG_MACH_RUBY) || defined(CONFIG_MACH_HOLIDAY)
#define OEM_RAPI_CLIENT_EVENT_SSD_DISABLE_WATCHDOG_SMPLD 2004
#endif

int mdm_check_bootmode_init(char *s)
{
	if (!strcmp(s, "normal") || !strcmp(s, "factory2"))
		mdm9k_serial_restart = 1;

	return 1;
}

static int get_mdm9k_serial(unsigned int *mdm9k_serial)
{
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	int err, ret_len = 4;
	int val = 0;
	char input = 0;

	if ((rpc_client == NULL) || IS_ERR(rpc_client)) {
		printk(KERN_INFO "%s: try to open oem rapi client\n", __func__);
		rpc_client = oem_rapi_client_init();
		printk(KERN_INFO "%s: open oem rapi client => %p\n", __func__, rpc_client);
	}
	if (IS_ERR(rpc_client))
		printk(KERN_INFO "%s: couldn't open oem rapi client\n", __func__);
	else {
		arg.event = OEM_RAPI_CLIENT_EVENT_SSD_MDM_SERIAL_NUM;
		arg.cb_func = NULL;
		arg.handle = (void *)0;
		arg.in_len = 1;
		arg.input = &input;

		arg.out_len_valid = 1;
		arg.output_valid = 1;
		arg.output_size = 4;
		ret.out_len = &ret_len;
		ret.output = NULL;

		err = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
		if (err)
			printk(KERN_INFO "%s: Send data from modem failed: %d\n", __func__, err);
		else if (ret_len == 4) {
			memcpy(mdm9k_serial, ret.output, 4);
			val = 1;
		} else
			printk(KERN_INFO "%s: return data from modem failed: %d\n", __func__, ret_len);
	}

	return val;
}

void check_mdm9k_serial(void)
{
	int val;
	unsigned int mdm9kserial;

	val = secure_get_security_level();
	if (val < 2) {
		if (get_mdm9k_serial(&mdm9kserial))
			printk(KERN_INFO "mdm9k serial number = 0x%08X\n", mdm9kserial);
		sprintf(mdm9k_serial, "%08X", mdm9kserial);
	}
}

static irqreturn_t charm_errfatal(int irq, void *dev_id)
{
	pr_info("%s: charm got errfatal interrupt\n", __func__);
	if (successful_boot) {
		pr_info("%s: scheduling work now\n", __func__);
		schedule_delayed_work(&charm_fatal_work, msecs_to_jiffies(CHARM_MEDEM_AVOID_SIGNAL_NOISE));	/* Modified by HTC to avoid signal noise */
		disable_irq_nosync(charm_errfatal_irq);
	}
	return IRQ_HANDLED;
}

static irqreturn_t charm_status_change(int irq, void *dev_id)
{
	pr_info("%s: Charm status went low!\n", __func__);
	if (successful_boot) {
		pr_info("%s: scheduling work now\n", __func__);
		schedule_delayed_work(&charm_status_work, msecs_to_jiffies(CHARM_MEDEM_AVOID_SIGNAL_NOISE));	/* Modified by HTC to avoid signal noise */
		disable_irq_nosync(charm_status_irq);
	}
	return IRQ_HANDLED;
}

static int charm_debug_on_set(void *data, u64 val)
{
	charm_debug_on = val;
	return 0;
}

static int charm_debug_on_get(void *data, u64 *val)
{
	*val = charm_debug_on;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(charm_debug_on_fops,
			charm_debug_on_get,
			charm_debug_on_set, "%llu\n");

static int charm_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("charm_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("debug_on", 0644, dent, NULL,
			&charm_debug_on_fops);
	return 0;
}

static int gsbi9_uart_notifier_cb(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	struct clk *uart_clk;

	switch (code) {
	case SUBSYS_AFTER_SHUTDOWN:
		uart_clk = clk_get(&msm_device_uart_gsbi9.dev, "gsbi_uart_clk");
		clk_set_rate(uart_clk, 0);
		clk_set_rate(uart_clk, 1843200);
		clk_put(uart_clk);
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block gsbi9_nb = {
	.notifier_call = gsbi9_uart_notifier_cb,
};

#ifdef CONFIG_PM
static int charm_modem_suspend(struct device *dev)
{
	if (suspend_charm)
		suspend_charm();
	return 0;
}

static int charm_modem_resume(struct device *dev)
{
	if (resume_charm)
		resume_charm();
	return 0;
}
#else
#define charm_modem_suspend NULL
#define charm_modem_resume NULL
#endif

static struct dev_pm_ops charm_modem_dev_pm_ops = {
        .suspend = charm_modem_suspend,
        .resume = charm_modem_resume,
};

static int __init charm_modem_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct charm_platform_data *d = pdev->dev.platform_data;

	/* Modified by HTC */
	AP2MDM_STATUS   = d->gpio_ap2mdm_status;
	AP2MDM_WAKEUP   = d->gpio_ap2mdm_wakeup;
	AP2MDM_ERRFATAL = d->gpio_ap2mdm_errfatal;
	/*AP2MDM_SYNC = d->gpio_ap2mdm_sync;*/
	AP2MDM_PMIC_RESET_N = d->gpio_ap2mdm_pmic_reset_n;
	AP2MDM_KPDPWR_N     = d->gpio_ap2mdm_kpdpwr_n;
	AP2PMIC_TMPNI_CKEN  = d->gpio_ap2pmic_tmpni_cken;

	MDM2AP_STATUS   = d->gpio_mdm2ap_status;
	MDM2AP_WAKEUP   = d->gpio_mdm2ap_wakeup;
	MDM2AP_ERRFATAL = d->gpio_mdm2ap_errfatal;
	MDM2AP_SYNC   = d->gpio_mdm2ap_sync;
	MDM2AP_VFR   = d->gpio_mdm2ap_vfr;
	/*------------------------------------------------*/

	gpio_request(AP2MDM_STATUS, "AP2MDM_STATUS");
	gpio_request(AP2MDM_ERRFATAL, "AP2MDM_ERRFATAL");
	gpio_request(AP2MDM_KPDPWR_N, "AP2MDM_KPDPWR_N");
	gpio_request(AP2MDM_PMIC_RESET_N, "AP2MDM_PMIC_RESET_N");

	gpio_direction_output(AP2MDM_STATUS, 1);
	gpio_direction_output(AP2MDM_ERRFATAL, 0);

	power_on_charm = d->charm_modem_on;
	power_down_charm = d->charm_modem_off;
	reset_charm = d->charm_modem_reset;		/* Added by HTC */
	suspend_charm = d->charm_modem_suspend;		/* Added by HTC */
	resume_charm = d->charm_modem_resume;		/* Added by HTC */

	gpio_request(MDM2AP_ERRFATAL, "MDM2AP_ERRFATAL");
	gpio_direction_input(MDM2AP_ERRFATAL);

#if defined(CONFIG_MACH_VIGOR) || defined(CONFIG_MACH_HOLIDAY)
	gpio_request(MDM2AP_WAKEUP, "MDM2AP_WAKEUP");
	gpio_direction_output(MDM2AP_WAKEUP, 1);
#endif

	atomic_notifier_chain_register(&panic_notifier_list, &charm_panic_blk);
	charm_debugfs_init();

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_ERRFATAL IRQ resource. \
			error=%d No IRQ will be generated on errfatal.",
			__func__, irq);
		goto errfatal_err;
	}

	ret = request_irq(irq, charm_errfatal,
		IRQF_TRIGGER_RISING , "charm errfatal", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_ERRFATAL IRQ#%d request failed with error=%d\
			. No IRQ will be generated on errfatal.",
			__func__, irq, ret);
		goto errfatal_err;
	}
	charm_errfatal_irq = irq;

errfatal_err:

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_STATUS IRQ resource. \
			error=%d No IRQ will be generated on status change.",
			__func__, irq);
		goto status_err;
	}

	ret = request_threaded_irq(irq, NULL, charm_status_change,
		IRQF_TRIGGER_FALLING, "charm status", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_STATUS IRQ#%d request failed with error=%d\
			. No IRQ will be generated on status change.",
			__func__, irq, ret);
		goto status_err;
	}
	charm_status_irq = irq;

status_err:
	subsys_notif_register_notifier("external_modem", &gsbi9_nb);

	pr_info("%s: Registering charm modem\n", __func__);

	return misc_register(&charm_modem_misc);
}


static int __devexit charm_modem_remove(struct platform_device *pdev)
{

	return misc_deregister(&charm_modem_misc);
}

#if defined(CONFIG_MACH_VERDI_LTE) || defined(CONFIG_MACH_RUBY) || defined(CONFIG_MACH_HOLIDAY)
static void notify_mdm9k_shutdown(void)
{
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	int err, ret_len = 4;
	char input = 0;

	if ((rpc_client == NULL) || IS_ERR(rpc_client)) {
		printk(KERN_INFO "%s: try to open oem rapi client\n", __func__);
		rpc_client = oem_rapi_client_init();
		printk(KERN_INFO "%s: open oem rapi client => %p\n", __func__, rpc_client);
	}
	if (IS_ERR(rpc_client))
		printk(KERN_INFO "%s: couldn't open oem rapi client\n", __func__);
	else {
		arg.event = OEM_RAPI_CLIENT_EVENT_SSD_DISABLE_WATCHDOG_SMPLD;
		arg.cb_func = NULL;
		arg.handle = (void *)0;
		arg.in_len = 1;
		arg.input = &input;

		arg.out_len_valid = 1;
		arg.output_valid = 1;
		arg.output_size = 4;
		ret.out_len = &ret_len;
		ret.output = NULL;

		err = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
		if (err)
			printk(KERN_INFO "%s: FATAL ERROR! Notify mdm9k shutdown failed! err=%d\n", __func__, err);
	}
}
#endif

static void charm_modem_shutdown(struct platform_device *pdev)
{
#ifndef CONFIG_MACH_RUBY
	int i;
#endif

	/* Modified by HTC */
	disable_irq_nosync(charm_errfatal_irq);
	disable_irq_nosync(charm_status_irq);

#if defined(CONFIG_MACH_VIGOR) || defined(CONFIG_MACH_HOLIDAY)
	if (system_state == SYSTEM_POWER_OFF)
		gpio_set_value(MDM2AP_WAKEUP, 0);
	else
		gpio_set_value(MDM2AP_WAKEUP, 1);
#endif

#if defined(CONFIG_MACH_VERDI_LTE) || defined(CONFIG_MACH_RUBY) || defined(CONFIG_MACH_HOLIDAY)
	if (system_state == SYSTEM_POWER_OFF) {
		pr_info("%s: Notify mdm9k shutdown (%d)\n", __func__, system_state);
		notify_mdm9k_shutdown();
		pet_watchdog();
		msleep(500); /* wait for mdm9K shutdown readiness */
		return;
	} else
#endif
	{
	pr_info("%s: setting AP2MDM_STATUS low for a graceful restart (%d)\n",
		__func__, system_state);
	gpio_set_value(AP2MDM_STATUS, 0);
	}

/*        per radio comment, mdm may not set MDM2APP_STATUS to 0 always, thus ignore this polling*/
#ifndef CONFIG_MACH_RUBY
	if (!charm_MDM_error_flag) {	/* Modified by HTC */
		for (i = CHARM_MODEM_TIMEOUT; i > 0; i -= CHARM_MODEM_DELTA) {
			pet_watchdog();
			msleep(CHARM_MODEM_DELTA);
			if (gpio_get_value(MDM2AP_STATUS) == 0)
				break;
		}

		if (i <= 0) {
			pr_err("%s: MDM2AP_STATUS never went low in %d(ms)!\n", __func__, CHARM_MODEM_TIMEOUT);
			gpio_direction_output(AP2MDM_PMIC_RESET_N, 1);
			for (i = CHARM_HOLD_TIME; i > 0; i -= CHARM_MODEM_DELTA) {
				pet_watchdog();
				msleep(CHARM_MODEM_DELTA);
			}
			gpio_direction_output(AP2MDM_PMIC_RESET_N, 0);
		}
	}
#endif
}

static struct platform_driver charm_modem_driver = {
	.remove         = charm_modem_remove,
	.shutdown	= charm_modem_shutdown,
	.driver         = {
		.name = "charm_modem",
		.owner = THIS_MODULE,
		.pm = &charm_modem_dev_pm_ops
	},
};

static int __init charm_modem_init(void)
{
	/* Added by HTC */
	if (get_kernel_flag() & BIT6)
		charm_debug_on = 1;

	INIT_DELAYED_WORK(&charm_status_work, charm_status_fn);
	INIT_DELAYED_WORK(&charm_fatal_work, charm_fatal_fn);

	return platform_driver_probe(&charm_modem_driver, charm_modem_probe);
}

static void __exit charm_modem_exit(void)
{
	platform_driver_unregister(&charm_modem_driver);
}

/* HTC: provide an accessor for other module to dump gpio */
void ex_charm_dump_mdm_related_gpio_status(void)
{
	charm_dump_mdm_related_GPIO_status();
}

module_init(charm_modem_init);
module_exit(charm_modem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("msm8660 charm modem driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("charm_modem")


