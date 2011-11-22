/* arch/arm/mach-msm/sdio_tty.c
 *
 * Copyright (C) 2011 HTC, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <mach/sdio_al.h>
#include <mach/sdio_cmux.h>
#include <mach/board.h>		/* Added by HTC */
#include <linux/kthread.h>
#include <linux/delay.h>
#include "sdio_al_private.h"
#define MAX_TRASH_CAN_SIZE 4096

char trashCan[MAX_TRASH_CAN_SIZE];
bool defaultEnableDUN = false;

#define MAX_SDIO_TTYS 3
//++SSD_RIL: Mars@20110614: define for CIQ & TTY
#define SDIO_TTY_INDEX_DUN 0
#define SDIO_TTY_INDEX_CIQ 2

#define SDIO_TTY_NAME_DUN "SDIO_DUN"
#define SDIO_TTY_NAME_CIQ "SDIO_CIQ"

#define SDIO_TTY_OPEN_FLAG_DUN 1
#define SDIO_TTY_OPEN_FLAG_CIQ 2
//--SSD_RIL
//++SSD_RIL: Mars@20110616: keep DUN SDIO_OPEN status
bool g_IsDUNOpened = false;
int g_DUN_res_code = -22;
//--SSD_RIL

unsigned int diag_enable_flag = 0x0;	/* flag for checking if the USB diag is enabled to avoid conflict with sdio_tty*/
extern int sdio_dun_ch_init; /* HTC: comes from sdio_al. for init. sync. */

//++SSD_RIL: Mars@20110614: since we only can call sdio_open once, so we save the open status in a gobal paramters.
unsigned int sdio_open_flag = 0x0;
//--SSD_RIL

static DEFINE_MUTEX(sdio_tty_lock);

struct sdio_tty_info {
	struct sdio_channel	*ch;
	int	sdio_open;
	struct tty_struct *tty;
	struct wake_lock wake_lock;
	int tty_open_count;
	struct work_struct tty_work;
};

static struct sdio_tty_info sdio_tty[MAX_SDIO_TTYS];
static struct workqueue_struct *sdio_tty_wq;

static void sdio_tty_work_func_dun(struct work_struct *work)
{
	unsigned char *ptr;
	int avail;
	int ret;

	struct sdio_tty_info *info = container_of(work,
						struct sdio_tty_info,
						tty_work);
	struct tty_struct *tty = info->tty;

	if (!tty && !defaultEnableDUN) {
		return;
	}

	mutex_lock(&sdio_tty_lock);

	for (;;) {
		if (!defaultEnableDUN && test_bit(TTY_THROTTLED, &tty->flags))
			break;

		if (!defaultEnableDUN && info->ch == 0) {
			printk(KERN_ERR "sdio_tty_work_func_dun: info->ch null\n");
			break;
		}

		avail = sdio_read_avail(info->ch);

		if (avail == 0) {
			break;
		}

		//++HTC_CSP:
		if (defaultEnableDUN && info->tty_open_count == 0) { //open by itself, for the workaroudn of unnecessary DUN data
			pr_info("[sdio_tty]sdio_tty_work_func_dun: drop unnecessary data. \n");
			sdio_read(info->ch, trashCan, avail);
			mutex_unlock(&sdio_tty_lock);
			return;
		}
		if (!tty || test_bit(TTY_THROTTLED, &tty->flags) || info->ch == 0) {
			mutex_unlock(&sdio_tty_lock);
			return;
		}
		//--HTC_CSP
		ptr = NULL;
		avail = tty_prepare_flip_string(tty, &ptr, avail);
		if (avail && ptr) {
			ret = sdio_read(info->ch, ptr, avail);
			if (ret) {
				pr_info("[sdio_tty]sdio_tty_work_func_dun: read 0 bytes from sdio_driver \n");
				break;
			}
			/*++SSD_RIL:Mars@20110830: remove un-useful garbage characters in log file.*/
			/*else
				pr_info("[sdio_tty]sdio_tty_work_func_dun: read bytes data %s \n", ptr);*/
			/*--SSD_RIL*/
			wake_lock_timeout(&info->wake_lock, HZ / 2);
			tty->low_latency = 1;
			tty_flip_buffer_push(tty);
		} else
			printk(KERN_ERR "sdio_tty_work_func_dun: tty_prepare_flip_string fail\n");
	}

	mutex_unlock(&sdio_tty_lock);

	/* XXX only when writable and necessary */
	tty_wakeup(tty);
}

#ifdef CONFIG_BUILD_CIQ
static void sdio_tty_work_func_ciq(struct work_struct *work)
{
	unsigned char *ptr;
	int avail;
	int ret;

	struct sdio_tty_info *info = container_of(work,
						struct sdio_tty_info,
						tty_work);
	struct tty_struct *tty = info->tty;

	if (!tty) {
		pr_info("[sdio_tty]sdio_tty_work_func_ciq: tty=NULL\n");
		return;
	}

	mutex_lock(&sdio_tty_lock);

	for (;;) {
		if (test_bit(TTY_THROTTLED, &tty->flags))
			break;
		if (info->ch == 0) {
			pr_info("[sdio_tty]sdio_tty_work_func_ciq: info->ch == 0 \n");
			printk(KERN_ERR "[sdio_tty]sdio_tty_work_func_ciq: info->ch null\n");
			break;
		}
		avail = sdio_read_avail(info->ch);
		if (avail == 0) {
			break;
		}

		ptr = NULL;
		avail = tty_prepare_flip_string(tty, &ptr, avail);
		if (avail && ptr) {
			ret = sdio_read(info->ch, ptr, avail);
			if (ret) {
				pr_info("[sdio_tty]sdio_tty_work_func_ciq: read 0 bytes from sdio_driver \n");
				break;
			}
			/*++SSD_RIL:Mars@20110830: remove un-useful garbage characters in log file.*/
			/*else
				pr_info("[sdio_tty]sdio_tty_work_func_ciq: read bytes data %s \n", ptr);*/
			/*--SSD_RIL*/
			wake_lock_timeout(&info->wake_lock, HZ / 2);
			tty->low_latency = 1;
			tty_flip_buffer_push(tty);
		} else
			printk(KERN_ERR "[sdio_tty]sdio_tty_work_func_ciq: tty_prepare_flip_string fail\n");
	}

	mutex_unlock(&sdio_tty_lock);

	/* XXX only when writable and necessary */
	tty_wakeup(tty);
}
#endif

static void sdio_tty_notify(void *priv, unsigned event)
{
	struct sdio_tty_info *info = priv;
	pr_info("[sdio_tty]sdio_tty_notify: receive notification \n");
	if (info == NULL) {
		pr_info("[sdio_tty]sdio_tty_notify: no one uses sdio_tty, just return  \n");
		return;
	}

	if (event != SDIO_EVENT_DATA_READ_AVAIL)
		return;
	pr_info("[sdio_tty]sdio_tty_notify: qeueu a work to get the data from sdio and flush it to tty buffer \n");
	queue_work(sdio_tty_wq, &info->tty_work);
}

static int sdio_tty_open(struct tty_struct *tty, struct file *f)
{
	int res = 0;
	int n = tty->index;
//++SSD_RIL: Mars@20110614: since we only can call sdio_open once, so we save the open status in a gobal paramters.
	int nOpenFlag = 0;
//--SSD_RIL
	struct sdio_tty_info *info;
	const char *name;

	if (n == SDIO_TTY_INDEX_DUN) {
		name = SDIO_TTY_NAME_DUN;
		nOpenFlag = SDIO_TTY_OPEN_FLAG_DUN;
	} else if (n ==SDIO_TTY_INDEX_CIQ){
		name = SDIO_TTY_NAME_CIQ;
		nOpenFlag = SDIO_TTY_OPEN_FLAG_CIQ;
	}else {
		return -ENODEV;
	}

	if (diag_enable_flag == 1 && (n == SDIO_TTY_INDEX_DUN)) //USB diag is enabled (sdio_dun is opened), return -1
		return -1;
	info = sdio_tty + n;

	mutex_lock(&sdio_tty_lock);
	tty->driver_data = info;
	info->tty = tty;
	//++HTC_CSP: Since the limitation of sdio_al(The channel can be opened once), we will keep the channel info if it's ever opened
	if(info->sdio_open == 1) {
		info->tty_open_count++;
		pr_info("[sdio_tty]sdio_tty_open: sdio_open is alreayd called by someone, info->tty_open_count=[%d]\n", info->tty_open_count);
	}
	//if (info->tty_open_count++ == 0) {
	if (info->sdio_open == 0) {
		info->tty_open_count ++;
	//--HTC_CSP:Since the limitation of sdio_al(The channel can be opened once), we will keep the channel info if it's ever opened
		wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, name);
		//++HTC_CSP: move the tty assignment to out side
		//info->tty = tty;
		//--HTC_CSP: move the tty assignment to out side
		//++HTC_CSP
		//if (info->ch) {
		//	sdio_kick(info->ch);
		//} else {
		//--HTC_CSP
			if (!defaultEnableDUN || (n != SDIO_TTY_INDEX_DUN)) {
				if((sdio_open_flag & nOpenFlag) == 0) {//++SSD_RIL: Mars@20110614: check the open status
					pr_info("[sdio_tty]sdio_tty_open: sdio_open+\n");
					res = sdio_open(name, &info->ch, info, sdio_tty_notify);
					pr_info("[sdio_tty]sdio_tty_open: sdio_open-, res=[%d]\n", res);
				}
			}

//++SSD_RIL: Mars@20110616: check dun open status
			if(defaultEnableDUN && n == SDIO_TTY_INDEX_DUN && g_IsDUNOpened == false)
			{
				res = g_DUN_res_code;
				pr_info("[sdio_tty]sdio_tty_open: g_IsDUNOpened=false, g_DUN_res_code=[%d]\n", g_DUN_res_code);
			}
//SSD_RIL

			if (res != 0) {
				info->sdio_open = 0;
				pr_info("[sdio_tty]sdio_tty_open: sdio_open FAIL, channel name %s error code: %d\n", name, res);
			} else {
				info->sdio_open = 1;
				pr_info("[sdio_tty]sdio_tty_open: sdio_open SUCCESS, channel name %s\n", name);
//++SSD_RIL: Mars@20110614: since we only can call sdio_open once, so we save the open status in a gobal paramters.
				sdio_open_flag	 |= nOpenFlag;//set the open status to 1
//--SSD_RIL
			}
#ifdef CONFIG_ARCH_QSD8X50
			/* 8x50 sdio bug: channel open is too late to handle
			 * sdio write request */
			/*HTC_CSP
			if (n == 19)
				sdio_wait_until_opened(info->ch, 200);
			HTC_CSP*/
#endif

	}
	mutex_unlock(&sdio_tty_lock);

	return res;
}

static void sdio_tty_close(struct tty_struct *tty, struct file *f)
{
	struct sdio_tty_info *info = tty->driver_data;
	pr_info("[sdio_tty]sdio_tty_close+\n");

	if (info == 0) {
		pr_info("[sdio_tty]sdio_tty_close-, info = 0\n");
		return;
	}

	if(info->ch != 0)
		pr_info("[sdio_tty]sdio_tty_close: info->ch->name=[%s]\n", info->ch->name);

	/* wait for the work in workqueue to complete */
	flush_work(&info->tty_work);

	mutex_lock(&sdio_tty_lock);
	if(info->tty_open_count > 0)
		info->tty_open_count--;

//	if (--info->tty_open_count == 0) {
//		info->tty = 0;
//		tty->driver_data = 0;
//		wake_lock_destroy(&info->wake_lock);
//		if (info->ch) {
//			sdio_close(info->ch);
			/*++HTC_CSP: Don't release the channel due to sdio_al haven't implemented close function of the channel
			   So, we keep the channel for later use
			info->ch = 0;
			*/
//		}
//	}
	//++HTC_CSP: We will not reset sdio_open to 0 if sdio_tty is ever opened
	//info->sdio_open = 0;
	//--HTC_CSP:  We will not reset sdio_open to 0 if sdio_tty is ever opened
	mutex_unlock(&sdio_tty_lock);
	pr_info("[sdio_tty]sdio_tty_close-\n");
}

static int sdio_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int len)
{
	struct sdio_tty_info *info = tty->driver_data;
	int avail;
	int ret;

	/* if we're writing to a packet channel we will
	** never be able to write more data than there
	** is currently space for
	*/
	if (!info->sdio_open) {
		pr_debug("[sdio_tty]%s: SDIO IO is not supported\n", __func__);
		//info->n_read = 0;
		return 0;
	}
	/*++SSD_RIL:Mars@20110830: remove SDIO_write message*/
	/*pr_info("[sdio_tty]%s: SDIO_write: %s\n", __func__, buf);*/
	/*--SSD_RIL*/
	mutex_lock(&sdio_tty_lock);
	avail = sdio_write_avail(info->ch);
	if (len > avail)
		len = avail;
	ret = sdio_write(info->ch, buf, len);
	if (ret) {
		pr_err("[sdio_tty]%s: sdio write failed err:%d", __func__, ret);
		/* try again later */
		return 0;
	}
	mutex_unlock(&sdio_tty_lock);

	return len;
}

static int sdio_tty_write_room(struct tty_struct *tty)
{
	struct sdio_tty_info *info = tty->driver_data;
	return sdio_write_avail(info->ch);
}

static int sdio_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct sdio_tty_info *info = tty->driver_data;
	return sdio_read_avail(info->ch);
}

static void sdio_tty_unthrottle(struct tty_struct *tty)
{
	struct sdio_tty_info *info = tty->driver_data;
	queue_work(sdio_tty_wq, &info->tty_work);
	return;
}

static struct tty_operations sdio_tty_ops = {
	.open = sdio_tty_open,
	.close = sdio_tty_close,
	.write = sdio_tty_write,
	.write_room = sdio_tty_write_room,
	.chars_in_buffer = sdio_tty_chars_in_buffer,
	.unthrottle = sdio_tty_unthrottle,
};

#include <mach/board_htc.h>

static struct tty_driver *sdio_tty_driver;

int open_sdio_dun(void* param) {
	bool opened = false;
	const char *name = "SDIO_DUN";
	int res = 0, count = 0;

	pr_info("[sdio_tty]open_sdio_dun+\n");
	/* HTC: wait until channel init done, retry every 2s,
		print out msg every 5 retries */
	msleep(10000);
	while (!sdio_dun_ch_init) {
		if (0 == ++count % 5)
			pr_info("[sdio_tty]open_sdio_dun: wait for SDIO_DUN channel initialization.\n");
		msleep(2000);
	}
	pr_info("[sdio_tty]open_sdio_dun: abstract layer init channel DUN done\n");


	while (!opened) {
		pr_info("[sdio_tty] open_sdio_dun: sdio_open+\n");
		res = sdio_open(name, &(sdio_tty[0].ch), &sdio_tty[0], sdio_tty_notify);
		pr_info("[sdio_tty] open_sdio_dun: sdio_open-, res=[%d]\n", res);
//++SSD_RIL: Mars@20110616: keep DUN SDIO_OPEN status
		g_DUN_res_code = res;
//--SSD_RIL
		if (res != 0) {
			pr_info("[sdio_tty]open_sdio_dun: sdio_open FAIL, channel name %s error code: %d\n", name, res);
			msleep(2000);
		} else {
			pr_info("[sdio_tty]open_sdio_dun: sdio_open SUCCESS, channel name %s\n", name);
			opened = true;
			g_IsDUNOpened = true;
		}
	}
	pr_info("[sdio_tty]open_sdio_dun-\n");
	return 0;
}
static int __init sdio_tty_init(void)
{
	int ret;

	/* Switch diag flag by radioflag */
	if (get_radio_flag() & BIT17)
		diag_enable_flag = 1;
	else
		diag_enable_flag = 0;
	pr_info("[sdio_tty]%s: get diag_enable_flag = %d\n", __func__, diag_enable_flag);

	sdio_tty_wq = create_singlethread_workqueue("sdio_tty");
	if (sdio_tty_wq == 0)
		return -ENOMEM;

	sdio_tty_driver = alloc_tty_driver(MAX_SDIO_TTYS);
	if (sdio_tty_driver == 0) {
		destroy_workqueue(sdio_tty_wq);
		return -ENOMEM;
	}

	sdio_tty_driver->owner = THIS_MODULE;
	sdio_tty_driver->driver_name = "sdio_tty_driver";
	sdio_tty_driver->name = "sdiotty";
	sdio_tty_driver->major = 0;
	sdio_tty_driver->minor_start = 0;
	sdio_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	sdio_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	sdio_tty_driver->init_termios = tty_std_termios;
	sdio_tty_driver->init_termios.c_iflag = 0;
	sdio_tty_driver->init_termios.c_oflag = 0;
	sdio_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD;
	sdio_tty_driver->init_termios.c_lflag = 0;
	sdio_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(sdio_tty_driver, &sdio_tty_ops);

	ret = tty_register_driver(sdio_tty_driver);
	if (ret)
		return ret;

	/* this should be dynamic */
	tty_register_device(sdio_tty_driver, 0, 0);
	INIT_WORK(&sdio_tty[0].tty_work, sdio_tty_work_func_dun);//SSD_RIL: Mars@20110620: separate DUN and CIQ work function, call sdio_tty_work_func_dun for DUN
//++SSD_RIL: Mars@20110620: remove unused code.
/*
	tty_register_device(sdio_tty_driver, 1, 0);
	INIT_WORK(&sdio_tty[1].tty_work, sdio_tty_work_func);
*/
//--SSD_RIL
#ifdef CONFIG_BUILD_CIQ
	tty_register_device(sdio_tty_driver, 2, 0);
	INIT_WORK(&sdio_tty[2].tty_work, sdio_tty_work_func_ciq);//SSD_RIL: Mars@20110620: separate DUN and CIQ work function, call sdio_tty_work_func_ciq for CIQ
#endif


	if (!diag_enable_flag) {
		defaultEnableDUN = true;
		kthread_run(open_sdio_dun, NULL, "open_sdio_dun");
	}

	return 0;
}

module_init(sdio_tty_init);
