/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
#ifdef CONFIG_ARCH_MSM8X60_LTE

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <asm/current.h>
#include <mach/board_htc.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#include "../../usb/gadget/f_diag.h"
#endif
#include "diagchar_hdlc.h"
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_sdio.h"


int sdio_diag_initialized;

static char usb_ch_name[16];
static int diag9k_debug_mask;

/*RIL flag */
static unsigned int ril_sdio_dbg_flag;
module_param_named(debug_mdm_mask, diag9k_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

void __diag_sdio_send_req(void)
{
	int r = 0;
	void *buf = NULL;
	int *in_busy_ptr = NULL;
	struct diag_request *write_ptr_modem = NULL;
	int retry = 0;
#if defined(CONFIG_MACH_VIGOR)
	int type;
#endif

	if (!driver->in_busy_sdio_1) {
		buf = driver->buf_in_sdio_1;
		write_ptr_modem = driver->write_ptr_mdm_1;
		in_busy_ptr = &(driver->in_busy_sdio_1);
	} else if (!driver->in_busy_sdio_2) {
		buf = driver->buf_in_sdio_2;
		write_ptr_modem = driver->write_ptr_mdm_2;
		in_busy_ptr = &(driver->in_busy_sdio_2);
	}

	APPEND_DEBUG('Z');
	if (driver->sdio_ch && buf) {
		r = sdio_read_avail(driver->sdio_ch);

		if (r > MAX_IN_BUF_SIZE) {
				DIAG_ERR("\n diag: SDIO sending"
					  " in packets more than %d bytes\n", r);
		}
		if (r > 0) {
			if (!buf)
				DIAG_INFO("Out of diagmem for SDIO\n");
			else {
drop:
				APPEND_DEBUG('i');
				sdio_read(driver->sdio_ch, buf, r);
				if ((driver->qxdm2sd_drop) && (driver->logging_mode == USB_MODE)) {
					/*Drop the diag payload */
					DIAG_INFO("%s:Drop the diag payload :%d\n", __func__, retry);
					print_hex_dump(KERN_DEBUG, "Drop Packet Data"
						" from 9K(first 16 bytes)", DUMP_PREFIX_ADDRESS, 16, 1, buf, 16, 1);
					driver->in_busy_sdio_1 = 0;
					driver->in_busy_sdio_2 = 0;
					r=sdio_read_avail(driver->sdio_ch);
					if (++retry > 20) {
						driver->qxdm2sd_drop = 0;
						return;
						}
					if (r)
						goto drop;
					else {
						driver->qxdm2sd_drop = 0;
						return;
						}
				}
				APPEND_DEBUG('j');

				if (diag9k_debug_mask) {
					switch (diag9k_debug_mask) {
					case 1:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 9K(first 16 bytes)", DUMP_PREFIX_ADDRESS, 16, 1, buf, 16, 1);
						break;
					case 2:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 9K(first 16 bytes)", DUMP_PREFIX_ADDRESS, 16, 1, buf, 16, 1);
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 9K(last 16 bytes) ", 16, 1, DUMP_PREFIX_ADDRESS, buf+r-16, 16, 1);
						break;
					default:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 9K ", DUMP_PREFIX_ADDRESS, 16, 1, buf, r, 1);

					}
				}
#if defined(CONFIG_MACH_VIGOR)
				type = checkcmd_modem_epst(buf);
				if (type) {
					modem_to_userspace(buf, r, type, 1);
					return;
				}
#endif

				write_ptr_modem->length = r;
				*in_busy_ptr = 1;
				diag_device_write(buf, SDIO_DATA,
						 write_ptr_modem);

			}
		}
	}
}

static void diag_read_sdio_work_fn(struct work_struct *work)
{
	APPEND_DEBUG('Y');
	__diag_sdio_send_req();
}

int diagfwd_connect_sdio(void)
{

	int err;

	DIAG_INFO("%s\n", __func__);

	if (!strcmp(DIAG_MDM, usb_ch_name)) {
		err = usb_diag_alloc_req(driver->mdm_ch, N_MDM_WRITE,
							 N_MDM_READ);
		if (err) {
			DIAG_ERR("diag: unable to alloc USB req on mdm ch");
			return -ENOMEM;
		}
	}

	driver->in_busy_sdio_1 = 0;
	driver->in_busy_sdio_2 = 0;
	/* Poll SDIO channel to check for data*/
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_sdio_work));
	return 0;
}

int diagfwd_disconnect_sdio(void)
{
	DIAG_INFO("%s \n", __func__);

	/* Clear variable to Flush remaining data from SDIO channel */
	driver->in_busy_sdio_1 = 0;
	driver->in_busy_sdio_2 = 0;
	if (!strcmp(DIAG_MDM, usb_ch_name)) {
		usb_diag_free_req(driver->mdm_ch);
	}
	return 0;
}

int diagfwd_write_complete_sdio(void)
{
	driver->in_busy_sdio_1 = 0;
	driver->in_busy_sdio_2 = 0;
	APPEND_DEBUG('q');
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_sdio_work));
	return 0;
}

int diagfwd_read_complete_sdio(void)
{
	queue_work(driver->diag_sdio_wq, &(driver->diag_read_mdm_work));
	return 0;
}

void diag_read_mdm_work_fn(struct work_struct *work)
{
		if (diag9k_debug_mask)
		DIAG_INFO("%s \n", __func__);

	if (driver->sdio_ch) {
		wait_event_interruptible(driver->wait_q, (sdio_write_avail
				(driver->sdio_ch) >= driver->read_len_mdm));
		if (!strcmp(DIAG_MDM, usb_ch_name)) {
			if (driver->sdio_ch && driver->usb_buf_mdm_out &&
						 (driver->read_len_mdm > 0))
				sdio_write(driver->sdio_ch, driver->usb_buf_mdm_out,
							 driver->read_len_mdm);

		} else {
			if (driver->sdio_ch && driver->usb_read_ptr &&
						 (driver->read_len_mdm > 0))
				sdio_write(driver->sdio_ch, driver->usb_buf_out,
							 driver->read_len_mdm);

		}

		APPEND_DEBUG('x');
		if (!strcmp(DIAG_MDM, usb_ch_name)) {
			driver->usb_read_mdm_ptr->buf = driver->usb_buf_mdm_out;
			driver->usb_read_mdm_ptr->length = USB_MAX_OUT_BUF;
			usb_diag_read(driver->mdm_ch, driver->usb_read_mdm_ptr);
		} else {
			driver->usb_read_ptr->buf = driver->usb_buf_out;
			driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
			usb_diag_read(driver->legacy_ch, driver->usb_read_ptr);
		}
		APPEND_DEBUG('y');
	}
}

static void diag_sdio_notify(void *ctxt, unsigned event)
{
	if (ril_sdio_dbg_flag)
		DIAG_INFO("%s event=%d(1:read 2:write)%d:%d\n", __func__, event, driver->in_busy_sdio_1, driver->in_busy_sdio_2);

	if (event == SDIO_EVENT_DATA_READ_AVAIL) {
		APPEND_DEBUG('T');
		queue_work(driver->diag_sdio_wq,
				 &(driver->diag_read_sdio_work));
}
	if (event == SDIO_EVENT_DATA_WRITE_AVAIL) {
		APPEND_DEBUG('S');
		wake_up_interruptible(&driver->wait_q);
}
}

static int diag_sdio_probe(struct platform_device *pdev)
{
	int err;
	if (diag9k_debug_mask)
		DIAG_INFO("%s\n", __func__);

	err = sdio_open("SDIO_DIAG", &driver->sdio_ch, driver,
							 diag_sdio_notify);
	if (err)
		DIAG_ERR("DIAG could not open SDIO channel");
	else {
		DIAG_INFO("DIAG opened SDIO channel");

		if (!strcmp(DIAG_LEGACY, usb_ch_name)) {
			driver->legacy_ch = usb_diag_open(DIAG_LEGACY, driver,
				diag_usb_legacy_notifier);
			if (IS_ERR(driver->legacy_ch)) {
				DIAG_ERR("Unable to open USB diag legacy channel\n");
				goto err;
			}
			DIAG_INFO("open USB diag legacy channel\n");
		} else if (!strcmp(DIAG_MDM, usb_ch_name)) {
			driver->legacy_ch = usb_diag_open(DIAG_LEGACY, driver,
				diag_usb_legacy_notifier);
			if (IS_ERR(driver->legacy_ch)) {
				DIAG_ERR("Unable to open USB diag legacy channel\n");
				goto err;
			}
			DIAG_INFO("open USB diag legacy channel\n");

			driver->mdm_ch = usb_diag_open(DIAG_MDM, driver,
				diag_usb_legacy_notifier);
			if (IS_ERR(driver->mdm_ch)) {
				DIAG_ERR("Unable to open USB diag MDM channel\n");
				goto err;
			}
			DIAG_INFO("open USB diag MDM channel\n");
		}
/*		queue_work(driver->diag_sdio_wq, &(driver->diag_read_mdm_work));*/
	}
	if (diag_configured)
		diagfwd_connect();
	driver->in_busy_sdio_1 = 0;
	driver->in_busy_sdio_2 = 0;
	driver->qxdm2sd_drop = 0;
	sdio_diag_initialized = 1;
err:
	return err;
}
static int diag_sdio_remove(struct platform_device *pdev)
{
	queue_work(driver->diag_sdio_wq, &(driver->diag_remove_sdio_work));
	return 0;
}

static void diag_remove_sdio_work_fn(struct work_struct *work)
{
	DIAG_INFO("diag: sdio remove called\n");
	/*Disable SDIO channel to prevent further read/write */
	driver->sdio_ch = NULL;
	sdio_diag_initialized = 0;
	driver->in_busy_sdio_1 = 1;
	driver->in_busy_sdio_2 = 1;
}
static int diagfwd_sdio_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_sdio_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_sdio_dev_pm_ops = {
	.runtime_suspend = diagfwd_sdio_runtime_suspend,
	.runtime_resume = diagfwd_sdio_runtime_resume,
};

static struct platform_driver msm_sdio_ch_driver = {
	.probe = diag_sdio_probe,
	.remove = diag_sdio_remove,
	.driver = {
		   .name = "SDIO_DIAG",
		   .owner = THIS_MODULE,
		   .pm   = &diagfwd_sdio_dev_pm_ops,
		   },
};

void diagfwd_sdio_init(const char *name)
{
	int ret;

	if (diag9k_debug_mask)
		DIAG_INFO("%s\n", __func__);

	driver->read_len_mdm = 0;
	if (driver->buf_in_sdio_1 == NULL)
		driver->buf_in_sdio_1 = kzalloc(MAX_IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_sdio_1 == NULL)
			goto err;
	if (driver->buf_in_sdio_2 == NULL)
		driver->buf_in_sdio_2 = kzalloc(MAX_IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_sdio_2 == NULL)
			goto err;
	if (driver->usb_buf_mdm_out  == NULL)
		driver->usb_buf_mdm_out = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
		if (driver->usb_buf_mdm_out == NULL)
			goto err;
	if (driver->write_ptr_mdm_1 == NULL)
		driver->write_ptr_mdm_1 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_mdm_1 == NULL)
			goto err;
	if (driver->write_ptr_mdm_2 == NULL)
		driver->write_ptr_mdm_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->write_ptr_mdm_2 == NULL)
			goto err;
	if (driver->usb_read_mdm_ptr == NULL)
		driver->usb_read_mdm_ptr = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_read_mdm_ptr == NULL)
			goto err;
	driver->diag_sdio_wq = create_singlethread_workqueue("diag_sdio_wq");
#ifdef CONFIG_DIAG_OVER_USB
	strncpy(usb_ch_name, name, 16);
	DIAG_INFO("%s using %s\n", __func__, usb_ch_name);
/*
	driver->mdm_ch = usb_diag_open(DIAG_MDM, driver,
			diag_usb_legacy_notifier);
	if (IS_ERR(driver->mdm_ch)) {
		printk(KERN_ERR "Unable to open USB diag MDM channel\n");
		goto err;
	}
*/

	INIT_WORK(&(driver->diag_read_mdm_work), diag_read_mdm_work_fn);
#endif
	INIT_WORK(&(driver->diag_read_sdio_work), diag_read_sdio_work_fn);
	INIT_WORK(&(driver->diag_remove_sdio_work), diag_remove_sdio_work_fn);
	ret = platform_driver_register(&msm_sdio_ch_driver);
	if (ret)
		DIAG_INFO("DIAG could not register SDIO device");
	else
		DIAG_INFO("DIAG registered SDIO device\n");

	ril_sdio_dbg_flag = (get_kernel_flag() & BIT19)?1 :0;
	return;
err:
		DIAG_INFO("\n Could not initialize diag buf for SDIO");
		kfree(driver->buf_in_sdio_1);
		kfree(driver->buf_in_sdio_2);
		kfree(driver->usb_buf_mdm_out);
		kfree(driver->write_ptr_mdm_1);
		kfree(driver->write_ptr_mdm_2);
		kfree(driver->usb_read_mdm_ptr);
		if (driver->diag_sdio_wq)
			destroy_workqueue(driver->diag_sdio_wq);
}

void diagfwd_sdio_exit(void)
{
#ifdef CONFIG_DIAG_OVER_USB
	if (driver->usb_connected)
		usb_diag_free_req(driver->mdm_ch);
#endif
	platform_driver_unregister(&msm_sdio_ch_driver);
#ifdef CONFIG_DIAG_OVER_USB
	usb_diag_close(driver->mdm_ch);
#endif
		kfree(driver->buf_in_sdio_1);
		kfree(driver->buf_in_sdio_2);
	kfree(driver->usb_buf_mdm_out);
		kfree(driver->write_ptr_mdm_1);
		kfree(driver->write_ptr_mdm_2);
	kfree(driver->usb_read_mdm_ptr);
	destroy_workqueue(driver->diag_sdio_wq);
}

#endif /* CONFIG_MSM_SDIO_AL */