/*
 *
 * Copyright (C) 2009 HTC, Inc.
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

#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <asm/cacheflush.h>

#include <mach/scm.h>
#include <mach/scm-log.h>
#include <mach/msm_iomap.h>

#define TZ_SCM_LOG_VIRT		MSM_TZLOG_BASE
#define TZ_SCM_LOG_PHYS		MSM_TZLOG_PHYS
#define TZ_SCM_LOG_SIZE		MSM_TZLOG_SIZE

struct scm_tzlog_dev {
	char (*buffer)[TZBSP_LOG_MAX_LINE_LEN];
	int row_cursor;
	int current_row;
	int row_flag;
};

struct scm_tzlog_dev *scm_tzlog;

int scm_fulldump(void)
{
	int i, row;
	char *row_flag;

	row = (TZ_SCM_LOG_SIZE - INT_SIZE)/TZBSP_LOG_MAX_LINE_LEN;

	for (i = 0; i < row; i++)
		if (strlen(scm_tzlog->buffer[i]))
			pr_info("%s \n", scm_tzlog->buffer[i]);


	row_flag = (char *)scm_tzlog->row_flag;

	return 0;
}
EXPORT_SYMBOL(scm_fulldump);

int scm_dump(void)
{
	int i, row;
	int *tz_row;

	row = (TZ_SCM_LOG_SIZE - INT_SIZE)/TZBSP_LOG_MAX_LINE_LEN;
	tz_row = (int *)scm_tzlog->row_cursor;

	if (scm_tzlog->buffer != 0) {
		if (scm_tzlog->current_row < *tz_row) {
			for (i = scm_tzlog->current_row; i < *tz_row; i++)
				if (strlen(scm_tzlog->buffer[i]))
					pr_info("%s \n", scm_tzlog->buffer[i]);

		}

		if (scm_tzlog->current_row > *tz_row) {
			for (i = scm_tzlog->current_row; i < row; i++)
				if (strlen(scm_tzlog->buffer[i]))
					pr_info("%s \n", scm_tzlog->buffer[i]);

			for (i = 0; i < *tz_row; i++)
				if (strlen(scm_tzlog->buffer[i]))
					pr_info("%s \n", scm_tzlog->buffer[i]);
		}

		if (scm_tzlog->current_row == *tz_row)
			pr_info("No New Trust Zone log \n");

	}

	scm_tzlog->current_row = *tz_row;

	return 0;
}
EXPORT_SYMBOL(scm_dump);

static ssize_t scm_tzlog_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	scm_dump();
	ret = sprintf(buf, " Dump tz log \n");

	return ret;
}
static DEVICE_ATTR(tzlog_dump, 0444, scm_tzlog_show, NULL);


static ssize_t scm_tzlogall_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	scm_fulldump();
	ret = sprintf(buf, " Dump all tz log \n");

	return ret;
}
static DEVICE_ATTR(tzlog_dumpall, 0444, scm_tzlogall_show, NULL);

static int scm_log_probe(struct platform_device *pdev)
{
	int ret, i;
	struct scm_tzlog_dev *scm_dev;

	scm_dev = kzalloc(sizeof(struct scm_tzlog_dev), GFP_KERNEL);
	if (!scm_dev) {
		ret = -ENOMEM;
		goto err_scm_dev;
	}
	scm_tzlog = scm_dev;


	scm_tzlog->buffer = (char(*)[TZBSP_LOG_MAX_LINE_LEN])TZ_SCM_LOG_VIRT;

	scm_tzlog->row_cursor = (int)(scm_tzlog->buffer) +
				 TZ_SCM_LOG_SIZE - INT_SIZE;

	scm_tzlog->current_row = 0;

	pr_info("address %x\n", TZ_SCM_LOG_PHYS);
	memset(scm_tzlog->buffer, 0, TZ_SCM_LOG_SIZE);

	secure_log_operation(0, 0, TZ_SCM_LOG_PHYS, 32*64, 0);

	for (i = 0; i < 32; i++)
		if (strlen(scm_tzlog->buffer[i]))
			pr_info("%s \n", scm_tzlog->buffer[i]);

	secure_log_operation(TZ_SCM_LOG_PHYS, TZ_SCM_LOG_SIZE, 0, 0, 0);

	ret = device_create_file(&pdev->dev, &dev_attr_tzlog_dump);
	if (ret < 0) {
		pr_err("%s: create device attribute fail \n", __func__);
		goto err_create_file;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_tzlog_dumpall);
	if (ret < 0) {
		pr_err("%s: create device attribute fail \n", __func__);
		goto err_create_file;
	}
	return 0;
err_create_file:
	kfree(scm_dev);
err_scm_dev:
	return ret;

}

static struct platform_driver scm_log_driver = {
	.probe = scm_log_probe,
	.driver = { .name = "scm-log", },
};


static int __init scm_device_init(void)
{
	return platform_driver_register(&scm_log_driver);
}

static void __exit scm_device_exit(void)
{
	platform_driver_unregister(&scm_log_driver);
}

module_init(scm_device_init);
module_exit(scm_device_exit);
