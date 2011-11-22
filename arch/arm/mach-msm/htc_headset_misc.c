/*
 *
 * /arch/arm/mach-msm/htc_headset_misc.c
 *
 * HTC MISC headset driver.
 *
 * Copyright (C) 2010 HTC, Inc.
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

#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_misc.h>

#define DRIVER_NAME "HS_MISC"

static struct workqueue_struct *detect_wq;
static void ext_hpin_detect_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(ext_hpin_detect_work, ext_hpin_detect_work_func);

static struct htc_headset_misc_info *hi;

int headset_indicator_enable(int enable)
{
	if (enable && headset_get_type_sync(1, 0) != HEADSET_INDICATOR) {
		HS_LOG("Indicator does not exist");
		return -ENXIO;
	}

	gpio_set_value(hi->pdata.indicator_power_gpio, (enable) ? 1 : 0);
	HS_LOG("%s HEADSET_INDICATOR", (enable) ? "Enable" : "Disable");

	return 0;
}

static void hs_misc_usb_audio_out_detect(int enable)
{
	HS_DBG();

	if (enable)
		headset_ext_detect(USB_AUDIO_OUT);
	else
		headset_ext_detect(USB_NO_HEADSET);
}

static void ext_hpin_detect_work_func(struct work_struct *work)
{
	int insert = 0;

	HS_DBG();

	insert = gpio_get_value(hi->pdata.ext_hpin_gpio) ? 0 : 1;
	hi->accessory_detect(insert);
}

static irqreturn_t ext_hpin_irq_handler(int irq, void *dev_id)
{
	unsigned int irq_mask = IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW;

	hi->ext_hpin_irq_type ^= irq_mask;
	set_irq_type(hi->ext_hpin_irq, hi->ext_hpin_irq_type);

	HS_LOG("External HPIN IRQ");

	wake_lock_timeout(&hi->hs_wake_lock, HS_WAKE_LOCK_TIMEOUT);
	cancel_delayed_work(&ext_hpin_detect_work);
	queue_delayed_work(detect_wq, &ext_hpin_detect_work,
			   hi->ext_hpin_debounce);

	if (hi->ext_hpin_irq_type & IRQF_TRIGGER_LOW)
		hi->ext_hpin_debounce = HS_JIFFIES_INSERT;
	else
		hi->ext_hpin_debounce = HS_JIFFIES_REMOVE;

	return IRQ_HANDLED;
}

static int hs_misc_request_irq(unsigned int gpio, unsigned int *irq,
			       irq_handler_t handler, unsigned long flags,
			       const char *name, unsigned int wake)
{
	int ret = 0;

	HS_DBG();

	ret = gpio_request(gpio, name);
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		gpio_free(gpio);
		return ret;
	}

	if (!(*irq)) {
		ret = gpio_to_irq(gpio);
		if (ret < 0) {
			gpio_free(gpio);
			return ret;
		}
		*irq = (unsigned int) ret;
	}

	ret = request_any_context_irq(*irq, handler, flags, name, NULL);
	if (ret < 0) {
		gpio_free(gpio);
		return ret;
	}

	ret = set_irq_wake(*irq, wake);
	if (ret < 0) {
		free_irq(*irq, 0);
		gpio_free(gpio);
		return ret;
	}

	return 1;
}

void headset_mhl_audio_jack_enable(int enable)
{
	int ret = 0;

	if (!(hi->pdata.driver_flag & DRIVER_HS_MISC_EXT_HP_DET)) {
		HS_LOG("NOT support MHL audio jack");
		return;
	}

	if (enable) {
		HS_LOG("Enable MHL audio jack");
		if (!hi->ext_hpin_enable) {
			hi->ext_hpin_irq_type = IRQF_TRIGGER_LOW;
			ret = hs_misc_request_irq(hi->pdata.ext_hpin_gpio,
					&hi->ext_hpin_irq, ext_hpin_irq_handler,
					hi->ext_hpin_irq_type, "EXT_HP_DET", 1);
			if (ret < 0) {
				HS_ERR("Failed to request EXT_HP_DET IRQ");
				return;
			}
			hi->ext_hpin_enable = 1;
		}
	} else {
		if (hi->ext_hpin_enable) {
			disable_irq(hi->ext_hpin_irq);
			free_irq(hi->ext_hpin_irq, 0);
			gpio_free(hi->pdata.ext_hpin_gpio);
			hi->ext_hpin_irq = 0;
			hi->ext_hpin_enable = 0;
		}
		HS_LOG("Disable MHL audio jack");
	}
}

static void hs_misc_register(void)
{
	struct headset_notifier notifier;

	if (hi->pdata.indicator_power_gpio) {
		notifier.id = HEADSET_REG_INDICATOR_ENABLE;
		notifier.func = headset_indicator_enable;
		headset_notifier_register(&notifier);
	}
}

static int htc_headset_misc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct htc_headset_misc_platform_data *pdata = pdev->dev.platform_data;

	HS_LOG("++++++++++++++++++++");

	hi = kzalloc(sizeof(struct htc_headset_misc_info), GFP_KERNEL);
	if (!hi) {
		HS_ERR("Failed to allocate memory for headset info");
		return -ENOMEM;
	}

	hi->pdata.driver_flag = pdata->driver_flag;
	hi->pdata.ext_hpin_gpio = pdata->ext_hpin_gpio;
	hi->pdata.ext_accessory_type = pdata->ext_accessory_type;

	hi->pdata.indicator_power_gpio = pdata->indicator_power_gpio;

	hi->ext_hpin_irq_type = IRQF_TRIGGER_LOW;
	hi->ext_hpin_debounce = HS_JIFFIES_INSERT;
	hi->ext_hpin_enable = 0;

	switch (hi->pdata.ext_accessory_type) {
	case USB_AUDIO_OUT:
		hi->accessory_detect = hs_misc_usb_audio_out_detect;
		break;
	default:
		if (hi->pdata.driver_flag & DRIVER_HS_MISC_EXT_HP_DET) {
			ret = -EINVAL;
			HS_ERR("Unknown accessory type");
			goto err_ext_hpin_accessory_type;
		}
	}

	wake_lock_init(&hi->hs_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_NAME);

	detect_wq = create_workqueue("HS_MISC_DETECT");
	if (detect_wq == NULL) {
		ret = -ENOMEM;
		HS_ERR("Failed to create detect workqueue");
		goto err_create_detect_work_queue;
	}

	hs_misc_register();
	hs_notify_driver_ready(DRIVER_NAME);

	HS_LOG("--------------------");

	return 0;

err_create_detect_work_queue:
err_ext_hpin_accessory_type:
	kfree(hi);

	HS_ERR("Failed to register %s driver", DRIVER_NAME);

	return ret;
}

static int htc_headset_misc_remove(struct platform_device *pdev)
{
	destroy_workqueue(detect_wq);

	kfree(hi);

	return 0;
}

static struct platform_driver htc_headset_misc_driver = {
	.probe		= htc_headset_misc_probe,
	.remove		= htc_headset_misc_remove,
	.driver		= {
		.name		= "HTC_HEADSET_MISC",
		.owner		= THIS_MODULE,
	},
};

static int __init htc_headset_misc_init(void)
{
	return platform_driver_register(&htc_headset_misc_driver);
}

static void __exit htc_headset_misc_exit(void)
{
	platform_driver_unregister(&htc_headset_misc_driver);
}

module_init(htc_headset_misc_init);
module_exit(htc_headset_misc_exit);

MODULE_DESCRIPTION("HTC MISC headset driver");
MODULE_LICENSE("GPL");
