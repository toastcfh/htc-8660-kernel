/* drivers/input/misc/gpio_microp.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
#include <linux/curcial_oj.h>
#endif
#include <mach/atmega_microp.h>

static struct workqueue_struct *gpio_microp_wq;
static struct wake_lock gpio_microp_wake_lock;

struct microp_key_state {
	struct microp_input_state *ds;
};

struct microp_input_state {
	struct gpio_event_input_devs *input_devs;
	const struct gpio_event_microp_info *info;
	spinlock_t irq_lock;
	struct microp_key_state key_state;
	struct work_struct work;
};

static int check_microp_info(uint8_t info)
{
	int err = 0;
	uint8_t data[2];

	memset(data, 0x00, sizeof(uint8_t)*2);
	err = microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
	if (err)
		return err;
	pr_info("%s: rev=0x%x proj=0x%x\n", __func__, data[0], data[1]);

	if (data[0] <= 0)
		return -EINVAL;
	if (data[1] != info)
		return -EINVAL;
	return err;
}
static uint8_t read_microp_gpio(uint8_t *data, uint32_t gpio)
{
	uint32_t temp = (data[0]<<16) + (data[1]<<8) + data[2];
	return !(temp & gpio);
}
static void microp_scan_key(struct microp_input_state *ds)
{
	const struct gpio_event_direct_entry *key_entry;
	uint8_t pressed = 0;
	uint8_t data[3];
	int i = 0;

	memset(data, 0x00, sizeof(uint8_t)*3);
	microp_i2c_read(MICROP_I2C_RCMD_GPIO_STATUS, data, 3);

	key_entry = ds->info->keymap;
	for (i = 0; i < ds->info->keymap_size; i++) {
		pressed = read_microp_gpio(data, key_entry[i].gpio);
		pr_info("microp_keys_scan_keys: key %d-%d, %d (%d) "
			"changed to %d\n", ds->info->type,
			key_entry[i].code, i, key_entry[i].gpio, pressed);
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
		if (ds->info->info.oj_btn && key_entry->code == BTN_MOUSE) {
			curcial_oj_send_key(BTN_MOUSE, pressed);
		} else {
#endif
			input_event(ds->input_devs->dev[key_entry[i].dev],
			ds->info->type, key_entry[i].code, pressed);
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
		}
#endif
	}
}
static int report_key_event(struct microp_input_state *ds, uint8_t keycode)
{
	uint8_t pressed = !(keycode & 0x80);
	uint8_t keymap_index = (keycode & 0xF) - 1;
	const struct gpio_event_direct_entry *key_entry;
	uint32_t err = 0;

	key_entry = &ds->info->keymap[keymap_index];
	pr_info("microp_keys_scan_keys: key %d-%d, %d (%d) "
		"changed to %d\n", ds->info->type, key_entry->code,
		keymap_index, key_entry->gpio, pressed);
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	if (ds->info->info.oj_btn && key_entry->code == BTN_MOUSE)
		curcial_oj_send_key(BTN_MOUSE, pressed);
	else
#endif
		input_event(ds->input_devs->dev[key_entry->dev],
		ds->info->type, key_entry->code, pressed);

	return err;
}
static void gpio_event_check_func(struct work_struct *work)
{
	struct microp_input_state *ds =
		container_of(work, struct microp_input_state, work);
	uint8_t data[3];

	memset(data, 0x00, sizeof(uint8_t)*3);
	microp_i2c_read(MICROP_I2C_RCMD_AP_KEY_CODE, data, 2);

	report_key_event(ds, data[1]);
	wake_unlock(&gpio_microp_wake_lock);
}
static irqreturn_t gpio_microp_irq_handler(int irq, void *dev_id)
{
	struct microp_key_state *ks = dev_id;
	struct microp_input_state *ds = ks->ds;

	wake_lock(&gpio_microp_wake_lock);
	queue_work(gpio_microp_wq, &ds->work);

	return IRQ_HANDLED;
}

static int gpio_microp_request_irqs(struct microp_input_state *ds)
{
	int err;
	unsigned int irq;
	unsigned long req_flags = IRQF_TRIGGER_NONE;

	err = irq = ds->info->irq;
	if (err < 0)
		goto err_gpio_get_irq_num_failed;

	err = request_irq(irq, gpio_microp_irq_handler,
				req_flags, "microp_keys", &ds->key_state);
	if (err) {
		pr_err("[GPIO MICROP] gpio_event_input_request_irqs: "
			"request_irq failed for irq %d\n", irq);
		goto err_request_irq_failed;
	}
	enable_irq_wake(irq);

	return 0;

	free_irq(irq, &ds->key_state);
err_request_irq_failed:
err_gpio_get_irq_num_failed:
	return err;
}

int gpio_event_microp_func(struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info, void **data, int func)
{
	int ret = 0;
	int i;

	struct gpio_event_microp_info *di;
	struct microp_input_state *ds = *data;

	di = container_of(info, struct gpio_event_microp_info, info);

	if (func == GPIO_EVENT_FUNC_SUSPEND) {
		return 0;
	}
	if (func == GPIO_EVENT_FUNC_RESUME) {
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
		*data = ds = kzalloc(sizeof(*ds) +
				sizeof(ds->key_state), GFP_KERNEL);
		if (ds == NULL) {
			ret = -ENOMEM;
			pr_err("[GPIO MICROP] gpio_event_microp_func: "
				"Failed to allocate private data\n");
			goto err_ds_alloc_failed;
		}

		ds->input_devs = input_devs;
		ds->info = di;
		wake_lock_init(&gpio_microp_wake_lock, WAKE_LOCK_SUSPEND, "gpio_microp");
		spin_lock_init(&ds->irq_lock);

		for (i = 0; i < di->keymap_size; i++) {
			int dev = di->keymap[i].dev;
			if (dev >= input_devs->count) {
				pr_err("gpio_event_microp_func: bad device "
					"index %d >= %d for key code %d\n",
					dev, input_devs->count,
					di->keymap[i].code);
				ret = -EINVAL;
				goto err_bad_keymap;
			}
			input_set_capability(input_devs->dev[dev], di->type,
					     di->keymap[i].code);
		}
		ds->key_state.ds = ds;

		INIT_WORK(&ds->work, gpio_event_check_func);
		gpio_microp_wq = create_singlethread_workqueue("gpio_microp_wq");

		ret = check_microp_info(ds->info->microp_info);
		if (ret)
			goto err_microp_info;

		ret = gpio_microp_request_irqs(ds);
		if (ret)
			goto err_microp_info;

		pr_info("GPIO Input Driver: Start gpio microp for %s%s in %s "
			"mode\n", input_devs->dev[0]->name,
			(input_devs->count > 1) ? "..." : "",
			ret == 0 ? "interrupt" : "polling");

		microp_scan_key(ds);

		return 0;
	}

err_microp_info:
	destroy_workqueue(gpio_microp_wq);
err_bad_keymap:
	wake_lock_destroy(&gpio_microp_wake_lock);
	kfree(ds);
err_ds_alloc_failed:
	return ret;
}
