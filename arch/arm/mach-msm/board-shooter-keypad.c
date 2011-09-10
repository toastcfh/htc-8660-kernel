/* arch/arm/mach-msm/board-shooter-keypad.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/gpio.h>

#include "board-shooter.h"
/*#include "proc_comm.h"*/

#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_shooter."
module_param_named(keycaps, keycaps, charp, 0);

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[keypad]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static struct gpio_event_direct_entry shooter_keypad_input_map[] = {
	{
		.gpio = SHOOTER_GPIO_KEY_POWER,
		.code = KEY_POWER,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_CAM_STEP1,
		.code = KEY_HP,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_CAM_STEP2,
		.code = KEY_CAMERA,
	},
};

static void shooter_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		GPIO_CFG(SHOOTER_GPIO_KEY_POWER, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_VOL_UP, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_VOL_DOWN, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_CAM_STEP1, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_CAM_STEP2, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_direct_entry shooter_keypad_input_map_xb[] = {
	{
		.gpio = SHOOTER_GPIO_KEY_POWER,
		.code = KEY_POWER,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_CAM_STEP1_XB,
		.code = KEY_HP,
	},
	{
		.gpio = SHOOTER_GPIO_KEY_CAM_STEP2,
		.code = KEY_CAMERA,
	},
};

static void shooter_setup_input_gpio_xb(void)
{
	uint32_t inputs_gpio_table[] = {
		GPIO_CFG(SHOOTER_GPIO_KEY_POWER, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_VOL_UP, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_VOL_DOWN, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_CAM_STEP1_XB, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_KEY_CAM_STEP2, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info shooter_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.keymap = shooter_keypad_input_map,
	.keymap_size = ARRAY_SIZE(shooter_keypad_input_map),
	.setup_input_gpio = shooter_setup_input_gpio,
};

static struct gpio_event_direct_entry shooter_gpio_switch[] = {
	{
		.gpio = SHOOTER_GPIO_SW_LCM_3D,
		.code = SW_CAM,
		.not_wakeup_src = true,
	},
};

static void shooter_setup_switch_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		GPIO_CFG(SHOOTER_GPIO_SW_LCM_3D, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		GPIO_CFG(SHOOTER_GPIO_SW_LCM_2D, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_switch_info shooter_keypad_switch_info = {
	.info.func = gpio_event_switch_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_SW,
	.keymap = shooter_gpio_switch,
	.keymap_size = ARRAY_SIZE(shooter_gpio_switch),
	.setup_switch_gpio = shooter_setup_switch_gpio,
};

static struct gpio_event_info *shooter_keypad_info[] = {
	&shooter_keypad_input_info.info,
	&shooter_keypad_switch_info.info,
};

static struct gpio_event_platform_data shooter_keypad_data = {
	.names = {
		"shooter-keypad",
		NULL,
	},
	.info = shooter_keypad_info,
	.info_count = ARRAY_SIZE(shooter_keypad_info),
};

static struct platform_device shooter_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &shooter_keypad_data,
	},
};
/*
static int shooter_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};
*/
static struct keyreset_platform_data shooter_reset_keys_pdata = {
	/*.keys_up = shooter_reset_keys_up,*/
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

struct platform_device shooter_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &shooter_reset_keys_pdata,
};

int __init shooter_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	if (platform_device_register(&shooter_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	if (system_rev > 0) {
		shooter_keypad_input_info.keymap = shooter_keypad_input_map_xb;
		shooter_keypad_input_info.keymap_size =
				ARRAY_SIZE(shooter_keypad_input_map_xb);
		shooter_keypad_input_info.setup_input_gpio =
				shooter_setup_input_gpio_xb;
	}
	return platform_device_register(&shooter_keypad_input_device);
}
