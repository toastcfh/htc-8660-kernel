/* arch/arm/mach-msm/board-doubleshot-keypad.c
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
#include <mach/irqs.h>
#include <mach/atmega_microp.h>

#include "board-doubleshot.h"
/*#include "proc_comm.h"*/

#include <linux/mfd/pmic8058.h>
#include <linux/input/pmic8058-keypad.h>

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_doubleshot."
module_param_named(keycaps, keycaps, charp, 0);

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			KEY_LOGE("[keypad]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static unsigned int doubleshot_pmic_col_gpios[] = {
	PM8058_GPIO_PM_TO_SYS(PMGPIO(1)), PM8058_GPIO_PM_TO_SYS(PMGPIO(2)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(3)), PM8058_GPIO_PM_TO_SYS(PMGPIO(4)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(5)), PM8058_GPIO_PM_TO_SYS(PMGPIO(6)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(7)),
};
static unsigned int doubleshot_pmic_row_gpios[] = {
	PM8058_GPIO_PM_TO_SYS(PMGPIO(9)), PM8058_GPIO_PM_TO_SYS(PMGPIO(10)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(11)), PM8058_GPIO_PM_TO_SYS(PMGPIO(12)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(13)), PM8058_GPIO_PM_TO_SYS(PMGPIO(14)),
	PM8058_GPIO_PM_TO_SYS(PMGPIO(15)),

};

#define KEYMAP_NUM_ROWS		ARRAY_SIZE(doubleshot_pmic_row_gpios)
#define KEYMAP_NUM_COLS		ARRAY_SIZE(doubleshot_pmic_col_gpios)
#define KEYMAP_INDEX(row, col)	(((row) * KEYMAP_NUM_COLS) + (col))
#define KEYMAP_SIZE		(KEYMAP_NUM_ROWS * KEYMAP_NUM_COLS)

static unsigned short doubleshot_pmic_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 2)] = KEY_WWW,
	[KEYMAP_INDEX(0, 3)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(0, 4)] = KEY_LEFTALT,
	[KEYMAP_INDEX(0, 5)] = KEY_EMAIL,
	[KEYMAP_INDEX(0, 6)] = KEY_F16, /*KEY_HOME*/

	[KEYMAP_INDEX(1, 0)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(1, 2)] = KEY_RIGHTSHIFT,
	[KEYMAP_INDEX(1, 3)] = KEY_P,
	[KEYMAP_INDEX(1, 4)] = KEY_RIGHTALT,
	[KEYMAP_INDEX(1, 5)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(1, 6)] = KEY_ENTER,

	[KEYMAP_INDEX(2, 0)] = KEY_Q,
	[KEYMAP_INDEX(2, 1)] = KEY_COMMA,
	[KEYMAP_INDEX(2, 2)] = KEY_A,
	[KEYMAP_INDEX(2, 3)] = KEY_X,
	[KEYMAP_INDEX(2, 4)] = KEY_F17, /*KEY_MENU*/
	[KEYMAP_INDEX(2, 5)] = KEY_S,
	[KEYMAP_INDEX(2, 6)] = KEY_Z,

	[KEYMAP_INDEX(3, 0)] = KEY_K,
	[KEYMAP_INDEX(3, 1)] = KEY_L,
	[KEYMAP_INDEX(3, 2)] = KEY_QUESTION,
	[KEYMAP_INDEX(3, 3)] = KEY_M,
	[KEYMAP_INDEX(3, 4)] = KEY_F13, /*SYM*/
	[KEYMAP_INDEX(3, 5)] = KEY_O,
	[KEYMAP_INDEX(3, 6)] = KEY_I,

	[KEYMAP_INDEX(4, 0)] = KEY_SPACE,
	[KEYMAP_INDEX(4, 1)] = KEY_D,
	[KEYMAP_INDEX(4, 2)] = KEY_W,
	[KEYMAP_INDEX(4, 3)] = KEY_E,
	[KEYMAP_INDEX(4, 4)] = KEY_V,
	[KEYMAP_INDEX(4, 5)] = KEY_F,
	[KEYMAP_INDEX(4, 6)] = KEY_C,

	[KEYMAP_INDEX(5, 0)] = KEY_N,
	[KEYMAP_INDEX(5, 1)] = KEY_F14, /*Genius*/
	[KEYMAP_INDEX(5, 2)] = KEY_U,
	[KEYMAP_INDEX(5, 3)] = KEY_Y,
	[KEYMAP_INDEX(5, 4)] = KEY_J,
	[KEYMAP_INDEX(5, 5)] = KEY_H,
	[KEYMAP_INDEX(5, 6)] = KEY_RESERVED,

	[KEYMAP_INDEX(6, 0)] = KEY_DOT,
	[KEYMAP_INDEX(6, 1)] = KEY_G,
	[KEYMAP_INDEX(6, 2)] = KEY_F15, /*KEY_BACK*/
	[KEYMAP_INDEX(6, 3)] = KEY_T,
	[KEYMAP_INDEX(6, 4)] = KEY_R,
	[KEYMAP_INDEX(6, 5)] = KEY_B,
	[KEYMAP_INDEX(6, 6)] = KEY_RESERVED,
};

struct pm8058_gpio matrix_sns_config = {
	.direction      = PM_GPIO_DIR_IN,
	.pull           = PM_GPIO_PULL_UP_31P5,
	.vin_sel        = PM_GPIO_VIN_S3,
	.out_strength   = PM_GPIO_STRENGTH_NO,
	.function       = PM_GPIO_FUNC_NORMAL,
};
static unsigned int doubleshot_sns_gpios[] = {
	PMGPIO(1), PMGPIO(2),
	PMGPIO(3), PMGPIO(4),
	PMGPIO(5), PMGPIO(6),
	PMGPIO(7),
};

static void doubleshot_setup_matrix_gpio(void)
{
	int rc = 0, id = 0;
	int index = 0;

	for (index = 0; index < KEYMAP_NUM_COLS; index++) {
		id = doubleshot_sns_gpios[index];
		rc = pm8058_gpio_config(id, &matrix_sns_config);
			if (rc)
				KEY_LOGE("%s: pm8058_gpio_config(%d): rc=%d\n",
					__func__, id, rc);
	}
}

static struct gpio_event_matrix_info doubleshot_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.info.no_suspend = true,
	.keymap = doubleshot_pmic_keymap,
	.output_gpios = doubleshot_pmic_row_gpios,
	.input_gpios = doubleshot_pmic_col_gpios,
	.noutputs = KEYMAP_NUM_ROWS,
	.ninputs = KEYMAP_NUM_COLS,
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.setup_matrix_gpio = doubleshot_setup_matrix_gpio,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS | GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry doubleshot_keypad_input_map[] = {
	{
		.gpio = DOUBLESHOT_GPIO_KEY_POWER,
		.code = KEY_POWER,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(DOUBLESHOT_VOL_UP),
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = PM8058_GPIO_PM_TO_SYS(DOUBLESHOT_VOL_DN),
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = DOUBLESHOT_GPIO_KEY_CAM_STEP1,
		.code = KEY_HP,
	},
	{
		.gpio = DOUBLESHOT_GPIO_KEY_CAM_STEP2,
		.code = KEY_CAMERA,
	},
};

static void doubleshot_setup_input_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		GPIO_CFG(DOUBLESHOT_GPIO_KEY_POWER, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		GPIO_CFG(DOUBLESHOT_GPIO_KEY_CAM_STEP1, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		GPIO_CFG(DOUBLESHOT_GPIO_KEY_CAM_STEP2, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_input_info doubleshot_keypad_input_info = {
	.info.func = gpio_event_input_func,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.keymap = doubleshot_keypad_input_map,
	.keymap_size = ARRAY_SIZE(doubleshot_keypad_input_map),
	.setup_input_gpio = doubleshot_setup_input_gpio,
};

static void doubleshot_set_qty_irq(uint8_t disable)
{
	uint32_t i;
	static uint8_t already_disabled;
	KEY_LOGI("%s disable=%d, already_disabled=%d\n",
			__func__, disable, already_disabled);

	if (!(disable ^ already_disabled))
		return;

	already_disabled = disable;
	for (i = 0; i < KEYMAP_NUM_COLS; i++) {
		if (disable)
			disable_irq(gpio_to_irq(doubleshot_pmic_col_gpios[i]));
		else
			enable_irq(gpio_to_irq(doubleshot_pmic_col_gpios[i]));
	}
}

static struct gpio_event_direct_entry doubleshot_sliding_switch[] = {
	{
		.gpio = DOUBLESHOT_GPIO_KEY_SLID_INT,
		.code = SW_LID,
	},
};

static void doubleshot_setup_sliding_gpio(void)
{
	uint32_t inputs_gpio_table[] = {
		GPIO_CFG(DOUBLESHOT_GPIO_KEY_SLID_INT, 0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	};

	config_gpio_table(inputs_gpio_table, ARRAY_SIZE(inputs_gpio_table));
}

static struct gpio_event_switch_info doubleshot_keypad_switch_info = {
	.info.func = gpio_event_switch_func,
	.info.no_suspend = true,
#if 1
	.flags = GPIOEDF_PRINT_KEYS,
#else
	.flags = GPIOKPF_ACTIVE_HIGH,
	/* FIXME: since sliding is opened on Latte barebone system,
	 * set the active direction inverse on purpose
	 * just for our convenience in current stage.
	 */
#endif
	.type = EV_SW,
	.keymap = doubleshot_sliding_switch,
	.keymap_size = ARRAY_SIZE(doubleshot_sliding_switch),
	.setup_switch_gpio = doubleshot_setup_sliding_gpio,
	.set_qty_irq = doubleshot_set_qty_irq,
};

#ifdef CONFIG_INPUT_MICROP
static struct gpio_event_direct_entry doubleshot_microp_key_map[] = {
	{
		.code = KEY_MENU,
		.gpio = 1 << 2,
	},
	{
		.code = KEY_HOME,
		.gpio = 1 << 4,
	},
	{
		.code = KEY_F18,/*Geninus key*/
		.gpio = 1 << 0,
	},
	{
		.code = KEY_BACK,
		.gpio = 1 << 7,
	},
	{
		.code = BTN_MOUSE,
		.gpio = 1 << 8,
	},
};

static struct gpio_event_microp_info doubleshot_keypad_microp_info = {
	.info.func = gpio_event_microp_func,
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	.info.oj_btn = true,
#endif
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.keymap = doubleshot_microp_key_map,
	.keymap_size = ARRAY_SIZE(doubleshot_microp_key_map),
	.irq = MSM_uP_TO_INT(0),
	.microp_info = 0x53, /*project code*/
};
#endif

static struct gpio_event_info *doubleshot_keypad_info[] = {
	&doubleshot_keypad_input_info.info,
	&doubleshot_keypad_switch_info.info,
	&doubleshot_keypad_matrix_info.info,
#ifdef CONFIG_INPUT_MICROP
	&doubleshot_keypad_microp_info.info,
#endif
};

static struct gpio_event_platform_data doubleshot_keypad_data = {
	.names = {
		"doubleshot-keypad",
		NULL,
	},
	.info = doubleshot_keypad_info,
	.info_count = ARRAY_SIZE(doubleshot_keypad_info),
};

static struct platform_device doubleshot_keypad_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &doubleshot_keypad_data,
	},
};

static int doubleshot_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data doubleshot_reset_keys_pdata = {
	.keys_up = doubleshot_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device doubleshot_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &doubleshot_reset_keys_pdata,
};

int __init doubleshot_init_keypad(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	if (platform_device_register(&doubleshot_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	if ((system_rev == 0) && !(doubleshot_get_engineerid() & 0x2))
		doubleshot_keypad_data.name = "doubleshot-keypad-v0";

	return platform_device_register(&doubleshot_keypad_input_device);
}
