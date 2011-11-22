/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2011 HTC Corporation.
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include <mach/htc_sleep_clk.h>
#include <mach/htc_fast_clk.h>

#include "board-ruby.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "ti1273L";

/* bt on configuration */
static uint32_t ruby_bt_on_table[] = {

	/* BT_RTS */
	GPIO_CFG(RUBY_GPIO_BT_UART1_RTS,
				1,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),
	/* BT_CTS */
	GPIO_CFG(RUBY_GPIO_BT_UART1_CTS,
				1,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_RX */
	GPIO_CFG(RUBY_GPIO_BT_UART1_RX,
				1,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_TX */
	GPIO_CFG(RUBY_GPIO_BT_UART1_TX,
				1,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),

	/* BT_EN */
	GPIO_CFG(RUBY_GPIO_BT_EN,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),

};

/* bt off configuration */
static uint32_t ruby_bt_off_table[] = {

	/* BT_RTS */
	GPIO_CFG(RUBY_GPIO_BT_UART1_RTS,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),
	/* BT_CTS */
	GPIO_CFG(RUBY_GPIO_BT_UART1_CTS,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_RX */
	GPIO_CFG(RUBY_GPIO_BT_UART1_RX,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	/* BT_TX */
	GPIO_CFG(RUBY_GPIO_BT_UART1_TX,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),

	/* BT_EN */
	GPIO_CFG(RUBY_GPIO_BT_EN,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_4MA),

};

static void config_bt_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[BT]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void ruby_config_bt_on(void)
{
	/* set bt on configuration*/
	config_bt_table(ruby_bt_on_table,
				ARRAY_SIZE(ruby_bt_on_table));
	mdelay(4);

	/* control osc */
	htc_wifi_bt_fast_clk_ctl(CLK_ON, ID_BT);
	mdelay(4);

	gpio_set_value(RUBY_GPIO_BT_EN, 0);
	mdelay(4);
	/* BT_EN */
	gpio_set_value(RUBY_GPIO_BT_EN, 1);
	//mdelay(200);

}

static void ruby_config_bt_off(void)
{
	/* BT_EN */
	gpio_set_value(RUBY_GPIO_BT_EN, 0);
	mdelay(4);

	/* control osc */
	htc_wifi_bt_fast_clk_ctl(CLK_OFF, ID_BT);
	mdelay(4);

	/* set bt off configuration*/
	config_bt_table(ruby_bt_off_table,
				ARRAY_SIZE(ruby_bt_off_table));
	mdelay(4);

	/* BT_RTS */
	gpio_set_value(RUBY_GPIO_BT_UART1_RTS, 1);

	mdelay(4);
	/* BT_CTS */

	/* BT_TX */
	gpio_set_value(RUBY_GPIO_BT_UART1_TX, 1);

	mdelay(4);
	/* BT_RX */

}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		ruby_config_bt_on();
	else
		ruby_config_bt_off();

	return 0;
}

/* Export an interface for the other drivers */
int ruby_bluetooth_set_power(int on)
{
	if (on)
		bluetooth_set_power(NULL, false);
	else
		bluetooth_set_power(NULL, true);

	return 0;
}

static struct rfkill_ops ruby_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int ruby_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	rc = gpio_request(RUBY_GPIO_BT_EN, "bt_en");
	if (rc)
		goto err_gpio_en;

	/* always turn on clock? */
	htc_wifi_bt_sleep_clk_ctl(CLK_ON, ID_BT);
	mdelay(2);

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&ruby_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	gpio_free(RUBY_GPIO_BT_EN);
err_gpio_en:
	return rc;
}

static int ruby_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	gpio_free(RUBY_GPIO_BT_EN);

	return 0;
}

static struct platform_driver ruby_rfkill_driver = {
	.probe = ruby_rfkill_probe,
	.remove = ruby_rfkill_remove,
	.driver = {
		.name = "ruby_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init ruby_rfkill_init(void)
{
	if (!machine_is_ruby())
		return 0;

	return platform_driver_register(&ruby_rfkill_driver);
}

static void __exit ruby_rfkill_exit(void)
{
	platform_driver_unregister(&ruby_rfkill_driver);
}

module_init(ruby_rfkill_init);
module_exit(ruby_rfkill_exit);
MODULE_DESCRIPTION("ruby rfkill");
MODULE_AUTHOR("htc_ssdbt <htc_ssdbt@htc.com>");
MODULE_LICENSE("GPL");
