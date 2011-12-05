/* arch/arm/mach-msm/board-vision-microp.c
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
/*#ifdef CONFIG_MICROP_COMMON*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/atmega_microp.h>

#include "board-doubleshot.h"

static int doubleshot_microp_function_init(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	int ret;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	/* Reset button interrupt */
	ret = microp_write_interrupt(client, (1<<8), 1);
	if (ret)
		goto exit;

	/* App key */
	ret = microp_function_check(client, MICROP_FUNCTION_KEY);
	if (ret >= 0) {
		pdata->function_node[MICROP_FUNCTION_KEY] = ret;
		cdata->int_pin.int_appkey =
			pdata->microp_function[ret].int_pin;

		ret = microp_write_interrupt(client,
				cdata->int_pin.int_appkey, 1);
		if (ret)
			goto exit;
	}

	return 0;

exit:
	return ret;
}

static struct microp_ops ops = {
	.init_microp_func = doubleshot_microp_function_init,
};

void __init doubleshot_microp_init(void)
{
	microp_register_ops(&ops);
}

/*#endif*/
