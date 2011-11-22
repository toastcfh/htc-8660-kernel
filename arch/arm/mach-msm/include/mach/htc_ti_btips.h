/* arch/arm/mach-msm/include/mach/htc_fast_clk.h
 *
 * Copyright (C) 2011 HTC, Inc.
 * Author: assd bt <htc_ssdbt@htc.com>
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
struct msm_bt_btips_platform_data {
	void *conn_gpios;
	int (*bluetooth_set_power)(int);
};
