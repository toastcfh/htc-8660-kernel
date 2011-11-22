/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_misc.h
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

#ifndef HTC_HEADSET_MISC_H
#define HTC_HEADSET_MISC_H

#define DRIVER_HS_MISC_EXT_HP_DET	(1 << 0)

struct htc_headset_misc_platform_data {
	unsigned int driver_flag;
	unsigned int ext_hpin_gpio;
	unsigned int ext_accessory_type;
	unsigned int indicator_power_gpio;
};

struct htc_headset_misc_info {
	struct htc_headset_misc_platform_data pdata;

	unsigned int ext_hpin_irq;
	unsigned int ext_hpin_irq_type;
	unsigned int ext_hpin_debounce;
	unsigned int ext_hpin_enable;
	struct wake_lock hs_wake_lock;

	void (*accessory_detect)(int);
};

int headset_indicator_enable(int enable);
void headset_mhl_audio_jack_enable(int enable);

#endif
