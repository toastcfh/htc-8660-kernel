/* include/linux/mhl.h
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

#ifndef _MHL_H_
#define _MHL_H_
//********************************************************************
//  Nested Include Files
//********************************************************************

//********************************************************************
//  Manifest Constants
//********************************************************************
#define MHL_SII9234_I2C_NAME   "sii9234"

//********************************************************************
//  Type Definitions
//********************************************************************
typedef enum
{
	E_MHL_FALSE = 0,
	E_MHL_TRUE  = 1,
	E_MHL_OK    = 0,
	E_MHL_FAIL  = 1,
	E_MHL_NA    = 0xFF
}T_MHL_RV;

typedef struct
{
	int gpio_intr;
	int gpio_reset;
	int ci2ca;
	void (*mhl_usb_switch)(int);
	void (*mhl_1v2_power)(bool enable);
	int (*power)(int);
}T_MHL_PLATFORM_DATA;
//********************************************************************
//  Define & Macro
//********************************************************************
#define M_MHL_SEND_DEBUG(x...) pr_info(x)
//********************************************************************
//  Prototype & Extern variable, function
//********************************************************************
#endif/*_MHL_H_*/

