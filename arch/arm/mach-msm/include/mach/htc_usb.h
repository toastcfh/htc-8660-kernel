/*
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
#ifndef __ASM_ARCH_MSM_HTC_USB_H
#define __ASM_ARCH_MSM_HTC_USB_H

#ifdef CONFIG_ARCH_QSD8X50
void msm_hsusb_8x50_phy_reset(void);
#endif

#ifdef CONFIG_USB_ANDROID
#ifdef ERROR
#undef ERROR
#endif
#include <linux/usb/android_composite.h>
#include <linux/usb/f_accessory.h>

#ifdef CONFIG_USB_ANDROID_USBNET
static char *usb_functions_usbnet[] = {
	"usbnet",
};

static char *usb_functions_usbnet_adb[] = {
	"usbnet",
	"adb",
};
#endif

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};
static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};
#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_rndis_diag[] = {
	"rndis",
	"diag",
};
static char *usb_functions_rndis_adb_diag[] = {
	"rndis",
	"adb",
	"diag",
};
#endif

#ifdef CONFIG_USB_ANDROID_ACCESSORY
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };
#endif

#ifdef CONFIG_USB_ANDROID_MTP
static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};
#endif

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_diag[] = {
	"usb_mass_storage",
	"diag",
};
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	"accessory",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id = 0x0c02, /* vary by board */
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_ACCESSORY
	{
		.vendor_id      = USB_ACCESSORY_VENDOR_ID,
		.product_id     = USB_ACCESSORY_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory),
		.functions      = usb_functions_accessory,
	},
	{
		.vendor_id      = USB_ACCESSORY_VENDOR_ID,
		.product_id     = USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_accessory_adb),
		.functions      = usb_functions_accessory_adb,
	},
#endif
#if defined(CONFIG_USB_ANDROID_DIAG) || defined(CONFIG_USB_ANDROID_QCT_DIAG)
	{
		.product_id	= 0x0c07,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
	{
		.product_id	= 0x0c08,
		.num_functions	= ARRAY_SIZE(usb_functions_diag),
		.functions	= usb_functions_diag,
	},
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	{
		.product_id	= 0x0ff4,
		.num_functions	= ARRAY_SIZE(usb_functions_acm),
		.functions	= usb_functions_acm,
	},
	{
		.product_id	= 0x0ff5,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_acm),
		.functions	= usb_functions_adb_acm,
	},
#endif
};
#endif

#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
/*
 * Verizon: 0x0e05 ~ 0x0e72
 *	Internet sharing(0)  and Internet pass through(1) both use rndis
 */
#ifdef CONFIG_MACH_VIGOR
static int vigor_usb_product_id_match_array[] = {
	0x0c93, 0x0e05, /* mtp */
	0x0ca8, 0x0e06, /* mtp + adb */
	0x0fda, 0x0e07, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e08, /* ums + 9k rmnet */
	0x0c07, 0x0e0b, /* ums + adb + diag */
	0x0c08, 0x0e0c, /* ums + diag */
	0x0ff8, 0x0e0d, /* CDC-ECM */
	0x0fd5, 0x0e0e, /* ums + diag + 9k diag + rmnet */
	0x0fd4, 0x0e0f, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0ccd, /* ums */
	-1,
};

static int vigor_usb_product_id_rndis[] = {
	0x0e09, 0x0e0a,
};
#endif /* CONFIG_MACH_VIGOR */

#ifdef CONFIG_MACH_VIVOW
static int vivow_usb_product_id_match_array[] = {
	0x0c93, 0x0e10, /* mtp */
	0x0ca8, 0x0e11, /* mtp + adb */
	0x0fda, 0x0e12, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e13, /* ums + 9k rmnet */
	0x0c07, 0x0e16, /* ums + adb + diag */
	0x0c08, 0x0e17, /* ums + diag */
	0x0ff8, 0x0e18, /* CDC-ECM */
	0x0fd5, 0x0e19, /* ums + diag + 9k diag + rmnet */
	0x0fd4, 0x0e1a, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0cad, /* ums */
	-1,
};

static int vivow_usb_product_id_rndis[] = {
	0x0e14, 0x0e15,
};
#endif /* CONFIG_MACH_VIVOW */

#ifdef CONFIG_MACH_MECHA
static int mecha_usb_product_id_match_array[] = {
	0x0c93, 0x0e1b, /* mtp */
	0x0ca8, 0x0e1c, /* mtp + adb */
	0x0fda, 0x0e1d, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e1e, /* ums + 9k rmnet */
	0x0c07, 0x0e21, /* ums + adb + diag */
	0x0c08, 0x0e22, /* ums + diag */
	0x0ff8, 0x0e23, /* CDC-ECM */
	0x0fd5, 0x0e24, /* ums + diag + 9k diag + rmnet */
	0x0fd4, 0x0e25, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0ca4, /* ums */
	-1,
};

static int mecha_usb_product_id_rndis[] = {
	0x0e1f, 0x0e20,
};
#endif /* CONFIG_MACH_MECHA */

#ifdef CONFIG_MACH_BLISSC
static int blissc_usb_product_id_match_array[] = {
	0x0c93, 0x0e26, /* mtp */
	0x0ca8, 0x0e27, /* mtp + adb */
	0x0fda, 0x0e28, /* ums + adb + 9k rmnet */
	0x0fdb, 0x0e29, /* ums + 9k rmnet */
	0x0c07, 0x0e2c, /* ums + adb + diag */
	0x0c08, 0x0e2d, /* ums + diag */
	0x0ff8, 0x0e2e, /* CDC-ECM */
	0x0fd5, 0x0e2f, /* ums + diag + 9k diag + rmnet */
	0x0fd4, 0x0e30, /* ums + adb + diag + 9k diag + rmnet */
	0x0ff9, 0x0ccb, /* ums */
	-1,
};

static int blissc_usb_product_id_rndis[] = {
	0x0e2a, 0x0e2b,
};
#endif /* CONFIG_MACH_BLISSC */
#endif /* CONFIG_USB_GADGET_VERIZON_PRODUCT_ID */

#endif
