/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */


#ifndef _SQN_FIRMWARE_H
#define _SQN_FIRMWARE_H

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
#include <linux/autoconf.h>
#else
#include <generated/autoconf.h>
#endif

//#define SQN_UNIQUE_FIRMWARE	//Flag to enable SQN_UNIQUE_FIRMWARE emchanism

#ifdef SQN_UNIQUE_FIRMWARE

#define SQN_DEFAULT_BOOTROM_NAME	"../../../data/wimax/Boot.bin"
#define SQN_DEFAULT_STARTUP_SCRIPT_NAME	"../../../data/wimax/startup-script.txt"
#define SQN_DEFAULT_FW_NAME	"/data/wimax/default.fw"
//Error handling for load default.fw fail.
#define SQN_DEFAULT_FW_NAME_RECOVERY	"/data/wimax/app_default.fw"

extern char *bootrom1130_name;
extern char *fw1130_name;
extern char *bootrom1210_name;
extern char *fw1210_name;

#else
#define SQN_DEFAULT_FW_NAME	"sequans_boot.bin"
extern char *fw1130_name;
extern char *fw1210_name;
#endif

int sqn_load_firmware(struct sdio_func *func);

#endif /* _SQN_FIRMWARE_H */
