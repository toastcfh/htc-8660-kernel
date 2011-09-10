/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <asm/atomic.h>

#include <mach/msm_iomap.h>
#include <mach/board_htc.h>
#include <mach/scm-io.h>
#include "pm.h"
#include "smd_private.h"
#include <mach/restart.h>
#include <mach/mdm.h>
#if defined(CONFIG_MSM_RMT_STORAGE_CLIENT)
#include <linux/rmt_storage_client-8x60.h>
#endif

#define TCSR_WDT_CFG 0x30

#define WDT0_RST       (MSM_TMR0_BASE + 0x38)
#define WDT0_EN        (MSM_TMR0_BASE + 0x40)
#define WDT0_BARK_TIME (MSM_TMR0_BASE + 0x4C)
#define WDT0_BITE_TIME (MSM_TMR0_BASE + 0x5C)

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

static int restart_mode;

struct htc_reboot_params {
	unsigned reboot_reason;
	unsigned radio_flag;
	char reserved[256 - SZ_DIAG_ERR_MSG - 8];
	char msg[SZ_DIAG_ERR_MSG];
};

static struct htc_reboot_params *reboot_params;

static int in_panic;

int check_in_panic(void)
{
	return in_panic;
}

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static atomic_t restart_counter = ATOMIC_INIT(0);
static int modem_cache_flush_done;

inline void notify_modem_cache_flush_done(void)
{
	modem_cache_flush_done = 1;
}

static inline unsigned get_restart_reason(void)
{
	return reboot_params->reboot_reason;
}
/*
   This function should not be called outside
   to ensure that others do not change restart reason.
   Use mode & cmd to set reason & msg in arch_reset().
*/
static inline void set_restart_reason(unsigned int reason)
{
	reboot_params->reboot_reason = reason;
}

/*
   This function should not be called outsite
   to ensure that others do no change restart reason.
   Use mode & cmd to set reason & msg in arch_reset().
*/
static inline void set_restart_msg(const char *msg)
{
	strncpy(reboot_params->msg, msg, sizeof(reboot_params->msg)-1);
}

/* This function expose others to restart message for entering ramdump mode. */
void set_ramdump_reason(const char *msg)
{
	/* only allow write msg before entering arch_rest */
	if (atomic_read(&restart_counter) != 0)
		return;

	set_restart_reason(RESTART_REASON_RAMDUMP);
	set_restart_msg(msg? msg: "");
}

static void msm_power_off(void)
{
#if defined(CONFIG_MSM_RMT_STORAGE_CLIENT)
	set_restart_reason(RESTART_REASON_POWEROFF);

	/* final efs_sync */
	printk(KERN_INFO "from %s\r\n", __func__);
	writel(1, WDT0_RST);
	writel(0, WDT0_EN);
	writel(32768 * 12, WDT0_BARK_TIME);
	writel(32768 * 12, WDT0_BITE_TIME);
	writel(3, WDT0_EN);
	dsb();
	wait_rmt_final_call_back(10);
	/* Rest watchdog timer to make sure we have enough time to power off */
	writel(1, WDT0_RST);
	printk(KERN_INFO "back %s\r\n", __func__);
#endif
	printk(KERN_NOTICE "Powering off the SoC\n");
	pm8058_reset_pwr_off(0);
	pm8901_reset_pwr_off(0);
	secure_writel(0, PSHOLD_CTL_SU);
	mdelay(10000);
	printk(KERN_ERR "Powering off has failed\n");
	return;
}

static bool console_flushed;

static void msm_pm_flush_console(void)
{
	if (console_flushed)
		return;
	console_flushed = true;

	printk("\n");
	printk(KERN_EMERG "Restarting %s\n", linux_banner);
	if (!try_acquire_console_sem()) {
		release_console_sem();
		return;
	}

	mdelay(50);

	local_irq_disable();
	if (try_acquire_console_sem())
		printk(KERN_EMERG "restart: Console was locked! Busting\n");
	else
		printk(KERN_EMERG "restart: Console was locked!\n");
	release_console_sem();
}

/* It seems that modem would like to lock kernel before restarting the system. */
inline void soc_restart(char mode, const char *cmd)
{
	lock_kernel();
	arm_pm_restart(mode, cmd);
	unlock_kernel();
}

void arch_reset(char mode, const char *cmd)
{
	/* arch_reset should only enter once*/
	if(atomic_add_return(1, &restart_counter) != 1)
		return;

	printk(KERN_NOTICE "%s: Going down for restart now.\n", __func__);
	printk(KERN_NOTICE "%s: mode %d\n", __func__, mode);
	if (cmd) {
		printk(KERN_NOTICE "%s: restart command `%s'.\n", __func__, cmd);
		/* XXX: modem will set msg itself.
		        Dying msg should be passed to this function directly. */
		if (mode != RESTART_MODE_MODEM_CRASH)
			set_restart_msg(cmd);
	}
	else
		printk(KERN_NOTICE "%s: no command restart.\n", __func__);

	if (in_panic) {
		set_restart_msg("Kernel panic");
	} else if (!cmd) {
		set_restart_reason(RESTART_REASON_REBOOT);
	} else if (!strcmp(cmd, "bootloader")) {
		set_restart_reason(RESTART_REASON_BOOTLOADER);
	} else if (!strcmp(cmd, "recovery")) {
		set_restart_reason(RESTART_REASON_RECOVERY);
	} else if (!strcmp(cmd, "eraseflash")) {
		set_restart_reason(RESTART_REASON_ERASE_FLASH);
	} else if (!strncmp(cmd, "oem-", 4)) {
		unsigned long code;
		code = simple_strtoul(cmd + 4, 0, 16) & 0xff;

		/* oem-97, 98, 99 are RIL fatal */
		if ((code == 0x97) || (code == 0x98))
			code = 0x99;

		set_restart_reason(RESTART_REASON_OEM_BASE | code);
	} else if (!strcmp(cmd, "force-hard") ||
			(RESTART_MODE_LEGECY < mode && mode < RESTART_MODE_MAX)
		) {
		/* The only situation modem user triggers reset is NV restore after erasing EFS. */
		if (mode == RESTART_MODE_MODEM_USER_INVOKED)
			set_restart_reason(RESTART_REASON_REBOOT);
		else
			set_restart_reason(RESTART_REASON_RAMDUMP);
	} else {
		/* unknown command */
		set_restart_reason(RESTART_REASON_REBOOT);
	}

#if defined(CONFIG_MSM_RMT_STORAGE_CLIENT)
	/* if modem crashed, do not sync efs. */
	if (mode != RESTART_MODE_MODEM_CRASH &&
		mode != RESTART_MODE_MODEM_UNWEDGE_TIMEOUT &&
		mode != RESTART_MODE_MODEM_WATCHDOG_BITE &&
		mode != RESTART_MODE_MODEM_ERROR_FATAL &&
		mode != RESTART_MODE_APP_WATCHDOG_BARK
		) {
		/* final efs_sync */
		printk(KERN_INFO "from %s\r\n", __func__);

		/* efs sync timeout is 10 seconds.
		   set watchdog to 12 seconds. */
		writel(1, WDT0_RST);
		writel(0, WDT0_EN);
		writel(32768 * 12, WDT0_BARK_TIME);
		writel(32768 * 13, WDT0_BITE_TIME);
		writel(3, WDT0_EN);
		dsb();
		if (in_panic) {
			rmt_storage_set_msm_client_status(0);
			smsm_change_state(SMSM_APPS_STATE, SMSM_APPS_REBOOT, SMSM_APPS_REBOOT);
		} else {
			wait_rmt_final_call_back(10);
		}
		printk(KERN_INFO "back %s\r\n", __func__);
	}
#endif

	switch (get_restart_reason()) {
		case RESTART_REASON_RIL_FATAL:
		case RESTART_REASON_RAMDUMP:
			if (!in_panic && mode != RESTART_MODE_APP_WATCHDOG_BARK) {
				/* Suspend wdog until all stacks are printed */
				msm_watchdog_suspend();
				dump_stack();
				show_state_filter(TASK_UNINTERRUPTIBLE);
				print_workqueue();
				msm_watchdog_resume();
			}
			break;
	}

	/* for kernel panic & ril fatal, kenrel needs waiting modem flushing caches at most 10 seconds. */
	if (in_panic || get_restart_reason() == RESTART_REASON_RIL_FATAL) {
		int timeout = 10;
		printk(KERN_INFO "%s: wait for modem flushing caches.\n", __func__);
		while (timeout > 0 && !modem_cache_flush_done) {
			/* Kick watchdog.
			   Do not assume efs sync will be executed.
			   Assume watchdog timeout is default 4 seconds. */
			writel(1, WDT0_RST);

			mdelay(1000);
			timeout--;
		}
		if (timeout <= 0)
			printk(KERN_NOTICE "%s: modem flushes cache timeout.\n", __func__);
	}

	msm_pm_flush_console();

	secure_writel(0, PSHOLD_CTL_SU); /* Actually reset the chip */
	mdelay(5000);

	printk(KERN_NOTICE "PS_HOLD didn't work, falling back to watchdog\n");

	writel(1, WDT0_RST);
	writel(0, WDT0_EN);
	writel(0x31F3, WDT0_BARK_TIME);
	writel(0x31F3, WDT0_BITE_TIME);
	writel(3, WDT0_EN);
	dsb();
	secure_writel(3, MSM_TCSR_BASE + TCSR_WDT_CFG);

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

static int __init msm_restart_init(void)
{
	modem_cache_flush_done = 0;
	reboot_params = (void *)MSM_REBOOT_REASON_BASE;
	arm_pm_restart = arch_reset;

	//clear reboot parameter
	memset(reboot_params, 0x0, sizeof(struct htc_reboot_params));
	set_restart_reason(RESTART_REASON_RAMDUMP);
	reboot_params->radio_flag = get_radio_flag();

#ifdef CONFIG_MSM_DLOAD_MODE
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
#endif

	pm_power_off = msm_power_off;

	return 0;
}

late_initcall(msm_restart_init);
