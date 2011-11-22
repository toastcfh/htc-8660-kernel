/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _ASM_ARCH_MSM_RESTART_H_
#define _ASM_ARCH_MSM_RESTART_H_

#define RESTART_NORMAL 0x0
#define RESTART_DLOAD  0x1

void msm_set_restart_mode(int mode);

/* if arch_reset is called from userspace,
   restart mode will be set to 'h' equal to 104.
   As a result, we need MAX to know the mode is valid. */
enum RESTART_MODE {
	/* for legecy cmd restart */
	RESTART_MODE_LEGECY = 0,

	/* all other restart rised by kernel.
	   these modes will all enter ramdump. */
	RESTART_MODE_Q6_WATCHDOG_BITE,

	RESTART_MODE_MODEM_CRASH,
	RESTART_MODE_MODEM_USER_INVOKED,
	RESTART_MODE_MODEM_UNWEDGE_TIMEOUT,
	RESTART_MODE_MODEM_WATCHDOG_BITE,
	RESTART_MODE_MODEM_ERROR_FATAL,

	RESTART_MODE_MDM_DOG_BITE,
	RESTART_MODE_MDM_FATAL,

	RESTART_MODE_APP_WATCHDOG_BARK,
	/* This is pseudo enum to indicate the maximum,
	   add new restart mode before this one. */
	RESTART_MODE_MAX
};

void set_ramdump_reason(const char *msg);
inline void soc_restart(char mode, const char *msg);
inline void notify_modem_cache_flush_done(void);
int check_in_panic(void);
#endif
