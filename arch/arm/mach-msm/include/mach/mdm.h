/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
 */

#ifndef _ARCH_ARM_MACH_MSM_MDM_H
#define _ARCH_ARM_MACH_MSM_MDM_H

struct charm_platform_data {
	void (*charm_modem_on)(void);
	void (*charm_modem_off)(void);
	void (*charm_modem_reset)(void);
	void (*charm_modem_suspend)(void);
	void (*charm_modem_resume)(void);

	unsigned gpio_ap2mdm_status;
	unsigned gpio_ap2mdm_wakeup;
	unsigned gpio_ap2mdm_errfatal;
	unsigned gpio_ap2mdm_sync;
	unsigned gpio_ap2mdm_pmic_reset_n;
	unsigned gpio_ap2mdm_kpdpwr_n;
	unsigned gpio_ap2pmic_tmpni_cken;

	unsigned gpio_mdm2ap_status;
	unsigned gpio_mdm2ap_wakeup;
	unsigned gpio_mdm2ap_errfatal;
	unsigned gpio_mdm2ap_sync;
	unsigned gpio_mdm2ap_vfr;
};

/* Added by HTC */
unsigned charm_get_MDM_error_flag(void);
/*---------------------------------------------*/
#endif
