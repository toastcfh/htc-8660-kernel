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
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <mach/clk.h>
#include <mach/dal_axi.h>
#include <mach/msm_bus.h>
#include <mach/msm_iomap.h>

#include "kgsl.h"
#include "kgsl_log.h"

#define SWITCH_OFF		200
#define TZ_UPDATE_ID	0x01404000
#define TZ_RESET_ID	0x01403000

#ifdef CONFIG_MSM_SECURE_IO
/* Trap into the TrustZone, and call funcs there. */
static int __secure_tz_entry(u32 cmd, u32 val)
{
	register u32 r0 asm("r0") = cmd;
	register u32 r1 asm("r1") = 0x0;
	register u32 r2 asm("r2") = val;

	__iowmb();
	asm(
		__asmeq("%0", "r0")
		__asmeq("%1", "r0")
		__asmeq("%2", "r1")
		__asmeq("%3", "r2")
		"smc    #0      @ switch to secure world\n"
		: "=r" (r0)
		: "r" (r0), "r" (r1), "r" (r2)
		);
	return r0;
}
#else
static int __secure_tz_entry(u32 cmd, u32 val)
{
	return 0;
}
#endif /* CONFIG_MSM_SECURE_IO */

static inline int kgsl_pwrctrl_tz_update(u32 idle)
{
	return __secure_tz_entry(TZ_UPDATE_ID, idle);
}

static inline void kgsl_pwrctrl_tz_reset(void)
{
	__secure_tz_entry(TZ_RESET_ID, 0);
}

static void kgsl_pwrctrl_pwrlevel_change(struct kgsl_device *device,
					unsigned int new_level)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	if (new_level < (pwr->num_pwrlevels - 1) &&
		new_level >= pwr->thermal_pwrlevel &&
		new_level != pwr->active_pwrlevel) {
		pwr->active_pwrlevel = new_level;
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_ON)
			clk_set_rate(pwr->grp_src_clk,
				pwr->pwrlevels[pwr->active_pwrlevel].
				gpu_freq);
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_ON)
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
					pwr->pwrlevels[pwr->active_pwrlevel].
					bus_freq);
		KGSL_PWR_WARN("pwr level changed to %d\n",
						pwr->active_pwrlevel);
	}
}

static int kgsl_pwrctrl_fraction_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long val;
	char temp[20];
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	mutex_lock(&device->mutex);

	if (val < 100)
		pwr->io_fraction = (unsigned int)val;
	else
		pwr->io_fraction = 100;

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_gpuclk_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char temp[20];
	int i, delta = 5000000;
	unsigned long val;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	mutex_lock(&device->mutex);

	/* Find the best match for the requested freq, if it exists */

	for (i = 0; i < pwr->num_pwrlevels; i++)
		if (abs(pwr->pwrlevels[i].gpu_freq - val) < delta) {
			pwr->thermal_pwrlevel = i;
			break;
	}

	kgsl_pwrctrl_pwrlevel_change(device, i);
	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_fraction_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n", pwr->io_fraction);
}

static struct device_attribute pwrio_fraction_attr = {
	.attr = { .name = "io_fraction", .mode = 0644, },
	.show = kgsl_pwrctrl_fraction_show,
	.store = kgsl_pwrctrl_fraction_store,
};

static int kgsl_pwrctrl_gpuclk_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n",
			pwr->pwrlevels[pwr->active_pwrlevel].gpu_freq);
}

static int kgsl_pwrctrl_pwrnap_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char temp[20];
	unsigned long val;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	mutex_lock(&device->mutex);

	if (val == 1)
		pwr->nap_allowed = true;
	else if (val == 0)
		pwr->nap_allowed = false;

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_pwrnap_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n", pwr->nap_allowed);
}


static int kgsl_pwrctrl_idle_timer_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char temp[20];
	unsigned long val;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	const long div = 1000/HZ;
	static unsigned int org_interval_timeout = 1;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	strict_strtoul(temp, 0, &val);

	if (org_interval_timeout == 1)
		org_interval_timeout = pwr->interval_timeout;

	mutex_lock(&device->mutex);

	/* Let the timeout be requested in ms, but convert to jiffies. */
	val /= div;
	if (val >= org_interval_timeout)
		pwr->interval_timeout = val;

	mutex_unlock(&device->mutex);

	return count;
}

static int kgsl_pwrctrl_idle_timer_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	return sprintf(buf, "%d\n", pwr->interval_timeout);
}

static struct device_attribute gpuclk_attr = {
	.attr = { .name = "gpuclk", .mode = 0644, },
	.show = kgsl_pwrctrl_gpuclk_show,
	.store = kgsl_pwrctrl_gpuclk_store,
};

static struct device_attribute pwrnap_attr = {
	.attr = { .name = "pwrnap", .mode = 0644, },
	.show = kgsl_pwrctrl_pwrnap_show,
	.store = kgsl_pwrctrl_pwrnap_store,
};

static struct device_attribute idle_timer_attr = {
	.attr = { .name = "idle_timer", .mode = 0644, },
	.show = kgsl_pwrctrl_idle_timer_show,
	.store = kgsl_pwrctrl_idle_timer_store,
};

int kgsl_pwrctrl_init_sysfs(struct kgsl_device *device)
{
	int ret = 0;
	ret = device_create_file(device->dev, &pwrio_fraction_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &pwrnap_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &gpuclk_attr);
	if (ret == 0)
		ret = device_create_file(device->dev, &idle_timer_attr);
	return ret;
}

void kgsl_pwrctrl_uninit_sysfs(struct kgsl_device *device)
{
	device_remove_file(device->dev, &pwrio_fraction_attr);
	device_remove_file(device->dev, &gpuclk_attr);
	device_remove_file(device->dev, &pwrnap_attr);
	device_remove_file(device->dev, &idle_timer_attr);
}

unsigned long kgsl_get_clkrate(struct clk *clk)
{
	if (clk != NULL)  {
		return clk_get_rate(clk);
	}  else {
		return 0;
	}
}

static void kgsl_pwrctrl_idle_calc(struct kgsl_device *device)
{
	int idle, val;
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	idle = device->ftbl.device_idle_calc(device);
	if (!idle)
		return;

	/* If the GPU has stayed in turbo mode for a while, *
	 * stop writing out values. */
	if (pwr->active_pwrlevel)
		pwr->no_switch_cnt = 0;
	else if (pwr->no_switch_cnt > SWITCH_OFF)
		return;
	pwr->no_switch_cnt++;
	val = kgsl_pwrctrl_tz_update(idle);
	if (val)
		kgsl_pwrctrl_pwrlevel_change(device,
				pwr->active_pwrlevel + val);
}

int kgsl_pwrctrl_clk(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_CLK_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_ON) {
			KGSL_PWR_DBG("clocks off, device %d\n", device->id);
			if (pwr->grp_pclk)
				clk_disable(pwr->grp_pclk);
			clk_disable(pwr->grp_clk);
			if (pwr->imem_clk != NULL)
				clk_disable(pwr->imem_clk);
			if (pwr->imem_pclk != NULL)
				clk_disable(pwr->imem_pclk);
			if ((pwr->pwrlevels[0].gpu_freq > 0) &&
				(device->requested_state != KGSL_STATE_NAP))
				clk_set_rate(pwr->grp_src_clk,
					pwr->pwrlevels[pwr->num_pwrlevels - 1].
						gpu_freq);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_CLK_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_CLK_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_OFF) {
			KGSL_PWR_DBG("clocks on, device %d\n", device->id);
			if ((pwr->pwrlevels[0].gpu_freq > 0) &&
				(device->state != KGSL_STATE_NAP))
				clk_set_rate(pwr->grp_src_clk,
					pwr->pwrlevels[pwr->active_pwrlevel].
						gpu_freq);
			if (pwr->grp_pclk)
				clk_enable(pwr->grp_pclk);
			clk_enable(pwr->grp_clk);
			if (pwr->imem_clk != NULL)
				clk_enable(pwr->imem_clk);
			if (pwr->imem_pclk != NULL)
				clk_enable(pwr->imem_pclk);

			pwr->power_flags &=
				~(KGSL_PWRFLAGS_CLK_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

int kgsl_pwrctrl_axi(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_AXI_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_ON) {
			KGSL_PWR_DBG("axi off, device %d\n", device->id);
			if (pwr->ebi1_clk)
				clk_disable(pwr->ebi1_clk);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
								    0);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_AXI_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_OFF) {
			KGSL_PWR_DBG("axi on, device %d\n", device->id);
			if (pwr->ebi1_clk)
				clk_enable(pwr->ebi1_clk);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
					pwr->pwrlevels[pwr->active_pwrlevel].
						bus_freq);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}


int kgsl_pwrctrl_pwrrail(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_POWER_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_ON) {
			KGSL_PWR_DBG("power off, device %d\n", device->id);
			if (internal_pwr_rail_ctl(pwr->pwr_rail, KGSL_FALSE)) {
				KGSL_DRV_ERR(
					"call internal_pwr_rail_ctl failed\n");
				return KGSL_FAILURE;
			}
			if (pwr->gpu_reg)
				regulator_disable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_ON);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_OFF) {
			KGSL_PWR_DBG("power on, device %d\n", device->id);
			if (internal_pwr_rail_ctl(pwr->pwr_rail, KGSL_TRUE)) {
				KGSL_PWR_ERR(
					"call internal_pwr_rail_ctl failed\n");
				return KGSL_FAILURE;
			}

			if (pwr->gpu_reg)
				regulator_enable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_OFF);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}


int kgsl_pwrctrl_irq(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	switch (pwrflag) {
	case KGSL_PWRFLAGS_IRQ_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_OFF) {
			KGSL_PWR_DBG("irq on, device %d\n", device->id);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_ON;
			enable_irq(pwr->interrupt_num);
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_IRQ_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_ON) {
			KGSL_PWR_DBG("irq off, device %d\n", device->id);
			disable_irq(pwr->interrupt_num);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_OFF;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

void kgsl_pwrctrl_close(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	KGSL_PWR_DBG("close device %d\n", device->id);

	if (pwr->interrupt_num > 0) {
		if (pwr->have_irq) {
			free_irq(pwr->interrupt_num, NULL);
			pwr->have_irq = 0;
		}
		pwr->interrupt_num = 0;
	}

	clk_put(pwr->ebi1_clk);

	if (pwr->pcl)
		msm_bus_scale_unregister_client(pwr->pcl);

	pwr->pcl = 0;

	if (pwr->gpu_reg) {
		regulator_put(pwr->gpu_reg);
		pwr->gpu_reg = NULL;
	}

	if (pwr->grp_pclk) {
		clk_put(pwr->grp_pclk);
		pwr->grp_pclk = NULL;
	}

	if (pwr->grp_clk) {
		clk_put(pwr->grp_clk);
		pwr->grp_clk = NULL;
	}

	if (pwr->imem_clk != NULL) {
		clk_put(pwr->imem_clk);
		pwr->imem_clk = NULL;
	}

	pwr->grp_src_clk = NULL;
	pwr->power_flags = 0;
}

void kgsl_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
							idle_check_ws);

	mutex_lock(&device->mutex);
	if ((device->pwrctrl.idle_pass) &&
		(device->requested_state != KGSL_STATE_SLEEP))
		kgsl_pwrctrl_idle_calc(device);

	if (device->state & (KGSL_STATE_ACTIVE | KGSL_STATE_NAP) &&
		device->pwrctrl.nap_allowed) {
		if (kgsl_pwrctrl_sleep(device) != 0)
			mod_timer(&device->idle_timer,
				jiffies +
				device->pwrctrl.interval_timeout);
	} else if (device->state & KGSL_STATE_HUNG) {
		device->requested_state = KGSL_STATE_NONE;
	}
	mutex_unlock(&device->mutex);
}

void kgsl_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;

	KGSL_PWR_DBG("idle timer expired device %d\n", device->id);
	if (device->requested_state != KGSL_STATE_SUSPEND) {
		device->requested_state = KGSL_STATE_SLEEP;
		/* Have work run in a non-interrupt context. */
		queue_work(device->work_queue, &device->idle_check_ws);
	}
}

void kgsl_pre_hwaccess(struct kgsl_device *device)
{
	if (device->state & (KGSL_STATE_SLEEP | KGSL_STATE_NAP))
		kgsl_pwrctrl_wake(device);
}

void kgsl_check_suspended(struct kgsl_device *device)
{
	if (device->requested_state == KGSL_STATE_SUSPEND ||
				device->state == KGSL_STATE_SUSPEND) {
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->hwaccess_gate);
		mutex_lock(&device->mutex);
	}
 }


/******************************************************************/
/* Caller must hold the device mutex. */
int kgsl_pwrctrl_sleep(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	KGSL_PWR_DBG("sleep device %d\n", device->id);

	/* Work through the legal state transitions */
	if (device->requested_state == KGSL_STATE_NAP) {
		if (device->ftbl.device_isidle(device))
			goto nap;
	} else if (device->requested_state == KGSL_STATE_SLEEP) {
		if (device->state == KGSL_STATE_NAP ||
			device->ftbl.device_isidle(device))
			goto sleep;
	}

	device->requested_state = KGSL_STATE_NONE;
	return KGSL_FAILURE;

sleep:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	if (pwr->pwrlevels[0].gpu_freq > 0)
		clk_set_rate(pwr->grp_src_clk,
				pwr->pwrlevels[pwr->num_pwrlevels - 1].
				gpu_freq);
	device->pwrctrl.no_switch_cnt = 0;
	device->pwrctrl.time = 0;
	kgsl_pwrctrl_tz_reset();
	goto clk_off;

nap:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
clk_off:
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);

	device->state = device->requested_state;
	device->requested_state = KGSL_STATE_NONE;
	wake_unlock(&device->idle_wakelock);
	KGSL_PWR_INFO("state -> NAP/SLEEP(%d), device %d\n",
				  device->state, device->id);

	return KGSL_SUCCESS;
}


/******************************************************************/
/* Caller must hold the device mutex. */
int kgsl_pwrctrl_wake(struct kgsl_device *device)
{
	int status = KGSL_SUCCESS;

	BUG_ON(!mutex_is_locked(&device->mutex));

	if (device->state == KGSL_STATE_SUSPEND)
		return status;

	/* Turn on the core clocks */
	status = kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_ON);
	if (device->state != KGSL_STATE_NAP) {
		if (device->pwrctrl.idle_pass)
			kgsl_pwrctrl_pwrlevel_change(device,
				device->pwrctrl.thermal_pwrlevel);
		kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_ON);
	}
	/* Enable state before turning on irq */
	device->state = KGSL_STATE_ACTIVE;
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);

	/* Re-enable HW access */
	KGSL_PWR_INFO("state -> ACTIVE, device %d\n", device->id);
	mod_timer(&device->idle_timer,
				jiffies + device->pwrctrl.interval_timeout);

	wake_lock(&device->idle_wakelock);
	KGSL_PWR_DBG("wake return value %d, device %d\n",
				  status, device->id);

	return status;
}

