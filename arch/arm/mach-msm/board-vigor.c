/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/bahama.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/pmic8058-pwrkey.h>
#include <linux/pmic8058-vibrator.h>
#include <linux/leds.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/bootmem.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pm8058.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/m_adc.h>
#include <linux/m_adcproc.h>
#include <linux/mfd/marimba.h>
#include <linux/proc_fs.h>

#include <mach/msm_serial_hs.h>
#include <mach/msm_serial_hs_lite.h>
#ifdef CONFIG_BT
#include <mach/htc_bdaddress.h>
#include <mach/htc_sleep_clk.h>
#endif

#include <linux/atmel_qt602240.h>
#include <linux/cm3629.h>
#include <linux/isl29028.h>
#include <linux/mpu.h>
#include <mach/panel_id.h>
#include <linux/msm-charger.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/input/tdisc_shinetsu.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/mpp.h>
#include <mach/irqs.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/htc_battery_core.h>
#include <mach/htc_battery_8x60.h>
#include <linux/tps65200.h>
#include <mach/msm_hsusb.h>
#ifdef CONFIG_MSM_DSPS
#include <mach/msm_dsps.h>
#endif
#include <mach/msm_xo.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_flashlight.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_misc.h>
#include <mach/htc_headset_8x60.h>
#include <linux/i2c/isl9519.h>
#include <mach/tpa2051d3.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/usb_gadget_fserial.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/akm8975.h>
#include <linux/bma250.h>
#include <linux/rmt_storage_client-8x60.h>
#include <mach/sdio_al.h>

#include <mach/socinfo.h>
#include "board-vigor.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "cpuidle.h"
#include "pm.h"
#include "rpm.h"
#include "spm.h"
#include "rpm_log.h"
#include "timer.h"
#include "saw-regulator.h"
#ifdef CONFIG_FB_MSM_HDMI_MHL
#include <mach/mhl.h>
#endif
#include "gpiomux.h"
#include "gpiomux-8x60.h"
#include "mpm.h"
#include "rpm-regulator.h"
#include "sysinfo-8x60.h"

#include <mach/htc_usb.h>
#include <mach/rpc_hsusb.h>
#include <mach/cable_detect.h>
#include <mach/irqs-8x60.h>
#include <mach/mdm.h>
#include <mach/htc_util.h>
#include <mach/board_htc.h>

#include "clock-8x60.h"
#include "rpm_stats.h"

extern int panel_type;
int cam_id_kvalue;

#define VIGOR_EVM_PCB_ID	0x99
unsigned int engineerid;
#define MSM_SHARED_RAM_PHYS 0x40000000
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
int set_two_phase_freq(int cpufreq);
#endif

#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

/*
 * The UI_INTx_N lines are pmic gpio lines which connect i2c
 * gpio expanders to the pm8058.
 */
#define UI_INT1_N 25
#define UI_INT2_N 34
#define UI_INT3_N 14
#define GPIO_LCM_RST_N	137

#define PM8058_MPP_BASE			(PM8058_GPIO_BASE + PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	(pm_gpio + PM8058_MPP_BASE)


// -----------------------------------------------------------------------------
//                         External routine declaration
// -----------------------------------------------------------------------------
#ifdef CONFIG_FB_MSM_HDMI_MHL
extern void sii9234_change_usb_owner(bool bMHL);
#endif //CONFIG_FB_MSM_HDMI_MHL
#ifdef CONFIG_FB_MSM_HDMI_MHL
static void mhl_sii9234_1v2_power(bool enable);
#endif

#define _GET_REGULATOR(var, name) do {				\
	var = regulator_get(NULL, name);			\
	if (IS_ERR(var)) {					\
		pr_err("'%s' regulator not found, rc=%ld\n",	\
			name, IS_ERR(var));			\
		var = NULL;					\
		return -ENODEV;					\
	}							\
} while (0)


#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static void (*sdc2_status_notify_cb)(int card_present, void *dev_id);
static void *sdc2_status_notify_cb_devid;
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static void (*sdc5_status_notify_cb)(int card_present, void *dev_id);
static void *sdc5_status_notify_cb_devid;
#endif

static struct msm_spm_platform_data msm_spm_data_v1[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x11,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x11,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x89,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x89,
		.collapse_mid_vlevel = 0x89,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x89,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x89,
		.collapse_mid_vlevel = 0x89,

		.vctl_timeout_us = 50,
	},
};

static struct msm_acpu_clock_platform_data msm8x60_acpu_clock_data = {
};

#ifdef CONFIG_PERFLOCK
static unsigned vigor_perf_acpu_table[] = {
	540000000,
	1026000000,
	1512000000,
};

static struct perflock_platform_data vigor_perflock_data = {
        .perf_acpu_table = vigor_perf_acpu_table,
        .table_size = ARRAY_SIZE(vigor_perf_acpu_table),
};
#endif

static struct regulator_consumer_supply saw_s0_supply =
	REGULATOR_SUPPLY("8901_s0", NULL);
static struct regulator_consumer_supply saw_s1_supply =
	REGULATOR_SUPPLY("8901_s1", NULL);

static struct regulator_init_data saw_s0_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1250000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s0_supply,
};

static struct regulator_init_data saw_s1_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1250000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s1_supply,
};

/*
static struct akm8975_platform_data compass_platform_data = {
	.layouts = VIGOR_LAYOUTS,
};

static struct i2c_board_info msm_i2c_gsbi12_akm8975_info[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(VIGOR_GPIO_COMPASS_INT),
	},
};

static struct bma250_platform_data gsensor_platform_data = {
	.intr = VIGOR_GPIO_GSENSOR_INT_N,
	.chip_layout = 0,
};

static struct i2c_board_info msm_i2c_gsbi12_bma250_info[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME, 0x30 >> 1),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(VIGOR_GPIO_GSENSOR_INT_N),
	},
};
*/
static struct platform_device msm_device_saw_s0 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S0,
	.dev           = {
		.platform_data = &saw_s0_init_data,
	},
};

static struct platform_device msm_device_saw_s1 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S1,
	.dev           = {
		.platform_data = &saw_s1_init_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR * 2] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 4000,
		.residency = 13000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 600,
		.residency = 7200,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static struct msm_cpuidle_state msm_cstates[] __initdata = {
	{0, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{0, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},

	{0, 2, "C2", "POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE},

	{1, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{1, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[Camera]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int vigor_phy_init_seq[] = { 0x06, 0x36, 0x0C, 0x31, 0x31, 0x32, 0x1, 0x0D, 0x1, 0x10, -1 };
static struct regulator *ldo6_3p3;
static struct regulator *ldo7_1p8;

#define USB_PHY_3P3_VOL_MIN	3500000 /* uV */
#define USB_PHY_3P3_VOL_MAX	3500000 /* uV */
#define USB_PHY_3P3_HPM_LOAD	50000	/* uA */
#define USB_PHY_3P3_LPM_LOAD	4000	/* uA */

#define USB_PHY_1P8_VOL_MIN	1800000 /* uV */
#define USB_PHY_1P8_VOL_MAX	1800000 /* uV */
#define USB_PHY_1P8_HPM_LOAD	50000	/* uA */
#define USB_PHY_1P8_LPM_LOAD	4000	/* uA */
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;
	if (init) {
		ldo6_3p3 = regulator_get(NULL, "8058_l6");
		if (IS_ERR(ldo6_3p3))
			return PTR_ERR(ldo6_3p3);

		ldo7_1p8 = regulator_get(NULL, "8058_l7");
		if (IS_ERR(ldo7_1p8)) {
			rc = PTR_ERR(ldo7_1p8);
			goto put_3p3;
		}

		rc = regulator_set_voltage(ldo6_3p3, USB_PHY_3P3_VOL_MIN,
				USB_PHY_3P3_VOL_MAX);
		if (rc) {
			pr_err("%s: Unable to set voltage level for"
				"ldo6_3p3 regulator\n", __func__);
			goto put_1p8;
		}
		rc = regulator_enable(ldo6_3p3);
		if (rc) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo6_3p3\n", __func__);
			goto put_1p8;
		}
		rc = regulator_set_voltage(ldo7_1p8, USB_PHY_1P8_VOL_MIN,
				USB_PHY_1P8_VOL_MAX);
		if (rc) {
			pr_err("%s: Unable to set voltage level for"
				"ldo7_1p8 regulator\n", __func__);
			goto disable_3p3;
		}
		rc = regulator_enable(ldo7_1p8);
		if (rc) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo7_1p8\n", __func__);
			goto disable_3p3;
		}

		return 0;
	}

	regulator_disable(ldo7_1p8);
disable_3p3:
	regulator_disable(ldo6_3p3);
put_1p8:
	regulator_put(ldo7_1p8);
put_3p3:
	regulator_put(ldo6_3p3);
	return rc;
}

static int msm_hsusb_ldo_enable(int on)
{
	int ret = 0;

	if (!ldo7_1p8 || IS_ERR(ldo7_1p8)) {
		pr_err("%s: ldo7_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!ldo6_3p3 || IS_ERR(ldo6_3p3)) {
		pr_err("%s: ldo6_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (on) {
		ret = regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"ldo7_1p8\n", __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(ldo6_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"ldo6_3p3\n", __func__);
			regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_LPM_LOAD);
			return ret;
		}
	} else {
		ret = regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_LPM_LOAD);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"ldo7_1p8\n", __func__);
		ret = regulator_set_optimum_mode(ldo6_3p3,
				USB_PHY_3P3_LPM_LOAD);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"ldo6_3p3\n", __func__);
	}

	pr_debug("reg (%s)\n", on ? "HPM" : "LPM");
	return ret < 0 ? ret : 0;
 }
#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	static int vbus_is_on;

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
		return;

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget	= 500, /* FIXME: 390, */
};
#endif

static uint32_t vigor_serial_debug_off_table[] = {
	GPIO_CFG(VIGOR_GPIO_UART_RX, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA),
	GPIO_CFG(VIGOR_GPIO_UART_TX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static void vigor_disable_serial_debug_gpios(void)
{
	config_gpio_table(vigor_serial_debug_off_table,
			  ARRAY_SIZE(vigor_serial_debug_off_table));
	mdelay(100);
	printk(KERN_INFO "%s:\n", __func__);
}

static uint32_t usb_uart_switch_table_evm[] = {
	GPIO_CFG(VIGOR_USBMHL_AUDz_SW, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
	GPIO_CFG(VIGOR_MHLAUD_USBz_SW_EVM, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
};

static uint32_t usb_uart_switch_table[] = {
	GPIO_CFG(VIGOR_USBMHL_AUDz_SW, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
};

static uint32_t notify_9k_ac_in_ouput_table[] = {
	GPIO_CFG(VIGOR_GPIO_NOTIFY_9K_AC_IN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static void config_notify_9k_ac_in_gpios(int charging_on)
{
	config_gpio_table(notify_9k_ac_in_ouput_table, ARRAY_SIZE(notify_9k_ac_in_ouput_table));

	printk(KERN_INFO "%s: %d\n", __func__, charging_on);
	if (charging_on)
		gpio_set_value(VIGOR_GPIO_NOTIFY_9K_AC_IN, 1);
	else
		gpio_set_value(VIGOR_GPIO_NOTIFY_9K_AC_IN, 0);
 }

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= vigor_phy_init_seq,
	.ldo_init		= msm_hsusb_ldo_init,
	.ldo_enable		= msm_hsusb_ldo_enable,
	.notify_9k_ac_in = config_notify_9k_ac_in_gpios,
};

#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	if (init)
		msm_charger_register_vbus_sn(callback);
	else
		msm_charger_unregister_vbus_sn(callback);
	return 0;
}
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.usb_in_sps = 1,
	.pclk_src_name		 = "dfab_usb_hs_clk",
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.se1_gating		 = SE1_GATING_DISABLE,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
	.pmic_notif_init         = msm_hsusb_pmic_notif_init,
#endif
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
#ifdef CONFIG_BATTERY_MSM8X60
	.chg_vbus_draw = msm_charger_vbus_draw,
#endif
	.drv_ampl		= HS_DRV_AMPLITUDE_DEFAULT,
};
#endif

static void pm8058_usb_config(void)
{
	int ret;

	ret = pm8058_mpp_config_digital_in(10, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DIN_TO_INT);
	if (ret != 0)
		printk(KERN_INFO "%s: MPP 10 fail\n", __func__);

	ret = pm8058_mpp_config_bi_dir(11, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_BI_PULLUP_10KOHM);
	if (ret != 0)
		printk(KERN_INFO "%s: MPP 11 fail\n", __func__);
}
static uint32_t usb_ID_PIN_input_table[] = {
	GPIO_CFG(VIGOR_GPIO_USB_ID1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	GPIO_CFG(VIGOR_GPIO_USB_ID1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};


static int wireless_charger_detect(void)
{
	int wireless_gpio = 1;

	wireless_gpio = gpio_get_value(
			PM8058_GPIO_PM_TO_SYS(VIGOR_WIRELESS_CHG_OK));
	pr_info("[CABLE] %s: wireless_gpio = %d\n", __func__, wireless_gpio);

	return wireless_gpio;
}

static void config_vigor_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(VIGOR_GPIO_USB_ID1, 1);
		pr_info("[CABLE] %s %d output high\n",  __func__, VIGOR_GPIO_USB_ID1);
	} else {
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		pr_info("[CABLE] %s %d input none pull\n",  __func__, VIGOR_GPIO_USB_ID1);
	}
}

static uint32_t mhl_reset_pin_ouput_table[] = {
	GPIO_CFG(VIGOR_GPIO_MHL_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t mhl_usb_switch_ouput_table[] = {
	GPIO_CFG(VIGOR_USBMHL_AUDz_SW, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

void config_vigor_mhl_gpios(void)
{
	config_gpio_table(mhl_usb_switch_ouput_table, ARRAY_SIZE(mhl_usb_switch_ouput_table));
	config_gpio_table(mhl_reset_pin_ouput_table, ARRAY_SIZE(mhl_reset_pin_ouput_table));
}

static void vigor_usb_dpdn_switch(int path)
{
	struct pm8058_gpio gpio_cfg_MHLAUD_USBz = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM_GPIO_VIN_S3,	/* S3 1.8 V */
		.inv_int_pol	= 0,
	};

	if (system_rev == VIGOR_EVM_PCB_ID)	/* VigorEVM */
		config_gpio_table(usb_uart_switch_table_evm,
				ARRAY_SIZE(usb_uart_switch_table_evm));
	else {
		config_gpio_table(usb_uart_switch_table, ARRAY_SIZE(usb_uart_switch_table));
		pm8058_gpio_config(VIGOR_MHLAUD_USBz_SW, &gpio_cfg_MHLAUD_USBz);
	}

	switch (path) {
	case PATH_USB:
	case PATH_MHL:
	{
		int polarity = 1; /* high = mhl */
		int mhl = (path == PATH_MHL);

		pr_info("[CABLE] %s: Set %s path\n", __func__, mhl ? "MHL" : "USB");
		gpio_set_value(VIGOR_USBMHL_AUDz_SW, 1);
		if (system_rev == VIGOR_EVM_PCB_ID)	/* VigorEVM */
			gpio_set_value(VIGOR_MHLAUD_USBz_SW_EVM,
						(mhl ^ !polarity) ? 1 : 0);
		else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_MHLAUD_USBz_SW),
						(mhl ^ !polarity) ? 1 : 0);

		break;
	}
	case PATH_USB_AUD:
		gpio_set_value(VIGOR_USBMHL_AUDz_SW, 0);
		if (system_rev == VIGOR_EVM_PCB_ID)	/* VigorEVM */
			gpio_set_value(VIGOR_MHLAUD_USBz_SW_EVM, 1);
		else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_MHLAUD_USBz_SW), 1);

		break;
	}

	#ifdef CONFIG_FB_MSM_HDMI_MHL
	sii9234_change_usb_owner((path==PATH_MHL)?1:0);
	#endif //CONFIG_FB_MSM_HDMI_MHL
}

static struct cable_detect_platform_data cable_detect_pdata = {
	.vbus_mpp_gpio		= PM8058_MPP_PM_TO_SYS(10),
	.vbus_mpp_config 	= pm8058_usb_config,
	.vbus_mpp_irq		= PM8058_CBLPWR_IRQ(PM8058_IRQ_BASE),
	.detect_type		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio	= VIGOR_GPIO_USB_ID1,
	.is_wireless_charger    = wireless_charger_detect,
	.mhl_reset_gpio		= VIGOR_GPIO_MHL_RST,
	.usb_dpdn_switch	= vigor_usb_dpdn_switch,
	.mpp_data = {
		.usbid_mpp	=  XOADC_MPP_4,
		.usbid_amux	= PM_MPP_AIN_AMUX_CH5,
	},
	.config_usb_id_gpios	= config_vigor_usb_id_gpios,
#ifdef CONFIG_FB_MSM_HDMI_MHL
	.mhl_1v2_power = mhl_sii9234_1v2_power,
#endif
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Android Phone",
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
static int vigor_usb_product_id_match(int product_id, int intrsharing)
{
	int *pid_array = vigor_usb_product_id_match_array;
	int *rndis_array = vigor_usb_product_id_rndis;

	if (!pid_array)
		return product_id;

	/* VZW pid re-match will not apply on MFG mode */
	if (board_mfg_mode() == 1)
		return product_id;

	while (pid_array[0] >= 0) {
		if (product_id == pid_array[0])
			return pid_array[1];
		pid_array += 2;
	}

	if (product_id == 0x0ffe) {
		/* rndis */
		if (intrsharing)
			return rndis_array[0];
		else
			return rndis_array[1];
	}

	return product_id;
}
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
	.product_id	= 0x0ccd,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.enable_fast_charge=NULL,
#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
	.match = vigor_usb_product_id_match,
#endif
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

static struct resource msm_vpe_resources[] = {
	{
		.start	= 0x05300000,
		.end	= 0x05300000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
	.name = "msm_vpe",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_vpe_resources),
	.resource = msm_vpe_resources,
};

#if CONFIG_ARCH_MSM_FLASHLIGHT
static int flashlight_control(int mode)
{
	if (system_rev == 0)
		return aat1271_flashlight_control(mode);
	return tps61310_flashlight_control(mode);
}

static void config_flashlight_gpios(void)
{
	static uint32_t flashlight_gpio_table[] = {
		GPIO_CFG(VIGOR_TORCH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(VIGOR_FLASH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	};

	static uint32_t flashlight_gpio_table_XB[] = {
		GPIO_CFG(VIGOR_TORCH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(VIGOR_FLASH_EN, 0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	};

	if (system_rev == 0)
		config_gpio_table(flashlight_gpio_table,
			ARRAY_SIZE(flashlight_gpio_table));
	else
		config_gpio_table(flashlight_gpio_table_XB,
			ARRAY_SIZE(flashlight_gpio_table_XB));
}

static struct flashlight_platform_data flashlight_data = {
	.gpio_init 		= config_flashlight_gpios,
	.torch 			= VIGOR_TORCH_EN,
	.flash 			= VIGOR_FLASH_EN,
	.flash_duration_ms 	= 600,
	.led_count 		= 1,
	.chip_model 		= AAT1271,
};

static struct platform_device flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev = {
		.platform_data	= &flashlight_data,
	},
};

static struct platform_device *flashlight_devices[] __initdata = {
	&flashlight_device,
};

static struct flashlight_platform_data vigor_flashlight_data_XB = {
	.gpio_init = config_flashlight_gpios,
	.tps61310_strb0 = VIGOR_TORCH_EN,
	.tps61310_strb1 = VIGOR_FLASH_EN,
	.led_count = 1,
	.flash_duration_ms = 600,
};

static struct i2c_board_info vigor_flashlight_XB[] = {
	{
		I2C_BOARD_INFO("TPS61310_FLASHLIGHT", 0x66 >> 1),
		.platform_data = &vigor_flashlight_data_XB,
	},
};
#endif

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(VIGOR_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* CAM_I2C_SDA */
	GPIO_CFG(VIGOR_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* CAM_I2C_SCL */
	GPIO_CFG(32, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* CAM_MCLK */
	GPIO_CFG(VIGOR_CAM2_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 	/* CAM2_RST# */
	GPIO_CFG(VIGOR_CLK_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CAM_SEL */
	GPIO_CFG(VIGOR_CAM1_VCM_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CAM1_VCM_PD */
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(VIGOR_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* CAM_I2C_SDA */
	GPIO_CFG(VIGOR_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), /* CAM_I2C_SCL */
	GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* CAM_MCLK */
	GPIO_CFG(VIGOR_CAM2_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM1_RST# */
	GPIO_CFG(VIGOR_CLK_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CAM_SEL */
	GPIO_CFG(VIGOR_CAM1_VCM_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CAM1_VCM_PD */
};

static int camera_sensor_power_enable(char *power, unsigned volt)
{
	struct regulator *sensor_power;
	int rc;
	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM]%s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}
	rc = regulator_set_voltage(sensor_power, volt, volt);
	if (rc) {
		pr_err("[CAM]%s: unable to set %s voltage to %d rc:%d\n",
			__func__, power, volt, rc);
	}
	rc = regulator_enable(sensor_power);
	if (rc) {
		pr_err("[CAM]%s: Enable regulator %s failed\n", __func__, power);
	}
	regulator_put(sensor_power);
	return rc;
}


static int camera_sensor_power_disable(char *power)
{
	struct regulator *sensor_power;
	int rc;
	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[CAM]%s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}
	rc = regulator_disable(sensor_power);
	if (rc) {
		pr_err("[CAM]%s: Enable regulator %s failed\n", __func__, power);
	}
	regulator_put(sensor_power);
	return rc;
}

static struct regulator *v_camera_io;/* for XB board*/

#ifdef CONFIG_S5K3H2YX
static int first_run = 1;
static int Vigor_s5k3h2yx_vreg_on(void)
{
	int rc;
	pr_info("[CAM]%s\n", __func__);
	/* L10 impact audio testing item in hboot mode */
	if (first_run == 1) {
		first_run = 0;
		/* VCM : L10*/
		rc = camera_sensor_power_enable("8058_l10", 2800000);
		/*pr_info("[CAM]sensor_power_enable(\"8058_l10\", 2.8V) == %d\n", rc);*/
		udelay(50);
	}
	gpio_set_value(VIGOR_CAM1_VCM_PD, 1); /* CAM1_VCM_PD */
	udelay(50);

	/* VDD 1V2 : L24*/
	if (system_rev >= 1) /* for XB board*/
		rc = camera_sensor_power_enable("8058_l23", 1200000);
	else
		rc = camera_sensor_power_enable("8058_l24", 1200000);
	/*pr_info("[CAM]sensor_power_enable(\"8058_l24\", 1.2V) == %d\n", rc);*/
	udelay(50);

	/* Analog : L15*/
	rc = camera_sensor_power_enable("8058_l15", 2800000);
	/*pr_info("[CAM]sensor_power_enable(\"8058_l15\", 2.8V) == %d\n", rc);*/

	udelay(50);

	/* IO : L8*/
	if (system_rev >= 1) { /* for XB board*/
		v_camera_io = regulator_get(NULL, "8901_lvs2");
		if (IS_ERR(v_camera_io)) {
			pr_err("%s#%d: get %s fail\n", __func__, __LINE__, "8901_lvs2");
			regulator_put(v_camera_io);
			return EIO;
		}

		if (regulator_enable(v_camera_io)) {
			pr_err("%s: enable %s fail\n", __func__, "8901_lvs2");
			return EIO;
		}
	} else {
		rc = camera_sensor_power_enable("8058_l8", 1800000);
		/*pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);*/
	}

	return rc;
}

static int Vigor_s5k3h2yx_vreg_off(void)
{
	int rc;
	pr_info("[CAM]%s\n", __func__);

	/* IO : L8*/
	if (system_rev >= 1) { /* for XB board*/
		v_camera_io = regulator_get(NULL, "8901_lvs2");
		if (IS_ERR(v_camera_io)) {
			pr_err("%s#%d: get %s fail\n", __func__, __LINE__, "8901_lvs2");
			regulator_put(v_camera_io);
			return EIO;
		}

		if (regulator_disable(v_camera_io)) {
			pr_err("%s: disable %s fail\n", __func__, "8901_lvs2");
			return EIO;
		}
	} else {
		rc = camera_sensor_power_disable("8058_l8");
		/*pr_info("[CAM]sensor_power_disable(\"8058_l8\") == %d\n", rc);*/
	}

	/* Analog : L15*/
	rc = camera_sensor_power_disable("8058_l15");
	/*pr_info("[CAM]sensor_power_disable(\"8058_l15\") == %d\n", rc);*/

	/* VDD 1V2 : L24*/
	if (system_rev >= 1) /* for XB board*/
		rc = camera_sensor_power_disable("8058_l23");
	else
		rc = camera_sensor_power_disable("8058_l24");
	/*pr_info("[CAM]sensor_power_disable(\"8058_l24\") == %d\n", rc);*/

#if 0 /* L10 impact audio testing item in hboot mode */
	/* VCM : L10*/
	rc = camera_sensor_power_disable("8058_l10");
	pr_info("[CAM]sensor_power_disable(\"8058_l10\") == %d\n", rc);
#else
	gpio_set_value(VIGOR_CAM1_VCM_PD, 0); /* CAM1_VCM_PD */
#endif

	return rc;
}
#endif

#ifdef CONFIG_OV8830
static int Vigor_ov8830_vreg_on(void)
{
	int rc = 0;
	struct regulator * votg_2_8v_switch;
	pr_info("[CAM] %s\n", __func__);

	/* VCM */
#if 0
	rc = camera_sensor_power_enable("8058_l10", 2800000);
	pr_info("[CAM] sensor_power_enable(\"8058_l10\", 2.85V) == %d\n", rc);
#else
if (first_run == 1) {
	first_run = 0;
	/* VCM : L10*/
	rc = camera_sensor_power_enable("8058_l10", 2800000);
	pr_info("[CAM]sensor_power_enable,1,(\"8058_l10\", 2.8V) == %d\n", rc);
	udelay(50);
}
gpio_set_value(VIGOR_CAM1_VCM_PD, 1); /* CAM1_VCM_PD */
udelay(50);

#endif


	if (rc < 0)
		goto init_fail;

	/* analog */
	rc = camera_sensor_power_enable("8058_l15", 2800000);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	votg_2_8v_switch = regulator_get(NULL, "8901_usb_otg");
	if (IS_ERR(votg_2_8v_switch)) {
		pr_err("[CAM] %s: unable to get votg_2_8v_switch\n", __func__);
		goto init_fail;
	}

	if (regulator_enable(votg_2_8v_switch)) {
		pr_err("[CAM] %s: Unable to enable the regulator: votg_2_8v_switch\n", __func__);
		goto init_fail;
	}

	msleep(10);

	/* IO : L8*/
	if (system_rev >= 1) { /* for XB board*/
		v_camera_io = regulator_get(NULL, "8901_lvs2");
		if (IS_ERR(v_camera_io)) {
			pr_err("%s#%d: get %s fail\n", __func__, __LINE__, "8901_lvs2");
			regulator_put(v_camera_io);
			return EIO;
		}

		if (regulator_enable(v_camera_io)) {
			pr_err("%s: enable %s fail\n", __func__, "8901_lvs2");
			return EIO;
		}
	} else {
		rc = camera_sensor_power_enable("8058_l8", 1800000);
		/*pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);*/
	}



	if (rc < 0)
		goto init_fail;

	msleep(10);

	pr_info("Vigor_ov8830_vreg_on,end");

init_fail:
	return rc;
}

static int Vigor_ov8830_vreg_off(void)
{
	int rc = 0;
	struct regulator * votg_2_8v_switch;
	pr_info("[CAM] %s\n", __func__);

	/* IO */
	if (system_rev >= 1) { /* for XB board*/
		v_camera_io = regulator_get(NULL, "8901_lvs2");
		if (IS_ERR(v_camera_io)) {
			pr_err("%s#%d: get %s fail\n", __func__, __LINE__, "8901_lvs2");
			regulator_put(v_camera_io);
			return EIO;
		}

		if (regulator_disable(v_camera_io)) {
			pr_err("%s: disable %s fail\n", __func__, "8901_lvs2");
			return EIO;
		}
	} else {
		rc = camera_sensor_power_disable("8058_l8");
		/*pr_info("[CAM]sensor_power_disable(\"8058_l8\") == %d\n", rc);*/
	}


	if (rc < 0)
		goto init_fail;

	/* analog */
	rc = camera_sensor_power_disable("8058_l15");

	if (rc < 0)
		goto init_fail;

	votg_2_8v_switch = regulator_get(NULL, "8901_usb_otg");
	if (IS_ERR(votg_2_8v_switch)) {
		pr_err("[CAM] %s: unable to get votg_2_8v_switch\n", __func__);
		goto init_fail;
	}
	if (regulator_disable(votg_2_8v_switch)) {
		pr_err("[CAM] %s: Unable to disable the regulator: votg_2_8v_switch\n", __func__);
		goto init_fail;
	}


#if 0 /* L10 impact audio testing item in hboot mode */
			/* VCM : L10*/
			rc = camera_sensor_power_disable("8058_l10");
			pr_info("[CAM]sensor_power_disable(\"8058_l10\") == %d\n", rc);
#else
			gpio_set_value(VIGOR_CAM1_VCM_PD, 0); /* CAM1_VCM_PD */
#endif

	udelay(50);

init_fail:
	return rc;
}
#endif

/* mt9d015 power */
#ifdef CONFIG_MT9D015
static int Vigor_mt9d015_vreg_on(void)
{
	int rc;
	pr_info("[CAM]%s\n", __func__);
	/* for 2nd CAM probed fail sometimes */
	gpio_set_value(VIGOR_CAM1_VCM_PD, 0); /* CAM1_VCM_PD */
	udelay(50);
	/* L10 impact audio testing item in hboot mode */
	if (first_run == 1) {
		first_run = 0;
		/* VCM : L10*/
		rc = camera_sensor_power_enable("8058_l10", 2800000);
	/*pr_info("[CAM]sensor_power_enable(\"8058_l10\", 2.8V) == %d\n", rc);*/
		udelay(50);
	}

	/* VDD 1V8 : L9*/
	if (system_rev >= 1) { /* for XB board*/
		v_camera_io = regulator_get(NULL, "8901_lvs3");
		if (IS_ERR(v_camera_io)) {
			pr_err("%s#%d: get %s fail\n", __func__, __LINE__, "8901_lvs3");
			regulator_put(v_camera_io);
			return EIO;
		}

		if (regulator_enable(v_camera_io)) {
			pr_err("%s: enable %s fail\n", __func__, "8901_lvs3");
			return EIO;
		}
	} else {
		rc = camera_sensor_power_enable("8058_l9", 1800000);
		/*pr_info("[CAM]sensor_power_enable(\"8058_l9\", 1.8V) == %d\n", rc);*/
	}
	udelay(50);

	/* Analog : PM8901 L6*/
	rc = camera_sensor_power_enable("8901_l6", 2800000);
	/*pr_info("[CAM]sensor_power_enable(\"8901_l6\", 2.8V) == %d\n", rc);*/
	udelay(50);

	/* IO : L8*/
	rc = camera_sensor_power_enable("8058_l8", 1800000);
	/*pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);*/

	return rc;
}

static int Vigor_mt9d015_vreg_off(void)
{
	int rc;
	pr_info("[CAM]%s\n", __func__);

	/* IO : L8*/
	rc = camera_sensor_power_disable("8058_l8");
	/*pr_info("[CAM]sensor_power_disable(\"8058_l8\") == %d\n", rc);*/

	/* Analog : PM8901 L6*/
	rc = camera_sensor_power_disable("8901_l6");
	/*pr_info("[CAM]sensor_power_disable(\"8901_l6\") == %d\n", rc);*/

	/* VDD 1V8 : L9*/
	if (system_rev >= 1) { /* for XB board*/
		v_camera_io = regulator_get(NULL, "8901_lvs3");
		if (IS_ERR(v_camera_io)) {
			pr_err("%s#%d: get %s fail\n", __func__, __LINE__, "8901_lvs3");
			regulator_put(v_camera_io);
			return EIO;
		}

		if (regulator_disable(v_camera_io)) {
			pr_err("%s: disable %s fail\n", __func__, "8901_lvs3");
			return EIO;
		}
	} else {
		rc = camera_sensor_power_disable("8058_l9");
		/*pr_info("[CAM]sensor_power_disable(\"8058_l9\") == %d\n", rc);*/
	}

	return rc;
}
#endif

static void Vigor_maincam_clk_switch(void)
{
	pr_info("[CAM]Doing clk switch (Main Cam)\n");
	gpio_set_value(VIGOR_CLK_SWITCH, 0);

	return;
}

static void Vigor_seccam_clk_switch(void)
{
	pr_info("[CAM]Doing clk switch (2nd Cam)\n");
	gpio_set_value(VIGOR_CLK_SWITCH, 1);

	return;
}

#ifdef CONFIG_HTC_BATT8x60
static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_NONE,
	.gpio_mbat_in = MSM_GPIO_TO_INT(VIGOR_GPIO_MBAT_IN),
	.gpio_mbat_in_trigger_level = MBAT_IN_HIGH_TRIGGER,
	.charger = SWITCH_CHARGER_TPS65200,
	.mpp_data = {
		{XOADC_MPP_3, PM_MPP_AIN_AMUX_CH6},
		{XOADC_MPP_5, PM_MPP_AIN_AMUX_CH6},
		{XOADC_MPP_7, PM_MPP_AIN_AMUX_CH6},
		{XOADC_MPP_8, PM_MPP_AIN_AMUX_CH6},
	},
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev    = {
		.platform_data = &htc_battery_pdev_data,
	},
};
#endif

#define GPIO_CAM_EN (GPIO_EXPANDER_GPIO_BASE + 13)
static void vigor_config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void vigor_config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}


static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = vigor_config_camera_on_gpios,
	.camera_gpio_off = vigor_config_camera_off_gpios,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 266667000,
};

static struct msm_camera_device_platform_data msm_camera_device_data_web_cam = {
	.camera_gpio_on  = vigor_config_camera_on_gpios,
	.camera_gpio_off = vigor_config_camera_off_gpios,
	.ioext.csiphy = 0x04900000, /* For front CAM */
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

static struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type				= MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	.camera_flash				= flashlight_control,
};

/* HTC start jason 20110811 */
static struct camera_flash_info msm_camera_sensor_flash_info = {
	.low_limit_led_state = FL_MODE_FLASH_LEVEL1,
	.max_led_current_ma = 800,
	.led_est_table = {
	{
		.enable = 0,
		.led_state = FL_MODE_OFF,
		.current_ma = 0,
		.lumen_value = 0,
		.min_focus_step = 0,
		.max_focus_step = 0
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH,
		.current_ma = 150,
		.lumen_value = 150,
		.min_focus_step = 0,
		.max_focus_step = 0
	},
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH,
		.current_ma = 800,
		.lumen_value = 800,
		.min_focus_step = 33,
		.max_focus_step = 42
	},
	{
		.enable = 0,
		.led_state = FL_MODE_PRE_FLASH,
		.current_ma = 100,
		.lumen_value = 100,
		.min_focus_step = 26,
		.max_focus_step = 30
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH_LED_A,
		.current_ma = 25,
		.lumen_value = 25,
		.min_focus_step = 0,
		.max_focus_step = 0
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH_LED_B,
		.current_ma = 25,
		.lumen_value = 25,
		.min_focus_step = 0,
		.max_focus_step = 0
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH_LEVEL_1,
		.current_ma = 50,
		.lumen_value = 50,
		.min_focus_step = 0,
		.max_focus_step = 0
	},
	{
		.enable = 0,
		.led_state = FL_MODE_TORCH_LEVEL_2,
		.current_ma = 100,
		.lumen_value = 100,
		.min_focus_step = 0,
		.max_focus_step = 0
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL1,
		.current_ma = 400,
		.lumen_value = 400,
		.min_focus_step = 33,
		.max_focus_step = 36
	},
	{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL2,
		.current_ma = 200,
		.lumen_value = 264,
		.min_focus_step = 0,
		.max_focus_step = 32
	},
	},
};
/* HTC end */

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
	.flash_info             = &msm_camera_sensor_flash_info,    /* HTC jason 20110811 */
};

#ifdef CONFIG_S5K3H2YX
static int camera_pm8058_power(int power)
{
	int rc = 0;

	/* PMIC -> Sensor : HW is PM_GPIO_DIR_OUT, SW is PM_GPIO_DIR_IN if invert is 0*/
	/*.inv_int_pol = 1 => invert director as SW definition if use HW defining Director*/
		struct pm8058_gpio camera_3h2_rst_pin_on = {
			.direction		= PM_GPIO_DIR_OUT,
			.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
			.output_value	= 1,
			.pull			= PM_GPIO_PULL_UP_31P5,
			.vin_sel 		= PM_GPIO_VIN_S3,
			.out_strength	= PM_GPIO_STRENGTH_HIGH,
			.function 		= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 1,
		};

			struct pm8058_gpio camera_3h2_rst_pin_off = {
			.direction		= PM_GPIO_DIR_OUT,
			.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
			.output_value	= 0,
			.pull			= PM_GPIO_PULL_DN,
			.vin_sel 		= PM_GPIO_VIN_S3,
			.out_strength	= PM_GPIO_STRENGTH_HIGH,
			.function 		= PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 1,
		};

	/*pr_info("Vigor: Doing CAM1_RST pin (Vigor)(s5k3h2yx)\n");*/

	if (power == 1)
	  pm8058_gpio_config(VIGOR_CAM1_RST, &camera_3h2_rst_pin_on);
	else
	  pm8058_gpio_config(VIGOR_CAM1_RST, &camera_3h2_rst_pin_off);

	if (rc < 0)
		pr_err("PM8058 GPIO (%d) request fail\n", VIGOR_CAM1_RST);
	else
		pr_info("PM8058 GPIO (%d) set OK\n", VIGOR_CAM1_RST);

	    return rc;
}

static struct msm_camera_sensor_flash_data flash_s5k3h2yx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3h2yx_data = {
	.sensor_name	= "s5k3h2yx",
	.sensor_reset = 0,
	.sensor_pwd = -1, /*RST pin does not connect to CPU*/
	.vcm_pwd = VIGOR_CAM1_VCM_PD,
	.vcm_enable = 1,
	.camera_pm8058_power = camera_pm8058_power,
	.camera_power_on = Vigor_s5k3h2yx_vreg_on,
	.camera_power_off = Vigor_s5k3h2yx_vreg_off,
	.camera_clk_switch = Vigor_maincam_clk_switch,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_s5k3h2yx,
	.flash_cfg = &msm_camera_sensor_flash_cfg,
	.mirror_mode = 0,
	.csi_if = 1,
	.dev_node	= 0
};

static struct platform_device msm_camera_sensor_s5k3h2yx = {
	.name	= "msm_camera_s5k3h2yx",
	.dev	= {
		.platform_data = &msm_camera_sensor_s5k3h2yx_data,
	},
};
#endif

#ifdef CONFIG_OV8830
static struct msm_camera_sensor_flash_data flash_ov8830 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};


static struct msm_camera_sensor_info msm_camera_sensor_ov8830_data = {
	.sensor_name	= "ov8830",
	.sensor_reset	=	0,
	.sensor_pwd	= -1,
	.vcm_pwd = VIGOR_CAM1_VCM_PD,
	.vcm_enable = 1,
	.camera_pm8058_power = camera_pm8058_power,
	.camera_power_on = Vigor_ov8830_vreg_on,
	.camera_power_off = Vigor_ov8830_vreg_off,
	.camera_clk_switch	= Vigor_maincam_clk_switch,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_data = &flash_ov8830,
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
	.mirror_mode = 0,
	.csi_if			= 1,
	.dev_node		= 0
};

static struct platform_device msm_camera_sensor_ov8830 = {
    .name	= "msm_camera_ov8830",
    .dev	= {
			.platform_data = &msm_camera_sensor_ov8830_data,
    },
};
#endif

#ifdef CONFIG_MT9D015
static struct msm_camera_sensor_flash_data flash_mt9d015 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
	/*.flash_src	= &msm_flash_src*/
	};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d015_data = {
	.sensor_name	= "mt9d015",
	.sensor_reset = VIGOR_CAM2_RST,
	.camera_power_on = Vigor_mt9d015_vreg_on,
	.camera_power_off = Vigor_mt9d015_vreg_off,
	.camera_clk_switch = Vigor_seccam_clk_switch,
	.vcm_enable	= 0,
	.pdata = &msm_camera_device_data_web_cam, /* For front CAM */
	.resource	= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data	= &flash_mt9d015,
	.mirror_mode = 1,
	.csi_if		= 1,
	.dev_node	= 1
};

static void __init msm8x60_init_camera(void)
{
	msm_camera_sensor_webcam.name = "msm_camera_webcam";
	msm_camera_sensor_webcam.dev.platform_data = &msm_camera_sensor_mt9d015_data;
}
#endif

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_S5K3H2YX
	{
			I2C_BOARD_INFO("s5k3h2yx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9D015
	{
			I2C_BOARD_INFO("mt9d015", 0x6C >> 1),
	},
#endif
};

static struct i2c_board_info msm_camera_boardinfo_OV8830[] __initdata = {
#ifdef CONFIG_OV8830
	{
		I2C_BOARD_INFO("ov8830", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9D015
	{
			I2C_BOARD_INFO("mt9d015", 0x6C >> 1),
	},
#endif
};

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0x04600000,
		.end    = 0x04600000 + SZ_1M - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_FB_MSM_HDMI_MHL
#define GPIO_MHL_RST_N   43
#define GPIO_MHL_INTR_N  58
static int pm8901_mpp_init(void);
static struct regulator *reg_8901_l0;
static struct regulator *reg_8058_l19;
static struct regulator *reg_8901_l3;

static uint32_t msm_hdmi_off_gpio[] = {
	GPIO_CFG(170,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(171,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(172,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t msm_hdmi_on_gpio[] = {
	GPIO_CFG(170,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(171,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(172,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

void hdmi_hpd_feature(int enable);

static void mhl_sii9234_1v2_power(bool enable)
{
	static bool prev_on, init = false;
	struct pm8058_gpio gpio_cfg_MHL_5V = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM_GPIO_VIN_S3,	/* S3 1.8 V */
		.inv_int_pol	= 0,
	};

	if (enable == prev_on)
		return;

	if (!init) {
		pm8058_gpio_config(VIGOR_MHL_5V, &gpio_cfg_MHL_5V);
		init = true;
	}

	if (enable) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_MHL_5V), 1);
		config_gpio_table(msm_hdmi_on_gpio, ARRAY_SIZE(msm_hdmi_on_gpio));
		hdmi_hpd_feature(1);
		pr_info("%s(on): success\n", __func__);
	} else {
		config_gpio_table(msm_hdmi_off_gpio, ARRAY_SIZE(msm_hdmi_off_gpio));
		hdmi_hpd_feature(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_MHL_5V), 0);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = enable;
}

static int mhl_sii9234_all_power(bool enable)
{
	static bool prev_on;
	int rc;

	if (enable == prev_on)
		return 0;

	if (!reg_8058_l19)
		_GET_REGULATOR(reg_8058_l19, "8058_l19");
	if (!reg_8901_l3)
		_GET_REGULATOR(reg_8901_l3, "8901_l3");
	if (!reg_8901_l0)
		_GET_REGULATOR(reg_8901_l0, "8901_l0");

	if (enable) {
		rc = regulator_set_voltage(reg_8058_l19, 1800000, 1800000);
		if (rc) {
			pr_err("%s: regulator_set_voltage reg_8058_l19 failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		rc = regulator_set_voltage(reg_8901_l3, 3300000, 3300000);
		if (rc) {
			pr_err("%s: regulator_set_voltage reg_8901_l3 failed rc=%d\n",
				__func__, rc);
			return rc;
		}

		rc = regulator_set_voltage(reg_8901_l0, 1200000, 1200000);
		if (rc) {
			pr_err("%s: regulator_set_voltage reg_8901_l0 failed rc=%d\n",
				__func__, rc);
			return rc;
		}	rc = regulator_enable(reg_8058_l19);

		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8058_l19", rc);
			return rc;
		}
		rc = regulator_enable(reg_8901_l3);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8901_l3", rc);
			return rc;
		}

		rc = regulator_enable(reg_8901_l0);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8901_l0", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8058_l19);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8058_l19", rc);
		rc = regulator_disable(reg_8901_l3);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8901_l3", rc);
		rc = regulator_disable(reg_8901_l0);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8901_l0", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = enable;

	return 0;
}

#ifdef CONFIG_FB_MSM_HDMI_MHL_SII9234
static uint32_t mhl_gpio_table[] = {
	GPIO_CFG(GPIO_MHL_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_MHL_INTR_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};
static int mhl_sii9234_power(int on)
{
	int rc = 0;

	switch (on) {
		case 0:
			mhl_sii9234_1v2_power(false);
			break;
		case 1:
			mhl_sii9234_all_power(true);
			pm8901_mpp_init();
			config_gpio_table(mhl_gpio_table, ARRAY_SIZE(mhl_gpio_table));
			break;
		default:
			break;
	}
	return rc;
}

static T_MHL_PLATFORM_DATA mhl_sii9234_device_data = {
	.gpio_intr = GPIO_MHL_INTR_N,
	.gpio_reset = GPIO_MHL_RST_N,
	.power = mhl_sii9234_power,
#ifdef CONFIG_FB_MSM_HDMI_MHL
	.mhl_usb_switch 	= vigor_usb_dpdn_switch,
	.mhl_1v2_power = mhl_sii9234_1v2_power,
#endif

};

static struct i2c_board_info msm_i2c_gsbi7_mhl_sii9234_info[] =
{
	{
		I2C_BOARD_INFO(MHL_SII9234_I2C_NAME, 0x72 >> 1),
		.platform_data = &mhl_sii9234_device_data,
		.irq = MSM_GPIO_TO_INT(GPIO_MHL_INTR_N)
	},
};
#endif
#endif

static struct tps65200_platform_data tps65200_data = {
	.gpio_chg_stat = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, VIGOR_CHG_STAT),
	.gpio_chg_int  = MSM_GPIO_TO_INT(VIGOR_GPIO_CHG_INT),
};

#ifdef CONFIG_SUPPORT_DQ_BATTERY
static int __init check_dq_setup(char *str)
{
	if (!strcmp(str, "PASS"))
		tps65200_data.dq_result = 1;
	else
		tps65200_data.dq_result = 0;

	return 1;
}
__setup("androidboot.dq=", check_dq_setup);
#endif

static struct i2c_board_info msm_tps_65200_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
};

#ifdef CONFIG_I2C_QUP

/* CAMERA setting */
static uint32_t gsbi4_gpio_table[] = {
	GPIO_CFG(VIGOR_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t gsbi5_gpio_table[] = {
	GPIO_CFG(VIGOR_TP_I2C_SDA, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_TP_I2C_SCL, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t gsbi10_gpio_table[] = {
	GPIO_CFG(VIGOR_GPIO_SENSOR_I2C_SDA_EVM, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_GPIO_SENSOR_I2C_SCL_EVM, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t gsbi12_gpio_table[] = {
	GPIO_CFG(VIGOR_GPIO_SENSOR_I2C_SDA, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_GPIO_SENSOR_I2C_SCL, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};


static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	printk(KERN_INFO "%s(): adap_id = %d, config_type = %d \n", __func__,adap_id,config_type);

/* CAMERA setting */
	if ((adap_id == MSM_GSBI4_QUP_I2C_BUS_ID) && (config_type == 1)) {
		gpio_tlmm_config(gsbi4_gpio_table[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(gsbi4_gpio_table[1], GPIO_CFG_ENABLE);
	}

	if ((adap_id == MSM_GSBI4_QUP_I2C_BUS_ID) && (config_type == 0)) {
		gpio_tlmm_config(gsbi4_gpio_table[0], GPIO_CFG_DISABLE);
		gpio_tlmm_config(gsbi4_gpio_table[1], GPIO_CFG_DISABLE);
	}

	if ((adap_id == MSM_GSBI5_QUP_I2C_BUS_ID) && (config_type == 1)) {
		gpio_tlmm_config(gsbi5_gpio_table[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(gsbi5_gpio_table[1], GPIO_CFG_ENABLE);
	}

	if ((adap_id == MSM_GSBI5_QUP_I2C_BUS_ID) && (config_type == 0)) {
		gpio_tlmm_config(gsbi5_gpio_table[0], GPIO_CFG_DISABLE);
		gpio_tlmm_config(gsbi5_gpio_table[1], GPIO_CFG_DISABLE);
	}

	if (system_rev == VIGOR_EVM_PCB_ID) {		// VigorEVM
		if ((adap_id == MSM_GSBI10_QUP_I2C_BUS_ID) && (config_type == 1)) {
			gpio_tlmm_config(gsbi10_gpio_table[0], GPIO_CFG_ENABLE);
			gpio_tlmm_config(gsbi10_gpio_table[1], GPIO_CFG_ENABLE);
		}

		if ((adap_id == MSM_GSBI10_QUP_I2C_BUS_ID) && (config_type == 0)) {
			gpio_tlmm_config(gsbi10_gpio_table[0], GPIO_CFG_DISABLE);
			gpio_tlmm_config(gsbi10_gpio_table[1], GPIO_CFG_DISABLE);
		}
	} else {
		if ((adap_id == MSM_GSBI12_QUP_I2C_BUS_ID) && (config_type == 1)) {
			gpio_tlmm_config(gsbi12_gpio_table[0], GPIO_CFG_ENABLE);
			gpio_tlmm_config(gsbi12_gpio_table[1], GPIO_CFG_ENABLE);
		}

		if ((adap_id == MSM_GSBI12_QUP_I2C_BUS_ID) && (config_type == 0)) {
			gpio_tlmm_config(gsbi12_gpio_table[0], GPIO_CFG_DISABLE);
			gpio_tlmm_config(gsbi12_gpio_table[1], GPIO_CFG_DISABLE);
		}
	}
}

static struct msm_i2c_platform_data msm_gsbi4_qup_i2c_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi5_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi7_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi10_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi12_qup_i2c_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};


#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
static struct msm_spi_platform_data msm_gsbi3_qup_spi_pdata = {
	.max_clock_speed = 10800000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
#endif

#if defined(CONFIG_I2C_SSBI) || defined(CONFIG_MSM8X60_SSBI)
/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi1_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi2_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* CODEC/TSSC SSBI */
static struct msm_ssbi_platform_data msm_ssbi3_pdata = {
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

#ifdef CONFIG_BATTERY_MSM
/* Use basic value for fake MSM battery */
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.avail_chg_sources = AC_CHG,
};

static struct platform_device msm_batt_device = {
	.name              = "msm-battery",
	.id                = -1,
	.dev.platform_data = &msm_psy_batt_data,
};
#endif

#ifdef CONFIG_FB_MSM_LCDC_DSUB
/* VGA = 1440 x 900 x 4(bpp) x 2(pages)
   prim = 1024 x 600 x 4(bpp) x 2(pages)
   This is the difference. */
#define MSM_FB_DSUB_PMEM_ADDER (0x9E3400-0x4B0000)
#else
#define MSM_FB_DSUB_PMEM_ADDER (0)
#endif

/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS
static struct dsps_gpio_info dsps_gpios[] = {
};

static void __init msm8x60_init_dsps(void)
{
	struct msm_dsps_platform_data *pdata =
		msm_dsps_device.dev.platform_data;

	pdata->gpios = dsps_gpios;
	pdata->gpios_num = ARRAY_SIZE(dsps_gpios);
}
#endif /* CONFIG_MSM_DSPS */

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
/* prim = 1280 x 720 x 4(bpp) x 3(pages) */
#define MSM_FB_PRIM_BUF_SIZE 0xA8C000
#else
/* prim = 1280 x 720 x 4(bpp) x 2(pages) */
#define MSM_FB_PRIM_BUF_SIZE 0x708000
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
/* prim = 1024 x 600 x 4(bpp) x 2(pages)
 * hdmi = 1920 x 1080 x 2(bpp) x 1(page)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + 0x3F4800 + 0x546000, 4096)
#elif defined(CONFIG_FB_MSM_TVOUT)
/* prim = 1024 x 600 x 4(bpp) x 2(pages)
 * tvout = 720 x 576 x 2(bpp) x 2(pages)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + 0x195000 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + 0x313800 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_PMEM_SF_SIZE 0x1000000 /* 16 Mbytes */
#define MSM_PMEM_RMT_STORAGE_SIZE 0x100000 /* 1 Mbytes */

#define MSM_PMEM_ADSP_SIZE         0x3300000
#define MSM_PMEM_AUDIO_SIZE        0x239000

#define MSM_PMEM_ADSP_BASE	(0x80000000 - MSM_PMEM_ADSP_SIZE)
#define MSM_PMEM_SF_BASE	(0x40400000)
#define MSM_PMEM_AUDIO_BASE	(MSM_PMEM_SF_BASE + MSM_PMEM_SF_SIZE)
#define MSM_FB_BASE		(MSM_PMEM_AUDIO_BASE + MSM_PMEM_AUDIO_SIZE)

#define MSM_SMI_BASE          0x38000000
/* Kernel SMI PMEM Region for video core, used for Firmware */
/* and encoder,decoder scratch buffers */
/* Kernel SMI PMEM Region Should always precede the user space */
/* SMI PMEM Region, as the video core will use offset address */
/* from the Firmware base */
#define PMEM_KERNEL_SMI_BASE  (MSM_SMI_BASE)
#define PMEM_KERNEL_SMI_SIZE  0x300000
/* User space SMI PMEM Region for video core*/
/* used for encoder, decoder input & output buffers  */
#define MSM_PMEM_SMIPOOL_BASE (PMEM_KERNEL_SMI_BASE + PMEM_KERNEL_SMI_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE 0x3D00000

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);


#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device mipi_dsi_cmd_sharp_qhd_panel_device = {
	.name = "mipi_novatek",
	.id = 0,
};

static int msm_fb_detect_panel(const char *name)
{
	if (panel_type == PANEL_ID_RUBY_SHARP) {
		if (!strcmp(name, "mipi_video_himax_720p"))
			return 0;
	}
	else if (panel_type == PANEL_ID_VIG_SHARP_HX ||
		panel_type == PANEL_ID_VIG_SHARP_HX_C2 ||
		panel_type == PANEL_ID_VIG_SHARP_HX_C25 ||
		panel_type == PANEL_ID_VIG_SHARP_HX_C3 ||
		panel_type == PANEL_ID_VIG_CHIMEI_HX_C3 ||
		panel_type == PANEL_ID_VIG_CHIMEI_HX_C25 ||
		panel_type == PANEL_ID_VIG_CHIMEI_HX ) {
		if (!strcmp(name, "mipi_video_himax_720p"))
			return 0;
	}
	else if (panel_type == PANEL_ID_PYD_AUO_NT) {
		if (!strcmp(name, "mipi_cmd_novatek_qhd"))
			return 0;
	}
	else if (panel_type == PANEL_ID_PYD_SHARP) {
		if (!strcmp(name, "mipi_cmd_novatek_qhd"))
			return 0;
	}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	if (!strcmp(name, "hdmi_msm"))
		return 0;
#endif

	pr_warning("%s: not supported '%s' (0x%x)", __func__, name, panel_type);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.width = 53,
	.height = 95,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

/* parameters for backlight value mapping */
#define BACKLIGHT_MAX 255

#define ORIG_PWM_MAX 255
#define ORIG_PWM_DEF 143
#define ORIG_PWM_MIN 30

#define MAP_PWM_MAX	255
#define MAP_PWM_DEF 	110
#define MAP_PWM_MIN	7

static unsigned char vigor_shrink_pwm(int val)
{
	unsigned char shrink_br;

	/* define line segments for Vigor */
	if ( val <= ORIG_PWM_MIN )
		shrink_br = MAP_PWM_MIN;
	else if ( val > ORIG_PWM_MIN && val <= ORIG_PWM_DEF )
		shrink_br = MAP_PWM_MIN +
			(val-ORIG_PWM_MIN)*(MAP_PWM_DEF-MAP_PWM_MIN)/(ORIG_PWM_DEF-ORIG_PWM_MIN);
	else
		shrink_br = MAP_PWM_DEF +
			(val-ORIG_PWM_DEF)*(MAP_PWM_MAX-MAP_PWM_DEF)/(ORIG_PWM_MAX-ORIG_PWM_DEF);

	pr_info("brightness orig = %d, transformed=%d\n", val, shrink_br);

	return shrink_br;
}

static struct msm_panel_common_pdata mipi_panel_data = {
	.shrink_pwm = vigor_shrink_pwm,
};

static struct platform_device mipi_dsi_cmd_panel_device = {
	.name = "mipi_himax",
	.id = 0,
	.dev = {
		.platform_data = &mipi_panel_data,
	}
};

#ifdef CONFIG_BT
static struct platform_device vigor_rfkill = {
	.name = "vigor_rfkill",
	.id = -1,
};

static struct htc_sleep_clk_platform_data htc_slp_clk_data = {
	.sleep_clk_pin = VIGOR_WIFI_BT_SLEEP_CLK,

};

static struct platform_device wifi_bt_slp_clk = {
	.name = "htc_slp_clk",
	.id = -1,
	.dev = {
		.platform_data = &htc_slp_clk_data,
	},
};
#endif


#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* defaults to bitmap don't edit */
	.cached = 0,
};

static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 6,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

#define PMEM_BUS_WIDTH(_bw) \
	{ \
		.vectors = &(struct msm_bus_vectors){ \
			.src = MSM_BUS_MASTER_AMPSS_M0, \
			.dst = MSM_BUS_SLAVE_SMI, \
			.ib = (_bw), \
			.ab = 0, \
		}, \
	.num_paths = 1, \
	}
static struct msm_bus_paths pmem_smi_table[] = {
	[0] = PMEM_BUS_WIDTH(0), /* Off */
	[1] = PMEM_BUS_WIDTH(1), /* On */
};

static struct msm_bus_scale_pdata smi_client_pdata = {
	.usecase = pmem_smi_table,
	.num_usecases = ARRAY_SIZE(pmem_smi_table),
	.name = "pmem_smi",
};

void pmem_request_smi_region(void *data)
{
	int bus_id = (int) data;

	msm_bus_scale_client_update_request(bus_id, 1);
}

void pmem_release_smi_region(void *data)
{
	int bus_id = (int) data;

	msm_bus_scale_client_update_request(bus_id, 0);
}

void *pmem_setup_smi_region(void)
{
	return (void *)msm_bus_scale_register_client(&smi_client_pdata);
}
static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.request_region = pmem_request_smi_region,
	.release_region = pmem_release_smi_region,
	.setup_region = pmem_setup_smi_region,
	.map_on_demand = 1,
};
static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 7,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};

#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static int hdmi_cec_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static void __init msm8x60_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	msm_fb_resources[0].start = MSM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, __va(msm_fb_resources[0].start),
		(unsigned long)msm_fb_resources[0].start);

	addr = alloc_bootmem(MSM_PMEM_RMT_STORAGE_SIZE);
	pr_info("allocating %d bytes at %p (0x%lx physical) for "
		"mdm rmt_storage\n", MSM_PMEM_RMT_STORAGE_SIZE,
		addr, __pa(addr));
	rmt_storage_add_mem(__pa(addr), MSM_PMEM_RMT_STORAGE_SIZE);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = PMEM_KERNEL_SMI_SIZE;
	if (size) {
		android_pmem_kernel_smi_pdata.start = PMEM_KERNEL_SMI_BASE;
		android_pmem_kernel_smi_pdata.size = size;
		pr_info("allocating %lu bytes at %lx physical for kernel"
			" smi pmem arena\n", size,
			(unsigned long) PMEM_KERNEL_SMI_BASE);
	}
#endif

#ifdef CONFIG_ANDROID_PMEM
	size = pmem_adsp_size;
	if (size) {
		android_pmem_adsp_pdata.start = MSM_PMEM_ADSP_BASE;
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, __va(android_pmem_adsp_pdata.start),
			(unsigned long)android_pmem_adsp_pdata.start);
	}

	size = MSM_PMEM_SMIPOOL_SIZE;
	if (size) {
		android_pmem_smipool_pdata.start = MSM_PMEM_SMIPOOL_BASE;
		android_pmem_smipool_pdata.size = size;
		pr_info("allocating %lu bytes at %lx physical for user"
			" smi  pmem arena\n", size,
			(unsigned long) MSM_PMEM_SMIPOOL_BASE);
	}

	size = MSM_PMEM_AUDIO_SIZE;
	if (size) {
		android_pmem_audio_pdata.start = MSM_PMEM_AUDIO_BASE;
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, __va(android_pmem_audio_pdata.start),
			(unsigned long)android_pmem_audio_pdata.start);
	}

	size = pmem_sf_size;
	if (size) {
		android_pmem_pdata.start = MSM_PMEM_SF_BASE;
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size,  __va(android_pmem_pdata.start),
			(unsigned long)android_pmem_pdata.start);
	}
#endif
}

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(VIGOR_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = VIGOR_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = VIGOR_GPIO_BT_HOST_WAKE,
};
#endif

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)

static struct msm_rpm_log_platform_data msm_rpm_log_pdata = {
	.phys_addr_base = 0x00106000,
	.reg_offsets = {
		[MSM_RPM_LOG_PAGE_INDICES] = 0x00000C80,
		[MSM_RPM_LOG_PAGE_BUFFER]  = 0x00000CA0,
	},
	.phys_size = SZ_8K,
	.log_len = 4096,		  /* log's buffer length in bytes */
	.log_len_mask = (4096 >> 2) - 1,  /* length mask in units of u32 */
};

static struct platform_device msm_rpm_log_device = {
	.name	= "msm_rpm_log",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_log_pdata,
	},
};
#endif

#ifdef CONFIG_BATTERY_MSM8X60
static struct msm_charger_platform_data msm_charger_data = {
	.safety_time = 180,
	.update_time = 1,
	.max_voltage = 4200,
	.min_voltage = 3200,
	.resume_voltage = 4100,
};

static struct platform_device msm_charger_device = {
	.name = "msm-charger",
	.id = -1,
	.dev = {
		.platform_data = &msm_charger_data,
	}
};
#endif

static struct regulator_consumer_supply rpm_vreg_supply[RPM_VREG_ID_MAX] = {
	[RPM_VREG_ID_PM8058_L0]  = REGULATOR_SUPPLY("8058_l0", NULL),
	[RPM_VREG_ID_PM8058_L1]  = REGULATOR_SUPPLY("8058_l1", NULL),
	[RPM_VREG_ID_PM8058_L2]  = REGULATOR_SUPPLY("8058_l2", NULL),
	[RPM_VREG_ID_PM8058_L3]  = REGULATOR_SUPPLY("8058_l3", NULL),
	[RPM_VREG_ID_PM8058_L4]  = REGULATOR_SUPPLY("8058_l4", NULL),
	[RPM_VREG_ID_PM8058_L5]  = REGULATOR_SUPPLY("8058_l5", NULL),
	[RPM_VREG_ID_PM8058_L6]  = REGULATOR_SUPPLY("8058_l6", NULL),
	[RPM_VREG_ID_PM8058_L7]  = REGULATOR_SUPPLY("8058_l7", NULL),
	[RPM_VREG_ID_PM8058_L8]  = REGULATOR_SUPPLY("8058_l8", NULL),
	[RPM_VREG_ID_PM8058_L9]  = REGULATOR_SUPPLY("8058_l9", NULL),
	[RPM_VREG_ID_PM8058_L10] = REGULATOR_SUPPLY("8058_l10", NULL),
	[RPM_VREG_ID_PM8058_L11] = REGULATOR_SUPPLY("8058_l11", NULL),
	[RPM_VREG_ID_PM8058_L12] = REGULATOR_SUPPLY("8058_l12", NULL),
	[RPM_VREG_ID_PM8058_L13] = REGULATOR_SUPPLY("8058_l13", NULL),
	[RPM_VREG_ID_PM8058_L14] = REGULATOR_SUPPLY("8058_l14", NULL),
	[RPM_VREG_ID_PM8058_L15] = REGULATOR_SUPPLY("8058_l15", NULL),
	[RPM_VREG_ID_PM8058_L16] = REGULATOR_SUPPLY("8058_l16", NULL),
	[RPM_VREG_ID_PM8058_L17] = REGULATOR_SUPPLY("8058_l17", NULL),
	[RPM_VREG_ID_PM8058_L18] = REGULATOR_SUPPLY("8058_l18", NULL),
	[RPM_VREG_ID_PM8058_L19] = REGULATOR_SUPPLY("8058_l19", NULL),
	[RPM_VREG_ID_PM8058_L20] = REGULATOR_SUPPLY("8058_l20", NULL),
	[RPM_VREG_ID_PM8058_L21] = REGULATOR_SUPPLY("8058_l21", NULL),
	[RPM_VREG_ID_PM8058_L22] = REGULATOR_SUPPLY("8058_l22", NULL),
	[RPM_VREG_ID_PM8058_L23] = REGULATOR_SUPPLY("8058_l23", NULL),
	[RPM_VREG_ID_PM8058_L24] = REGULATOR_SUPPLY("8058_l24", NULL),
	[RPM_VREG_ID_PM8058_L25] = REGULATOR_SUPPLY("8058_l25", NULL),

	[RPM_VREG_ID_PM8058_S0] = REGULATOR_SUPPLY("8058_s0", NULL),
	[RPM_VREG_ID_PM8058_S1] = REGULATOR_SUPPLY("8058_s1", NULL),
	[RPM_VREG_ID_PM8058_S2] = REGULATOR_SUPPLY("8058_s2", NULL),
	[RPM_VREG_ID_PM8058_S3] = REGULATOR_SUPPLY("8058_s3", NULL),
	[RPM_VREG_ID_PM8058_S4] = REGULATOR_SUPPLY("8058_s4", NULL),

	[RPM_VREG_ID_PM8058_LVS0] = REGULATOR_SUPPLY("8058_lvs0", NULL),
	[RPM_VREG_ID_PM8058_LVS1] = REGULATOR_SUPPLY("8058_lvs1", NULL),

	[RPM_VREG_ID_PM8058_NCP] = REGULATOR_SUPPLY("8058_ncp", NULL),

	[RPM_VREG_ID_PM8901_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[RPM_VREG_ID_PM8901_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[RPM_VREG_ID_PM8901_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[RPM_VREG_ID_PM8901_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[RPM_VREG_ID_PM8901_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[RPM_VREG_ID_PM8901_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[RPM_VREG_ID_PM8901_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[RPM_VREG_ID_PM8901_S2] = REGULATOR_SUPPLY("8901_s2", NULL),
	[RPM_VREG_ID_PM8901_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[RPM_VREG_ID_PM8901_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[RPM_VREG_ID_PM8901_LVS0] = REGULATOR_SUPPLY("8901_lvs0", NULL),
	[RPM_VREG_ID_PM8901_LVS1] = REGULATOR_SUPPLY("8901_lvs1", NULL),
	[RPM_VREG_ID_PM8901_LVS2] = REGULATOR_SUPPLY("8901_lvs2", NULL),
	[RPM_VREG_ID_PM8901_LVS3] = REGULATOR_SUPPLY("8901_lvs3", NULL),
	[RPM_VREG_ID_PM8901_MVS0] = REGULATOR_SUPPLY("8901_mvs0", NULL),
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct msm_serial_hslite_platform_data msm_uart_gsbi9_pdata = {
	.config_gpio	= 1,
	.uart_tx_gpio	= 67,
	.uart_rx_gpio	= 66,
};
#endif

#define RPM_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
		      _default_uV, _peak_uA, _avg_uA, _pull_down, _pin_ctrl, \
		      _freq, _pin_fn, _rpm_mode, _state, _sleep_selectable, \
		      _always_on) \
	[RPM_VREG_ID_##_id] = { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = \
				&rpm_vreg_supply[RPM_VREG_ID_##_id], \
		}, \
		.default_uV = _default_uV, \
		.peak_uA = _peak_uA, \
		.avg_uA = _avg_uA, \
		.pull_down_enable = _pull_down, \
		.pin_ctrl = _pin_ctrl, \
		.freq = _freq, \
		.pin_fn = _pin_fn, \
		.mode = _rpm_mode, \
		.state = _state, \
		.sleep_selectable = _sleep_selectable, \
	}

/*
 * The default LPM/HPM state of an RPM controlled regulator can be controlled
 * via the peak_uA value specified in the table below.  If the value is less
 * than the high power min threshold for the regulator, then the regulator will
 * be set to LPM.  Otherwise, it will be set to HPM.
 *
 * This value can be further overridden by specifying an initial mode via
 * .init_data.constraints.initial_mode.
 */

#define RPM_VREG_INIT_LDO(_id, _always_on, _pd, _sleep_selectable, _min_uV, _max_uV, _init_peak_uA, _pin_ctrl)\
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_LDO_PF(_id, _always_on, _pd, _sleep_selectable, _min_uV,  _max_uV, _init_peak_uA, _pin_ctrl, _pin_fn)\
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      _pin_fn, RPM_VREG_MODE_NONE, RPM_VREG_STATE_OFF, \
		      _sleep_selectable, _always_on)

#define RPM_VREG_INIT_SMPS(_id, _always_on, _pd, _sleep_selectable, _min_uV, _max_uV, _init_peak_uA, _pin_ctrl, _freq)\
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, _freq, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_VS(_id, _always_on, _pd, _sleep_selectable, _pin_ctrl) \
	RPM_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE, \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE, 0, 0, \
		      1000, 1000, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_NCP(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
		      REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, 0, \
		      _min_uV, 1000, 1000, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define LDO50HMIN	RPM_VREG_LDO_50_HPM_MIN_LOAD
#define LDO150HMIN	RPM_VREG_LDO_150_HPM_MIN_LOAD
#define LDO300HMIN	RPM_VREG_LDO_300_HPM_MIN_LOAD
#define SMPS_HMIN	RPM_VREG_SMPS_HPM_MIN_LOAD
#define FTS_HMIN	RPM_VREG_FTSMPS_HPM_MIN_LOAD

static struct rpm_vreg_pdata rpm_vreg_init_pdata[RPM_VREG_ID_MAX] = {

	RPM_VREG_INIT_LDO(PM8058_L0,  0, 1, 0, 1200000, 1200000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L1,  0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L2,  0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L3,  0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L4,  0, 1, 0, 2850000, 2850000, LDO50HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L5,  0, 1, 0, 2850000, 2850000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L6,  0, 1, 0, 3000000, 3600000, LDO50HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L7,  0, 1, 0, 1800000, 1800000, LDO50HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L8,  0, 1, 0, 1800000, 1800000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L9,  0, 1, 0, 1800000, 1800000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L10, 0, 1, 0, 2800000, 2800000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L11, 0, 1, 0, 2850000, 2850000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L12, 0, 1, 0, 3200000, 3200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L13, 0, 1, 0, 2050000, 2050000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L14, 0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L15, 0, 1, 0, 2800000, 2800000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L16, 1, 1, 1, 1800000, 1800000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L17, 0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L18, 0, 1, 1, 2200000, 2200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L19, 0, 1, 0, 1800000, 1800000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L20, 0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L21, 1, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L22, 0, 1, 0, 1200000, 1200000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L23, 0, 1, 0, 1200000, 1200000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8058_L24, 0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L25, 0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */

	RPM_VREG_INIT_SMPS(PM8058_S0, 0, 1, 1, 500000, 1250000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S1, 0, 1, 1, 500000, 1250000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S2, 0, 1, 0, 1200000, 1400000, SMPS_HMIN,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S3, 1, 1, 0, 1800000, 1800000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S4, 1, 1, 0, 2200000, 2200000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),

	RPM_VREG_INIT_VS(PM8058_LVS0, 0, 1, 0, 0),
	RPM_VREG_INIT_VS(PM8058_LVS1, 0, 1, 0, 0),

	RPM_VREG_INIT_NCP(PM8058_NCP, 0, 1, 0, 1800000, 1800000, 0),

	RPM_VREG_INIT_LDO(PM8901_L0,  0, 1, 0, 1200000, 1200000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8901_L1,  0, 1, 0, 3300000, 3300000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8901_L2,  0, 1, 0, 2850000, 3300000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8901_L3,  0, 1, 0, 3300000, 3300000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8901_L4,  0, 1, 0, 1200000, 1200000, LDO150HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */
	RPM_VREG_INIT_LDO(PM8901_L5,  0, 1, 0, 2850000, 2850000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE),
	RPM_VREG_INIT_LDO(PM8901_L6,  0, 1, 0, 2800000, 2800000, LDO300HMIN, RPM_VREG_PIN_CTRL_NONE), /* N/A */

	RPM_VREG_INIT_SMPS(PM8901_S2, 0, 1, 0, 1200000, 1200000, FTS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8901_S3, 0, 1, 0, 1100000, 1100000, FTS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8901_S4, 0, 1, 0, 1250000, 1250000, FTS_HMIN,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p60),

	RPM_VREG_INIT_VS(PM8901_LVS0, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_LVS1, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_LVS2, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_LVS3, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_MVS0, 0, 1, 0,		     0),
};


#define RPM_VREG(_id) \
	[_id] = { \
		.name = "rpm-regulator", \
		.id = _id, \
		.dev = { \
			.platform_data = &rpm_vreg_init_pdata[_id], \
		}, \
	}

static struct platform_device rpm_vreg_device[RPM_VREG_ID_MAX] = {
	RPM_VREG(RPM_VREG_ID_PM8058_L0),
	RPM_VREG(RPM_VREG_ID_PM8058_L1),
	RPM_VREG(RPM_VREG_ID_PM8058_L2),
	RPM_VREG(RPM_VREG_ID_PM8058_L3),
	RPM_VREG(RPM_VREG_ID_PM8058_L4),
	RPM_VREG(RPM_VREG_ID_PM8058_L5),
	RPM_VREG(RPM_VREG_ID_PM8058_L6),
	RPM_VREG(RPM_VREG_ID_PM8058_L7),
	RPM_VREG(RPM_VREG_ID_PM8058_L8),
	RPM_VREG(RPM_VREG_ID_PM8058_L9),
	RPM_VREG(RPM_VREG_ID_PM8058_L10),
	RPM_VREG(RPM_VREG_ID_PM8058_L11),
	RPM_VREG(RPM_VREG_ID_PM8058_L12),
	RPM_VREG(RPM_VREG_ID_PM8058_L13),
	RPM_VREG(RPM_VREG_ID_PM8058_L14),
	RPM_VREG(RPM_VREG_ID_PM8058_L15),
	RPM_VREG(RPM_VREG_ID_PM8058_L16),
	RPM_VREG(RPM_VREG_ID_PM8058_L17),
	RPM_VREG(RPM_VREG_ID_PM8058_L18),
	RPM_VREG(RPM_VREG_ID_PM8058_L19),
	RPM_VREG(RPM_VREG_ID_PM8058_L20),
	RPM_VREG(RPM_VREG_ID_PM8058_L21),
	RPM_VREG(RPM_VREG_ID_PM8058_L22),
	RPM_VREG(RPM_VREG_ID_PM8058_L23),
	RPM_VREG(RPM_VREG_ID_PM8058_L24),
	RPM_VREG(RPM_VREG_ID_PM8058_L25),
	RPM_VREG(RPM_VREG_ID_PM8058_S0),
	RPM_VREG(RPM_VREG_ID_PM8058_S1),
	RPM_VREG(RPM_VREG_ID_PM8058_S2),
	RPM_VREG(RPM_VREG_ID_PM8058_S3),
	RPM_VREG(RPM_VREG_ID_PM8058_S4),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8058_NCP),
	RPM_VREG(RPM_VREG_ID_PM8901_L0),
	RPM_VREG(RPM_VREG_ID_PM8901_L1),
	RPM_VREG(RPM_VREG_ID_PM8901_L2),
	RPM_VREG(RPM_VREG_ID_PM8901_L3),
	RPM_VREG(RPM_VREG_ID_PM8901_L4),
	RPM_VREG(RPM_VREG_ID_PM8901_L5),
	RPM_VREG(RPM_VREG_ID_PM8901_L6),
	RPM_VREG(RPM_VREG_ID_PM8901_S2),
	RPM_VREG(RPM_VREG_ID_PM8901_S3),
	RPM_VREG(RPM_VREG_ID_PM8901_S4),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS2),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS3),
	RPM_VREG(RPM_VREG_ID_PM8901_MVS0),
};

static struct platform_device *early_regulators[] __initdata = {
	&msm_device_saw_s0,
	&msm_device_saw_s1,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S1],
#endif
};

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_MSM_BUS_SCALING
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
#endif
	&msm_device_dmov_adm0,
	&msm_device_dmov_adm1,
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name   = "aux_pcm_dout",
		.start  = 111,
		.end    = 111,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 112,
		.end    = 112,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 113,
		.end    = 113,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 114,
		.end    = 114,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

#if 0
static uint32_t mi2s_config_gpio[] = {
	GPIO_CFG(107, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(101, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(102, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(103, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void fm_mi2s_enable(void)
{
	gpio_tlmm_config(mi2s_config_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(mi2s_config_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(mi2s_config_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(mi2s_config_gpio[3], GPIO_CFG_ENABLE);
}

static void fm_mi2s_disable(void)
{
	gpio_tlmm_config(mi2s_config_gpio[0], GPIO_CFG_DISABLE);
	gpio_tlmm_config(mi2s_config_gpio[1], GPIO_CFG_DISABLE);
	gpio_tlmm_config(mi2s_config_gpio[2], GPIO_CFG_DISABLE);
	gpio_tlmm_config(mi2s_config_gpio[3], GPIO_CFG_DISABLE);
}

static struct resource msm_mi2s_gpio_resources[] = {

	{
		.name   = "mi2s_ws",
		.start  = 101,
		.end    = 101,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "mi2s_sclk",
		.start  = 102,
		.end    = 102,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "mi2s_mclk",
		.start  = 103,
		.end    = 103,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "fm_i2s_sd",
		.start  = 107,
		.end    = 107,
		.flags  = IORESOURCE_IO,
	},
};

static struct msm_mi2s_gpio_data gpio_data = {

	.enable		 = fm_mi2s_enable,
	.disable	 = fm_mi2s_disable,
};

static struct platform_device msm_mi2s_device = {
	.name		= "msm_mi2s",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(msm_mi2s_gpio_resources),
	.resource	= msm_mi2s_gpio_resources,
	.dev		= { .platform_data = &gpio_data },
};
#endif

#ifdef CONFIG_SENSORS_M_ADC
static struct adc_access_fn xoadc_fn = {
	pm8058_xoadc_select_chan_and_start_conv,
	pm8058_xoadc_read_adc_code,
	pm8058_xoadc_get_properties,
	pm8058_xoadc_slot_request,
	pm8058_xoadc_restore_slot,
	pm8058_xoadc_calibrate,
};

static struct msm_adc_channels msm_adc_channels_data[] = {
	{"vbatt", CHANNEL_ADC_VBATT, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"vcoin", CHANNEL_ADC_VCOIN, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
	{"vcharger_channel", CHANNEL_ADC_VCHG, 0, &xoadc_fn, CHAN_PATH_TYPE13,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE4, scale_default},
	{"charger_current_monitor", CHANNEL_ADC_CHG_MONITOR, 0, &xoadc_fn,
		CHAN_PATH_TYPE4,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_default},
	{"vph_pwr", CHANNEL_ADC_VPH_PWR, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"usb_vbus", CHANNEL_ADC_USB_VBUS, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"pmic_therm", CHANNEL_ADC_DIE_TEMP, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_pmic_therm},
	{"pmic_therm_4K", CHANNEL_ADC_DIE_TEMP_4K, 0, &xoadc_fn,
		CHAN_PATH_TYPE12,
		ADC_CONFIG_TYPE1, ADC_CALIB_CONFIG_TYPE7, scale_pmic_therm},
	{"xo_therm", CHANNEL_ADC_XOTHERM, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE5, tdkntcgtherm},
	{"xo_therm_4K", CHANNEL_ADC_XOTHERM_4K, 0, &xoadc_fn,
		CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE1, ADC_CALIB_CONFIG_TYPE6, tdkntcgtherm},
	{"hdset_detect", CHANNEL_ADC_HDSET, 0, &xoadc_fn, CHAN_PATH_TYPE6,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_default},
	{"chg_batt_amon", CHANNEL_ADC_BATT_AMON, 0, &xoadc_fn, CHAN_PATH_TYPE7,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1,
		scale_default},
	{"batt_therm", CHANNEL_ADC_BATT_THERM, 0, &xoadc_fn,
		CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
	{"batt_id", CHANNEL_ADC_BATT_ID, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
};

static struct msm_adc_platform_data msm_adc_pdata = {
	.channel = msm_adc_channels_data,
	.num_chan_supported = ARRAY_SIZE(msm_adc_channels_data),
};

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};
#endif

static uint32_t headset_gpio_table[] = {
	GPIO_CFG(VIGOR_GPIO_AUD_HP_DET, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
		 GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_AUD_HP_CHARM_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_AUD_HP_CHARM_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
};

static uint32_t h2w_gpio_table[] = {
	GPIO_CFG(VIGOR_H2W_IO1_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_H2W_IO2_DAT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_H2W_CABLE_IN1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_H2W_CABLE_IN2, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
};

static void headset_init(void)
{
	pr_info("[HS_BOARD] (%s) Headset initiation\n", __func__);

	config_gpio_table(headset_gpio_table, ARRAY_SIZE(headset_gpio_table));
	gpio_set_value(VIGOR_AUD_HP_CHARM_PWR, 1);
	gpio_set_value(VIGOR_AUD_HP_CHARM_SEL, 0);

	if ((board_build_flag() != MFG_BUILD) && !(get_kernel_flag() & BIT1)) {
		pr_info("[HS_BOARD] (%s) Enable H2W GPIO\n", __func__);
		config_gpio_table(h2w_gpio_table, ARRAY_SIZE(h2w_gpio_table));
	} else
		pr_info("[HS_BOARD] (%s) Disable H2W GPIO\n", __func__);
}

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= VIGOR_GPIO_AUD_HP_DET,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.driver_flag		= 0,
	.hpin_gpio		= 0,
	.hpin_irq		= 0,
	.key_gpio		= PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_REMO_PRES),
	.key_irq		= 0,
	.key_enable_gpio	= 0,
	.adc_mic_bias		= {0, 0},
	.adc_remote		= {0, 0, 0, 0, 0, 0},
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

/* HTC_HEADSET_MISC Driver */
static struct htc_headset_misc_platform_data htc_headset_misc_data = {
	.driver_flag		= DRIVER_HS_MISC_EXT_HP_DET,
	.ext_hpin_gpio		= VIGOR_H2W_IO1_CLK, /* VIGOR_H2W_CABLE_IN1 */
	.ext_accessory_type	= USB_AUDIO_OUT,
	.indicator_power_gpio	= VIGOR_AUD_HP_CHARM_SEL,
};

static struct platform_device htc_headset_misc = {
	.name	= "HTC_HEADSET_MISC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_misc_data,
	},
};

/* HTC_HEADSET_8X60 Driver */
static struct htc_headset_8x60_platform_data htc_headset_8x60_data = {
	.adc_mpp	= XOADC_MPP_10,
	.adc_amux	= PM_MPP_AIN_AMUX_CH5,
	.adc_mic_bias	= {HS_DEF_MIC_ADC_15_BIT_MIN,
			   HS_DEF_MIC_ADC_15_BIT_MAX},
	.adc_remote	= {0, 857, 858, 2815, 2816, 6024},
};

static struct platform_device htc_headset_8x60 = {
	.name	= "HTC_HEADSET_8X60",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_8x60_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_pmic,
	&htc_headset_misc,
	&htc_headset_8x60,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct headset_adc_config htc_headset_mgr_config[] = {
	{
		.type = HEADSET_MIC,
		.adc_max = 27713,
		.adc_min = 22092,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 22091,
		.adc_min = 16271,
	},
	{
		.type = HEADSET_BEATS_SOLO,
		.adc_max = 16270,
		.adc_min = 10045,
	},
	{
		.type = HEADSET_INDICATOR,
		.adc_max = 10044,
		.adc_min = 4523,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 4522,
		.adc_min = 0,
	},
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.driver_flag		= 0,
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
	.headset_config_num	= ARRAY_SIZE(htc_headset_mgr_config),
	.headset_config		= htc_headset_mgr_config,
	.headset_init		= headset_init,
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct pm8058_led_config pm_led_config[] = {
	{
		.name = "green",
		.type = PM8058_LED_RGB,
		.bank = 0,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "amber",
		.type = PM8058_LED_RGB,
		.bank = 1,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "charming-led",
		.type = PM8058_LED_DRVX,
		.bank = 5,
		.flags = PM8058_LED_LTU_EN | PM8058_LED_BLINK_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 16,
		.duites_size = 16,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_LOOP |
			    PM_PWM_LUT_REVERSE,
		.out_current = 10,
	},
	{
		.name = "button-backlight",
		.type = PM8058_LED_DRVX,
		.bank = 6,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},

};

static struct pm8058_led_platform_data pm8058_leds_data = {
	.led_config = pm_led_config,
	.num_leds = ARRAY_SIZE(pm_led_config),
	.duties = {0, 15, 30, 45, 60, 75, 90, 100,
		   100, 90, 75, 60, 45, 30, 15, 0,
		   0, 0, 4, 12, 20, 28, 36, 44,
		   52, 60, 68, 76, 84, 92, 100, 100,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0},
};

static struct platform_device pm8058_leds = {
	.name	= "leds-pm8058",
	.id	= -1,
	.dev	= {
		.platform_data	= &pm8058_leds_data,
	},
};

#ifdef CONFIG_MSM_SDIO_AL
static uint32_t mdm2ap_gpio_table[] = {
	GPIO_CFG(VIGOR_MDM2AP_STATUS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_MDM2AP_VDDMIN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_AP2MDM_WAKEUP, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static int configure_mdm2ap_status(int on)
{
	return 0;
#if 0
	int ret = 0;
	if (on)
		ret = msm_gpiomux_get(VIGOR_MDM2AP_STATUS);
	else
		ret = msm_gpiomux_put(VIGOR_MDM2AP_STATUS);

	if (ret)
		pr_err("%s: mdm2ap_status config failed, on = %d\n", __func__,
		       on);

	return ret;
#endif
}


static int get_mdm2ap_status(void)
{
	return gpio_get_value(VIGOR_MDM2AP_VDDMIN);
}

static void trigger_mdm_fatal(void)
{
	gpio_set_value(VIGOR_AP2MDM_ERRFATAL, 1);
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.trigger_mdm_fatal = trigger_mdm_fatal,
	.allow_sdioc_version_major_2 = 0,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
	.mdm2ap_errfatal_gpio = VIGOR_MDM2AP_ERRFATAL,
};

static struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};
#endif /* CONFIG_MSM_SDIO_AL */

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int mdm9k_status;

static int mdm_loaded_status_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	int ret;
	char *p = page;

	if (off > 0) {
		ret = 0;
	} else {
		p += sprintf(p, "%d\n", mdm9k_status);
		ret = p - page;
	}

	return ret;
}

static void mdm_loaded_info(void)
{
	struct proc_dir_entry *entry = NULL;

	mdm9k_status = 0;
	entry = create_proc_read_entry("mdm9k_status", 0, NULL, mdm_loaded_status_proc, NULL);
}

static void charm_ap2mdm_kpdpwr_on(void)
{
	pr_info("Trigger ap2mdm_kpdpwr...\n");
	if (system_rev == VIGOR_EVM_PCB_ID)	// VigorEVM
		gpio_set_value(VIGOR_AP2MDM_KPDPWR_N_EVM, 1);
	else
		gpio_set_value(VIGOR_AP2MDM_KPDPWR_N, 1);
}

static void charm_ap2mdm_kpdpwr_off(void)
{
	pr_info("Release ap2mdm_kpdpwr...\n");
	if (system_rev == VIGOR_EVM_PCB_ID)	// VigorEVM
		gpio_set_value(VIGOR_AP2MDM_KPDPWR_N_EVM, 0);
	else
		gpio_set_value(VIGOR_AP2MDM_KPDPWR_N, 0);

	mdm9k_status = 1;
}

static void charm_ap2mdm_pmic_reset(void)
{
	pr_info("Trigger ap2mdm_pmic_reset...(%d ms)\n", VIGOR_AP2MDM_PMIC_RESET_TIME_MS);
	gpio_set_value(VIGOR_AP2MDM_PMIC_RESET_N, 1);
	msleep(VIGOR_AP2MDM_PMIC_RESET_TIME_MS);
	pr_info("Release ap2mdm_pmic_reset...\n");
	gpio_set_value(VIGOR_AP2MDM_PMIC_RESET_N, 0);
}

static void charm_ap_suspend(void)
{
	pr_info("charm_ap suspending...\n");
	gpio_set_value(VIGOR_MDM2AP_WAKEUP, 0);
}

static void charm_ap_resume(void)
{
	pr_info("charm_ap resuming...\n");
	gpio_set_value(VIGOR_MDM2AP_WAKEUP, 1);
}

static struct resource charm_resources[] = {
	{
		.start	= MSM_GPIO_TO_INT(VIGOR_MDM2AP_ERRFATAL),
		.end	= MSM_GPIO_TO_INT(VIGOR_MDM2AP_ERRFATAL),
		.flags = IORESOURCE_IRQ,
	},
	/* MDM2AP_STATUS */
	{
		.start	= MSM_GPIO_TO_INT(VIGOR_MDM2AP_STATUS),
		.end	= MSM_GPIO_TO_INT(VIGOR_MDM2AP_STATUS),
		.flags = IORESOURCE_IRQ,
	}
};

static struct charm_platform_data mdm_platform_data_evm = {
	.charm_modem_on	 = charm_ap2mdm_kpdpwr_on,
	.charm_modem_off = charm_ap2mdm_kpdpwr_off,
	.charm_modem_reset = charm_ap2mdm_pmic_reset,
	.charm_modem_suspend = charm_ap_suspend,
	.charm_modem_resume = charm_ap_resume,

	.gpio_ap2mdm_status = VIGOR_AP2MDM_STATUS,
	.gpio_ap2mdm_wakeup = VIGOR_AP2MDM_WAKEUP,
	.gpio_ap2mdm_errfatal = VIGOR_AP2MDM_ERRFATAL,
	/*.gpio_ap2mdm_sync = VIGOR_AP2MDM_SYNC,*/
	.gpio_ap2mdm_pmic_reset_n = VIGOR_AP2MDM_PMIC_RESET_N_EVM,
	.gpio_ap2mdm_kpdpwr_n = VIGOR_AP2MDM_KPDPWR_N_EVM,
//	.gpio_ap2pmic_tmpni_cken = VIGOR_AP2PMIC_TMPNI_CKEN,

	.gpio_mdm2ap_status = VIGOR_MDM2AP_STATUS,
	.gpio_mdm2ap_wakeup = VIGOR_MDM2AP_WAKEUP,
	.gpio_mdm2ap_errfatal = VIGOR_MDM2AP_ERRFATAL,
	.gpio_mdm2ap_sync = VIGOR_MDM2AP_SYNC,
//	.gpio_mdm2ap_vfr = VIGOR_MDM2AP_VFR,
};

static struct platform_device msm_charm_modem_evm = {
	.name		= "charm_modem",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(charm_resources),
	.resource	= charm_resources,
	.dev		= {
		.platform_data = &mdm_platform_data_evm,
	},
};

static struct platform_device *charm_devices_evm[] __initdata = {
	&msm_charm_modem_evm,
};

static struct charm_platform_data mdm_platform_data = {
	.charm_modem_on	 = charm_ap2mdm_kpdpwr_on,
	.charm_modem_off = charm_ap2mdm_kpdpwr_off,
	.charm_modem_reset = charm_ap2mdm_pmic_reset,
	.charm_modem_suspend = charm_ap_suspend,
	.charm_modem_resume = charm_ap_resume,

	.gpio_ap2mdm_status = VIGOR_AP2MDM_STATUS,
	.gpio_ap2mdm_wakeup = VIGOR_AP2MDM_WAKEUP,
	.gpio_ap2mdm_errfatal = VIGOR_AP2MDM_ERRFATAL,
	/*.gpio_ap2mdm_sync = VIGOR_AP2MDM_SYNC,*/
	.gpio_ap2mdm_pmic_reset_n = VIGOR_AP2MDM_PMIC_RESET_N,
	.gpio_ap2mdm_kpdpwr_n = VIGOR_AP2MDM_KPDPWR_N,
//	.gpio_ap2pmic_tmpni_cken = VIGOR_AP2PMIC_TMPNI_CKEN,

	.gpio_mdm2ap_status = VIGOR_MDM2AP_STATUS,
	.gpio_mdm2ap_wakeup = VIGOR_MDM2AP_WAKEUP,
	.gpio_mdm2ap_errfatal = VIGOR_MDM2AP_ERRFATAL,
	.gpio_mdm2ap_sync = VIGOR_MDM2AP_SYNC,
//	.gpio_mdm2ap_vfr = VIGOR_MDM2AP_VFR,
};

static struct platform_device msm_charm_modem = {
	.name		= "charm_modem",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(charm_resources),
	.resource	= charm_resources,
	.dev		= {
		.platform_data = &mdm_platform_data,
	},
};

static struct platform_device *charm_devices[] __initdata = {
	&msm_charm_modem,
};

static struct platform_device msm_tsens_device = {
	.name   = "tsens-tm",
	.id = -1,
};

static struct platform_device scm_log_device = {
	.name	= "scm-log",
	.id = -1,
};

static struct platform_device *surf_devices[] __initdata = {
	&ram_console_device,
	&msm_device_smd,
#ifdef CONFIG_WIMAX_SERIAL_MSM
	&msm_device_uart3,
#endif
#ifdef CONFIG_I2C_QUP
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi5_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi3_qup_spi_device,
#endif
#ifdef CONFIG_BT
	&wifi_bt_slp_clk,
	&vigor_rfkill,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_MSM8X60_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
	&msm_device_pm8058,
	&msm_device_pm8901,
#else
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#endif /* CONFIG_MSM8X60_SSBI */
#ifdef CONFIG_MSM_DSPS
	&msm_dsps_device,
#endif
#ifdef CONFIG_USB_ANDROID_QCT_DIAG
       &usb_diag_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
        &msm_kgsl_3d0,
#ifdef CONFIG_MSM_KGSL_2D
        &msm_kgsl_2d0,
        &msm_kgsl_2d1,
#endif
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
	&mipi_dsi_cmd_sharp_qhd_panel_device,
	&mipi_dsi_cmd_panel_device,
#ifdef CONFIG_S5K3H2YX
	&msm_camera_sensor_s5k3h2yx,
#endif
#ifdef CONFIG_MT9D015
	&msm_camera_sensor_webcam, /* for front camera */
#endif
#ifdef CONFIG_OV8830
	&msm_camera_sensor_ov8830,
#endif

#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
#if defined(CONFIG_MSM_RPM_STATS_LOG)
	&msm_rpm_stat_device,
#endif
#ifdef CONFIG_BATTERY_MSM8X60
	&msm_charger_device,
#endif
	&msm_device_vidc,
	&msm_aux_pcm_device,
#ifdef CONFIG_SENSORS_M_ADC
	&msm_adc_device,
#endif
#ifdef CONFIG_HTC_BATT8x60
	&htc_battery_pdev,
#endif
	&htc_headset_mgr,
	&pm8058_leds,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L7],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L8],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L9],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L10],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L11],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L12],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L13],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L14],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L15],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L16],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L17],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L18],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L19],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L20],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L21],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L22],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L23],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L24],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L25],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_NCP],
#endif
#ifdef CONFIG_PMIC8901
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_MVS0],
#endif
	&cable_detect_device,
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif

#ifdef CONFIG_HW_RANDOM_MSM
	&msm_device_rng,
#endif
	&msm_tsens_device,
	&scm_log_device,
};

#ifdef CONFIG_PMIC8058

static int pm8058_gpios_init(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};

	struct pm8058_gpio_cfg gpio_cfgs[] = {
		{ /* SD card detect */
			VIGOR_SD_DETECT_PIN,	/* 3 */
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* Audio remote button event */
			VIGOR_AUD_REMO_PRES,	/* 6 */
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM_GPIO_VIN_L5,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
		{ /* Audio Speaker Amplifier */
			VIGOR_AUD_AMP_EN,	/* 18 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 6,	/* LDO5 2.85 V */
				.inv_int_pol	= 0,
			}
		},
		{ /* Audio Receiver Amplifier */
			VIGOR_AUD_REC_EN,	/* 19 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 6,	/* LDO5 2.85 V */
				.inv_int_pol	= 0,
			}
		},
		{ /* Timpani Reset */
			VIGOR_AUD_QTR_RESET,	/* 20 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_DN,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 2,
				.inv_int_pol	= 0,
			}
		},
		{ /* Green LED */
			VIGOR_GREEN_LED,	/* 23 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
			}
		},
		{ /* AMBER */
			VIGOR_AMBER_LED,	/* 24 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
			}
		},
		{ /* cm3629 P/L-sensor */
			VIGOR_PS_INT,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_31P5,
				.vin_sel        = PM_GPIO_VIN_L5,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
		},
		{ /* Audio Microphone Selector */
			VIGOR_AUD_MIC_SEL,	/* 36 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 6,	/* LDO5 2.85 V */
				.inv_int_pol	= 0,
			}
		},
	};

	for (i = 0; i < ARRAY_SIZE(gpio_cfgs); ++i) {
		rc = pm8058_gpio_config(gpio_cfgs[i].gpio,
				&gpio_cfgs[i].cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				__func__);
			return rc;
		}
	}

	return 0;
}

static struct resource resources_pwrkey[] = {
	{
		.start	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pmic8058_pwrkey_pdata pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us   = 970,
	.wakeup			= 1,
	.pwrkey_time_ms		= 500,
};

static struct pmic8058_vibrator_pdata pmic_vib_pdata = {
	.initial_vibrate_ms  = 500,
	.level_mV = 3000,
	.max_timeout_ms = 15000,
};

#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
#define PM8058_OTHC_CNTR_BASE0	0xA0
#define PM8058_OTHC_CNTR_BASE1	0x134
#define PM8058_OTHC_CNTR_BASE2	0x137

/* HTC OTHC Driver - BEGIN */
#if 0
static struct othc_switch_info htc_headset_othc_switch_info[] = {
	{
		.min_adc_threshold = 0,
		.max_adc_threshold = 50,
		.key_code = HS_MGR_KEY_PLAY,
	},
	{
		.min_adc_threshold = 50,
		.max_adc_threshold = 200,
		.key_code = HS_MGR_KEY_BACKWARD,
	},
	{
		.min_adc_threshold = 200,
		.max_adc_threshold = 500,
		.key_code = HS_MGR_KEY_FORWARD,
	},
};

static struct othc_n_switch_config htc_headset_othc_switch_config = {
	.voltage_settling_time_ms = 0,
	.num_adc_samples = 1,
	.adc_channel = CHANNEL_ADC_HDSET,
	.switch_info = htc_headset_othc_switch_info,
	.num_keys = ARRAY_SIZE(htc_headset_othc_switch_info),
};

static struct hsed_bias_config htc_headset_othc_bias_config = {
	.othc_headset = OTHC_HEADSET_NO,
	.othc_lowcurr_thresh_uA = 100,
	.othc_highcurr_thresh_uA = 500,
	.othc_hyst_prediv_us = 3000,
	.othc_period_clkdiv_us = 3000,
	.othc_hyst_clk_us = 45000,
	.othc_period_clk_us = 6000,
	.othc_wakeup = 1,
};

static struct othc_hsed_config htc_headset_othc_config = {
	.hsed_bias_config = &htc_headset_othc_bias_config,
	.detection_delay_ms = 200,
	/* Switch info */
	.switch_debounce_ms = 1000,
	.othc_support_n_switch = true,
	.switch_config = &htc_headset_othc_switch_config,
};

static struct pmic8058_othc_config_pdata htc_headset_othc_pdata = {
	.micbias_select = OTHC_MICBIAS_2,
	.micbias_capability = OTHC_MICBIAS_HSED,
	.micbias_enable = OTHC_SIGNAL_PWM_TCXO,
	.hsed_config = &htc_headset_othc_config,
	.hsed_name = "htc_headset",
};

static struct resource htc_headset_othc_resources[] = {
	{
		.start = PM8058_SW_2_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_SW_2_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8058_IR_2_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_IR_2_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE2,
		.end   = PM8058_OTHC_CNTR_BASE2,
		.flags = IORESOURCE_IO,
	},
};
#endif
/* HTC OTHC Driver - END */

/* MIC_BIAS0 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_0 = {
	.micbias_select = OTHC_MICBIAS_0,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

/* MIC_BIAS1 is configured as normal for OTHC */
static struct pmic8058_othc_config_pdata othc_config_pdata_1 = {
	.micbias_select = OTHC_MICBIAS_1,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

static struct resource resources_othc_0[] = {
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE0,
		.end   = PM8058_OTHC_CNTR_BASE0,
		.flags = IORESOURCE_IO,
	},
};

static struct resource resources_othc_1[] = {
	{
		.start = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE1,
		.end   = PM8058_OTHC_CNTR_BASE1,
		.flags = IORESOURCE_IO,
	},
};
#endif /* CONFIG_PMIC8058_OTHC || CONFIG_PMIC8058_OTHC_MODULE */

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_VPH,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

	int rc = -EINVAL;
	int id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
					__func__, id, rc);
		}
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	case 7:
		id = PM_PWM_LED_FLASH1;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	default:
		break;
	}

	if (ch >= 6 && ch <= 7) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}
	return rc;

}

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
};

#define PM8058_GPIO_INT           88

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PM8058_IRQ_BASE, 0),
};

#define PM8058_VREG(_id) { \
	.name = "pm8058-regulator", \
	.id = _id, \
	.platform_data = &pm8058_vreg_init[_id], \
	.data_size = sizeof(pm8058_vreg_init[_id]), \
}

static struct resource resources_rtc[] = {
       {
		.start  = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
       {
		.start  = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
};

static struct pmic8058_led pmic8058_flash_leds[] = {
	[0] = {
		.name		= "camera:flash0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[1] = {
		.name		= "camera:flash1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_flash_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_flash_leds),
	.leds	= pmic8058_flash_leds,
};

static struct mfd_cell pm8058_subdevs[] = {
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwrkey",
		.id	= -1,
		.resources = resources_pwrkey,
		.num_resources = ARRAY_SIZE(resources_pwrkey),
		.platform_data = &pwrkey_pdata,
		.data_size = sizeof(pwrkey_pdata),
	},
	{
		.name = "pm8058-vib",
		.id = -1,
		.platform_data = &pmic_vib_pdata,
		.data_size     = sizeof(pmic_vib_pdata),
	},
	{
		.name = "pm8058-pwm",
		.id = -1,
		.platform_data = &pm8058_pwm_data,
		.data_size = sizeof(pm8058_pwm_data),
	},
#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
	{
		.name = "pm8058-othc",
		.id = 0,
		.platform_data = &othc_config_pdata_0,
		.data_size = sizeof(othc_config_pdata_0),
		.num_resources = ARRAY_SIZE(resources_othc_0),
		.resources = resources_othc_0,
	},
	{
		/* OTHC1 module has headset/switch dection */
		.name = "pm8058-othc",
		.id = 1,
		.num_resources = ARRAY_SIZE(resources_othc_1),
		.resources = resources_othc_1,
		.platform_data = &othc_config_pdata_1,
		.data_size = sizeof(othc_config_pdata_1),
	},
#if 0
	{
		.name = "pm8058-othc",
		.id = 2,
		.platform_data = &htc_headset_othc_pdata,
		.data_size = sizeof(htc_headset_othc_pdata),
		.num_resources = ARRAY_SIZE(htc_headset_othc_resources),
		.resources = htc_headset_othc_resources,
	},
#endif
#endif /* CONFIG_PMIC8058_OTHC || CONFIG_PMIC8058_OTHC_MODULE */
#ifdef CONFIG_SENSORS_M_ADC
	{
		.name = "pm8058-xoadc",
		.id = -1,
		.num_resources = 1,
		.resources = &resources_adc,
		.platform_data = &xoadc_pdata,
		.data_size = sizeof(xoadc_pdata),
	},
#endif
	{
		.name = "pm8058-rtc",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{	.name = "pm8058-led",
		.id		= -1,
		.platform_data = &pm8058_flash_leds_data,
		.data_size = sizeof(pm8058_flash_leds_data),
	},
	{	.name = "pm8058-upl",
		.id		= -1,
	},
#ifdef CONFIG_PMIC8058_BATTALARM
	{
		.name = "pm8058-batt-alarm",
		.id = -1,
		.num_resources = 1,
		.resources = &resources_batt_alarm,
	},
#endif
};

#ifdef CONFIG_MSM8X60_SSBI
static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,
	.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};
#else
static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
		.platform_data = &pm8058_platform_data,
	},
};
#endif /* CONFIG_MSM8X60_SSBI */
#endif /* CONFIG_PMIC8058 */

static struct regulator *vreg_timpani_1;
static struct regulator *vreg_timpani_2;

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	pr_info("%s", __func__);

	vreg_timpani_1 = regulator_get(NULL, "8058_l0");
	if (IS_ERR(vreg_timpani_1)) {
		pr_err("%s: Unable to get 8058_l0\n", __func__);
		return -ENODEV;
	}

	vreg_timpani_2 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(vreg_timpani_2)) {
		pr_err("%s: Unable to get 8058_s3\n", __func__);
		regulator_put(vreg_timpani_1);
		return -ENODEV;
	}

	rc = regulator_set_voltage(vreg_timpani_1, 1200000, 1200000);
	if (rc) {
		pr_err("%s: unable to set L0 voltage to 1.2V\n", __func__);
		goto fail;
	}

	rc = regulator_set_voltage(vreg_timpani_2, 1800000, 1800000);
	if (rc) {
		pr_err("%s: unable to set S3 voltage to 1.8V\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_1);
	if (rc) {
		pr_err("%s: Enable regulator 8058_l0 failed\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_2);
	if (rc) {
		pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);
		regulator_disable(vreg_timpani_1);
		goto fail;
	}

	return rc;

fail:
	regulator_put(vreg_timpani_1);
	regulator_put(vreg_timpani_2);
	return rc;
}

static void msm_timpani_shutdown_power(void)
{
	int rc;

	pr_info("%s", __func__);

	rc = regulator_disable(vreg_timpani_1);
	if (rc)
		pr_err("%s: Disable regulator 8058_l0 failed\n", __func__);

	regulator_put(vreg_timpani_1);

	rc = regulator_disable(vreg_timpani_2);
	if (rc)
		pr_err("%s: Disable regulator 8058_s3 failed\n", __func__);

	regulator_put(vreg_timpani_2);
}

/* Power analog function of codec */
static struct regulator *vreg_timpani_cdc_apwr;
static int msm_timpani_codec_power(int vreg_on)
{
	int rc = 0;

	pr_info("%s: %d", __func__, vreg_on);

	if (!vreg_timpani_cdc_apwr) {

		vreg_timpani_cdc_apwr = regulator_get(NULL, "8058_s4");

		if (IS_ERR(vreg_timpani_cdc_apwr)) {
			pr_err("%s: vreg_get failed (%ld)\n",
			__func__, PTR_ERR(vreg_timpani_cdc_apwr));
			rc = PTR_ERR(vreg_timpani_cdc_apwr);
			return rc;
		}
	}

	if (vreg_on) {

		rc = regulator_set_voltage(vreg_timpani_cdc_apwr,
				2200000, 2200000);
		if (rc) {
			pr_err("%s: unable to set 8058_s4 voltage to 2.2 V\n",
					__func__);
			goto vreg_fail;
		}

		rc = regulator_enable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_enable failed %d\n", __func__, rc);
			goto vreg_fail;
		}
	} else {
		rc = regulator_disable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_disable failed %d\n",
			__func__, rc);
			goto vreg_fail;
		}
	}

	return 0;

vreg_fail:
	regulator_put(vreg_timpani_cdc_apwr);
	vreg_timpani_cdc_apwr = NULL;
	return rc;
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_timpani_codec_power,
};

#define TIMPANI_SLAVE_ID_CDC_ADDR		0X77
#define TIMPANI_SLAVE_ID_QMEMBIST_ADDR		0X66

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= TIMPANI_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = TIMPANI_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

static struct tpa2051d3_platform_data tpa2051d3_pdata = {
	.gpio_tpa2051_spk_en = VIGOR_AUD_AMP_EN,
	.spkr_cmd = {0x00, 0x82, 0x25, 0x57, 0x2D, 0xCD, 0x0D},
	.hsed_cmd = {0x00, 0x0C, 0x25, 0x57, 0x6D, 0x4D, 0x0D},
	.rece_cmd = {0x00, 0x82, 0x25, 0x57, 0x2D, 0x4D, 0x0D},
};

#define TPA2051D3_I2C_SLAVE_ADDR	(0xE0 >> 1)

static struct i2c_board_info msm_i2c_gsbi7_tpa2051d3_info[] = {
	{
		I2C_BOARD_INFO(TPA2051D3_I2C_NAME, TPA2051D3_I2C_SLAVE_ADDR),
		.platform_data = &tpa2051d3_pdata,
	},
};

static int vigor_ts_atmel_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_TP_RST), 0);
	msleep(5);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_TP_RST), 1);
	msleep(40);

	return 0;
}

struct atmel_i2c_platform_data vigor_ts_atmel_data[] = {
	{
		.version = 0x0011,
		.build = 0x01,
		.source = 1, /* YFO */
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1885,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = VIGOR_TP_ATT_N,
		.power = vigor_ts_atmel_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 12, 25},
		.config_T8 = {25, 0, 5, 5, 0, 64, 5, 35, 5, 192},
		.config_T9 = {139, 0, 0, 20, 10, 0, 32, 50, 2, 1, 0, 5, 2, 1, 4, 15, 15, 10, 255, 7, 255, 3, 3, 254, 15, 18, 143, 45, 145, 75, 27, 15, 53, 58, 1},
		.config_T18 = {0, 0},
		.config_T19 = {3, 4, 0, 60, 0, 0},
		.config_T25 = {3, 0, 176, 104, 4, 91},
		.config_T35 = {63, 50, 4, 1, 50, 0, 0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 40, 35, 128, 2, 0, 10},
		.config_T46 = {0, 4, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 66, 0, 0, 0, 0, 0, 0, 0, 16, 25, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T55 = {0, 0, 0, 0},
		.object_crc = {0x79, 0x6C, 0xA3},
		.wlc_freq = {40, 10, 20, 1},
		.mferr_config = {100, 3, 2, 0, 0, 0, 0, 0, 63, 18, 30, 90},
	},
	{
		.version = 0x0011,
		.build = 0x01,
		.source = 0, /* TPK */
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1885,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = VIGOR_TP_ATT_N,
		.power = vigor_ts_atmel_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 12, 25},
		.config_T8 = {25, 0, 5, 5, 0, 64, 5, 35, 5, 192},
		.config_T9 = {139, 0, 0, 20, 10, 0, 32, 50, 2, 1, 0, 5, 2, 1, 4, 15, 15, 10, 255, 7, 255, 3, 6, 0, 32, 32, 162, 36, 129, 240, 27, 15, 53, 58, 1},
		.config_T18 = {0, 0},
		.config_T19 = {3, 4, 0, 60, 0, 0},
		.config_T25 = {3, 0, 176, 104, 4, 91},
		.config_T35 = {63, 50, 4, 1, 50, 0, 0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 40, 35, 128, 2, 0, 10},
		.config_T46 = {0, 4, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 66, 0, 0, 0, 0, 0, 0, 0, 16, 25, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T55 = {0, 0, 0, 0},
		.object_crc = {0x24, 0xD5, 0xDD},
		.wlc_freq = {40, 10, 20, 1},
		.mferr_config = {100, 3, 2, 0, 0, 0, 0, 0, 63, 18, 30, 90},
	},
	{
		.version = 0x0011,
		.build = 0xF8,
		.source = 1, /* YFO */
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1885,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = VIGOR_TP_ATT_N,
		.power = vigor_ts_atmel_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 8, 25},
		.config_T8 = {25, 0, 5, 5, 0, 64, 5, 35, 5, 192},
		.config_T9 = {139, 0, 0, 20, 10, 0, 32, 50, 2, 1, 0, 5, 2, 1, 4, 10, 10, 10, 255, 7, 255, 3, 3, 254, 15, 18, 143, 45, 145, 75, 30, 13, 53, 58, 4},
		.config_T18 = {0, 0},
		.config_T19 = {3, 4, 0, 60, 0, 0},
		.config_T25 = {3, 0, 176, 104, 4, 91},
		.config_T42 = {0, 0, 40, 35, 128, 2, 0, 10},
		.config_T46 = {0, 4, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 64, 25, 0, 0, 0, 0, 0, 0, 16, 15, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 30, 0, 1, 15, 5, 0, 0, 0, 35, 3, 5, 2, 1, 5, 10, 10, 241, 241, 22, 27, 158, 60, 153, 75, 18, 17, 4},
		/*.config_T48 = {15, 0, 64, 0, 0, 0, 0, 0, 12, 12, 16, 32, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 32, 1, 15, 5, 0, 0, 0, 35, 3, 5, 2, 1, 5, 10, 10, 241, 241, 22, 27, 158, 60, 153, 75, 18, 17, 4},*/
		.config_T55 = {0, 0, 0, 0},
		.object_crc = {0xC9, 0xE1, 0x75},
		.wlc_freq = {34, 12, 24, 45},
	},
	{
		.version = 0x0011,
		.build = 0xF8,
		.source = 0, /* TPK */
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1885,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = VIGOR_TP_ATT_N,
		.power = vigor_ts_atmel_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 8, 25},
		.config_T8 = {25, 0, 5, 5, 0, 64, 5, 35, 5, 192},
		.config_T9 = {139, 0, 0, 20, 10, 0, 32, 50, 2, 1, 0, 5, 2, 1, 4, 10, 10, 10, 255, 7, 255, 3, 10, 0, 10, 20, 134, 60, 148, 82, 30, 13, 53, 58, 4},
		.config_T18 = {0, 0},
		.config_T19 = {3, 4, 0, 60, 0, 0},
		.config_T25 = {3, 0, 176, 104, 4, 91},
		.config_T42 = {0, 0, 40, 35, 128, 2, 0, 10},
		.config_T46 = {0, 4, 16, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 0, 64, 25, 0, 0, 0, 0, 0, 0, 16, 15, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 30, 0, 1, 15, 5, 0, 0, 0, 35, 3, 5, 2, 1, 5, 10, 10, 241, 241, 22, 27, 158, 60, 153, 75, 18, 17, 4},
		/*.config_T48 = {15, 0, 64, 0, 0, 0, 0, 0, 12, 12, 16, 32, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 32, 1, 15, 5, 0, 0, 0, 35, 3, 5, 2, 1, 5, 10, 10, 241, 241, 22, 27, 158, 60, 153, 75, 18, 17, 4},*/
		.config_T55 = {0, 0, 0, 0},
		.object_crc = {0x8B, 0xA5, 0x4C},
		.wlc_freq = {34, 12, 24, 45},
	},
	{
		.version = 0x0010,
		.source = 1, /* YFO */
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1885,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = VIGOR_TP_ATT_N,
		.power = vigor_ts_atmel_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 8, 25},
		.config_T8 = {25, 0, 5, 20, 0, 0, 5, 35, 5, 192},
		.config_T9 = {139, 0, 0, 20, 10, 0, 32, 50, 3, 1, 0, 5, 2, 1, 4, 10, 10, 10, 255, 7, 255, 3, 3, 254, 15, 18, 143, 45, 145, 75, 30, 13, 53, 58, 4},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 4, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 176, 104, 4, 91, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 40, 35, 128, 2, 0, 10},
		.config_T46 = {0, 4, 8, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 196, 96, 0, 0, 0, 0, 0, 12, 12, 0, 0, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 0, 0, 0, 0, 0, 0, 35, 3, 5, 2, 1, 5, 10, 10, 241, 241, 22, 27, 158, 60, 153, 75, 18, 17, 4},
		.object_crc = {0x0B, 0x76, 0x2E},
		.wlc_freq = {34, 12, 24, 45},
	},
	{
		.version = 0x0010,
		.source = 0, /* TPK */
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 1885,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = VIGOR_TP_ATT_N,
		.power = vigor_ts_atmel_power,
		.unlock_attr = 1,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 8, 25},
		.config_T8 = {25, 0, 5, 20, 0, 0, 5, 35, 5, 192},
		.config_T9 = {139, 0, 0, 20, 10, 0, 32, 50, 3, 1, 0, 5, 2, 1, 4, 10, 10, 10, 255, 7, 255, 3, 10, 0, 10, 20, 134, 60, 148, 82, 30, 13, 53, 58, 4},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 4, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 176, 104, 4, 91, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T40 = {0, 0, 0, 0, 0},
		.config_T42 = {0, 0, 40, 35, 128, 2, 0, 10},
		.config_T46 = {0, 4, 8, 16, 0, 0, 0, 0, 0},
		.config_T47 = {0, 20, 50, 5, 2, 50, 40, 0, 0, 63},
		.config_T48 = {15, 196, 96, 0, 0, 0, 0, 0, 12, 12, 0, 0, 0, 6, 6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20, 0, 0, 0, 0, 0, 0, 0, 35, 3, 5, 2, 1, 5, 10, 10, 241, 241, 22, 27, 158, 60, 153, 75, 18, 17, 4},
		.object_crc = {0x22, 0xC5, 0xD7},
		.wlc_freq = {34, 12, 24, 45},
	},
};

static struct i2c_board_info msm_i2c_gsbi5_info[] = {
	{
		I2C_BOARD_INFO(ATMEL_MXT224E_NAME, 0x94 >> 1),
		.platform_data = &vigor_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(VIGOR_TP_ATT_N)
	},
};

#ifdef CONFIG_PMIC8901

#define PM8901_GPIO_INT           91

#ifdef CONFIG_FB_MSM_HDMI_MHL
static int pm8901_mpp_init(void)
{
	int rc;
	pr_err("%s\n", __func__);

	rc = pm8901_mpp_config(0, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_MSMIO,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);


	rc = pm8901_mpp_config(1, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_L5,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);


	rc = pm8901_mpp_config(2, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_MSMIO,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);


	rc = pm8901_mpp_config(3, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_L5,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);

	return rc;
}
#endif

static struct pm8901_gpio_platform_data pm8901_mpp_data = {
	.gpio_base	= PM8901_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8901_MPP_IRQ(PM8901_IRQ_BASE, 0),
};

static struct resource pm8901_temp_alarm[] = {
	{
		.start = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
};

static struct regulator_consumer_supply pm8901_vreg_supply[PM8901_VREG_MAX] = {
	[PM8901_VREG_ID_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[PM8901_VREG_ID_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[PM8901_VREG_ID_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[PM8901_VREG_ID_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[PM8901_VREG_ID_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[PM8901_VREG_ID_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[PM8901_VREG_ID_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[PM8901_VREG_ID_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[PM8901_VREG_ID_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[PM8901_VREG_ID_LVS0]     = REGULATOR_SUPPLY("8901_lvs0",     NULL),
	[PM8901_VREG_ID_LVS1]     = REGULATOR_SUPPLY("8901_lvs1",     NULL),
	[PM8901_VREG_ID_LVS2]     = REGULATOR_SUPPLY("8901_lvs2",     NULL),
	[PM8901_VREG_ID_LVS3]     = REGULATOR_SUPPLY("8901_lvs3",     NULL),
	[PM8901_VREG_ID_MVS0]     = REGULATOR_SUPPLY("8901_mvs0",     NULL),
	[PM8901_VREG_ID_USB_OTG]  = REGULATOR_SUPPLY("8901_usb_otg",  NULL),
	[PM8901_VREG_ID_HDMI_MVS] = REGULATOR_SUPPLY("8901_hdmi_mvs", NULL),
};

#define PM8901_VREG_INIT(_id, _min_uV, _max_uV, \
		_modes, _ops, _apply_uV, _init) \
	[_id] = { \
		.constraints = { \
			.valid_modes_mask = _modes, \
			.valid_ops_mask = _ops, \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.apply_uV = _apply_uV, \
		}, \
		.num_consumer_supplies = 1, \
		.consumer_supplies = &pm8901_vreg_supply[_id], \
		.regulator_init = _init, \
	}

#define PM8901_VREG_INIT_LDO(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, NULL)

#define PM8901_VREG_INIT_SMPS(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, NULL)

#define PM8901_VREG_INIT_VS(_id, _init) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, _init)

static struct regulator_init_data pm8901_vreg_init[PM8901_VREG_MAX] = {
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_USB_OTG,  NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_HDMI_MVS, NULL),
};

#define PM8901_VREG(_id) { \
	.name = "pm8901-regulator", \
	.id = _id, \
	.platform_data = &pm8901_vreg_init[_id], \
	.data_size = sizeof(pm8901_vreg_init[_id]), \
}

static struct mfd_cell pm8901_subdevs[] = {
	{	.name = "pm8901-mpp",
		.id		= -1,
		.platform_data	= &pm8901_mpp_data,
		.data_size	= sizeof(pm8901_mpp_data),
	},
	{	.name = "pm8901-tm",
		.id		= -1,
		.num_resources  = ARRAY_SIZE(pm8901_temp_alarm),
		.resources      = pm8901_temp_alarm,
	},
	PM8901_VREG(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG(PM8901_VREG_ID_HDMI_MVS),
};

#ifdef CONFIG_MSM8X60_SSBI
static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};
#else
static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8901_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8901-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
		.platform_data = &pm8901_platform_data,
	},
};
#endif /* CONFIG_MSM8X60_SSBI */
#endif /* CONFIG_PMIC8901 */

static struct cm3629_platform_data cm3629_XA02_pdata = {
	.model = CAPELLA_CM36292,
	.ps_select = CM3629_PS2_ONLY,
	.intr = PM8058_GPIO_PM_TO_SYS(VIGOR_PS_INT),
	.levels = { 25, 28, 34, 51, 372, 899, 1474, 2433, 3392, 65535},
	.golden_adc = 0x410,
	.power = NULL,
	.cm3629_slave_address = 0xC0>>1,
	.ps2_thd_set = 0x19,
	.ps2_thd_no_cal = 0xD8,
	.ps2_thd_with_cal = 0x19,
	.ps_calibration_rule = 3,
	.ps_conf1_val = CM3629_PS_DR_1_80 | CM3629_PS_IT_1_6T |
			CM3629_PS1_PERS_1,
	.ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
			CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
	.ps_conf3_val = CM3629_PS2_PROL_32,
};

static struct i2c_board_info i2c_CM3629_XA02_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME, 0xC0 >> 1),
		.platform_data = &cm3629_XA02_pdata,
		.irq = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, VIGOR_PS_INT),
	},
};

static struct cm3629_platform_data cm3629_pdata = {
	.model = CAPELLA_CM36292,
	.ps_select = CM3629_PS2_ONLY,
	.intr = PM8058_GPIO_PM_TO_SYS(VIGOR_PS_INT),
	.levels = { 25, 28, 34, 96, 372, 899, 1474, 2433, 3392, 65535},
	.golden_adc = 0x410,
	.power = NULL,
	.cm3629_slave_address = 0xC0>>1,
	.ps2_thd_set = 0x07,
	.ps2_thd_no_cal = 0xBF,
	.ps2_thd_with_cal = 0x07,
	.ps_calibration_rule = 3,
	.ps_conf1_val = CM3629_PS_DR_1_80 | CM3629_PS_IT_1_6T |
			CM3629_PS1_PERS_1,
	.ps_conf2_val = CM3629_PS_ITB_1 | CM3629_PS_ITR_1 |
			CM3629_PS2_INT_DIS | CM3629_PS1_INT_DIS,
	.ps_conf3_val = CM3629_PS2_PROL_32,
};

static struct i2c_board_info i2c_CM3629_devices[] = {
	{
		I2C_BOARD_INFO(CM3629_I2C_NAME, 0xC0 >> 1),
		.platform_data = &cm3629_pdata,
		.irq = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, VIGOR_PS_INT),
	},
};

#if 0
static int isl29028_power(int pwr_device, uint8_t enable)
{
	return 0;
}

static struct isl29028_platform_data isl29028_pdata = {
	.intr = PM8058_GPIO_PM_TO_SYS(VIGOR_PLS_INT),
	.levels = { 1, 3, 15, 29, 58, 339,
			588, 728, 869, 4095},
	.golden_adc = 450,
	.power = isl29028_power,
	.lt = 10,
	.ht = 11,
};

static struct i2c_board_info i2c_isl29028_devices[] = {
	{
		I2C_BOARD_INFO(ISL29028_I2C_NAME, 0x8A >> 1),
		.platform_data = &isl29028_pdata,
		.irq = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, VIGOR_PLS_INT),
	},
};
#endif
static struct mpu3050_platform_data mpu3050_data = {
	.int_config = 0x10,
	.orientation = { -1, 0, 0,
			 0, 1, 0,
			 0, 0, -1 },
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num = 10, /* i2c bus: MSM_GSBI12_QUP_I2C_BUS_ID */
		.bus = EXT_SLAVE_BUS_SECONDARY,
		.address = 0x30 >> 1,
			.orientation = { 1, 0, 0,
					0, 1, 0,
					0, 0, 1 },

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num = 10, /* i2c bus: MSM_GSBI12_QUP_I2C_BUS_ID */
		.bus = EXT_SLAVE_BUS_PRIMARY,
		.address = 0x1A >> 1,
			.orientation = { 1, 0, 0,
					0, 1, 0,
					0, 0, 1 },
	},
};

#if 0
static struct i2c_board_info __initdata mpu3050_GSBI10_boardinfo[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0xD0 >> 1),
		/*.irq = MSM_GPIO_TO_INT(VIGOR_GPIO_GYRO_INT),*/
		.platform_data = &mpu3050_data,
	},
};
#endif

static struct i2c_board_info __initdata mpu3050_GSBI12_boardinfo[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0xD0 >> 1),
		.irq = MSM_GPIO_TO_INT(VIGOR_GPIO_GYRO_INT),
		.platform_data = &mpu3050_data,
	},
};

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry msm8x60_i2c_devices[] __initdata = {
#ifndef CONFIG_MSM8X60_SSBI
#ifdef CONFIG_PMIC8058
	{
		I2C_SURF | I2C_FFA,
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo,
		ARRAY_SIZE(pm8058_boardinfo),
	},
#endif
#ifdef CONFIG_PMIC8901
	{
		I2C_SURF | I2C_FFA,
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
#endif
#endif /* CONFIG_MSM8X60_SSBI */
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_tps_65200_boardinfo,
		ARRAY_SIZE(msm_tps_65200_boardinfo),
	},
#ifdef CONFIG_FB_MSM_HDMI_MHL
#ifdef CONFIG_FB_MSM_HDMI_MHL_SII9234
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_mhl_sii9234_info,
		ARRAY_SIZE(msm_i2c_gsbi7_mhl_sii9234_info),
	},
#endif
#endif
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI5_QUP_I2C_BUS_ID,
		msm_i2c_gsbi5_info,
		ARRAY_SIZE(msm_i2c_gsbi5_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_timpani_info,
		ARRAY_SIZE(msm_i2c_gsbi7_timpani_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_tpa2051d3_info,
		ARRAY_SIZE(msm_i2c_gsbi7_tpa2051d3_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI12_QUP_I2C_BUS_ID,
		mpu3050_GSBI12_boardinfo,
		ARRAY_SIZE(mpu3050_GSBI12_boardinfo),
	},
};
#endif /* CONFIG_I2C */

static struct i2c_registry msm8x60_i2c_devices_cam_s5k3h2yx[] __initdata = {
#ifdef CONFIG_MSM_CAMERA
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo),
	},
#endif
};

static struct i2c_registry msm8x60_i2c_devices_cam_ov8830[] __initdata = {
#ifdef CONFIG_MSM_CAMERA
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo_OV8830,
		ARRAY_SIZE(msm_camera_boardinfo_OV8830),
	},
#endif
};

static void register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;

	/* Build the matching 'supported_machs' bitmask */
	mach_mask = I2C_SURF;

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8x60_i2c_devices); ++i) {
		pr_err("%s: i = %d\n", __func__, i);
		pr_err("%s\n", msm8x60_i2c_devices[i].info->type);
		if (msm8x60_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(msm8x60_i2c_devices[i].bus,
						msm8x60_i2c_devices[i].info,
						msm8x60_i2c_devices[i].len);
	}

	if (cam_id_kvalue == 0)
	{
		i2c_register_board_info(msm8x60_i2c_devices_cam_s5k3h2yx[0].bus,
		msm8x60_i2c_devices_cam_s5k3h2yx[0].info,
		msm8x60_i2c_devices_cam_s5k3h2yx[0].len);
	}
	else
	{
		i2c_register_board_info(msm8x60_i2c_devices_cam_ov8830[0].bus,
		msm8x60_i2c_devices_cam_ov8830[0].info,
		msm8x60_i2c_devices_cam_ov8830[0].len);
	}
	if (engineerid >= 0x2) {
		i2c_register_board_info(MSM_GSBI12_QUP_I2C_BUS_ID,
				i2c_CM3629_XA02_devices, ARRAY_SIZE(i2c_CM3629_XA02_devices));
		pr_info("[PS][cm3629]%s: use P-senor THH = 0x1a, engineerid = 0x%x\n", __func__, engineerid);
	} else {
		i2c_register_board_info(MSM_GSBI12_QUP_I2C_BUS_ID,
				i2c_CM3629_devices, ARRAY_SIZE(i2c_CM3629_devices));
		pr_info("[PS][cm3629]%s: use P-senor THH = 0x8, engineerid = 0x%x\n", __func__, engineerid);
	}

#endif
}

static void __init msm8x60_init_buses(void)
{
#ifdef CONFIG_I2C_QUP
	msm_gsbi4_qup_i2c_device.dev.platform_data = &msm_gsbi4_qup_i2c_pdata;
	msm_gsbi5_qup_i2c_device.dev.platform_data = &msm_gsbi5_qup_i2c_pdata;
	msm_gsbi7_qup_i2c_device.dev.platform_data = &msm_gsbi7_qup_i2c_pdata;
	if (system_rev == VIGOR_EVM_PCB_ID)	// VigorEVM
		msm_gsbi10_qup_i2c_device.dev.platform_data = &msm_gsbi10_qup_i2c_pdata;
	else
		msm_gsbi12_qup_i2c_device.dev.platform_data = &msm_gsbi12_qup_i2c_pdata;
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	msm_gsbi3_qup_spi_device.dev.platform_data = &msm_gsbi3_qup_spi_pdata;
#endif
#ifdef CONFIG_MSM8X60_SSBI
	msm_device_ssbi1.dev.platform_data = &msm_ssbi1_pdata;
	msm_device_ssbi2.dev.platform_data = &msm_ssbi2_pdata;
	msm_device_ssbi3.dev.platform_data = &msm_ssbi3_pdata;
	msm_device_pm8058.dev.platform_data = &pm8058_platform_data;
	msm_device_pm8901.dev.platform_data = &pm8901_platform_data;
#else
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi1.dev.platform_data = &msm_ssbi1_pdata;
	msm_device_ssbi2.dev.platform_data = &msm_ssbi2_pdata;
	msm_device_ssbi3.dev.platform_data = &msm_ssbi3_pdata;
#endif
#endif /* CONFIG_MSM8X60_SSBI */
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif

#if defined(CONFIG_USB_F_SERIAL_SDIO) || defined(CONFIG_USB_F_SERIAL_SMD)
	{
		struct usb_gadget_fserial_platform_data *fserial_pdata =
				usb_gadget_fserial_device.dev.platform_data;
		fserial_pdata->no_ports = 5;

		fserial_pdata->transport[0] =
			USB_GADGET_FSERIAL_TRANSPORT_SMD; /* modem */
		fserial_pdata->transport[1] =
			USB_GADGET_FSERIAL_TRANSPORT_TTY;
		fserial_pdata->transport[2] =
			USB_GADGET_FSERIAL_TRANSPORT_TTY;
		fserial_pdata->transport[3] =
			USB_GADGET_FSERIAL_TRANSPORT_TTY;
		fserial_pdata->transport[4] =
			USB_GADGET_FSERIAL_TRANSPORT_SDIO; /* 9k modem */

		fserial_pdata->func_type[0] = USB_FSER_FUNC_MODEM;
		fserial_pdata->func_type[1] = USB_FSER_FUNC_NONE;
		fserial_pdata->func_type[2] = USB_FSER_FUNC_NONE;
		fserial_pdata->func_type[3] = USB_FSER_FUNC_SERIAL;
		fserial_pdata->func_type[4] = USB_FSER_FUNC_MODEM_MDM;

		/* mfgkernel need sdio_tty to open SDIO_DUN */
		if (board_mfg_mode() != 6 && board_mfg_mode() != 7)
			fserial_pdata->usdio_init_flag = 1;
	}
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.rx_wakeup_irq = gpio_to_irq(VIGOR_GPIO_BT_HOST_WAKE);
	msm_device_uart_dm1.name = "msm_serial_hs_brcm"; /* for brcm */
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#ifdef CONFIG_MSM_GSBI9_UART
	msm_device_uart_gsbi9.dev.platform_data =
					&msm_uart_gsbi9_pdata;
	platform_device_register(&msm_device_uart_gsbi9);
#endif

#ifdef CONFIG_MSM_BUS_SCALING

	/* RPM calls are only enabled on V2 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2) {
		msm_bus_apps_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fabric_pdata.rpm_enabled = 1;
		msm_bus_mm_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fpb_pdata.rpm_enabled = 1;
		msm_bus_cpss_fpb_pdata.rpm_enabled = 1;
	}

	msm_bus_apps_fabric.dev.platform_data = &msm_bus_apps_fabric_pdata;
	msm_bus_sys_fabric.dev.platform_data = &msm_bus_sys_fabric_pdata;
	msm_bus_mm_fabric.dev.platform_data = &msm_bus_mm_fabric_pdata;
	msm_bus_sys_fpb.dev.platform_data = &msm_bus_sys_fpb_pdata;
	msm_bus_cpss_fpb.dev.platform_data = &msm_bus_cpss_fpb_pdata;
#endif

}

static void __init vigor_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8x60_io();
	msm8x60_allocate_memory_regions();
}

static void __init msm8x60_init_tlmm(void)
{
}

#define GPIO_SDC3_WP_SWITCH (GPIO_EXPANDER_GPIO_BASE + (16 * 1) + 6)
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC5_SUPPORT))

/* 8x60 is having 5 SDCC controllers */
#define MAX_SDCC_CONTROLLER	5

struct msm_sdcc_gpio {
	/* maximum 10 GPIOs per SDCC controller */
	s16 no;
	/* name of this GPIO */
	const char *name;
};

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct msm_sdcc_gpio sdc1_gpio_cfg[] = {
	{159, "sdc1_dat_0"},
	{160, "sdc1_dat_1"},
	{161, "sdc1_dat_2"},
	{162, "sdc1_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	{163, "sdc1_dat_4"},
	{164, "sdc1_dat_5"},
	{165, "sdc1_dat_6"},
	{166, "sdc1_dat_7"},
#endif
	{167, "sdc1_clk"},
	{168, "sdc1_cmd"}
};
static uint32_t sdc1_on_gpio_table[] = {
	GPIO_CFG(159, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT0 */
	GPIO_CFG(160, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT1 */
	GPIO_CFG(161, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT2 */
	GPIO_CFG(162, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT3 */
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	GPIO_CFG(163, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT4 */
	GPIO_CFG(164, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT5 */
	GPIO_CFG(165, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT6 */
	GPIO_CFG(166, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* DAT7 */
#endif
	GPIO_CFG(167, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* CLK */
	GPIO_CFG(168, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), /* CMD */
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct msm_sdcc_gpio sdc2_gpio_cfg[] = {
	{143, "sdc2_dat_0"},
	{144, "sdc2_dat_1"},
	{145, "sdc2_dat_2"},
	{146, "sdc2_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{147, "sdc2_dat_4"},
	{148, "sdc2_dat_5"},
	{149, "sdc2_dat_6"},
	{150, "sdc2_dat_7"},
#endif
	{151, "sdc2_cmd"},
	{152, "sdc2_clk"}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static struct msm_sdcc_gpio sdc5_gpio_cfg[] = {
	{95, "sdc5_cmd"},
	{96, "sdc5_dat_3"},
	{97, "sdc5_clk"},
	{98, "sdc5_dat_2"},
	{99, "sdc5_dat_1"},
	{100, "sdc5_dat_0"}
};
#endif

struct msm_sdcc_pad_pull_cfg {
	enum msm_tlmm_pull_tgt pull;
	u32 pull_val;
};

struct msm_sdcc_pad_drv_cfg {
	enum msm_tlmm_hdrive_tgt drv;
	u32 drv_val;
};

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc3_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_16MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_10MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_10MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc3_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_DOWN}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc4_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc4_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_DOWN}
};
#endif

struct msm_sdcc_pin_cfg {
	/*
	 * = 1 if controller pins are using gpios
	 * = 0 if controller has dedicated MSM pins
	 */
	u8 is_gpio;
	u8 cfg_sts;
	u8 gpio_data_size;
	struct msm_sdcc_gpio *gpio_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_on_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_off_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_on_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_off_data;
	u8 pad_drv_data_size;
	u8 pad_pull_data_size;
};


static struct msm_sdcc_pin_cfg sdcc_pin_cfg_data[MAX_SDCC_CONTROLLER] = {
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	[0] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc1_gpio_cfg),
		.gpio_data = sdc1_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[1] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc2_gpio_cfg),
		.gpio_data = sdc2_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	[2] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc3_pad_on_drv_cfg,
		.pad_drv_off_data = sdc3_pad_off_drv_cfg,
		.pad_pull_on_data = sdc3_pad_on_pull_cfg,
		.pad_pull_off_data = sdc3_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc3_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc3_pad_on_pull_cfg)
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[3] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc4_pad_on_drv_cfg,
		.pad_drv_off_data = sdc4_pad_off_drv_cfg,
		.pad_pull_on_data = sdc4_pad_on_pull_cfg,
		.pad_pull_off_data = sdc4_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc4_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc4_pad_on_pull_cfg)
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	[4] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc5_gpio_cfg),
		.gpio_data = sdc5_gpio_cfg
	}
#endif
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->gpio_data)
		goto out;

	for (n = 0; n < curr->gpio_data_size; n++) {
		if (enable) {
			rc = gpio_request(curr->gpio_data[n].no,
				curr->gpio_data[n].name);
			if (rc) {
				pr_err("%s: gpio_request(%d, %s)"
					"failed", __func__,
					curr->gpio_data[n].no,
					curr->gpio_data[n].name);
				goto free_gpios;
			}
			/* set direction as output for all GPIOs */
			rc = gpio_direction_output(
				curr->gpio_data[n].no, 1);
			if (rc) {
				pr_err("%s: gpio_direction_output"
					"(%d, 1) failed\n", __func__,
					curr->gpio_data[n].no);
				goto free_gpios;
			}
		} else {
			/*
			 * now free this GPIO which will put GPIO
			 * in low power mode and will also put GPIO
			 * in input mode
			 */
			gpio_free(curr->gpio_data[n].no);
		}
	}
	curr->cfg_sts = enable;
	goto out;

free_gpios:
	for (; n >= 0; n--)
		gpio_free(curr->gpio_data[n].no);
out:
	return rc;
}

static int msm_sdcc_setup_pad(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->pad_drv_on_data || !curr->pad_pull_on_data)
		goto out;

	if (enable) {
		/*
		 * set up the normal driver strength and
		 * pull config for pads
		 */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(curr->pad_drv_on_data[n].drv,
				curr->pad_drv_on_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(curr->pad_pull_on_data[n].pull,
				curr->pad_pull_on_data[n].pull_val);
	} else {
		/* set the low power config for pads */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(
				curr->pad_drv_off_data[n].drv,
				curr->pad_drv_off_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(
				curr->pad_pull_off_data[n].pull,
				curr->pad_pull_off_data[n].pull_val);
	}
	curr->cfg_sts = enable;
out:
	return rc;
}

struct sdcc_reg {
	/* VDD/VCC/VCCQ regulator name on PMIC8058/PMIC8089*/
	const char *reg_name;
	/*
	 * is set voltage supported for this regulator?
	 * 0 = not supported, 1 = supported
	 */
	unsigned char set_voltage_sup;
	/* voltage level to be set */
	unsigned int level;
	/* VDD/VCC/VCCQ voltage regulator handle */
	struct regulator *reg;
	/* is this regulator enabled? */
	bool enabled;
	/* is this regulator needs to be always on? */
	bool always_on;
	/* is operating power mode setting required for this regulator? */
	bool op_pwr_mode_sup;
	/* Load values for low power and high power mode */
	unsigned int lpm_uA;
	unsigned int hpm_uA;
};
/* all SDCC controllers requires VDD/VCC voltage */
static struct sdcc_reg sdcc_vdd_reg_data[MAX_SDCC_CONTROLLER];
/* only SDCC1 requires VCCQ voltage */
static struct sdcc_reg sdcc_vccq_reg_data[1];

struct sdcc_reg_data {
	struct sdcc_reg *vdd_data; /* keeps VDD/VCC regulator info */
	struct sdcc_reg *vccq_data; /* keeps VCCQ regulator info */
	struct sdcc_reg *vddp_data; /* keeps VDD Pad regulator info */
	unsigned char sts; /* regulator enable/disable status */
};
/* msm8x60 have 5 SDCC controllers */
static struct sdcc_reg_data sdcc_vreg_data[MAX_SDCC_CONTROLLER];

static int msm_sdcc_vreg_init_reg(struct sdcc_reg *vreg)
{
	int rc = 0;

	/* Get the regulator handle */
	vreg->reg = regulator_get(NULL, vreg->reg_name);
	if (IS_ERR(vreg->reg)) {
		rc = PTR_ERR(vreg->reg);
		pr_err("%s: regulator_get(%s) failed. rc=%d\n",
			__func__, vreg->reg_name, rc);
		goto out;
	}

	/* Set the voltage level if required */
	if (vreg->set_voltage_sup) {
		rc = regulator_set_voltage(vreg->reg, vreg->level,
					vreg->level);
		if (rc) {
			pr_err("%s: regulator_set_voltage(%s) failed rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto vreg_put;
		}
	}
	goto out;

vreg_put:
	regulator_put(vreg->reg);
out:
	return rc;
}

static inline void msm_sdcc_vreg_deinit_reg(struct sdcc_reg *vreg)
{
	regulator_put(vreg->reg);
}

/* this init function should be called only once for each SDCC */
static int msm_sdcc_vreg_init(int dev_id, unsigned char init)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg, *curr_vccq_reg, *curr_vddp_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;
	curr_vddp_reg = curr->vddp_data;

	if (init) {
		/*
		 * get the regulator handle from voltage regulator framework
		 * and then try to set the voltage level for the regulator
		 */
		if (curr_vdd_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vdd_reg);
			if (rc)
				goto out;
		}
		if (curr_vccq_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vccq_reg);
			if (rc)
				goto vdd_reg_deinit;
		}
		if (curr_vddp_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vddp_reg);
			if (rc)
				goto vccq_reg_deinit;
		}
		goto out;
	} else
		/* deregister with all regulators from regulator framework */
		goto vddp_reg_deinit;

vddp_reg_deinit:
	if (curr_vddp_reg)
		msm_sdcc_vreg_deinit_reg(curr_vddp_reg);
vccq_reg_deinit:
	if (curr_vccq_reg)
		msm_sdcc_vreg_deinit_reg(curr_vccq_reg);
vdd_reg_deinit:
	if (curr_vdd_reg)
		msm_sdcc_vreg_deinit_reg(curr_vdd_reg);
out:
	return rc;
}

static int msm_sdcc_vreg_enable(struct sdcc_reg *vreg)
{
	int rc;

	if (!vreg->enabled) {
		mdelay(5);
		rc = regulator_enable(vreg->reg);
		if (rc) {
			pr_err("%s: regulator_enable(%s) failed. rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto out;
		}
		vreg->enabled = 1;
	}

	/* Put always_on regulator in HPM (high power mode) */
	if (vreg->always_on && vreg->op_pwr_mode_sup) {
		rc = regulator_set_optimum_mode(vreg->reg, vreg->hpm_uA);
		if (rc < 0) {
			pr_err("%s: reg=%s: HPM setting failed"
				" hpm_uA=%d, rc=%d\n",
				__func__, vreg->reg_name,
				vreg->hpm_uA, rc);
			goto vreg_disable;
		}
		rc = 0;
	}
	goto out;

vreg_disable:
	regulator_disable(vreg->reg);
	vreg->enabled = 0;
out:
	return rc;
}

static int msm_sdcc_vreg_disable(struct sdcc_reg *vreg)
{
	int rc;

	/* Never disable always_on regulator */
	if (!vreg->always_on) {
		rc = regulator_disable(vreg->reg);
		if (rc) {
			pr_err("%s: regulator_disable(%s) failed. rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto out;
		}
		vreg->enabled = 0;
	}

	/* Put always_on regulator in LPM (low power mode) */
	if (vreg->always_on && vreg->op_pwr_mode_sup) {
		rc = regulator_set_optimum_mode(vreg->reg, vreg->lpm_uA);
		if (rc < 0) {
			pr_err("%s: reg=%s: LPM setting failed"
				" lpm_uA=%d, rc=%d\n",
				__func__,
				vreg->reg_name,
				vreg->lpm_uA, rc);
			goto out;
		}
		rc = 0;
	}

out:
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned char enable)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg, *curr_vccq_reg, *curr_vddp_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;
	curr_vddp_reg = curr->vddp_data;

	/* check if regulators are initialized or not? */
	if ((curr_vdd_reg && !curr_vdd_reg->reg) ||
		(curr_vccq_reg && !curr_vccq_reg->reg) ||
		(curr_vddp_reg && !curr_vddp_reg->reg)) {
		/* initialize voltage regulators required for this SDCC */
		rc = msm_sdcc_vreg_init(dev_id, 1);
		if (rc) {
			pr_err("%s: regulator init failed = %d\n",
				__func__, rc);
			goto out;
		}
	}

	if (curr->sts == enable)
		goto out;

	if (curr_vdd_reg) {
		if (enable) {
			if(dev_id == 3)
				printk(KERN_INFO "%s: Enabling SD slot power\n", __func__);
			mdelay(5);
			rc = msm_sdcc_vreg_enable(curr_vdd_reg);
		} else {
			if(dev_id == 3)
				printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
			mdelay(5);
			rc = msm_sdcc_vreg_disable(curr_vdd_reg);
		}
		if (rc)
			goto out;
	}

	if (curr_vccq_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vccq_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vccq_reg);
		if (rc)
			goto out;
	}

	if (curr_vddp_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vddp_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vddp_reg);
		if (rc)
			goto out;
	}
	curr->sts = enable;

out:
	return rc;
}

#define EMMC_ID 1

static u32 msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	u32 rc_pin_cfg = 0;
	u32 rc_vreg_cfg = 0;
	u32 rc = 0;
	struct platform_device *pdev;
	struct msm_sdcc_pin_cfg *curr_pin_cfg;

	pdev = container_of(dv, struct platform_device, dev);
	if ((pdev->id == EMMC_ID) && (vdd == 0))
		return rc;
	/* setup gpio/pad */
	curr_pin_cfg = &sdcc_pin_cfg_data[pdev->id - 1];
	if (curr_pin_cfg->cfg_sts == !!vdd)
		goto setup_vreg;

	if (curr_pin_cfg->is_gpio)
		rc_pin_cfg = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	else
		rc_pin_cfg = msm_sdcc_setup_pad(pdev->id, !!vdd);

setup_vreg:
	/* setup voltage regulators */
	rc_vreg_cfg = msm_sdcc_setup_vreg(pdev->id, !!vdd);

	if (rc_pin_cfg || rc_vreg_cfg)
		rc = rc_pin_cfg ? rc_pin_cfg : rc_vreg_cfg;

	return rc;
}

static int msm_sdc3_get_wpswitch(struct device *dev)
{
	struct platform_device *pdev;
	int status;
	pdev = container_of(dev, struct platform_device, dev);

	status = gpio_request(GPIO_SDC3_WP_SWITCH, "SD_WP_Switch");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n",
					__func__, GPIO_SDC3_WP_SWITCH);
	} else {
		status = gpio_get_value_cansleep(GPIO_SDC3_WP_SWITCH);
		pr_info("%s: WP Status for Slot %d = %d\n", __func__,
							pdev->id, status);
		gpio_free(GPIO_SDC3_WP_SWITCH);
	}
	return (unsigned int) status;
}

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
int sdc5_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	sdc5_status_notify_cb = callback;
	sdc5_status_notify_cb_devid = dev_id;
	return 0;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
int sdc2_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	sdc2_status_notify_cb = callback;
	sdc2_status_notify_cb_devid = dev_id;
	return 0;
}
#endif

/* Interrupt handler for SDC2 and SDC5 detection
 * This function uses dual-edge interrputs settings in order
 * to get SDIO detection when the GPIO is rising and SDIO removal
 * when the GPIO is falling */
static irqreturn_t msm8x60_multi_sdio_slot_status_irq(int irq, void *dev_id)
{
	int status;

	status = gpio_get_value(VIGOR_MDM2AP_SYNC);
	pr_info("%s: VIGOR_MDM2AP_SYNC Status = %d\n",
		 __func__, status);

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (sdc2_status_notify_cb) {
		pr_info("%s: calling sdc2_status_notify_cb\n", __func__);
		sdc2_status_notify_cb(status,
			sdc2_status_notify_cb_devid);
	}
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	if (sdc5_status_notify_cb) {
		pr_info("%s: calling sdc5_status_notify_cb\n", __func__);
		sdc5_status_notify_cb(status,
			sdc5_status_notify_cb_devid);
	}
#endif
	return IRQ_HANDLED;
}

static int msm8x60_multi_sdio_init(void)
{
	int ret, irq_num;

	ret = msm_gpiomux_get(VIGOR_MDM2AP_SYNC);
	if (ret) {
		pr_err("%s:Failed to request GPIO %d, ret=%d\n",
					__func__, VIGOR_MDM2AP_SYNC, ret);
		return ret;
	}

	irq_num = gpio_to_irq(VIGOR_MDM2AP_SYNC);

	ret = request_irq(irq_num,
		msm8x60_multi_sdio_slot_status_irq,
		IRQ_TYPE_EDGE_BOTH,
		"sdio_multidetection", NULL);

	if (ret) {
		pr_err("%s:Failed to request irq, ret=%d\n",
					__func__, ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm8x60_sdcc_slot_status(struct device *dev)
{
	struct platform_device *pdev;
	int status;
	pdev = container_of(dev, struct platform_device, dev);

	status = gpio_request(PM8058_GPIO_PM_TO_SYS(VIGOR_SD_DETECT_PIN)
				, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request PMIC GPIO %d\n", __func__,
				(VIGOR_SD_DETECT_PIN + 1));
	} else {
		status = !(gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(VIGOR_SD_DETECT_PIN)));
		/*pr_info("%s: WP Status for Slot %d = %d\n", __func__,
							pdev->id, status);*/
		gpio_free(PM8058_GPIO_PM_TO_SYS(VIGOR_SD_DETECT_PIN));
	}
	return (unsigned int) status;
}
#endif
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static unsigned int vigor_emmc_slot_type = MMC_TYPE_MMC;
static struct mmc_platform_data msm8x60_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.slot_type		= &vigor_emmc_slot_type,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
	.pclk_src_dfab	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int vigor_sdc2_slot_type = MMC_TYPE_SDIO_SVLTE;
static struct mmc_platform_data msm8x60_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
	.slot_type	= &vigor_sdc2_slot_type,
	.register_status_notify = sdc2_register_status_notify,
	.pclk_src_dfab	= 1,
	.dummy52_required = 1,
	.is_sdio_al_client = 1,
	.trigger_mdm_fatal = trigger_mdm_fatal,
	.get_mdm2ap_status = get_mdm2ap_status,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static unsigned int vigor_sdslot_type = MMC_TYPE_SD;
static struct mmc_platform_data msm8x60_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch  	= msm_sdc3_get_wpswitch,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm8x60_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, VIGOR_SD_DETECT_PIN),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.slot_type		= &vigor_sdslot_type,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
	.pclk_src_dfab	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#if 0
static struct mmc_platform_data msm8x60_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static unsigned int vigor_sdc5_slot_type = MMC_TYPE_SDIO_SVLTE;
static struct mmc_platform_data msm8x60_sdc5_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
	.slot_type	= &vigor_sdc5_slot_type,
	.register_status_notify = sdc5_register_status_notify,
	.pclk_src_dfab	= 1,
	.dummy52_required = 1,
	.is_sdio_al_client = 1,
	.trigger_mdm_fatal = trigger_mdm_fatal,
	.get_mdm2ap_status = get_mdm2ap_status,
};
#endif

static void __init msm8x60_init_mmc(void)
{
	int ret = 0;
#ifdef CONFIG_MSM_SDIO_AL
	config_gpio_table(mdm2ap_gpio_table,
			  ARRAY_SIZE(mdm2ap_gpio_table));
#endif
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	config_gpio_table(sdc1_on_gpio_table, ARRAY_SIZE(sdc1_on_gpio_table));
	/* SDCC1 : eMMC card connected */
	sdcc_vreg_data[0].vdd_data = &sdcc_vdd_reg_data[0];
	sdcc_vreg_data[0].vdd_data->reg_name = "8901_l5";
	sdcc_vreg_data[0].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[0].vdd_data->level = 2850000;
	sdcc_vreg_data[0].vccq_data = &sdcc_vccq_reg_data[0];
	sdcc_vreg_data[0].vccq_data->reg_name = "8901_lvs0";
	sdcc_vreg_data[0].vccq_data->set_voltage_sup = 0;
	msm_add_sdcc(1, &msm8x60_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/*
	 * MDM SDIO client is connected to SDC2 on vigor
	 */
	sdcc_vreg_data[1].vdd_data = &sdcc_vdd_reg_data[1];
	sdcc_vreg_data[1].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[1].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[1].vdd_data->level = 1800000;
	sdcc_vreg_data[1].vccq_data = NULL;
	msm8x60_sdc2_data.msmsdcc_fmax = 24000000;
	msm8x60_sdc2_data.sdiowakeup_irq = gpio_to_irq(144);
	msm_sdcc_setup_gpio(2, 1);
	msm_add_sdcc(2, &msm8x60_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	/* SDCC3 : External card slot connected */
	sdcc_vreg_data[2].vdd_data = &sdcc_vdd_reg_data[2];
	sdcc_vreg_data[2].vdd_data->reg_name = "8058_l14";
	sdcc_vreg_data[2].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[2].vdd_data->level = 2850000;
	sdcc_vreg_data[2].vccq_data = NULL;
	msm_add_sdcc(3, &msm8x60_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	/* SDCC4 : WLAN WCN1314 chip is connected */
	/*sdcc_vreg_data[3].vdd_data = &sdcc_vdd_reg_data[3];
	sdcc_vreg_data[3].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[3].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[3].vdd_data->level = 1800000;
	sdcc_vreg_data[3].vccq_data = NULL;
	msm_add_sdcc(4, &msm8x60_sdc4_data);*/
	ret = vigor_init_mmc();
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC (SDCC4)\n", __func__);
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	/*
	 * MDM SDIO client is connected to SDC5 on vigor
	 */
	sdcc_vreg_data[4].vdd_data = &sdcc_vdd_reg_data[4];
	sdcc_vreg_data[4].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[4].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[4].vdd_data->level = 1800000;
	sdcc_vreg_data[4].vccq_data = NULL;
	msm8x60_sdc5_data.msmsdcc_fmax = 24000000;
	msm8x60_sdc5_data.sdiowakeup_irq = gpio_to_irq(99);
	msm_sdcc_setup_gpio(5, 1);
	msm_add_sdcc(5, &msm8x60_sdc5_data);
#endif
}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL

static struct regulator *reg_8901_l3;		/* HDMI_CEC */

static int hdmi_enable_5v(int on)
{
	static struct regulator *reg_8901_hdmi_mvs;	/* HDMI_5V */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_hdmi_mvs)
		_GET_REGULATOR(reg_8901_hdmi_mvs, "8901_hdmi_mvs");

	if (on) {
		rc = regulator_enable(reg_8901_hdmi_mvs);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8901_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_core_power(int on, int show)
{
	static struct regulator *reg_8058_l16;		/* VDD_HDMI */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8058_l16)
		_GET_REGULATOR(reg_8058_l16, "8058_l16");

	if (on) {
		rc = regulator_set_voltage(reg_8058_l16, 1800000, 1800000);
		if (!rc)
			rc = regulator_enable(reg_8058_l16);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8058_l16", rc);
			return rc;
		}

		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8058_l16);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8058_l16", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_cec_power(int on)
{
	static struct regulator *reg_8901_l3;		/* HDMI_CEC */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_l3)
		_GET_REGULATOR(reg_8901_l3, "8901_l3");

	if (on) {
		rc = regulator_set_voltage(reg_8901_l3, 3300000, 3300000);
		if (!rc)
			rc = regulator_enable(reg_8901_l3);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_l3", rc);
			return rc;
		}

		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8901_l3);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_l3", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}
#undef _GET_REGULATOR

#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define	VIGOR_DC_DC	(1<<0)
static struct regulator *v_lcm;
static struct regulator *v_lcmio;

static void vigor_panel_power(int on)
{
	static int init;
	int ret;
	int rc;

	char *lcm_str = "8058_l12";
	char *lcmio_str = "8901_lvs1";

	struct pm8058_gpio gpio_cfg_LCM_3V = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM_GPIO_VIN_S3,	/* S3 1.8 V */
		.inv_int_pol	= 0,
	};

	// VigorEVM
	if (system_rev == VIGOR_EVM_PCB_ID)
	{
		lcm_str = "8901_l1";
		lcmio_str = "8901_lvs2";
	}

	/* If panel is already on (or off), do nothing. */
	if (!init) {
		if (engineer_id < VIGOR_DC_DC) {
			v_lcm = regulator_get(NULL, lcm_str);
			if (IS_ERR(v_lcm)) {
				pr_err("%s: unable to get %s\n", __func__, lcm_str);
				goto fail;
			}
		} else
			pm8058_gpio_config(VIGOR_LCM_3V, &gpio_cfg_LCM_3V);

		v_lcmio = regulator_get(NULL, lcmio_str);
		if (IS_ERR(v_lcmio)) {
			pr_err("%s#%d: unable to get %s\n", __func__, __LINE__, lcmio_str);
			goto fail;
		}

		if (engineer_id < VIGOR_DC_DC) {
			ret = regulator_set_voltage(v_lcm, 3200000, 3200000);
			if (ret) {
				pr_err("%s: error setting %s voltage\n", __func__, lcm_str);
				goto fail;
			}
		}

		/* LCM Reset */
		rc = gpio_request(GPIO_LCM_RST_N, "LCM_RST_N");
		if (rc) {
			printk(KERN_ERR "%s:LCM gpio %d request"
				"failed\n", __func__,  GPIO_LCM_RST_N);
			return;
		}

		// gpio_direction_output(GPIO_LCM_RST_N, 0);
		init = 1;
	}

	if (engineer_id < VIGOR_DC_DC) {
		if (!v_lcm || IS_ERR(v_lcm)) {
			pr_err("%s: %s is not initialized\n", __func__, lcm_str);
			return;
		}
	}

	if (!v_lcmio || IS_ERR(v_lcmio)) {
		pr_err("%s: %s is not initialized\n", __func__, lcmio_str);
		return;
	}

	if (on) {
		if (engineer_id < VIGOR_DC_DC) {
			if (regulator_enable(v_lcm)) {
				pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcm_str);
				return;
			}
		} else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_LCM_3V), 1);

		mdelay(5);

		if (regulator_enable(v_lcmio)) {
			pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcmio_str);
			return;
		}

		mdelay(10);
		gpio_set_value(GPIO_LCM_RST_N, 1);
		mdelay(1);
		gpio_set_value(GPIO_LCM_RST_N, 0);
		mdelay(1);
		gpio_set_value(GPIO_LCM_RST_N, 1);
		mdelay(20);
	} else {
		mdelay(110);
		gpio_set_value(GPIO_LCM_RST_N, 0);
		mdelay(5);
		if (engineer_id < VIGOR_DC_DC) {
			if (regulator_disable(v_lcm)) {
				pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcm_str);
				return;
			}
		} else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_LCM_3V), 0);
		mdelay(5);
		if (regulator_disable(v_lcmio)) {
			pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcmio_str);
			return;
		}
	}
	return;

fail:
	if (v_lcm)
		regulator_put(v_lcm);
	if (v_lcmio)
		regulator_put(v_lcmio);
}

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_smi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 147460000,
		.ib = 184325000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_ebi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 273408000,
		.ib = 341760000,
	},
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 273408000,
		.ib = 341760000,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 328704000,
		.ib = 410880000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 328704000,
		.ib = 410880000,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 432384000,
		.ib = 540480000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 432384000,
		.ib = 540480000,
	},
};
static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_smi_vectors),
		mdp_sd_smi_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_ebi_vectors),
		mdp_sd_ebi_vectors,
	},

	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};
static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 435456000,
		.ib = 544320000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 435456000,
		.ib = 544320000,
	},
};
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif

static uint32_t lcd_on_gpio[] = {
	GPIO_CFG(VIGOR_GPIO_LCM_ID0,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_GPIO_LCM_ID1,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t lcd_off_gpio[] = {
	GPIO_CFG(VIGOR_GPIO_LCM_ID0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_GPIO_LCM_ID1,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int mipi_panel_power(int on)
{
	int flag_on = !!on;
	static int mipi_power_save_on;

	if (mipi_power_save_on == flag_on)
		return 0;

	mipi_power_save_on = flag_on;

	/* config panel id GPIO per H/W spec */
	if (on)
		config_gpio_table(lcd_on_gpio, ARRAY_SIZE(lcd_on_gpio));
	else
		config_gpio_table(lcd_off_gpio, ARRAY_SIZE(lcd_off_gpio));

	vigor_panel_power(on);

	return 0;
}

static struct mipi_dsi_platform_data mipi_pdata = {
	.vsync_gpio = 28,
	.dsi_power_save   = mipi_panel_power,
};

uint32_t gpio_init_for_power_table_xa[] = {
	GPIO_CFG(BOOT_CONFIG_6, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, 0),
	GPIO_CFG(RMA_DEBUG, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, 0),
};

uint32_t gpio_init_for_power_table[] = {
	GPIO_CFG(BOOT_CONFIG_6, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, 0),
	GPIO_CFG(RMA_DEBUG, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, 0),
};


#ifdef CONFIG_MSM8X60_AUDIO_LTE
static uint32_t auxpcm_gpio_table[] = {
	GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(112, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void msm_auxpcm_init(void)
{
	gpio_tlmm_config(auxpcm_gpio_table[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(auxpcm_gpio_table[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(auxpcm_gpio_table[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(auxpcm_gpio_table[3], GPIO_CFG_ENABLE);
}

#if 1
void msm_snddev_voltage_on(void)
{
}

void msm_snddev_voltage_off(void)
{
}
#endif

#endif /* CONFIG_MSM8X60_AUDIO */

int mdp_core_clk_rate_table[] = {
	85330000,
	128000000,
	200000000,
	200000000,
};
static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 28,
	.mdp_core_clk_rate = 85330000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
};

static void __init msm_fb_add_devices(void)
{
	printk(KERN_INFO "panel ID= 0x%x\n", panel_type);
	msm_fb_register_device("mdp", &mdp_pdata);

	if (panel_type != PANEL_ID_NONE)
		msm_fb_register_device("mipi_dsi", &mipi_pdata);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

#ifdef CONFIG_MSM_RPM
static struct msm_rpm_platform_data msm_rpm_data = {
	.reg_base_addrs = {
		[MSM_RPM_PAGE_STATUS] = MSM_RPM_BASE,
		[MSM_RPM_PAGE_CTRL] = MSM_RPM_BASE + 0x400,
		[MSM_RPM_PAGE_REQ] = MSM_RPM_BASE + 0x600,
		[MSM_RPM_PAGE_ACK] = MSM_RPM_BASE + 0xa00,
		[MSM_RPM_PAGE_STAT] = MSM_RPM_BASE + 0x3E04,
	},

	.irq_ack = RPM_SCSS_CPU0_GP_HIGH_IRQ,
	.irq_err = RPM_SCSS_CPU0_GP_LOW_IRQ,
	.irq_vmpm = RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
};
#endif

static ssize_t vigor_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":76:1370:110:120"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":265:1370:120:120"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":460:1370:100:120"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":647:1370:120:120"
		"\n");
}

static struct kobj_attribute vigor_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &vigor_virtual_keys_show,
};

static struct attribute *vigor_properties_attrs[] = {
	&vigor_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group vigor_properties_attr_group = {
	.attrs = vigor_properties_attrs,
};

void msm_snddev_rx_route_config(void)
{
	pr_debug("%s\n", __func__);
}

void msm_snddev_rx_route_deconfig(void)
{
	pr_debug("%s\n", __func__);
}
#ifdef CONFIG_USB_ANDROID
static void vigor_add_usb_devices(void)
{
	struct usb_mass_storage_platform_data *p;
	printk("%s\n", __func__);
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;

	config_vigor_mhl_gpios();

#if defined(CONFIG_USB_OTG)
	msm_otg_pdata.idgnd_gpio = VIGOR_GPIO_USB_ID;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_usb_host_pdata;

	platform_device_register(&msm_device_otg);
#endif
       /* add cdrom support in normal mode */
	if (board_mfg_mode() == 0) {
		p = usb_mass_storage_device.dev.platform_data;
		p->nluns = 3;
		p->cdrom_lun = 0x4;
       }

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);

#if defined(CONFIG_USB_F_SERIAL_SDIO) || defined(CONFIG_USB_F_SERIAL_SMD)
	platform_device_register(&usb_gadget_fserial_device);
#endif
}
#endif
int __initdata irq_ignore_tbl[] =
{
	MSM_GPIO_TO_INT(133),
	MSM_GPIO_TO_INT(134),
};
unsigned __initdata irq_num_ignore_tbl = ARRAY_SIZE(irq_ignore_tbl);

int __initdata clk_ignore_tbl[] =
{
	L_GSBI11_UART_CLK,
	L_SDC4_CLK,
	L_SDC4_P_CLK,
};
unsigned __initdata clk_num_ignore_tbl = ARRAY_SIZE(clk_ignore_tbl);

#define PM8058_LPM_SET(id)	(1 << RPM_VREG_ID_##id)
#define PM8901_LPM_SET(id)	(1 << (RPM_VREG_ID_##id - RPM_VREG_ID_PM8901_L0))

uint32_t __initdata regulator_lpm_set[] =
{
	PM8058_LPM_SET(PM8058_L0) | PM8058_LPM_SET(PM8058_L1) | PM8058_LPM_SET(PM8058_L2) |
	PM8058_LPM_SET(PM8058_L3) | PM8058_LPM_SET(PM8058_L4) |
	PM8058_LPM_SET(PM8058_L5) | PM8058_LPM_SET(PM8058_L6) | PM8058_LPM_SET(PM8058_L7) |
	PM8058_LPM_SET(PM8058_L8) | PM8058_LPM_SET(PM8058_L9) | PM8058_LPM_SET(PM8058_L10) |
	PM8058_LPM_SET(PM8058_L11) | PM8058_LPM_SET(PM8058_L13) |
	PM8058_LPM_SET(PM8058_L15) | PM8058_LPM_SET(PM8058_L16) | PM8058_LPM_SET(PM8058_L17) |
	PM8058_LPM_SET(PM8058_L18) | PM8058_LPM_SET(PM8058_L19) | PM8058_LPM_SET(PM8058_L20) |
	PM8058_LPM_SET(PM8058_L21) | PM8058_LPM_SET(PM8058_L22) | PM8058_LPM_SET(PM8058_L23) |
	PM8058_LPM_SET(PM8058_L24) | PM8058_LPM_SET(PM8058_L25),
	PM8901_LPM_SET(PM8901_L0) | PM8901_LPM_SET(PM8901_L1) | PM8901_LPM_SET(PM8901_L2) | PM8901_LPM_SET(PM8901_L3) | PM8901_LPM_SET(PM8901_L4) | PM8901_LPM_SET(PM8901_L6),
};
static void __init vigor_init(void)
{
	int rc = 0;
	struct kobject *properties_kobj;
	struct regulator *margin_power;
	msm_mpm_defer_ignore_list = 1;
	/*
	 * Initialize RPM first as other drivers and devices may need
	 * it for their initialization.
	 */
#ifdef CONFIG_MSM_RPM
	BUG_ON(msm_rpm_init(&msm_rpm_data));
	msm_rpm_lpm_init(regulator_lpm_set, ARRAY_SIZE(regulator_lpm_set));
#endif
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
	msm8x60_check_2d_hardware();

	/* initialize SPM before acpuclock as the latter calls into SPM
	 * driver to set ACPU voltages.
	 */

	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1) {
		/* Change SPM handling of core 1 for PMM 8160. */
		msm_spm_data[1].reg_init_values[MSM_SPM_REG_SAW_CFG] &= ~0x0F00UL;
		msm_spm_data[1].reg_init_values[MSM_SPM_REG_SAW_CFG] |= 0x0100UL;
		msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	} else{
		/* Change SPM handling of core 1 for PMM 8160. */
		msm_spm_data_v1[1].reg_init_values[MSM_SPM_REG_SAW_CFG] &= ~0x0F00UL;
		msm_spm_data_v1[1].reg_init_values[MSM_SPM_REG_SAW_CFG] |= 0x0100UL;
		msm_spm_init(msm_spm_data_v1, ARRAY_SIZE(msm_spm_data_v1));
	}

	/*
	 * Disable regulator info printing so that regulator registration
	 * messages do not enter the kmsg log.
	 */
	regulator_suppress_info_printing();

	/* Initialize regulators needed for clock_init. */
	platform_add_devices(early_regulators, ARRAY_SIZE(early_regulators));

	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);

	/* Buses need to be initialized before early-device registration
	 * to get the platform data for fabrics.
	 */
	msm8x60_init_buses();

#ifdef CONFIG_BT
	bt_export_bd_address();
#endif

	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	/* CPU frequency control is not supported on simulated targets. */
	msm_acpu_clock_init(&msm8x60_acpu_clock_data);

#ifdef CONFIG_PERFLOCK
        perflock_init(&vigor_perflock_data);
#endif
#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	set_two_phase_freq(1134000);
#endif

	msm8x60_init_tlmm();
	msm8x60_init_gpiomux(htc_vigor_htc_gpiomux_cfgs);
	msm8x60_init_mmc();

#ifdef CONFIG_MSM_DSPS
	msm8x60_init_dsps();
#endif
	platform_add_devices(msm_footswitch_devices,
					     msm_num_footswitch_devices);

	/* 2nd CAM setting */
	msm8x60_init_camera();

	/* disable uart under OS normal for MHL audio jack */
	if ((board_build_flag() != MFG_BUILD) && !(get_kernel_flag() & BIT1))
		vigor_disable_serial_debug_gpios();

	platform_add_devices(surf_devices, ARRAY_SIZE(surf_devices));

	if (system_rev == VIGOR_EVM_PCB_ID)	// VigorEVM
		platform_device_register(&msm_device_uart_dm12);
	else
		platform_device_register(&msm_device_uart_dm11);

#ifdef CONFIG_I2C_QUP
	if (system_rev == VIGOR_EVM_PCB_ID)	// VigorEVM
		platform_device_register(&msm_gsbi10_qup_i2c_device);
	else
		platform_device_register(&msm_gsbi12_qup_i2c_device);
#endif

#ifdef CONFIG_USB_EHCI_MSM
		msm_add_host(0, &msm_usb_host_pdata);
#endif

	if (system_rev == VIGOR_EVM_PCB_ID)	// VigorEVM
		platform_add_devices(charm_devices_evm, ARRAY_SIZE(charm_devices_evm));
	else
		platform_add_devices(charm_devices, ARRAY_SIZE(charm_devices));

#ifdef CONFIG_FB_MSM_HDMI_MHL_SII9234
	mhl_sii9234_device_data.ci2ca = 0;
#endif

	msm_fb_add_devices();

	register_i2c_devices();
#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
	if (system_rev == 0)
		platform_add_devices(flashlight_devices, ARRAY_SIZE(flashlight_devices));
	else
		i2c_register_board_info(MSM_GSBI7_QUP_I2C_BUS_ID, vigor_flashlight_XB, ARRAY_SIZE(vigor_flashlight_XB));
#endif
#ifdef CONFIG_USB_ANDROID
	/*usb driver won't be loaded in MFG 58 station*/
	if (board_mfg_mode() != 6)
		vigor_add_usb_devices();
#endif
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_cpuidle_set_states(msm_cstates, ARRAY_SIZE(msm_cstates),
				msm_pm_data);

#ifdef CONFIG_MSM8X60_AUDIO_LTE
	msm_auxpcm_init();
	msm_snddev_init();

	vigor_audio_init();
#endif

#if 0
	spi_register_board_info(msm_spi_panel_board_info, ARRAY_SIZE(msm_spi_panel_board_info));
	gpio_tlmm_config(msm_spi_panel_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_panel_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_panel_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_panel_gpio[3], GPIO_CFG_ENABLE);
#endif

	vigor_init_keypad();

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
				&vigor_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	vigor_wifi_init();
	msm8x60_multi_sdio_init();

	sysinfo_proc_init();
	mdm_loaded_info();

	msm_mpm_set_irq_ignore_list(irq_ignore_tbl, irq_num_ignore_tbl);
	msm_clk_soc_set_ignore_list(clk_ignore_tbl, clk_num_ignore_tbl);

	/* change S4B to 1.25v, L22A to 1.2v for DDR stability issue */
	margin_power = regulator_get(NULL, "8901_s4");
	regulator_set_voltage(margin_power, 1250000, 1250000);
	regulator_enable(margin_power);
	regulator_put(margin_power);
	margin_power = regulator_get(NULL, "8058_l22");
	regulator_set_voltage(margin_power, 1200000, 1200000);
	regulator_enable(margin_power);
	regulator_put(margin_power);

	if (system_rev >= 1) /*above XB*/
		config_gpio_table(gpio_init_for_power_table, ARRAY_SIZE(gpio_init_for_power_table));
	else
		config_gpio_table(gpio_init_for_power_table_xa, ARRAY_SIZE(gpio_init_for_power_table_xa));

	if (get_kernel_flag() & BIT25) {
		htc_monitor_init();
		htc_PM_monitor_init();
	}
}

/* PHY_BASE_ADDR1 should be 8 MB alignment */
/* 0x48000000~0x48700000 is reserved for Vigor 8K AMSS */
#define PHY_BASE_ADDR1  0x48800000
/* 0x40400000~0x42A00000 is 38MB for SF/AUDIO/FB PMEM */
/* 0x48800000~0x7CD00000 is 837MB for APP */
/* 0x7DC00000~0x80000000 is 51MB for ADSP PMEM */
#define SIZE_ADDR1      0x34500000

static void __init vigor_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	engineerid = parse_tag_engineerid(tags);
	pr_info("[%s]\n, engineerid = 0x%x", __func__, engineerid);

	mi->nr_banks = 1;
	mi->bank[0].start = PHY_BASE_ADDR1;
	mi->bank[0].node = PHYS_TO_NID(PHY_BASE_ADDR1);
	mi->bank[0].size = SIZE_ADDR1;

	pr_info("vigor_fixup,parse_tag_cam(tags)=%d",parse_tag_cam(tags));
	cam_id_kvalue = parse_tag_cam(tags);
}

MACHINE_START(VIGOR, "vigor")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.fixup = vigor_fixup,
	.map_io = vigor_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = vigor_init,
	.timer = &msm_timer,
MACHINE_END

