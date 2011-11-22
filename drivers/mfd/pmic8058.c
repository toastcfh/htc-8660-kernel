/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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
/*
 * Qualcomm PMIC8058 driver
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/mfd/pmic8058.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#ifdef CONFIG_MSM8X60_SSBI
#include <mach/msm_ssbi.h>
#endif

/* PMIC8058 Revision */
#define SSBI_REG_REV			0x002  /* PMIC4 revision */

/* PMIC8058 IRQ */
#define	SSBI_REG_ADDR_IRQ_BASE		0x1BB

#define	SSBI_REG_ADDR_IRQ_ROOT		(SSBI_REG_ADDR_IRQ_BASE + 0)
#define	SSBI_REG_ADDR_IRQ_M_STATUS1	(SSBI_REG_ADDR_IRQ_BASE + 1)
#define	SSBI_REG_ADDR_IRQ_M_STATUS2	(SSBI_REG_ADDR_IRQ_BASE + 2)
#define	SSBI_REG_ADDR_IRQ_M_STATUS3	(SSBI_REG_ADDR_IRQ_BASE + 3)
#define	SSBI_REG_ADDR_IRQ_M_STATUS4	(SSBI_REG_ADDR_IRQ_BASE + 4)
#define	SSBI_REG_ADDR_IRQ_BLK_SEL	(SSBI_REG_ADDR_IRQ_BASE + 5)
#define	SSBI_REG_ADDR_IRQ_IT_STATUS	(SSBI_REG_ADDR_IRQ_BASE + 6)
#define	SSBI_REG_ADDR_IRQ_CONFIG	(SSBI_REG_ADDR_IRQ_BASE + 7)
#define	SSBI_REG_ADDR_IRQ_RT_STATUS	(SSBI_REG_ADDR_IRQ_BASE + 8)

#define	PM8058_IRQF_LVL_SEL		0x01	/* level select */
#define	PM8058_IRQF_MASK_FE		0x02	/* mask falling edge */
#define	PM8058_IRQF_MASK_RE		0x04	/* mask rising edge */
#define	PM8058_IRQF_CLR			0x08	/* clear interrupt */
#define	PM8058_IRQF_BITS_MASK		0x70
#define	PM8058_IRQF_BITS_SHIFT		4
#define	PM8058_IRQF_WRITE		0x80

#define	PM8058_IRQF_MASK_ALL		(PM8058_IRQF_MASK_FE | \
					PM8058_IRQF_MASK_RE)
#define PM8058_IRQF_W_C_M		(PM8058_IRQF_WRITE |	\
					PM8058_IRQF_CLR |	\
					PM8058_IRQF_MASK_ALL)

/* MISC register */
#define	SSBI_REG_ADDR_MISC		0x1CC

/* PON CNTL 1 register */
#define SSBI_REG_ADDR_PON_CNTL_1	0x01C

#define PM8058_PON_PUP_MASK		0xF0

#define PM8058_PON_WD_EN_MASK		0x08
#define PM8058_PON_WD_EN_RESET		0x08
#define PM8058_PON_WD_EN_PWR_OFF	0x00

/* Regulator L22 control register */
#define SSBI_REG_ADDR_L22_CTRL		0x121

#ifdef CONFIG_MACH_VERDI_LTE //[Puccini] QCT patch: SMPS actvie pull-down
/* Regulator master enable addresses */
#define SSBI_REG_ADDR_VREG_EN_MSM	0x018
#define SSBI_REG_ADDR_VREG_EN_GRP_5_4	0x1C8

/* Regulator control registers for shutdown/reset */
#define SSBI_REG_ADDR_S0_CTRL		0x004
#define SSBI_REG_ADDR_S1_CTRL		0x005
#define SSBI_REG_ADDR_S3_CTRL		0x111
#define SSBI_REG_ADDR_L21_CTRL		0x120
//#define SSBI_REG_ADDR_L22_CTRL		0x121 // defined above

#define REGULATOR_ENABLE_MASK		0x80
#define REGULATOR_ENABLE		0x80
#define REGULATOR_DISABLE		0x00
#define REGULATOR_PULL_DOWN_MASK	0x40
#define REGULATOR_PULL_DOWN_EN		0x40
#define REGULATOR_PULL_DOWN_DIS		0x00

/* Buck CTRL register */
#define SMPS_LEGACY_VREF_SEL		0x20
#define SMPS_LEGACY_VPROG_MASK		0x1F
#define SMPS_ADVANCED_BAND_MASK		0xC0
#define SMPS_ADVANCED_BAND_SHIFT	6
#define SMPS_ADVANCED_VPROG_MASK	0x3F

/* Buck TEST2 registers for shutdown/reset */
#define SSBI_REG_ADDR_S0_TEST2		0x084
#define SSBI_REG_ADDR_S1_TEST2		0x085
#define SSBI_REG_ADDR_S3_TEST2		0x11A

#define REGULATOR_BANK_WRITE		0x80
#define REGULATOR_BANK_MASK		0x70
#define REGULATOR_BANK_SHIFT		4
#define REGULATOR_BANK_SEL(n)		((n) << REGULATOR_BANK_SHIFT)

/* Buck TEST2 register bank 1 */
#define SMPS_LEGACY_VLOW_SEL		0x01

/* Buck TEST2 register bank 7 */
#define SMPS_ADVANCED_MODE_MASK		0x02
#define SMPS_ADVANCED_MODE		0x02
#define SMPS_LEGACY_MODE		0x00
#endif //CONFIG_MACH_VERDI_LTE

/* SLEEP CNTL register */
#define SSBI_REG_ADDR_SLEEP_CNTL	0x02B

#define PM8058_SLEEP_SMPL_EN_MASK	0x04
#define PM8058_SLEEP_SMPL_EN_RESET	0x04
#define PM8058_SLEEP_SMPL_EN_PWR_OFF	0x00

#define PM8058_SLEEP_SMPL_SEL_MASK	0x03
#define PM8058_SLEEP_SMPL_SEL_MIN	0
#define PM8058_SLEEP_SMPL_SEL_MAX	3

#define	MAX_PM_IRQ		256
#define	MAX_PM_BLOCKS		(MAX_PM_IRQ / 8 + 1)
#define	MAX_PM_MASTERS		(MAX_PM_BLOCKS / 8 + 1)

struct pm8058_chip {
	struct pm8058_platform_data	pdata;
#ifdef CONFIG_MSM8X60_SSBI
	struct device *dev;
	int id;
#else
	struct i2c_client		*dev;
#endif
	u8	irqs_allowed[MAX_PM_BLOCKS];
	u8	blocks_allowed[MAX_PM_MASTERS];
	u8	masters_allowed;
	int	pm_max_irq;
	int	pm_max_blocks;
	int	pm_max_masters;

	u8	config[MAX_PM_IRQ];
	u8	bus_unlock_config[MAX_PM_IRQ];
	u8	wake_enable[MAX_PM_IRQ];
	u16	count_wakeable;

	u8	revision;

#ifdef CONFIG_MSM8X60_SSBI
	spinlock_t  pm_lock;
#else
	struct mutex	pm_lock;
#endif
};

#if defined(CONFIG_DEBUG_FS)
struct pm8058_dbg_device {
	struct mutex		dbg_mutex;
	struct pm8058_chip	*pm_chip;
	struct dentry		*dent;
	int			addr;
};

static struct pm8058_dbg_device *pmic_dbg_device;
#endif

static struct pm8058_chip *pmic_chip;

/* Helper Functions */
DEFINE_RATELIMIT_STATE(pm8058_msg_ratelimit, 60 * HZ, 10);

static inline int pm8058_can_print(void)
{
	return __ratelimit(&pm8058_msg_ratelimit);
}

#ifdef CONFIG_MSM8X60_SSBI
#define ssbi_write(client, addr, buf, len) \
	msm_ssbi_write(pmic_chip->id, addr, buf, len)
#define ssbi_read(client, addr, buf, len) \
	msm_ssbi_read(pmic_chip->id, addr, buf, len)
#else /*CONFIG_MSM8X60_SSBI*/
static inline int
ssbi_write(struct i2c_client *client, u16 addr, const u8 *buf, size_t len)
{
	int	rc;
	struct	i2c_msg msg = {
		.addr           = addr,
		.flags          = 0x0,
		.buf            = (u8 *)buf,
		.len            = len,
	};

	rc = i2c_transfer(client->adapter, &msg, 1);

	return (rc == 1) ? 0 : rc;
}

static inline int
ssbi_read(struct i2c_client *client, u16 addr, u8 *buf, size_t len)
{
	int	rc;
	struct	i2c_msg msg = {
		.addr           = addr,
		.flags          = I2C_M_RD,
		.buf            = buf,
		.len            = len,
	};

	rc = i2c_transfer(client->adapter, &msg, 1);
	return (rc == 1) ? 0 : rc;
}
#endif/*CONFIG_MSM8X60_SSBI*/
static int pm8058_masked_write(u16 addr, u8 val, u8 mask)
{
	int rc;
	u8 reg;

#ifndef CONFIG_MACH_VERDI_LTE //[Puccini] QCT patch: SMPS actvie pull-down
#ifdef CONFIG_MSM8X60_SSBI
	unsigned long irqsave;
#endif
#endif // !CONFIG_MACH_VERDI_LTE

	if (pmic_chip == NULL)
		return -ENODEV;

#ifndef CONFIG_MACH_VERDI_LTE //[Puccini] QCT patch: SMPS actvie pull-down
#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&pmic_chip->pm_lock, irqsave);
#else
	mutex_lock(&pmic_chip->pm_lock);
#endif
#endif // !CONFIG_MACH_VERDI_LTE

	rc = ssbi_read(pmic_chip->dev, addr, &reg, 1);
	if (rc) {
		pr_err("%s: ssbi_read(0x%03X) failed: rc=%d\n", __func__, addr,
			rc);
		goto done;
	}

	reg &= ~mask;
	reg |= val & mask;

	rc = ssbi_write(pmic_chip->dev, addr, &reg, 1);
	if (rc)
		pr_err("%s: ssbi_write(0x%03X)=0x%02X failed: rc=%d\n",
			__func__, addr, reg, rc);
done:
#ifndef CONFIG_MACH_VERDI_LTE //[Puccini] QCT patch: SMPS actvie pull-down
#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&pmic_chip->pm_lock, irqsave);
#else
	mutex_unlock(&pmic_chip->pm_lock);
#endif
#endif // !CONFIG_MACH_VERDI_LTE

	return rc;
}
/* External APIs */
int pm8058_rev(struct pm8058_chip *chip)
{
	if (chip == NULL)
		return -EINVAL;

	return chip->revision;
}
EXPORT_SYMBOL(pm8058_rev);

int pm8058_irq_get_rt_status(struct pm8058_chip *chip, int irq)
{
	int     rc;
	u8      block, bits, bit;
#ifdef CONFIG_MSM8X60_SSBI
	unsigned long irqsave;
#endif

	if (chip == NULL || irq < chip->pdata.irq_base ||
			irq >= chip->pdata.irq_base + MAX_PM_IRQ)
		return -EINVAL;

	irq -= chip->pdata.irq_base;

	block = irq / 8;
	bit = irq % 8;

#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&chip->pm_lock, irqsave);
#else
	mutex_lock(&chip->pm_lock);
#endif

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_BLK_SEL, &block, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(): rc=%d (Select Block)\n",
				__func__, rc);
		goto bail_out;
	}

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_RT_STATUS, &bits, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read RT Status)\n",
				__func__, rc);
		goto bail_out;
	}

	rc = (bits & (1 << bit)) ? 1 : 0;

bail_out:
#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);
#else
	mutex_unlock(&chip->pm_lock);
#endif

	return rc;
}
EXPORT_SYMBOL(pm8058_irq_get_rt_status);

#ifdef CONFIG_MSM8X60_SSBI
int pm8058_read(struct pm8058_chip *chip, u16 addr, u8 *values,
		unsigned int len)
{
	int rc;
	unsigned long irqsave;
	if (chip == NULL)
		return -EINVAL;
	spin_lock_irqsave(&chip->pm_lock, irqsave);

	rc = ssbi_read(chip->dev, addr, values, len);

	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	return rc;
}
EXPORT_SYMBOL(pm8058_read);

int pm8058_write(struct pm8058_chip *chip, u16 addr, u8 *values,
		 unsigned int len)
{
	int rc;
	unsigned long irqsave;
	if (chip == NULL)
		return -EINVAL;
	spin_lock_irqsave(&chip->pm_lock, irqsave);

	rc = ssbi_write(chip->dev, addr, values, len);

	spin_unlock_irqrestore(&chip->pm_lock, irqsave);

	return rc;
}
EXPORT_SYMBOL(pm8058_write);
#else
int pm8058_read(struct pm8058_chip *chip, u16 addr, u8 *values,
		unsigned int len)
{
	if (chip == NULL)
		return -EINVAL;

	return ssbi_read(chip->dev, addr, values, len);
}
EXPORT_SYMBOL(pm8058_read);

int pm8058_write(struct pm8058_chip *chip, u16 addr, u8 *values,
		 unsigned int len)
{
	if (chip == NULL)
		return -EINVAL;

	return ssbi_write(chip->dev, addr, values, len);
}
EXPORT_SYMBOL(pm8058_write);
#endif /*CONFIG_MSM8X60_SSBI */
int pm8058_misc_control(struct pm8058_chip *chip, int mask, int flag)
{
	int		rc;
	u8		misc;
#ifdef CONFIG_MSM8X60_SSBI
	unsigned long irqsave;
#endif

	if (chip == NULL)
		chip = pmic_chip;	/* for calls from non child */
	if (chip == NULL)
		return -ENODEV;

#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&chip->pm_lock, irqsave);
#else
	mutex_lock(&chip->pm_lock);
#endif

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_MISC, &misc, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(0x%x): rc=%d\n",
		       __func__, SSBI_REG_ADDR_MISC, rc);
		goto get_out;
	}

	misc &= ~mask;
	misc |= flag;

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_MISC, &misc, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(0x%x)=0x%x: rc=%d\n",
		       __func__, SSBI_REG_ADDR_MISC, misc, rc);
		goto get_out;
	}

get_out:
#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);
#else
	mutex_unlock(&chip->pm_lock);
#endif

	return rc;
}
EXPORT_SYMBOL(pm8058_misc_control);

/**
 * pm8058_smpl_control - enables/disables SMPL detection
 * @enable: 0 = shutdown PMIC on power loss, 1 = reset PMIC on power loss
 *
 * This function enables or disables the Sudden Momentary Power Loss detection
 * module.  If SMPL detection is enabled, then when a sufficiently long power
 * loss event occurs, the PMIC will automatically reset itself.  If SMPL
 * detection is disabled, then the PMIC will shutdown when power loss occurs.
 *
 * RETURNS: an appropriate -ERRNO error value on error, or zero for success.
 */
int pm8058_smpl_control(int enable)
{
	return pm8058_masked_write(SSBI_REG_ADDR_SLEEP_CNTL,
				   (enable ? PM8058_SLEEP_SMPL_EN_RESET
					   : PM8058_SLEEP_SMPL_EN_PWR_OFF),
				   PM8058_SLEEP_SMPL_EN_MASK);
}
EXPORT_SYMBOL(pm8058_smpl_control);

/**
* pm8058_smpl_set_delay - sets the SMPL detection time delay
* @delay: enum value corresponding to delay time
*
* This function sets the time delay of the SMPL detection module.  If power
* is reapplied within this interval, then the PMIC reset automatically.  The
* SMPL detection module must be enabled for this delay time to take effect.
*
* RETURNS: an appropriate -ERRNO error value on error, or zero for success.
*/
int pm8058_smpl_set_delay(enum pm8058_smpl_delay delay)
{
	if (delay < PM8058_SLEEP_SMPL_SEL_MIN
	    || delay > PM8058_SLEEP_SMPL_SEL_MAX) {
		pr_err("%s: invalid delay specified: %d\n", __func__, delay);
		return -EINVAL;
	}

	return pm8058_masked_write(SSBI_REG_ADDR_SLEEP_CNTL, delay,
				   PM8058_SLEEP_SMPL_SEL_MASK);
}
EXPORT_SYMBOL(pm8058_smpl_set_delay);

/**
* pm8058_watchdog_reset_control - enables/disables watchdog reset detection
* @enable: 0 = shutdown when PS_HOLD goes low, 1 = reset when PS_HOLD goes low
*
* This function enables or disables the PMIC watchdog reset detection feature.
* If watchdog reset detection is enabled, then the PMIC will reset itself
* when PS_HOLD goes low.  If it is not enabled, then the PMIC will shutdown
* when PS_HOLD goes low.
*
* RETURNS: an appropriate -ERRNO error value on error, or zero for success.
*/
int pm8058_watchdog_reset_control(int enable)
{
	return pm8058_masked_write(SSBI_REG_ADDR_PON_CNTL_1,
				   (enable ? PM8058_PON_WD_EN_RESET
					   : PM8058_PON_WD_EN_PWR_OFF),
				   PM8058_PON_WD_EN_MASK);
}
EXPORT_SYMBOL(pm8058_watchdog_reset_control);

#ifdef CONFIG_MACH_VERDI_LTE //[Puccini] QCT patch: SMPS actvie pull-down
/*
 * Set an SMPS regulator to be disabled in its CTRL register, but enabled
 * in the master enable register.  Also set it's pull down enable bit.
 * Take care to make sure that the output voltage doesn't change if switching
 * from advanced mode to legacy mode.
 */
static int disable_smps_locally_set_pull_down(u16 ctrl_addr, u16 test2_addr,
		u16 master_enable_addr, u8 master_enable_bit)
{
	int rc = 0;
	u8 vref_sel, vlow_sel, band, vprog, bank, reg;

	if (pmic_chip == NULL)
		return -ENODEV;

	bank = REGULATOR_BANK_SEL(7);
	rc = ssbi_write(pmic_chip->dev, test2_addr, &bank, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(0x%03X): rc=%d\n", __func__,
			test2_addr, rc);
		goto done;
	}

	rc = ssbi_read(pmic_chip->dev, test2_addr, &reg, 1);
	if (rc) {
		pr_err("%s: FAIL pm8058_read(0x%03X): rc=%d\n",
		       __func__, test2_addr, rc);
		goto done;
	}

	/* Check if in advanced mode. */
	if ((reg & SMPS_ADVANCED_MODE_MASK) == SMPS_ADVANCED_MODE) {
		/* Determine current output voltage. */
		rc = ssbi_read(pmic_chip->dev, ctrl_addr, &reg, 1);
		if (rc) {
			pr_err("%s: FAIL pm8058_read(0x%03X): rc=%d\n",
			       __func__, ctrl_addr, rc);
			goto done;
		}

		band = (reg & SMPS_ADVANCED_BAND_MASK)
			>> SMPS_ADVANCED_BAND_SHIFT;
		switch (band) {
		case 3:
			vref_sel = 0;
			vlow_sel = 0;
			break;
		case 2:
			vref_sel = SMPS_LEGACY_VREF_SEL;
			vlow_sel = 0;
			break;
		case 1:
			vref_sel = SMPS_LEGACY_VREF_SEL;
			vlow_sel = SMPS_LEGACY_VLOW_SEL;
			break;
		default:
			pr_err("%s: regulator already disabled\n", __func__);
			return -EPERM;
		}
		vprog = (reg & SMPS_ADVANCED_VPROG_MASK);
		/* Round up if fine step is in use. */
		vprog = (vprog + 1) >> 1;
		if (vprog > SMPS_LEGACY_VPROG_MASK)
			vprog = SMPS_LEGACY_VPROG_MASK;

		/* Set VLOW_SEL bit. */
		bank = REGULATOR_BANK_SEL(1);
		rc = ssbi_write(pmic_chip->dev, test2_addr, &bank, 1);
		if (rc) {
			pr_err("%s: FAIL ssbi_write(0x%03X): rc=%d\n",
			       __func__, test2_addr, rc);
			goto done;
		}
		rc = pm8058_masked_write(test2_addr,
			REGULATOR_BANK_WRITE | REGULATOR_BANK_SEL(1)
				| vlow_sel,
			REGULATOR_BANK_WRITE | REGULATOR_BANK_MASK
				| SMPS_LEGACY_VLOW_SEL);
		if (rc)
			goto done;

		/* Switch to legacy mode */
		bank = REGULATOR_BANK_SEL(7);
		rc = ssbi_write(pmic_chip->dev, test2_addr, &bank, 1);
		if (rc) {
			pr_err("%s: FAIL ssbi_write(0x%03X): rc=%d\n", __func__,
				test2_addr, rc);
			goto done;
		}
		rc = pm8058_masked_write(test2_addr,
				REGULATOR_BANK_WRITE | REGULATOR_BANK_SEL(7)
					| SMPS_LEGACY_MODE,
				REGULATOR_BANK_WRITE | REGULATOR_BANK_MASK
					| SMPS_ADVANCED_MODE_MASK);
		if (rc)
			goto done;

		/* Enable locally, enable pull down, keep voltage the same. */
		rc = pm8058_masked_write(ctrl_addr,
			REGULATOR_ENABLE | REGULATOR_PULL_DOWN_EN
				| vref_sel | vprog,
			REGULATOR_ENABLE_MASK | REGULATOR_PULL_DOWN_MASK
			       | SMPS_LEGACY_VREF_SEL | SMPS_LEGACY_VPROG_MASK);
		if (rc)
			goto done;
	}

	/* Enable in master control register. */
	rc = pm8058_masked_write(master_enable_addr, master_enable_bit,
				 master_enable_bit);
	if (rc)
		goto done;

	/* Disable locally and enable pull down. */
	rc = pm8058_masked_write(ctrl_addr,
		REGULATOR_DISABLE | REGULATOR_PULL_DOWN_EN,
		REGULATOR_ENABLE_MASK | REGULATOR_PULL_DOWN_MASK);

done:
	return rc;
}
#endif // CONFIG_MACH_VERDI_LTE

int pm8058_reset_pwr_off(int reset)
{
	int		rc;
	u8		pon;
	u8		ctrl;
	u8		smpl;

#ifndef CONFIG_MACH_VERDI_LTE    //[Puccini] QCT patch: SMPS actvie pull-down
#ifdef CONFIG_MSM8X60_SSBI
	unsigned long irqsave;
#endif
#endif // !CONFIG_MACH_VERDI_LTE

	if (pmic_chip == NULL)
		return -ENODEV;

#ifndef CONFIG_MACH_VERDI_LTE    //[Puccini] QCT patch: SMPS actvie pull-down
#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&pmic_chip->pm_lock, irqsave);
#else
	mutex_lock(&pmic_chip->pm_lock);
#endif
#endif // !CONFIG_MACH_VERDI_LTE 

#ifdef CONFIG_MACH_VERDI_LTE    //[Puccini] QCT patch: SMPS actvie pull-down
	/* Disable SMPSes 0,1,3 locally and set pull down enable bits. */
	disable_smps_locally_set_pull_down(SSBI_REG_ADDR_S0_CTRL,
		SSBI_REG_ADDR_S0_TEST2, SSBI_REG_ADDR_VREG_EN_MSM, BIT(7));
	disable_smps_locally_set_pull_down(SSBI_REG_ADDR_S1_CTRL,
		SSBI_REG_ADDR_S1_TEST2, SSBI_REG_ADDR_VREG_EN_MSM, BIT(6));
	disable_smps_locally_set_pull_down(SSBI_REG_ADDR_S3_CTRL,
		SSBI_REG_ADDR_S3_TEST2, SSBI_REG_ADDR_VREG_EN_GRP_5_4,
		BIT(7) | BIT(4));

	/* Enable LDO21 in master control registers. */
	rc = pm8058_masked_write(SSBI_REG_ADDR_VREG_EN_GRP_5_4, BIT(1), BIT(1));
	if (rc)
		goto get_out4;

	/* Disable LDO21 regulator and set pull down */
	pm8058_masked_write(SSBI_REG_ADDR_L21_CTRL,
		REGULATOR_DISABLE | REGULATOR_PULL_DOWN_EN,
		REGULATOR_ENABLE_MASK | REGULATOR_PULL_DOWN_MASK);

get_out4:
#endif // CONFIG_MACH_VERDI_LTE

	/* Set regulator L22 to 1.225V in high power mode. */
	rc = ssbi_read(pmic_chip->dev, SSBI_REG_ADDR_L22_CTRL, &ctrl, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(0x%x): rc=%d\n", __func__,
			SSBI_REG_ADDR_L22_CTRL, rc);
		goto get_out3;
	}
	/* Leave pull-down state intact. */
	ctrl &= 0x40;
	ctrl |= 0x93;
	rc = ssbi_write(pmic_chip->dev, SSBI_REG_ADDR_L22_CTRL, &ctrl, 1);
	if (rc)
		pr_err("%s: FAIL ssbi_write(0x%x)=0x%x: rc=%d\n", __func__,
			SSBI_REG_ADDR_L22_CTRL, ctrl, rc);

get_out3:
	if (!reset) {
		/* Only modify the SLEEP_CNTL reg if shutdown is desired. */
		rc = ssbi_read(pmic_chip->dev, SSBI_REG_ADDR_SLEEP_CNTL,
			       &smpl, 1);
		if (rc) {
			pr_err("%s: FAIL ssbi_read(0x%x): rc=%d\n",
			       __func__, SSBI_REG_ADDR_SLEEP_CNTL, rc);
			goto get_out2;
		}

		smpl &= ~PM8058_SLEEP_SMPL_EN_MASK;
		smpl |= PM8058_SLEEP_SMPL_EN_PWR_OFF;

		rc = ssbi_write(pmic_chip->dev, SSBI_REG_ADDR_SLEEP_CNTL,
				&smpl, 1);
		if (rc)
			pr_err("%s: FAIL ssbi_write(0x%x)=0x%x: rc=%d\n",
			       __func__, SSBI_REG_ADDR_SLEEP_CNTL, smpl, rc);
	}

get_out2:
	rc = ssbi_read(pmic_chip->dev, SSBI_REG_ADDR_PON_CNTL_1, &pon, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(0x%x): rc=%d\n",
		       __func__, SSBI_REG_ADDR_PON_CNTL_1, rc);
		goto get_out;
	}

	pon &= ~PM8058_PON_WD_EN_MASK;
	pon |= reset ? PM8058_PON_WD_EN_RESET : PM8058_PON_WD_EN_PWR_OFF;

	/* Enable all pullups */
	pon |= PM8058_PON_PUP_MASK;

	rc = ssbi_write(pmic_chip->dev, SSBI_REG_ADDR_PON_CNTL_1, &pon, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(0x%x)=0x%x: rc=%d\n",
		       __func__, SSBI_REG_ADDR_PON_CNTL_1, pon, rc);
		goto get_out;
	}

get_out:
#ifndef CONFIG_MACH_VERDI_LTE    //[Puccini] QCT patch: SMPS actvie pull-down
#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&pmic_chip->pm_lock, irqsave);
#else
	mutex_unlock(&pmic_chip->pm_lock);
#endif
#endif // !CONFIG_MACH_VERDI_LTE

	return rc;
}
EXPORT_SYMBOL(pm8058_reset_pwr_off);

/* Internal functions */
static inline int
pm8058_config_irq(struct pm8058_chip *chip, u8 *bp, u8 *cp)
{
	int	rc;

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_BLK_SEL, bp, 1);
	if (rc) {
		pr_err("%s: ssbi_write: rc=%d (Select block)\n",
			__func__, rc);
		goto bail_out;
	}

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_CONFIG, cp, 1);
	if (rc)
		pr_err("%s: ssbi_write: rc=%d (Configure IRQ)\n",
			__func__, rc);

bail_out:
	return rc;
}

static void pm8058_irq_mask(unsigned int irq)
{
	int	master, irq_bit;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= chip->pdata.irq_base;
	block = irq / 8;
	master = block / 8;
	irq_bit = irq % 8;

	chip->irqs_allowed[block] &= ~(1 << irq_bit);
	if (!chip->irqs_allowed[block]) {
		chip->blocks_allowed[master] &= ~(1 << (block % 8));

		if (!chip->blocks_allowed[master])
			chip->masters_allowed &= ~(1 << master);
	}

	config = PM8058_IRQF_WRITE | chip->config[irq] |
		PM8058_IRQF_MASK_FE | PM8058_IRQF_MASK_RE;
	chip->bus_unlock_config[irq] = config;
}

static void pm8058_irq_unmask(unsigned int irq)
{
	int	master, irq_bit;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config, old_irqs_allowed, old_blocks_allowed;

	irq -= chip->pdata.irq_base;
	block = irq / 8;
	master = block / 8;
	irq_bit = irq % 8;

	old_irqs_allowed = chip->irqs_allowed[block];
	chip->irqs_allowed[block] |= 1 << irq_bit;
	if (!old_irqs_allowed) {
		master = block / 8;

		old_blocks_allowed = chip->blocks_allowed[master];
		chip->blocks_allowed[master] |= 1 << (block % 8);

		if (!old_blocks_allowed)
			chip->masters_allowed |= 1 << master;
	}

	config = PM8058_IRQF_WRITE | chip->config[irq];
	chip->bus_unlock_config[irq] = config;
}

static void pm8058_irq_ack(unsigned int irq)
{
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= chip->pdata.irq_base;
	block = irq / 8;

	config = PM8058_IRQF_WRITE | chip->config[irq] | PM8058_IRQF_CLR;
	/* Keep the mask */
	if (!(chip->irqs_allowed[block] & (1 << (irq % 8))))
		config |= PM8058_IRQF_MASK_FE | PM8058_IRQF_MASK_RE;
	chip->bus_unlock_config[irq] = config;
}

static int pm8058_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	int	master, irq_bit;
	struct	pm8058_chip *chip = get_irq_data(irq);
	u8	block, config;

	irq -= chip->pdata.irq_base;
	if (irq > chip->pm_max_irq) {
		chip->pm_max_irq = irq;
		chip->pm_max_blocks =
			chip->pm_max_irq / 8 + 1;
		chip->pm_max_masters =
			chip->pm_max_blocks / 8 + 1;
	}
	block = irq / 8;
	master = block / 8;
	irq_bit = irq % 8;

	chip->config[irq] = (irq_bit << PM8058_IRQF_BITS_SHIFT) |
			PM8058_IRQF_MASK_RE | PM8058_IRQF_MASK_FE;
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		if (flow_type & IRQF_TRIGGER_RISING)
			chip->config[irq] &= ~PM8058_IRQF_MASK_RE;
		if (flow_type & IRQF_TRIGGER_FALLING)
			chip->config[irq] &= ~PM8058_IRQF_MASK_FE;
	} else {
		chip->config[irq] |= PM8058_IRQF_LVL_SEL;

		if (flow_type & IRQF_TRIGGER_HIGH)
			chip->config[irq] &= ~PM8058_IRQF_MASK_RE;
		else
			chip->config[irq] &= ~PM8058_IRQF_MASK_FE;
	}

	config = PM8058_IRQF_WRITE | chip->config[irq] | PM8058_IRQF_CLR;
	chip->bus_unlock_config[irq] = config;
	return 0;
}

static int pm8058_irq_set_wake(unsigned int irq, unsigned int on)
{
	struct	pm8058_chip *chip = get_irq_data(irq);

	irq -= chip->pdata.irq_base;
	if (on) {
		if (!chip->wake_enable[irq]) {
			chip->wake_enable[irq] = 1;
			chip->count_wakeable++;
		}
	} else {
		if (chip->wake_enable[irq]) {
			chip->wake_enable[irq] = 0;
			chip->count_wakeable--;
		}
	}

	return 0;
}

#ifdef CONFIG_MSM8X60_SSBI
unsigned long irqsave_for_pm8058_irq_bus;
#endif
static void pm8058_irq_bus_lock(unsigned int irq)
{
	u8	block;
	struct	pm8058_chip *chip = get_irq_data(irq);

	irq -= chip->pdata.irq_base;
	block = irq / 8;
	chip->bus_unlock_config[irq] = 0;

#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&chip->pm_lock, irqsave_for_pm8058_irq_bus);
#else
	mutex_lock(&chip->pm_lock);
#endif
}

static void pm8058_irq_bus_sync_unlock(unsigned int irq)
{
	u8	block, config;
	struct	pm8058_chip *chip = get_irq_data(irq);

	irq -= chip->pdata.irq_base;
	block = irq / 8;
	config = chip->bus_unlock_config[irq];
	/* dont waste cpu cycles if we dont have data to write */
	if (config)
		pm8058_config_irq(chip, &block, &config);

#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&chip->pm_lock, irqsave_for_pm8058_irq_bus);
#else
	mutex_unlock(&chip->pm_lock);
#endif
}

static inline int
pm8058_read_root(struct pm8058_chip *chip, u8 *rp)
{
	int	rc;

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_ROOT, rp, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read Root)\n",
			__func__, rc);
		*rp = 0;
	}

	return rc;
}

static inline int
pm8058_read_master(struct pm8058_chip *chip, u8 m, u8 *bp)
{
	int	rc;

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_M_STATUS1 + m, bp, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read Master)\n",
			__func__, rc);
		*bp = 0;
	}

	return rc;
}

static inline int
pm8058_read_block(struct pm8058_chip *chip, u8 *bp, u8 *ip)
{
	int	rc;

	rc = ssbi_write(chip->dev, SSBI_REG_ADDR_IRQ_BLK_SEL, bp, 1);
	if (rc) {
		pr_err("%s: FAIL ssbi_write(): rc=%d (Select Block)\n",
		       __func__, rc);
		*bp = 0;
		goto bail_out;
	}

	rc = ssbi_read(chip->dev, SSBI_REG_ADDR_IRQ_IT_STATUS, ip, 1);
	if (rc)
		pr_err("%s: FAIL ssbi_read(): rc=%d (Read Status)\n",
		       __func__, rc);

bail_out:
	return rc;
}

static irqreturn_t pm8058_isr_thread(int irq_requested, void *data)
{
	struct pm8058_chip *chip = data;
	int	i, j, k;
	u8	root, block, config, bits;
	u8	blocks[MAX_PM_MASTERS];
	int	masters = 0, irq, handled = 0, spurious = 0;
	u16     irqs_to_handle[MAX_PM_IRQ];
#ifdef CONFIG_MSM8X60_SSBI
	unsigned long irqsave;

	spin_lock_irqsave(&chip->pm_lock, irqsave);
#else
	mutex_lock(&chip->pm_lock);
#endif

	/* Read root for masters */
	if (pm8058_read_root(chip, &root))
		goto bail_out;

	masters = root >> 1;

	if (!(masters & chip->masters_allowed) ||
	    (masters & ~chip->masters_allowed)) {
		spurious = 1000000;
	}

	/* Read allowed masters for blocks. */
	for (i = 0; i < chip->pm_max_masters; i++) {
		if (masters & (1 << i)) {
			if (pm8058_read_master(chip, i, &blocks[i]))
				goto bail_out;

			if (!blocks[i]) {
				if (pm8058_can_print())
					pr_err("%s: Spurious master: %d "
					       "(blocks=0)", __func__, i);
				spurious += 10000;
			}
		} else
			blocks[i] = 0;
	}

	/* Select block, read status and call isr */
	for (i = 0; i < chip->pm_max_masters; i++) {
		if (!blocks[i])
			continue;

		for (j = 0; j < 8; j++) {
			if (!(blocks[i] & (1 << j)))
				continue;

			block = i * 8 + j;	/* block # */
			if (pm8058_read_block(chip, &block, &bits))
				goto bail_out;

			if (!bits) {
				if (pm8058_can_print())
					pr_err("%s: Spurious block: "
					       "[master, block]=[%d, %d] "
					       "(bits=0)\n", __func__, i, j);
				spurious += 100;
				continue;
			}

			/* Check IRQ bits */
			for (k = 0; k < 8; k++) {
				if (!(bits & (1 << k)))
					continue;

				/* Check spurious interrupts */
				if (((1 << i) & chip->masters_allowed) &&
				    (blocks[i] & chip->blocks_allowed[i]) &&
				    (bits & chip->irqs_allowed[block])) {

					/* Found one */
					irq = block * 8 + k;
					irqs_to_handle[handled] = irq +
						chip->pdata.irq_base;
					handled++;
				} else {
					/* Clear and mask wrong one */
					config = PM8058_IRQF_W_C_M |
						(k << PM8058_IRQF_BITS_SHIFT);

					pm8058_config_irq(chip,
							  &block, &config);

					if (pm8058_can_print())
						pr_err("%s: Spurious IRQ: "
						       "[master, block, bit]="
						       "[%d, %d (%d), %d]\n",
							__func__,
						       i, j, block, k);
					spurious++;
				}
			}
		}

	}

bail_out:

#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);
#else
	mutex_unlock(&chip->pm_lock);
#endif

	for (i = 0; i < handled; i++)
		handle_nested_irq(irqs_to_handle[i]);
#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&chip->pm_lock, irqsave);
#endif
	for (i = 0; i < handled; i++) {
		irqs_to_handle[i] -= chip->pdata.irq_base;
		block  = irqs_to_handle[i] / 8 ;
		config = PM8058_IRQF_WRITE | chip->config[irqs_to_handle[i]]
				| PM8058_IRQF_CLR;
		pm8058_config_irq(chip, &block, &config);
	}
#ifdef CONFIG_MSM8X60_SSBI
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);
#endif
	if (spurious) {
		if (!pm8058_can_print())
			return IRQ_HANDLED;

		pr_err("%s: spurious = %d (handled = %d)\n",
		       __func__, spurious, handled);
		pr_err("   root = 0x%x (masters_allowed<<1 = 0x%x)\n",
		       root, chip->masters_allowed << 1);
		for (i = 0; i < chip->pm_max_masters; i++) {
			if (masters & (1 << i))
				pr_err("   blocks[%d]=0x%x, "
				       "allowed[%d]=0x%x\n",
				       i, blocks[i],
				       i, chip->blocks_allowed[i]);
		}
	}

	return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)

static int check_addr(int addr, const char *func_name)
{
	if (addr < 0 || addr > 0x3FF) {
		pr_err("%s: PMIC 8058 register address is invalid: %d\n",
			func_name, addr);
		return -EINVAL;
	}
	return 0;
}

static int data_set(void *data, u64 val)
{
	struct pm8058_dbg_device *dbgdev = data;
	u8 reg = val;
	int rc;

	mutex_lock(&dbgdev->dbg_mutex);

	rc = check_addr(dbgdev->addr, __func__);
	if (rc)
		goto done;

	rc = pm8058_write(dbgdev->pm_chip, dbgdev->addr, &reg, 1);

	if (rc)
		pr_err("%s: FAIL pm8058_write(0x%03X)=0x%02X: rc=%d\n",
			__func__, dbgdev->addr, reg, rc);
done:
	mutex_unlock(&dbgdev->dbg_mutex);
	return rc;
}

static int data_get(void *data, u64 *val)
{
	struct pm8058_dbg_device *dbgdev = data;
	int rc;
	u8 reg;

	mutex_lock(&dbgdev->dbg_mutex);

	rc = check_addr(dbgdev->addr, __func__);
	if (rc)
		goto done;

	rc = pm8058_read(dbgdev->pm_chip, dbgdev->addr, &reg, 1);

	if (rc) {
		pr_err("%s: FAIL pm8058_read(0x%03X)=0x%02X: rc=%d\n",
			__func__, dbgdev->addr, reg, rc);
		goto done;
	}

	*val = reg;
done:
	mutex_unlock(&dbgdev->dbg_mutex);
	return rc;
}

DEFINE_SIMPLE_ATTRIBUTE(dbg_data_fops, data_get, data_set, "0x%02llX\n");

static int addr_set(void *data, u64 val)
{
	struct pm8058_dbg_device *dbgdev = data;
	int rc;

	rc = check_addr(val, __func__);
	if (rc)
		return rc;

	mutex_lock(&dbgdev->dbg_mutex);
	dbgdev->addr = val;
	mutex_unlock(&dbgdev->dbg_mutex);

	return 0;
}

static int addr_get(void *data, u64 *val)
{
	struct pm8058_dbg_device *dbgdev = data;
	int rc;

	mutex_lock(&dbgdev->dbg_mutex);

	rc = check_addr(dbgdev->addr, __func__);
	if (rc) {
		mutex_unlock(&dbgdev->dbg_mutex);
		return rc;
	}
	*val = dbgdev->addr;

	mutex_unlock(&dbgdev->dbg_mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(dbg_addr_fops, addr_get, addr_set, "0x%03llX\n");

static int __devinit pmic8058_dbg_probe(struct pm8058_chip *chip)
{
	struct pm8058_dbg_device *dbgdev;
	struct dentry *dent;
	struct dentry *temp;

	if (chip == NULL) {
		pr_err("%s: no parent data passed in.\n", __func__);
		return -EINVAL;
	}

	dbgdev = kzalloc(sizeof *dbgdev, GFP_KERNEL);
	if (dbgdev == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dbgdev->dbg_mutex);

	dbgdev->pm_chip = chip;
	dbgdev->addr = -1;

	dent = debugfs_create_dir("pm8058-dbg", NULL);
	if (dent == NULL || IS_ERR(dent)) {
		pr_err("%s: ERR debugfs_create_dir: dent=0x%X\n",
					__func__, (unsigned)dent);
		return -ENOMEM;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, dent,
					dbgdev, &dbg_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("%s: ERR debugfs_create_file: dent=0x%X\n",
					__func__, (unsigned)temp);
		goto debug_error;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, dent,
					dbgdev, &dbg_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("%s: ERR debugfs_create_file: dent=0x%X\n",
					__func__, (unsigned)temp);
		goto debug_error;
	}

	dbgdev->dent = dent;

	pmic_dbg_device = dbgdev;

	return 0;

debug_error:
	debugfs_remove_recursive(dent);
	return -ENOMEM;
}

static int __devexit pmic8058_dbg_remove(void)
{
	if (pmic_dbg_device) {
		debugfs_remove_recursive(pmic_dbg_device->dent);
		kfree(pmic_dbg_device);
	}
	return 0;
}

#else

static int __devinit pmic8058_dbg_probe(struct pm8058_chip *chip)
{
	return 0;
}

static int __devexit pmic8058_dbg_remove(void)
{
	return 0;
}

#endif

static struct irq_chip pm8058_irq_chip = {
	.name      = "pm8058",
	.ack       = pm8058_irq_ack,
	.mask      = pm8058_irq_mask,
	.unmask    = pm8058_irq_unmask,
	.set_type  = pm8058_irq_set_type,
	.set_wake  = pm8058_irq_set_wake,
	.bus_lock  = pm8058_irq_bus_lock,
	.bus_sync_unlock  = pm8058_irq_bus_sync_unlock,
};

static int pm8058_probe(
	#ifdef CONFIG_MSM8X60_SSBI
			struct platform_device *pdev
	#else
			struct i2c_client *client,
			const struct i2c_device_id *id
	#endif
	)
{
	int	i, rc;
#ifdef CONFIG_MSM8X60_SSBI
	struct	pm8058_platform_data *pdata = pdev->dev.platform_data;
	unsigned long irqsave;
#else
	struct	pm8058_platform_data *pdata = client->dev.platform_data;
#endif
	struct	pm8058_chip *chip;


#ifdef CONFIG_MSM8X60_SSBI
	if (pdata == NULL || !pdata->irq)
#else
	if (pdata == NULL || !client->irq)
#endif
	{
		pr_err("%s: No platform_data or IRQ.\n", __func__);
		return -ENODEV;
	}

	if (pdata->num_subdevs == 0) {
		pr_err("%s: No sub devices to support.\n", __func__);
		return -ENODEV;
	}

#ifndef CONFIG_MSM8X60_SSBI
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		pr_err("%s: i2c_check_functionality failed.\n", __func__);
		return -ENODEV;
	}
#endif

	chip = kzalloc(sizeof *chip, GFP_KERNEL);
	if (chip == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_MSM8X60_SSBI
	chip->dev = &pdev->dev;
	chip->id = pdev->id;
#else
	chip->dev = client;
#endif

	(void) memcpy((void *)&chip->pdata, (const void *)pdata,
		      sizeof(chip->pdata));

#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_init(&chip->pm_lock);
#else
	mutex_init(&chip->pm_lock);
#endif
#ifdef CONFIG_MSM8X60_SSBI
	set_irq_data(chip->pdata.irq, (void *)chip);
	set_irq_wake(chip->pdata.irq, 1);
#else
	set_irq_data(chip->dev->irq, (void *)chip);
	set_irq_wake(chip->dev->irq, 1);
#endif

	chip->pm_max_irq = 0;
	chip->pm_max_blocks = 0;
	chip->pm_max_masters = 0;

#ifdef CONFIG_MSM8X60_SSBI
	platform_set_drvdata(pdev, chip);
#else
	i2c_set_clientdata(client, chip);
#endif
	pmic_chip = chip;

	/* Read PMIC chip revision */
#ifdef CONFIG_MSM8X60_SSBI
	spin_lock_irqsave(&chip->pm_lock, irqsave);
	rc = ssbi_read(chip->dev, SSBI_REG_REV, &chip->revision, 1);
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);
#else
	rc = ssbi_read(chip->dev, SSBI_REG_REV, &chip->revision, 1);
#endif /* CONFIG_MSM8X60_SSBI */

	if (rc)
		pr_err("%s: Failed on ssbi_read for revision: rc=%d.\n",
			__func__, rc);
	pr_info("%s: PMIC revision: %X\n", __func__, chip->revision);

	/* Register for all reserved IRQs */
	for (i = pdata->irq_base; i < (pdata->irq_base + MAX_PM_IRQ); i++) {
		set_irq_chip(i, &pm8058_irq_chip);
		set_irq_data(i, (void *)chip);
		set_irq_handler(i, handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
		set_irq_nested_thread(i, 1);
	}

	/* Add sub devices with the chip parameter as driver data */
	for (i = 0; i < pdata->num_subdevs; i++)
		pdata->sub_devices[i].driver_data = chip;
#ifdef CONFIG_MSM8X60_SSBI
	rc = mfd_add_devices(chip->dev, 0, pdata->sub_devices,
			     pdata->num_subdevs, NULL, 0);
#else
	rc = mfd_add_devices(&chip->dev->dev, 0, pdata->sub_devices,
			     pdata->num_subdevs, NULL, 0);
#endif

	/* Add charger sub device with the chip parameter as driver data */
	if (pdata->charger_sub_device) {
		pdata->charger_sub_device->driver_data = chip;
#ifdef CONFIG_MSM8X60_SSBI
		rc = mfd_add_devices(chip->dev, 0,
			pdata->charger_sub_device,
			1, NULL, 0);
#else
		rc = mfd_add_devices(&chip->dev->dev, 0,
			pdata->charger_sub_device,
			1, NULL, 0);
#endif
	}

	if (pdata->init) {
		rc = pdata->init(chip);
		if (rc != 0) {
			pr_err("%s: board init failed\n", __func__);
			chip->dev = NULL;
			kfree(chip);
			return -ENODEV;
		}
	}

#ifdef CONFIG_MSM8X60_SSBI
	rc = request_threaded_irq(pdata->irq, NULL, pm8058_isr_thread,
			IRQF_ONESHOT | IRQF_DISABLED | pdata->irq_trigger_flags,
			"pm8058-irq", chip);
	if (rc < 0)
		pr_err("%s: could not request irq %d: %d\n", __func__,
				pdata->irq, rc);
#else
	rc = request_threaded_irq(chip->dev->irq, NULL, pm8058_isr_thread,
			IRQF_ONESHOT | IRQF_DISABLED | pdata->irq_trigger_flags,
			"pm8058-irq", chip);
	if (rc < 0)
		pr_err("%s: could not request irq %d: %d\n", __func__,
				chip->dev->irq, rc);
#endif

	rc = pmic8058_dbg_probe(chip);
	if (rc < 0)
		pr_err("%s: could not set up debugfs: %d\n", __func__, rc);

	return 0;
}

#ifdef CONFIG_MSM8X60_SSBI
static int __devexit pm8058_remove(struct platform_device *pdev)
{
	struct	pm8058_chip *chip;

	chip = platform_get_drvdata(pdev);
	if (chip) {
		if (chip->pm_max_irq) {
			set_irq_wake(chip->pdata.irq, 0);
			free_irq(chip->pdata.irq, chip);
		}
		chip->dev = NULL;

		kfree(chip);
	}

	pmic8058_dbg_remove();

	return 0;
}
#else
static int __devexit pm8058_remove(struct i2c_client *client)
{
	struct	pm8058_chip *chip;

	chip = i2c_get_clientdata(client);
	if (chip) {
		if (chip->pm_max_irq) {
			set_irq_wake(chip->dev->irq, 0);
			free_irq(chip->dev->irq, chip);
		}
		mutex_destroy(&chip->pm_lock);
		chip->dev = NULL;

		kfree(chip);
	}

	pmic8058_dbg_remove();

	return 0;
}
#endif

#ifdef CONFIG_PM
#ifdef CONFIG_MSM8X60_SSBI
static int pm8058_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct	pm8058_chip *chip;
	int	i, irq;

	chip = platform_get_drvdata(pdev);

	for (i = 0; i < MAX_PM_IRQ; i++) {
		if (chip->config[i] && !chip->wake_enable[i]) {
			if (!((chip->config[i] & PM8058_IRQF_MASK_ALL)
			      == PM8058_IRQF_MASK_ALL)) {
				irq = i + chip->pdata.irq_base;
				pm8058_irq_bus_lock(irq);
				pm8058_irq_mask(irq);
				pm8058_irq_bus_sync_unlock(irq);
			}
		}
	}

	if (!chip->count_wakeable)
		disable_irq(chip->pdata.irq);

	return 0;
}

void pm8058_show_resume_irq(void)
{
	u8	block, bits;
	int i;
	struct pm8058_chip *chip = pmic_chip;
	unsigned long irqsave;

	spin_lock_irqsave(&chip->pm_lock, irqsave);
	for (i = 0; i < MAX_PM_IRQ; i++) {
		if (chip->wake_enable[i]) {
			block = i / 8;
			if (!pm8058_read_block(chip, &block, &bits)) {
				if (bits & (1 << (i & 0x7)))
					pr_warning("%s:%d triggered\n",
					__func__, i + chip->pdata.irq_base);
			}
		}
	}
	spin_unlock_irqrestore(&chip->pm_lock, irqsave);
}

static int pm8058_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct	pm8058_chip *chip;
	int	i, irq;

	chip = platform_get_drvdata(pdev);

	for (i = 0; i < MAX_PM_IRQ; i++) {
		if (chip->config[i] && !chip->wake_enable[i]) {
			if (!((chip->config[i] & PM8058_IRQF_MASK_ALL)
			      == PM8058_IRQF_MASK_ALL)) {
				irq = i + chip->pdata.irq_base;
				pm8058_irq_bus_lock(irq);
				pm8058_irq_unmask(irq);
				pm8058_irq_bus_sync_unlock(irq);
			}
		}
	}

	if (!chip->count_wakeable)
		enable_irq(chip->pdata.irq);

	return 0;
}
#else /*CONFIG_MSM8X60_SSBI*/
static int pm8058_suspend(struct device *dev)
{
	struct i2c_client *client;
	struct	pm8058_chip *chip;
	int	i, irq;

	client = to_i2c_client(dev);
	chip = i2c_get_clientdata(client);

	for (i = 0; i < MAX_PM_IRQ; i++) {
		if (chip->config[i] && !chip->wake_enable[i]) {
			if (!((chip->config[i] & PM8058_IRQF_MASK_ALL)
			      == PM8058_IRQF_MASK_ALL)) {
				irq = i + chip->pdata.irq_base;
				pm8058_irq_bus_lock(irq);
				pm8058_irq_mask(irq);
				pm8058_irq_bus_sync_unlock(irq);
			}
		}
	}

	if (!chip->count_wakeable)
		disable_irq(chip->dev->irq);

	return 0;
}

void pm8058_show_resume_irq(void)
{
	u8	block, bits;
	int i;
	struct pm8058_chip *chip = pmic_chip;

	for (i = 0; i < MAX_PM_IRQ; i++) {
		if (chip->wake_enable[i]) {
			block = i / 8;
			if (!pm8058_read_block(chip, &block, &bits)) {
				if (bits & (1 << (i & 0x7)))
					pr_warning("%s:%d triggered\n",
					__func__, i + chip->pdata.irq_base);
			}
		}
	}
}

static int pm8058_resume(struct device *dev)
{
	struct i2c_client *client;
	struct	pm8058_chip *chip;
	int	i, irq;

	client = to_i2c_client(dev);
	chip = i2c_get_clientdata(client);

	for (i = 0; i < MAX_PM_IRQ; i++) {
		if (chip->config[i] && !chip->wake_enable[i]) {
			if (!((chip->config[i] & PM8058_IRQF_MASK_ALL)
			      == PM8058_IRQF_MASK_ALL)) {
				irq = i + chip->pdata.irq_base;
				pm8058_irq_bus_lock(irq);
				pm8058_irq_unmask(irq);
				pm8058_irq_bus_sync_unlock(irq);
			}
		}
	}

	if (!chip->count_wakeable)
		enable_irq(chip->dev->irq);

	return 0;
}
#endif /*CONFIG_MSM8X60_SSBI*/
#else
#define	pm8058_suspend		NULL
#define	pm8058_resume		NULL
#endif

static struct dev_pm_ops pm8058_pm = {
	.suspend = pm8058_suspend,
	.resume = pm8058_resume,
};
#ifdef CONFIG_MSM8X60_SSBI
static struct platform_driver pm8058_driver = {
	.probe		= pm8058_probe,
	.remove		= __devexit_p(pm8058_remove),
	.driver		= {
		.name	= "pm8058-core",
		.pm		= &pm8058_pm,
		.owner	= THIS_MODULE,
	},
};
#else
static const struct i2c_device_id pm8058_ids[] = {
	{ "pm8058-core", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, pm8058_ids);

static struct i2c_driver pm8058_driver = {
	.driver.name	= "pm8058-core",
	.driver.pm      = &pm8058_pm,
	.id_table	= pm8058_ids,
	.probe		= pm8058_probe,
	.remove		= __devexit_p(pm8058_remove),
};
#endif /*CONFIG_MSM8X60_SSBI */

static int __init pm8058_init(void)
{
#ifdef CONFIG_MSM8X60_SSBI

	pr_info("%s()\n", __func__);
	return platform_driver_register(&pm8058_driver);
#else

	int rc = i2c_add_driver(&pm8058_driver);
	pr_notice("%s: i2c_add_driver: rc = %d\n", __func__, rc);

	return rc;
#endif
}

static void __exit pm8058_exit(void)
{
#ifdef CONFIG_MSM8X60_SSBI
	platform_driver_unregister(&pm8058_driver);
#else
	i2c_del_driver(&pm8058_driver);
#endif
}

arch_initcall(pm8058_init);
module_exit(pm8058_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 core driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-core");
