/* arch/arm/mach-msm/ssbi.c
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, Google Inc.
 *
 * Original authors: Code Aurura Forum
 *
 * Author: Dima Zavin <dima@android.com>
 *  - Largely rewritten from original to not be an i2c driver.
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
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/msm_ssbi.h>
#include <linux/remote_spinlock.h>

/* SSBI 2.0 controller registers */
#define SSBI2_CMD			0x0008
#define SSBI2_RD			0x0010
#define SSBI2_STATUS			0x0014
#define SSBI2_MODE2			0x001C

/* SSBI_CMD fields */
#define SSBI_CMD_RDWRN			(0x01 << 24)
#define SSBI_CMD_REG_ADDR_SHFT		(0x10)
#define SSBI_CMD_REG_ADDR_MASK		(0xFF << SSBI_CMD_REG_ADDR_SHFT)
#define SSBI_CMD_REG_DATA_SHFT		(0x00)
#define SSBI_CMD_REG_DATA_MASK		(0xFF << SSBI_CMD_REG_DATA_SHFT)

/* SSBI_STATUS fields */
#define SSBI_STATUS_DATA_IN		0x10
#define SSBI_STATUS_RD_CLOBBERED	0x08
#define SSBI_STATUS_RD_READY		0x04
#define SSBI_STATUS_READY		0x02
#define SSBI_STATUS_MCHN_BUSY		0x01

/* SSBI_RD fields */
#define SSBI_RD_RDWRN			0x01000000
#define SSBI_RD_REG_ADDR_SHFT		0x10
#define SSBI_RD_REG_ADDR_MASK		(0xFF << SSBI_RD_REG_ADDR_SHFT)
#define SSBI_RD_REG_DATA_SHFT		(0x00)
#define SSBI_RD_REG_DATA_MASK		(0xFF << SSBI_RD_REG_DATA_SHFT)

/* SSBI_MODE2 fields */
#define SSBI_MODE2_REG_ADDR_15_8_SHFT	0x04
#define SSBI_MODE2_REG_ADDR_15_8_MASK	(0x7F << SSBI_MODE2_REG_ADDR_15_8_SHFT)
#define SSBI_MODE2_ADDR_WIDTH_SHFT	0x01
#define SSBI_MODE2_ADDR_WIDTH_MASK	(0x07 << SSBI_MODE2_ADDR_WIDTH_SHFT)
#define SSBI_MODE2_SSBI2_MODE		0x00000001

#define SSBI_MODE2_REG_ADDR_15_8(MD, AD) \
	(((MD) & 0x0F) | ((((AD) >> 8) << SSBI_MODE2_REG_ADDR_15_8_SHFT) & \
	SSBI_MODE2_REG_ADDR_15_8_MASK))

#define SSBI_MODE2_ADDR_WIDTH(N) \
	((((N) - 8) << SSBI_MODE2_ADDR_WIDTH_SHFT) & SSBI_MODE2_ADDR_WIDTH_MASK)

#define SSBI_TIMEOUT_US			100

#define SSBI_CMD_READ(AD) \
	(SSBI_CMD_RDWRN | (((AD) & 0xFF) << SSBI_CMD_REG_ADDR_SHFT))

#define SSBI_CMD_WRITE(AD, DT) \
	((((AD) & 0xFF) << SSBI_CMD_REG_ADDR_SHFT) | \
	 (((DT) & 0xFF) << SSBI_CMD_REG_DATA_SHFT))

/* SSBI PMIC Arbiter command registers */
#define SSBI_PA_CMD			0x0000
#define SSBI_PA_RD_STATUS		0x0004

/* SSBI_PA_CMD fields */
#define SSBI_PA_CMD_RDWRN		(0x01 << 24)
#define SSBI_PA_CMD_REG_ADDR_14_8_SHFT	(0x10)
#define SSBI_PA_CMD_REG_ADDR_14_8_MASK	(0x7F << SSBI_PA_CMD_REG_ADDR_14_8_SHFT)
#define SSBI_PA_CMD_REG_ADDR_7_0_SHFT	(0x08)
#define SSBI_PA_CMD_REG_ADDR_7_0_MASK	(0xFF << SSBI_PA_CMD_REG_ADDR_7_0_SHFT)
#define SSBI_PA_CMD_REG_DATA_SHFT	(0x00)
#define SSBI_PA_CMD_REG_DATA_MASK	(0xFF << SSBI_PA_CMD_REG_DATA_SHFT)

#define SSBI_PA_CMD_REG_DATA(DT) \
	(((DT) << SSBI_PA_CMD_REG_DATA_SHFT) & SSBI_PA_CMD_REG_DATA_MASK)

#define SSBI_PA_CMD_REG_ADDR(AD) \
	(((AD) << SSBI_PA_CMD_REG_ADDR_7_0_SHFT) & \
	(SSBI_PA_CMD_REG_ADDR_14_8_MASK|SSBI_PA_CMD_REG_ADDR_7_0_MASK))

/* SSBI_PA_RD_STATUS fields */
#define SSBI_PA_RD_STATUS_TRANS_DONE	(0x01 << 27)
#define SSBI_PA_RD_STATUS_TRANS_DENIED	(0x01 << 26)
#define SSBI_PA_RD_STATUS_REG_DATA_SHFT	(0x00)
#define SSBI_PA_RD_STATUS_REG_DATA_MASK	(0xFF << SSBI_PA_CMD_REG_DATA_SHFT)
#define SSBI_PA_RD_STATUS_TRANS_COMPLETE \
	(SSBI_PA_RD_STATUS_TRANS_DONE|SSBI_PA_RD_STATUS_TRANS_DENIED)

static struct device *all_ssbi_devices[3];

struct msm_ssbi {
	struct device		*dev;
	struct device		*slave;
	void __iomem		*base;
	remote_spinlock_t	rspin_lock;
	int use_rlock;
	enum msm_ssbi_controller_type controller_type;
	int (*ssbi_read)(struct device *dev, u16 addr, u8 *buf, int len);
	int (*ssbi_write)(struct device *dev, u16 addr, u8 *buf, int len);
};

#define to_msm_ssbi(dev)	platform_get_drvdata(to_platform_device(dev))

/*
poll_for_device_ready === SSBI_STATUS_READY
poll_for_transfer_completed === SSBI_STATUS_MCHN_BUSY
poll_for_read_completed === SSBI_STATUS_RD_READY
*/
static int ssbi_wait_mask(struct msm_ssbi *ssbi, u32 set_mask)
{
	u32 timeout = SSBI_TIMEOUT_US;

	while (!(readl(ssbi->base + SSBI2_STATUS) & set_mask)) {
		if (--timeout == 0) {
			dev_err(ssbi->dev, "%s: timeout, status %x\n", __func__,
				readl(ssbi->base + SSBI2_STATUS));
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	dev_err(ssbi->dev, "%s: timeout (status %x set_mask %x)\n",
		__func__, readl(ssbi->base + SSBI2_STATUS), set_mask);
	return -ETIMEDOUT;
}

static inline int
msm_ssbi_pa_transfer(struct msm_ssbi *ssbi, u32 cmd, u8 *data)
{
	u32 rd_status;
	u32 timeout = SSBI_TIMEOUT_US;

	writel(cmd, ssbi->base + SSBI_PA_CMD);
	rd_status = readl(ssbi->base + SSBI_PA_RD_STATUS);

	while ((rd_status & (SSBI_PA_RD_STATUS_TRANS_COMPLETE)) == 0) {

		if (--timeout == 0) {
			dev_err(ssbi->dev, "%s: timeout, status %x\n",
					__func__, rd_status);
			return -ETIMEDOUT;
		}
		udelay(1);
		rd_status = readl(ssbi->base + SSBI_PA_RD_STATUS);
	}

	if (rd_status & SSBI_PA_RD_STATUS_TRANS_DENIED) {
		dev_err(ssbi->dev, "%s: transaction denied, status %x\n",
				__func__, rd_status);
		return -EPERM;
	}

	if (data)
		*data = (rd_status & SSBI_PA_RD_STATUS_REG_DATA_MASK) >>
					SSBI_PA_CMD_REG_DATA_SHFT;
	return 0;
}

int msm_ssbi_read_bytes(struct device *dev, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	unsigned long flags = 0;
	u32 read_cmd = SSBI_CMD_READ(addr);
	u32 mode2;
	int ret = 0;

	BUG_ON(ssbi->dev != dev);
	if (ssbi->use_rlock)
		remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);

	if (ssbi->controller_type == MSM_SBI_CTRL_SSBI2) {
		mode2 = readl(ssbi->base + SSBI2_MODE2);
		writel(SSBI_MODE2_REG_ADDR_15_8(mode2, addr),
				ssbi->base + SSBI2_MODE2);
	}

	while (len) {
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_READY);
		if (ret)
			goto err;

		writel(read_cmd, ssbi->base + SSBI2_CMD);

		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_RD_READY);
		if (ret)
			goto err;

		*buf++ = readl(ssbi->base + SSBI2_RD) & SSBI_RD_REG_DATA_MASK;
		len--;
	}

err:
	if (ssbi->use_rlock)
		remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	return ret;
}

int msm_ssbi_pa_read_bytes(struct device *dev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	u32 read_cmd = (SSBI_PA_CMD_RDWRN | SSBI_PA_CMD_REG_ADDR(addr));
	u8 data;
	unsigned long flags = 0;

	BUG_ON(ssbi->dev != dev);
	if (ssbi->use_rlock)
		remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);
	while (len) {
		ret = msm_ssbi_pa_transfer(ssbi, read_cmd, &data);
		if (ret)
			goto read_failed;
		*buf++ = data;
		len--;
	}

read_failed:
	if (ssbi->use_rlock)
		remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	return ret;
}

int msm_ssbi_read(unsigned id, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi;
	if (id > 3) {
		pr_err("%s: id#%u is not availible\n", __func__, id);
		return 1;
	}
	ssbi = to_msm_ssbi(all_ssbi_devices[id]);
	return ssbi->ssbi_read(all_ssbi_devices[id], addr, buf, len);
}
EXPORT_SYMBOL(msm_ssbi_read);

int msm_ssbi_write_bytes(struct device *dev, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	unsigned long flags = 0;
	u32 mode2;
	int ret = 0;

	BUG_ON(ssbi->dev != dev);
	if (ssbi->use_rlock)
		remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);

	if (ssbi->controller_type == MSM_SBI_CTRL_SSBI2) {
		mode2 = readl(ssbi->base + SSBI2_MODE2);
		writel(SSBI_MODE2_REG_ADDR_15_8(mode2, addr),
				ssbi->base + SSBI2_MODE2);
	}

	while (len) {
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_READY);
		if (ret)
			goto err;

		writel(SSBI_CMD_WRITE(addr, *buf++), ssbi->base + SSBI2_CMD);

		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_MCHN_BUSY);
		if (ret)
			goto err;

		len--;
	}

err:
	if (ssbi->use_rlock)
		remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	return ret;
}

int msm_ssbi_pa_write_bytes(struct device *dev, u16 addr, u8 *buf, int len)
{
	int ret = 0;
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	u32 write_cmd;
	u32 w_addr = SSBI_PA_CMD_REG_ADDR(addr);
	unsigned long flags = 0;

	BUG_ON(ssbi->dev != dev);
	if (ssbi->use_rlock)
		remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);

	while (len) {
		write_cmd = w_addr | (*buf++ & SSBI_PA_CMD_REG_DATA_MASK);
		ret = msm_ssbi_pa_transfer(ssbi, write_cmd, NULL);
		if (ret)
			goto write_failed;
		len--;
	}

write_failed:
	if (ssbi->use_rlock)
		remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	return ret;

}

int msm_ssbi_write(unsigned id, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi;
	if (id > 3) {
		pr_err("%s: id#%u is not availible\n", __func__, id);
		return 1;
	}

	ssbi = to_msm_ssbi(all_ssbi_devices[id]);

	return ssbi->ssbi_write(all_ssbi_devices[id], addr, buf, len);
}
EXPORT_SYMBOL(msm_ssbi_write);

static int msm_ssbi_probe(struct platform_device *pdev)
{
	struct msm_ssbi_platform_data *pdata = pdev->dev.platform_data;
	struct resource *mem_res;
	struct msm_ssbi *ssbi;
	int ret = 0;

	if (!pdata) {
		pr_err("%s: missing platform data\n", __func__);
		return -EINVAL;
	}

	ssbi = kzalloc(sizeof(struct msm_ssbi), GFP_KERNEL);
	if (!ssbi) {
		pr_err("%s: cannot allocate ssbi_data\n", __func__);
		return -ENOMEM;
	}

	mem_res = platform_get_resource_byname(pdev,
						IORESOURCE_MEM, "ssbi_base");
	if (!mem_res) {
		pr_err("%s: missing mem resource\n", __func__);
		ret = -EINVAL;
		goto err_get_mem_res;
	}

	if (!request_mem_region(mem_res->start, resource_size(mem_res),
				"msm_ssbi")) {
		ret = -ENXIO;
		pr_err("%s request_mem_region failed\n", __func__);
		goto err_ioremap;
	}

	ssbi->base = ioremap(mem_res->start, resource_size(mem_res));
	if (!ssbi->base) {
		pr_err("%s: ioremap of 0x%p failed\n", __func__,
		       (void *)mem_res->start);
		ret = -EINVAL;
		goto err_ioremap;
	}
	ssbi->dev = &pdev->dev;

	ssbi->controller_type = pdata->controller_type;
	if (pdata->controller_type == MSM_SBI_CTRL_PMIC_ARBITER) {
		ssbi->ssbi_read	= msm_ssbi_pa_read_bytes;
		ssbi->ssbi_write = msm_ssbi_pa_write_bytes;
	} else {
		ssbi->ssbi_read	= msm_ssbi_read_bytes;
		ssbi->ssbi_write = msm_ssbi_write_bytes;
	}

	if (pdata->rsl_id) {
		ret = remote_spin_lock_init(&ssbi->rspin_lock, pdata->rsl_id);
		if (ret) {
			pr_err(
				"%s: cannot init remote spinlock '%s'\n",
				__func__, pdata->rsl_id);
			goto err_remote_spinlock_init;
		}
		ssbi->use_rlock = 1;
	}

	switch (pdev->id) {
	case 0:
	case 1:
	case 2:
		all_ssbi_devices[pdev->id] = ssbi->dev;
		break;
	default:
		pr_err("%s: wrong id assigned for ssbi device. id = %d\n",
			__func__, pdev->id);
	}

	pr_info("msm_ssbi: io=%08x\n", mem_res->start);

	platform_set_drvdata(pdev, ssbi);

	return 0;

err_remote_spinlock_init:
	platform_set_drvdata(pdev, NULL);
	iounmap(ssbi->base);
err_ioremap:
err_get_mem_res:
	kfree(ssbi);
	return ret;
}

static struct platform_driver msm_ssbi_driver = {
	.probe		= msm_ssbi_probe,
	.driver		= {
		.name	= "msm_ssbi",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_ssbi_init(void)
{
	pr_info("%s()\n", __func__);
	return platform_driver_register(&msm_ssbi_driver);
}

postcore_initcall(msm_ssbi_init);
