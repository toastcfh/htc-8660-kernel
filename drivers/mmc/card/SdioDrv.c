 /*
 * SdioDrv.c
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Texas Instruments nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file SdioDrv.c
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>

#include "SdioDrvDbg.h"
#include "SdioDrv.h"

static DECLARE_COMPLETION(sdio_ready);

typedef struct OMAP3430_sdiodrv
{
	void          (*BusTxnCB)(void* BusTxnHandle, int status);
	void*         BusTxnHandle;
	unsigned int  uBlkSize;
	unsigned int  uBlkSizeShift;
	void          *async_buffer;
	unsigned int  async_length;
	int           async_status;
	struct device *dev;
	void		(*notify_sdio_ready)(void);
	int			sdio_host_claim_ref;
	struct work_struct sdio_opp_set_work;
	struct timer_list inact_timer;
	int    inact_timer_running;
} OMAP3430_sdiodrv_t;

int g_sdio_debug_level = SDIO_DEBUGLEVEL_ERR;
extern int sdio_reset_comm(struct mmc_card *card);
unsigned char *pElpData;

static OMAP3430_sdiodrv_t g_drv;
static struct sdio_func *tiwlan_func[1 + SDIO_TOTAL_FUNCS];

void sdioDrv_Register_Notification(void (*notify_sdio_ready)(void))
{
	g_drv.notify_sdio_ready = notify_sdio_ready;

	/* do we already have an sdio function available ?
	 * (this is relevant in real card-detect scenarios like external boards)
	 * If so, notify its existence to the WLAN driver */
	if (tiwlan_func[SDIO_WLAN_FUNC] && g_drv.notify_sdio_ready)
			g_drv.notify_sdio_ready();
}

static void sdioDrv_inact_timer(unsigned long data)
{
	g_drv.inact_timer_running = 0;
	schedule_work(&g_drv.sdio_opp_set_work);
}

void sdioDrv_start_inact_timer(void)
{
	mod_timer(&g_drv.inact_timer, jiffies + msecs_to_jiffies(1000));
	g_drv.inact_timer_running = 1;
}

void sdioDrv_cancel_inact_timer(void)
{
	if(g_drv.inact_timer_running) {
		del_timer_sync(&g_drv.inact_timer);
		g_drv.inact_timer_running = 0;
	}
	cancel_work_sync(&g_drv.sdio_opp_set_work);
}

static void sdioDrv_opp_setup(struct work_struct *work)
{
	sdioDrv_ReleaseHost(SDIO_WLAN_FUNC);
}

void sdioDrv_ClaimHost(unsigned int uFunc)
{
	if (g_drv.sdio_host_claim_ref)
		return;

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	g_drv.sdio_host_claim_ref = 1;

	sdio_claim_host(tiwlan_func[uFunc]);
}

void sdioDrv_ReleaseHost(unsigned int uFunc)
{
	if (!g_drv.sdio_host_claim_ref)
		return;

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	g_drv.sdio_host_claim_ref = 0;

	sdio_release_host(tiwlan_func[uFunc]);
}

int sdioDrv_ConnectBus(void *fCbFunc,
                        void *hCbArg,
                        unsigned int uBlkSizeShift,
                        unsigned int uSdioThreadPriority)
{
	g_drv.BusTxnCB      = fCbFunc;
	g_drv.BusTxnHandle  = hCbArg;
	g_drv.uBlkSizeShift = uBlkSizeShift;
	g_drv.uBlkSize      = 1 << uBlkSizeShift;

	return 0;
}

int sdioDrv_DisconnectBus(void)
{

	return 0;
}

static int generic_read_bytes(unsigned int uFunc, unsigned int uHwAddr,
								unsigned char *pData, unsigned int uLen,
								unsigned int bIncAddr, unsigned int bMore)
{
	unsigned int i;
	int ret;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);

	BUG_ON(uFunc != SDIO_CTRL_FUNC && uFunc != SDIO_WLAN_FUNC);

	for (i = 0; i < uLen; i++) {
		if (uFunc == 0)
			*pData = sdio_f0_readb(tiwlan_func[uFunc], uHwAddr, &ret);
		else
			*pData = sdio_readb(tiwlan_func[uFunc], uHwAddr, &ret);

		if (0 != ret) {
			printk(KERN_ERR "%s: function %d sdio error: %d\n", __func__, uFunc, ret);
            return -1;
        }

        pData++;
		if (bIncAddr)
			uHwAddr++;
	}

	return 0;
}

static int generic_write_bytes(unsigned int uFunc, unsigned int uHwAddr,
								unsigned char *pData, unsigned int uLen,
								unsigned int bIncAddr, unsigned int bMore)
{
	unsigned int i;
	int ret;
	unsigned int defFunc;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int) pData, uLen);

	BUG_ON(uFunc != SDIO_CTRL_FUNC && uFunc != SDIO_WLAN_FUNC);

	for (i = 0; i < uLen; i++) {
		if (uFunc == 0) {
			/* sdio_f0_writeb(tiwlan_func[uFunc], *pData, uHwAddr, &ret); */
			/* WorkAround:
			 * Using sdio_writeb API for bypassing address out of range issue.
			 * Simulating function number to 0 and then restoring it back
			 */
			defFunc = tiwlan_func[uFunc]->num;
			tiwlan_func[uFunc]->num = 0;
			sdio_writeb(tiwlan_func[uFunc], *pData, uHwAddr, &ret);
			tiwlan_func[uFunc]->num = defFunc;
		}
		else
			sdio_writeb(tiwlan_func[uFunc], *pData, uHwAddr, &ret);

		if (0 != ret) {
			printk(KERN_ERR "%s: function %d sdio error: %d\n", __func__, uFunc, ret);
            return -1;
        }

        pData++;
		if (bIncAddr)
			uHwAddr++;
	}

	return 0;
}

int sdioDrv_ReadSync(unsigned int uFunc,
                      unsigned int uHwAddr,
                      void *pData,
                      unsigned int uLen,
                      unsigned int bIncAddr,
                      unsigned int bMore)
{
	int ret;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);

	/* If request is either for sdio function 0 or not a multiple of 4 (OMAP DMA limit)
	    then we have to use CMD 52's */
	if (uFunc == SDIO_CTRL_FUNC || uLen % 4 != 0)
    {
		ret = generic_read_bytes(uFunc, uHwAddr, pData, uLen, bIncAddr, bMore);
    }
	else
    {
		if (bIncAddr)
			ret = sdio_memcpy_fromio(tiwlan_func[uFunc], pData, uHwAddr, uLen);
		else
			ret = sdio_readsb(tiwlan_func[uFunc], pData, uHwAddr, uLen);
    }

	if (ret) {
		printk(KERN_ERR "%s: sdio error: %d\n", __func__, ret);
		return -1;
	}

	return 0;
}

int sdioDrv_ReadAsync(unsigned int uFunc,
                       unsigned int uHwAddr,
                       void *pData,
                       unsigned int uLen,
                       unsigned int bBlkMode,
                       unsigned int bIncAddr,
                       unsigned int bMore)
{
	PERR("%s not yet supported!\n", __func__);

	return -1;
}

int sdioDrv_WriteSync(unsigned int uFunc,
                       unsigned int uHwAddr,
                       void *pData,
                       unsigned int uLen,
                       unsigned int bIncAddr,
                       unsigned int bMore)
{
	int ret;

	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d bIncAddr %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen, bIncAddr);

	/* If request is either for sdio function 0 or not a multiple of 4 (OMAP DMA limit)
	    then we have to use CMD 52's */
	if (uFunc == SDIO_CTRL_FUNC || uLen % 4 != 0)
	{
		ret = generic_write_bytes(uFunc, uHwAddr, pData, uLen, bIncAddr, bMore);
	}
	else
	{
		if (bIncAddr)
			ret = sdio_memcpy_toio(tiwlan_func[uFunc], uHwAddr, pData, uLen);
		else
			ret = sdio_writesb(tiwlan_func[uFunc], uHwAddr, pData, uLen);
	}

	if (ret) {
		printk(KERN_ERR "%s: sdio error: %d\n", __func__, ret);
		return -1;
	}

	return 0;
}

int sdioDrv_WriteAsync(unsigned int uFunc,
                        unsigned int uHwAddr,
                        void *pData,
                        unsigned int uLen,
                        unsigned int bBlkMode,
                        unsigned int bIncAddr,
                        unsigned int bMore)
{
	PERR("%s not yet supported!\n", __func__);

	return -1;
}

int sdioDrv_ReadSyncBytes(unsigned int uFunc,
                           unsigned int uHwAddr,
                           unsigned char *pData,
                           unsigned int uLen,
                           unsigned int bMore)
{
	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int)pData, uLen);

	return generic_read_bytes(uFunc, uHwAddr, pData, uLen, 1, bMore);
}

int sdioDrv_WriteSyncBytes(unsigned int uFunc,
                            unsigned int uHwAddr,
                            unsigned char *pData,
                            unsigned int uLen,
                            unsigned int bMore)
{
	PDEBUG("%s: uFunc %d uHwAddr %d pData %x uLen %d\n", __func__, uFunc, uHwAddr, (unsigned int) pData, uLen);
	return generic_write_bytes(uFunc, uHwAddr, pData, uLen, 1, bMore);
}

static void tiwlan_sdio_irq(struct sdio_func *func)
{
	PDEBUG("%s:\n", __func__);
}

int sdioDrv_DisableFunction(unsigned int uFunc)
{
	PDEBUG("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_disable_func(tiwlan_func[uFunc]);
}

int sdioDrv_EnableFunction(unsigned int uFunc)
{
	PDEBUG("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_enable_func(tiwlan_func[uFunc]);
}

int sdioDrv_EnableInterrupt(unsigned int uFunc)
{
	PDEBUG("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_claim_irq(tiwlan_func[uFunc], tiwlan_sdio_irq);
}

int sdioDrv_DisableInterrupt(unsigned int uFunc)
{
	PDEBUG("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_release_irq(tiwlan_func[uFunc]);
}

int sdioDrv_SetBlockSize(unsigned int uFunc, unsigned int blksz)
{
	PDEBUG("%s: func %d\n", __func__, uFunc);

	/* currently only wlan sdio function is supported */
	BUG_ON(uFunc != SDIO_WLAN_FUNC);
	BUG_ON(tiwlan_func[uFunc] == NULL);

	return sdio_set_block_size(tiwlan_func[uFunc], blksz);
}

static int tiwlan_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	PDEBUG("TIWLAN: probed with vendor 0x%x, device 0x%x, class 0x%x\n",
           func->vendor, func->device, func->class);
	printk(KERN_INFO "TIWLAN: tiwlan_sdio_probe +++\n");
	if (func->vendor != SDIO_VENDOR_ID_TI ||
		func->device != SDIO_DEVICE_ID_TI_WL12xx ||
		func->class != SDIO_CLASS_WLAN)
        return -ENODEV;

	printk(KERN_INFO "TIWLAN: Found TI/WLAN SDIO controller (vendor 0x%x, device 0x%x, class 0x%x)\n",
           func->vendor, func->device, func->class);

	tiwlan_func[SDIO_WLAN_FUNC] = func;
	tiwlan_func[SDIO_CTRL_FUNC] = func;

	if (g_drv.notify_sdio_ready)
		g_drv.notify_sdio_ready();

	init_timer(&g_drv.inact_timer);
	g_drv.inact_timer.function = sdioDrv_inact_timer;
	g_drv.inact_timer_running = 0;
	INIT_WORK(&g_drv.sdio_opp_set_work, sdioDrv_opp_setup);
	printk(KERN_INFO "TIWLAN: tiwlan_sdio_probe ---\n");
	return 0;
}

static void tiwlan_sdio_remove(struct sdio_func *func)
{
	PDEBUG("%s\n", __func__);

	sdioDrv_cancel_inact_timer();

	tiwlan_func[SDIO_WLAN_FUNC] = NULL;
	tiwlan_func[SDIO_CTRL_FUNC] = NULL;
}

#if 0
static const struct sdio_device_id tiwl12xx_devices[] = {
       {.class = SDIO_CLASS_WLAN,
  	   	.vendor = SDIO_VENDOR_ID_TI,
  	   	.device = SDIO_DEVICE_ID_TI_WL12xx},
       {}
};
#endif

static const struct sdio_device_id tiwl12xx_devices[] = {
	{ SDIO_DEVICE_CLASS(SDIO_CLASS_WLAN) },
       {}
};

MODULE_DEVICE_TABLE(sdio, tiwl12xx_devices);

int sdio_tiwlan_suspend(struct device *dev)
{
	return 0;
}

int sdio_tiwlan_resume(struct device *dev)
{
	/* Waking up the wifi chip for sdio_reset_comm */
	*pElpData = 1;
	sdioDrv_ClaimHost(SDIO_WLAN_FUNC);
	generic_write_bytes(0, ELP_CTRL_REG_ADDR, pElpData, 1, 1, 0);
	sdioDrv_ReleaseHost(SDIO_WLAN_FUNC);
	//mdelay(5);

	/* Configuring the host and chip back to maximum capability
	 * (bus width and speed)
	 */
	sdio_reset_comm(tiwlan_func[SDIO_WLAN_FUNC]->card);
	return 0;
}

const struct dev_pm_ops sdio_tiwlan_pmops = {
	.suspend = sdio_tiwlan_suspend,
	.resume = sdio_tiwlan_resume,
};

static struct sdio_driver tiwlan_sdio_drv = {
	.probe          = tiwlan_sdio_probe,
	.remove         = tiwlan_sdio_remove,
	.name           = "sdio_tiwlan",
	.id_table       = tiwl12xx_devices,
	.drv = {
		.pm     = &sdio_tiwlan_pmops,
	 },
};

int sdioDrv_wlan_init(void)
{
	int ret;

	PDEBUG("%s: Debug mode\n", __func__);

	memset(&g_drv, 0, sizeof(g_drv));

	ret = sdio_register_driver(&tiwlan_sdio_drv);
	if (ret < 0) {
		sdio_unregister_driver(&tiwlan_sdio_drv);
		ret = sdio_register_driver(&tiwlan_sdio_drv);

		if(ret < 0)
		{
		  printk(KERN_ERR "sdioDrv_init: sdio register failed: %d\n", ret);
		  goto out;
	    }
	}

	pElpData = kmalloc(sizeof (unsigned char), GFP_KERNEL);
	if (!pElpData)
		printk(KERN_ERR "Running out of memory\n");

	printk(KERN_INFO "TI WiLink 1283 SDIO: Driver loaded\n");
	printk("%s complete \n", __FUNCTION__);
	complete(&sdio_ready);
out:
	return ret;
}

void __exit sdioDrv_exit(void)
{
	sdio_unregister_driver(&tiwlan_sdio_drv);

	if(pElpData);
		kfree(pElpData);
	printk(KERN_INFO "TI WiLink 1283 SDIO Driver unloaded\n");
}


module_param(g_sdio_debug_level, int, SDIO_DEBUGLEVEL_ERR);
MODULE_PARM_DESC(g_sdio_debug_level, "TIWLAN SDIO debug level");
#if 0
EXPORT_SYMBOL(g_sdio_debug_level);
EXPORT_SYMBOL(sdioDrv_ConnectBus);
EXPORT_SYMBOL(sdioDrv_DisconnectBus);
EXPORT_SYMBOL(sdioDrv_ReadSync);
EXPORT_SYMBOL(sdioDrv_WriteSync);
EXPORT_SYMBOL(sdioDrv_ReadAsync);
EXPORT_SYMBOL(sdioDrv_WriteAsync);
EXPORT_SYMBOL(sdioDrv_ReadSyncBytes);
EXPORT_SYMBOL(sdioDrv_WriteSyncBytes);
EXPORT_SYMBOL(sdioDrv_EnableFunction);
EXPORT_SYMBOL(sdioDrv_EnableInterrupt);
EXPORT_SYMBOL(sdioDrv_DisableFunction);
EXPORT_SYMBOL(sdioDrv_DisableInterrupt);
EXPORT_SYMBOL(sdioDrv_SetBlockSize);
EXPORT_SYMBOL(sdioDrv_Register_Notification);
#endif
MODULE_DESCRIPTION("TI WLAN 1283 SDIO interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS(SDIO_DRIVER_NAME);
MODULE_AUTHOR("Ohad Ben-Cohen <ohad@wizery.com>");

