/*
 * SdioDrv.h
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


/** \file SdioDrv.h
*/

#ifndef __OMAP3430_SDIODRV_API_H
#define __OMAP3430_SDIODRV_API_H

#include <asm/types.h>
#include <linux/mmc/mmc.h>

#define TIWLAN_MMC_MAX_DMA			(8192)
#define SDIO_DRIVER_NAME 			"TIWLAN_SDIO"
#define SDIO_TOTAL_FUNCS			(2)
#define SDIO_WLAN_FUNC				(2)
#define SDIO_BT_FUNC				(1)
#define SDIO_CTRL_FUNC				(0)

/* Card Common Control Registers (CCCR) */

#define CCCR_SDIO_REVISION                  0x00
#define CCCR_SD_SPECIFICATION_REVISION      0x01
#define CCCR_IO_ENABLE                      0x02
#define CCCR_IO_READY                       0x03
#define CCCR_INT_ENABLE                     0x04
#define CCCR_INT_PENDING                    0x05
#define CCCR_IO_ABORT                       0x06
#define CCCR_BUS_INTERFACE_CONTOROL         0x07
#define CCCR_CARD_CAPABILITY	            0x08
#define CCCR_COMMON_CIS_POINTER             0x09 /*0x09-0x0B*/
#define CCCR_FNO_BLOCK_SIZE                 0x10 /*0x10-0x11*/
#define FN0_CCCR_REG_32                     0x64

/* Protocol defined constants */
         
#define SD_IO_GO_IDLE_STATE                 0  
#define SD_IO_SEND_RELATIVE_ADDR            3 
#define SDIO_CMD5                           5
#define SD_IO_SELECT_CARD                   7 

#define VDD_VOLTAGE_WINDOW                  0xffffc0
#define ELP_CTRL_REG_ADDR                   0x1fffc

#define OMAP_MPU_OPP_1GHZ                   1008000000
#define OMAP_MPU_OPP_300MHZ                 300000000

/********************************************************************/
/*	SDIO driver functions prototypes                                */
/********************************************************************/
int sdioDrv_ConnectBus     (void *       fCbFunc,
                            void *       hCbArg,
                            unsigned int uBlkSizeShift,
                            unsigned int uSdioThreadPriority);

int sdioDrv_DisconnectBus  (void);

int sdioDrv_ReadSync       (unsigned int uFunc, 
                            unsigned int uHwAddr, 
                            void *       pData, 
                            unsigned int uLen, 
                            unsigned int bIncAddr,
                            unsigned int bMore);

int sdioDrv_ReadAsync      (unsigned int uFunc, 
                            unsigned int uHwAddr, 
                            void *       pData, 
                            unsigned int uLen, 
                            unsigned int bBlkMode,
                            unsigned int bIncAddr,
                            unsigned int bMore);

int sdioDrv_WriteSync      (unsigned int uFunc, 
                            unsigned int uHwAddr, 
                            void *       pData, 
                            unsigned int uLen,
                            unsigned int bIncAddr,
                            unsigned int bMore);

int sdioDrv_WriteAsync     (unsigned int uFunc, 
                            unsigned int uHwAddr, 
                            void *       pData, 
                            unsigned int uLen, 
                            unsigned int bBlkMode,
                            unsigned int bIncAddr,
                            unsigned int bMore);

int sdioDrv_ReadSyncBytes  (unsigned int  uFunc, 
                            unsigned int  uHwAddr, 
                            unsigned char *pData, 
                            unsigned int  uLen, 
                            unsigned int  bMore);
                           
int sdioDrv_WriteSyncBytes (unsigned int  uFunc, 
                            unsigned int  uHwAddr, 
                            unsigned char *pData, 
                            unsigned int  uLen, 
                            unsigned int  bMore);

int sdioDrv_EnableFunction(unsigned int uFunc);
int sdioDrv_EnableInterrupt(unsigned int uFunc);
int sdioDrv_DisableFunction(unsigned int uFunc);
int sdioDrv_DisableInterrupt(unsigned int uFunc);
int sdioDrv_SetBlockSize(unsigned int uFunc, unsigned int blksz);
void sdioDrv_Register_Notification(void (*notify_sdio_ready)(void));
void sdioDrv_ReleaseHost(unsigned int uFunc);
void sdioDrv_ClaimHost(unsigned int uFunc);
void sdioDrv_start_inact_timer(void);
void sdioDrv_cancel_inact_timer(void);

int sdioDrv_wlan_init(void);
void sdioDrv_exit(void);

#endif/* _OMAP3430_SDIODRV_H */
