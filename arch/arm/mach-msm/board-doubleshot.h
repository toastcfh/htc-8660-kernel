/* linux/arch/arm/mach-msm/board-doubleshot.h
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_DOUBLESHOT_H
#define __ARCH_ARM_MACH_MSM_BOARD_DOUBLESHOT_H

#include <mach/board.h>

#define DOUBLESHOT_PROJECT_NAME	"doubleshot"


/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_GPIO_BASE)
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)

#define PM8901_GPIO_BASE			(PM8058_GPIO_BASE + \
						PM8058_GPIOS + PM8058_MPPS)
#define PM8901_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8901_GPIO_BASE)
#define PM8901_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM901_GPIO_BASE)
#define PM8901_IRQ_BASE				(PM8058_IRQ_BASE + \
						NR_PMIC8058_IRQS)

#define GPIO_EXPANDER_GPIO_BASE \
	(PM8901_GPIO_BASE + PM8901_MPPS)
#define GPIO_EXPANDER_IRQ_BASE (PM8901_IRQ_BASE + NR_PMIC8901_IRQS)

#if 0
#define MSM_LINUX_BASE1		0x04000000
#define MSM_LINUX_SIZE1		0x0C000000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0BB00000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE  	0x2BB00000
#define MSM_PMEM_ADSP_SIZE	0x01C00000 /* for 8M(4:3) + gpu effect */
#define PMEM_KERNEL_EBI1_BASE   0x2D700000
#define PMEM_KERNEL_EBI1_SIZE   0x00600000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000
#endif

#define MSM_RAM_CONSOLE_BASE	MSM_HTC_RAM_CONSOLE_PHYS
#define MSM_RAM_CONSOLE_SIZE	MSM_HTC_RAM_CONSOLE_SIZE

/* GPIO definition */


/* Direct Keys */
#define DOUBLESHOT_GPIO_KEY_CAM_STEP1   (99)
#define DOUBLESHOT_GPIO_KEY_CAM_STEP2   (124)
#define DOUBLESHOT_GPIO_KEY_POWER       (125)
#define DOUBLESHOT_GPIO_KEY_SLID_INT    (127)

/* Battery */
#define DOUBLESHOT_GPIO_MBAT_IN         (61)
#define DOUBLESHOT_GPIO_CHG_INT		(126)

/* Wifi */
#define DOUBLESHOT_GPIO_WIFI_IRQ              (46)
#define DOUBLESHOT_GPIO_WIFI_SHUTDOWN_N       (96)
#define DOUBLESHOT_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)
/* Sensors */
#define DOUBLESHOT_SENSOR_I2C_SDA		(72)
#define DOUBLESHOT_SENSOR_I2C_SCL		(73)
#define DOUBLESHOT_GSENSOR_INT		(129)
#define DOUBLESHOT_COMPASS_INT		(128)

/* Microp */
#define DOUBLESHOT_GPIO_UP_RESET_N           (39)
#define DOUBLESHOT_GPIO_UP_INT_N           (57)

/* TP */
#define DOUBLESHOT_TP_I2C_SDA           (51)
#define DOUBLESHOT_TP_I2C_SCL           (52)
#define DOUBLESHOT_TP_ATT_N             (65)
#define DOUBLESHOT_TP_ATT_N_XB          (50)

/* General */
#define DOUBLESHOT_GENERAL_I2C_SDA           (59)
#define DOUBLESHOT_GENERAL_I2C_SCL           (60)

/* LCD */
#define GPIO_LCM_ID1_IM1		(50)
#define GPIO_LCM_ID1_IM1_XB		(65)
#define GPIO_LCM_RST_N			(66)

/* Audio */
#define DOUBLESHOT_AUD_CODEC_RST        (67)
#define DOUBLESHOT_AUD_QTR_TX_MCLK1    (108)
#define DOUBLESHOT_AUD_RX_MCLK1        (109)
#define DOUBLESHOT_AUD_TX_I2S_SD2      (110)

/* BT */
#define DOUBLESHOT_GPIO_BT_HOST_WAKE      (45)
#define DOUBLESHOT_GPIO_BT_UART1_TX       (53)
#define DOUBLESHOT_GPIO_BT_UART1_RX       (54)
#define DOUBLESHOT_GPIO_BT_UART1_CTS      (55)
#define DOUBLESHOT_GPIO_BT_UART1_RTS      (56)
#define DOUBLESHOT_GPIO_BT_SHUTDOWN_N     (100)
#define DOUBLESHOT_GPIO_BT_CHIP_WAKE      (130)
#define DOUBLESHOT_GPIO_BT_RESET_N        (142)

/* USB */
#define DOUBLESHOT_GPIO_USB_ID            (63)

/* DOCK */
#define DOUBLESHOT_GPIO_DOCK_PIN	  (71)

/* Camera */
#define DOUBLESHOT_CAM_I2C_SDA           (47)
#define DOUBLESHOT_CAM_I2C_SCL           (48)


/* Flashlight */
#define DOUBLESHOT_FLASH_EN             (29)
#define DOUBLESHOT_TORCH_EN             (30)

/* Accessory */
#define DOUBLESHOT_GPIO_AUD_HP_DET        (31)

/* SPI */
#define DOUBLESHOT_SPI_DO                 (33)
#define DOUBLESHOT_SPI_DI                 (34)
#define DOUBLESHOT_SPI_CS                 (35)
#define DOUBLESHOT_SPI_CLK                (36)

/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define DOUBLESHOT_FMTX_ANT_SW        PMGPIO(8)
#define DOUBLESHOT_KEYMATRIX_DRV1     PMGPIO(9)
#define DOUBLESHOT_KEYMATRIX_DRV2     PMGPIO(10)
#define DOUBLESHOT_KEYMATRIX_DRV3     PMGPIO(11)
#define DOUBLESHOT_KEYMATRIX_DRV4     PMGPIO(12)
#define DOUBLESHOT_KEYMATRIX_DRV5     PMGPIO(13)
#define DOUBLESHOT_KEYMATRIX_DRV6     PMGPIO(14)
#define DOUBLESHOT_KEYMATRIX_DRV7     PMGPIO(15)
#define DOUBLESHOT_VOL_UP             PMGPIO(16)
#define DOUBLESHOT_VOL_DN             PMGPIO(17)
#define DOUBLESHOT_AUD_HANDSET_ENO    PMGPIO(18)
#define DOUBLESHOT_AUD_SPK_ENO        PMGPIO(19)
#define DOUBLESHOT_WIRELESS_CHG_OK1   PMGPIO(20)
#define DOUBLESHOT_AUD_QTR_RESET      PMGPIO(23)
#define DOUBLESHOT_AUD_HPTV_DET_HP    PMGPIO(24)
#define DOUBLESHOT_AUD_HPTV_DET_TV    PMGPIO(25)
#define DOUBLESHOT_AUD_MIC_SEL        PMGPIO(26)
#define DOUBLESHOT_CHG_STAT	      PMGPIO(33)
#define DOUBLESHOT_PLS_INT            PMGPIO(35)
#define DOUBLESHOT_AUD_TVOUT_HP_SEL   PMGPIO(36)
#define DOUBLESHOT_AUD_REMO_PRES      PMGPIO(37)
#define DOUBLESHOT_WIFI_BT_SLEEP_CLK  PMGPIO(38)

#define DOUBLESHOT_LAYOUTS			{ \
		{ { 0,  1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
			}
#define DOUBLESHOT_TP_RST             PMGPIO(21)
void __init doubleshot_audio_init(void);
int __init doubleshot_wifi_init(void);
int __init doubleshot_init_mmc(void);
int __init doubleshot_init_keypad(void);
unsigned int doubleshot_get_engineerid(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_DOUBLESHOT_H */
