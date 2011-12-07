/* linux/arch/arm/mach-msm/board-holiday.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_HOLIDAY_H
#define __ARCH_ARM_MACH_MSM_BOARD_HOLIDAY_H

#include <mach/board.h>

#define HOLIDAY_PROJECT_NAME	"holiday"

#define HOLIDAY_AP2MDM_PMIC_RESET_TIME_MS		1400

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
#define MSM_RAM_CONSOLE_SIZE	(MSM_HTC_RAM_CONSOLE_SIZE - SZ_64K) /* 64K for CIQ */

#ifdef CONFIG_BUILD_CIQ
#define MSM_PMEM_CIQ_BASE	MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE
#define MSM_PMEM_CIQ_SIZE	SZ_64K
#define MSM_PMEM_CIQ1_BASE	MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ1_SIZE	MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ2_BASE	MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ2_SIZE	MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ3_BASE	MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ3_SIZE	MSM_PMEM_CIQ_SIZE
#endif


/* GPIO definition */

/* Audio */
#define HOLIDAY_GPIO_AUD_A1026_WAKEUP	(70)
#define HOLIDAY_GPIO_AUD_A1026_INT	(94)
#define HOLIDAY_AUD_A1026_CLK   -1


/* Direct Keys */
#define HOLIDAY_GPIO_KEY_VOL_DOWN    (103)
#define HOLIDAY_GPIO_KEY_VOL_UP      (104)
#define HOLIDAY_GPIO_KEY_POWER       (125)

/* Battery */

/* Wifi */
#define HOLIDAY_GPIO_WIFI_IRQ              (46)
#define HOLIDAY_GPIO_WIFI_SHUTDOWN_N       (62)

/* Sensors */
#define HOLIDAY_GPIO_SENSOR_I2C_SCL		(115)
#define HOLIDAY_GPIO_SENSOR_I2C_SDA		(116)
#define HOLIDAY_GPIO_GYRO_INT			(126)
#define HOLIDAY_GPIO_COMPASS_INT		(128)
#define HOLIDAY_GPIO_GSENSOR_INT_N		(127)
#define HOLIDAY_LAYOUTS			{ \
		{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 0, -1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
		{ { 1,  0, 0}, { 0,  0,  1}, {0, 1,  0} }  \
					}
/* General */
#define HOLIDAY_GENERAL_I2C_SCL		(59)
#define HOLIDAY_GENERAL_I2C_SDA		(60)

/* Microp */

/* TP */
#define HOLIDAY_TP_I2C_SDA           (51)
#define HOLIDAY_TP_I2C_SCL           (52)
#define HOLIDAY_TP_ATT_N             (117)

/* LCD */
#define HOLIDAY_GPIO_LCM_RST_N		(137)
#define HOLIDAY_GPIO_LCM_ID0		(64)
#define HOLIDAY_GPIO_LCM_ID1        (65)
#define HOLIDAY_GPIO_LCM_TE			(28)

/* Audio */
#define HOLIDAY_AUD_CODEC_RST        (118)

/* Battery */
#define HOLIDAY_GPIO_MBAT_IN		(61)
#define HOLIDAY_GPIO_CHG_INT		(124)

/* BT */
#define HOLIDAY_GPIO_BT_HOST_WAKE      (45)
#define HOLIDAY_GPIO_BT_UART1_TX       (53)
#define HOLIDAY_GPIO_BT_UART1_RX       (54)
#define HOLIDAY_GPIO_BT_UART1_CTS      (55)
#define HOLIDAY_GPIO_BT_UART1_RTS      (56)
#define HOLIDAY_GPIO_BT_CHIP_WAKE      (130)
#define HOLIDAY_GPIO_BT_RESET_N        (142)
#define HOLIDAY_GPIO_BT_SHUTDOWN_N     (57)

/* USB and UART */
#define HOLIDAY_GPIO_UART_RX           (105)
#define HOLIDAY_GPIO_UART_TX           (106)

/* Cable detect */
#define HOLIDAY_GPIO_MHL_USB_SEL		(1)
#define HOLIDAY_GPIO_USB_ID				(63)

/* Camera */
#define HOLIDAY_CAM_I2C_SDA             (47)
#define HOLIDAY_CAM_I2C_SCL             (48)
#define HOLIDAY_CLK_SWITCH 				(44)
#define HOLIDAY_CAM1_RST				(49)
#define HOLIDAY_CAM1_VCM_PD				(58)
#define HOLIDAY_CAM2_RST				(101)
#define HOLIDAY_CAM2_STANDBY			(102)
#define HOLIDAY_CAM2_CAM_ID				(43)

/* Flashlight */
#define HOLIDAY_FLASH_EN             (138)
#define HOLIDAY_TORCH_EN             (30)

#ifdef CONFIG_FB_MSM_HDMI_MHL
/* MHL */
#define HOLIDAY_GPIO_MHL_RST_N   		(2)
#define HOLIDAY_GPIO_MHL_INTR_N  		(50)
#define HOLIDAY_GPIO_MHL_SCL			(170)
#define HOLIDAY_GPIO_MHL_SDA			(171)
#define HOLIDAY_GPIO_MHL_HPD			(172)
#endif

/* Accessory */
#define HOLIDAY_GPIO_AUD_HP_DET        (31)

/* SPI */
#define HOLIDAY_SPI_DO                 (33)
#define HOLIDAY_SPI_DI                 (34)
#define HOLIDAY_SPI_CS                 (35)
#define HOLIDAY_SPI_CLK                (36)

/* SD */
#define HOLIDAY_SD_DETECT_PIN                (37)

/* LTE */
#define HOLIDAY_AP2MDM_STATUS         (136)
#define HOLIDAY_MDM2AP_STATUS         (134)
#define HOLIDAY_MDM2AP_WAKEUP          (40)
#define HOLIDAY_MDM2AP_ERRFATAL       (133)
#define HOLIDAY_AP2MDM_ERRFATAL        (93)

#define HOLIDAY_AP2MDM_PMIC_RESET_N   (131)
#define HOLIDAY_AP2MDM_KPDPWR_N        (38)
#define HOLIDAY_AP2PMIC_TMPNI_CKEN    (141)

#define HOLIDAY_MDM2AP_VDDMIN		(140)
#define HOLIDAY_MDM2AP_SYNC           (129)
#define HOLIDAY_AP2MDM_WAKEUP         (135)
#define HOLIDAY_MDM2AP_VFR             (29)

#define PSNENOR_INTz		(123)
/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define HOLIDAY_VOL_UP             (104)
#define HOLIDAY_VOL_DN             (103)
#define HOLIDAY_AUD_MIC_SEL2	PMGPIO(16)
#define HOLIDAY_AUD_HANDSET_ENO    PMGPIO(18)
#define HOLIDAY_AUD_A1026_RST	PMGPIO(19)
/* #define HOLIDAY_AUD_QTR_RESET      PMGPIO(21) */
#define HOLIDAY_PS_VOUT            PMGPIO(22)
#define HOLIDAY_GREEN_LED          PMGPIO(24)
#define HOLIDAY_AMBER_LED          PMGPIO(25)
#define HOLIDAY_AUD_MIC_SEL1        PMGPIO(37)
#define HOLIDAY_PLS_INT            PMGPIO(35)
#define HOLIDAY_WIFI_BT_SLEEP_CLK  PMGPIO(38)
#define HOLIDAY_TP_RST             PMGPIO(23)
#define HOLIDAY_CHG_STAT	   PMGPIO(33)
#define HOLIDAY_AUD_REMO_PRES      PMGPIO(7)

int __init holiday_init_mmc(void);
void __init holiday_audio_init(void);
int __init holiday_init_keypad(void);
int __init holiday_wifi_init(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_HOLIDAY_H */
