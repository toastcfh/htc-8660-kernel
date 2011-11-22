/* linux/arch/arm/mach-msm/board-ruby.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_RUBY_H
#define __ARCH_ARM_MACH_MSM_BOARD_RUBY_H

#include <mach/board.h>

#define RUBY_PROJECT_NAME	"ruby"

#define RUBY_AP2MDM_PMIC_RESET_TIME_MS		1400

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

/* Audio */
#if 0  // Fix Me
#define RUBY_GPIO_AUD_A1026_WAKEUP	(70)
#define RUBY_GPIO_AUD_A1026_INT	(94)
#define RUBY_AUD_A1026_CLK   -1
#endif
/*Gyro*/

#define RUBY_GPIO_PANA_GYRO_SLEEP		(70)
#define RUBY_GPIO_GYRO_ID		(130)
#define RUBY_GPIO_GYRO_DIAG	(41)
/* Direct Keys */
#define RUBY_GPIO_KEY_VOL_DOWN    (103)
#define RUBY_GPIO_KEY_VOL_UP      (104)
#define RUBY_GPIO_KEY_POWER       (125)
#define RUBY_GPIO_KEY_CAPTURE     (94)

/* Wifi */
#define RUBY_GPIO_WIFI_IRQ              (46)
#define RUBY_GPIO_WIFI_SHUTDOWN_N       (62)

/* Sensors */
#define RUBY_GPIO_SENSOR_I2C_SCL		(115)
#define RUBY_GPIO_SENSOR_I2C_SDA		(116)
#define RUBY_GPIO_GYRO_INT			(126)
#define RUBY_GPIO_COMPASS_INT		(128)
#define RUBY_GPIO_GSENSOR_INT_N		(127)

#define RUBY_LAYOUTS			{ \
			{ { 0,  1, 0}, {-1,  0,  0}, {0, 0, 1} }, \
			{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
			{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
			{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }   \
				}
/* General */
#define RUBY_GENERAL_I2C_SCL		(59)
#define RUBY_GENERAL_I2C_SDA		(60)

/* Microp */

/* TP */
#define RUBY_TP_I2C_SDA           (51)
#define RUBY_TP_I2C_SCL           (52)
#define RUBY_TP_ATT_N             (117)

/* LCD */
#define RUBY_GPIO_LCM_RST_N			(137)
#define RUBY_GPIO_LCM_ID0		(64)
#define RUBY_GPIO_LCM_ID1		(65)

/* Audio */
#define RUBY_AUD_CODEC_RST        (118)
#define RUBY_AUD_QTR_RESET        (158)

/* Battery */
#define RUBY_GPIO_MBAT_IN	    (61)
#define RUBY_GPIO_CHG_INT           (124)

/* BT */
#define RUBY_GPIO_BT_UART1_TX       (53)
#define RUBY_GPIO_BT_UART1_RX       (54)
#define RUBY_GPIO_BT_UART1_CTS      (55)
#define RUBY_GPIO_BT_UART1_RTS      (56)
#define RUBY_GPIO_BT_EN             (142)

/* USB and UART */
#define RUBY_GPIO_UART_RX           (105)
#define RUBY_GPIO_UART_TX           (106)

/* Cable detect */
#define RUBY_GPIO_MHL_USB_SEL		(1)
#define RUBY_GPIO_USB_ID			(63)

/* Camera */
#define RUBY_CAM_I2C_SDA             (47)
#define RUBY_CAM_I2C_SCL             (48)
#define RUBY_CLK_SWITCH 				(18)
#define RUBY_CAM1_RST				(49)
#define RUBY_CAM1_VCM_PD				(19)
#define RUBY_CAM2_RST				(101)
/* #define RUBY_CAM2_STANDBY			(102) */
#define RUBY_CAM1_CAM_ID				(157)

/* Flashlight */
#define RUBY_FLASH_EN             (138)
#define RUBY_TORCH_EN             (30)

#ifdef CONFIG_FB_MSM_HDMI_MHL
/* MHL */
#define RUBY_GPIO_H2W_3V3_EN       (0)
#define RUBY_GPIO_MHL_RST_N   		(2)
#define RUBY_GPIO_MHL_INTR_N  		(50)
#define RUBY_GPIO_MHL_SCL			(170)
#define RUBY_GPIO_MHL_SDA			(171)
#define RUBY_GPIO_MHL_HPD			(172)
#endif

/* Accessory */
#define RUBY_GPIO_AUD_HP_DET        (31)

/* SPI */
#define RUBY_SPI_DO                 (33)
#define RUBY_SPI_DI                 (34)
#define RUBY_SPI_CS                 (35)
#define RUBY_SPI_CLK                (36)

/* SD */
#define RUBY_SD_DETECT_PIN                (37)

#define RUBY_PSNENOR_INTz		(123)

/* LTE */
#define RUBY_AP2MDM_STATUS         (136)
#define RUBY_MDM2AP_STATUS         (134)
#define RUBY_MDM2AP_WAKEUP          (40)
#define RUBY_MDM2AP_ERRFATAL       (133)
#define RUBY_AP2MDM_ERRFATAL        (93)

#define RUBY_AP2MDM_PMIC_RESET_N   (131)
#define RUBY_AP2MDM_KPDPWR_N        (38)
#define RUBY_AP2PMIC_TMPNI_CKEN    (141)

#define RUBY_MDM2AP_VDDMIN		(140)
#define RUBY_MDM2AP_SYNC           (129)
#define RUBY_AP2MDM_WAKEUP         (135)
#define RUBY_MDM2AP_VFR             (29)

/* others */
#define RUBY_REGION_ID		    (39)

/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define RUBY_VOL_UP             (104)
#define RUBY_VOL_DN             (103)
#define RUBY_GPIO_KEY_CAMCODER  PMGPIO(22)
#define RUBY_GPIO_KEY_CAMAF     PMGPIO(35)

#define RUBY_LED_3V3          	PMGPIO(24)
#define RUBY_AUD_HANDSET_ENO    PMGPIO(18)
/* #define RUBY_PS_VOUT            PMGPIO(22) */
#define RUBY_AUD_MIC_SEL        PMGPIO(37)
/* #define RUBY_PLS_INT            PMGPIO(35) */
#define RUBY_WIFI_BT_FAST_CLK  PMGPIO(8)
#define RUBY_WIFI_BT_SLEEP_CLK  PMGPIO(38)
#define RUBY_TP_RST             PMGPIO(23)
#define RUBY_CHG_STAT	   PMGPIO(33)
#define RUBY_AUD_REMO_PRES	PMGPIO(7)
#define RUBY_H2W_CABLE_IN1	PMGPIO(36)
#define RUBY_H2W_CABLE_IN2	PMGPIO(9)
#define RUBY_H2W_IO1_CLK	PMGPIO(34)
#define RUBY_H2W_IO2_DAT	PMGPIO(10)
#define RUBY_PSENSOR_PVT_INTz	PMGPIO(15)

/* NFC */
#define RUBY_NFC_I2C_SDA             (43)
#define RUBY_NFC_I2C_SCL             (44)
#define RUBY_GPIO_NFC_VEN		(20)
#define RUBY_GPIO_NFC_INT		(58)
#define RUBY_GPIO_NFC_DL		(21)

int __init ruby_init_mmc(void);
void __init ruby_audio_init(void);
int __init ruby_init_keypad(void);
int __init ruby_wifi_init(void);
int __init ruby_init_panel(struct resource *res, size_t size);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_RUBY_H */
