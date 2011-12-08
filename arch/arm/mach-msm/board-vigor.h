/* linux/arch/arm/mach-msm/board-vigor.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_VIGOR_H
#define __ARCH_ARM_MACH_MSM_BOARD_VIGOR_H

#include <mach/board.h>

#define VIGOR_PROJECT_NAME	"vigor"

#define VIGOR_AP2MDM_PMIC_RESET_TIME_MS		4000

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


#define MSM_RAM_CONSOLE_BASE	MSM_HTC_RAM_CONSOLE_PHYS
#define MSM_RAM_CONSOLE_SIZE	MSM_HTC_RAM_CONSOLE_SIZE

/* GPIO definition */
/* Direct Keys */
#define VIGOR_GPIO_KEY_VOL_DOWN    (103)
#define VIGOR_GPIO_KEY_VOL_UP      (104)
#define VIGOR_GPIO_KEY_POWER       (125)

/* Battery */

/* Wifi */
#define VIGOR_GPIO_WIFI_IRQ              (46)
#define VIGOR_GPIO_WIFI_SHUTDOWN_N       (157)

/* Sensors */
#define VIGOR_GPIO_SENSOR_I2C_SCL_EVM		(73)
#define VIGOR_GPIO_SENSOR_I2C_SDA_EVM		(72)

#define VIGOR_GPIO_SENSOR_I2C_SCL		(115)
#define VIGOR_GPIO_SENSOR_I2C_SDA		(116)

#define VIGOR_GPIO_COMPASS_INT		(128)
#define VIGOR_GPIO_GSENSOR_INT_N		(127)
#define VIGOR_GPIO_GYRO_INT               (126)
#define VIGOR_LAYOUTS			{ \
		{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 0, -1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
		{ { 1,  0, 0}, { 0,  0,  1}, {0, 1,  0} }  \
					}

/* Microp */

/* TP */
#define VIGOR_TP_I2C_SDA           (51)
#define VIGOR_TP_I2C_SCL           (52)
#define VIGOR_TP_ATT_N          (117)

/* LCD */

/* Battery */
#define VIGOR_GPIO_MBAT_IN	    (61)
#define VIGOR_GPIO_CHG_INT	    (124)

/* BT */
#define VIGOR_GPIO_BT_HOST_WAKE      (45)
#define VIGOR_GPIO_BT_UART1_TX       (53)
#define VIGOR_GPIO_BT_UART1_RX       (54)
#define VIGOR_GPIO_BT_UART1_CTS      (55)
#define VIGOR_GPIO_BT_UART1_RTS      (56)
#define VIGOR_GPIO_BT_CHIP_WAKE      (130)
#define VIGOR_GPIO_BT_RESET_N        (141)
#define VIGOR_GPIO_BT_SHUTDOWN_N     (156)

/* USB and UART */
#define VIGOR_USBMHL_AUDz_SW		(44)
#define VIGOR_MHLAUD_USBz_SW_EVM		(102)
#define VIGOR_GPIO_USB_ID            (63)
#define VIGOR_GPIO_NOTIFY_9K_AC_IN            (37)

#define VIGOR_GPIO_UART_RX        (105)
#define VIGOR_GPIO_UART_TX        (106)

/* Camera */
#define VIGOR_CAM_I2C_SDA            (47)
#define VIGOR_CAM_I2C_SCL            (48)
#define VIGOR_CLK_SWITCH 				(65)
#define VIGOR_CAM1_RST				PMGPIO(2)
#define VIGOR_CAM1_VCM_PD				(5)
#define VIGOR_CAM2_RST				(138)
/* #define RUBY_CAM2_STANDBY			(64) */

/* Flashlight */
#define VIGOR_FLASH_EN             (29)
#define VIGOR_TORCH_EN             (30)

/* Accessory */
#define VIGOR_GPIO_AUD_HP_DET		(31)
#define VIGOR_AUD_HP_CHARM_PWR		(6)
#define VIGOR_AUD_HP_CHARM_SEL		(7)
#define VIGOR_H2W_IO1_CLK		(106)
#define VIGOR_H2W_IO2_DAT		(105)
#define VIGOR_H2W_CABLE_IN1		(123)
#define VIGOR_H2W_CABLE_IN2		(158)

#define VIGOR_GPIO_MHL_RST          (43)
#define VIGOR_GPIO_USB_ID1          (63)

/* SPI */
#define VIGOR_SPI_DO                 (33)
#define VIGOR_SPI_DI                 (34)
#define VIGOR_SPI_CS                 (35)
#define VIGOR_SPI_CLK                (36)

/* LCM */
#define VIGOR_GPIO_LCM_ID0			(49)
#define VIGOR_GPIO_LCM_ID1			(155)

/* LTE */
#define VIGOR_MDM2AP_SYNC           (129)
#define VIGOR_MDM2AP_VDDMIN           (140)

#define VIGOR_MDM2AP_STATUS         (134)
#define VIGOR_AP2MDM_WAKEUP         (135)

#define VIGOR_AP2MDM_STATUS         (136)
#define VIGOR_MDM2AP_WAKEUP          (40)

#define VIGOR_MDM2AP_ERRFATAL       (133)
#define VIGOR_AP2MDM_ERRFATAL        (93)

//#define VIGOR_MDM2AP_VFR             (94)	/* No this pin on Vigor */

#define BOOT_CONFIG_6	(76)
#define RMA_DEBUG	(131)
#define VIGOR_AP2MDM_PMIC_RESET_N_EVM   (131)
#define VIGOR_AP2MDM_KPDPWR_N_EVM        (132)

#define VIGOR_AP2MDM_PMIC_RESET_N   (20)
#define VIGOR_AP2MDM_KPDPWR_N        (4)
//#define VIGOR_AP2PMIC_TMPNI_CKEN    (169)	/* No this pin on Vigor */

/* PMIC */
/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)

/* USB SW */
#define VIGOR_MHLAUD_USBz_SW		PMGPIO(1)
#define VIGOR_WIRELESS_CHG_OK   PMGPIO(14)
/* LCM */
#define VIGOR_LCM_3V			PMGPIO(3)
#define VIGOR_MHL_5V			PMGPIO(13)
/* SD */
#define VIGOR_SD_DETECT_PIN         PMGPIO(4)

/* Audio */
#define VIGOR_AUD_REMO_PRES	PMGPIO(7)
#define VIGOR_AUD_AMP_EN    PMGPIO(18)
#define VIGOR_AUD_REC_EN        PMGPIO(19)
#define VIGOR_AUD_QTR_RESET      PMGPIO(21)
#define VIGOR_AUD_MIC_SEL        PMGPIO(37)

/* TP */
#define VIGOR_TP_RST             PMGPIO(23)

/* LED */
#define VIGOR_GREEN_LED          PMGPIO(24)
#define VIGOR_AMBER_LED          PMGPIO(25)

/* Misc */
#define VIGOR_CHG_STAT		PMGPIO(33)
#define VIGOR_PS_INT            PMGPIO(35)
#define VIGOR_WIFI_BT_SLEEP_CLK  PMGPIO(38)

int __init vigor_init_mmc(void);
void __init vigor_audio_init(void);
int __init vigor_init_keypad(void);
int __init vigor_wifi_init(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_VIGOR_H */
