/* linux/arch/arm/mach-msm/board-rider.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_RIDER_H
#define __ARCH_ARM_MACH_MSM_BOARD_RIDER_H

#include <mach/board.h>

#define RIDER_PROJECT_NAME	"rider"


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
#define RIDER_GPIO_KEY_CAM_STEP1   (123)
#define RIDER_GPIO_KEY_VOL_DOWN    (103)
#define RIDER_GPIO_KEY_VOL_UP      (104)
#define RIDER_GPIO_KEY_CAM_STEP2   (115)
#define RIDER_GPIO_KEY_POWER       (125)

#define RIDER_GPIO_CAPTURE_MODE_KEY       (64)
#define RIDER_GPIO_VIDEO_MODE_KEY       (68)

/* Battery */
#define RIDER_GPIO_MBAT_IN		   (61)
#define RIDER_GPIO_CHG_INT		   (126)

/* Wifi */
#define RIDER_GPIO_WIFI_IRQ              (46)
#define RIDER_GPIO_WIFI_SHUTDOWN_N       (96)

/* WiMax */
#define RIDER_GPIO_WIMAX_UART_SIN        (41)
#define RIDER_GPIO_WIMAX_UART_SOUT       (42)
#define RIDER_GPIO_V_WIMAX_1V2_RF_EN     (43)
#define RIDER_GPIO_WIMAX_EXT_RST         (49)
#define RIDER_GPIO_V_WIMAX_DVDD_EN       (94)
#define RIDER_GPIO_V_WIMAX_PVDD_EN       (105)
#define RIDER_GPIO_WIMAX_SDIO_D0         (143)
#define RIDER_GPIO_WIMAX_SDIO_D1         (144)
#define RIDER_GPIO_WIMAX_SDIO_D2         (145)
#define RIDER_GPIO_WIMAX_SDIO_D3         (146)
#define RIDER_GPIO_WIMAX_SDIO_CMD        (151)
#define RIDER_GPIO_WIMAX_SDIO_CLK_CPU    (152)
#define RIDER_GPIO_CPU_WIMAX_SW          (156)
#define RIDER_GPIO_CPU_WIMAX_UART_EN     (157)

/* Sensors */
#define RIDER_SENSOR_I2C_SDA		(72)
#define RIDER_SENSOR_I2C_SCL		(73)
#define RIDER_GYRO_INT               (127)

/* General */
#define RIDER_GENERAL_I2C_SDA		(59)
#define RIDER_GENERAL_I2C_SCL		(60)

/* Microp */

/* TP */
#define RIDER_TP_I2C_SDA           (51)
#define RIDER_TP_I2C_SCL           (52)
#define RIDER_TP_ATT_N             (65)
#define RIDER_TP_ATT_N_XB          (57)

/* LCD */
#define GPIO_LCM_ID	50
#define GPIO_LCM_RST_N	66

/* Audio */
#define RIDER_AUD_CODEC_RST        (67)
#define RIDER_AUD_CDC_LDO_SEL      (116)
/* BT */
#define RIDER_GPIO_BT_HOST_WAKE      (45)
#define RIDER_GPIO_BT_UART1_TX       (53)
#define RIDER_GPIO_BT_UART1_RX       (54)
#define RIDER_GPIO_BT_UART1_CTS      (55)
#define RIDER_GPIO_BT_UART1_RTS      (56)
#define RIDER_GPIO_BT_SHUTDOWN_N     (100)
#define RIDER_GPIO_BT_CHIP_WAKE      (130)
#define RIDER_GPIO_BT_RESET_N        (142)

/* USB */
#define RIDER_GPIO_USB_ID        (63)
#define RIDER_GPIO_MHL_RESET        (70)
#define RIDER_GPIO_MHL_USB_EN         (139)
#define RIDER_GPIO_MHL_USB_SW        (99)

/* Camera */
#define RIDER_CAM_I2C_SDA           (47)
#define RIDER_CAM_I2C_SCL           (48)
#define RIDER_CAM_ID           (135)


/* Flashlight */
#define RIDER_FLASH_EN             (29)
#define RIDER_TORCH_EN             (30)

/* Accessory */
#define RIDER_GPIO_AUD_HP_DET        (31)

/* SPI */
#define RIDER_SPI_DO                 (33)
#define RIDER_SPI_DI                 (34)
#define RIDER_SPI_CS                 (35)
#define RIDER_SPI_CLK                (36)

/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define RIDER_VOL_UP             (104)
#define RIDER_VOL_DN             (103)
#define RIDER_AUD_HANDSET_ENO    PMGPIO(18)
#define RIDER_AUD_SPK_ENO        PMGPIO(19)
#define RIDER_PS_VOUT            PMGPIO(22)
#define RIDER_GREEN_LED          PMGPIO(24)
#define RIDER_AMBER_LED          PMGPIO(25)
#define RIDER_AUD_MIC_SEL        PMGPIO(14)
#define RIDER_WIFI_BT_SLEEP_CLK  PMGPIO(38)
#define RIDER_WIMAX_HOST_WAKEUP  PMGPIO(17)
#define RIDER_TP_RST             PMGPIO(23)
#define RIDER_TORCH_SET1         PMGPIO(40)
#define RIDER_TORCH_SET2         PMGPIO(31)
#define RIDER_CHG_STAT		 PMGPIO(33)
#define RIDER_AUD_REMO_EN        PMGPIO(15)
#define RIDER_AUD_REMO_PRES      PMGPIO(37)

int __init rider_init_mmc(void);
void __init rider_audio_init(void);
int __init rider_init_keypad(void);
int __init rider_wifi_init(void);
int __init rider_init_panel(struct resource *res, size_t size);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_RIDER_H */
