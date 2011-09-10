#ifndef __ARCH_ARM_BOARD_COMMON_WIMAX_H
#define __ARCH_ARM_BOARD_COMMON_WIMAX_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/gpio.h>

int mmc_wimax_set_status(int on);
int mmc_wimax_get_status(void);
int mmc_wimax_set_cliam_host_status(int on);
int mmc_wimax_get_cliam_host_status(void);
int mmc_wimax_set_netlog_status(int on);
int mmc_wimax_get_netlog_status(void);
int mmc_wimax_set_netlog_withraw_status(int on);
int mmc_wimax_get_netlog_withraw_status(void);
int mmc_wimax_set_sdio_interrupt_log(int on);
int mmc_wimax_get_sdio_interrupt_log(void);
int mmc_wimax_set_packet_filter(int on);
int mmc_wimax_get_packet_filter(void);
int mmc_wimax_set_thp_log(int on);
int mmc_wimax_get_thp_log(void);
int mmc_wimax_set_irq_log(int on);
int mmc_wimax_get_irq_log(void);
int mmc_wimax_set_busclk_pwrsave(int on);
int mmc_wimax_get_busclk_pwrsave(void);
int mmc_wimax_set_sdcclk_highspeed(int on);
int mmc_wimax_get_sdcclk_highspeed(void);
int mmc_wimax_set_sdio_hw_reset(int on);
int mmc_wimax_get_sdio_hw_reset(void);
int mmc_wimax_set_CMD53_timeout_trigger_counter(int counter);
int mmc_wimax_get_CMD53_timeout_trigger_counter(void);
int mmc_wimax_set_gpio_irq_enabled(int on);
int mmc_wimax_get_gpio_irq_enabled(void);

int mmc_wimax_trigger_RD_FIFO_LEVEL_ERROR(int on);
int mmc_wimax_get_RD_FIFO_LEVEL_ERROR(void);

#endif

