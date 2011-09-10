#include "board-common-wimax.h"

static int mmc_wimax_sdio_status = 0;
static int mmc_wimax_netlog_status = 0;
static int mmc_wimax_sdio_interrupt_log_status = 0;
static int mmc_wimax_netlog_withraw_status = 0;
static int mmc_wimax_cliam_host_status = 0;
static int mmc_wimax_busclk_pwrsave = 1; // Default is dynamic CLK OFF
static int mmc_wimax_sdcclk_highspeed = 0; // 1: sdc clk = 49152000  0: sdc clk=25MHz
static int mmc_wimax_CMD53_timeout_trigger_counter = 0;
static int mmc_wimax_thp_log_status = 0;
static int mmc_wimax_irq_log_status = 0;
static int mmc_wimax_sdio_hw_reset = 0; // Default is disable HW reset and counter > 5
static int mmc_wimax_packet_filter = 0;
static int mmc_wimax_is_gpio_irq_enabled = 0;

static int mmc_wimax_RD_FIFO_LEVEL_ERROR = 0;
static int mmc_wimax_FW_freeze_WK_RX = 1;

int mmc_wimax_set_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_status);

int mmc_wimax_get_status(void)
{
	return mmc_wimax_sdio_status;
}
EXPORT_SYMBOL(mmc_wimax_get_status);

int mmc_wimax_set_cliam_host_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_cliam_host_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_cliam_host_status);

int mmc_wimax_get_cliam_host_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_cliam_host_status;
}
EXPORT_SYMBOL(mmc_wimax_get_cliam_host_status);

int mmc_wimax_set_netlog_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_netlog_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_netlog_status);

int mmc_wimax_get_netlog_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_netlog_status;
}
EXPORT_SYMBOL(mmc_wimax_get_netlog_status);

int mmc_wimax_set_netlog_withraw_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_netlog_withraw_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_netlog_withraw_status);

int mmc_wimax_get_netlog_withraw_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_netlog_withraw_status);
	return mmc_wimax_netlog_withraw_status;
}
EXPORT_SYMBOL(mmc_wimax_get_netlog_withraw_status);

int mmc_wimax_set_sdio_interrupt_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_interrupt_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdio_interrupt_log);

int mmc_wimax_get_sdio_interrupt_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_interrupt_log_status);
	return mmc_wimax_sdio_interrupt_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_sdio_interrupt_log);

int mmc_wimax_set_packet_filter(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_packet_filter = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_packet_filter);

int mmc_wimax_get_packet_filter(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_packet_filter);
	return mmc_wimax_packet_filter;
}
EXPORT_SYMBOL(mmc_wimax_get_packet_filter);

int mmc_wimax_set_thp_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_thp_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_thp_log);

int mmc_wimax_get_thp_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_thp_log_status);
	return mmc_wimax_thp_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_thp_log);

int mmc_wimax_set_irq_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);	
    mmc_wimax_irq_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_irq_log);

int mmc_wimax_get_irq_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_irq_log_status);
	return mmc_wimax_irq_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_irq_log);

int mmc_wimax_set_busclk_pwrsave(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_busclk_pwrsave = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_busclk_pwrsave);

int mmc_wimax_get_busclk_pwrsave(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_busclk_pwrsave);
	return mmc_wimax_busclk_pwrsave;
}
EXPORT_SYMBOL(mmc_wimax_get_busclk_pwrsave);

int mmc_wimax_set_sdcclk_highspeed(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on); 
    mmc_wimax_sdcclk_highspeed = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdcclk_highspeed);

int mmc_wimax_get_sdcclk_highspeed(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdcclk_highspeed);
	return mmc_wimax_sdcclk_highspeed;
}
EXPORT_SYMBOL(mmc_wimax_get_sdcclk_highspeed);

int mmc_wimax_set_sdio_hw_reset(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_hw_reset = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdio_hw_reset);

int mmc_wimax_get_sdio_hw_reset(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_hw_reset);
	return mmc_wimax_sdio_hw_reset;
}
EXPORT_SYMBOL(mmc_wimax_get_sdio_hw_reset);

int mmc_wimax_set_CMD53_timeout_trigger_counter(int counter)
{
	printk(KERN_INFO "%s counter:%d\n", __func__, counter);
	mmc_wimax_CMD53_timeout_trigger_counter = counter;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_CMD53_timeout_trigger_counter);

int mmc_wimax_get_CMD53_timeout_trigger_counter(void)
{
	//printk(KERN_INFO "%s counter:%d\n", __func__, mmc_wimax_CMD53_timeout_trigger_counter);
	return mmc_wimax_CMD53_timeout_trigger_counter;
}
EXPORT_SYMBOL(mmc_wimax_get_CMD53_timeout_trigger_counter);

int mmc_wimax_set_gpio_irq_enabled(int on)
{
	// printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_is_gpio_irq_enabled = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_gpio_irq_enabled);

int mmc_wimax_get_gpio_irq_enabled(void)
{
	return mmc_wimax_is_gpio_irq_enabled;
}
EXPORT_SYMBOL(mmc_wimax_get_gpio_irq_enabled);

int mmc_wimax_trigger_RD_FIFO_LEVEL_ERROR(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on); 
  	mmc_wimax_RD_FIFO_LEVEL_ERROR = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_trigger_RD_FIFO_LEVEL_ERROR);

int mmc_wimax_get_RD_FIFO_LEVEL_ERROR(void)
{
  	return mmc_wimax_RD_FIFO_LEVEL_ERROR;
}
EXPORT_SYMBOL(mmc_wimax_get_RD_FIFO_LEVEL_ERROR);
// For FW freeze RX WK
int mmc_wimax_set_wimax_FW_freeze_WK_RX(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_FW_freeze_WK_RX = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_wimax_FW_freeze_WK_RX);

int mmc_wimax_get_wimax_FW_freeze_WK_RX(void)
{
	return mmc_wimax_FW_freeze_WK_RX;
}
EXPORT_SYMBOL(mmc_wimax_get_wimax_FW_freeze_WK_RX);
