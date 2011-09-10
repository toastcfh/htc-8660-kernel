/* drivers/i2c/chips/sii9234.c - sii9234 optical sensors driver
 *
 * Copyright (C) 2010 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
//********************************************************************
//  Inculde
//********************************************************************
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <mach/mhl.h>
#include <mach/debug_display.h>
#include "sii9234.h"
#include "TPI.h"
#include <mach/board.h>

//********************************************************************
//  Define & Macro
//********************************************************************
#define SII9234_I2C_RETRY_COUNT 2
//********************************************************************
//  Type Definitions
//********************************************************************
typedef struct
{
	struct i2c_client *i2c_client;
	struct workqueue_struct *wq;
	struct wake_lock wake_lock;
	struct work_struct work;
	int (*pwrCtrl)(int); /* power to the chip */
	void (*mhl_usb_switch)(int);
	void (*mhl_1v2_power)(bool enable);
	struct delayed_work init_delay_work;
	struct delayed_work init_complete_work;
	struct delayed_work mhl_on_delay_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int reset_pin;
	int intr_pin;
	int ci2ca_pin;
	int irq;
	bool isMHL;
	struct work_struct mhl_notifier_work;
}T_MHL_SII9234_INFO;
int TPI_SLAVE_ADDR;
int HDMI_SLAVE_ADDR;
int CBUS_SLAVE_ADDR;
//********************************************************************
//  Variable & Extern variable
//********************************************************************
static T_MHL_SII9234_INFO *sii9234_info_ptr;
//********************************************************************
//  Prototype & Extern function
//********************************************************************
static void sii9234_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sii9234_irq_work, sii9234_irq_do_work);

static DEFINE_MUTEX(mhl_early_suspend_sem);
bool g_bEnterEarlySuspend = false;
static bool g_bGotUsbBus = false;
static bool g_bNeedSimulateCableOut = false;
static bool g_bInitCompleted = false;

#define MHL_RCP_KEYEVENT

#ifdef MHL_RCP_KEYEVENT
struct input_dev *input_dev;
#endif
static struct platform_device *mhl_dev; /* Device structure */

//********************************************************************
//	Functions
//********************************************************************
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
static DEFINE_MUTEX(mhl_notify_sem);
void update_mhl_status(bool isMHL)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	pInfo->isMHL = isMHL;

	queue_work(pInfo->wq, &pInfo->mhl_notifier_work);
}
static void send_mhl_connect_notify(struct work_struct *w)
{
	static struct t_mhl_status_notifier *mhl_notifier;
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	if (!pInfo)
		return;

	printk(KERN_INFO "%s: %d\n", __func__, pInfo->isMHL);
	mutex_lock(&mhl_notify_sem);
	list_for_each_entry(mhl_notifier,
		&g_lh_mhl_detect_notifier_list,
		mhl_notifier_link) {
			if (mhl_notifier->func != NULL)
				mhl_notifier->func(pInfo->isMHL, false);
		}
	mutex_unlock(&mhl_notify_sem);
}

int mhl_detect_register_notifier(struct t_mhl_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&mhl_notify_sem);
	list_add(&notifier->mhl_notifier_link,
		&g_lh_mhl_detect_notifier_list);
	mutex_unlock(&mhl_notify_sem);
	return 0;
}
#endif
int sii9234_I2C_RxData(uint8_t deviceID, char *rxData, uint32_t length)
{
	uint8_t loop_i;
	uint8_t slave_addr = deviceID >> 1;
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = slave_addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	sii9234_info_ptr->i2c_client->addr = slave_addr;
	for (loop_i = 0; loop_i < SII9234_I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(sii9234_info_ptr->i2c_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= SII9234_I2C_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n",
			__func__, SII9234_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int sii9234_I2C_TxData(uint8_t deviceID, char *txData, uint32_t length)
{
	uint8_t loop_i;
	uint8_t slave_addr = deviceID >> 1;
	struct i2c_msg msg[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

//	M_MHL_SEND_DEBUG("%s: sii9234_info_ptr->i2c_client->addr = 0x%x, deviceID = 0x%x\n", __func__, sii9234_info_ptr->i2c_client->addr, slave_addr);
	sii9234_info_ptr->i2c_client->addr = slave_addr;
	for (loop_i = 0; loop_i < SII9234_I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(sii9234_info_ptr->i2c_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= SII9234_I2C_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n",
			__func__, SII9234_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int sii9234_get_intr_status(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	return gpio_get_value(pInfo->intr_pin);
}

void sii9234_reset(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	gpio_set_value(pInfo->reset_pin, 0);
	mdelay(2);
	gpio_set_value(pInfo->reset_pin, 1);
}

int sii9234_get_ci2ca(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	return pInfo->ci2ca_pin;
}

static void sii9234_irq_do_work(struct work_struct *work)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	mutex_lock(&mhl_early_suspend_sem);
	if( !g_bEnterEarlySuspend ){
		uint8_t		event;
		uint8_t		eventParameter;

		M_MHL_SEND_DEBUG("MHL ISR\r\n");

		SiiMhlTxGetEvents(&event, &eventParameter);
		ProcessRcp(event, eventParameter);
	}
	mutex_unlock(&mhl_early_suspend_sem);

	enable_irq(pInfo->irq);
}

static irqreturn_t sii9234_irq_handler(int irq, void *data)
{
	T_MHL_SII9234_INFO *pInfo = data;

	disable_irq_nosync(pInfo->irq);
	if(pInfo->wq)
		queue_work(pInfo->wq, &sii9234_irq_work);
	return IRQ_HANDLED;
}

void sii9234_send_keyevent(uint32_t key, uint32_t type)
{
#ifdef MHL_RCP_KEYEVENT
        printk ("CBUS key_event: %d\n", key);
	if (type == 0) {
		input_report_key(input_dev, key, 1);
		input_report_key(input_dev, key, 0);
		input_sync(input_dev);
	}
#endif
}

#ifdef MHL_RCP_KEYEVENT
/* Sysfs method to input simulated coordinates */
static ssize_t write_keyevent(struct device *dev,
                              struct device_attribute *attr,
                              const char *buffer, size_t count)
{
        int key;

        /* parsing input data */
        sscanf(buffer, "%d", &key);


	printk ("key_event: %d\n", key);

	/* Report key event */
	switch(key) {
	case 0:
                input_report_key(input_dev, KEY_HOME, 1);
                input_report_key(input_dev, KEY_HOME, 0);
               break;
	case 1:
                input_report_key(input_dev, KEY_UP, 1);
                input_report_key(input_dev, KEY_UP, 0);
               break;
	case 2:
                input_report_key(input_dev, KEY_DOWN, 1);
                input_report_key(input_dev, KEY_DOWN, 0);
               break;
	case 3:
                input_report_key(input_dev, KEY_LEFT, 1);
                input_report_key(input_dev, KEY_LEFT, 0);
               break;
	case 4:
                input_report_key(input_dev, KEY_RIGHT, 1);
                input_report_key(input_dev, KEY_RIGHT, 0);
               break;
	case 5:
                input_report_key(input_dev, KEY_ENTER, 1);
                input_report_key(input_dev, KEY_ENTER, 0);
		break;
	case 6:
                input_report_key(input_dev, KEY_SELECT, 1);
                input_report_key(input_dev, KEY_SELECT, 0);
		break;
	default:
		input_report_key(input_dev, KEY_OK, 1);
		input_report_key(input_dev, KEY_OK, 0);
		break;
	}
        input_sync(input_dev);

        return count;

}

/* Attach the sysfs write method */
static DEVICE_ATTR(rcp_event, 0644, NULL, write_keyevent);
#endif

void sii9234_mhl_device_wakeup(void)
{
	int err;
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	PR_DISP_INFO("sii9234_mhl_device_wakeup()\n");

	if( !g_bInitCompleted ){
		PR_DISP_INFO("MHL inserted before HDMI related function was ready! Wait more 5 sec...\n");
		queue_delayed_work(pInfo->wq, &pInfo->init_delay_work, HZ*5);
		return;
	}

	/* MHL_USB_SW for Verdi  0: switch to MHL;  others projects, 1 : switch to MHL */
	if (pInfo->mhl_usb_switch)
		pInfo->mhl_usb_switch(1);

	gpio_set_value(pInfo->reset_pin, 1); /* Reset High */

	if (pInfo->mhl_1v2_power)
		pInfo->mhl_1v2_power(1);

	err = TPI_Init();
	if (err != 1)
		PR_DISP_INFO("TPI can't init\n");
}

static void init_delay_handler(struct work_struct *w)
{
	PR_DISP_INFO("init_delay_handler()\n");

	// Notify the USB driver to re-detect...
	update_mhl_status(false);
}

static void init_complete_handler(struct work_struct *w)
{
	PR_DISP_INFO("init_complete_handler()\n");

	g_bInitCompleted = true;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int sii9234_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if (Status_Query() != POWER_STATE_D3)
		SiiMhlTxDrvTmdsControl(false);
	return 0;
}

#if 0
static int sii9234_resume(struct i2c_client *client)
{
	if (Status_Query() != POWER_STATE_D3)
		SiiMhlTxDrvTmdsControl(true);
	return 0;
}
#endif

void sii9234_change_usb_owner(bool bMHL)
{
	PR_DISP_DEBUG("%s(%d)\n", __func__, bMHL );

	g_bGotUsbBus = bMHL;
}

static void sii9234_early_suspend(struct early_suspend *h)
{
	T_MHL_SII9234_INFO *pInfo;
	pInfo = container_of(h, T_MHL_SII9234_INFO, early_suspend);

	PR_DISP_DEBUG("%s(isMHL=%d, g_bGotUsbBus=%d)\n", __func__, pInfo->isMHL, g_bGotUsbBus );

	mutex_lock(&mhl_early_suspend_sem);
	// Enter the early suspend state...
	g_bEnterEarlySuspend = true;

	// Cancel the previous TMDS on delay work...
	cancel_delayed_work(&pInfo->mhl_on_delay_work);
	if( pInfo->isMHL ){
		// Turn-off the TMDS output...
		sii9234_suspend(pInfo->i2c_client, PMSG_SUSPEND);
	}
	// Check already power on or not?
	if( g_bGotUsbBus ){
		// Power off the chip...
		if (pInfo->mhl_1v2_power)
			pInfo->mhl_1v2_power(0);
		// Got the USB bus owner, but didn't have time to make sure the connected device was MHL client?
		g_bNeedSimulateCableOut = true;
	}
	mutex_unlock(&mhl_early_suspend_sem);
}

static void sii9234_late_resume(struct early_suspend *h)
{
	T_MHL_SII9234_INFO *pInfo;
	pInfo = container_of(h, T_MHL_SII9234_INFO, early_suspend);

	PR_DISP_DEBUG("sii9234_early_suspend()\n");

	mutex_lock(&mhl_early_suspend_sem);
	queue_delayed_work(pInfo->wq, &pInfo->mhl_on_delay_work, HZ);

	// Leave the early suspend state...
	g_bEnterEarlySuspend = false;
	mutex_unlock(&mhl_early_suspend_sem);
}

static void mhl_on_delay_handler(struct work_struct *w)
{
	PR_DISP_DEBUG("%s(%d, %d)\n", __func__, g_bEnterEarlySuspend, g_bNeedSimulateCableOut );

	mutex_lock(&mhl_early_suspend_sem);
	if( !g_bEnterEarlySuspend && g_bNeedSimulateCableOut ){
		// Simulate the cable out to reinitiate the MHL...
		update_mhl_status(false);
		// Reset the variable...
		g_bNeedSimulateCableOut = false;
		g_bGotUsbBus = false;
	}
	mutex_unlock(&mhl_early_suspend_sem);
}
#endif

static int sii9234_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = E_MHL_OK;
//	int timeoutCnt = 10;
	bool rv = TRUE;
	T_MHL_SII9234_INFO *pInfo;
	T_MHL_PLATFORM_DATA *pdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	pInfo = kzalloc(sizeof(T_MHL_SII9234_INFO), GFP_KERNEL);
	if (!pInfo)
	{
		pr_err("%s: alloc memory error!!\n", __func__);
		return -ENOMEM;
	}
	pInfo->i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata)
	{
		pr_err("%s: Assign platform_data error!!\n", __func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	pInfo->irq = client->irq;
	i2c_set_clientdata(client, pInfo);
	pInfo->reset_pin = pdata->gpio_reset;
	pInfo->intr_pin = pdata->gpio_intr;
	pInfo->ci2ca_pin = pdata->ci2ca;
	if (pInfo->ci2ca_pin) {
		TPI_SLAVE_ADDR =  0x76;//0x72
		HDMI_SLAVE_ADDR = 0x96;//0x92
		CBUS_SLAVE_ADDR = 0xCC;//0xC8
	} else {
		TPI_SLAVE_ADDR =  0x72;
		HDMI_SLAVE_ADDR = 0x92;
		CBUS_SLAVE_ADDR = 0xC8;
	}
	pInfo->pwrCtrl = pdata->power;
	pInfo->mhl_usb_switch = pdata->mhl_usb_switch;
	pInfo->mhl_1v2_power = pdata->mhl_1v2_power;
	sii9234_info_ptr = pInfo;
	/* Power ON */
	if (pInfo->pwrCtrl)
	{
		pInfo->pwrCtrl(1);
	}
	/* Pin Config */
	gpio_request(pInfo->reset_pin, "mhl_sii9234_gpio_reset");
	gpio_direction_output(pInfo->reset_pin, 0);
	gpio_request(pInfo->intr_pin, "mhl_sii9234_gpio_intr");
	gpio_direction_input(pInfo->intr_pin);
	rv = TPI_Init();
	if (rv != TRUE)
	{
		pr_err("%s: can't init\n", __func__);
		ret = -ENOMEM;
		goto err_init;
	}

	INIT_DELAYED_WORK(&pInfo->init_delay_work, init_delay_handler);
	INIT_DELAYED_WORK(&pInfo->init_complete_work, init_complete_handler);
	INIT_DELAYED_WORK(&pInfo->mhl_on_delay_work, mhl_on_delay_handler);

	/* Enter D3 mode */
//	do
//	{
//		TPI_Poll();
//	} while ((--timeoutCnt) && (Status_Query() != TX_POWER_STATE_D3));
	ret = request_irq(pInfo->irq, sii9234_irq_handler, IRQF_TRIGGER_LOW, "mhl_sii9234_evt", pInfo);
	if (ret < 0)
	{
		pr_err("%s: request_irq(%d) failed for gpio %d (%d)\n", __func__, pInfo->irq, pInfo->intr_pin, ret);
		ret = -EIO;
		goto err_request_intr_pin;
	}
	ret = set_irq_wake(pInfo->irq, 1);
	if (ret < 0)
	{
		pr_err("%s: (%d) set_irq_wake failed for gpio %d (%d)\n", __func__, pInfo->irq, pInfo->intr_pin, ret);
		ret = -EIO;
		goto err_set_irq_wake;
	}
	pInfo->wq = create_workqueue("mhl_sii9234_wq");
	if (!pInfo->wq)
	{
		pr_err("%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_workqueue;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	pInfo->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	pInfo->early_suspend.suspend = sii9234_early_suspend;
	pInfo->early_suspend.resume = sii9234_late_resume;
	register_early_suspend(&pInfo->early_suspend);
#endif
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
	INIT_WORK(&pInfo->mhl_notifier_work, send_mhl_connect_notify);
#endif
	/* Register a platform device */
	mhl_dev = platform_device_register_simple("mhl", -1, NULL, 0);
	if (IS_ERR(mhl_dev)){
	printk ("mhl init: error\n");
		return PTR_ERR(mhl_dev);
	}
	/* Create a sysfs node to read simulated coordinates */

#ifdef MHL_RCP_KEYEVENT
	ret = device_create_file(&mhl_dev->dev, &dev_attr_rcp_event);

	input_dev = input_allocate_device();
	/* indicate that we generate key events */
	set_bit(EV_KEY, input_dev->evbit);
	/* indicate that we generate *any* key event */
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_ENTER, input_dev->keybit);
	set_bit(KEY_LEFT, input_dev->keybit);
	set_bit(KEY_UP, input_dev->keybit);
	set_bit(KEY_DOWN, input_dev->keybit);
	set_bit(KEY_RIGHT, input_dev->keybit);

	input_dev->name = "rcp_events";

	ret = input_register_device(input_dev);
	if (ret < 0)
		pr_err("MHL: can't register input devce\n");
#endif

	// Initiate a 10 sec delay which will change the "g_bInitCompleted" be true after it...
	queue_delayed_work(pInfo->wq, &pInfo->init_complete_work, HZ*10);

	M_MHL_SEND_DEBUG("%s: Probe success!\n", __func__);
	return ret;

err_create_workqueue:
err_set_irq_wake:
	gpio_free(pInfo->reset_pin);
	gpio_free(pInfo->intr_pin);
err_init:
err_request_intr_pin:
err_platform_data_null:
	kfree(pInfo);
err_check_functionality_failed:
	return ret;
}

static int sii9234_remove(struct i2c_client *client)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	gpio_free(pInfo->reset_pin);
	gpio_free(pInfo->intr_pin);
#ifndef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&pInfo->early_suspend);
#endif
	destroy_workqueue(pInfo->wq);
	kfree(pInfo);
	return E_MHL_OK;
}

static const struct i2c_device_id sii9234_i2c_id[] = {
	{MHL_SII9234_I2C_NAME, 0},
	{}
};

static struct i2c_driver sii9234_driver = {
	.id_table = sii9234_i2c_id,
	.probe = sii9234_probe,
	.remove = sii9234_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = sii9234_suspend,
	.resume = sii9234_resume,
#endif
	.driver = {
		.name = MHL_SII9234_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init sii9234_init(void)
{
	return i2c_add_driver(&sii9234_driver);
}

static void __exit sii9234_exit(void)
{
	i2c_del_driver(&sii9234_driver);
}

module_init(sii9234_init);
module_exit(sii9234_exit);

MODULE_DESCRIPTION("SiI9234 Driver");
MODULE_LICENSE("GPL");
