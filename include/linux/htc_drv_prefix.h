/*
 *   This is the header file for htc driver log prefix.
 *   File Location: /include/linux/htc_drv_prefix.h
 */

#ifndef _HTC_DRIVER_LOG_PREFIX_H
#define _HTC_DRIVER_LOG_PREFIX_H

//Kernel functions
#define DRV_KERNEL          "[KERN]"
#define DRV_SMD             "[SMD]"

//Peripheral functions
#define DRV_USB_HOST        "[USBH] "
#define DRV_USB_FUNC        "[USBF] "
#define DRV_SD              "[SD]"
#define DRV_BATTERY         "[BATT]"
#define DRV_H2W             "[H2W]"
#define DRV_HEADSET         "[HS]"
#define DRV_MISC            "[MISC]"
#define DRV_I2C             "[I2C]"
#define DRV_SPI             "[SPI]"
#define DRV_SERIAL          "[SER]"

//UI functions
#define DRV_KEY             "[KEY]"
#define DRV_TOUCH           "[TP]"
#define DRV_GSENSOR         "[GSNR]"
#define DRV_PSENSOR         "[PSNR]"
#define DRV_LIGHTSENSOR     "[LSNR]"
#define DRV_COMPASS         "[COMP]"
#define DRV_GYRO            "[GYRO]"
#define DRV_LED             "[LED]"
#define DRV_JAGBALL         "[JB]"
#define DRV_OJ              "[OJ]"
#define DRV_VIBRATOR        "[VIB]"

//MM functions
#define DRV_DISPLAY         "[DISP]"
#define DRV_CAMERA          "[CAM]"
#define DRV_VIDEO           "[VID]"
#define DRV_AUDIO           "[AUD]"
#define DRV_BACKLIGHT       "[BKL]"
#define DRV_FLASHLIGHT      "[FLT]"

//Wireless functions
#define DRV_WLAN            "[WLAN]"
#define DRV_WIMAX           "[WMX]"
#define DRV_BT              "[BT]"
#define DRV_GPS             "[GPS]"
#define DRV_CMMB            "[CMMB]"
#define DRV_FM              "[FM]"

/* MSM functions  */
#define DRV_QMI             "[QMI]"
#define DRV_RPC             "[RPC]"
#define DRV_RMNET           "[RMNT]"
#define DRV_SDIOAL          "[SDAL]"

/* RIL and Radio functions */
#define DRV_RIL             "[RIL] "

#endif /* _HTC_DRIVER_LOG_PREFIX_H */

