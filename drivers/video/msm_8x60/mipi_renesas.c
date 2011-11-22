/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_renesas.h"

extern int mipi_status;
#define DEFAULT_BRIGHTNESS 83
extern int bl_level_prevset;
extern struct mutex cmdlock;

static struct msm_panel_common_pdata *mipi_renesas_pdata;

static struct dsi_buf renesas_tx_buf;
static struct dsi_buf renesas_rx_buf;

/* renesas blue panel */
static char column_address[5] = {0x2A, 0x00, 0x00, 0x01, 0xDF}; /* DTYPE_DCS_LWRITE */
static char set_page_addr[5] = {0x2B, 0x00, 0x00, 0x03, 0x1F}; /* DTYPE_DCS_LWRITE */
static char address_mode[2] = {0x36, 0x00}; /* DTYPE_DCS_WRITE1 */
static char pixel_format[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */
static char unlock_command[2] = {0xB0, 0x04}; /* DTYPE_GEN_WRITE2 */
static char panel_driving[4] = {0xC1, 0x42, 0x31, 0x04}; /* DTYPE_GEN_LWRITE */
//static char lock_command[2] = {0xB0, 0x03}; /* DTYPE_GEN_WRITE2 */
static char memory_start[2] = {0x2C, 0x00}; /* DTYPE_DCS_WRITE */

//static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char enable_te[2] = {0x35, 0x00};/* DTYPE_DCS_WRITE1 */
static char test_reg[3] = {0x44, 0x02, 0xCF};/* DTYPE_DCS_WRITE1 */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
static char led_pwm[5] = {0xB9, 0x00, 0x7F, 0x04, 0x08}; /* DTYPE_DCS_WRITE */
static char gama_a[] = {	/* DTYPE_GEN_LWRITE */
	0xC8, 0x2D, 0x2F, 0x31, 0x36, 0x3E, 0x51, 0x36,
	0x23, 0x16, 0x0B, 0x02, 0x01, 0x2D, 0x2F, 0x31,
	0x36, 0x3E, 0x51, 0x36, 0x23, 0x16, 0x0B, 0x02,
	0x01};

static char gama_b[] = {	/* DTYPE_GEN_LWRITE */
	0xC9, 0x00, 0x0F, 0x18, 0x25, 0x33, 0x4D, 0x38,
	0x25, 0x18, 0x11, 0x02, 0x01, 0x00, 0x0F, 0x18,
	0x25, 0x33, 0x4D, 0x38, 0x25, 0x18, 0x11, 0x02,
	0x01};

static char gama_c[] = {	/* DTYPE_GEN_LWRITE */
	0xCA, 0x27, 0x2A, 0x2E, 0x34, 0x3C, 0x51, 0x36,
	0x24, 0x16, 0x0C, 0x02, 0x01, 0x27, 0x2A, 0x2E,
	0x34, 0x3C, 0x51, 0x36, 0x24, 0x16, 0x0C, 0x02,
	0x01};

//static char vreg_set[] = {0xD5, 0x14, 0x14}; /* DTYPE_GEN_LWRITE */

static struct dsi_cmd_desc renesas_video_on_cmds[] = {
/*	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(column_address), column_address},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(set_page_addr), set_page_addr},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(address_mode), address_mode},
*/		
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(pixel_format), pixel_format},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 170,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 10,
		sizeof(unlock_command), unlock_command},
/*	{DTYPE_GEN_LWRITE, 1, 0, 0, 10,
		sizeof(panel_driving), panel_driving},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 10,
		sizeof(lock_command), lock_command},*/
	{DTYPE_DCS_WRITE, 1, 0, 0, 200,
		sizeof(display_on), display_on},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 200,
		sizeof(memory_start), memory_start},		
		
};

static struct dsi_cmd_desc renesas_cmd_on_cmds[] = {
	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(column_address), column_address},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(set_page_addr), set_page_addr},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(address_mode), address_mode},
		
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(pixel_format), pixel_format},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 1,
		sizeof(unlock_command), unlock_command},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(panel_driving), panel_driving},
/*	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vreg_set), vreg_set},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 10,
		sizeof(lock_command), lock_command},
*/
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(memory_start), memory_start},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(test_reg), test_reg},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
			sizeof(display_off), display_off},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(gama_a), gama_a},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(gama_b), gama_b},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(gama_c), gama_c},
};

static struct dsi_cmd_desc renesas_display_off_cmds[] = {
		{DTYPE_DCS_WRITE, 1, 0, 0, 1,
			sizeof(display_off), display_off},
		{DTYPE_DCS_WRITE, 1, 0, 0, 90,
			sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc renesas_cmd_backlight_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(led_pwm), led_pwm},
};

static struct dsi_cmd_desc renesas_cmd_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_on), display_on},
};

static int mipi_renesas_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	static int init;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi  = &mfd->panel_info.mipi;

	mutex_lock(&cmdlock);
	if (init == 0) {
		init = 1;
		goto end;
	} else {
		if (mipi->mode == DSI_VIDEO_MODE) {
			mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_video_on_cmds,
				ARRAY_SIZE(renesas_video_on_cmds));
		} else {
			mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_cmd_on_cmds,
				ARRAY_SIZE(renesas_cmd_on_cmds));
		}
	}
end:
	mutex_unlock(&cmdlock);
	return 0;
}

static int mipi_renesas_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mutex_lock(&cmdlock);
	mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_display_off_cmds,
	ARRAY_SIZE(renesas_display_off_cmds));
	mutex_unlock(&cmdlock);
	return 0;
}

static int mipi_dsi_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	static int bl_level_old;

	mutex_lock(&cmdlock);
	mipi  = &mfd->panel_info.mipi;
	pr_debug("%s+:bl=%d status=%d\n", __func__, mfd->bl_level, mipi_status);

	if (mipi_status == 0)
		goto end;

	if (mipi_renesas_pdata && mipi_renesas_pdata->shrink_pwm && mfd->bl_level!=0)
		led_pwm[2] = mipi_renesas_pdata->shrink_pwm(mfd->bl_level);
	else
		led_pwm[2] = (unsigned char)(mfd->bl_level);

	if (mfd->bl_level == 0 || board_mfg_mode() == 4 || board_mfg_mode() == 5) {
		led_pwm[2] = 0;
	}

	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_cmd_backlight_cmds,
	ARRAY_SIZE(renesas_cmd_backlight_cmds));
	bl_level_prevset = bl_level_old = mfd->bl_level;
end:
	mutex_unlock(&cmdlock);
	return 0;
}

static void mipi_renesas_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;

	bl_level = mfd->bl_level;
	pr_debug("%s+ bl_level=%d\n", __func__, mfd->bl_level);
	mipi_dsi_set_backlight(mfd);
}

static void mipi_renesas_display_on(struct msm_fb_data_type *mfd)
{
	pr_info("%s+\n", __func__);
	mutex_lock(&cmdlock);
	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_cmd_display_on_cmds,
	ARRAY_SIZE(renesas_cmd_display_on_cmds));
	mutex_unlock(&cmdlock);
}

static void mipi_renesas_bkl_switch(struct msm_fb_data_type *mfd, bool on)
{
	unsigned int val = 0;

	pr_debug("%s+:on/off=%d\n", __func__, on);
	if (on) {
		mipi_status = 1;
		val = mfd->bl_level;
		if (val == 0) {
			if (bl_level_prevset != 1) {
				val = bl_level_prevset;
				mfd->bl_level = val;
			} else {
				val = DEFAULT_BRIGHTNESS;
				mfd->bl_level = val;
			}
		}
		mipi_dsi_set_backlight(mfd);
	} else {
		mipi_status = 0;
		mfd->bl_level = 0;
		mipi_dsi_set_backlight(mfd);
	}
}

static void mipi_renesas_bkl_ctrl(bool on)
{
	pr_debug("%s+:on/off=%d\n", __func__, on);
	return;
}

static int mipi_renesas_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_renesas_pdata = pdev->dev.platform_data;
		mutex_init(&cmdlock);
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_renesas_lcd_probe,
	.driver = {
		.name   = "mipi_renesas",
	},
};

static struct msm_fb_panel_data renesas_panel_data = {
	.on		= mipi_renesas_lcd_on,
	.off		= mipi_renesas_lcd_off,
	.set_backlight  = mipi_renesas_set_backlight,
	.display_on  	= mipi_renesas_display_on,
	.bklswitch      = mipi_renesas_bkl_switch,
	.bklctrl        = mipi_renesas_bkl_ctrl,
};

static int ch_used[3];

int mipi_renesas_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_renesas", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	renesas_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &renesas_panel_data,
		sizeof(renesas_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_renesas_lcd_init(void)
{
	mipi_dsi_buf_alloc(&renesas_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&renesas_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_renesas_lcd_init);
