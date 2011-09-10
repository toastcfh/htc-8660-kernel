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
#include <mach/panel_id.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_himax.h"


//#define MIPI_READ_DISPLAY_ID	1
// -----------------------------------------------------------------------------
//                             Constant value define
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
//                         External routine declaration
// -----------------------------------------------------------------------------
extern int mipi_status;

static struct msm_panel_common_pdata *mipi_himax_pdata;

static struct dsi_buf himax_tx_buf;
static struct dsi_buf himax_rx_buf;

static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */

#ifdef THREE_LANES
static char set_threelane[2] = {0xBA, 0x12}; /* DTYPE_DCS_WRITE-1 */
#else
static char set_twolane[2] = {0xBA, 0x11}; /* DTYPE_DCS_WRITE-1 */
#endif
static char max_pkt_size[2] = {0x03, 0x00};
static char himax_password[4] = {0xB9, 0xFF, 0x83, 0x92}; /* DTYPE_DCS_LWRITE */
static char display_mode_video[2] = {0xC2, 0x03}; /* DTYPE_DCS_WRITE-1 */
static char display_mode_cmd[2] = {0xC2, 0x00}; /* DTYPE_DCS_WRITE */

static char enable_te[2] = {0x35, 0x00};/* DTYPE_DCS_WRITE1 */
static char himax_cc[2] = {0xCC, 0x08}; /* DTYPE_DCS_WRITE-1 */
//static char himax_mx_cmd[2] = {0x36, 0x02}; /* DTYPE_DCS_WRITE-1 */

static char himax_b2[13] = {0xB2, 0x0F, 0xC8, 0x04, 0x0C, 0x04, 0xF4, 0x00,
							0xFF, 0x04, 0x0C, 0x04, 0x20}; /* DTYPE_DCS_LWRITE */ /*Set display related register */
static char himax_b4[21] = {0xB4, 0x12, 0x00, 0x05, 0x00, 0x9A, 0x05, 0x06,
							0x95, 0x00, 0x01, 0x06, 0x00, 0x08, 0x08, 0x00,
							0x1D, 0x08, 0x08, 0x08, 0x00}; /* DTYPE_DCS_LWRITE */ /* MPU/Command CYC */

static char himax_d8[21] = {0xD8, 0x12, 0x00, 0x05, 0x00, 0x9A, 0x05, 0x06,
							0x95, 0x00, 0x01, 0x06, 0x00, 0x08, 0x08, 0x00,
							0x1D, 0x08, 0x08, 0x08, 0x00}; /* DTYPE_DCS_LWRITE */ /* MPU/Command CYC */
static char himax_d4[2] = {0xD4, 0x0C}; /* DTYPE_DCS_WRITE-1 */

static char himax_b1[14] = {0xB1, 0x7C, 0x00, 0x44, 0x76, 0x00, 0x12, 0x12,
							0x2A, 0x25, 0x1E, 0x1E, 0x42, 0x74}; /* DTYPE_DCS_LWRITE */ /* Set Power */
//static char himax_b6[2] = {0xB6, 0x21}; /* DTYPE_DCS_WRITE-1 */

/* Gamma */
static char himax_e0[35] = {0xE0, 0x00, 0x1D, 0x27, 0x3D, 0x3C, 0x3F, 0x38,
							0x4F, 0x07, 0x0E, 0x0E, 0x10, 0x17, 0x15, 0x15,
							0x16, 0x1F, 0x00, 0x1D, 0x27, 0x3D, 0x3C, 0x3F,
							0x38, 0x4F, 0x07, 0x0E, 0x0E, 0x10, 0x17, 0x15,
							0x15, 0x16, 0x1F};
static char himax_e1[35] = {0xE1, 0x25, 0x30, 0x33, 0x3B, 0x3A, 0x3F, 0x3B,
							0x50, 0x06, 0x0E, 0x0E, 0x10, 0x14, 0x11, 0x13,
							0x15, 0x1E, 0x25, 0x30, 0x33, 0x3B, 0x3A, 0x3F,
							0x3B, 0x50, 0x06, 0x0E, 0x0E, 0x10, 0x14, 0x11,
							0x13, 0x15, 0x1E};
static char himax_e2[35] = {0xE2, 0x2E, 0x34, 0x33, 0x3A, 0x39, 0x3F, 0x39,
							0x4E, 0x07, 0x0D, 0x0E, 0x10, 0x15, 0x11, 0x15,
							0x15, 0x1E, 0x2E, 0x34, 0x33, 0x3A, 0x39, 0x3F,
							0x39, 0x4E, 0x07, 0x0D, 0x0E, 0x10, 0x15, 0x11,
							0x15, 0x15, 0x1E};

static char led_pwm1[] = {0x51, 0xFF};
static char led_pwm2[] = {0x53, 0x24};
static char led_pwm3[] = {0x55, 0x00}; /* set CABC mode , 0x00=100%, 0x01=UI mode, 0x02= still mode, 0x03= Moving mode*/

static struct dsi_cmd_desc himax_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 250,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(himax_password), himax_password},
#ifdef THREE_LANES
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_threelane), set_threelane},
#else
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_twolane), set_twolane},
#endif
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(display_mode_video), display_mode_video},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(display_on), display_on},
};
#if 1
static struct dsi_cmd_desc himax_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 250,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(himax_password), himax_password},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(himax_d4), himax_d4},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(set_twolane), set_twolane},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pkt_size), max_pkt_size},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(display_mode_cmd), display_mode_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(himax_cc), himax_cc},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(himax_b1), himax_b1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10,
		sizeof(himax_b2), himax_b2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(himax_b4), himax_b4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(himax_d8), himax_d8},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(himax_e0), himax_e0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(himax_e1), himax_e1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(himax_e2), himax_e2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm1), led_pwm1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm3),led_pwm3},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(display_on), display_on}
};
#endif

static struct dsi_cmd_desc himax_display_off_cmds[] = {
		{DTYPE_DCS_WRITE, 1, 0, 0, 1,
			sizeof(display_off), display_off},
		{DTYPE_DCS_WRITE, 1, 0, 0, 90,
			sizeof(enter_sleep), enter_sleep}
};

static char manufacture_id[2] = {0x04, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc himax_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

#if 0
static char chip_id[2] = {0xB9, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc himax_chip_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(chip_id), chip_id};
#endif
static uint32 mipi_himax_manufacture_id(void)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;
	uint32 *lp;

	tp = &himax_tx_buf;
	rp = &himax_rx_buf;
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &himax_manufacture_id_cmd;
	mipi_dsi_cmds_rx(tp, rp, cmd, 3);

{
	int i;
	char *cp;

	cp = (char *)rp->data;
	printk("rx-data: ");
	for (i = 0; i < rp->len; i++, cp++)
		printk("%x ", *cp);
	printk("\n");
}

	lp = (uint32 *)rp->data;

	printk("%s: manu_id=%x", __func__, *lp);

	return *lp;
}

static struct dsi_cmd_desc himax_cmd_backlight_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(led_pwm1), led_pwm1},
};

static struct dsi_cmd_desc himax_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_on), display_on},
};

// -----------------------------------------------------------------------------
//                         Common Routine Implementation
// -----------------------------------------------------------------------------
static int mipi_himax_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
//	static int init;

	pr_info("Daniel:%s+++",__func__);
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi  = &mfd->panel_info.mipi;

/*
	if (init == 0) {
		init = 1;
		goto end;
	} else
*/      {
		if (mipi->mode == DSI_VIDEO_MODE) {
		pr_info("Daniel:DSI_VIDEO_MODE.%s",__func__);
			mipi_dsi_cmds_tx(&himax_tx_buf, himax_video_on_cmds,
				ARRAY_SIZE(himax_video_on_cmds));
		} else {
				pr_info("Daniel:DSI_CMD_MODE.%s",__func__);
				mipi_dsi_cmds_tx(&himax_tx_buf, himax_cmd_on_cmds,
				ARRAY_SIZE(himax_cmd_on_cmds));
				mipi_dsi_cmd_bta_sw_trigger();
				mipi_himax_manufacture_id();
			}
	}
	pr_info("Daniel:%s---",__func__);
//end:
	return 0;
}

static int mipi_himax_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(&himax_tx_buf, himax_display_off_cmds,
	ARRAY_SIZE(himax_display_off_cmds));
	return 0;
}

static int mipi_dsi_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	static int bl_level_old;

	mipi  = &mfd->panel_info.mipi;
	pr_debug("%s+:bl=%d status=%d\n", __func__, mfd->bl_level, mipi_status);
	if (mipi_status == 0 || bl_level_old == mfd->bl_level)
		goto end;
	if (mipi_himax_pdata && mipi_himax_pdata->shrink_pwm)
		led_pwm1[1] = mipi_himax_pdata->shrink_pwm(mfd->bl_level);
	else
		led_pwm1[1] = (unsigned char)(mfd->bl_level);

	if (mipi->mode == DSI_VIDEO_MODE) {
		mipi_dsi_cmd_mode_ctrl(1);	/* enable cmd mode */
		mipi_dsi_cmds_tx(&himax_tx_buf, himax_cmd_backlight_cmds,
		ARRAY_SIZE(himax_cmd_backlight_cmds));
		mipi_dsi_cmd_mode_ctrl(0);	/* disable cmd mode */
	} else {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
		mipi_dsi_cmds_tx(&himax_tx_buf, himax_cmd_backlight_cmds,
		ARRAY_SIZE(himax_cmd_backlight_cmds));
	}
	bl_level_old = mfd->bl_level;
end:
	return 0;
}

static void mipi_himax_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;

	bl_level = mfd->bl_level;
	pr_debug("%s+ bl_level=%d\n", __func__, mfd->bl_level);

	mipi_dsi_set_backlight(mfd);
}

static void mipi_himax_display_on(struct msm_fb_data_type *mfd)
{
	pr_debug("%s+\n", __func__);
	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	mipi_dsi_cmds_tx(&himax_tx_buf, himax_display_on_cmds,
		ARRAY_SIZE(himax_display_on_cmds));
}

static int mipi_himax_lcd_probe(struct platform_device *pdev)
{

	pr_info("Daniel:%s",__func__);
	if (pdev->id == 0) {
		mipi_himax_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);
	pr_info("Daniel:%s:end",__func__);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_himax_lcd_probe,
	.driver = {
		.name   = "mipi_himax",
	},
};

static struct msm_fb_panel_data himax_panel_data = {
	.on		= mipi_himax_lcd_on,
	.off		= mipi_himax_lcd_off,
	.set_backlight  = mipi_himax_set_backlight,
	.display_on  = mipi_himax_display_on,
};

static int ch_used[3];

int mipi_himax_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	pr_info("Daniel:%s",__func__);

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_himax", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	himax_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &himax_panel_data,
		sizeof(himax_panel_data));
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

static int __init mipi_himax_lcd_init(void)
{

	pr_info("Daniel:%s",__func__);
	mipi_dsi_buf_alloc(&himax_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&himax_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
	pr_info("Daniel:%s---",__func__);
}

module_init(mipi_himax_lcd_init);
