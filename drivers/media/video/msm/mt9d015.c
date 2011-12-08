/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera_sensor.h>
#include <mach/gpio.h>
#ifdef CONFIG_CAMERA_ZSL
#include <mach/camera-8x60_ZSL.h>
#else
#include <mach/camera-8x60.h>
#endif
#include "mt9d015.h"
/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define MT9D015_MODE_SELECT				0x0100
#define MT9D015_START_STREAM			0x01
#define MT9D015_STOP_STREAM				0x00
#define MT9D015_RESET_REGISTER			0x301A
#define MT9D015_SOFTWARE_RESET			0x0103
#define MT9D015_EMBEDDED_DATA_ENBL		0x3064
#define GROUPED_PARAMETER_HOLD_OFF		0x00
#define GROUPED_PARAMETER_HOLD			0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME		0x0202
/* Gain */
#define REG_GLOBAL_GAIN	0x0204
/* PLL registers */
#define REG_FRAME_LENGTH_LINES		0x0340
#define REG_LINE_LENGTH_PCK		0x0342
/* Test Pattern */
#define REG_TEST_PATTERN_MODE			0x0601
#define REG_VCM_NEW_CODE			0x30F2

/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8	0x00000100
#define Q10	0x00000400
#define MT9D015_MASTER_CLK_RATE 24000000
#define MT9D015_REG_MODEL_ID 0x0000
#define MT9D015_MODEL_ID 0x1501

/* AF Total steps parameters */
#define MT9D015_TOTAL_STEPS_NEAR_TO_FAR    32

uint16_t mt9d015_step_position_table[MT9D015_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t mt9d015_nl_region_boundary1 = 3;
uint16_t mt9d015_nl_region_code_per_step1 = 30;
uint16_t mt9d015_l_region_code_per_step = 4;
uint16_t mt9d015_damping_threshold = 10;
uint16_t mt9d015_sw_damping_time_wait = 1;

struct mt9d015_work_t {
	struct work_struct work;
};

static struct mt9d015_work_t *mt9d015_sensorw;
static struct i2c_client *mt9d015_client;
static struct platform_device *mt9d015_pdev;

struct mt9d015_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum mt9d015_resolution_t prev_res;
	enum mt9d015_resolution_t pict_res;
	enum mt9d015_resolution_t curr_res;
	enum mt9d015_test_mode_t  set_test;
};


static bool CSI_CONFIG;
static struct mt9d015_ctrl_t *mt9d015_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9d015_wait_queue);
DEFINE_MUTEX(mt9d015_mut);

#if 0
static int cam_debug_init(void);
static struct dentry *debugfs_base;
#endif

#ifdef CONFIG_MACH_RUBY
void vcm_workaround_set_camera_running(int isRunning);
#endif
/*=============================================================*/

static int mt9d015_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9d015_client->adapter, msgs, 2) < 0) {
		CDBG("mt9d015_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t mt9d015_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(mt9d015_client->adapter, msg, 1) < 0) {
		CDBG("mt9d015_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9d015_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = mt9d015_i2c_rxdata(mt9d015_client->addr, buf, rlen);
	if (rc < 0) {
		CDBG("mt9d015_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("mt9d015_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t mt9d015_i2c_write_w_sensor(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9d015_i2c_txdata(mt9d015_client->addr, buf, 4);
	if (rc < 0) {
		pr_info("[CAM]i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t mt9d015_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = mt9d015_i2c_txdata(mt9d015_client->addr, buf, 3);
	if (rc < 0) {
		pr_info("[CAM]i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t mt9d015_i2c_write_w_table(struct mt9d015_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num; i++) {
		rc = mt9d015_i2c_write_w_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

#if 1 /* get fuse id */
static int mt9d015_i2c_read_fuseid(struct sensor_cfg_data *cdata)
{

	int32_t  rc;
	uint16_t reg_status = 0;
	unsigned short  fuseid[4] = {0};

	pr_info("[CAM]%s: sensor OTP information:\n", __func__);
	/* Read a word value from 0x301A  */
	rc = mt9d015_i2c_read(0x301A, &reg_status, 2);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_read value from 0x301A fail\n", __func__);
	/* OR 0x0020 with 0x301A[5] as 1 */
	reg_status = (reg_status|0x0020);
	pr_info("[CAM]%s: reg_status = %x\n", __func__, reg_status);

	/* set reg_status to 0x301A */
	rc = mt9d015_i2c_write_w_sensor(0x301A, reg_status);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_w_sensor 0x301A fail\n", __func__);

	/* Read fuseid value from 0x31F4, 0x31F6,0x31F8,0x31FA */
	rc = mt9d015_i2c_read(0x31F4, &fuseid[0], 2);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_w_sensor 0x31F4 fail\n", __func__);

	rc = mt9d015_i2c_read(0x31F6, &fuseid[1], 2);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_w_sensor 0x31F6 fail\n", __func__);

	rc = mt9d015_i2c_read(0x31F8, &fuseid[2], 2);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_w_sensor 0x318 fail\n", __func__);

	rc = mt9d015_i2c_read(0x31FA, &fuseid[3], 2);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_w_sensor 0x31A fail\n", __func__);

	pr_info("[CAM]%s: fuseid: %x%x%x%x\n",
		__func__, fuseid[3], fuseid[2], fuseid[1], fuseid[0]);

	/* return user space */
	cdata->cfg.fuse.fuse_id_word1 = 0;
	cdata->cfg.fuse.fuse_id_word2 = 0;
	cdata->cfg.fuse.fuse_id_word3 =
		(fuseid[3]<<16)|(fuseid[2]);/* High value*/
	cdata->cfg.fuse.fuse_id_word4 =
		(fuseid[1]<<16)|(fuseid[0]);/* Low value*/

	pr_info("[CAM]mt9d015: fuse->fuse_id_word1:%d\n",
		cdata->cfg.fuse.fuse_id_word1);
	pr_info("[CAM]mt9d015: fuse->fuse_id_word2:%d\n",
		cdata->cfg.fuse.fuse_id_word2);
	pr_info("[CAM]mt9d015: fuse->fuse_id_word3:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word3);
	pr_info("[CAM]mt9d015: fuse->fuse_id_word4:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word4);
	return 0;
}
#endif

static void mt9d015_group_hold_on(void)
{
	mt9d015_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void mt9d015_group_hold_off(void)
{
	mt9d015_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void mt9d015_start_stream(void)
{
	mt9d015_i2c_write_b_sensor(MT9D015_MODE_SELECT, MT9D015_START_STREAM);
}

static void mt9d015_stop_stream(void)
{
	mt9d015_i2c_write_b_sensor(MT9D015_MODE_SELECT, MT9D015_STOP_STREAM);
}

static void mt9d015_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = mt9d015_regs.reg_prev[MT9D015_FRAME_LENGTH_LINES].wdata
		* 0x00000400/
		mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata;
	d2 = mt9d015_regs.reg_prev[MT9D015_LINE_LENGTH_PCK].wdata
		* 0x00000400/
		mt9d015_regs.reg_snap[MT9D015_LINE_LENGTH_PCK].wdata;
	divider = d1 * d2 / 0x400;

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */
}

static uint16_t mt9d015_get_prev_lines_pf(void)
{
	if (mt9d015_ctrl->prev_res == QTR_SIZE)
		return mt9d015_regs.reg_prev[MT9D015_FRAME_LENGTH_LINES].wdata;
	else
		return mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9d015_get_prev_pixels_pl(void)
{
	if (mt9d015_ctrl->prev_res == QTR_SIZE)
		return mt9d015_regs.reg_prev[MT9D015_LINE_LENGTH_PCK].wdata;
	else
		return mt9d015_regs.reg_snap[MT9D015_LINE_LENGTH_PCK].wdata;
}

static uint16_t mt9d015_get_pict_lines_pf(void)
{
	if (mt9d015_ctrl->pict_res == QTR_SIZE)
		return mt9d015_regs.reg_prev[MT9D015_FRAME_LENGTH_LINES].wdata;
	else
		return mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9d015_get_pict_pixels_pl(void)
{
	if (mt9d015_ctrl->pict_res == QTR_SIZE)
		return mt9d015_regs.reg_prev[MT9D015_LINE_LENGTH_PCK].wdata;
	else
		return mt9d015_regs.reg_snap[MT9D015_LINE_LENGTH_PCK].wdata;
}

static uint32_t mt9d015_get_pict_max_exp_lc(void)
{
	if (mt9d015_ctrl->pict_res == QTR_SIZE)
		return mt9d015_regs.reg_prev[MT9D015_FRAME_LENGTH_LINES].wdata
				* 24;
	else
		return mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata
				* 24;
}

static int32_t mt9d015_write_exp_gain(uint16_t gain, uint32_t line)
{
	/* remove for AEC unstable issue under low light */
	uint16_t max_legal_gain = 0xE7F;/*0701 use max_legal_gain to E7F*/
	/* uint16_t max_legal_gain = 0x7F; */
	uint32_t ll_ratio; /* Q10 */
	uint32_t ll_pck, fl_lines;
	uint16_t offset = 8;
	int32_t rc = 0;

	if (mt9d015_ctrl->sensormode == SENSOR_PREVIEW_MODE) {

		mt9d015_ctrl->my_reg_gain = gain;
		mt9d015_ctrl->my_reg_line_count = (uint16_t)line;

		if (mt9d015_ctrl->prev_res == QTR_SIZE) {
			fl_lines = mt9d015_regs.reg_prev[MT9D015_FRAME_LENGTH_LINES].wdata;
			ll_pck = mt9d015_regs.reg_prev[MT9D015_LINE_LENGTH_PCK].wdata;
		} else {
			fl_lines = mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata;
			ll_pck = mt9d015_regs.reg_snap[MT9D015_LINE_LENGTH_PCK].wdata;
		}
	} else {
		fl_lines = mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata;
		ll_pck = mt9d015_regs.reg_snap[MT9D015_LINE_LENGTH_PCK].wdata;
	}

	/* workaround */
	if (gain == 0)
		return rc;

	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}

	/* in Q10 */
	line = (line * 0x400);//mt9d015_ctrl->fps_divider);

	CDBG("mt9d015_ctrl->fps_divider = %d\n", mt9d015_ctrl->fps_divider);
	CDBG("fl_lines = %d\n", fl_lines);
	CDBG("line = %d\n", line);

	if ((fl_lines - offset) < (line / 0x400))
		ll_ratio = (line / (fl_lines - offset));
	else
		ll_ratio = 0x400;
	 CDBG("ll_ratio = %d\n", ll_ratio);

	ll_pck = ll_pck * ll_ratio / 0x400 * mt9d015_ctrl->fps_divider;
	CDBG("ll_pck/0x400 = %d\n", ll_pck / 0x400);

	line = line / ll_ratio;
	CDBG("line = %d\n", line);

#if 0
	if (mt9d015_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		mt9d015_ctrl->my_reg_gain = gain;
		mt9d015_ctrl->my_reg_line_count = (uint16_t) line;
		line = (uint32_t) (line * mt9d015_ctrl->fps_divider /
						   0x00000400);
	} else {

	/* remove line * 2 for snapshot */
	/*
		line = (uint32_t) (line * 2 * mt9d015_ctrl->pict_fps_divider /
						   0x00000400);
	*/
		line = (uint32_t) (line * mt9d015_ctrl->pict_fps_divider /
						   0x00000400);
	}
#endif

	/* remove for AEC unstable issue under low light */
	/* gain |= 0x1000; */
	gain = gain/4; /* 20110617 shuji fine tune gain/4 */

	mt9d015_group_hold_on();
	rc = mt9d015_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	rc = mt9d015_i2c_write_w_sensor(REG_LINE_LENGTH_PCK, ll_pck / 0x400);
	rc = mt9d015_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
	mt9d015_group_hold_off();
	return rc;
}

static int32_t mt9d015_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = mt9d015_write_exp_gain(gain, line);
	return rc;
}

static int32_t mt9d015_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	uint32_t pre_fps = mt9d015_ctrl->fps_divider;
	pr_info("[CAM]mt9d015_set_fps\n");
	if (mt9d015_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		total_lines_per_frame = (uint16_t)
		((mt9d015_regs.reg_prev[MT9D015_FRAME_LENGTH_LINES].wdata)
		* mt9d015_ctrl->fps_divider/0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		((mt9d015_regs.reg_snap[MT9D015_FRAME_LENGTH_LINES].wdata)
		 * mt9d015_ctrl->pict_fps_divider/0x400);
	}
	mt9d015_ctrl->fps_divider = fps->fps_div;
	mt9d015_ctrl->pict_fps_divider = fps->pict_fps_div;

	if (mt9d015_ctrl->sensormode == SENSOR_PREVIEW_MODE &&
		(mt9d015_ctrl->my_reg_gain != 0 || mt9d015_ctrl->my_reg_line_count != 0)) {
		rc =
			mt9d015_write_exp_gain(mt9d015_ctrl->my_reg_gain,
				mt9d015_ctrl->my_reg_line_count * pre_fps / mt9d015_ctrl->fps_divider);
	}

#if 0 /* Micro sensor reduse frame rate automatically by Shuji */
	mt9d015_group_hold_on();
	/*20110617 shuiji fine tune for AEC unstable*/
	total_lines_per_frame = 0x0549;

	rc = mt9d015_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	mt9d015_group_hold_off();
#endif
	return rc;
}

#define DIV_CEIL(x, y) (x/y + ((x%y) ? 1 : 0))

static int32_t mt9d015_move_focus(int direction,
	int32_t num_steps)
{
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;
	return 0;
	if (direction == MOVE_NEAR)
		step_direction = 1;
	else
		step_direction = -1;

	dest_step_position = mt9d015_ctrl->curr_step_pos
						+ (step_direction * num_steps);

	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > MT9D015_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = MT9D015_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_position == mt9d015_ctrl->curr_step_pos)
		return 0;

	dest_lens_position = mt9d015_step_position_table[dest_step_position];
	target_dist = step_direction *
		(dest_lens_position - mt9d015_ctrl->curr_lens_pos);

	if (step_direction < 0 && (target_dist >=
		mt9d015_step_position_table[mt9d015_damping_threshold])) {
		small_step = DIV_CEIL(target_dist, 10);
		mt9d015_sw_damping_time_wait = 10;
	} else {
		small_step = DIV_CEIL(target_dist, 4);
		mt9d015_sw_damping_time_wait = 4;
	}

	for (next_lens_position = mt9d015_ctrl->curr_lens_pos
		+ (step_direction * small_step);
		(step_direction * next_lens_position) <=
		(step_direction * dest_lens_position);
		next_lens_position += (step_direction * small_step)) {
		mt9d015_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		next_lens_position);
		mt9d015_ctrl->curr_lens_pos = next_lens_position;
		usleep(mt9d015_sw_damping_time_wait*50);
	}

	if (mt9d015_ctrl->curr_lens_pos != dest_lens_position) {
		mt9d015_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		dest_lens_position);
		usleep(mt9d015_sw_damping_time_wait*50);
	}
	mt9d015_ctrl->curr_lens_pos = dest_lens_position;
	mt9d015_ctrl->curr_step_pos = dest_step_position;
	return 0;
}

static int32_t mt9d015_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	return 0;
	if (mt9d015_ctrl->curr_step_pos != 0) {
		rc = mt9d015_move_focus(MOVE_FAR,
		mt9d015_ctrl->curr_step_pos);
	} else {
		mt9d015_i2c_write_w_sensor(REG_VCM_NEW_CODE, 0x00);
	}

	mt9d015_ctrl->curr_lens_pos = 0;
	mt9d015_ctrl->curr_step_pos = 0;

	return rc;
}

#if 0
static void mt9d015_init_focus(void)
{
	uint8_t i;
	mt9d015_step_position_table[0] = 0;
	for (i = 1; i <= MT9D015_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= mt9d015_nl_region_boundary1) {
			mt9d015_step_position_table[i] =
				mt9d015_step_position_table[i-1]
				+ mt9d015_nl_region_code_per_step1;
		} else {
			mt9d015_step_position_table[i] =
				mt9d015_step_position_table[i-1]
				+ mt9d015_l_region_code_per_step;
		}

		if (mt9d015_step_position_table[i] > 255)
			mt9d015_step_position_table[i] = 255;
	}
}
#endif

static int32_t mt9d015_test(enum mt9d015_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF)
		return rc;
	else {
		/* REG_0x30D8[4] is TESBYPEN: 0: Normal Operation,
		1: Bypass Signal Processing
		REG_0x30D8[5] is EBDMASK: 0:
		Output Embedded data, 1: No output embedded data */
		if (mt9d015_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}

static int32_t mt9d015_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;
	struct msm_camera_csi_params mt9d015_csi_params;

	CDBG("sensor_settings\n");
#if 0
	stored_af_step = mt9d015_ctrl->curr_step_pos;
	mt9d015_set_default_focus(0);
	uint8_t stored_af_step = 0;
#endif

	mt9d015_stop_stream(); /*0x0100, 0x00*/
	msleep(50);
	if (update_type == REG_INIT) {
		mt9d015_i2c_write_w_sensor(MT9D015_RESET_REGISTER, 0x2118);
		mt9d015_i2c_write_w_table(mt9d015_regs.reg_mipi,
			mt9d015_regs.reg_mipi_size);
		mt9d015_i2c_write_w_table(mt9d015_regs.rec_settings,
			mt9d015_regs.rec_size);
		mt9d015_i2c_write_w_table(mt9d015_regs.reg_pll,
			mt9d015_regs.reg_pll_size);
		msleep(5);/* from Micro FAE */
		/*cam_debug_init();*/
		CSI_CONFIG = 0;
	} else if (update_type == UPDATE_PERIODIC) {
		pr_info("[CAM]sensor_settings: UPDATE_PERIODIC\n");
		msleep(100);
		mt9d015_group_hold_on(); /*0x0104,0x0100*/
		if (rt == RES_PREVIEW) {
			pr_info("[CAM]sensor_settings: preview settings\n");
			mt9d015_i2c_write_w_table(mt9d015_regs.reg_prev,
				mt9d015_regs.reg_prev_size);
		} else {
			pr_info("[CAM]sensor_settings: sanpshot settings\n");
			mt9d015_i2c_write_w_table(mt9d015_regs.reg_snap,
				mt9d015_regs.reg_snap_size);
		}

		msleep(5);/* from Micro FAE */

		if (!CSI_CONFIG) {
			mt9d015_csi_params.data_format = CSI_10BIT;
			mt9d015_csi_params.lane_cnt = 1;
			mt9d015_csi_params.lane_assign = 0xe4;
			mt9d015_csi_params.dpcm_scheme = 0;
			mt9d015_csi_params.settle_cnt = 0x20;
			mt9d015_csi_params.mipi_driving_strength = 0;
			mt9d015_csi_params.hs_impedence = 0x0F;
			rc = msm_camio_csi_config(&mt9d015_csi_params);
			msleep(50);
			CSI_CONFIG = 1;
		}

	/*0520: No Mirror/flip: 0: normal,1:mirror,2:flip,3:mirro and flip*/
		if (mt9d015_ctrl->sensordata->mirror_mode) {
			pr_info("[CAM]mt9d015_sensor_setting():mirror/flip\n");
			mt9d015_i2c_write_b_sensor(0x0101, 0x03);
		} else {
			pr_info("[CAM]mt9d015_sensor_setting(): No mirror/flip\n");
			mt9d015_i2c_write_b_sensor(0x301D, 0x00);
		}

		mt9d015_group_hold_off();/*0x0104,0x00 (byte)*/
		/*mt9d015_move_focus(MOVE_NEAR, stored_af_step);*/
		pr_info("[CAM]sensor_settings: streaming ON\n");
		mt9d015_start_stream(); /*0x0100, 0x01*/

		msleep(100);
	}
	return rc;
}

static int32_t mt9d015_video_config(int mode)
{

	int32_t rc = 0;
	int rt;
	pr_info("[CAM] mt9d015_video_config\n");
	/* change sensor resolution if needed */
	if (mt9d015_ctrl->prev_res == QTR_SIZE)
		rt = RES_PREVIEW;
	else
		rt = RES_CAPTURE;
	if (mt9d015_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	if (mt9d015_ctrl->set_test) {
		if (mt9d015_test(mt9d015_ctrl->set_test) < 0)
			return  rc;
	}

	mt9d015_ctrl->curr_res = mt9d015_ctrl->prev_res;
	mt9d015_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9d015_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	pr_info("[CAM] mt9d015_snapshot_config\n");
	/*change sensor resolution if needed */
	if (mt9d015_ctrl->curr_res != mt9d015_ctrl->pict_res) {
		if (mt9d015_ctrl->pict_res == QTR_SIZE)
			rt = RES_PREVIEW;
		else
			rt = RES_CAPTURE;
	if (mt9d015_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	}

	mt9d015_ctrl->curr_res = mt9d015_ctrl->pict_res;
	mt9d015_ctrl->sensormode = mode;
	return rc;
} /*end of mt9d015_snapshot_config*/

static int32_t mt9d015_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	if (mt9d015_ctrl->curr_res != mt9d015_ctrl->pict_res) {
		if (mt9d015_ctrl->pict_res == QTR_SIZE)
			rt = RES_PREVIEW;
		else
			rt = RES_CAPTURE;
		if (mt9d015_sensor_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;
	}

	mt9d015_ctrl->curr_res = mt9d015_ctrl->pict_res;
	mt9d015_ctrl->sensormode = mode;
	return rc;
} /*end of mt9d015_raw_snapshot_config*/

static int32_t mt9d015_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9d015_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = mt9d015_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = mt9d015_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t mt9d015_power_down(void)
{
	return 0;
}

#ifdef CONFIG_MSM_CAMERA_8X60
static int mt9d015_vreg_enable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;

	pr_info("[CAM]%s camera vreg on\n", __func__);

	if (sdata->camera_power_on == NULL) {
		pr_err("[CAM]sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_power_on();
	return rc;
}
static int mt9d015_vreg_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;
	printk(KERN_INFO "%s camera vreg off\n", __func__);
	if (sdata->camera_power_off == NULL) {
		pr_err("[CAM]sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_power_off();
	return rc;
}
#endif

static int mt9d015_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("probe done\n");
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9d015_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	pr_info("[CAM] %s: %d\n", __func__, __LINE__);

	if (!data)
	{
		rc = -EINVAL;
		pr_err("[CAM] %s: data = NULL\n", __func__);
		return rc;
	}

	rc = gpio_request(data->sensor_reset, "mt9d015");

	if (!rc) {
		pr_info("[CAM] sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(10);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(10);
	} else {
		goto init_probe_done;
	}


	rc = mt9d015_i2c_read(0x0000, &chipid, 2);
	CDBG("[CAM] ID: %d\n", chipid);
	/* 4. Compare sensor ID to MT9E013 ID: */
	if (chipid != 0x1501) {
		rc = -ENODEV;
		pr_info("[CAM] mt9d015_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}

	mt9d015_ctrl = kzalloc(sizeof(struct mt9d015_ctrl_t), GFP_KERNEL);
	if (!mt9d015_ctrl) {
		pr_info("[CAM] mt9d015_init failed!\n");
		rc = -ENOMEM;
		goto init_probe_fail;
	}
	mt9d015_ctrl->fps_divider = 1 * 0x00000400;
	mt9d015_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9d015_ctrl->set_test = TEST_OFF;
	mt9d015_ctrl->prev_res = QTR_SIZE;
	mt9d015_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9d015_ctrl->sensordata = data;

	goto init_probe_done;
init_probe_fail:
	pr_info("[CAM] mt9d015_probe_init_sensor fails\n");
	gpio_set_value_cansleep(data->sensor_reset, 0);
	mt9d015_probe_init_done(data);
init_probe_done:
	pr_info("[CAM] mt9d015_probe_init_sensor finishes\n");
	return rc;
}
/* camsensor_mt9d015_reset */

int mt9d015_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	pr_info("[CAM] mt9d015_sensor_open_init\n");

	mt9d015_ctrl = kzalloc(sizeof(struct mt9d015_ctrl_t), GFP_KERNEL);
	if (!mt9d015_ctrl) {
		CDBG("mt9d015_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9d015_ctrl->fps_divider = 1 * 0x00000400;
	mt9d015_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9d015_ctrl->set_test = TEST_OFF;
	mt9d015_ctrl->prev_res = QTR_SIZE;
	mt9d015_ctrl->pict_res = FULL_SIZE;
	mt9d015_ctrl->my_reg_gain = 0;
	mt9d015_ctrl->my_reg_line_count = 0;

	if (data)
		mt9d015_ctrl->sensordata = data;
	else
	{
		rc = -EINVAL;
		pr_err("[CAM] Calling mt9d015_sensor_open_init fail for data = NULL\n");
		return rc;
	}

	if (rc < 0) {
		pr_err("[CAM] Calling mt9d015_sensor_open_init fail1\n");
		return rc;
	}

	pr_info("[CAM]  mt9d015_sensor_open_init()  camera_running=1\n");
#ifdef CONFIG_MACH_RUBY
	vcm_workaround_set_camera_running(1);
#endif
	msleep(1);

	/* HTC Camera Add Start */
#ifndef CONFIG_MSM_CAMERA_8X60
	data->pdata->camera_gpio_on();
#endif

#ifdef CONFIG_MSM_CAMERA_8X60
	if (!data->power_down_disable) {
		rc = mt9d015_vreg_enable(mt9d015_pdev);
	}

	if (rc < 0)
		pr_err("[CAM] mt9d015_sensor_open_init fail sensor power on error\n");
#endif

	mdelay(5);

	/*switch PCLK and MCLK to 2nd cam*/
	pr_info("[CAM] mt9d015: mt9d015_sensor_open_init: switch clk\n");
	if (data->camera_clk_switch != NULL)
		data->camera_clk_switch();

#ifdef CONFIG_MSM_CAMERA_8X60
	data->pdata->camera_gpio_on();
#endif
	mdelay(1);
	/* HTC Camera Add End */

	/* Configure CAM GPIO ON (CAM_MCLK)*/
	pr_info("[CAM]%s msm_camio_probe_on()\n", __func__);
	msm_camio_probe_on(mt9d015_pdev);

	CDBG("%s: %d\n", __func__, __LINE__);
	/* enable mclk first */
	msm_camio_clk_rate_set(MT9D015_MASTER_CLK_RATE);

	rc = mt9d015_probe_init_sensor(data);

	if (rc < 0)
		goto init_fail;

	/* HTC Camera Add Start */
#ifndef CONFIG_MSM_CAMERA_8X60
	msm_camio_camif_pad_reg_reset();
#else
    #if 0
	if (!data->power_down_disable) {
		/* follow optical team Power Flow */
		rc = gpio_request(data->sensor_reset, "mt9d015");
		if (!rc) {
			rc = gpio_direction_output(data->sensor_reset, 1);
			msleep(1);
		} else
			pr_err("[CAM]GPIO(%d) request faile", data->sensor_reset);
		gpio_free(data->sensor_reset);
	}
	#endif
#endif
	/* HTC Camera Add End */

    /*Reset Sensor*/
	mt9d015_i2c_write_b_sensor(MT9D015_SOFTWARE_RESET, 0x01);
	msleep(300);
	CDBG("[CAM]%s init settings\n", __func__);
	if (mt9d015_ctrl->prev_res == QTR_SIZE)
		rc = mt9d015_sensor_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9d015_sensor_setting(REG_INIT, RES_CAPTURE);
	mt9d015_ctrl->fps = 30*Q8;
	/*mt9d015_init_focus();*/
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;

init_fail:
	pr_info("[CAM] init_fail\n");
	mt9d015_probe_init_done(data);

	pr_info("[CAM]  mt9d015_sensor_open_init()  camera_running=0\n");
#ifdef CONFIG_MACH_RUBY
	vcm_workaround_set_camera_running(0);
#endif
init_done:
	pr_info("[CAM] init_done\n");
	return rc;
} /*endof mt9d015_sensor_open_init*/

static int mt9d015_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9d015_wait_queue);
	return 0;
}

static const struct i2c_device_id mt9d015_i2c_id[] = {
	{"mt9d015", 0},
	{ }
};

static int mt9d015_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("[CAM] mt9d015_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("[CAM] i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9d015_sensorw = kzalloc(sizeof(struct mt9d015_work_t), GFP_KERNEL);
	if (!mt9d015_sensorw) {
		pr_info("[CAM] kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9d015_sensorw);
	mt9d015_init_client(client);
	mt9d015_client = client;


	pr_info("[CAM] mt9d015_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("mt9d015_probe failed! rc = %d\n", rc);
	return rc;
}

static int mt9d015_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

} /*end of mt9d015_snapshot_config*/

static int __exit mt9d015_remove(struct i2c_client *client)
{
	struct mt9d015_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9d015_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9d015_i2c_driver = {
	.id_table = mt9d015_i2c_id,
	.probe  = mt9d015_i2c_probe,
	.remove = __exit_p(mt9d015_i2c_remove),
	.driver = {
		.name = "mt9d015",
	},
};

int mt9d015_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&mt9d015_mut);
	CDBG("mt9d015_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
			mt9d015_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			mt9d015_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				mt9d015_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				mt9d015_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				mt9d015_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				mt9d015_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = mt9d015_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				mt9d015_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				mt9d015_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = mt9d015_set_sensor_mode(cdata.mode,
					cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = mt9d015_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				mt9d015_move_focus(
				cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				mt9d015_set_default_focus(
				cdata.cfg.focus.steps);
			break;

		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = MT9D015_TOTAL_STEPS_NEAR_TO_FAR;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_EFFECT:
			rc = mt9d015_set_default_focus(
				cdata.cfg.effect);
			break;


		case CFG_SEND_WB_INFO:
			rc = mt9d015_send_wb_info(
				&(cdata.cfg.wb_info));
			break;

		case CFG_I2C_IOCTL_R_OTP:
		pr_info("[CAM]Line:%d CFG_I2C_IOCTL_R_OTP : read fuse id.\n",
				__LINE__);
		rc = mt9d015_i2c_read_fuseid(&cdata);
		if (copy_to_user(argp, &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(&mt9d015_mut);

	return rc;
}

static int mt9d015_sensor_release(void)
{
	int rc = -EBADF;
	struct msm_camera_sensor_info *sdata = mt9d015_pdev->dev.platform_data;

	mutex_lock(&mt9d015_mut);

	mt9d015_stop_stream(); /*0x0100, 0x00*/
	msleep(110);

	mt9d015_power_down();


/*HTC_START Horng 20110905*/
	msm_mipi_csi_disable();
/*HTC_END*/


	gpio_set_value_cansleep(mt9d015_ctrl->sensordata->sensor_reset, 0);
	msleep(5);

/*
#ifdef CONFIG_MSM_CAMERA_8X60
	if (!sdata->power_down_disable) {
		rc = gpio_request(sdata->sensor_reset, "mt9d015");
		if (!rc) {
			rc = gpio_direction_output(sdata->sensor_reset, 0);
			mdelay(2);
		} else
			pr_err("[CAM]GPIO(%d) request faile", sdata->sensor_reset);
		gpio_free(sdata->sensor_reset);
	}
#endif
*/

	/* Configure CAM GPIO OFF (CAM_MCLK)*/
	pr_info("[CAM]%s msm_camio_probe_off()\n", __func__);
	msm_camio_probe_off(mt9d015_pdev);
	sdata->pdata->camera_gpio_off();

	mdelay(2);

	gpio_free(mt9d015_ctrl->sensordata->sensor_reset);

#ifdef CONFIG_MSM_CAMERA_8X60
	if (!sdata->power_down_disable) {
		mt9d015_vreg_disable(mt9d015_pdev);
	}
#endif

	pr_info("[CAM]  mt9d015_sensor_release()  camera_running=0\n");
	msleep(1);
#ifdef CONFIG_MACH_RUBY
	vcm_workaround_set_camera_running(0);
#endif
	kfree(mt9d015_ctrl);
	mt9d015_ctrl = NULL;
	CDBG("mt9d015_release completed\n");
	mutex_unlock(&mt9d015_mut);

	return rc;
}


static const char *mt9d015Vendor = "Micron";
static const char *mt9d015NAME = "mt9d015";
static const char *mt9d015Size = "2M";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", mt9d015Vendor, mt9d015NAME, mt9d015Size);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t htcwc_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", htcwc_value);
	return length;
}

static ssize_t htcwc_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

	htcwc_value = tmp;
	/* pr_info("[CAM]current_comm = %s\n", current->comm); */
	pr_info("[CAM]mt9d015 : htcwc_value = %d\n", htcwc_value);
	return count;
}

static int sensor_probe_node;
static ssize_t sensor_read_node(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", sensor_probe_node);
	return length;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(htcwc, 0777, htcwc_get, htcwc_set);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);

static struct kobject *android_mt9d015;

static int mt9d015_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM]mt9d015:kobject creat and add\n");
	android_mt9d015 = kobject_create_and_add("android_camera2", NULL);
	if (android_mt9d015 == NULL) {
		pr_info("[CAM]mt9d015_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]mt9d015:sysfs_create_file\n");
	ret = sysfs_create_file(android_mt9d015, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]mt9d015_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_mt9d015);
	}

	/* Camera AP detecte 2nd Cam*/
	ret = sysfs_create_file(android_mt9d015, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM]mt9d015_sysfs_init: sysfs_create_file htcwc failed\n");
		kobject_del(android_mt9d015);
	}

	ret = sysfs_create_file(android_mt9d015, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]mt9d015_sysfs_init: dev_attr_node failed\n");
		kobject_del(android_mt9d015);
	}

	return 0 ;
}

static int mt9d015_sensor_probe(struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&mt9d015_i2c_driver);
	if (rc < 0 || mt9d015_client == NULL) {
		rc = -ENOTSUPP;
		CDBG("I2C add driver failed");
		goto probe_fail;
	}

	pr_info("[CAM]s5k3h2yx s->node 0x%x\n", s->node);
	sensor_probe_node = s->node;

	msm_camio_clk_rate_set(MT9D015_MASTER_CLK_RATE);
	rc = mt9d015_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = mt9d015_sensor_open_init;
	s->s_release = mt9d015_sensor_release;
	s->s_config  = mt9d015_sensor_config;

	/*init done*/
	msleep(10);

	mt9d015_sysfs_init();
	msleep(10);

	/*s->s_mount_angle = 90;*/
	gpio_set_value_cansleep(info->sensor_reset, 0);

	info->pdata->camera_gpio_off();
	mdelay(10);

	if (!info->power_down_disable)
	    mt9d015_vreg_disable(mt9d015_pdev);

	mt9d015_probe_init_done(info);
	return rc;

probe_fail:
	pr_info("[CAM]mt9d015_sensor_probe: SENSOR PROBE FAILS!\n");
	info->pdata->camera_gpio_off();
	mdelay(10);

	if (!info->power_down_disable)
	    mt9d015_vreg_disable(mt9d015_pdev);

	return rc;
}

static int __mt9d015_probe(struct platform_device *pdev)
{
/* Camera Add */
#ifdef CONFIG_MSM_CAMERA_8X60
	int rc;
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;

	rc = mt9d015_vreg_enable(pdev);
	if (rc < 0)
		pr_err("[CAM]__mt9d015_probe fail sensor power on error\n");

	/*switch clk source*/
	pr_info("[CAM]mt9d015: mt9d015_sensor_probe switch clk\n");
	if (sdata->camera_clk_switch != NULL)
		sdata->camera_clk_switch();

	sdata->pdata->camera_gpio_on();
	mdelay(1);
#endif
	pr_info("[CAM]__mt9d015_probe\n");
	mt9d015_pdev = pdev;

/* Camera Add */

	return msm_camera_drv_start(pdev, mt9d015_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
		.probe = __mt9d015_probe,
		.driver = {
#ifdef CONFIG_MSM_CAMERA_8X60
		.name = "msm_camera_webcam",
#else
		.name = "msm_camera_mt9d015",
#endif
		.owner = THIS_MODULE,
		},
};


static int __init mt9d015_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9d015_init);
void mt9d015_exit(void)
{
	i2c_del_driver(&mt9d015_i2c_driver);
}
MODULE_DESCRIPTION("Aptina 2M Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static bool streaming = 1;

static int mt9d015_focus_test(void *data, u64 *val)
{
	int i = 0;
	mt9d015_set_default_focus(0);

	for (i = 90; i < 256; i++) {
		mt9d015_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	msleep(5000);
	for (i = 255; i > 90; i--) {
		mt9d015_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_focus, mt9d015_focus_test,
			NULL, "%lld\n");

static int mt9d015_step_test(void *data, u64 *val)
{
	int i = 0;
	mt9d015_set_default_focus(0);

	for (i = 0; i < MT9D015_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		mt9d015_move_focus(MOVE_NEAR, 1);
		msleep(5000);
	}

	mt9d015_move_focus(MOVE_FAR, MT9D015_TOTAL_STEPS_NEAR_TO_FAR);
	msleep(5000);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_step, mt9d015_step_test,
			NULL, "%lld\n");

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		mt9d015_start_stream();
		streaming = 1;
	} else {
		mt9d015_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");

#if 0
static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("mt9d015", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("focus", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_focus))
		return -ENOMEM;
	if (!debugfs_create_file("step", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_step))
		return -ENOMEM;
	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;

	return 0;
}
#endif


