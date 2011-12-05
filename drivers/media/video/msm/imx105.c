/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <media/msm_camera_sensor.h>
#include <mach/gpio.h>
#include <mach/camera-8x60.h>
#include <mach/vreg.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include "imx105.h"


#define REG_GROUPED_PARAMETER_HOLD			0x0104
#define GROUPED_PARAMETER_HOLD_OFF			0x00
#define GROUPED_PARAMETER_HOLD				0x01
#define REG_MODE_SELECT						0x0100
#define MODE_SELECT_STANDBY_MODE			0x00
#define MODE_SELECT_STREAM					0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME_HI		0x0202
#define REG_COARSE_INTEGRATION_TIME_LO		0x0203
/* Gain */
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_HI	0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LO	0x0205

#define REG_DIGITAL_GAIN_GREEN_R_HI	0x020E
#define REG_DIGITAL_GAIN_GREEN_R_LO	0x020F
#define REG_DIGITAL_GAIN_RED_HI	0x0210
#define REG_DIGITAL_GAIN_RED_LO	0x0211
#define REG_DIGITAL_GAIN_BLUE_HI	0x0212
#define REG_DIGITAL_GAIN_BLUE_LO	0x0213
#define REG_DIGITAL_GAIN_GREEN_B_HI	0x0214
#define REG_DIGITAL_GAIN_GREEN_B_LO	0x0215

/* mode setting */
#define REG_FRAME_LENGTH_LINES_HI   0x0340
#define REG_FRAME_LENGTH_LINES_LO	0x0341

#define REG_LINE_LENGTH_PCK_HI		0x0342
#define REG_LINE_LENGTH_PCK_LO		0x0343

#define	IMX105_STEPS_NEAR_TO_CLOSEST_INF		42
#define	IMX105_TOTAL_STEPS_NEAR_TO_FAR			42

#define REG_VCM_DAMP_STBY				0x3400
#define REG_VCM_CODE_LO_8BITS			0x3402
#define REG_VCM_CODE_HI_2BITS			0x3403
#define VCM_ENABLE				0x00
#define VCM_DISABLE				0x02
#define VCM_DAMP_CNTRL_ON		0x01
#define VCM_DAMP_CNTRL_OFF		0x00

uint16_t imx105_step_position_table[IMX105_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t imx105_nl_region_boundary1 = 3;
uint16_t imx105_nl_region_boundary2 = 5;
uint16_t imx105_nl_region_code_per_step1 = 48;
uint16_t imx105_nl_region_code_per_step2 = 12;
uint16_t imx105_l_region_code_per_step = 16;
uint16_t imx105_damping_threshold = 10;
uint16_t imx105_sw_damping_time_wait = 1;


/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define	IMX105_OFFSET	5
#define	Q8		0x00000100
#define	IMX105_DEFAULT_MASTER_CLK_RATE	24000000

/* Full	Size */
#define	IMX105_FULL_SIZE_WIDTH      3280
#define	IMX105_FULL_SIZE_HEIGHT		2464
#define	IMX105_FULL_SIZE_DUMMY_PIXELS	0
#define	IMX105_FULL_SIZE_DUMMY_LINES	0
/* Quarter Size	*/
#define	IMX105_QTR_SIZE_WIDTH	1640
#define	IMX105_QTR_SIZE_HEIGHT	1232
#define	IMX105_QTR_SIZE_DUMMY_PIXELS	0
#define	IMX105_QTR_SIZE_DUMMY_LINES		0
/* Video 1080P Size */
#define	IMX105_VIDEO_SIZE_WIDTH 3084
#define	IMX105_VIDEO_SIZE_HEIGHT 1156
#define	IMX105_VIDEO_SIZE_DUMMY_PIXELS	0
#define	IMX105_VIDEO_SIZE_DUMMY_LINES	0

/* Full	Size */
#define	IMX105_HRZ_FULL_BLK_PIXELS	256 //Line_length_pck-X_output_size=0x0DD0-0CD0
#define	IMX105_VER_FULL_BLK_LINES	70//38 Frame_length_lines-Y_output_size= 0x09E6-0x09A0
/* Quarter Size	*/
#define	IMX105_HRZ_QTR_BLK_PIXELS	1896
#define	IMX105_VER_QTR_BLK_LINES	34
/* Video 1080P Size */
#define	IMX105_HRZ_VIDEO_BLK_PIXELS	452
#define	IMX105_VER_VIDEO_BLK_LINES	110



/*20101011 QCT mesh LSC calibration*/
//static int global_mode = 0;

static int sensor_probe_node;
//static int preview_frame_count;
static struct wake_lock imx105_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&imx105_wake_lock, WAKE_LOCK_IDLE, "imx105");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&imx105_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&imx105_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&imx105_wake_lock);
}
struct imx105_work_t {
	struct work_struct work;
};

static struct imx105_work_t *imx105_sensorw;
static struct i2c_client *imx105_client;
static int32_t config_csi;


struct imx105_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;   	/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;  /* init to 1 * 0x00000400 */
	uint16_t fps;

	int16_t curr_lens_pos;
	int16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum imx105_resolution_t prev_res;
	enum imx105_resolution_t pict_res;
	enum imx105_resolution_t curr_res;
	enum imx105_test_mode_t  set_test;

	unsigned short imgaddr;
};


static uint8_t imx105_delay_msecs_stdby = 20;
static uint16_t imx105_delay_msecs_stream = 60;

static struct imx105_ctrl_t *imx105_ctrl;
static struct platform_device *imx105_pdev;

struct imx105_waitevent{
	uint32_t waked_up;
	wait_queue_head_t event_wait;
};

static DECLARE_WAIT_QUEUE_HEAD(imx105_wait_queue);
DECLARE_MUTEX(imx105_sem);

/*=============================================================*/

static int imx105_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr << 1,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr << 1,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(imx105_client->adapter, msgs, 2) < 0) {
		CDBG("imx105_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}
static int32_t imx105_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr << 1,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(imx105_client->adapter, msg, 1) < 0) {
		CDBG("imx105_i2c_txdata faild 0x%x\n", imx105_client->addr);
		return -EIO;
	}

	return 0;
}


static int32_t imx105_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	int count = 0;
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
retry:
	rc = imx105_i2c_rxdata(imx105_client->addr, buf, rlen);
	if (rc < 0) {
		pr_err("imx105_i2c_read 0x%x failed!\n", raddr);
		printk(KERN_ERR "starting read retry policy count:%d\n", count);
		udelay(10);
		count++;
		if (count < 20) {
			if (count > 10)
				udelay(100);
		} else
			return rc;
		goto retry;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}
static int32_t imx105_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	int count = 0;
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
retry:
	rc = imx105_i2c_txdata(imx105_client->addr, buf, 3);
  CDBG("imx105 i2c_write addr:0x%x val= 0x%x\n",waddr,bdata);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
    	pr_err(KERN_ERR "starting read retry policy count:%d\n", count);
		udelay(10);
		count++;
		if (count < 20) {
			if (count > 10)
		udelay(100);
		} else
			return rc;
		goto retry;
	}
	return rc;
}

static int32_t imx105_i2c_write_w_table(struct imx105_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = imx105_i2c_write_b_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void imx105_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint16_t preview_frame_length_lines, snapshot_frame_length_lines;
	uint16_t preview_line_length_pck, snapshot_line_length_pck;
	uint32_t divider, d1, d2;
	/* Total frame_length_lines and line_length_pck for preview */
	if (imx105_ctrl->prev_res == QTR_SIZE) {
		preview_frame_length_lines = IMX105_QTR_SIZE_HEIGHT +
			IMX105_VER_QTR_BLK_LINES;
		preview_line_length_pck = IMX105_QTR_SIZE_WIDTH +
			IMX105_HRZ_QTR_BLK_PIXELS;
	} else if (imx105_ctrl->prev_res == VIDEO_SIZE) {
		preview_frame_length_lines = IMX105_VIDEO_SIZE_HEIGHT +
			IMX105_VER_VIDEO_BLK_LINES;
		preview_line_length_pck = IMX105_VIDEO_SIZE_WIDTH +
			IMX105_HRZ_VIDEO_BLK_PIXELS;
	} else {
		preview_frame_length_lines = IMX105_FULL_SIZE_HEIGHT +
			IMX105_VER_FULL_BLK_LINES;
		preview_line_length_pck = IMX105_FULL_SIZE_WIDTH +
			IMX105_HRZ_FULL_BLK_PIXELS;
	}
	/* Total frame_length_lines and line_length_pck for snapshot */
	if (imx105_ctrl->pict_res == QTR_SIZE) {
		snapshot_frame_length_lines = IMX105_QTR_SIZE_HEIGHT +
			IMX105_VER_QTR_BLK_LINES;
		snapshot_line_length_pck = IMX105_QTR_SIZE_WIDTH +
			IMX105_HRZ_QTR_BLK_PIXELS;
	} else {
		snapshot_frame_length_lines = IMX105_FULL_SIZE_HEIGHT +
			IMX105_VER_FULL_BLK_LINES;
		snapshot_line_length_pck = IMX105_FULL_SIZE_WIDTH +
			IMX105_HRZ_FULL_BLK_PIXELS;
	}

	d1 = preview_frame_length_lines * 0x00000400/
		snapshot_frame_length_lines;
	d2 = preview_line_length_pck * 0x00000400/
		snapshot_line_length_pck;
	divider = d1 * d2 / 0x400;
	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */

}

static uint16_t imx105_get_prev_lines_pf(void)
{
	if (imx105_ctrl->prev_res == QTR_SIZE)
		return IMX105_QTR_SIZE_HEIGHT + IMX105_VER_QTR_BLK_LINES;
	else if (imx105_ctrl->prev_res == VIDEO_SIZE)
		return IMX105_VIDEO_SIZE_HEIGHT + IMX105_VER_VIDEO_BLK_LINES;
	else
		return IMX105_FULL_SIZE_HEIGHT + IMX105_VER_FULL_BLK_LINES;

}

static uint16_t imx105_get_prev_pixels_pl(void)
{
	if (imx105_ctrl->prev_res == QTR_SIZE)
		return IMX105_QTR_SIZE_WIDTH + IMX105_HRZ_QTR_BLK_PIXELS;
	else if (imx105_ctrl->prev_res == VIDEO_SIZE)
		return IMX105_VIDEO_SIZE_WIDTH + IMX105_HRZ_VIDEO_BLK_PIXELS;
	else
		return IMX105_FULL_SIZE_WIDTH + IMX105_HRZ_FULL_BLK_PIXELS;
}

static uint16_t imx105_get_pict_lines_pf(void)
{
		if (imx105_ctrl->pict_res == QTR_SIZE)
			return IMX105_QTR_SIZE_HEIGHT +
				IMX105_VER_QTR_BLK_LINES;
		else
			return IMX105_FULL_SIZE_HEIGHT +
				IMX105_VER_FULL_BLK_LINES;
}

static uint16_t imx105_get_pict_pixels_pl(void)
{
	if (imx105_ctrl->pict_res == QTR_SIZE)
		return IMX105_QTR_SIZE_WIDTH +
			IMX105_HRZ_QTR_BLK_PIXELS;
	else
		return IMX105_FULL_SIZE_WIDTH +
			IMX105_HRZ_FULL_BLK_PIXELS;
}

static uint32_t imx105_get_pict_max_exp_lc(void)
{
	if (imx105_ctrl->pict_res == QTR_SIZE)
		return (IMX105_QTR_SIZE_HEIGHT +
			IMX105_VER_QTR_BLK_LINES)*24;
	else
		return (IMX105_FULL_SIZE_HEIGHT +
			IMX105_VER_FULL_BLK_LINES)*24;
}

static int32_t imx105_set_fps(struct fps_cfg	*fps)
{
	int32_t rc = 0;
	imx105_ctrl->fps_divider = fps->fps_div;
	imx105_ctrl->pict_fps_divider = fps->pict_fps_div;
	imx105_ctrl->fps = fps->f_mult;
	return rc;
}

static int32_t imx105_write_exp_gain(uint16_t gain, uint32_t line, uint16_t dig_gain)
{
	static uint16_t max_analog_gain  = 0x00E0;
	static uint16_t max_digital_gain  = 0x0200;
	uint8_t again_msb, again_lsb;
	uint8_t dgain_msb, dgain_lsb;
	uint8_t intg_time_msb, intg_time_lsb;
	uint8_t frame_length_line_msb, frame_length_line_lsb;
	uint16_t frame_length_lines;
	int32_t rc = -1;
	

	CDBG("imx105_write_exp_gain : gain = %d line = %d", gain, line);
	CDBG("Default exposure for the time being\n");


	if (imx105_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		if (imx105_ctrl->curr_res  == QTR_SIZE) {
			frame_length_lines = IMX105_QTR_SIZE_HEIGHT +
				IMX105_VER_QTR_BLK_LINES;
		} else if (imx105_ctrl->curr_res  == VIDEO_SIZE) {
			frame_length_lines = IMX105_VIDEO_SIZE_HEIGHT +
				IMX105_VER_VIDEO_BLK_LINES;
		} else {
			frame_length_lines = IMX105_FULL_SIZE_HEIGHT +
				IMX105_VER_FULL_BLK_LINES;
		}
	} else {
		frame_length_lines = IMX105_FULL_SIZE_HEIGHT +
			IMX105_VER_FULL_BLK_LINES;
	}

	line = (uint32_t) (line * imx105_ctrl->fps_divider /
		0x00000400);

	if (line > (frame_length_lines - IMX105_OFFSET))
		frame_length_lines = line + IMX105_OFFSET;

	CDBG("imx105 setting frame_length_lines = %d line = %d\n",
					frame_length_lines, line);

	if (gain > max_analog_gain)
		/* range: 0 to 240 */
		gain = max_analog_gain;

	if (dig_gain > max_digital_gain)
		dig_gain = max_digital_gain;

	/* update gain registers */
	again_msb = (uint8_t) ((gain & 0xFF00) >> 8);
	again_lsb = (uint8_t) (gain & 0x00FF);

	dgain_msb = (uint8_t) ((dig_gain & 0xFF00) >> 8);
	dgain_lsb = (uint8_t) (dig_gain & 0x00FF);

	frame_length_line_msb = (uint8_t) ((frame_length_lines & 0xFF00) >> 8);
	frame_length_line_lsb = (uint8_t) (frame_length_lines & 0x00FF);

	/* update line count registers */
	intg_time_msb = (uint8_t) ((line & 0xFF00) >> 8);
	intg_time_lsb = (uint8_t) (line & 0x00FF);

	rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
					GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_ANALOGUE_GAIN_CODE_GLOBAL_HI = 0x%X\n", again_msb);
	rc = imx105_i2c_write_b_sensor(REG_ANALOGUE_GAIN_CODE_GLOBAL_HI,
					again_msb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_ANALOGUE_GAIN_CODE_GLOBAL_LO = 0x%X\n", again_lsb);
	rc = imx105_i2c_write_b_sensor(REG_ANALOGUE_GAIN_CODE_GLOBAL_LO,
					again_lsb);
	if (rc < 0)
		return rc;

	CDBG("imx105 setting REG_DIGITAL_GAIN_HI = 0x%X REG_DIGITAL_GAIN_LO = 0x%X\n",
					dgain_msb, dgain_lsb);
	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_GREEN_R_HI,
					dgain_msb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_GREEN_R_LO,
					dgain_lsb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_RED_HI,
					dgain_msb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_RED_LO,
					dgain_lsb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_BLUE_HI,
					dgain_msb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_BLUE_LO,
					dgain_lsb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_GREEN_B_HI,
					dgain_msb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_DIGITAL_GAIN_GREEN_B_LO,
					dgain_lsb);
	if (rc < 0)
		return rc;

	CDBG("imx105 setting REG_FRAME_LENGTH_LINES_HI = 0x%X\n",
					frame_length_line_msb);
	rc = imx105_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_HI,
			frame_length_line_msb);
	if (rc < 0)
		return rc;

	CDBG("imx105 setting REG_FRAME_LENGTH_LINES_LO = 0x%X\n",
			frame_length_line_lsb);
	rc = imx105_i2c_write_b_sensor(REG_FRAME_LENGTH_LINES_LO,
			frame_length_line_lsb);
	if (rc < 0)
		return rc;
	CDBG("imx105 setting REG_COARSE_INTEGRATION_TIME_HI = 0x%X\n", intg_time_msb);
	rc = imx105_i2c_write_b_sensor(REG_COARSE_INTEGRATION_TIME_HI,
					intg_time_msb);
	if (rc < 0)
		return rc;

	CDBG("imx105 setting REG_COARSE_INTEGRATION_TIME_LO = 0x%X\n", intg_time_lsb);
	rc = imx105_i2c_write_b_sensor(REG_COARSE_INTEGRATION_TIME_LO,
					intg_time_lsb);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
					GROUPED_PARAMETER_HOLD_OFF);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t imx105_set_pict_exp_gain(uint16_t gain, uint32_t line, uint16_t dig_gain)
{
	int32_t rc = 0;
	rc = imx105_write_exp_gain(gain, line, dig_gain);
	return rc;
} /* endof imx105_set_pict_exp_gain*/


static int16_t imx105_af_init(void)
{
	int32_t rc=0;

	uint8_t i;
	imx105_step_position_table[0] = 0;
	for(i=1; i <= IMX105_TOTAL_STEPS_NEAR_TO_FAR; i++){
		if ( i <= imx105_nl_region_boundary1){
			imx105_step_position_table[i] = imx105_step_position_table[i-1] + imx105_nl_region_code_per_step1;
			}
		else if ( i <= imx105_nl_region_boundary2){
			imx105_step_position_table[i] = imx105_step_position_table[i-1] + imx105_nl_region_code_per_step2;
			}
		else{
			imx105_step_position_table[i] = imx105_step_position_table[i-1] + imx105_l_region_code_per_step;
			}
		if (imx105_step_position_table[i] >1023)
			imx105_step_position_table[i] = 1023;
	}

	rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_w_table(imx105_regs.vcm_tbl,
		imx105_regs.vcmtbl_size);
	if (rc < 0)
		return rc;

	rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
	if (rc < 0)
		return rc;
	return rc;
}

/*
#define DIV_CEIL(x, y) (x/y + (x%y) ? 1 : 0)
*/
#define DIV_CEIL(x, y) x/y

static int32_t imx105_move_focus(int direction,
	int32_t num_steps)
{

	int32_t rc = 0;
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;
	CDBG("imx105_move_focus entering.... \n");
	if (direction == MOVE_NEAR){
		step_direction = 1;
	}
	else if (direction == MOVE_FAR){
		step_direction = -1;
	}
	else{
		pr_err("Illegal focus direction \n");
		return -EINVAL;
	}
	CDBG("imx105_move_focus calculating dest_step_position \n");
	dest_step_position = imx105_ctrl->curr_step_pos + (step_direction * num_steps);
	if (dest_step_position < 0){
		dest_step_position = 0;
	}
	else if (dest_step_position > IMX105_TOTAL_STEPS_NEAR_TO_FAR){
		dest_step_position = IMX105_TOTAL_STEPS_NEAR_TO_FAR;
	}
	if(dest_step_position == imx105_ctrl->curr_step_pos){
		CDBG("imx105_move_focus ==  imx105_ctrl->curr_step_pos No Move exit \n");
		return rc;
	}
	CDBG("Cur Step: %hd Step Direction: %hd Dest Step Pos: %hd Num Step: %hd\n", imx105_ctrl->curr_step_pos, step_direction, dest_step_position, num_steps);

	dest_lens_position = imx105_step_position_table[dest_step_position];
	target_dist = step_direction *
		(dest_lens_position - imx105_ctrl->curr_lens_pos);

	if (step_direction < 0 && (target_dist >=
		imx105_step_position_table[imx105_damping_threshold])) {
		small_step = DIV_CEIL(target_dist, 4);
		imx105_sw_damping_time_wait = 10;
	} else {
		small_step = DIV_CEIL(target_dist, 2);
		imx105_sw_damping_time_wait = 4;
    }

	for (next_lens_position = imx105_ctrl->curr_lens_pos
		+ (step_direction * small_step);
		(step_direction * next_lens_position) <=
		(step_direction * dest_lens_position);
		next_lens_position += (step_direction * small_step)) {
		rc = imx105_i2c_write_b_sensor(REG_VCM_CODE_LO_8BITS, (next_lens_position&0x00FF));
		CDBG("imx105_move_focus REG_VCM_CODE_LO_8BITS = %d \n", next_lens_position&0x00FF);
		rc = imx105_i2c_write_b_sensor(REG_VCM_CODE_HI_2BITS, (next_lens_position&0x0300)>>8);
		CDBG("imx105_move_focus REG_VCM_CODE_HI_2BITS = %d \n",(next_lens_position&0x0300)>>8);
		imx105_ctrl->curr_lens_pos = next_lens_position;
		usleep(imx105_sw_damping_time_wait*500);
	}

	if(imx105_ctrl->curr_lens_pos != dest_lens_position){
		CDBG("imx105_move_focus writing i2c at line %d ...\n", __LINE__);
		CDBG("imx105_move_focus curr_lens_pos = %d  dest_lens_position = %d ...\n", imx105_ctrl->curr_lens_pos, dest_lens_position);
		rc = imx105_i2c_write_b_sensor(REG_VCM_CODE_LO_8BITS, (dest_lens_position&0x00FF));
		CDBG("imx105_move_focus REG_VCM_CODE_LO_8BITS = %d \n", dest_lens_position&0x00FF);
		rc = imx105_i2c_write_b_sensor(REG_VCM_CODE_HI_2BITS, (dest_lens_position&0x0300)>>8);
		CDBG("imx105_move_focus REG_VCM_CODE_HI_2BITS = %d \n",(dest_lens_position&0x0300)>>8);

		if (rc < 0){
			CDBG("imx105_move_focus failed writing i2c at line %d ...\n", __LINE__);
			return rc;
			}
		CDBG("imx105_move_focus writing Success i2c at line %d ...\n", __LINE__);

		usleep(imx105_sw_damping_time_wait*500);

	}
	imx105_ctrl->curr_lens_pos = dest_lens_position;
	imx105_ctrl->curr_step_pos = dest_step_position;
	CDBG("imx105_move_focus exit.... \n");
	return rc;

}


static int32_t imx105_set_default_focus(void)
{
	int32_t rc=0;
	CDBG("imx105_set_default_focus entering.... \n");
	if(imx105_ctrl->curr_step_pos != 0){
		rc = imx105_move_focus(MOVE_FAR, imx105_ctrl->curr_step_pos);
		if (rc < 0)
			return rc;
	}
	else{

		rc = imx105_i2c_write_b_sensor(REG_VCM_CODE_LO_8BITS, 0x00);
		rc = imx105_i2c_write_b_sensor(REG_VCM_CODE_HI_2BITS, 0x00);
		if (rc < 0) {
		CDBG("Imx105 Writing to VCM register Failed!!!\n");
		return rc;
		}
	}
	imx105_ctrl->curr_lens_pos = 0;
	imx105_ctrl->curr_step_pos = 0;
	CDBG("mt9p017_set_default_focus exit.... \n");
	return rc;
}

static int32_t imx105_sensor_setting(int update_type, int rt)
{
	int32_t rc = 0;
	struct msm_camera_csi_params imx105_csi_params;
	switch (update_type) {
	case REG_INIT:
		if (rt == RES_PREVIEW || rt == RES_VIDEO || rt == RES_CAPTURE) {
		CDBG("Sensor setting Init = %d\n", rt);
			/* reset fps_divider */
			imx105_ctrl->fps = 30 * Q8;
			imx105_ctrl->fps_divider = 1* 0x400;
			CDBG("%s: %d\n", __func__, __LINE__);
			/* stop streaming */
			rc = imx105_i2c_write_b_sensor(REG_MODE_SELECT,
				MODE_SELECT_STANDBY_MODE);
			if (rc < 0)
				return rc;

			/*imx105_delay_msecs_stdby*/
			//msleep(imx105_delay_msecs_stdby);
			rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
								GROUPED_PARAMETER_HOLD);
				if (rc < 0)
					return rc;


			rc = imx105_i2c_write_w_table(imx105_regs.init_tbl,
				imx105_regs.inittbl_size);
			if (rc < 0)
				return rc;

			rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
								GROUPED_PARAMETER_HOLD_OFF);
			if (rc < 0)
				return rc;
			CDBG("%s: %d\n", __func__, __LINE__);
			/*imx105_delay_msecs_stdby*/
			mdelay(imx105_delay_msecs_stdby);

			return rc;
		}
		break;

	case UPDATE_PERIODIC:
		if (rt == RES_PREVIEW || rt == RES_VIDEO  || rt == RES_CAPTURE) {
			CDBG("%s: %d\n", __func__, __LINE__);

			/* config mipi csi controller */
			if (config_csi == 0) {
			imx105_csi_params.lane_cnt = 2;
			imx105_csi_params.data_format = CSI_10BIT;
			imx105_csi_params.lane_assign = 0xe4;
			imx105_csi_params.dpcm_scheme = 0;
			imx105_csi_params.settle_cnt = 0x14;

			rc = msm_camio_csi_config(&imx105_csi_params);
			if (rc < 0)
				CDBG("config csi controller failed \n");

			mdelay(imx105_delay_msecs_stream);
			config_csi = 1;
			}
			/* stop streaming */
			CDBG("Sensor setting snap or preview = %d\n", rt);
			rc =imx105_i2c_write_b_sensor(REG_MODE_SELECT,
				MODE_SELECT_STANDBY_MODE);
			if(rc < 0)
				return rc;
			//msleep(imx105_delay_msecs_stdby);
			rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
								GROUPED_PARAMETER_HOLD);
				if (rc < 0)
					return rc;

			/* write mode settings */
			if (rt == RES_PREVIEW)
			{
			rc = imx105_i2c_write_w_table(imx105_regs.prev_tbl,
				imx105_regs.prevtbl_size);
			CDBG(" IMX105 Preview configs done  \n");
			if (rc < 0)
				return rc;
			} else if (rt == RES_VIDEO)
			{
			rc = imx105_i2c_write_w_table(imx105_regs.video_tbl,
				imx105_regs.videotbl_size);
			pr_info(" IMX105 video 1080p configs done  \n");
			if (rc < 0)
				return rc;
			}else{
			rc = imx105_i2c_write_w_table(imx105_regs.snap_tbl,
				imx105_regs.snaptbl_size);
			if (rc < 0)
				return rc;
			}

			rc = imx105_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
								GROUPED_PARAMETER_HOLD_OFF);
			if (rc < 0)
				return rc;
			CDBG(" IMX105 Turn on streaming \n");

			/* turn on streaming */
			rc = imx105_i2c_write_b_sensor(REG_MODE_SELECT,
				MODE_SELECT_STREAM);
			if (rc < 0)
				return rc;
			mdelay(imx105_delay_msecs_stdby);

		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}


static int32_t imx105_video_config(int mode)
{

	int32_t	rc = 0;
	int	rt;

	pr_info("[CAM]imx105_video_config mode %d\n", mode);

	/* change sensor resolution	if needed */
	if (imx105_ctrl->prev_res == QTR_SIZE) {
		pr_info("QTR_SIZE\n");
			rt = RES_PREVIEW;
			imx105_delay_msecs_stdby	=
				(((2 * 1000 * Q8 * imx105_ctrl->fps_divider) / 0x400) /
				imx105_ctrl->fps) + 1;
	} else if (imx105_ctrl->prev_res == VIDEO_SIZE) {
		pr_info("VIDEO_SIZE\n");
			rt = RES_VIDEO;
			imx105_delay_msecs_stdby	=
				(((2 * 1000 * Q8 * imx105_ctrl->fps_divider) / 0x400) /
				imx105_ctrl->fps) + 1;
		} else {
		pr_info("FULL_SIZE\n");
			rt = RES_CAPTURE;
			imx105_delay_msecs_stdby	=
				(((2 * 1000 * Q8 * imx105_ctrl->fps_divider) /0x400) /
				imx105_ctrl->fps) + 1;
		}
		if (imx105_sensor_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;


	imx105_ctrl->curr_res = imx105_ctrl->prev_res;
	imx105_ctrl->sensormode = mode;
	return rc;
}

static int32_t imx105_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	if (imx105_ctrl->curr_res != imx105_ctrl->pict_res) {
		if (imx105_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			imx105_delay_msecs_stdby =
				(((2 * 1000 * Q8 * imx105_ctrl->fps_divider)/0x400) /
				imx105_ctrl->fps) + 1;
		} else {
			rt = RES_CAPTURE;
			imx105_delay_msecs_stdby =
				(((2 * 1000 * Q8 * imx105_ctrl->fps_divider)/0x400) /
				imx105_ctrl->fps) + 1;
		}

	CDBG("Calling imx105_snapshot_config\n");
	if (imx105_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
}

	imx105_ctrl->curr_res = imx105_ctrl->pict_res;
	imx105_ctrl->sensormode = mode;
	return rc;
} /*end of imx105_snapshot_config*/

static int32_t imx105_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	if (imx105_ctrl->curr_res != imx105_ctrl->pict_res) {
		if (imx105_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;
			imx105_delay_msecs_stdby =
				((2 * 1000 * Q8 *
				imx105_ctrl->fps_divider) /
				imx105_ctrl->fps) + 1;
		} else {
			rt = RES_CAPTURE;
			imx105_delay_msecs_stdby =
				((2 * 1000 * Q8 * imx105_ctrl->fps_divider)/
				imx105_ctrl->fps) + 1;
		}
		if (imx105_sensor_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;
	}
	imx105_ctrl->curr_res = imx105_ctrl->pict_res;
	imx105_ctrl->sensormode = mode;
	return rc;
} /*end of imx105_raw_snapshot_config*/

static int32_t imx105_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = imx105_pdev->dev.platform_data;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		pr_info("mode=%d\n",mode);
		imx105_ctrl->prev_res = res; /* VIDEO_SIZE, FULL_SIZE, QTR_SIZE */
		rc = imx105_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		imx105_ctrl->pict_res = res;
		rc = imx105_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		imx105_ctrl->pict_res = res;
		rc = imx105_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}
static int32_t imx105_power_down(void)
{
	imx105_i2c_write_b_sensor(REG_VCM_DAMP_STBY, VCM_DISABLE | VCM_DAMP_CNTRL_OFF);
	imx105_i2c_write_b_sensor(REG_MODE_SELECT,
		MODE_SELECT_STANDBY_MODE);
	msleep(imx105_delay_msecs_stdby);
	return 0;

}

static int imx105_vreg_enable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;
	pr_info("%s camera vreg on\n", __func__);

	if (sdata->camera_power_on == NULL) {
		pr_err("sensor platform_data didnt register camera_power_on\n");
		return -EIO;
	}
	rc = sdata->camera_power_on();
	return rc;
}

static int imx105_vreg_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;
	pr_info("%s camera vreg off\n", __func__);

	if (sdata->camera_power_off == NULL) {
		pr_err("sensor platform_data didnt register camera_power_off\n");
		return -EIO;
	}
	rc = sdata->camera_power_off();
	return rc;
}

static int imx105_probe_init_done(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	rc = gpio_request(data->sensor_reset, "imx105");
	if (!rc) {
		gpio_direction_output(data->sensor_reset, 0);
	} else {
		pr_err("[CAM]GPIO (%d) request failed\n", data->sensor_reset);
	}
	gpio_free(data->sensor_reset);
	mdelay(1);

	data->pdata->camera_gpio_off();
	mdelay(1);

	if (!data->power_down_disable)
		imx105_vreg_disable(imx105_pdev);

	return 0;
}
static int imx105_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	unsigned short chipidl, chipidh;
	pr_info("%s: %d\n", __func__, __LINE__);

	imx105_vreg_enable(imx105_pdev);

	/* switch PCLK and MCLK to Main cam */
	pr_info("imx105: imx105_sensor_probe: switch clk\n");
	if (data->camera_clk_switch != NULL)
		data->camera_clk_switch();

	data->pdata->camera_gpio_on();
	msm_camio_clk_rate_set(IMX105_DEFAULT_MASTER_CLK_RATE);
	mdelay(1);

	rc = gpio_request(data->sensor_reset, "imx105");
	if (!rc) {
		gpio_direction_output(data->sensor_reset, 1);
	} else {
		pr_err("[CAM]GPIO (%d) request failed\n", data->sensor_reset);
		goto init_probe_done;
	}
	gpio_free(data->sensor_reset);
	mdelay(1);

	pr_info(" imx105_probe_init_sensor is called \n");
	/* Read sensor Model ID: */
	rc = imx105_i2c_read(0x0000, &chipidh, 1);
	if (rc < 0) {
		pr_err(" imx105_probe_init_sensor 3 \n");
		goto init_probe_fail;
	}
	rc = imx105_i2c_read(0x0001, &chipidl, 1);
	if (rc < 0) {
		pr_err(" imx105_probe_init_sensor4 \n");
		goto init_probe_fail;
	}
	pr_info("imx105 model_id = 0x%x  0x%x\n", chipidh, chipidl);
	/* Compare sensor ID to IMX105 ID: */
	if (chipidh != 0x01 || chipidl != 0x05) {
		rc = -ENODEV;
		pr_err("imx105_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}
	goto init_probe_done;
init_probe_fail:
	CDBG(" imx105_probe_init_sensor fails\n");
	imx105_probe_init_done(data);
init_probe_done:
	CDBG(" imx105_probe_init_sensor finishes\n");
	return rc;
	}

static int imx105_i2c_read_fuseid(struct sensor_cfg_data *cdata)
{

	int32_t  rc;
	unsigned short info_value = 0, info_index = 0;
	unsigned short  OTP[10] = {0};

	pr_info("[CAM]%s: sensor OTP information:\n", __func__);

	for (info_index = 0; info_index < 10; info_index++) {
		rc = imx105_i2c_write_b_sensor(0x34C9, info_index);
		if (rc < 0)
			pr_info("[CAM]%s: i2c_write_b 0x34c9 (select info_index %d) fail\n", __func__, info_index);

		/* read Information 0~9 according to SPEC*/
		rc = imx105_i2c_read(0x3500, &info_value, 1);
		if (rc < 0)
			pr_info("[CAM]%s: i2c_read_b 0x3500 fail\n", __func__);

		OTP[info_index] = (short)(info_value);
		info_value = 0;
	}

	pr_info("[CAM]%s: VenderID=%x,LensID=%x,SensorID=%x%x\n", __func__,
		OTP[0], OTP[1], OTP[2], OTP[3]);
	pr_info("[CAM]%s: ModuleFuseID= %x%x%x%x%x%x\n", __func__,
		OTP[4], OTP[5], OTP[6], OTP[7], OTP[8], OTP[9]);

    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = 0;
	cdata->cfg.fuse.fuse_id_word3 = (OTP[0]);
	cdata->cfg.fuse.fuse_id_word4 =
		(OTP[4]<<20) |
		(OTP[5]<<16) |
		(OTP[6]<<12) |
		(OTP[7]<<8) |
		(OTP[8]<<4) |
		(OTP[9]);

	pr_info("[CAM]s5k3h2yx: fuse->fuse_id_word1:%d\n",
		cdata->cfg.fuse.fuse_id_word1);
	pr_info("[CAM]s5k3h2yx: fuse->fuse_id_word2:%d\n",
		cdata->cfg.fuse.fuse_id_word2);
	pr_info("[CAM]s5k3h2yx: fuse->fuse_id_word3:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word3);
	pr_info("[CAM]s5k3h2yx: fuse->fuse_id_word4:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word4);
	return 0;
}

/* camsensor_iu060f_imx105_reset */
int imx105_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	pr_info("Calling imx105_sensor_open_init\n");

  down(&imx105_sem);

  if (data == NULL) {
    pr_info("data is a NULL pointer\n");
    goto init_fail;
  }
	imx105_ctrl = (struct imx105_ctrl_t *)kzalloc(sizeof(struct imx105_ctrl_t), GFP_KERNEL);

	if (!imx105_ctrl) {
		rc = -ENOMEM;
		goto init_done;
	}
	imx105_ctrl->fps_divider = 1 * 0x00000400;
	imx105_ctrl->pict_fps_divider = 1 * 0x00000400;
	imx105_ctrl->fps = 30 * Q8;
	imx105_ctrl->set_test = TEST_OFF;
	imx105_ctrl->prev_res = QTR_SIZE;
	imx105_ctrl->pict_res = FULL_SIZE;
	imx105_ctrl->curr_res = INVALID_SIZE;
	config_csi = 0;

	if (data)
		imx105_ctrl->sensordata = data;
	CDBG("%s: %d\n", __func__, __LINE__);

	rc = imx105_probe_init_sensor(data);
	if (rc < 0) {
		CDBG("Calling imx105_sensor_open_init fail\n");
		goto init_fail;
	}
	CDBG("%s: %d\n", __func__, __LINE__);
	rc = imx105_sensor_setting(REG_INIT, RES_PREVIEW);
	CDBG("%s: %d\n", __func__, __LINE__);
	if (rc < 0)
		goto init_fail;
	rc = imx105_af_init();
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
init_fail:
	pr_err("imx105_sensor_open_init fail\n");
	pr_err("%s: %d\n", __func__, __LINE__);
	imx105_probe_init_done(data);
	kfree(imx105_ctrl);
init_done:
  up(&imx105_sem);
  pr_info("%s: init_done\n", __func__);
  return rc;

} /* end of imx105_sensor_open_init */

static int imx105_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&imx105_wait_queue);
	return 0;
}

static const struct i2c_device_id imx105_i2c_id[] = {
	{"imx105", 0},
	{ }
};

static int imx105_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("imx105_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	imx105_sensorw = kzalloc(sizeof(struct imx105_work_t), GFP_KERNEL);
	if (!imx105_sensorw) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, imx105_sensorw);
	imx105_init_client(client);
	imx105_client = client;

	msleep(50);

	pr_info("imx105_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("imx105_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit imx105_remove(struct i2c_client *client)
{
	struct imx105_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	deinit_suspend();
	imx105_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver imx105_i2c_driver = {
	.id_table = imx105_i2c_id,
	.probe  = imx105_i2c_probe,
	.remove = __exit_p(imx105_i2c_remove),
	.driver = {
		.name = "imx105",
	},
};


/*--------------------------------------------------------------*/
static int lens_info;
static void read_lens_info(void)
{
	int32_t  rc;
	unsigned short info_value = 0;
	lens_info = 0;

	pr_info("[CAM]%s\n", __func__);

	rc = imx105_i2c_write_b_sensor(0x34C9, 0x01);
	if (rc < 0) {
		pr_info("[CAM]%s: i2c_write_b 0x34C9 fail\n", __func__);
		goto read_lens_info_fail;
	}

	rc = imx105_i2c_read(0x3500, &info_value, 1);
	if (rc < 0) {
		pr_info("[CAM]%s: imx105_i2c_read 0x3500 fail\n", __func__);
		goto read_lens_info_fail;
	}

	if (info_value == 1)
		lens_info = 9535;
	else if (info_value == 2)
		lens_info = 9547;
	else
		lens_info = 0;

	pr_info("[CAM]%s  lens_info=%d\n", __func__, lens_info);

read_lens_info_fail :
	return;
}
/*--------------------------------------------------------------*/


static const char *IMX105Vendor = "sony";
static const char *IMX105NAME = "imx105";
static const char *IMX105Size = "8M";
static const char *IMX105MacroThreshold = "22";

static ssize_t sensor_vendor_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", IMX105Vendor, IMX105NAME, IMX105Size);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t sensor_read_node(struct device *dev,
  struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", sensor_probe_node);
	return length;
}

static ssize_t lens_info_show(struct device *dev,
  struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%d\n", lens_info);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t macro_threshold(struct device *dev,
  struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s\n", IMX105MacroThreshold);
	ret = strlen(buf) + 1;

	return ret;
}


static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);
static DEVICE_ATTR(lensinfo, 0444, lens_info_show, NULL);
static DEVICE_ATTR(macrothreshold, 0444, macro_threshold, NULL);


static struct kobject *android_imx105 = NULL;

static int imx105_sysfs_init(void)
{
	int ret = 0;
	pr_info("imx105:kobject creat and add\n");
	android_imx105 = kobject_create_and_add("android_camera", NULL);
	if (android_imx105 == NULL) {
		pr_info("imx105_sysfs_init: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("imx105:sysfs_create_file\n");

	ret = sysfs_create_file(android_imx105, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("imx105_sysfs_init: sysfs_create_file failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_imx105, &dev_attr_node.attr);
	if (ret) {
		pr_info("imx105_sysfs_init: dev_attr_node failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_imx105, &dev_attr_lensinfo.attr);
	if (ret) {
		pr_info("imx105_sysfs_init: dev_attr_node failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_imx105, &dev_attr_macrothreshold.attr);
	if (ret) {
		pr_info("imx105_sysfs_init: dev_attr_macrothreshold failed\n");
		ret = -EFAULT;
		goto error;
	}

	return ret;

error:
	kobject_del(android_imx105);
	return ret;
}


int imx105_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	down(&imx105_sem);
	CDBG("imx105_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
			imx105_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			imx105_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				imx105_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				imx105_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				imx105_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				imx105_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = imx105_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				imx105_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line,
					cdata.cfg.exp_gain.mul);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc = imx105_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line,
				cdata.cfg.exp_gain.mul);
			break;

		case CFG_SET_MODE:
			rc = imx105_set_sensor_mode(cdata.mode,
					cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = imx105_power_down();
			break;
		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = IMX105_STEPS_NEAR_TO_CLOSEST_INF;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		case CFG_MOVE_FOCUS:
			rc =
				imx105_move_focus(
				cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				imx105_set_default_focus();
			break;

		case CFG_I2C_IOCTL_R_OTP:{
		/* pr_info("[CAM]Line:%d CFG_I2C_IOCTL_R_OTP \n", __LINE__);*/
			rc = imx105_i2c_read_fuseid(&cdata);
			if (copy_to_user(argp, &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
			}
			break;

#if 0
		case CFG_SET_QCT_LSC_RAW_CAPTURE:
			pr_info("Line:%d : CFG_SET_QCT_LSC_RAW_CAPTURE \n", __LINE__);
			rc = imx105_QCT_LSC_calibration_set_rawflag();
			break;
#endif
		case CFG_SET_EFFECT:
		default:
			rc = -EFAULT;
			break;
		}

	prevent_suspend();
	up(&imx105_sem);

	return rc;
}




static int imx105_sensor_release(void)
{
	int rc = -EBADF;

	down(&imx105_sem);
	pr_info("%s, %d\n", __func__, __LINE__);

	imx105_power_down();

	imx105_probe_init_done(imx105_ctrl->sensordata);

	if(imx105_ctrl != NULL) {
		kfree(imx105_ctrl);
		imx105_ctrl = NULL;
	}

	allow_suspend();
	pr_info("imx105_release completed\n");
	up(&imx105_sem);
	
	return rc;
}

static int imx105_sensor_probe(struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	pr_info("imx105_sensor_probe()\n");

	rc = i2c_add_driver(&imx105_i2c_driver);
	if (rc < 0 || imx105_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}

	pr_info("imx105 s->node %d\n", s->node);
	sensor_probe_node = s->node;

	/* read sensor id */
	rc = imx105_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	/* read lens info */
	read_lens_info();

//  /* Initialize Sensor registers */
//  rc = initialize_imx105_registers();

	init_suspend();
	s->s_init 	 = imx105_sensor_open_init;
	s->s_release = imx105_sensor_release;
	s->s_config  = imx105_sensor_config;
	imx105_probe_init_done(info);
	imx105_sysfs_init();

	pr_info("%s: imx105_probe_init_done %d\n",  __func__, __LINE__);
	return rc;

probe_fail:
	pr_err("imx105_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static int __imx105_probe(struct platform_device *pdev)
{
	imx105_pdev = pdev;
	pr_info("imx105_probe\n");
	return msm_camera_drv_start(pdev, imx105_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __imx105_probe,
	.driver = {
		.name = "msm_camera_imx105",
		.owner = THIS_MODULE,
	},
};

static int __init imx105_init(void)
{
	pr_info("[CAM]imx105_init\n");
	return platform_driver_register(&msm_camera_driver);
}

module_init(imx105_init);

void imx105_exit(void)
{
	i2c_del_driver(&imx105_i2c_driver);
}


