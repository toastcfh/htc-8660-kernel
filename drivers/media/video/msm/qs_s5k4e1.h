/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef QS_S5K4E1_H
#define QS_S5K4E1_H
#include <linux/types.h>
#include <mach/board.h>
extern struct qs_s5k4e1_reg qs_s5k4e1_regs;

#define LENS_SHADE_TABLE 16
#define FOUR_LANE_SETTING

struct qs_s5k4e1_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

struct qs_s5k4e1_i2c_conf_array {
       struct qs_s5k4e1_i2c_reg_conf *conf;
       unsigned short size;
};

enum qs_s5k4e1_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum qs_s5k4e1_resolution_t {
	QTR_2D_SIZE,
	FULL_2D_SIZE,
	QTR_3D_SIZE,
	FULL_3D_SIZE,
	INVALID_SIZE
};
enum qs_s5k4e1_setting {
	RES_PREVIEW,
	RES_CAPTURE,
	RES_3D_PREVIEW,
	RES_3D_CAPTURE
};
enum qs_s5k4e1_cam_mode_t {
    MODE_2D_RIGHT,
	MODE_2D_LEFT,
	MODE_3D,
	MODE_INVALID
};
enum qs_s5k4e1_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum qs_s5k4e1_reg_mode {
	QS_S5K4E1_FRAME_LENGTH_LINES_H = 1,
	QS_S5K4E1_FRAME_LENGTH_LINES_L,
	QS_S5K4E1_LINE_LENGTH_PCK_H,
	QS_S5K4E1_LINE_LENGTH_PCK_L,
};

struct qs_s5k4e1_reg {
	const struct qs_s5k4e1_i2c_reg_conf *rec_settings;
	const unsigned short rec_size;
	const struct qs_s5k4e1_i2c_reg_conf *reg_prev;
	const unsigned short reg_prev_size;
	const struct qs_s5k4e1_i2c_reg_conf *reg_snap;
	const unsigned short reg_snap_size;
	const struct qs_s5k4e1_i2c_reg_conf (*reg_lens)[LENS_SHADE_TABLE];
	const unsigned short reg_lens_size;
	const struct qs_s5k4e1_i2c_reg_conf *reg_default_lens;
	const unsigned short reg_default_lens_size;
	const struct qs_s5k4e1_i2c_conf_array *conf_array;
};
#endif /* QS_S5K4E1_H */
