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

#ifndef MT9D015_H
#define MT9D015_H
#include <linux/types.h>
#include <mach/board.h>
extern struct mt9d015_reg mt9d015_regs;

struct mt9d015_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

enum mt9d015_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum mt9d015_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};
enum mt9d015_setting {
	RES_PREVIEW,
	RES_CAPTURE
};
enum mt9d015_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum mt9d015_reg_mode {
	MT9D015_FINE_INTEGRATION_TIME,
	MT9D015_COARSE_INTEGRATION_TIME,
	FINE_CORRECTION,
	MT9D015_FRAME_LENGTH_LINES,
	MT9D015_LINE_LENGTH_PCK,
	MT9D015_X_ADDR_START,
	MT9D015_Y_ADDR_START,
	MT9D015_X_ADDR_END,
	MT9D015_Y_ADDR_END,
	MT9D015_X_OUTPUT_SIZE,
	MT9D015_Y_OUTPUT_SIZE,
	MT9D015_SCALING_MODE,
	MT9D015_SPATIAL_SAMPLING_BAYER,
	MT9D015_SCALE_M,
};

struct mt9d015_reg {
	const struct mt9d015_i2c_reg_conf *reg_mipi;
	const unsigned short reg_mipi_size;
	const struct mt9d015_i2c_reg_conf *rec_settings;
	const unsigned short rec_size;
	const struct mt9d015_i2c_reg_conf *reg_pll;
	const unsigned short reg_pll_size;
	const struct mt9d015_i2c_reg_conf *reg_prev;
	const unsigned short reg_prev_size;
	const struct mt9d015_i2c_reg_conf *reg_snap;
	const unsigned short reg_snap_size;
};
#endif /* MT9D015_H */
