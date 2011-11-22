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


#include "mt9d015.h"

struct mt9d015_i2c_reg_conf mt9d015_mipi_settings[] = {
	/*Disable embedded data*/
	{0x3064, 0x0805},
	{0x31BE, 0xC007}, /* from Micro FAE */
};

/*PLL Configuration
16MHz_ExtCLK 91MHz_vt_pix_clk 64MHz_pixclk */
struct mt9d015_i2c_reg_conf mt9d015_pll_settings[] = {
	/*set data to RAW10 format*/
	{0x0112, 0x0A0A},/*CCP2_DATA_FORMAT*/
	{0x0300, 0x0007},/*VT_PIX_CLK_DIV*/
	{0x0302, 0x0001},/*VT_SYS_CLK_DIV*/
	{0x0304, 0x0002},/*PRE_PLL_CLK_DIV*/
	{0x0306, 0x0038},/*(old: 0x0050) PLL_MULTIPLIER*/
	{0x0308, 0x000A},/*OP_PIX_CLK_DIV*/
	{0x030A, 0x0001},/*OP_SYS_CLK_DIV*/
};

struct mt9d015_i2c_reg_conf mt9d015_prev_settings[] = {
	/*Timing configuration*/
	{0x0200, 0x01E5},/*FINE_INTEGRATION_TIME_*/
	{0x0202, 0x0543},/*COARSE_INTEGRATION_TIME*/
	{0x3010, 0x0094},/*FINE_CORRECTION*//*0426 by Micro FAE*/
	{0x0340, 0x054B},/*FRAME_LENGTH_LINES*/
	{0x0342, 0x096C},/*LINE_LENGTH_PCK*/
	/*Output Size (1612x1208)*/
	{0x0344, 0x0000},/*X_ADDR_START*/
	{0x0346, 0x0000},/*Y_ADDR_START*/
	{0x0348, 0x064B},/*X_ADDR_END*/
	{0x034A, 0x04B7},/*Y_ADDR_END*/
	{0x034C, 0x064C},/*X_OUTPUT_SIZE*/
	{0x034E, 0x04B8},/*Y_OUTPUT_SIZE*/
	{0x0400, 0x0000},/*SCALING_MODE*/
	{0x0402, 0x0000},/*SPATIAL_SAMPLING_BAYER*/
	{0x0404, 0x0010},/*SCALE_M*/
};

#if 1
struct mt9d015_i2c_reg_conf mt9d015_snap_settings[] = {
	/*Timing configuration*/
	{0x0200, 0x01E5},/*FINE_INTEGRATION_TIME_*/
	{0x0202, 0x0543},/*COARSE_INTEGRATION_TIME*/
	{0x3010, 0x0094},/*FINE_CORRECTION*//*0426 by Micro FAE*/
	{0x0340, 0x054B},/*FRAME_LENGTH_LINES*/
	{0x0342, 0x096C},/*LINE_LENGTH_PCK*/
	/*Output Size (1612x1208*/
	{0x0344, 0x0000},/*X_ADDR_START*/
	{0x0346, 0x0000},/*Y_ADDR_START*/
	{0x0348, 0x064B},/*X_ADDR_END*/
	{0x034A, 0x04B7},/*Y_ADDR_END*/
	{0x034C, 0x064C},/*X_OUTPUT_SIZE*/
	{0x034E, 0x04B8},/*Y_OUTPUT_SIZE*/
	{0x0400, 0x0000},/*SCALING_MODE*/
	{0x0402, 0x0000},/*SPATIAL_SAMPLING_BAYER*/
	{0x0404, 0x0010},/*SCALE_M*/
};

#else
/* not modify */
struct mt9d015_i2c_reg_conf mt9d015_snap_settings[] = {
	/*Timing configuration*/
	{0x0200, 0x01E5},/*FINE_INTEGRATION_TIME_*/
	{0x0202, 0x0503},/*COARSE_INTEGRATION_TIME*/
	{0x0340, 0x0503},/*FRAME_LENGTH_LINES*/
	{0x0342, 0x0938},/*LINE_LENGTH_PCK*/
	/*Output Size (1280x720)*/
	{0x0344, 0x0004},/*X_ADDR_START*/
	{0x0346, 0x00A0},/*Y_ADDR_START*/
	{0x0348, 0x0643},/*X_ADDR_END*/
	{0x034A, 0x0423},/*Y_ADDR_END*/
	{0x034C, 0x0500},/*X_OUTPUT_SIZE*/
	{0x034E, 0x02D0},/*Y_OUTPUT_SIZE*/
	{0x0400, 0x0002},/*SCALING_MODE*/
	{0x0402, 0x0000},/*SPATIAL_SAMPLING_BAYER*/
	{0x0404, 0x0014},/*SCALE_M*/
};
#endif

struct mt9d015_i2c_reg_conf mt9d015_recommend_settings[] = {
	{0x3E00, 0x0430},
	{0x3E02, 0x3FFF},
	{0x3E1E, 0x67CA},
	{0x3E2A, 0xCA67},
	{0x3E2E, 0x8054},
	{0x3E30, 0x8255},
	{0x3E32, 0x8410},
	{0x3E36, 0x5FB0},
	{0x3E38, 0x4C82},
	{0x3E3A, 0x4DB0},
	{0x3E3C, 0x5F82},
	{0x3E3E, 0x1170},
	{0x3E40, 0x8055},
	{0x3E42, 0x8061},
	{0x3E44, 0x68D8},
	{0x3E46, 0x6882},
	{0x3E48, 0x6182},
	{0x3E4A, 0x4D82},
	{0x3E4C, 0x4C82},
	{0x3E4E, 0x6368},
	{0x3E50, 0xD868},
	{0x3E52, 0x8263},
	{0x3E54, 0x824D},
	{0x3E56, 0x8203},
	{0x3E58, 0x9D66},
	{0x3E5A, 0x8045},
	{0x3E5C, 0x4E7C},
	{0x3E5E, 0x0970},
	{0x3E60, 0x8072},
	{0x3E62, 0x5484},
	{0x3E64, 0x2037},
	{0x3E66, 0x8216},
	{0x3E68, 0x0486},
	{0x3E6A, 0x1070},
	{0x3E6C, 0x825E},
	{0x3E6E, 0xEE54},
	{0x3E70, 0x825E},
	{0x3E72, 0x8212},
	{0x3E74, 0x7086},
	{0x3E76, 0x1404},
	{0x3E78, 0x8220},
	{0x3E7A, 0x377C},
	{0x3E7C, 0x6170},
	{0x3E7E, 0x8082},
	{0x3E80, 0x4F82},
	{0x3E82, 0x4E82},
	{0x3E84, 0x5FCA},
	{0x3E86, 0x5F82},
	{0x3E88, 0x4E82},
	{0x3E8A, 0x4F81},
	{0x3E8C, 0x7C7F},
	{0x3E8E, 0x7000},
	{0x30D4, 0xE200}, /*{0x30D4, 0xC200},*/
	{0x3174, 0x8000},
	{0x3EE0, 0x0020},
	{0x3EE2, 0x0016},
	{0x3F00, 0x0002},
	{0x3F02, 0x0028},
	{0x3F0A, 0x0300},
	{0x3F0C, 0x1008},
	{0x3F10, 0x0405},
	{0x3F12, 0x0101}, /*{0x3F12, 0x021E},*/
	{0x3F14, 0x0000},
};

struct mt9d015_reg mt9d015_regs = {
	.reg_mipi = &mt9d015_mipi_settings[0],
	.reg_mipi_size = ARRAY_SIZE(mt9d015_mipi_settings),
	.rec_settings = &mt9d015_recommend_settings[0],
	.rec_size = ARRAY_SIZE(mt9d015_recommend_settings),
	.reg_pll = &mt9d015_pll_settings[0],
	.reg_pll_size = ARRAY_SIZE(mt9d015_pll_settings),
	.reg_prev = &mt9d015_prev_settings[0],
	.reg_prev_size = ARRAY_SIZE(mt9d015_prev_settings),
	.reg_snap = &mt9d015_snap_settings[0],
	.reg_snap_size = ARRAY_SIZE(mt9d015_snap_settings),
};
