/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include "s5k3h2yx.h"

struct s5k3h2yx_i2c_reg_conf s5k3h2yx_common_settings_array_mipi[] =
{
  /* Reset for operation */
  { 0x0100 , 0x00 }, /* stream off */

  /* MIPI Setting */
  { 0x3065 , 0x35 },
  { 0x310E , 0x00 },
  { 0x3098 , 0xAB },
  { 0x30C7 , 0x0A },
  { 0x309A , 0x01 },
  { 0x310D , 0xC6 },
  { 0x30C3 , 0x40 },
  { 0x30BB , 0x02 },
  { 0x30BC , 0x38 },
  { 0x30BD , 0x40 },
  { 0x3110 , 0x70 },
  { 0x3111 , 0x80 },
  { 0x3112 , 0x7B },
  { 0x3113 , 0xC0 },
  { 0x30C7 , 0x1A },

  /* Manufacture Setting */
  { 0x3000 , 0x08 },
  { 0x3001 , 0x05 },
  { 0x3002 , 0x0D },
  { 0x3003 , 0x21 },
  { 0x3004 , 0x62 },
  { 0x3005 , 0x0B },
  { 0x3006 , 0x6D },
  { 0x3007 , 0x02 },
  { 0x3008 , 0x62 },
  { 0x3009 , 0x62 },
  { 0x300A , 0x41 },
  { 0x300B , 0x10 },
  { 0x300C , 0x21 },
  { 0x300D , 0x04 },
  { 0x307E , 0x03 },
  { 0x307F , 0xA5 },
  { 0x3080 , 0x04 },
  { 0x3081 , 0x29 },
  { 0x3082 , 0x03 },
  { 0x3083 , 0x21 },
  { 0x3011 , 0x5F },
  { 0x3156 , 0xE2 },
  { 0x3027 , 0xBE }, // DBR_CLK enable for EMI Change from 8E to BE by COASIA 110317 Setting
  { 0x300f , 0x02 },
  { 0x3010 , 0x10 },
  { 0x3017 , 0x74 },
  { 0x3018 , 0x00 },
  { 0x3020 , 0x02 },
  { 0x3021 , 0x00 }, // EMI
  { 0x3023 , 0x80 },
  { 0x3024 , 0x08 },
  { 0x3025 , 0x08 },
  { 0x301C , 0xD4 },
  { 0x315D , 0x00 },
//  { 0x3053 , 0xCF },
  { 0x3054 , 0x00 },
  { 0x3055 , 0x35 },
  { 0x3062 , 0x04 },
  { 0x3063 , 0x38 },
  { 0x31A4 , 0x04 },
  { 0x3016 , 0x54 },
  { 0x3157 , 0x02 },
  { 0x3158 , 0x00 },
  { 0x315B , 0x02 },
  { 0x315C , 0x00 },
  { 0x301B , 0x05 },
  { 0x3028 , 0x41 },
  { 0x302A , 0x10 },  //20100503
  { 0x3060 , 0x00 },
  { 0x302D , 0x19 },
  { 0x302B , 0x05 },
  { 0x3072 , 0x13 },
  { 0x3073 , 0x21 },
  { 0x3074 , 0x82 },
  { 0x3075 , 0x20 },
  { 0x3076 , 0xA2 },
  { 0x3077 , 0x02 },
  { 0x3078 , 0x91 },
  { 0x3079 , 0x91 },
  { 0x307A , 0x61 },
  { 0x307B , 0x28 },
  { 0x307C , 0x31 },

  /* black level =64 @ 10bit */
  { 0x304E , 0x40 }, //Pedestal
  { 0x304F , 0x01 }, //Pedestal
  { 0x3050 , 0x00 }, //Pedestal
  { 0x3088 , 0x01 }, //Pedestal
  { 0x3089 , 0x00 }, //Pedestal
  { 0x3210 , 0x01 }, //Pedestal
  { 0x3211 , 0x00 }, //Pedestal
  { 0x308E , 0x01 }, //Add register by COASIA 110512 setting.
  { 0x308F , 0x8F }, //Add register by COASIA 110322 setting.
  { 0x3064 , 0x03 }, //Add register by COASIA 110331 setting.
  { 0x31A7 , 0x0F }, //Add register by COASIA 110331 setting.
  
  { 0x30C9 , 0x03 }, //Dphy_m_Ctrl: 0x00 : Default, 0x01 : 10% Up, 0x02 : 30% Up, 0x03 : 30% Up
};

struct s5k3h2yx_i2c_reg_conf s5k3h2yx_qtr_settings_array_mipi[] =
{
  /* PLL Setting 648Mbps */
  { 0x0305 , 0x04 }, /* pre_pll_clk_div = 4 */
  { 0x0306 , 0x00 }, /* pll_multiplier */
  { 0x0307 , 0x6C }, /* pll_multiplier = 108 */ // For 648Mbps
//  { 0x0307 , 0x98 }, /* pll_multiplier = 152 */ // For 912Mbps
  { 0x0303 , 0x01 }, /* vt_sys_clk_div = 1 */
  { 0x0301 , 0x05 }, /* vt_pix_clk_div = 5 */
  { 0x030B , 0x01 }, /* op_sys_clk_div = 1 */
  { 0x0309 , 0x05 }, /* op_pix_clk_div = 5 */
  { 0x30CC , 0xB0 }, /* DPHY_band_ctrl 640 ~ 690Mbps */ // For 648Mbps
//  { 0x30CC , 0xE0 }, /* DPHY_band_ctrl 870 ~ 950Mbps */ // For 912Mbps
  { 0x31A1 , 0x58 }, // EMI control For 648Mbps Change from 56 to 58 by COASIA 110317 Setting

  { 0x0344 , 0x00 }, /* X addr start 0d */
  { 0x0345 , 0x00 },
  { 0x0346 , 0x00 }, /* Y addr start 0d */
  { 0x0347 , 0x00 },
  { 0x0348 , 0x0C }, /* X addr end 3277d */
  { 0x0349 , 0xCD },
  { 0x034A , 0x09 }, /* Y addr end 2463d */
  { 0x034B , 0x9F },
  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x03 }, /* x_odd_inc = 3 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x03 }, /* y_odd_inc = 3 */

  { 0x0105 , 0x01 }, /*Set at init skip corrupted frame - for preview flash when doing hjr af */

  { 0x0401 , 0x00 }, /* Derating_en  = 0 (disable) */
  { 0x0405 , 0x10 },
  { 0x0700 , 0x05 }, /* fifo_water_mark_pixels = 1328 */
  { 0x0701 , 0x30 },

  { 0x034C , 0x06 }, /* x_output_size = 1640 */
  { 0x034D , 0x68 },
  { 0x034E , 0x04 }, /* y_output_size = 1232 */
  { 0x034F , 0xD0 },

  { 0x0200 , 0x02 }, /* fine integration time */
  { 0x0201 , 0x50 },
  { 0x0202 , 0x04 }, /* Coarse integration time */
  { 0x0203 , 0xDB },
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck 3470d */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x04 }, /* Frame_length_lines 1248d */
  { 0x0341 , 0xE0 },

  { 0x300E , 0x2D }, /* Reserved */
  { 0x31A3 , 0x40 }, /* Reserved */
  { 0x301A , 0x77 }, /* Reserved */ // For 648Mbps
  { 0x3053 , 0xCF }, /* CF for full , preview size.  CB for HD/FHD/QVGA120fps */
};

struct s5k3h2yx_i2c_reg_conf s5k3h2yx_video_settings_array_mipi[] =
{
  /* PLL Setting 912Mbps */
  { 0x0305 , 0x04 }, /* pre_pll_clk_div = 4 */
  { 0x0306 , 0x00 }, /* pll_multiplier */
  { 0x0307 , 0x98 }, /* pll_multiplier = 152 */ // For 912Mbps
  { 0x0303 , 0x01 }, /* vt_sys_clk_div = 1 */
  { 0x0301 , 0x05 }, /* vt_pix_clk_div = 5 */
  { 0x030B , 0x01 }, /* op_sys_clk_div = 1 */
  { 0x0309 , 0x05 }, /* op_pix_clk_div = 5 */
  { 0x30CC , 0xE0 }, /* DPHY_band_ctrl 870 ~ 950Mbps */ // For 912Mbps
  { 0x31A1 , 0x5A }, //11032 COASIA change from 56 to 5A //"DBR_CLK = PLL_CLK / DIV_DBR(0x31A1[3:0])

  // For 912Mbps 1080P setting.
  { 0x0344 , 0x00 }, /* X addr start 96d */
  { 0x0345 , 0x60 },
  { 0x0346 , 0x01 }, /* Y addr start 364d */
  { 0x0347 , 0x6C },
  { 0x0348 , 0x0C }, /* X addr end 3181d */
  { 0x0349 , 0x6D },
  { 0x034A , 0x08 }, /* Y addr end 2099d */
  { 0x034B , 0x33 },

  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x01 }, /* x_odd_inc = 1 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x01 }, /* y_odd_inc = 1 */

  { 0x0105 , 0x01 }, /*Set at init skip corrupted frame - for preview flash when doing hjr af */

  { 0x0401 , 0x00 }, /* Derating_en  = 0 (disable) */
  { 0x0405 , 0x10 },
  { 0x0700 , 0x05 }, /* fifo_water_mark_pixels = 1328 */
  { 0x0701 , 0x30 },

  { 0x034C , 0x0C }, /* x_output_size = 3084 */
  { 0x034D , 0x0C },
  { 0x034E , 0x06 }, /* y_output_size = 1736 */
  { 0x034F , 0xC8 },

  { 0x0200 , 0x02 }, /* fine integration time */
  { 0x0201 , 0x50 },
  { 0x0202 , 0x04 }, /* Coarse integration time */
  { 0x0203 , 0xDB },
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck 3470d */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x06 }, /* Frame_length_lines 1752d */
  { 0x0341 , 0xD8 },

  { 0x300E , 0x29 }, /* Reserved */
  { 0x31A3 , 0x00 }, /* Reserved */
  { 0x301A , 0xA7 }, /* Reserved */
  { 0x3053 , 0xCB }, /* CF for full , preview size.  CB for HD/FHD/QVGA120fps */
};

struct s5k3h2yx_i2c_reg_conf s5k3h2yx_full_settings_array_mipi[] =
{
  /* PLL Setting 648Mbps */
  { 0x0305 , 0x04 }, /* pre_pll_clk_div = 4 */
  { 0x0306 , 0x00 }, /* pll_multiplier */
  { 0x0307 , 0x6C }, /* pll_multiplier = 108 */ // For 648Mbps
//  { 0x0307 , 0x98 }, /* pll_multiplier = 152 */ // For 912Mbps
  { 0x0303 , 0x01 }, /* vt_sys_clk_div = 1 */
  { 0x0301 , 0x05 }, /* vt_pix_clk_div = 5 */
  { 0x030B , 0x01 }, /* op_sys_clk_div = 1 */
  { 0x0309 , 0x05 }, /* op_pix_clk_div = 5 */
  { 0x30CC , 0xB0 }, /* DPHY_band_ctrl 640 ~ 690Mbps */ // For 648Mbps
//  { 0x30CC , 0xE0 }, /* DPHY_band_ctrl 870 ~ 950Mbps */ // For 912Mbps
  { 0x31A1 , 0x58 }, // For 648Mbps EMI control Change from 56 to 58 by COASIA 110317 Setting
//  { 0x31A1 , 0x5A }, // For 912Mbps

  { 0x0344 , 0x00 }, /* X addr start 0d */
  { 0x0345 , 0x00 },
  { 0x0346 , 0x00 }, /* Y addr start 0d */
  { 0x0347 , 0x00 },
  { 0x0348 , 0x0C }, /* X addr end 3279d */
  { 0x0349 , 0xCF },
  { 0x034A , 0x09 }, /* Y addr end 2463d */
  { 0x034B , 0x9F },

  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x01 }, /* x_odd_inc = 1 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x01 }, /* y_odd_inc = 1 */

  { 0x0105 , 0x01 }, /*Set at init skip corrupted frame - for preview flash when doing hjr af */

  { 0x0401 , 0x00 }, /* Derating_en  = 0 (disable) */
  { 0x0405 , 0x10 },
  { 0x0700 , 0x05 }, /* fifo_water_mark_pixels = 1328 */
  { 0x0701 , 0x30 },

  { 0x034C , 0x0C }, /* x_output_size = 3280 */
  { 0x034D , 0xD0 },
  { 0x034E , 0x09 }, /* y_output_size = 2464 */
  { 0x034F , 0xA0 },

  { 0x0200 , 0x02 }, /* fine integration time */
  { 0x0201 , 0x50 },
  { 0x0202 , 0x04 }, /* Coarse integration time */
  { 0x0203 , 0xE7 },
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck 3470d */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x09 }, /* Frame_length_lines 2480d */
  { 0x0341 , 0xB0 },

  { 0x300E , 0x29 }, /* Reserved */ // Change 648Mbps from E9 to 29 by COASIA 110315 Setting
  { 0x31A3 , 0x00 }, /* Reserved */
  { 0x301A , 0x77 }, /* Reserved */ //For 648Mbps
  { 0x3053 , 0xCF }, /* CF for full , preview size.  CB for HD/FHD/QVGA120fps */
//  { 0x301A , 0xA7 }, /* Reserved */ //For 912Mbps
};

struct s5k3h2yx_reg_t s5k3h2yx_regs = {
	.common_mipi = &s5k3h2yx_common_settings_array_mipi[0],
	.common_mipi_size = ARRAY_SIZE(s5k3h2yx_common_settings_array_mipi),

	.qtr_mipi = &s5k3h2yx_qtr_settings_array_mipi[0],
	.qtr_mipi_size = ARRAY_SIZE(s5k3h2yx_qtr_settings_array_mipi),

	.video_mipi = &s5k3h2yx_video_settings_array_mipi[0],
	.video_mipi_size = ARRAY_SIZE(s5k3h2yx_video_settings_array_mipi),

	.full_mipi = &s5k3h2yx_full_settings_array_mipi[0],
	.full_mipi_size = ARRAY_SIZE(s5k3h2yx_full_settings_array_mipi),
};
