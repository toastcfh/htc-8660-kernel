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
#include "mipi_novatek.h"

//#define HTC_USED_0_3_MIPI_INIT
//#define MIPI_READ_DISPLAY_ID	1
// -----------------------------------------------------------------------------
//                             Constant value define
// -----------------------------------------------------------------------------
#ifndef HTC_USED_0_3_MIPI_INIT
const char NOVATEK_SONY_C3_MIPI_INIT[] = {
	3,
	DTYPE_DCS_WRITE, 0x11, 0x00,

	1, 120, // Wait time...

	3,
	DTYPE_DCS_WRITE, 0x29, 0x00,

	1, 40, // Wait time...

	3,
	DTYPE_DCS_WRITE1, 0x51, 0x00,
	3,
	DTYPE_DCS_WRITE1, 0x53, 0x24,
	3,
	DTYPE_DCS_WRITE1, 0x55, 0x00,
	3,
	DTYPE_DCS_WRITE1, 0x5E, 0x00,
	0,
};

static struct dsi_cmd_desc NOVATEK_SONY_MIPI_INIT[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100, 5, (char[]){0xFF, 0xAA, 0x55, 0x25, 0x01} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 39, (char[]){0xF8, 0x02, 0x02, 0x00, 0x20, 0x33, 0x00, 0x00, 0x40, 0x00, 0x00, 0x23, 0x02, 0x99, 0xC8, 0x02, 0x00, 0x00, 0x00, 0x00, 0x23, 0x0F, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x05, 0x05, 0x00, 0x34, 0x00, 0x04, 0x55, 0x00, 0x04, 0x11, 0x38} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 9, (char[]){0xF3, 0x00, 0x32, 0x00, 0x38, 0x31, 0x08, 0x11, 0x00} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 6, (char[]){0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3, (char[]){0xB1, 0xEC, 0x0A} },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xB6, 0x07} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3, (char[]){0xB7, 0x00, 0x73} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 5, (char[]){0xB8, 0x01, 0x04, 0x06, 0x05} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3, (char[]){0xB9, 0x00, 0x00} },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xBA, 0x02} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xBB, 0x88, 0x08, 0x88} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xBC, 0x01, 0x01, 0x01} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 6, (char[]){0xBD, 0x01, 0x84, 0x1D, 0x1C, 0x00} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 6, (char[]){0xBE, 0x01, 0x84, 0x1D, 0x1C, 0x00} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 6, (char[]){0xBF, 0x01, 0x84, 0x1D, 0x1C, 0x00} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 13, (char[]){0xCA, 0x01, 0xE4, 0xE4, 0xE4, 0xE4, 0xE4, 0xE4, 0xE4, 0x08, 0x08, 0x00, 0x01} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3, (char[]){0xE0, 0x01, 0x03} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 6, (char[]){0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB0, 0x0A, 0x0A, 0x0A} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB1, 0x0A, 0x0A, 0x0A} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB2, 0x02, 0x02, 0x02} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB3, 0x08, 0x08, 0x08} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB5, 0x05, 0x05, 0x05} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB6, 0x45, 0x45, 0x45} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB7, 0x25, 0x25, 0x25} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB8, 0x36, 0x36, 0x36} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xB9, 0x36, 0x36, 0x36} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4, (char[]){0xBA, 0x15, 0x15, 0x15} },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xBF, 0x01} },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xC2, 0x01} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 8, (char[]){0xCE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} },
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, 2, (char[]){0x11, 0x00} },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0x2C, 0x00} },
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0x35, 0x00} },
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3, (char[]){0x44, 0x02, 0xCF} },
//	{DTYPE_DCS_WRITE, 1, 0, 0, 40, 2, (char[]){0x28, 0x00} },
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0x51, 0x00} },
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0x55, 0x00} },
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0x5E, 0x00} },
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0x53, 0x24} },
};
#endif //HTC_USED_0_3_MIPI_INIT


// -----------------------------------------------------------------------------
//                         External routine declaration
// -----------------------------------------------------------------------------
extern int mipi_status;
#define DEFAULT_BRIGHTNESS 83
extern int bl_level_prevset;
extern struct mutex cmdlock;
extern struct dsi_cmd_desc *mipi_power_on_cmd;
extern struct dsi_cmd_desc *mipi_power_off_cmd;
extern int mipi_power_on_cmd_size;
extern int mipi_power_off_cmd_size;
extern char ptype[60];

static struct msm_panel_common_pdata *mipi_novatek_pdata;

static struct dsi_buf novatek_tx_buf;
static struct dsi_buf novatek_rx_buf;


/* novatek blue panel */
// &*&*&*PM
static char led_pwm1[] = 
{
	0x51, 0x0,
};

static char led_pwm2[] = 
{
	0x53, 0x24,
};

static char led_pwm3[] = 
{
	0x55, 0x00,
};
// &*&*&*PM
static unsigned char bkl_enable_cmds[] = {0x53, 0x24};/* DTYPE_DCS_WRITE1 *///bkl on and no dim
static unsigned char bkl_disable_cmds[] = {0x53, 0x00};/* DTYPE_DCS_WRITE1 *///bkl off

#ifdef NOVETAK_COMMANDS_UNUSED
static char display_config_cmd_mode1[] = {
	/* TYPE_DCS_LWRITE */
	0x2A, 0x00, 0x00, 0x01,
	0x3F, 0xFF, 0xFF, 0xFF
};

static char display_config_cmd_mode2[] = {
	/* DTYPE_DCS_LWRITE */
	0x2B, 0x00, 0x00, 0x01,
	0xDF, 0xFF, 0xFF, 0xFF
};

static char display_config_cmd_mode3_666[] = {
	/* DTYPE_DCS_WRITE1 */
	0x3A, 0x66, 0x15, 0x80 /* 666 Packed (18-bits) */
};

static char display_config_cmd_mode3_565[] = {
	/* DTYPE_DCS_WRITE1 */
	0x3A, 0x55, 0x15, 0x80 /* 565 mode */
};

static char display_config_321[] = {
	/* DTYPE_DCS_WRITE1 */
	0x66, 0x2e, 0x15, 0x00 /* Reg 0x66 : 2E */
};

static char display_config_323[] = {
	/* DTYPE_DCS_WRITE */
	0x13, 0x00, 0x05, 0x00 /* Reg 0x13 < Set for Normal Mode> */
};

static char display_config_2lan[] = {
	/* DTYPE_DCS_WRITE */
	0x61, 0x01, 0x02, 0xff /* Reg 0x61 : 01,02 < Set for 2 Data Lane > */
};

static char display_config_exit_sleep[] = {
	/* DTYPE_DCS_WRITE */
	0x11, 0x00, 0x05, 0x80 /* Reg 0x11 < exit sleep mode> */
};

static char display_config_TE_ON[] = {
	/* DTYPE_DCS_WRITE1 */
	0x35, 0x00, 0x15, 0x80
};

static char display_config_39H[] = {
	/* DTYPE_DCS_WRITE */
	0x39, 0x00, 0x05, 0x80
};

static char display_config_set_tear_scanline[] = {
	/* DTYPE_DCS_LWRITE */
	0x44, 0x02, 0xcf, 0xff
};

static char display_config_set_twolane[] = {
	/* DTYPE_DCS_WRITE1 */
	0xae, 0x03, 0x15, 0x80
};

static char display_config_set_threelane[] = {
	/* DTYPE_DCS_WRITE1 */
	0xae, 0x05, 0x15, 0x80
};


#else

static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
static char enable_te[2] = {0x35, 0x00};/* DTYPE_DCS_WRITE1 */
static char test_reg[3] = {0x44, 0x02, 0xCF};/* DTYPE_DCS_WRITE1 */
static char test_reg_qhd[3] = {0x44, 0x01, 0x3f};/* DTYPE_DCS_LWRITE */
//static char pixel_off[2] = {0x22, 0x00}; /* DTYPE_DCS_WRITE */
//static char normal_on[2] = {0x13, 0x00}; /* DTYPE_DCS_WRITE */
//static char set_onelane[2] = {0xae, 0x01}; /* DTYPE_DCS_WRITE1 */
static char set_twolane[2] = {0xae, 0x03}; /* DTYPE_DCS_WRITE1 */
static char rgb_888[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */
/* commands by Novatke */
static char novatek_fd[2] = {0xfd, 0x01}; /* DTYPE_DCS_WRITE */
static char novatek_eq1200[4] = {0x92, 0x00, 0xA2, 0x4C}; /* DTYPE_DCS_WRITE */
static char novatek_2vci[2] = {0x03, 0x33}; /* DTYPE_DCS_WRITE */
static char novatek_f3[2] = {0xF3, 0xAA }; /* DTYPE_DCS_WRITE1 */
static char novatek_ff_aa[2] = {0xff, 0xAA }; /* DTYPE_DCS_WRITE1 */

static char novatek_f4[2] = {0xf4, 0x55}; /* DTYPE_DCS_WRITE1 */
static char novatek_8c[16] = { /* DTYPE_DCS_LWRITE */
	0x8C, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x08, 0x08, 0x00, 0x30, 0xC0, 0xB7, 0x37};
static char novatek_ff[2] = {0xff, 0x55 }; /* DTYPE_DCS_WRITE1 */

static char set_width[5] = { /* DTYPE_DCS_LWRITE */
	0x2A, 0x00, 0x00, 0x02, 0x1B}; /* 540 - 1 */
static char set_height[5] = { /* DTYPE_DCS_LWRITE */
	0x2B, 0x00, 0x00, 0x03, 0xBF}; /* 960 - 1 */

static char novatek_pwm_f3[2] = {0xF3, 0xAA }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_00[2] = {0x00, 0x01 }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_21[2] = {0x21, 0x20 }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_22[2] = {0x22, 0x03 }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_7d[2] = {0x7D, 0x01 }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_7f[2] = {0x7F, 0xAA }; /* DTYPE_DCS_WRITE1 */

static char novatek_pwm_cp[2] = {0x09, 0x34 }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_cp2[2] = {0xc9, 0x01 }; /* DTYPE_DCS_WRITE1 */
static char novatek_pwm_cp3[2] = {0xff, 0xaa }; /* DTYPE_DCS_WRITE1 */

//static char novatek_5e[2] = {0x5E, 0x00}; /* DTYPE_DCS_WRITE1 */
static char maucctr_0[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};/* DTYPE_DCS_LWRITE */
static char maucctr_1[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}; /* DTYPE_DCS_LWRITE */
static char novatek_b5x[4] = {0xB5, 0x05, 0x05, 0x05}; /* DTYPE_DCS_LWRITE */
static char novatek_ce[8] = {0xCE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* DTYPE_DCS_LWRITE */
static char novatek_e0[3] = {0xE0, 0x01, 0x03}; /* DTYPE_DCS_LWRITE */

#ifdef HTC_USED_0_3_MIPI_INIT
/* NT35510 init command */

static char novatek_b1[3] = {0xB1, 0xEC, 0x0A}; /* DTYPE_DCS_LWRITE */
static char novatek_b5[2] = {0xB5, 0x50}; /* DTYPE_DCS_WRITE1 */
static char novatek_b6[2] = {0xB6, 0x07}; /* DTYPE_DCS_WRITE1 */
static char novatek_b7[3] = {0xB7, 0x00, 0x73}; /* DTYPE_DCS_LWRITE */
static char novatek_b8[5] = {0xB8, 0x01, 0x04, 0x06, 0x05}; /* DTYPE_DCS_LWRITE */
static char novatek_b9[3] = {0xB9, 0x00, 0x00}; /* DTYPE_DCS_LWRITE */
static char novatek_ba[2] = {0xBA, 0x02}; /* DTYPE_DCS_WRITE1 */
static char novatek_bb[4] = {0xBB, 0x88, 0x08, 0x88}; /* DTYPE_DCS_LWRITE */
static char novatek_bc[4] = {0xBC, 0x01, 0x01, 0x01}; /* DTYPE_DCS_LWRITE */
static char novatek_bd[6] = {0xBD, 0x01, 0x84, 0x1D, 0x1C, 0x00}; /* DTYPE_DCS_LWRITE */
static char novatek_be[6] = {0xBE, 0x01, 0x84, 0x1D, 0x1C, 0x00}; /* DTYPE_DCS_LWRITE */
static char novatek_bf[6] = {0xBF, 0x01, 0x84, 0x1D, 0x1C, 0x00}; /* DTYPE_DCS_LWRITE */
static char novatek_ca[13] = { /* DTYPE_DCS_LWRITE */
	0xCA, 0x01, 0xE4, 0xE4, 0xE4, 0xE4, 0xE4,
	0xE4, 0xE4, 0x08, 0x08, 0x00, 0x01};

static char novatek_b0x[4] = {0xB0, 0x0A, 0x0A, 0x0A}; /* DTYPE_DCS_LWRITE */
static char novatek_b1x[4] = {0xB1, 0x0A, 0x0A, 0x0A}; /* DTYPE_DCS_LWRITE */
static char novatek_b2x[4] = {0xB2, 0x02, 0x02, 0x02}; /* DTYPE_DCS_LWRITE */
static char novatek_b3x[4] = {0xB3, 0x08, 0x08, 0x08}; /* DTYPE_DCS_LWRITE */
static char novatek_b6x[4] = {0xB6, 0x44, 0x44, 0x44}; /* DTYPE_DCS_LWRITE */
static char novatek_b7x[4] = {0xB7, 0x24, 0x24, 0x24}; /* DTYPE_DCS_LWRITE */
static char novatek_b8x[4] = {0xB8, 0x36, 0x36, 0x36}; /* DTYPE_DCS_LWRITE */
static char novatek_b9x[4] = {0xB9, 0x36, 0x36, 0x36}; /* DTYPE_DCS_LWRITE */
static char novatek_bax[4] = {0xBA, 0x14, 0x14, 0x14}; /* DTYPE_DCS_LWRITE */
static char novatek_bbx[2] = {0xBB, 0x44}; /* DTYPE_DCS_WRITE1 */
static char novatek_bfx[2] = {0xBF, 0x01}; /* DTYPE_DCS_WRITE1 */
static char btenctr_1[2] = {0xC2, 0x01}; /* DTYPE_DCS_WRITE1 */
static char maucctr_2[6] = {0xF0, 0x55, 0xAA, 0x52, 0x00, 0x00}; /* DTYPE_DCS_LWRITE */
static char lv3[5] = {0xFF, 0xAA, 0x55, 0x25, 0x01}; /* DTYPE_DCS_LWRITE */
static char novatek_f3[9] = { /* DTYPE_DCS_LWRITE */
	0xF3, 0x00, 0x32, 0x00, 0x38, 0x31, 0x08,
	0x11, 0x00};
static char novatek_f8[39] = { /* DTYPE_DCS_LWRITE */
	0xF8, 0x02, 0x02, 0x00, 0x20, 0x33, 0x00,
	0x00, 0x40, 0x00, 0x00, 0x23, 0x02, 0x99,
	0xC8, 0x02, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x0F, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00,
	0x05, 0x05, 0x00, 0x34, 0x00, 0x04, 0x55,
	0x00, 0x04, 0x11, 0x38};
//static char maucctr_3[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}; /* DTYPE_DCS_LWRITE */

static char novatek_d2[2] = {0xD2, 0x00}; /* DTYPE_DCS_WRITE1 */
static char novatek_d3[2] = {0xD3, 0x00}; /* DTYPE_DCS_WRITE1 */
static char novatek_d4[2] = {0xD4, 0x04}; /* DTYPE_DCS_WRITE1 */
static char novatek_d5[2] = {0xD5, 0x11}; /* DTYPE_DCS_WRITE1 */
static char novatek_dd[2] = {0xDD, 0x44}; /* DTYPE_DCS_WRITE1 */
static char novatek_e1[3] = {0xE1, 0x00, 0xFF}; /* DTYPE_DCS_LWRITE */
//static char maucctr_4[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}; /* DTYPE_DCS_LWRITE */
static char novatek_3a[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */
static char novatek_36[2] = {0x36, 0x00}; /* DTYPE_DCS_WRITE1 */
static char novatek_2a[5] = {0x2A, 0x00, 0x00, 0x01, 0xDF}; /* DTYPE_DCS_LWRITE */
static char novatek_2b[5] = {0x2B, 0x00, 0x00, 0x03, 0x1F}; /* DTYPE_DCS_LWRITE */
static char memory_start[2] = {0x2C, 0x00}; /* DTYPE_DCS_LWRITE */
//static char novatek_51[2] = {0x51, 0xFF}; /* DTYPE_DCS_WRITE1 */
//static char novatek_55[2] = {0x55, 0x00}; /* DTYPE_DCS_WRITE1 */

//static char novatek_53[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */
#endif //HTC_USED_0_3_MIPI_INIT
#endif

static struct dsi_cmd_desc novatek_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_twolane), set_twolane},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(rgb_888), rgb_888},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm1), led_pwm1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm3), led_pwm3},		
};

static struct dsi_cmd_desc novatek_wvga_c3_cmd_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(maucctr_0), maucctr_0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(novatek_e0), novatek_e0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(maucctr_1), maucctr_1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(novatek_b5x), novatek_b5x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(novatek_ce), novatek_ce},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(test_reg), test_reg},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
//		sizeof(display_off), display_off},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm1), led_pwm1},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm3), led_pwm3},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(novatek_5e), novatek_5e},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm2), led_pwm2},
};

#ifdef HTC_USED_0_3_MIPI_INIT
static struct dsi_cmd_desc novatek_wvga_cmd_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 100,
		sizeof(lv3), lv3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_f8), novatek_f8},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_f3), novatek_f3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(maucctr_0), maucctr_0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b1), novatek_b1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(novatek_b6), novatek_b6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b7), novatek_b7},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b8), novatek_b8},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b9), novatek_b9},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_ba), novatek_ba},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_bb), novatek_bb},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_bc), novatek_bc},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_bd), novatek_bd},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_be), novatek_be},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_bf), novatek_bf},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_ca), novatek_ca},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_e0), novatek_e0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(maucctr_1), maucctr_1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b0x), novatek_b0x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b1x), novatek_b1x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b2x), novatek_b2x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b3x), novatek_b3x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b5x), novatek_b5x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b6x), novatek_b6x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b7x), novatek_b7x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b8x), novatek_b8x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_b9x), novatek_b9x},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_bax), novatek_bax},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(novatek_bfx), novatek_bfx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(btenctr_1), btenctr_1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(novatek_ce), novatek_ce},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(memory_start), memory_start},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_WRITE, 1, 0, 0, 40,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(led_pwm1), led_pwm1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(led_pwm3), led_pwm3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(novatek_5e), novatek_5e},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(led_pwm2), led_pwm2},
};
#endif //HTC_USED_0_3_MIPI_INIT

/* Gamma table of PYD, sharp panel */
static char pyd_sharp_gm[] = {
	/* into gamma page */
	0xf3, 0xaa,
	/* R positive */
	0x24, 0x00, 0x25, 0x04, 0x26, 0x11, 0x27, 0x1c, 0x28, 0x1a, 0x29, 0x2e,
	0x2a, 0x5e, 0x2b, 0x21, 0x2d, 0x1f, 0x2f, 0x27, 0x30, 0x60, 0x31, 0x15,
	0x32, 0x3e, 0x33, 0x5f, 0x34, 0x7c, 0x35, 0x86, 0x36, 0x87, 0x37, 0x08,
	/* R negative */
	0x38, 0x01, 0x39, 0x06, 0x3a, 0x14, 0x3b, 0x21, 0x3d, 0x1a, 0x3f, 0x2d,
	0x40, 0x5f, 0x41, 0x33, 0x42, 0x20, 0x43, 0x27, 0x44, 0x7b, 0x45, 0x15,
	0x46, 0x3e, 0x47, 0x5f, 0x48, 0xa7, 0x49, 0xb3, 0x4a, 0xb4, 0x4b, 0x35,
	/* G positive */
	0x4c, 0x2a, 0x4d, 0x2d, 0x4e, 0x36, 0x4f, 0x3e, 0x50, 0x18, 0x51, 0x2a,
	0x52, 0x5c, 0x53, 0x2c, 0x54, 0x1d, 0x55, 0x25, 0x56, 0x65, 0x57, 0x12,
	0x58, 0x3a, 0x59, 0x57, 0x5a, 0x93, 0x5b, 0xb2, 0x5c, 0xb6, 0x5d, 0x37,
	/* G negative */
	0x5e, 0x30, 0x5f, 0x34, 0x60, 0x3e, 0x61, 0x46, 0x62, 0x19, 0x63, 0x2b,
	0x64, 0x5c, 0x65, 0x3f, 0x66, 0x1f, 0x67, 0x26, 0x68, 0x80, 0x69, 0x13,
	0x6a, 0x3c, 0x6b, 0x57, 0x6c, 0xc0, 0x6d, 0xe2, 0x6e, 0xe7, 0x6f, 0x68,
	/* B positive */
	0x70, 0x00, 0x71, 0x0a, 0x72, 0x26, 0x73, 0x37, 0x74, 0x1e, 0x75, 0x32,
	0x76, 0x60, 0x77, 0x32, 0x78, 0x1f, 0x79, 0x26, 0x7a, 0x68, 0x7b, 0x14,
	0x7c, 0x39, 0x7d, 0x59, 0x7e, 0x85, 0x7f, 0x86, 0x80, 0x87, 0x81, 0x08,
	/* B negative */
	0x82, 0x01, 0x83, 0x0c, 0x84, 0x2b, 0x85, 0x3e, 0x86, 0x1f, 0x87, 0x33,
	0x88, 0x61, 0x89, 0x45, 0x8a, 0x1f, 0x8b, 0x26, 0x8c, 0x84, 0x8d, 0x14,
	0x8e, 0x3a, 0x8f, 0x59, 0x90, 0xb1, 0x91, 0xb3, 0x92, 0xb4, 0x93, 0x35,
	0xc9, 0x01,
	/* out of gamma page */
	0xff, 0xaa,
};

static struct dsi_cmd_desc pyd_sharp_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(sw_reset), sw_reset},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +0},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +4},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +6},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +8},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +10},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +12},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +14},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +16},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +18},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +20},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +22},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +24},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +26},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +28},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +30},/* don't change this line */
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +32},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +34},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +36},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +38},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +40},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +42},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +44},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +46},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +48},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +50},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +52},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +54},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +56},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +58},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +60},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +62},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +64},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +66},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +68},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +70},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +72},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +74},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +76},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +78},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +80},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +82},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +84},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +86},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +88},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +90},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +92},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +94},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +96},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +98},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +100},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +102},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +104},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +106},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +108},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +110},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +112},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +114},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +116},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +118},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +120},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +122},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +124},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +126},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +128},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +130},/* don't change this line */
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +132},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +134},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +136},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +138},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +140},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +142},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +144},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +146},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +148},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +150},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +152},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +154},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +156},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +158},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +160},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +162},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +164},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +166},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +168},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +170},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +172},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +174},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +176},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +178},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +180},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +182},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +184},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +186},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +188},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +190},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +192},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +194},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +196},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +198},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +200},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +202},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +204},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +206},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +208},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +210},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +212},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +214},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +216},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +218},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_sharp_gm +220},

	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_f3), novatek_pwm_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_00), novatek_pwm_00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_21), novatek_pwm_21},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_22), novatek_pwm_22},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_7d), novatek_pwm_7d},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_7f), novatek_pwm_7f},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_f3), novatek_pwm_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp), novatek_pwm_cp},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp3), novatek_pwm_cp3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(test_reg), test_reg},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
//		sizeof(display_off), display_off},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pktsize), max_pktsize},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_f4), novatek_f4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(novatek_8c), novatek_8c},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_ff), novatek_ff},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(set_twolane), set_twolane},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_width), set_width},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_height), set_height},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(rgb_888), rgb_888},
// &*&*&*PM		
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm1), led_pwm1},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm2), led_pwm2},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm3), led_pwm3},
// &*&*&*PM		
};

/* Gamma table of PYD, sharp panel */
static char pyd_auo_gm[] = {
	/* into gamma page */
	0xf3, 0xaa,
	/* R positive */
	0x24,0X63,0x25,0X6B,0x26,0X78,0x27,0X7E,0x28,0X19,0x29,0X2E,
	0x2A,0X61,0x2B,0X61,0x2D,0X1b,0x2F,0X22,0x30,0X84,0x31,0X1B,
	0x32,0X4F,0x33,0X63,0x34,0x28,0x35,0XDF,0x36,0XC9,0x37,0X69,
	/* R negative */
	0x38,0X63,0x39,0X6B,0x3A,0X78,0x3B,0X7E,0x3D,0X19,0x3F,0X2E,
	0x40,0X61,0x41,0X61,0x42,0X1b,0x43,0X22,0x44,0X84,0x45,0X1B,
	0x46,0X4F,0x47,0X63,0x48,0XC7,0x49,0XDF,0x4A,0XC9,0x4B,0X69,
	/* G positive */
	0x4C,0X45,0x4D,0X54,0x4E,0X64,0x4F,0X75,0x50,0X18,0x51,0X2E,
	0x52,0X62,0x53,0X61,0x54,0X1D,0x55,0X26,0x56,0X9D,0x57,0X10,
	0x58,0X39,0x59,0X55,0x5A,0XC3,0x5B,0XD7,0x5C,0XFF,0x5D,0X6B,
	/* G negative */
	0x5E,0X45,0x5F,0X54,0x60,0X64,0x61,0X75,0x62,0X18,0x63,0X2E,
	0x64,0X62,0x65,0X61,0x66,0X1D,0x67,0X26,0x68,0x65,0x69,0X10,
	0x6A,0X39,0x6B,0X55,0x6C,0XC3,0x6D,0XD7,0x6E,0XFF,0x6F,0X6B,
	/* B positive */
	0x70,0X7D,0x71,0X82,0x72,0X89,0x73,0X97,0x74,0X19,0x75,0X2E,
	0x76,0X61,0x77,0X6E,0x78,0X1A,0x79,0X1E,0x7A,0X8E,0x7B,0X0C,
	0x7C,0X27,0x7D,0X58,0x7E,0XCF,0x7F,0XD9,0x80,0XFc,0x81,0X68,
	/* B negative */
	0x82,0X7D,0x83,0X82,0x84,0X89,0x85,0X97,0x86,0X19,0x87,0X2E,
	0x88,0X61,0x89,0X6E,0x8A,0X1A,0x8B,0X1E,0x8C,0X8E,0x8D,0X0C,
	0x8E,0X27,0x8F,0X58,0x90,0XCF,0x91,0XD9,0x92,0XFc,0x93,0X68,
	/* out of gamma page */
	0xC9,0x01,
	0xff, 0xaa,
};

/* Gamma table of RIR, sharp panel */
static char rir_shp_gm[] = {
       0xf3, 0xaa,
       // 2.5 gamma
       /* R + */
       0x24, 0x00, 0x25, 0x03, 0x26, 0x0e, 0x27, 0x19, 0x28, 0x18, 0x29, 0x2c,
       0x2A, 0x5d, 0x2B, 0x18, 0x2D, 0x1f, 0x2F, 0x26, 0x30, 0x58, 0x31, 0x16,
       0x32, 0x3a, 0x33, 0x53, 0x34, 0x69, 0x35, 0x6f, 0x36, 0x71, 0x37, 0x08,
       /* R - */
       0x38, 0x01, 0x39, 0x05, 0x3A, 0x11, 0x3B, 0x1d, 0x3D, 0x18, 0x3F, 0x2c,
       0x40, 0x5d, 0x41, 0x29, 0x42, 0x1f, 0x43, 0x26, 0x44, 0x72, 0x45, 0x15,
       0x46, 0x3a, 0x47, 0x53, 0x48, 0x93, 0x49, 0x9a, 0x4A, 0x9c, 0x4B, 0x35,
       /* G + */
       0x4C, 0x17, 0x4D, 0x1A, 0x4E, 0x24, 0x4F, 0x2d, 0x50, 0x19, 0x51, 0x2c,
       0x52, 0x5d, 0x53, 0x21, 0x54, 0x1E, 0x55, 0x25, 0x56, 0x5d, 0x57, 0x13,
       0x58, 0x37, 0x59, 0x4f, 0x5A, 0x78, 0x5B, 0xaa, 0x5C, 0xb6, 0x5D, 0x37,
       /* G - */
       0x5E, 0x1A, 0x5F, 0x1e, 0x60, 0x29, 0x61, 0x33, 0x62, 0x19, 0x63, 0x2c,
       0x64, 0x5d, 0x65, 0x33, 0x66, 0x1f, 0x67, 0x26, 0x68, 0x77, 0x69, 0x14,
       0x6A, 0x38, 0x6B, 0x4f, 0x6C, 0xa3, 0x6D, 0xdb, 0x6E, 0xe8, 0x6F, 0x68,
       /* B + */
       0x70, 0x4B, 0x71, 0x4d, 0x72, 0x55, 0x73, 0x5a, 0x74, 0x17, 0x75, 0x2a,
       0x76, 0x5b, 0x77, 0x32, 0x78, 0x1E, 0x79, 0x25, 0x7A, 0x64, 0x7B, 0x19,
       0x7C, 0x4c, 0x7D, 0x62, 0x7E, 0x53, 0x7F, 0x67, 0x80, 0x71, 0x81, 0x08,
       /* B - */
       0x82, 0x54, 0x83, 0x57, 0x84, 0x5f, 0x85, 0x65, 0x86, 0x17, 0x87, 0x29,
       0x88, 0x5a, 0x89, 0x47, 0x8A, 0x1d, 0x8B, 0x25, 0x8C, 0x7f, 0x8D, 0x1a,
       0x8E, 0x4c, 0x8F, 0x62, 0x90, 0x7a, 0x91, 0x90, 0x92, 0x9B, 0x93, 0x35,

       // 2.x gamma

       0xff, 0xaa,
};

static char rir_auo_gm[] = {
       0xf3, 0xaa,
       // 2.5 gamma
       /* R + */
       0x24, 0x00, 0x25, 0x0A, 0x26, 0x25, 0x27, 0x3A, 0x28, 0x20, 0x29, 0x35,
       0x2A, 0x64, 0x2B, 0x41, 0x2D, 0x21, 0x2F, 0x27, 0x30, 0x76, 0x31, 0x10,
       0x32, 0x34, 0x33, 0x50, 0x34, 0x86, 0x35, 0xAB, 0x36, 0x3A, 0x37, 0x4C,
       /* R - */
       0x38, 0x00, 0x39, 0x0A, 0x3A, 0x25, 0x3B, 0x3A, 0x3D, 0x20, 0x3F, 0x35,
       0x40, 0x64, 0x41, 0x41, 0x42, 0x21, 0x43, 0x27, 0x44, 0x76, 0x45, 0x10,
       0x46, 0x34, 0x47, 0x50, 0x48, 0x86, 0x49, 0xAB, 0x4A, 0x3A, 0x4B, 0x4C,
       /* G + */
       0x4C, 0x00, 0x4D, 0x0A, 0x4E, 0x25, 0x4F, 0x3A, 0x50, 0x20, 0x51, 0x35,
       0x52, 0x64, 0x53, 0x41, 0x54, 0x21, 0x55, 0x27, 0x56, 0x76, 0x57, 0x10,
       0x58, 0x34, 0x59, 0x50, 0x5A, 0x86, 0x5B, 0xAB, 0x5C, 0x3A, 0x5D, 0x4C,
       /* G - */
       0x5E, 0x00, 0x5F, 0x0A, 0x60, 0x25, 0x61, 0x3A, 0x62, 0x20, 0x63, 0x35,
       0x64, 0x64, 0x65, 0x41, 0x66, 0x21, 0x67, 0x27, 0x68, 0x76, 0x69, 0x10,
       0x6A, 0x34, 0x6B, 0x50, 0x6C, 0x86, 0x6D, 0xAB, 0x6E, 0x3A, 0x6F, 0x4C,
       /* B + */
       0x70, 0x00, 0x71, 0x0A, 0x72, 0x25, 0x73, 0x3a, 0x74, 0x20, 0x75, 0x35,
       0x76, 0x64, 0x77, 0x41, 0x78, 0x21, 0x79, 0x27, 0x7A, 0x76, 0x7B, 0x10,
       0x7C, 0x34, 0x7D, 0x50, 0x7E, 0x86, 0x7F, 0xAB, 0x80, 0x3A, 0x81, 0x4C,
       /* B - */
       0x82, 0x00, 0x83, 0x0A, 0x84, 0x25, 0x85, 0x3A, 0x86, 0x20, 0x87, 0x35,
       0x88, 0x64, 0x89, 0x41, 0x8A, 0x21, 0x8B, 0x27, 0x8C, 0x76, 0x8D, 0x10,
       0x8E, 0x34, 0x8F, 0x50, 0x90, 0x86, 0x91, 0xAB, 0x92, 0x3A, 0x93, 0x4C,

       // 2.x gamma
       0xC9, 0x01,
       0xff, 0xaa,
};

static struct dsi_cmd_desc pyd_auo_cmd_on_cmds[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 10,
            sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xf3, 0xaa}},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xA3, 0xFF}},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xA4, 0xFF}},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xA5, 0xFF}},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xA6, 0x01}},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xFF, 0xAA}},

	/* Gamma table start */
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +0},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +4},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +6},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +8},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +10},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +12},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +14},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +16},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +18},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +20},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +22},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +24},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +26},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +28},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +30},/* don't change this line! */
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +32},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +34},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +36},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +38},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +40},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +42},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +44},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +46},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +48},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +50},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +52},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +54},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +56},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +58},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +60},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +62},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +64},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +66},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +68},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +70},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +72},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +74},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +76},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +78},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +80},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +82},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +84},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +86},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +88},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +90},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +92},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +94},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +96},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +98},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +100},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +102},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +104},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +106},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +108},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +110},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +112},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +114},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +116},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +118},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +120},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +122},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +124},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +126},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +128},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +130},/* don't change this line! */
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +132},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +134},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +136},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +138},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +140},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +142},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +144},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +146},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +148},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +150},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +152},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +154},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +156},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +158},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +160},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +162},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +164},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +166},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +168},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +170},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +172},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +174},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +176},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +178},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +180},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +182},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +184},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +186},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +188},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +190},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +192},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +194},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +196},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +198},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +200},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +202},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +204},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +206},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +208},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +210},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +212},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +214},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +216},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +218},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, pyd_auo_gm +220},
	/* Gamma table end */
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
			sizeof(exit_sleep), exit_sleep},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_f3), novatek_pwm_f3},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_00), novatek_pwm_00},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_21), novatek_pwm_21},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_22), novatek_pwm_22},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_7d), novatek_pwm_7d},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_7f), novatek_pwm_7f},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_f3), novatek_pwm_f3},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_cp), novatek_pwm_cp},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(novatek_pwm_cp3), novatek_pwm_cp3},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
            sizeof(enable_te), enable_te},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
            sizeof(test_reg), test_reg},
//        {DTYPE_DCS_WRITE, 1, 0, 0, 50,
//                sizeof(display_off), display_off},
    {DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
            sizeof(max_pktsize), max_pktsize},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
            sizeof(set_width), set_width},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
            sizeof(set_height), set_height},
//        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//                sizeof(led_pwm1), led_pwm1},
//        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//                sizeof(led_pwm2), led_pwm2},
//        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//                sizeof(led_pwm3), led_pwm3},
};

static struct dsi_cmd_desc shp_novatek_cmd_on_cmds[] = {
       {DTYPE_DCS_WRITE, 1, 0, 0, 10,
               sizeof(sw_reset), sw_reset},
       {DTYPE_DCS_WRITE, 1, 0, 0, 150,
               sizeof(exit_sleep), exit_sleep},

       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm },
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +2},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +4},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +6},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +8},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +10},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +12},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +14},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +16},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +18},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +20},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +22},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +24},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +26},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +28},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +20},/* don't change this line! */
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +32},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +34},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +36},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +38},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +40},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +42},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +44},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +46},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +48},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +50},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +52},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +54},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +56},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +58},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +60},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +62},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +64},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +66},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +68},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +70},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +72},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +74},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +76},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +78},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +80},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +82},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +84},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +86},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +88},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +90},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +92},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +94},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +96},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +98},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +100},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +102},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +104},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +106},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +108},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +110},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +112},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +114},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +116},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +118},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +120},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +122},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +124},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +126},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +128},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +120},/* don't change this line! */
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +132},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +134},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +136},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +138},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +140},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +142},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +144},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +146},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +148},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +150},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +152},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +154},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +156},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +158},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +160},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +162},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +164},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +166},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +168},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +170},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +172},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +174},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +176},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +178},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +180},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +182},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +184},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +186},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +188},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +190},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +192},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +194},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +196},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +198},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +200},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +202},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +204},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +206},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +208},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +210},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +212},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +214},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_shp_gm +216},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 50, 2, rir_shp_gm +218},

       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_f3), novatek_pwm_f3},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_00), novatek_pwm_00},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_21), novatek_pwm_21},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_22), novatek_pwm_22},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_7d), novatek_pwm_7d},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_7f), novatek_pwm_7f},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_f3), novatek_pwm_f3},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_cp), novatek_pwm_cp},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_cp3), novatek_pwm_cp3},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(enable_te), enable_te},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(test_reg), test_reg},
//       {DTYPE_DCS_WRITE, 1, 0, 0, 50,
//               sizeof(display_off), display_off},
       {DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
               sizeof(max_pktsize), max_pktsize},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_f4), novatek_f4},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(novatek_8c), novatek_8c},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_ff), novatek_ff},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(set_twolane), set_twolane},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(set_width), set_width},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(set_height), set_height},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(rgb_888), rgb_888},
// &*&*&*PM
//       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//               sizeof(led_pwm1), led_pwm1},
//       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//               sizeof(led_pwm2), led_pwm2},
//       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//               sizeof(led_pwm3), led_pwm3},
// &*&*&*PM
};

static struct dsi_cmd_desc auo_nov_cmd_on_cmds[] = {
       {DTYPE_DCS_WRITE, 1, 0, 0, 10,
               sizeof(sw_reset), sw_reset},
       {DTYPE_DCS_WRITE, 1, 0, 0, 0,
               sizeof(exit_sleep), exit_sleep},

       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm },
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +2},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +4},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +6},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +8},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +10},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +12},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +14},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +16},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +18},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +20},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +22},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +24},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +26},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +28},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +20},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +32},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +34},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +36},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +38},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +40},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +42},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +44},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +46},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +48},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +40},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +50},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +52},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +54},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +56},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +58},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +60},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +62},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +64},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +66},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +68},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +70},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +72},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +74},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +76},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +78},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +80},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +82},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +84},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +86},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +88},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +90},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +92},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +94},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +96},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +98},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +100},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +102},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +104},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +106},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +108},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +110},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +112},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +114},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +116},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +118},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +120},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +122},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +124},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +126},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +128},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +120},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +132},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +134},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +136},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +138},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +140},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +142},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +144},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +146},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +148},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +140},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +150},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +152},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +154},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +156},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +158},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +160},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +162},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +164},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +166},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +168},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +170},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +172},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +174},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +176},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +178},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +180},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +182},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +184},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +186},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +188},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +190},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +192},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +194},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +196},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +198},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +200},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +202},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +204},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +206},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +208},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +210},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +212},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +214},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +216},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, rir_auo_gm +218},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 120, 2, rir_auo_gm +220},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_f3), novatek_pwm_f3},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_00), novatek_pwm_00},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_21), novatek_pwm_21},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_22), novatek_pwm_22},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_7d), novatek_pwm_7d},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_7f), novatek_pwm_7f},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_f3), novatek_pwm_f3},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_cp), novatek_pwm_cp},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_pwm_cp3), novatek_pwm_cp3},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(enable_te), enable_te},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(test_reg), test_reg},
//       {DTYPE_DCS_WRITE, 1, 0, 0, 50,
//               sizeof(display_off), display_off},
       {DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
               sizeof(max_pktsize), max_pktsize},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_f4), novatek_f4},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(novatek_8c), novatek_8c},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(novatek_ff), novatek_ff},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(set_twolane), set_twolane},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(set_width), set_width},
       {DTYPE_DCS_LWRITE, 1, 0, 0, 0,
               sizeof(set_height), set_height},
       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
               sizeof(rgb_888), rgb_888},
// &*&*&*PM
//       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//               sizeof(led_pwm1), led_pwm1},
//       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//               sizeof(led_pwm2), led_pwm2},
//       {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//               sizeof(led_pwm3), led_pwm3},
// &*&*&*PM
};

static struct dsi_cmd_desc shr_sharp_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_f4), novatek_f4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(novatek_eq1200), novatek_eq1200},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_fd), novatek_fd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_ff), novatek_ff},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_f3), novatek_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_2vci), novatek_2vci},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_ff_aa), novatek_ff_aa},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_f3), novatek_pwm_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_00), novatek_pwm_00},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_21), novatek_pwm_21},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_22), novatek_pwm_22},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_7d), novatek_pwm_7d},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_7f), novatek_pwm_7f},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_f3), novatek_pwm_f3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp), novatek_pwm_cp},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp2), novatek_pwm_cp2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(novatek_pwm_cp3), novatek_pwm_cp3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(enable_te), enable_te},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(test_reg_qhd), test_reg_qhd},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
//		sizeof(display_off), display_off},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pktsize), max_pktsize},
	//{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
	//	sizeof(novatek_f4), novatek_f4},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
	//	sizeof(novatek_8c), novatek_8c},
	//{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
	//	sizeof(novatek_ff), novatek_ff},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(set_twolane), set_twolane},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_width), set_width},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_height), set_height},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(rgb_888), rgb_888},
// &*&*&*PM
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm1), led_pwm1},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm2), led_pwm2},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
//		sizeof(led_pwm3), led_pwm3},
// &*&*&*PM
};

static struct dsi_cmd_desc novatek_display_off_cmds[] = {
		{DTYPE_DCS_WRITE, 1, 0, 0, 0,
			sizeof(display_off), display_off},
		{DTYPE_DCS_WRITE, 1, 0, 0, 110,
			sizeof(enter_sleep), enter_sleep}
};

static char peripheral_off[2] = {0x00, 0x00}; /* DTYPE_DCS_READ */
static char vsync_start[2] = {0x00, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc novatek_restart_vcounter_cmd[] = {
	{DTYPE_VSYNC_START, 1, 0, 1, 0,
		sizeof(vsync_start), vsync_start},
	{DTYPE_PERIPHERAL_OFF, 1, 0, 1, 0,
		sizeof(peripheral_off), peripheral_off}
};

int mipi_novatek_restart_vcounter(void)
{
	uint32 dma_ctrl;

	/* DSI_COMMAND_MODE_DMA_CTRL */
	dma_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x38);
	/* pr_info("%s+ dma_ctrl=0x%x\n", __func__, dma_ctrl); */
	MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x10000000);
	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_restart_vcounter_cmd,
			ARRAY_SIZE(novatek_restart_vcounter_cmd));
	MIPI_OUTP(MIPI_DSI_BASE + 0x38, dma_ctrl);
	/* pr_info("%s-\n", __func__); */
	return 0;
}

static char read_scan_line[2] = {0x45, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc read_scan_line_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 0, sizeof(read_scan_line), read_scan_line};

uint32 mipi_novatek_read_scan_line(void)
{
	struct dsi_cmd_desc *cmd;
	uint32 dma_ctrl;

	/* DSI_COMMAND_MODE_DMA_CTRL */
	dma_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x38);
	pr_debug("%s+ dma_ctrl=0x%x\n", __func__, dma_ctrl);
	MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x10000000);
	mipi_dsi_buf_init(&novatek_rx_buf);
	mipi_dsi_buf_init(&novatek_rx_buf);

	cmd = &read_scan_line_cmd;
	mipi_dsi_cmds_rx(&novatek_tx_buf, &novatek_rx_buf, cmd, 4);
	MIPI_OUTP(MIPI_DSI_BASE + 0x38, dma_ctrl);
#if 0
	{
		int i;
		char *cp;

		cp = (char *)novatek_rx_buf.data;
		pr_info("rx-data: ");
		for (i = 0; i < novatek_rx_buf.len; i++, cp++)
			pr_info("%x ", *cp);
		pr_info("\n");
	}
	pr_info("%s: read_scan_line=%x\n", __func__,
		(uint32 *)novatek_rx_buf.data);
#endif
	return *((uint32 *)novatek_rx_buf.data);
}

#ifdef MIPI_READ_DISPLAY_ID
static char manufacture_id[2] = {0x04, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc novatek_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static uint32 mipi_novatek_manufacture_id(void)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;
	uint32 *lp;

	tp = &novatek_tx_buf;
	rp = &novatek_rx_buf;
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &novatek_manufacture_id_cmd;
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
#endif

/* Decide how many times should the write-then-read sequence goes on */
/* don't change this definition unless you are going for a stress test */
#define GAMMA_STRESS_TEST_LOOP 1

#if (GAMMA_STRESS_TEST_LOOP > 1)

static uint8_t gm_regs[] = {
0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2D, 0x2F, 0x30, 0x31,
0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3D, 0x3F,
0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B,
0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63,
0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B,
0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93,
};

static int find_gamma_value(uint8_t reg, int panel, uint8_t *value)
{
	int i;
	uint8_t *cmds;
	int len_cmds;

	switch(panel) {
	case PANEL_ID_PYD_SHARP:
		cmds = pyd_sharp_gm;
		len_cmds = ARRAY_SIZE(gm_regs);
		break;
	case PANEL_ID_PYD_AUO_NT:
	default:
		cmds = pyd_auo_gm;
		len_cmds = ARRAY_SIZE(gm_regs);
		break;
	}

	for(i=2; i<=len_cmds*2; i+=2)
		if(reg == cmds[i]) {
			*value = cmds[i+1];
			return 1;
		}

	return 0;
}

/* change gamma value each time */
/* the algorithm should be able to change back to the original values */
static void invert_gamma_value(int panel)
{
	int i;
	uint8_t *cmds;
	int len_cmds;

	switch(panel) {
	case PANEL_ID_PYD_SHARP:
		cmds = pyd_sharp_gm;
		len_cmds = ARRAY_SIZE(gm_regs);
		for(i=2; i<=len_cmds*2; i+=2) {
			if(0x34==cmds[i] || 0x68==cmds[i]) continue;
			if(cmds[i+1]<=0x4F)
				cmds[i+1] = 0x4F - cmds[i+1];
		}
		break;
	case PANEL_ID_PYD_AUO_NT:
		cmds = pyd_auo_gm;
		len_cmds = ARRAY_SIZE(gm_regs);
		for(i=2; i<=len_cmds*2; i+=2) {
			if(cmds[i+1]<=0x4F)
				cmds[i+1] = 0x4F - cmds[i+1];				
			else if(cmds[i+1]<=0x7F)
				cmds[i+1] = 0x7F - cmds[i+1];
		}
		break;
	default:
		pr_info("warning: panel not supported so far\n");
		break;
	}
}

#endif

/* write gamma value into driver IC, then read back to check */
/* set GAMMA_STRESS_TEST_LOOP = 1 to skip the read back */
/* currently only support Pyramid's sharp and AUO panels */
static void mipi_novatek_lcd_setup(int panel)
{
	int i;	
	int err_count;
	uint8_t err_seen;
#if (GAMMA_STRESS_TEST_LOOP > 1)
	int j;
	uint8_t gamma_value;
    struct dsi_buf *rp, *tp;
	char gm_cmd[] = {0x00, 0x00};

	struct dsi_cmd_desc gen_read_cmd = {
		DTYPE_DCS_READ, 1, 0, 1, 0, 2, gm_cmd};
	struct dsi_cmd_desc in_gamma_cmd =
		{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xf3, 0xaa}};
	struct dsi_cmd_desc out_gamma_cmd =
		{DTYPE_DCS_WRITE1, 1, 0, 0, 0, 2, (char[]){0xff, 0xaa}};
#endif

	for(i = 0, err_count = 0; i < GAMMA_STRESS_TEST_LOOP; i++, err_seen = 0) {

		mipi_dsi_cmds_tx(&novatek_tx_buf, mipi_power_on_cmd, mipi_power_on_cmd_size);

		/* only support PYD's sharp and auo panel */
		if(panel != PANEL_ID_PYD_SHARP && panel != PANEL_ID_PYD_AUO_NT) continue;

#if (GAMMA_STRESS_TEST_LOOP > 1)
		/* not going to read back gamma value */
		if(1 == GAMMA_STRESS_TEST_LOOP) continue;

		mipi_dsi_cmd_bta_sw_trigger();
		gm_regs[0] = 0x24;

        tp = &novatek_tx_buf; rp = &novatek_rx_buf;
        mipi_dsi_buf_init(rp); mipi_dsi_buf_init(tp);
		mipi_dsi_cmds_tx(&novatek_tx_buf, &in_gamma_cmd, 1);

//		pr_info("--start of loop %d\n", i);
		for (j = 0; j < ARRAY_SIZE(gm_regs); j++) {
			gm_cmd[0] = gm_regs[j];
        	mipi_dsi_cmds_rx(tp, rp, &gen_read_cmd, 3);
//			pr_info("(%d)NOV-REG[%02x]=%02x\n", i, gm_regs[j],
//				((char *)rp->data)[0]);

			if(0 != find_gamma_value(gm_regs[j], panel, &gamma_value)) {
				if(!(PANEL_ID_PYD_SHARP==panel && (0x34==gm_regs[j] || 0x68==gm_regs[j]))) {
					if(((char *)rp->data)[0] != gamma_value) {
						pr_info("(loop %d)(%02Xh): read(%02X), supposed(%02X)\n",
							i, gm_regs[j], ((char *)rp->data)[0], gamma_value);
						err_seen = 1;
					}
				}
			} else
				pr_err("%s: bad return value for reg %02X. Go check why.\n",
					__func__, gm_regs[j]);
		}

		if(err_seen) err_count++;
//		pr_info("--end of loop %d\n", i);

		mipi_dsi_cmds_tx(&novatek_tx_buf, &out_gamma_cmd, 1);

		/* won't invert the gamma table the loop runs for only once or odd times */
		if(! (i==GAMMA_STRESS_TEST_LOOP-1 && GAMMA_STRESS_TEST_LOOP/2))
			invert_gamma_value(panel);

		gamma_value = 1;
		find_gamma_value(gm_regs[100], panel, &gamma_value);
#endif
	}

#if (GAMMA_STRESS_TEST_LOOP > 1)
	pr_info("%s: stress test result -- %d/%d %s\n",
		__func__, err_count, GAMMA_STRESS_TEST_LOOP, err_count?" -> Something wrong!":"");
#endif
}


static struct dsi_cmd_desc novatek_cmd_backlight_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(led_pwm1), led_pwm1},
};

static struct dsi_cmd_desc novatek_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc novatek_bkl_enable_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(bkl_enable_cmds), bkl_enable_cmds},
};

static struct dsi_cmd_desc novatek_bkl_disable_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(bkl_disable_cmds), bkl_disable_cmds},
};

// -----------------------------------------------------------------------------
//                         Common Routine Implementation
// -----------------------------------------------------------------------------
#ifndef HTC_USED_0_3_MIPI_INIT
/*
static int mipi_sony_panel_power_on(const char *init_cmds, int size)
{
	const char *pCurrRawDataPos = init_cmds;
	const char *pOrgRawDataPos = init_cmds;
	struct dsi_cmd_desc dsi_cmd_item = {0};
	char cPackageSize;

	printk(KERN_ERR "init_cmds=0x%x size=%d\n", (unsigned int)init_cmds, size);
	dsi_cmd_item.last = 1;
	dsi_cmd_item.wait = 1;

	while (*pCurrRawDataPos && pCurrRawDataPos < (pOrgRawDataPos + size)) {
		// Get the current package size...
		cPackageSize = *pCurrRawDataPos;
		// Move to the package data start address...
		pCurrRawDataPos++;
		// Parse the package...
		if( cPackageSize==1 ){
			// Wait command...
			hr_msleep(*pCurrRawDataPos);
			// Move to next package...
			pCurrRawDataPos += cPackageSize;
		}else{
			// Check the MIPI header was valid?
			switch( *pCurrRawDataPos ){
				case DTYPE_DCS_WRITE:
				case DTYPE_DCS_WRITE1:
				case DTYPE_DCS_READ:
				case DTYPE_DCS_LWRITE:
					dsi_cmd_item.dtype = *pCurrRawDataPos;
					break;
				default:
					// Invalid package, check the raw data...
					printk(KERN_ERR "%s: Can't find proper data type!\n", __func__);
					return -EPERM;
			}
			// Set the package size and data location...
			dsi_cmd_item.dlen = cPackageSize-1;
			dsi_cmd_item.payload = (char*)(pCurrRawDataPos+1);
			// Send out the MIPI package...
			mipi_dsi_buf_init(&novatek_tx_buf);
			mipi_dsi_cmd_dma_add(&novatek_tx_buf, &dsi_cmd_item);
			mipi_dsi_cmd_dma_tx(&novatek_tx_buf);
			if (dsi_cmd_item.wait)
				hr_msleep(dsi_cmd_item.wait);
			// Move to next package...
			pCurrRawDataPos += cPackageSize;
		}
	}

	return 0;
}
*/
#endif //HTC_USED_0_3_MIPI_INIT

void mipi_novatek_panel_type_detect(void) {
	if (panel_type == PANEL_ID_PYD_SHARP) {
		pr_info("%s: panel_type=PANEL_ID_PYD_SHARP\n", __func__);
		strcat (ptype,"PANEL_ID_PYD_SHARP");
		mipi_power_on_cmd = pyd_sharp_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(pyd_sharp_cmd_on_cmds);
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else if (panel_type == PANEL_ID_PYD_AUO_NT) {
		pr_info("%s: panel_type=PANEL_ID_PYD_AUO_NT\n", __func__);
		strcat (ptype,"PANEL_ID_PYD_AUO_NT");
		mipi_power_on_cmd = pyd_auo_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(pyd_auo_cmd_on_cmds);
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else if (panel_type == PANEL_ID_DOT_SONY) {
		pr_info("%s: panel_type=PANEL_ID_DOT_SONY\n", __func__);
		strcat (ptype,"PANEL_ID_DOT_SONY");
		#ifdef HTC_USED_0_3_MIPI_INIT
		mipi_power_on_cmd = novatek_wvga_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(novatek_wvga_cmd_on_cmds);
		#else
		mipi_power_on_cmd = NOVATEK_SONY_MIPI_INIT;
		mipi_power_on_cmd_size = ARRAY_SIZE(NOVATEK_SONY_MIPI_INIT);
		#endif //HTC_USED_0_3_MIPI_INIT
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else if (panel_type == PANEL_ID_DOT_SONY_C3) {
		pr_info("%s: panel_type=PANEL_ID_DOT_SONY_C3\n", __func__);
		strcat (ptype,"PANEL_ID_DOT_SONY_C3");
		mipi_power_on_cmd = novatek_wvga_c3_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(novatek_wvga_c3_cmd_on_cmds);
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else if (panel_type == PANEL_ID_RIR_SHARP_NT) {
		pr_info("%s: panel_type=PANEL_ID_RIR_SHARP_NT\n", __func__);
		strcat (ptype,"PANEL_ID_RIR_SHARP_NT");
		mipi_power_on_cmd = shp_novatek_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(shp_novatek_cmd_on_cmds);
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else if (panel_type == PANEL_ID_RIR_AUO_NT) {
		pr_info("%s: panel_type=PANEL_ID_RIR_AUO_NT\n", __func__);
		strcat (ptype,"PANEL_ID_RIR_AUO_NT");
		mipi_power_on_cmd = auo_nov_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(auo_nov_cmd_on_cmds);
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else if (panel_type == PANEL_ID_SHR_SHARP_NT) {
		pr_info("%s: panel_type=PANEL_ID_SHR_SHARP_NT\n", __func__);
		strcat (ptype,"PANEL_ID_SHR_SHARP_NT");
		mipi_power_on_cmd = shr_sharp_cmd_on_cmds;
		mipi_power_on_cmd_size = ARRAY_SIZE(shr_sharp_cmd_on_cmds);
		mipi_power_off_cmd = novatek_display_off_cmds;
		mipi_power_off_cmd_size = ARRAY_SIZE(novatek_display_off_cmds);
	} else {
		printk(KERN_ERR "%s: panel_type=0x%x not support\n", __func__, panel_type);
		strcat (ptype,"PANEL_ID_NONE");
	}
	return;
}

static int mipi_novatek_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct msm_fb_panel_data *pdata = NULL;
	struct mipi_panel_info *mipi;
	static int init;

	mfd = platform_get_drvdata(pdev);
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi  = &mfd->panel_info.mipi;
	mutex_lock(&cmdlock);
	if (init == 0) {
		if(pdata && pdata->panel_type_detect)
			pdata->panel_type_detect();
		init = 1;
		goto end;
	} else {
		if (mipi->mode == DSI_VIDEO_MODE) {
			mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_video_on_cmds,
				ARRAY_SIZE(novatek_video_on_cmds));
		} else {
			if (panel_type != PANEL_ID_NONE) {
				pr_info("%s\n", ptype);
				mipi_novatek_lcd_setup(panel_type);
#ifdef MIPI_READ_DISPLAY_ID /* mipi read command verify */
				/* clean up ack_err_status */
				mipi_dsi_cmd_bta_sw_trigger();
				mipi_novatek_manufacture_id();
#endif
			} else {
				printk(KERN_ERR "panel_type=0x%x not support at power on\n", panel_type);
				mutex_unlock(&cmdlock);
				return -EINVAL;
			}
		}
	}
end:
	mutex_unlock(&cmdlock);
	return 0;
}

static int mipi_novatek_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	mutex_lock(&cmdlock);

	if (panel_type != PANEL_ID_NONE) {
		pr_info("%s\n", ptype);

		/* For ESD fixup */
		if (mfd->esd_fixup) {
			mipi_dsi_cmd_bta_sw_trigger();
			mipi_novatek_read_scan_line();
		}

		mipi_dsi_cmds_tx(&novatek_tx_buf, mipi_power_off_cmd,
			mipi_power_off_cmd_size);
	} else
		printk(KERN_ERR "panel_type=0x%x not support at power off\n",
			panel_type);

	mutex_unlock(&cmdlock);

	return 0;
}

static int mipi_dsi_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	static int bl_level_old = 0;
	mutex_lock(&cmdlock);

	mipi  = &mfd->panel_info.mipi;
	pr_debug("%s+:bl=%d status=%d\n", __func__, mfd->bl_level, mipi_status);
	if (mipi_status == 0)
		goto end;
	if (mipi_novatek_pdata && mipi_novatek_pdata->shrink_pwm)
		led_pwm1[1] = mipi_novatek_pdata->shrink_pwm(mfd->bl_level);
	else
		led_pwm1[1] = (unsigned char)(mfd->bl_level);

	if(mfd->bl_level == 0 || board_mfg_mode() == 4 || board_mfg_mode() == 5) {
		//mipi_dsi_op_mode_config(DSI_CMD_MODE);
		//mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_bkl_disable_cmds,
			//ARRAY_SIZE(novatek_bkl_disable_cmds));
		led_pwm1[1] = 0;
	}
	if (mipi->mode == DSI_VIDEO_MODE) {
		mipi_dsi_cmd_mode_ctrl(1);	/* enable cmd mode */
		mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_cmd_backlight_cmds,
		ARRAY_SIZE(novatek_cmd_backlight_cmds));
		mipi_dsi_cmd_mode_ctrl(0);	/* disable cmd mode */
	} else {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
		mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_cmd_backlight_cmds,
		ARRAY_SIZE(novatek_cmd_backlight_cmds));
	}
	bl_level_prevset = bl_level_old = mfd->bl_level;
end:
	mutex_unlock(&cmdlock);
	return 0;
}

static void mipi_novatek_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;

	bl_level = mfd->bl_level;
	pr_debug("%s+ bl_level=%d\n", __func__, mfd->bl_level);

		mipi_dsi_set_backlight(mfd);
}

static void mipi_novatek_display_on(struct msm_fb_data_type *mfd)
{
	pr_debug("%s+\n", __func__);
	mutex_lock(&cmdlock);
	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_display_on_cmds,
		ARRAY_SIZE(novatek_display_on_cmds));
	mutex_unlock(&cmdlock);
}

static int mipi_novatek_send_cmds(struct dsi_cmd_desc *novatek_cmds, uint32_t size)
{
	pr_debug("%s+\n", __func__);
	mutex_lock(&cmdlock);
	if (mipi_status == 0)
		goto end;
	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_cmds, size);
end:
	mutex_unlock(&cmdlock);
	pr_debug("%s-\n", __func__);
	return 0;
}

static void mipi_novatek_bkl_switch(struct msm_fb_data_type *mfd, bool on)
{
	unsigned int val = 0;

	if(on) {
		mipi_status = 1;
		val = mfd->bl_level;
		if(val == 0) {
			if(bl_level_prevset != 1) {
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

static void mipi_novatek_bkl_ctrl(bool on)
{
	mutex_lock(&cmdlock);
	if(on) {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
		mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_bkl_enable_cmds,
			ARRAY_SIZE(novatek_bkl_enable_cmds));
	} else {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
		mipi_dsi_cmds_tx(&novatek_tx_buf, novatek_bkl_disable_cmds,
			ARRAY_SIZE(novatek_bkl_disable_cmds));
	}
	mutex_unlock(&cmdlock);
}

static int mipi_novatek_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);
	if (pdev->id == 0) {
		mipi_novatek_pdata = pdev->dev.platform_data;
		if (mipi_novatek_pdata)
			mipi_novatek_pdata->mipi_send_cmds = mipi_novatek_send_cmds;
		mutex_init(&cmdlock);
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_novatek_lcd_probe,
	.driver = {
		.name   = "mipi_novatek",
	},
};

static struct msm_fb_panel_data novatek_panel_data = {
	.on		= mipi_novatek_lcd_on,
	.off		= mipi_novatek_lcd_off,
	.set_backlight  = mipi_novatek_set_backlight,
	.display_on  = mipi_novatek_display_on,
	.bklswitch	= mipi_novatek_bkl_switch,
	.bklctrl	= mipi_novatek_bkl_ctrl,
	.panel_type_detect = mipi_novatek_panel_type_detect,
};

static int ch_used[3];

int mipi_novatek_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_novatek", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	novatek_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &novatek_panel_data,
		sizeof(novatek_panel_data));
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

static int __init mipi_novatek_lcd_init(void)
{
	mipi_dsi_buf_alloc(&novatek_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&novatek_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_novatek_lcd_init);
