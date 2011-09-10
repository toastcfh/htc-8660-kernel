/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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
 */

#ifndef SP3D_H
#define SP3D_H
#include <linux/types.h>
#include <mach/board.h>

/* SP 3D module operation mode */
#define SP3D_REGADDR_WR 0x70
#define SP3D_REGVAL_WR	0x72
#define SP3D_REGVAL_RD	0x73

extern struct sp3d_snapshot_reg sp3d_snapshot_regs;
extern struct sp3d_com_feature_reg sp3d_com_feature_regs;
extern struct sp3d_2d_reg sp3d_2D_regs;
extern struct sp3d_3d_reg sp3d_3D_regs;
extern struct sp3d_calibration_reg sp3d_calibration_regs;

struct reg_struct_init {
    /* PLL setting */
	uint8_t pre_pll_clk_div; /* 0x0305 */
	uint8_t plstatim; /* 0x302b */
	uint8_t reg_3024; /*ox3024*/
	uint8_t image_orientation;  /* 0x0101*/
	uint8_t vndmy_ablmgshlmt; /*0x300a*/
	uint8_t y_opbaddr_start_di; /*0x3014*/
	uint8_t reg_0x3015; /*0x3015*/
	uint8_t reg_0x301c; /*0x301c*/
	uint8_t reg_0x302c; /*0x302c*/
	uint8_t reg_0x3031; /*0x3031*/
	uint8_t reg_0x3041; /* 0x3041 */
	uint8_t reg_0x3051; /* 0x3051 */
	uint8_t reg_0x3053; /* 0x3053 */
	uint8_t reg_0x3057; /* 0x3057 */
	uint8_t reg_0x305c; /* 0x305c */
	uint8_t reg_0x305d; /* 0x305d */
	uint8_t reg_0x3060; /* 0x3060 */
	uint8_t reg_0x3065; /* 0x3065 */
	uint8_t reg_0x30aa; /* 0x30aa */
	uint8_t reg_0x30ab;
	uint8_t reg_0x30b0;
	uint8_t reg_0x30b2;
	uint8_t reg_0x30d3;
	uint8_t reg_0x3106;
	uint8_t reg_0x310c;
	uint8_t reg_0x3304;
	uint8_t reg_0x3305;
	uint8_t reg_0x3306;
	uint8_t reg_0x3307;
	uint8_t reg_0x3308;
	uint8_t reg_0x3309;
	uint8_t reg_0x330a;
	uint8_t reg_0x330b;
	uint8_t reg_0x330c;
	uint8_t reg_0x330d;
	uint8_t reg_0x330f;
	uint8_t reg_0x3381;
};

struct reg_struct {
	uint8_t pll_multiplier; /* 0x0307 */
	uint8_t frame_length_lines_hi; /* 0x0340*/
	uint8_t frame_length_lines_lo; /* 0x0341*/
	uint8_t y_addr_start;  /* 0x347 */
	uint8_t y_add_end;  /* 0x034b */
	uint8_t x_output_size_msb;  /* 0x034c */
	uint8_t x_output_size_lsb;  /* 0x034d */
	uint8_t y_output_size_msb; /* 0x034e */
	uint8_t y_output_size_lsb; /* 0x034f */
	uint8_t x_even_inc;  /* 0x0381 */
	uint8_t x_odd_inc; /* 0x0383 */
	uint8_t y_even_inc;  /* 0x0385 */
	uint8_t y_odd_inc; /* 0x0387 */
	uint8_t hmodeadd;   /* 0x3001 */
	uint8_t vmodeadd;   /* 0x3016 */
	uint8_t vapplinepos_start;/*ox3069*/
	uint8_t vapplinepos_end;/*306b*/
	uint8_t shutter;	/* 0x3086 */
	uint8_t haddave;	/* 0x30e8 */
	uint8_t lanesel;    /* 0x3301 */
};

struct sp3d_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

enum sp3d_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum sp3d_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};
enum sp3d_setting {
	RES_PREVIEW,
	RES_CAPTURE,
	RES_VIDEO
};
enum mt9p012_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

struct sharp_core{
	struct task_struct         *kthread;
};


struct sp3d_calibration_reg {
	struct sp3d_reg_conf const *reg_para1_koj_mixer_cal_3d;
	uint16_t reg_para1_koj_mixer_cal_3d_size;
	struct sp3d_reg_conf const *reg_para1_koj_mixer_cal_3d_1;
	uint16_t reg_para1_koj_mixer_cal_3d_size_1;
	struct sp3d_reg_conf const *reg_para2_koj_init_cal_3d_s;
	uint16_t reg_para2_koj_init_cal_3d_s_size;
	struct sp3d_reg_conf const *reg_para2_koj_init_cal_3d_s_1;
	uint16_t reg_para2_koj_init_cal_3d_s_size_1;
	struct sp3d_reg_conf const *reg_para2_koj_init_cal_3d_s_2;
	uint16_t reg_para2_koj_init_cal_3d_s_size_2;
	struct sp3d_reg_conf const *reg_para3_koj_scs_cal_3d_s;
    	uint16_t reg_para3_koj_scs_cal_3d_s_size;
    	struct sp3d_reg_conf const *reg_para4_koj_start_cal_3d_s;
    	uint16_t reg_para4_koj_start_cal_3d_s_size;
    	struct sp3d_reg_conf const *reg_para5_koj_init_cal_3d_m;
    	uint16_t reg_para5_koj_init_cal_3d_m_size;
    	struct sp3d_reg_conf const *reg_para5_koj_init_cal_3d_m_1;
    	uint16_t reg_para5_koj_init_cal_3d_m_size_1;
    	struct sp3d_reg_conf const *reg_para5_koj_init_cal_3d_m_2;
    	uint16_t reg_para5_koj_init_cal_3d_m_size_2;
    	struct sp3d_reg_conf const *reg_para6_koj_scs_cal_3d_m;
    	uint16_t reg_para6_koj_scs_cal_3d_m_size;
    	struct sp3d_reg_conf const *reg_para7_koj_cal_3d_m;
    	uint16_t reg_para7_koj_cal_3d_m_size;
};

struct sp3d_snapshot_reg{
	struct sp3d_reg_conf const *reg_koj_shutter_on;
	uint16_t reg_koj_shutter_on_size;
	struct sp3d_reg_conf const *reg_koj_shutter_off;
	uint16_t reg_koj_shutter_off_size;
	struct sp3d_reg_conf const *reg_koj_continue1_shutter;
	uint16_t reg_koj_continue1_shutter_size;
	struct sp3d_reg_conf const *reg_koj_continue3_shutter;
	uint16_t reg_koj_continue3_shutter_size;
	struct sp3d_reg_conf const *reg_koj_2d_shutter_full;
	uint16_t reg_koj_2d_shutter_full_size;
	struct sp3d_reg_conf const *reg_koj_2d_shutter_full_INT;
	uint16_t reg_koj_2d_shutter_full_INT_size;
	struct sp3d_reg_conf const *reg_koj_2d_shutter_stop;
	uint16_t reg_koj_2d_shutter_stop_size;
	struct sp3d_reg_conf const *reg_koj_3d_shutter_full_m;
	uint16_t reg_koj_3d_shutter_full_m_size;
	struct sp3d_reg_conf const *reg_koj_3d_shutter_full_s;
	uint16_t reg_koj_3d_shutter_full_s_size;
	struct sp3d_reg_conf const *reg_koj_3d_shutter_stop;
	uint16_t reg_koj_3d_shutter_stop_size;
	struct sp3d_reg_conf *reg_koj_shutter_on_evoffset;
	uint16_t reg_koj_shutter_on_evoffset_size;
};

struct sp3d_com_feature_reg{
	struct sp3d_reg_conf const *reg_koj_effect_normal;
	uint16_t reg_koj_effect_normal_size;
	struct sp3d_reg_conf const *reg_koj_effect_BW;
	uint16_t reg_koj_effect_BW_size;
	struct sp3d_reg_conf const *reg_koj_effect_negative;
	uint16_t reg_koj_effect_negative_size;	
	struct sp3d_reg_conf const *reg_koj_effect_solarize;
	uint16_t reg_koj_effect_solarize_size;	
	struct sp3d_reg_conf const *reg_koj_effect_sepia;
	uint16_t reg_koj_effect_sepia_size;
	struct sp3d_reg_conf const *reg_koj_effect_posterize;
	uint16_t reg_koj_effect_posterize_size;
	struct sp3d_reg_conf const *reg_koj_effect_blue_tone;
	uint16_t reg_koj_effect_blue_tone_size;
	struct sp3d_reg_conf const *reg_koj_wb_auto;
	uint16_t reg_koj_wb_auto_size;
	struct sp3d_reg_conf const *reg_koj_wb_cloudy;
	uint16_t reg_koj_wb_cloudy_size;
	struct sp3d_reg_conf const *reg_koj_wb_fluorescent;
	uint16_t reg_koj_wb_fluorescent_size;	
	struct sp3d_reg_conf const *reg_koj_wb_incandescent;
	uint16_t reg_koj_wb_incandescent_size;	
	struct sp3d_reg_conf const *reg_koj_wb_daylight;
	uint16_t reg_koj_wb_daylight_size;
	struct sp3d_reg_conf const *reg_koj_iso_auto;
	uint16_t reg_koj_iso_auto_size;
	struct sp3d_reg_conf const *reg_koj_iso_100;
	uint16_t reg_koj_iso_100_size;
	struct sp3d_reg_conf const *reg_koj_iso_200;
	uint16_t reg_koj_iso_200_size;	
	struct sp3d_reg_conf const *reg_koj_iso_400;
	uint16_t reg_koj_iso_400_size;	
	struct sp3d_reg_conf const *reg_koj_iso_800;
	uint16_t reg_koj_iso_800_size;	
	struct sp3d_reg_conf const *reg_koj_ev_0;
	uint16_t reg_koj_ev_0_size;
	struct sp3d_reg_conf const *reg_koj_ev_1;
	uint16_t reg_koj_ev_1_size;
	struct sp3d_reg_conf const *reg_koj_ev_2;
	uint16_t reg_koj_ev_2_size;	
	struct sp3d_reg_conf const *reg_koj_ev_neg1;
	uint16_t reg_koj_ev_neg1_size;	
	struct sp3d_reg_conf const *reg_koj_ev_neg2;
	uint16_t reg_koj_ev_neg2_size;	
	struct sp3d_reg_conf const *reg_koj_contrast_0;
	uint16_t reg_koj_contrast_0_size;
	struct sp3d_reg_conf const *reg_koj_contrast_1;
	uint16_t reg_koj_contrast_1_size;
	struct sp3d_reg_conf const *reg_koj_contrast_2;
	uint16_t reg_koj_contrast_2_size;	
	struct sp3d_reg_conf const *reg_koj_contrast_neg1;
	uint16_t reg_koj_contrast_neg1_size;	
	struct sp3d_reg_conf const *reg_koj_contrast_neg2;
	uint16_t reg_koj_contrast_neg2_size;
	struct sp3d_reg_conf const *reg_koj_sharpness_0;
	uint16_t reg_koj_sharpness_0_size;
	struct sp3d_reg_conf const *reg_koj_sharpness_1;
	uint16_t reg_koj_sharpness_1_size;
	struct sp3d_reg_conf const *reg_koj_sharpness_2;
	uint16_t reg_koj_sharpness_2_size;
	struct sp3d_reg_conf const *reg_koj_sharpness_3;
	uint16_t reg_koj_sharpness_3_size;
	struct sp3d_reg_conf const *reg_koj_sharpness_4;
	uint16_t reg_koj_sharpness_4_size;
	struct sp3d_reg_conf const *reg_koj_sharpness_neg1;
	uint16_t reg_koj_sharpness_neg1_size;	
	struct sp3d_reg_conf const *reg_koj_sharpness_neg2;
	uint16_t reg_koj_sharpness_neg2_size;
	struct sp3d_reg_conf const *reg_koj_saturation_0;
	uint16_t reg_koj_saturation_0_size;
	struct sp3d_reg_conf const *reg_koj_saturation_1;
	uint16_t reg_koj_saturation_1_size;
	struct sp3d_reg_conf const *reg_koj_saturation_2;
	uint16_t reg_koj_saturation_2_size;	
	struct sp3d_reg_conf const *reg_koj_saturation_neg1;
	uint16_t reg_koj_saturation_neg1_size;	
	struct sp3d_reg_conf const *reg_koj_saturation_neg2;
	uint16_t reg_koj_saturation_neg2_size;
	struct sp3d_reg_conf const *reg_koj_antibanding_auto;
	uint16_t reg_koj_antibanding_auto_size;
	struct sp3d_reg_conf const *reg_koj_antibanding_50;
	uint16_t reg_koj_antibanding_50_size;
	struct sp3d_reg_conf const *reg_koj_antibanding_60;
	uint16_t reg_koj_antibanding_60_size;	
	struct sp3d_reg_conf const *reg_koj_af_mode_normal;
	uint16_t reg_koj_af_mode_normal_size;
	struct sp3d_reg_conf const *reg_koj_af_mode_macro;
	uint16_t reg_koj_af_mode_macro_size;
	struct sp3d_reg_conf const *reg_koj_af_mode_allrange;
	uint16_t reg_koj_af_mode_allrange_size;
	struct sp3d_reg_conf const *reg_pat_koj_af;
	uint16_t reg_pat_koj_af_size;
	struct sp3d_reg_conf const *reg_pat_koj_cancel_af;
	uint16_t reg_pat_koj_cancel_af_size;
};

struct sp3d_2d_reg {/*2D init setting*/
    struct sp3d_reg_conf const *reg_para1_koj_mixer_2d_htc;
    uint16_t reg_para1_koj_mixer_2d_htc_size;
    struct sp3d_reg_conf const *reg_para1_koj_mixer_2d_htc_1;
    uint16_t reg_para1_koj_mixer_2d_htc_size_1;

    struct sp3d_reg_conf const *reg_para2_koj_init_2d_htc_s;
    uint16_t reg_para2_koj_init_2d_htc_s_size;
    struct sp3d_reg_conf const *reg_para2_koj_init_2d_htc_s_1;
    uint16_t reg_para2_koj_init_2d_htc_s_size_1;
    struct sp3d_reg_conf const *reg_para2_koj_init_2d_htc_s_2;
    uint16_t reg_para2_koj_init_2d_htc_s_size_2;

    struct sp3d_reg_conf const *reg_para3_koj_scs_2d_boot_htc_s;
    uint16_t reg_para3_koj_scs_2d_boot_htc_s_size;
    struct sp3d_reg_conf const *reg_para4_koj_start_htc_s;
    uint16_t reg_para4_koj_start_htc_s_size;

    struct sp3d_reg_conf const *reg_para5_koj_init_2d_htc_m;
    uint16_t reg_para5_koj_init_2d_htc_m_size;
    struct sp3d_reg_conf const *reg_para5_koj_init_2d_htc_m_1;
    uint16_t reg_para5_koj_init_2d_htc_m_size_1;
    struct sp3d_reg_conf const *reg_para5_koj_init_2d_htc_m_2;
    uint16_t reg_para5_koj_init_2d_htc_m_size_2;

    struct sp3d_reg_conf const *reg_para6_koj_scs_2d_boot_htc_m;
    uint16_t reg_para6_koj_scs_2d_boot_htc_m_size;
    struct sp3d_reg_conf const *reg_para7_koj_start_htc_m;
    uint16_t reg_para7_koj_start_htc_m_size;
    struct sp3d_reg_conf const *reg_para8_koj_stop_htc_s;
    uint16_t reg_para8_koj_stop_htc_s_size;
    struct sp3d_reg_conf const *reg_para9_koj_scs_main_htc_s;
    uint32_t reg_para9_koj_scs_main_htc_s_size;
    struct sp3d_reg_conf const *reg_para10_koj_start_htc_s;
    uint16_t reg_para10_koj_start_htc_s_size;
    struct sp3d_reg_conf const *reg_para11_koj_stop_htc_m;
    uint16_t reg_para11_koj_stop_htc_m_size;
    struct sp3d_reg_conf const *reg_para12_koj_scs_main_htc_m;
    uint32_t reg_para12_koj_scs_main_htc_m_size;
    struct sp3d_reg_conf const *reg_para13_koj_start_htc_m;
    uint16_t reg_para13_koj_start_htc_m_size;

    struct sp3d_reg_conf const *reg_koj_2d_preview;
    uint16_t reg_koj_2d_preview_size;
    struct sp3d_reg_conf const *reg_koj_2d_720p_video;
    uint16_t reg_koj_2d_720p_video_size;

    struct sp3d_reg_conf const *reg_koj_2d_preview_15fps;
    uint16_t reg_koj_2d_preview_15fps_size;
    struct sp3d_reg_conf const *reg_koj_2d_720p_video_15fps;
    uint16_t reg_koj_2d_720p_video_15fps_size;
};

struct sp3d_3d_reg {/*3D setting*/
    struct sp3d_reg_conf const *reg_para1_koj_mixer_3d_htc;
    uint32_t reg_para1_koj_mixer_3d_htc_size;
    struct sp3d_reg_conf const *reg_para1_koj_mixer_3d_htc_1;
    uint32_t reg_para1_koj_mixer_3d_htc_size_1;

    struct sp3d_reg_conf const *reg_para2_koj_init_3d_htc_s;
    uint32_t reg_para2_koj_init_3d_htc_s_size;
    struct sp3d_reg_conf const *reg_para2_koj_init_3d_htc_s_1;
    uint32_t reg_para2_koj_init_3d_htc_s_size_1;
    struct sp3d_reg_conf const *reg_para2_koj_init_3d_htc_s_2;
    uint32_t reg_para2_koj_init_3d_htc_s_size_2;

    struct sp3d_reg_conf const *reg_para3_koj_scs_3d_boot_htc_s;
    uint32_t reg_para3_koj_scs_3d_boot_htc_s_size;
    struct sp3d_reg_conf const *reg_para4_koj_start_htc_s;
    uint32_t reg_para4_koj_start_htc_s_size;

    struct sp3d_reg_conf const *reg_para5_koj_init_3d_htc_m;
    uint32_t reg_para5_koj_init_3d_htc_m_size;
    struct sp3d_reg_conf const *reg_para5_koj_init_3d_htc_m_1;
    uint32_t reg_para5_koj_init_3d_htc_m_size_1;
    struct sp3d_reg_conf const *reg_para5_koj_init_3d_htc_m_2;
    uint32_t reg_para5_koj_init_3d_htc_m_size_2;

    struct sp3d_reg_conf const *reg_para6_koj_scs_3d_boot_htc_m;
    uint32_t reg_para6_koj_scs_3d_boot_htc_m_size;
    struct sp3d_reg_conf const *reg_para7_koj_start_htc_m;
    uint32_t reg_para7_koj_start_htc_m_size;
    struct sp3d_reg_conf const *reg_para8_koj_stop_htc_s;
    uint32_t reg_para8_koj_stop_htc_s_size;
    struct sp3d_reg_conf const *reg_para9_koj_scs_main_htc_s;
    uint32_t reg_para9_koj_scs_main_htc_s_size;
    struct sp3d_reg_conf const *reg_para10_koj_start_htc_s;
    uint32_t reg_para10_koj_start_htc_s_size;
    struct sp3d_reg_conf const *reg_para11_koj_stop_htc_m;
    uint32_t reg_para11_koj_stop_htc_m_size;
    struct sp3d_reg_conf const *reg_para12_koj_scs_main_htc_m;
    uint32_t reg_para12_koj_scs_main_htc_m_size;
    struct sp3d_reg_conf const *reg_para13_koj_start_htc_m;
    uint32_t reg_para13_koj_start_htc_m_size;
    struct sp3d_reg_conf const *reg_koj_3d_preview;
    uint32_t reg_koj_3d_preview_size;
    struct sp3d_reg_conf const *reg_koj_IQ_setting;
    uint32_t reg_koj_IQ_setting_size;
};

struct sp3d_feature_reg{
    struct sp3d_reg_conf const *reg_pat_koj_af;
    uint16_t reg_pat_koj_af_size;
};

#endif /* SP3D_H */
