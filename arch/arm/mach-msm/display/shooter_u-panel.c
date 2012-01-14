/* linux/arch/arm/mach-msm/display/shooter-panel_u.c
 *
 * Copyright (c) 2011 HTC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/panel_id.h>
#include <mach/msm_bus_board.h>
#include <linux/mfd/pmic8058.h>
#include <linux/leds-pm8058.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds.h>
#include <mach/debug_display.h>

#include "../devices.h"
#include "../board-shooter_u.h"
#include "../devices-msm8x60.h"
#include "../../../../drivers/video/msm_8x60/mdp_hw.h"
#include "../../../../drivers/video/msm_8x60/mipi_dsi.h"


extern int panel_type;

enum MODE_3D{
	LANDSCAPE_3D  = 0,
	PORTRAIT_3D,
	OFF_3D,
};

static struct pwm_device* pwm_3d = NULL;
static atomic_t g_3D_mode = ATOMIC_INIT(OFF_3D);
struct kobject *kobj, *uevent_kobj;
struct kset* uevent_kset;

void mdp_color_enhancement(const struct mdp_reg *reg_seq, int size);

static struct pm8058_gpio pwm_gpio_config = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM_GPIO_VIN_L5,
		.inv_int_pol	= 0,
};

static struct pm8058_gpio clk_gpio_config_on = {
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
};

static struct pm8058_gpio clk_gpio_config_off = {
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
};

/*
TODO:
1. move regulator initialization to shooter_u_panel_init()
*/
static struct regulator *l12_3v;
static struct regulator *lvs1_1v8;

static void shooter_u_panel_power(int onoff)
{
	static int init;
	int ret;
	int rc;
	static int isorise = 0;

	pr_info("%s(%d): init=%d onoff=%d panel_type=0x%x\n", __func__, __LINE__, init, onoff, panel_type);
	if(panel_type == 	PANEL_ID_SHR_SHARP_OTM ||
		panel_type == 	PANEL_ID_SHR_SHARP_OTM_C2)
		isorise = 1;
	/* If panel is already on (or off), do nothing. */
	if (!init) {
		l12_3v = regulator_get(NULL, "8058_l12");
		if (IS_ERR(l12_3v)) {
			pr_err("%s: unable to get 8058_l12\n", __func__);
			goto fail;
		}

		lvs1_1v8 = regulator_get(NULL, "8901_lvs1");
		if (IS_ERR(lvs1_1v8)) {
			pr_err("%s: unable to get 8901_lvs1\n", __func__);
			goto fail;
		}

		if(isorise == 0)
			ret = regulator_set_voltage(l12_3v, 3000000, 3000000);
		else
			ret = regulator_set_voltage(l12_3v, 3200000, 3200000);
		if (ret) {
			pr_err("%s: error setting l12_3v voltage\n", __func__);
			goto fail;
		}

		/* LCM Reset */
		rc = gpio_request(GPIO_LCM_RST_N,
			"LCM_RST_N");
		if (rc) {
			printk(KERN_ERR "%s:LCM gpio %d request"
				"failed\n", __func__,
				 GPIO_LCM_RST_N);
			return;
		}

		//gpio_direction_output(GPIO_LCM_RST_N, 0);
		init = 1;
	}

	if (!l12_3v || IS_ERR(l12_3v)) {
		pr_err("%s: l12_3v is not initialized\n", __func__);
		return;
	}

	if (!lvs1_1v8 || IS_ERR(lvs1_1v8)) {
		pr_err("%s: lvs1_1v8 is not initialized\n", __func__);
		return;
	}

	if (onoff) {
		if (regulator_enable(l12_3v)) {
			pr_err("%s: Unable to enable the regulator:"
					" l12_3v\n", __func__);
			return;
		}
		hr_msleep(1);

		if (regulator_enable(lvs1_1v8)) {
			pr_err("%s: Unable to enable the regulator:"
					" lvs1_1v8\n", __func__);
			return;
		}
		if ( init == 1 ) {
			init = 2;
			return;
		} else {
			if(isorise == 0)
				hr_msleep(10);
			else
				hr_msleep(1);
			gpio_set_value(GPIO_LCM_RST_N, 1);
			hr_msleep(1);
			gpio_set_value(GPIO_LCM_RST_N, 0);
			hr_msleep(1);
			gpio_set_value(GPIO_LCM_RST_N, 1);
			if(isorise == 0)
				hr_msleep(10);
			else
				hr_msleep(1);
		}
	} else {
		gpio_set_value(GPIO_LCM_RST_N, 0);
		hr_msleep(1);
		if (regulator_disable(lvs1_1v8)) {
			pr_err("%s: Unable to enable the regulator:"
					" lvs1_1v8\n", __func__);
			return;
		}
		hr_msleep(1);
		if (regulator_disable(l12_3v)) {
			pr_err("%s: Unable to enable the regulator:"
					" l12_3v\n", __func__);
			return;
		}
	}
	return;

fail:
	if (l12_3v)
		regulator_put(l12_3v);
	if (lvs1_1v8)
		regulator_put(lvs1_1v8);
}

static int mipi_panel_power(int on)
{
	int flag_on = !!on;
	static int mipi_power_save_on;

	if (mipi_power_save_on == flag_on)
		return 0;

	mipi_power_save_on = flag_on;

	shooter_u_panel_power(on);

	return 0;
}

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_smi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 147460000,
		.ib = 184325000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_ebi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 175110000,
		.ib = 218887500,
	},
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 175110000,
		.ib = 218887500,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 230400000,
		.ib = 288000000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 230400000,
		.ib = 288000000,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 334080000,
		.ib = 417600000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_smi_vectors),
		mdp_sd_smi_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_ebi_vectors),
		mdp_sd_ebi_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};
static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 435456000,
		.ib = 544320000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 435456000,
		.ib = 544320000,
	},
};
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif

static struct mipi_dsi_platform_data mipi_pdata = {
	.vsync_gpio = 28,
	.dsi_power_save   = mipi_panel_power,
};

static char novatek_unlock[] =
{
        0xF3, 0xAA,
};

static char novatek_lock[] =
{
        0xFF, 0xAA,
};

static char novatek_2vci[] =
{
        0x03, 0x33,
};

static char novatek_3vci[] =
{
        0x03, 0x36,
};

static struct dsi_cmd_desc novatek_2vci_cmds[] = {
        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
                sizeof(novatek_unlock), novatek_unlock},
        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
                sizeof(novatek_2vci), novatek_2vci},
        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
                sizeof(novatek_lock), novatek_lock},
};

static struct dsi_cmd_desc novatek_3vci_cmds[] = {
        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
                sizeof(novatek_unlock), novatek_unlock},
        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
                sizeof(novatek_2vci), novatek_3vci},
        {DTYPE_DCS_WRITE1, 1, 0, 0, 0,
                sizeof(novatek_lock), novatek_lock},
};

#define BRI_SETTING_MIN		30
#define BRI_SETTING_DEF		143
#define BRI_SETTING_MAX		255

#define PWM_MIN				8
#define PWM_DEFAULT			91
#define PWM_MAX				232

unsigned char shrink_br = BRI_SETTING_MAX;
unsigned char last_br_2d = BRI_SETTING_MAX;
static unsigned char shooter_u_shrink_pwm(int val)
{

	if (val <= 0) {
		shrink_br = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN)) {
		shrink_br = PWM_MIN;
	} else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
		shrink_br = (val - BRI_SETTING_MIN) * (PWM_DEFAULT - PWM_MIN) /
			(BRI_SETTING_DEF - BRI_SETTING_MIN) + PWM_MIN;
	} else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_MAX) {
		shrink_br = (val - BRI_SETTING_DEF) * (PWM_MAX - PWM_DEFAULT) /
			(BRI_SETTING_MAX - BRI_SETTING_DEF) + PWM_DEFAULT;
	} else if (val > BRI_SETTING_MAX)
		shrink_br = PWM_MAX;

	if (atomic_read(&g_3D_mode) != OFF_3D) {
		shrink_br = 254;
		//always refresh brightness value from AP if have but skip 254 which is trigger by 3Dpanel_on func
		if(val != 254)
			last_br_2d = val;
	}
	else if(val && atomic_read(&g_3D_mode) == OFF_3D)
		last_br_2d = val;

	PR_DISP_DEBUG("brightness orig=%d, transformed=%d last_2d %d\n", val, shrink_br, last_br_2d);

	return shrink_br;
}

static struct msm_panel_common_pdata mipi_novatek_panel_data = {
	.shrink_pwm = NULL,
};

static struct platform_device mipi_dsi_cmd_sharp_qhd_panel_device = {
	.name = "mipi_novatek",
	.id = 0,
	.dev = {
               .platform_data = &mipi_novatek_panel_data,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (panel_type == PANEL_ID_SHR_SHARP_NT) {
		if (!strcmp(name, "mipi_cmd_novatek_qhd"))
			return 0;
	}
	else if(panel_type == 	PANEL_ID_SHR_SHARP_OTM ||
		panel_type == 	PANEL_ID_SHR_SHARP_OTM_C2) {
		if (!strcmp(name, "mipi_cmd_orise_qhd"))
			return 0;
	}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	else if (!strcmp(name, "hdmi_msm"))
		return 0;
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.blt_mode = 1,
	.width = 53,
	.height = 95,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	//.num_resources     = ARRAY_SIZE(msm_fb_resources),
	//.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

int mdp_core_clk_rate_table[] = {
	59080000,
	128000000,
	160000000,
	200000000,
};

struct mdp_reg shooter_u_color_v11[] = {
	{0x93400, 0x0222, 0x0},
	{0x93404, 0xFFE4, 0x0},
	{0x93408, 0xFFFD, 0x0},
	{0x9340C, 0xFFF1, 0x0},
	{0x93410, 0x0212, 0x0},
	{0x93414, 0xFFF9, 0x0},
	{0x93418, 0xFFF1, 0x0},
	{0x9341C, 0xFFE6, 0x0},
	{0x93420, 0x022D, 0x0},
	{0x93600, 0x0000, 0x0},
	{0x93604, 0x00FF, 0x0},
	{0x93608, 0x0000, 0x0},
	{0x9360C, 0x00FF, 0x0},
	{0x93610, 0x0000, 0x0},
	{0x93614, 0x00FF, 0x0},
	{0x93680, 0x0000, 0x0},
	{0x93684, 0x00FF, 0x0},
	{0x93688, 0x0000, 0x0},
	{0x9368C, 0x00FF, 0x0},
	{0x93690, 0x0000, 0x0},
	{0x93694, 0x00FF, 0x0},
	{0x90070, 0xCD298008, 0x0},
};

struct mdp_reg mdp_sharp_barrier_on[] = {
{0x94800, 0x000000, 0x0},
{0x94804, 0x020202, 0x0},
{0x94808, 0x040404, 0x0},
{0x9480C, 0x060606, 0x0},
{0x94810, 0x080808, 0x0},
{0x94814, 0x090909, 0x0},
{0x94818, 0x0B0B0B, 0x0},
{0x9481C, 0x0C0C0C, 0x0},
{0x94820, 0x0E0E0E, 0x0},
{0x94824, 0x0F0F0F, 0x0},
{0x94828, 0x111111, 0x0},
{0x9482C, 0x121212, 0x0},
{0x94830, 0x141414, 0x0},
{0x94834, 0x151515, 0x0},
{0x94838, 0x161616, 0x0},
{0x9483C, 0x181818, 0x0},
{0x94840, 0x191919, 0x0},
{0x94844, 0x1A1A1A, 0x0},
{0x94848, 0x1C1C1C, 0x0},
{0x9484C, 0x1D1D1D, 0x0},
{0x94850, 0x1E1E1E, 0x0},
{0x94854, 0x1F1F1F, 0x0},
{0x94858, 0x212121, 0x0},
{0x9485C, 0x222222, 0x0},
{0x94860, 0x232323, 0x0},
{0x94864, 0x242424, 0x0},
{0x94868, 0x252525, 0x0},
{0x9486C, 0x272727, 0x0},
{0x94870, 0x282828, 0x0},
{0x94874, 0x292929, 0x0},
{0x94878, 0x2A2A2A, 0x0},
{0x9487C, 0x2B2B2B, 0x0},
{0x94880, 0x2D2D2D, 0x0},
{0x94884, 0x2E2E2E, 0x0},
{0x94888, 0x2F2F2F, 0x0},
{0x9488C, 0x303030, 0x0},
{0x94890, 0x313131, 0x0},
{0x94894, 0x323232, 0x0},
{0x94898, 0x343434, 0x0},
{0x9489C, 0x353535, 0x0},
{0x948A0, 0x363636, 0x0},
{0x948A4, 0x373737, 0x0},
{0x948A8, 0x383838, 0x0},
{0x948AC, 0x393939, 0x0},
{0x948B0, 0x3A3A3A, 0x0},
{0x948B4, 0x3B3B3B, 0x0},
{0x948B8, 0x3D3D3D, 0x0},
{0x948BC, 0x3E3E3E, 0x0},
{0x948C0, 0x3F3F3F, 0x0},
{0x948C4, 0x404040, 0x0},
{0x948C8, 0x414141, 0x0},
{0x948CC, 0x424242, 0x0},
{0x948D0, 0x434343, 0x0},
{0x948D4, 0x444444, 0x0},
{0x948D8, 0x454545, 0x0},
{0x948DC, 0x464646, 0x0},
{0x948E0, 0x474747, 0x0},
{0x948E4, 0x484848, 0x0},
{0x948E8, 0x4A4A4A, 0x0},
{0x948EC, 0x4B4B4B, 0x0},
{0x948F0, 0x4C4C4C, 0x0},
{0x948F4, 0x4D4D4D, 0x0},
{0x948F8, 0x4E4E4E, 0x0},
{0x948FC, 0x4F4F4F, 0x0},
{0x94900, 0x505050, 0x0},
{0x94904, 0x515151, 0x0},
{0x94908, 0x525252, 0x0},
{0x9490C, 0x535353, 0x0},
{0x94910, 0x545454, 0x0},
{0x94914, 0x555555, 0x0},
{0x94918, 0x565656, 0x0},
{0x9491C, 0x575757, 0x0},
{0x94920, 0x585858, 0x0},
{0x94924, 0x595959, 0x0},
{0x94928, 0x5A5A5A, 0x0},
{0x9492C, 0x5B5B5B, 0x0},
{0x94930, 0x5C5C5C, 0x0},
{0x94934, 0x5D5D5D, 0x0},
{0x94938, 0x5E5E5E, 0x0},
{0x9493C, 0x5F5F5F, 0x0},
{0x94940, 0x606060, 0x0},
{0x94944, 0x616161, 0x0},
{0x94948, 0x626262, 0x0},
{0x9494C, 0x636363, 0x0},
{0x94950, 0x646464, 0x0},
{0x94954, 0x656565, 0x0},
{0x94958, 0x666666, 0x0},
{0x9495C, 0x676767, 0x0},
{0x94960, 0x686868, 0x0},
{0x94964, 0x696969, 0x0},
{0x94968, 0x6A6A6A, 0x0},
{0x9496C, 0x6B6B6B, 0x0},
{0x94970, 0x6C6C6C, 0x0},
{0x94974, 0x6D6D6D, 0x0},
{0x94978, 0x6E6E6E, 0x0},
{0x9497C, 0x6F6F6F, 0x0},
{0x94980, 0x707070, 0x0},
{0x94984, 0x717171, 0x0},
{0x94988, 0x727272, 0x0},
{0x9498C, 0x737373, 0x0},
{0x94990, 0x747474, 0x0},
{0x94994, 0x757575, 0x0},
{0x94998, 0x767676, 0x0},
{0x9499C, 0x777777, 0x0},
{0x949A0, 0x787878, 0x0},
{0x949A4, 0x797979, 0x0},
{0x949A8, 0x7A7A7A, 0x0},
{0x949AC, 0x7B7B7B, 0x0},
{0x949B0, 0x7C7C7C, 0x0},
{0x949B4, 0x7D7D7D, 0x0},
{0x949B8, 0x7E7E7E, 0x0},
{0x949BC, 0x7F7F7F, 0x0},
{0x949C0, 0x808080, 0x0},
{0x949C4, 0x818181, 0x0},
{0x949C8, 0x828282, 0x0},
{0x949CC, 0x838383, 0x0},
{0x949D0, 0x848484, 0x0},
{0x949D4, 0x858585, 0x0},
{0x949D8, 0x858585, 0x0},
{0x949DC, 0x868686, 0x0},
{0x949E0, 0x878787, 0x0},
{0x949E4, 0x888888, 0x0},
{0x949E8, 0x898989, 0x0},
{0x949EC, 0x8A8A8A, 0x0},
{0x949F0, 0x8B8B8B, 0x0},
{0x949F4, 0x8C8C8C, 0x0},
{0x949F8, 0x8D8D8D, 0x0},
{0x949FC, 0x8E8E8E, 0x0},
{0x94A00, 0x8F8F8F, 0x0},
{0x94A04, 0x909090, 0x0},
{0x94A08, 0x919191, 0x0},
{0x94A0C, 0x929292, 0x0},
{0x94A10, 0x939393, 0x0},
{0x94A14, 0x949494, 0x0},
{0x94A18, 0x959595, 0x0},
{0x94A1C, 0x959595, 0x0},
{0x94A20, 0x969696, 0x0},
{0x94A24, 0x979797, 0x0},
{0x94A28, 0x989898, 0x0},
{0x94A2C, 0x999999, 0x0},
{0x94A30, 0x9A9A9A, 0x0},
{0x94A34, 0x9B9B9B, 0x0},
{0x94A38, 0x9C9C9C, 0x0},
{0x94A3C, 0x9D9D9D, 0x0},
{0x94A40, 0x9E9E9E, 0x0},
{0x94A44, 0x9F9F9F, 0x0},
{0x94A48, 0xA0A0A0, 0x0},
{0x94A4C, 0xA1A1A1, 0x0},
{0x94A50, 0xA1A1A1, 0x0},
{0x94A54, 0xA2A2A2, 0x0},
{0x94A58, 0xA3A3A3, 0x0},
{0x94A5C, 0xA4A4A4, 0x0},
{0x94A60, 0xA5A5A5, 0x0},
{0x94A64, 0xA6A6A6, 0x0},
{0x94A68, 0xA7A7A7, 0x0},
{0x94A6C, 0xA8A8A8, 0x0},
{0x94A70, 0xA9A9A9, 0x0},
{0x94A74, 0xAAAAAA, 0x0},
{0x94A78, 0xABABAB, 0x0},
{0x94A7C, 0xABABAB, 0x0},
{0x94A80, 0xACACAC, 0x0},
{0x94A84, 0xADADAD, 0x0},
{0x94A88, 0xAEAEAE, 0x0},
{0x94A8C, 0xAFAFAF, 0x0},
{0x94A90, 0xB0B0B0, 0x0},
{0x94A94, 0xB1B1B1, 0x0},
{0x94A98, 0xB2B2B2, 0x0},
{0x94A9C, 0xB3B3B3, 0x0},
{0x94AA0, 0xB4B4B4, 0x0},
{0x94AA4, 0xB4B4B4, 0x0},
{0x94AA8, 0xB5B5B5, 0x0},
{0x94AAC, 0xB6B6B6, 0x0},
{0x94AB0, 0xB7B7B7, 0x0},
{0x94AB4, 0xB8B8B8, 0x0},
{0x94AB8, 0xB9B9B9, 0x0},
{0x94ABC, 0xBABABA, 0x0},
{0x94AC0, 0xBBBBBB, 0x0},
{0x94AC4, 0xBCBCBC, 0x0},
{0x94AC8, 0xBDBDBD, 0x0},
{0x94ACC, 0xBDBDBD, 0x0},
{0x94AD0, 0xBEBEBE, 0x0},
{0x94AD4, 0xBFBFBF, 0x0},
{0x94AD8, 0xC0C0C0, 0x0},
{0x94ADC, 0xC1C1C1, 0x0},
{0x94AE0, 0xC2C2C2, 0x0},
{0x94AE4, 0xC3C3C3, 0x0},
{0x94AE8, 0xC4C4C4, 0x0},
{0x94AEC, 0xC5C5C5, 0x0},
{0x94AF0, 0xC5C5C5, 0x0},
{0x94AF4, 0xC6C6C6, 0x0},
{0x94AF8, 0xC7C7C7, 0x0},
{0x94AFC, 0xC8C8C8, 0x0},
{0x94B00, 0xC9C9C9, 0x0},
{0x94B04, 0xCACACA, 0x0},
{0x94B08, 0xCBCBCB, 0x0},
{0x94B0C, 0xCCCCCC, 0x0},
{0x94B10, 0xCCCCCC, 0x0},
{0x94B14, 0xCDCDCD, 0x0},
{0x94B18, 0xCECECE, 0x0},
{0x94B1C, 0xCFCFCF, 0x0},
{0x94B20, 0xD0D0D0, 0x0},
{0x94B24, 0xD1D1D1, 0x0},
{0x94B28, 0xD2D2D2, 0x0},
{0x94B2C, 0xD3D3D3, 0x0},
{0x94B30, 0xD3D3D3, 0x0},
{0x94B34, 0xD4D4D4, 0x0},
{0x94B38, 0xD5D5D5, 0x0},
{0x94B3C, 0xD6D6D6, 0x0},
{0x94B40, 0xD7D7D7, 0x0},
{0x94B44, 0xD8D8D8, 0x0},
{0x94B48, 0xD9D9D9, 0x0},
{0x94B4C, 0xD9D9D9, 0x0},
{0x94B50, 0xDADADA, 0x0},
{0x94B54, 0xDBDBDB, 0x0},
{0x94B58, 0xDCDCDC, 0x0},
{0x94B5C, 0xDDDDDD, 0x0},
{0x94B60, 0xDEDEDE, 0x0},
{0x94B64, 0xDFDFDF, 0x0},
{0x94B68, 0xE0E0E0, 0x0},
{0x94B6C, 0xE0E0E0, 0x0},
{0x94B70, 0xE1E1E1, 0x0},
{0x94B74, 0xE2E2E2, 0x0},
{0x94B78, 0xE3E3E3, 0x0},
{0x94B7C, 0xE4E4E4, 0x0},
{0x94B80, 0xE5E5E5, 0x0},
{0x94B84, 0xE6E6E6, 0x0},
{0x94B88, 0xE6E6E6, 0x0},
{0x94B8C, 0xE7E7E7, 0x0},
{0x94B90, 0xE8E8E8, 0x0},
{0x94B94, 0xE9E9E9, 0x0},
{0x94B98, 0xEAEAEA, 0x0},
{0x94B9C, 0xEBEBEB, 0x0},
{0x94BA0, 0xECECEC, 0x0},
{0x94BA4, 0xECECEC, 0x0},
{0x94BA8, 0xEDEDED, 0x0},
{0x94BAC, 0xEEEEEE, 0x0},
{0x94BB0, 0xEFEFEF, 0x0},
{0x94BB4, 0xF0F0F0, 0x0},
{0x94BB8, 0xF1F1F1, 0x0},
{0x94BBC, 0xF1F1F1, 0x0},
{0x94BC0, 0xF2F2F2, 0x0},
{0x94BC4, 0xF3F3F3, 0x0},
{0x94BC8, 0xF4F4F4, 0x0},
{0x94BCC, 0xF5F5F5, 0x0},
{0x94BD0, 0xF6F6F6, 0x0},
{0x94BD4, 0xF7F7F7, 0x0},
{0x94BD8, 0xF7F7F7, 0x0},
{0x94BDC, 0xF8F8F8, 0x0},
{0x94BE0, 0xF9F9F9, 0x0},
{0x94BE4, 0xFAFAFA, 0x0},
{0x94BE8, 0xFBFBFB, 0x0},
{0x94BEC, 0xFCFCFC, 0x0},
{0x94BF0, 0xFCFCFC, 0x0},
{0x94BF4, 0xFDFDFD, 0x0},
{0x94BF8, 0xFEFEFE, 0x0},
{0x94BFC, 0xFFFFFF, 0x0},
{0x90070, 0x17    , 0x17},
};

struct mdp_reg mdp_sharp_barrier_off[] = {
	{0x94800, 0x000000, 0x0},
	{0x94804, 0x010101, 0x0},
	{0x94808, 0x020202, 0x0},
	{0x9480C, 0x030303, 0x0},
	{0x94810, 0x040404, 0x0},
	{0x94814, 0x050505, 0x0},
	{0x94818, 0x060606, 0x0},
	{0x9481C, 0x070707, 0x0},
	{0x94820, 0x080808, 0x0},
	{0x94824, 0x090909, 0x0},
	{0x94828, 0x0A0A0A, 0x0},
	{0x9482C, 0x0B0B0B, 0x0},
	{0x94830, 0x0C0C0C, 0x0},
	{0x94834, 0x0D0D0D, 0x0},
	{0x94838, 0x0E0E0E, 0x0},
	{0x9483C, 0x0F0F0F, 0x0},
	{0x94840, 0x101010, 0x0},
	{0x94844, 0x111111, 0x0},
	{0x94848, 0x121212, 0x0},
	{0x9484C, 0x131313, 0x0},
	{0x94850, 0x141414, 0x0},
	{0x94854, 0x151515, 0x0},
	{0x94858, 0x161616, 0x0},
	{0x9485C, 0x171717, 0x0},
	{0x94860, 0x181818, 0x0},
	{0x94864, 0x191919, 0x0},
	{0x94868, 0x1A1A1A, 0x0},
	{0x9486C, 0x1B1B1B, 0x0},
	{0x94870, 0x1C1C1C, 0x0},
	{0x94874, 0x1D1D1D, 0x0},
	{0x94878, 0x1E1E1E, 0x0},
	{0x9487C, 0x1F1F1F, 0x0},
	{0x94880, 0x202020, 0x0},
	{0x94884, 0x212121, 0x0},
	{0x94888, 0x222222, 0x0},
	{0x9488C, 0x232323, 0x0},
	{0x94890, 0x242424, 0x0},
	{0x94894, 0x252525, 0x0},
	{0x94898, 0x262626, 0x0},
	{0x9489C, 0x272727, 0x0},
	{0x948A0, 0x282828, 0x0},
	{0x948A4, 0x292929, 0x0},
	{0x948A8, 0x2A2A2A, 0x0},
	{0x948AC, 0x2B2B2B, 0x0},
	{0x948B0, 0x2C2C2C, 0x0},
	{0x948B4, 0x2D2D2D, 0x0},
	{0x948B8, 0x2E2E2E, 0x0},
	{0x948BC, 0x2F2F2F, 0x0},
	{0x948C0, 0x303030, 0x0},
	{0x948C4, 0x313131, 0x0},
	{0x948C8, 0x323232, 0x0},
	{0x948CC, 0x333333, 0x0},
	{0x948D0, 0x343434, 0x0},
	{0x948D4, 0x353535, 0x0},
	{0x948D8, 0x363636, 0x0},
	{0x948DC, 0x373737, 0x0},
	{0x948E0, 0x383838, 0x0},
	{0x948E4, 0x393939, 0x0},
	{0x948E8, 0x3A3A3A, 0x0},
	{0x948EC, 0x3B3B3B, 0x0},
	{0x948F0, 0x3C3C3C, 0x0},
	{0x948F4, 0x3D3D3D, 0x0},
	{0x948F8, 0x3E3E3E, 0x0},
	{0x948FC, 0x3F3F3F, 0x0},
	{0x94900, 0x404040, 0x0},
	{0x94904, 0x414141, 0x0},
	{0x94908, 0x424242, 0x0},
	{0x9490C, 0x434343, 0x0},
	{0x94910, 0x444444, 0x0},
	{0x94914, 0x454545, 0x0},
	{0x94918, 0x464646, 0x0},
	{0x9491C, 0x474747, 0x0},
	{0x94920, 0x484848, 0x0},
	{0x94924, 0x494949, 0x0},
	{0x94928, 0x4A4A4A, 0x0},
	{0x9492C, 0x4B4B4B, 0x0},
	{0x94930, 0x4C4C4C, 0x0},
	{0x94934, 0x4D4D4D, 0x0},
	{0x94938, 0x4E4E4E, 0x0},
	{0x9493C, 0x4F4F4F, 0x0},
	{0x94940, 0x505050, 0x0},
	{0x94944, 0x515151, 0x0},
	{0x94948, 0x525252, 0x0},
	{0x9494C, 0x535353, 0x0},
	{0x94950, 0x545454, 0x0},
	{0x94954, 0x555555, 0x0},
	{0x94958, 0x565656, 0x0},
	{0x9495C, 0x575757, 0x0},
	{0x94960, 0x585858, 0x0},
	{0x94964, 0x595959, 0x0},
	{0x94968, 0x5A5A5A, 0x0},
	{0x9496C, 0x5B5B5B, 0x0},
	{0x94970, 0x5C5C5C, 0x0},
	{0x94974, 0x5D5D5D, 0x0},
	{0x94978, 0x5E5E5E, 0x0},
	{0x9497C, 0x5F5F5F, 0x0},
	{0x94980, 0x606060, 0x0},
	{0x94984, 0x616161, 0x0},
	{0x94988, 0x626262, 0x0},
	{0x9498C, 0x636363, 0x0},
	{0x94990, 0x646464, 0x0},
	{0x94994, 0x656565, 0x0},
	{0x94998, 0x666666, 0x0},
	{0x9499C, 0x676767, 0x0},
	{0x949A0, 0x686868, 0x0},
	{0x949A4, 0x696969, 0x0},
	{0x949A8, 0x6A6A6A, 0x0},
	{0x949AC, 0x6B6B6B, 0x0},
	{0x949B0, 0x6C6C6C, 0x0},
	{0x949B4, 0x6D6D6D, 0x0},
	{0x949B8, 0x6E6E6E, 0x0},
	{0x949BC, 0x6F6F6F, 0x0},
	{0x949C0, 0x707070, 0x0},
	{0x949C4, 0x717171, 0x0},
	{0x949C8, 0x727272, 0x0},
	{0x949CC, 0x737373, 0x0},
	{0x949D0, 0x747474, 0x0},
	{0x949D4, 0x757575, 0x0},
	{0x949D8, 0x767676, 0x0},
	{0x949DC, 0x777777, 0x0},
	{0x949E0, 0x787878, 0x0},
	{0x949E4, 0x797979, 0x0},
	{0x949E8, 0x7A7A7A, 0x0},
	{0x949EC, 0x7B7B7B, 0x0},
	{0x949F0, 0x7C7C7C, 0x0},
	{0x949F4, 0x7D7D7D, 0x0},
	{0x949F8, 0x7E7E7E, 0x0},
	{0x949FC, 0x7F7F7F, 0x0},
	{0x94A00, 0x808080, 0x0},
	{0x94A04, 0x818181, 0x0},
	{0x94A08, 0x828282, 0x0},
	{0x94A0C, 0x838383, 0x0},
	{0x94A10, 0x848484, 0x0},
	{0x94A14, 0x858585, 0x0},
	{0x94A18, 0x868686, 0x0},
	{0x94A1C, 0x878787, 0x0},
	{0x94A20, 0x888888, 0x0},
	{0x94A24, 0x898989, 0x0},
	{0x94A28, 0x8A8A8A, 0x0},
	{0x94A2C, 0x8B8B8B, 0x0},
	{0x94A30, 0x8C8C8C, 0x0},
	{0x94A34, 0x8D8D8D, 0x0},
	{0x94A38, 0x8E8E8E, 0x0},
	{0x94A3C, 0x8F8F8F, 0x0},
	{0x94A40, 0x909090, 0x0},
	{0x94A44, 0x919191, 0x0},
	{0x94A48, 0x929292, 0x0},
	{0x94A4C, 0x939393, 0x0},
	{0x94A50, 0x949494, 0x0},
	{0x94A54, 0x959595, 0x0},
	{0x94A58, 0x969696, 0x0},
	{0x94A5C, 0x979797, 0x0},
	{0x94A60, 0x989898, 0x0},
	{0x94A64, 0x999999, 0x0},
	{0x94A68, 0x9A9A9A, 0x0},
	{0x94A6C, 0x9B9B9B, 0x0},
	{0x94A70, 0x9C9C9C, 0x0},
	{0x94A74, 0x9D9D9D, 0x0},
	{0x94A78, 0x9E9E9E, 0x0},
	{0x94A7C, 0x9F9F9F, 0x0},
	{0x94A80, 0xA0A0A0, 0x0},
	{0x94A84, 0xA1A1A1, 0x0},
	{0x94A88, 0xA2A2A2, 0x0},
	{0x94A8C, 0xA3A3A3, 0x0},
	{0x94A90, 0xA4A4A4, 0x0},
	{0x94A94, 0xA5A5A5, 0x0},
	{0x94A98, 0xA6A6A6, 0x0},
	{0x94A9C, 0xA7A7A7, 0x0},
	{0x94AA0, 0xA8A8A8, 0x0},
	{0x94AA4, 0xA9A9A9, 0x0},
	{0x94AA8, 0xAAAAAA, 0x0},
	{0x94AAC, 0xABABAB, 0x0},
	{0x94AB0, 0xACACAC, 0x0},
	{0x94AB4, 0xADADAD, 0x0},
	{0x94AB8, 0xAEAEAE, 0x0},
	{0x94ABC, 0xAFAFAF, 0x0},
	{0x94AC0, 0xB0B0B0, 0x0},
	{0x94AC4, 0xB1B1B1, 0x0},
	{0x94AC8, 0xB2B2B2, 0x0},
	{0x94ACC, 0xB3B3B3, 0x0},
	{0x94AD0, 0xB4B4B4, 0x0},
	{0x94AD4, 0xB5B5B5, 0x0},
	{0x94AD8, 0xB6B6B6, 0x0},
	{0x94ADC, 0xB7B7B7, 0x0},
	{0x94AE0, 0xB8B8B8, 0x0},
	{0x94AE4, 0xB9B9B9, 0x0},
	{0x94AE8, 0xBABABA, 0x0},
	{0x94AEC, 0xBBBBBB, 0x0},
	{0x94AF0, 0xBCBCBC, 0x0},
	{0x94AF4, 0xBDBDBD, 0x0},
	{0x94AF8, 0xBEBEBE, 0x0},
	{0x94AFC, 0xBFBFBF, 0x0},
	{0x94B00, 0xC0C0C0, 0x0},
	{0x94B04, 0xC1C1C1, 0x0},
	{0x94B08, 0xC2C2C2, 0x0},
	{0x94B0C, 0xC3C3C3, 0x0},
	{0x94B10, 0xC4C4C4, 0x0},
	{0x94B14, 0xC5C5C5, 0x0},
	{0x94B18, 0xC6C6C6, 0x0},
	{0x94B1C, 0xC7C7C7, 0x0},
	{0x94B20, 0xC8C8C8, 0x0},
	{0x94B24, 0xC9C9C9, 0x0},
	{0x94B28, 0xCACACA, 0x0},
	{0x94B2C, 0xCBCBCB, 0x0},
	{0x94B30, 0xCCCCCC, 0x0},
	{0x94B34, 0xCDCDCD, 0x0},
	{0x94B38, 0xCECECE, 0x0},
	{0x94B3C, 0xCFCFCF, 0x0},
	{0x94B40, 0xD0D0D0, 0x0},
	{0x94B44, 0xD1D1D1, 0x0},
	{0x94B48, 0xD2D2D2, 0x0},
	{0x94B4C, 0xD3D3D3, 0x0},
	{0x94B50, 0xD4D4D4, 0x0},
	{0x94B54, 0xD5D5D5, 0x0},
	{0x94B58, 0xD6D6D6, 0x0},
	{0x94B5C, 0xD7D7D7, 0x0},
	{0x94B60, 0xD8D8D8, 0x0},
	{0x94B64, 0xD9D9D9, 0x0},
	{0x94B68, 0xDADADA, 0x0},
	{0x94B6C, 0xDBDBDB, 0x0},
	{0x94B70, 0xDCDCDC, 0x0},
	{0x94B74, 0xDDDDDD, 0x0},
	{0x94B78, 0xDEDEDE, 0x0},
	{0x94B7C, 0xDFDFDF, 0x0},
	{0x94B80, 0xE0E0E0, 0x0},
	{0x94B84, 0xE1E1E1, 0x0},
	{0x94B88, 0xE2E2E2, 0x0},
	{0x94B8C, 0xE3E3E3, 0x0},
	{0x94B90, 0xE4E4E4, 0x0},
	{0x94B94, 0xE5E5E5, 0x0},
	{0x94B98, 0xE6E6E6, 0x0},
	{0x94B9C, 0xE7E7E7, 0x0},
	{0x94BA0, 0xE8E8E8, 0x0},
	{0x94BA4, 0xE9E9E9, 0x0},
	{0x94BA8, 0xEAEAEA, 0x0},
	{0x94BAC, 0xEBEBEB, 0x0},
	{0x94BB0, 0xECECEC, 0x0},
	{0x94BB4, 0xEDEDED, 0x0},
	{0x94BB8, 0xEEEEEE, 0x0},
	{0x94BBC, 0xEFEFEF, 0x0},
	{0x94BC0, 0xF0F0F0, 0x0},
	{0x94BC4, 0xF1F1F1, 0x0},
	{0x94BC8, 0xF2F2F2, 0x0},
	{0x94BCC, 0xF3F3F3, 0x0},
	{0x94BD0, 0xF4F4F4, 0x0},
	{0x94BD4, 0xF5F5F5, 0x0},
	{0x94BD8, 0xF6F6F6, 0x0},
	{0x94BDC, 0xF7F7F7, 0x0},
	{0x94BE0, 0xF8F8F8, 0x0},
	{0x94BE4, 0xF9F9F9, 0x0},
	{0x94BE8, 0xFAFAFA, 0x0},
	{0x94BEC, 0xFBFBFB, 0x0},
	{0x94BF0, 0xFCFCFC, 0x0},
	{0x94BF4, 0xFDFDFD, 0x0},
	{0x94BF8, 0xFEFEFE, 0x0},
	{0x94BFC, 0xFFFFFF, 0x0},
	{0x90070, 0x17	  , 0x17},
};


int shooter_u_mdp_color_enhance(void)
{
	PR_DISP_INFO("%s\n", __func__);
	//mdp_color_enhancement(shooter_u_color_v11, ARRAY_SIZE(shooter_u_color_v11));

	return 0;
}

int shooter_u_mdp_gamma(void)
{
	PR_DISP_INFO("%s\n", __func__);
	mdp_color_enhancement(mdp_sharp_barrier_off, ARRAY_SIZE(mdp_sharp_barrier_off));

	return 0;
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 28,
	.mdp_core_clk_rate = 200000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_color_enhance = shooter_u_mdp_color_enhance,
	.mdp_gamma = shooter_u_mdp_gamma,
};

static void __init msm_fb_add_devices(void)
{
	printk(KERN_INFO "panel ID= 0x%x\n", panel_type);
	msm_fb_register_device("mdp", &mdp_pdata);

	if (panel_type != PANEL_ID_NONE)
		msm_fb_register_device("mipi_dsi", &mipi_pdata);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

/*
TODO:
1.find a better way to handle msm_fb_resources, to avoid passing it across file.
2.error handling
 */
int __init shooter_u_init_panel(struct resource *res, size_t size)
{
	int ret=0;

	if(panel_type == 	PANEL_ID_SHR_SHARP_OTM ||
		panel_type == 	PANEL_ID_SHR_SHARP_OTM_C2)
		mipi_dsi_cmd_sharp_qhd_panel_device.name = "mipi_orise";

	PR_DISP_INFO("%s: %s\n", __func__, mipi_dsi_cmd_sharp_qhd_panel_device.name);

	mipi_novatek_panel_data.shrink_pwm = shooter_u_shrink_pwm;

	msm_fb_device.resource = res;
	msm_fb_device.num_resources = size;

	ret = platform_device_register(&msm_fb_device);
	ret = platform_device_register(&mipi_dsi_cmd_sharp_qhd_panel_device);

	msm_fb_add_devices();

	return 0;
}

static void shooter_u_3Dpanel_on(bool bLandscape)
{
	struct pw8058_pwm_config pwm_conf;
	int rc;

	led_brightness_switch("lcd-backlight", 254);

	if (mipi_novatek_panel_data.mipi_send_cmds) {
		mipi_novatek_panel_data.mipi_send_cmds(novatek_3vci_cmds, ARRAY_SIZE(novatek_3vci_cmds));
	}
	pwm_gpio_config.output_value = 1;
	rc = pm8058_gpio_config(SHOOTER_U_3DLCM_PD, &pwm_gpio_config);
	if (rc < 0)
		pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_U_3DLCM_PD);

	rc = pm8058_gpio_config(SHOOTER_U_3DCLK, &clk_gpio_config_on);
	if (rc < 0)
		pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_U_3DCLK);

	pwm_disable(pwm_3d);
	pwm_conf.pwm_size = 9;
	pwm_conf.clk = PM_PWM_CLK_19P2MHZ;
	pwm_conf.pre_div = PM_PWM_PREDIVIDE_3;
	pwm_conf.pre_div_exp = 6;
	pwm_conf.pwm_value = 255;
	pwm_conf.bypass_lut = 1;
	pwm_configure(pwm_3d, &pwm_conf);
	pwm_enable(pwm_3d);

	if(bLandscape) {
		mdp_color_enhancement(mdp_sharp_barrier_on, ARRAY_SIZE(mdp_sharp_barrier_on));
		gpio_set_value(SHOOTER_U_CTL_3D_1, 1);
		gpio_set_value(SHOOTER_U_CTL_3D_2, 1);
		gpio_set_value(SHOOTER_U_CTL_3D_3, 1);
		gpio_set_value(SHOOTER_U_CTL_3D_4, 0);
	} else {
		mdp_color_enhancement(mdp_sharp_barrier_on, ARRAY_SIZE(mdp_sharp_barrier_on));
		gpio_set_value(SHOOTER_U_CTL_3D_1, 1);
		gpio_set_value(SHOOTER_U_CTL_3D_2, 1);
		gpio_set_value(SHOOTER_U_CTL_3D_3, 0);
		gpio_set_value(SHOOTER_U_CTL_3D_4, 1);
	}
}

static void shooter_u_3Dpanel_off(void)
{
	int rc;
	pwm_gpio_config.output_value = 0;

	if (mipi_novatek_panel_data.mipi_send_cmds) {
		mipi_novatek_panel_data.mipi_send_cmds(novatek_2vci_cmds, ARRAY_SIZE(novatek_2vci_cmds));
	}

	rc = pm8058_gpio_config(SHOOTER_U_3DLCM_PD, &pwm_gpio_config);
	if (rc < 0)
		pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_U_3DLCM_PD);
	mdp_color_enhancement(mdp_sharp_barrier_off, ARRAY_SIZE(mdp_sharp_barrier_off));
	pwm_disable(pwm_3d);

	rc = pm8058_gpio_config(SHOOTER_U_3DCLK, &clk_gpio_config_off);
	if (rc < 0)
		pr_err("%s pmic gpio config gpio %d failed\n", __func__, SHOOTER_U_3DCLK);
	gpio_set_value(SHOOTER_U_CTL_3D_1, 0);
	gpio_set_value(SHOOTER_U_CTL_3D_2, 0);
	gpio_set_value(SHOOTER_U_CTL_3D_3, 0);
	gpio_set_value(SHOOTER_U_CTL_3D_4, 0);
	led_brightness_switch("lcd-backlight", last_br_2d);
}

static ssize_t show_3D_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += scnprintf(buf+len, PAGE_SIZE-1, "%d\n", atomic_read(&g_3D_mode));
	return len;
}

static ssize_t store_3D_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	if ((buf[0] == '0' || buf[0] == '1' || buf[0] == '2')
						&& buf[1] == '\n') {
		val = buf[0] - 0x30;
		if (val == atomic_read(&g_3D_mode)) {
			printk(KERN_NOTICE "%s: status is same(%d)\n", __func__, atomic_read(&g_3D_mode));
			return count;
		}

		atomic_set(&g_3D_mode, val);
		PR_DISP_INFO("%s mode = %d\n", __func__, val);
		switch (val) {
		case LANDSCAPE_3D:
			shooter_u_3Dpanel_on(true);
			break;
		case PORTRAIT_3D:
			shooter_u_3Dpanel_on(false);
			break;
		case OFF_3D:
			shooter_u_3Dpanel_off();
			break;
		default:
			break;
		}
	}
	return count;
}

static DEVICE_ATTR(3D_mode, 0666, show_3D_mode, store_3D_mode);

static int shooter_u_3Dpanel_probe(struct platform_device * pdev)
{
	int err = 0;

	printk(KERN_INFO "%s\n", __func__);

	if(pwm_3d == NULL) {
		pwm_3d = pwm_request(2, "3DCLK");
	}
	err = device_create_file(&pdev->dev, &dev_attr_3D_mode);
	if (err != 0)
		printk(KERN_WARNING "attr_3D_mode failed\n");

	return 0;
}

static int shooter_u_3Dpanel_remove(struct platform_device * pdev)
{
	printk(KERN_INFO "%s\n", __func__);
	if(pwm_3d) {
		pwm_free(pwm_3d);
		pwm_3d = NULL;
	}
	device_remove_file(&pdev->dev, &dev_attr_3D_mode);
	kobject_del(uevent_kobj);
	kobject_del(kobj);
	return 0;
}

struct platform_driver shooter_u_3Dpanel_driver = {
	.probe	= shooter_u_3Dpanel_probe,
	.remove	= shooter_u_3Dpanel_remove,
	.driver	= {
		.name = "panel_3d",
	},
};

static int __init shooter_u_3Dpanel_init(void)
{
	pr_info("%s(%d)\n", __func__, __LINE__);
	return platform_driver_register(&shooter_u_3Dpanel_driver);
}

static void __exit shooter_u_3Dpanel_exit(void)
{
	platform_driver_unregister(&shooter_u_3Dpanel_driver);
}

module_init(shooter_u_3Dpanel_init);
module_exit(shooter_u_3Dpanel_exit);
