/* linux/arch/arm/mach-msm/display/rider-panel.c
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
#include <mach/debug_display.h>

#include "../devices.h"
#include "../board-rider.h"
#include "../devices-msm8x60.h"
#include "../../../../drivers/video/msm_8x60/mdp_hw.h"

extern int panel_type;

/*
TODO:
1. move regulator initialization to shooter_panel_init()
*/
static struct regulator *l12_3v;
static struct regulator *lvs1_1v8;

static void rider_panel_power(int onoff)
{
	static int init;
	int ret;
	int rc;
	static int isorise = 0;

	pr_info("%s(%d): init=%d onoff=%d panel_type=%d\n", __func__, __LINE__, init, onoff, panel_type);
	if (panel_type == PANEL_ID_RIR_AUO_OTM_C3 || panel_type == PANEL_ID_RIR_AUO_OTM_C2 || panel_type == PANEL_ID_RIR_SHARP_OTM || panel_type == PANEL_ID_RIR_AUO_OTM)
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

		ret = regulator_set_voltage(l12_3v, 3000000, 3000000);
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

	rider_panel_power(on);

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

#define BRI_SETTING_MIN                 30
#define BRI_SETTING_DEF                 143
#define BRI_SETTING_MAX                 255

#define PWM_MIN                   6       /* 3.5% of max pwm */
#define PWM_DEFAULT               96     /* 43% of max pwm  */
#define PWM_MAX                   255

static unsigned char rider_shrink_pwm(int val)
{
	unsigned char shrink_br = BRI_SETTING_MAX;

	if (val <= 0) {
		shrink_br = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN)) {
			shrink_br = PWM_MIN;
	} else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
			shrink_br = (val - 30) * (PWM_DEFAULT - PWM_MIN) /
		(BRI_SETTING_DEF - BRI_SETTING_MIN) + PWM_MIN;
	} else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_MAX) {
			shrink_br = (val - 143) * (PWM_MAX - PWM_DEFAULT) /
		(BRI_SETTING_MAX - BRI_SETTING_DEF) + PWM_DEFAULT;
	} else if (val > BRI_SETTING_MAX)
			shrink_br = PWM_MAX;

	PR_DISP_DEBUG("brightness orig=%d, transformed=%d\n", val, shrink_br);

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
	if (panel_type == PANEL_ID_RIR_SHARP_NT || panel_type == PANEL_ID_RIR_AUO_NT) {
		if (!strcmp(name, "mipi_cmd_novatek_qhd"))
			return 0;
	} else if (panel_type == PANEL_ID_RIR_AUO_OTM_C3 || panel_type == PANEL_ID_RIR_AUO_OTM_C2 || panel_type == PANEL_ID_RIR_SHARP_OTM || panel_type == PANEL_ID_RIR_AUO_OTM) {
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

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 28,
	.mdp_core_clk_rate = 200000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
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
int __init rider_init_panel(struct resource *res, size_t size)
{
	int ret=0;

	if (panel_type == PANEL_ID_RIR_AUO_OTM_C3 || panel_type == PANEL_ID_RIR_AUO_OTM_C2 || panel_type == PANEL_ID_RIR_AUO_OTM)
		mipi_dsi_cmd_sharp_qhd_panel_device.name = "mipi_orise";

	PR_DISP_INFO("%s: %s\n", __func__, mipi_dsi_cmd_sharp_qhd_panel_device.name);

	mipi_novatek_panel_data.shrink_pwm = rider_shrink_pwm;

	msm_fb_device.resource = res;
	msm_fb_device.num_resources = size;

	ret = platform_device_register(&msm_fb_device);
	ret = platform_device_register(&mipi_dsi_cmd_sharp_qhd_panel_device);

	msm_fb_add_devices();

	return 0;
}
