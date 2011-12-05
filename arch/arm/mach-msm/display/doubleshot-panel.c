/* linux/arch/arm/mach-msm/display/doubleshot-panel.c
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

#include "../devices.h"
#include "../board-doubleshot.h"
#include "../devices-msm8x60.h"
#include "../../../../drivers/video/msm_8x60/mdp_hw.h"

extern int panel_type;

static struct regulator *l1_3v;
static struct regulator *lvs1_1v8;
static struct regulator *l4_1v8;

static void doubleshot_id1_im1_switch(int on)
{
	int rc = 0;
	pr_info("panel id1 im1 switch %d\n", on);

	if (on) {
	if (system_rev >= 1) {
		rc = gpio_request(GPIO_LCM_ID1_IM1_XB, "LCM_ID1_IM1");
		if (rc < 0)
			pr_err("GPIO (%d) request fail\n", GPIO_LCM_ID1_IM1_XB);
		else if (panel_type == PANEL_ID_DOT_HITACHI)
				gpio_direction_output(GPIO_LCM_ID1_IM1_XB, 1);
			else
				gpio_direction_output(GPIO_LCM_ID1_IM1_XB, 0);
		gpio_free(GPIO_LCM_ID1_IM1_XB);
	} else {
		rc = gpio_request(GPIO_LCM_ID1_IM1, "LCM_ID1_IM1");
		if (rc < 0)
			pr_err("GPIO (%d) request fail\n", GPIO_LCM_ID1_IM1);
		else if (panel_type == PANEL_ID_DOT_HITACHI)
				gpio_direction_output(GPIO_LCM_ID1_IM1, 1);
			else
				gpio_direction_output(GPIO_LCM_ID1_IM1, 0);
		gpio_free(GPIO_LCM_ID1_IM1);
	}
	} else {
		if (system_rev >= 1) {
			rc = gpio_request(GPIO_LCM_ID1_IM1_XB, "LCM_ID1_IM1");
			if (rc < 0)
				pr_err("GPIO (%d) request fail\n", GPIO_LCM_ID1_IM1_XB);
			else
				gpio_direction_input(GPIO_LCM_ID1_IM1_XB);

			gpio_free(GPIO_LCM_ID1_IM1_XB);
		} else {
			rc = gpio_request(GPIO_LCM_ID1_IM1, "LCM_ID1_IM1");
			if (rc < 0)
				pr_err("GPIO (%d) request fail\n", GPIO_LCM_ID1_IM1);
			else
				gpio_direction_input(GPIO_LCM_ID1_IM1);
			gpio_free(GPIO_LCM_ID1_IM1);
		}
	}
	return;
}

/*
TODO:
1. move regulator initialization to doubleshot_panel_init()
*/
static void doubleshot_panel_power(int on)
{
	static int init;
	int ret;
	int rc;

	pr_info("%s(%d): init=%d\n", __func__, on, init);
	/* If panel is already on (or off), do nothing. */
	if (!init) {
		l1_3v = regulator_get(NULL, "8901_l1");
		if (IS_ERR(l1_3v)) {
			pr_err("%s: unable to get 8901_l1\n", __func__);
			goto fail;
		}

		if (system_rev >= 1) {
			l4_1v8 = regulator_get(NULL, "8901_l4");
			if (IS_ERR(l4_1v8)) {
				pr_err("%s: unable to get 8901_l4\n", __func__);
				goto fail;
			}
		} else {
		lvs1_1v8 = regulator_get(NULL, "8901_lvs1");
		if (IS_ERR(lvs1_1v8)) {
			pr_err("%s: unable to get 8901_lvs1\n", __func__);
			goto fail;
		}
		}

		ret = regulator_set_voltage(l1_3v, 3000000, 3000000);
		if (ret) {
			pr_err("%s: error setting l1_3v voltage\n", __func__);
			goto fail;
		}

		if (system_rev >= 1) {
			ret = regulator_set_voltage(l4_1v8, 1800000, 1800000);
			if (ret) {
				pr_err("%s: error setting l4_1v8 voltage\n", __func__);
				goto fail;
			}
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

	if (!l1_3v || IS_ERR(l1_3v)) {
		pr_err("%s: l1_3v is not initialized\n", __func__);
		return;
	}

	if (system_rev >= 1) {
		if (!l4_1v8 || IS_ERR(l4_1v8)) {
			pr_err("%s: l4_1v8 is not initialized\n", __func__);
			return;
		}
	} else {
	if (!lvs1_1v8 || IS_ERR(lvs1_1v8)) {
		pr_err("%s: lvs1_1v8 is not initialized\n", __func__);
		return;
	}
	}

	if (on) {
		if (system_rev >= 1) {
			if (regulator_enable(l4_1v8)) {
				pr_err("%s: Unable to enable the regulator:"
					" l4_1v8\n", __func__);
				return;
			}
		} else {
		if (regulator_enable(lvs1_1v8)) {
			pr_err("%s: Unable to enable the regulator:"
					" lvs1_1v8\n", __func__);
			return;
		}
		}
		mdelay(1);
		if (regulator_enable(l1_3v)) {
			pr_err("%s: Unable to enable the regulator:"
					" l1_3v\n", __func__);
			return;
		}
		/* skip reset for the first time panel power up */
		if ( init == 1 ) {
				init = 2;
				return;
		} else {
			if (panel_type == PANEL_ID_DOT_HITACHI) {
				mdelay(1);
				gpio_set_value(GPIO_LCM_RST_N, 1);
				mdelay(1);
				doubleshot_id1_im1_switch(on);
			} else if (panel_type == PANEL_ID_DOT_SONY ||
				panel_type == PANEL_ID_DOT_SONY_C3) {
				mdelay(1);
				doubleshot_id1_im1_switch(on);
				mdelay(1);
				gpio_set_value(GPIO_LCM_RST_N, 1);
				mdelay(10);
				gpio_set_value(GPIO_LCM_RST_N, 0);
				mdelay(10);
				gpio_set_value(GPIO_LCM_RST_N, 1);
				mdelay(10);
			} else {
				pr_err("panel_type=0x%x not support\n", panel_type);
				return;
			}
		}
	} else {
		gpio_set_value(GPIO_LCM_RST_N, 0);
		doubleshot_id1_im1_switch(on);
		mdelay(120);
		if (regulator_disable(l1_3v)) {
			pr_err("%s: Unable to enable the regulator:"
					" l1_3v\n", __func__);
			return;
		}
		if (system_rev >= 1) {
			if (regulator_disable(l4_1v8)) {
				pr_err("%s: Unable to enable the regulator:"
					" l4_1v8\n", __func__);
				return;
			}
		} else {
		if (regulator_disable(lvs1_1v8)) {
			pr_err("%s: Unable to enable the regulator:"
					" lvs1_1v8\n", __func__);
			return;
		}
			}
	}
	return;

fail:
	if (l1_3v)
		regulator_put(l1_3v);
	if (lvs1_1v8)
		regulator_put(lvs1_1v8);
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

#ifdef CONFIG_FB_MSM_TVOUT
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors atv_bus_init_vectors[] = {
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
static struct msm_bus_vectors atv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 236390400,
		.ib = 265939200,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 236390400,
		.ib = 265939200,
	},
};
static struct msm_bus_paths atv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(atv_bus_init_vectors),
		atv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(atv_bus_def_vectors),
		atv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata atv_bus_scale_pdata = {
	atv_bus_scale_usecases,
	ARRAY_SIZE(atv_bus_scale_usecases),
	.name = "atv",
};
#endif
#endif

static int mipi_panel_power(int on)
{
	int flag_on = !!on;
	static int mipi_power_save_on;

	if (mipi_power_save_on == flag_on)
		return 0;

	mipi_power_save_on = flag_on;

	doubleshot_panel_power(on);

	return 0;
}

#ifdef CONFIG_FB_MSM_TVOUT
static struct regulator *reg_8058_l13;

static int atv_dac_power(int on)
{
	int rc = 0;
	pr_info("%s: on/off=%d\n", __func__, on);

	#define _GET_REGULATOR(var, name) do {				\
		var = regulator_get(NULL, name);			\
		if (IS_ERR(var)) {					\
			pr_err("'%s' regulator not found, rc=%ld\n",	\
				name, IS_ERR(var));			\
			var = NULL;					\
			return -ENODEV;					\
		}							\
	} while (0)

	if (!reg_8058_l13)
		_GET_REGULATOR(reg_8058_l13, "8058_l13");
	#undef _GET_REGULATOR

	if (on) {
		//gpio_set_value(PM8058_GPIO_PM_TO_SYS(DOUBLESHOT_TVOUT_SW - 1), 1);
		rc = regulator_set_voltage(reg_8058_l13, 2050000, 2050000);
		if (rc) {
			pr_err("%s: '%s' regulator set voltage failed,\
				rc=%d\n", __func__, "8058_l13", rc);
			goto end;
		}

		rc = regulator_enable(reg_8058_l13);
		if (rc) {
			pr_err("%s: '%s' regulator enable failed,\
				rc=%d\n", __func__, "8058_l13", rc);
			goto end;
		}
	} else {
		//gpio_set_value(PM8058_GPIO_PM_TO_SYS(DOUBLESHOT_TVOUT_SW - 1), 0);
		rc = regulator_force_disable(reg_8058_l13);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "8058_l13", rc);
	}
end:
	return rc;
}
#endif

static struct mipi_dsi_platform_data mipi_pdata = {
	.vsync_gpio = 28,
	.dsi_power_save   = mipi_panel_power,
};

#ifdef CONFIG_FB_MSM_TVOUT
static struct tvenc_platform_data atv_pdata = {
	.poll		 = 0,
	.pm_vid_en	 = atv_dac_power,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &atv_bus_scale_pdata,
#endif
};
#endif

#define GPIO_BACKLIGHT_PWM0 0
#define GPIO_BACKLIGHT_PWM1 1

#define BRI_SETTING_MIN                 30
#define BRI_SETTING_DEF                 143
#define BRI_SETTING_MAX                 255

#define PWM_MIN              		9	/* 3.5% of max pwm */
#define PWM_DEFAULT			140	/* 55% of max pwm  */
#define PWM_MAX				255	/* 100% of max pwm */

static unsigned char doubleshot_shrink_pwm(int val)
{
	unsigned char shrink_br;
	if (val <= 0) {
		shrink_br = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN))
		shrink_br = PWM_MIN;
	else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF))
		shrink_br = (PWM_DEFAULT - PWM_MIN) * (val - BRI_SETTING_MIN) /
		(BRI_SETTING_DEF - BRI_SETTING_MIN) + PWM_MIN;
	else if ((val > BRI_SETTING_DEF) && (val <= BRI_SETTING_MAX))
		shrink_br = (PWM_MAX - PWM_DEFAULT) * (val - BRI_SETTING_DEF) /
		(BRI_SETTING_MAX - BRI_SETTING_DEF) + PWM_DEFAULT;
	else
		shrink_br = PWM_MAX;

	pr_debug("%s: brightness orig=%d, transformed=%d\n", __func__, val, shrink_br);
	return shrink_br;
}

static struct msm_panel_common_pdata mipi_panel_data = {
	.shrink_pwm = doubleshot_shrink_pwm,
};

static struct platform_device mipi_dsi_cmd_wvga_panel_device = {
	.name = "mipi_novatek",
	.id = 0,
	.dev = {
		.platform_data = &mipi_panel_data,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (panel_type == PANEL_ID_DOT_HITACHI) {
		if (!strcmp(name, "mipi_cmd_renesas_wvga"))
		return 0;
	} else if (panel_type == PANEL_ID_DOT_SONY ||
		panel_type == PANEL_ID_DOT_SONY_C3) {
		if (!strcmp(name, "mipi_cmd_novatek_wvga"))
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
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
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
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvenc", &atv_pdata);
	msm_fb_register_device("tvout_device", NULL);
#endif
}

/*
TODO:
1.find a better way to handle msm_fb_resources, to avoid passing it across file.
2.error handling
 */
int __init dot_init_panel(struct resource *res, size_t size)
{
	int ret;

	pr_info("%s: res=%p, size=%d\n", __func__, res, size);
	mipi_panel_data.shrink_pwm = doubleshot_shrink_pwm;
	if (panel_type == PANEL_ID_DOT_HITACHI)
		mipi_dsi_cmd_wvga_panel_device.name = "mipi_renesas";
	pr_info("%s: %s\n", __func__, mipi_dsi_cmd_wvga_panel_device.name);

	msm_fb_device.resource = res;
	msm_fb_device.num_resources = size;

	ret = platform_device_register(&msm_fb_device);
	ret = platform_device_register(&mipi_dsi_cmd_wvga_panel_device);

	msm_fb_add_devices();

	return 0;
}
