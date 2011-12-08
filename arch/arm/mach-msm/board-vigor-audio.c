/* linux/arch/arm/mach-msm/board-vigor-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/delay.h>
#include <linux/pmic8058-othc.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include <mach/tpa2051d3.h>
#include <mach/qdsp6v3/snddev_icodec.h>
#include <mach/qdsp6v3/snddev_ecodec.h>
#include <mach/qdsp6v3/snddev_hdmi.h>
#include <mach/qdsp6v3/audio_dev_ctl.h>
#include <mach/htc_acoustic_8x60.h>

#include "board-vigor.h"

static struct mutex bt_sco_lock;
static struct mutex mic_lock;

static uint32_t msm_snddev_gpio[] = {
	GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t msm_spi_gpio[] = {
	GPIO_CFG(VIGOR_SPI_DO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_SPI_DI,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_SPI_CS,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VIGOR_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static struct regulator *snddev_reg_l11;

void vigor_mic_bias_on(int en)
{
	int rc;
	pr_aud_info("%s\n", __func__);

	if (en) {
		snddev_reg_l11 = regulator_get(NULL, "8058_l11");
		if (IS_ERR(snddev_reg_l11)) {
			pr_aud_err("%s: regulator_get(%s) failed (%ld)\n",
				__func__, "8058_l11", PTR_ERR(snddev_reg_l11));
			return;
		}

		rc = regulator_set_voltage(snddev_reg_l11, 2850000, 2850000);
		if (rc < 0)
			pr_aud_err("%s: regulator_set_voltage(8058_l11) failed (%d)\n",
				__func__, rc);

		rc = regulator_enable(snddev_reg_l11);
		if (rc < 0)
			pr_aud_err("%s: regulator_enable(8058_l11) failed (%d)\n",
				__func__, rc);
	} else {

		if (!snddev_reg_l11)
			return;

		rc = regulator_disable(snddev_reg_l11);
		if (rc < 0)
			pr_aud_err("%s: regulator_disable(8058_l11) failed (%d)\n",
					__func__, rc);
		regulator_put(snddev_reg_l11);

		snddev_reg_l11 = NULL;
	}
}

void vigor_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_AMP_EN), 1);
		set_speaker_amp(1);
	} else {
		set_speaker_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_AMP_EN), 0);
	}
}

void vigor_snddev_usb_headset_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_AMP_EN), 1);
		set_headset_amp(1);
	} else {
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_AMP_EN), 0);
	}
}

void vigor_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_AMP_EN), 1);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_AMP_EN), 0);
	}
}

void vigor_snddev_hs_spk_pamp_on(int en)
{
	vigor_snddev_poweramp_on(en);
	vigor_snddev_hsed_pamp_on(en);
}

void vigor_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_REC_EN), 1);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_REC_EN), 0);
	}
}

void vigor_snddev_bt_sco_pamp_on(int en)
{
	/* to be implemented */
}

/* power on/off externnal mic bias */
void vigor_mic_enable(int en, int shift)
{
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);

	mutex_lock(&mic_lock);

	if (en)
		vigor_mic_bias_on(en);
	else
		vigor_mic_bias_on(en);

	mutex_unlock(&mic_lock);
}

void vigor_snddev_imic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

	}
}

void vigor_snddev_bmic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		/* select external mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_MIC_SEL), 0);

	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		/* select external mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_MIC_SEL), 0);

	}
}

void vigor_snddev_stereo_mic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		vigor_snddev_imic_pamp_on(en);
		vigor_snddev_bmic_pamp_on(en);
	} else {
		vigor_snddev_imic_pamp_on(en);
		vigor_snddev_bmic_pamp_on(en);
	}
}

void vigor_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		/* select external mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_MIC_SEL), 1);

	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_AUD_MIC_SEL), 0);

	}
}

void vigor_snddev_fmspk_pamp_on(int en)
{
	vigor_snddev_poweramp_on(en);
}

void vigor_snddev_fmhs_pamp_on(int en)
{
	vigor_snddev_hsed_pamp_on(en);
}

static struct regulator *snddev_reg_ncp;

void vigor_voltage_on (int en)
{
	int rc;
	pr_aud_info("%s\n", __func__);

	if (en) {
		snddev_reg_ncp = regulator_get(NULL, "8058_ncp");
		if (IS_ERR(snddev_reg_ncp)) {
			pr_aud_err("%s: regulator_get(%s) failed (%ld)\n", __func__,
				"ncp", PTR_ERR(snddev_reg_ncp));
			return;
		}

		rc = regulator_set_voltage(snddev_reg_ncp, 1800000, 1800000);
		if (rc < 0)
			pr_aud_err("%s: regulator_set_voltage(ncp) failed (%d)\n",
				__func__, rc);

		rc = regulator_enable(snddev_reg_ncp);
		if (rc < 0)
			pr_aud_err("%s: regulator_enable(ncp) failed (%d)\n",
				__func__, rc);
	} else {

		if (!snddev_reg_ncp)
			return;

		rc = regulator_disable(snddev_reg_ncp);
		if (rc < 0)
			pr_aud_err("%s: regulator_disable(ncp) failed (%d)\n",
					__func__, rc);
		regulator_put(snddev_reg_ncp);

		snddev_reg_ncp = NULL;
	}
}

int vigor_get_rx_vol(uint8_t hw, int network, int level)
{
	int vol = 0;

	/* to be implemented */

	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);

	return vol;
}

int vigor_get_speaker_channels(void)
{
	/* 1 - Mono, 2 - Stereo */
	return 1;
}

int vigor_is_msm_i2s_slave(void)
{
	/* 1 - CPU slave, 0 - CPU master */
	return 0;
}

int vigor_support_aic3254(void)
{
	return 0;
}

int vigor_support_adie(void)
{
	return 1;
}

int vigor_support_back_mic(void)
{
	return 0;
}

int vigor_is_msm_i2s_master(void)
{
	/* 0 - CPU slave, 1 - CPU master */
	return 1;
}

int vigor_support_opendsp(void)
{

	return 1;
}

#ifdef CONFIG_MSM8X60_AUDIO_LTE
static struct q6v2audio_analog_ops ops = {
	.speaker_enable	        = vigor_snddev_poweramp_on,
	.headset_enable	        = vigor_snddev_hsed_pamp_on,
	.handset_enable	        = vigor_snddev_receiver_pamp_on,
	.headset_speaker_enable	= vigor_snddev_hs_spk_pamp_on,
	.bt_sco_enable	        = vigor_snddev_bt_sco_pamp_on,
	.int_mic_enable         = vigor_snddev_imic_pamp_on,
	.back_mic_enable        = vigor_snddev_bmic_pamp_on,
	.ext_mic_enable         = vigor_snddev_emic_pamp_on,
	.fm_headset_enable      = vigor_snddev_fmhs_pamp_on,
	.fm_speaker_enable      = vigor_snddev_fmspk_pamp_on,
	.stereo_mic_enable      = vigor_snddev_stereo_mic_pamp_on,
	.usb_headset_enable     = vigor_snddev_usb_headset_pamp_on,
	.voltage_on             = vigor_voltage_on,
};

static struct q6v2audio_icodec_ops iops = {
	.support_aic3254 = vigor_support_aic3254,
	.is_msm_i2s_slave = vigor_is_msm_i2s_slave,
	.support_adie = vigor_support_adie,
};

static struct q6v2audio_ecodec_ops eops = {
	.bt_sco_enable  = vigor_snddev_bt_sco_pamp_on,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = vigor_mic_enable,
	.support_aic3254 = vigor_support_aic3254,
	.support_adie = vigor_support_adie,
	.support_back_mic = vigor_support_back_mic,
	.get_speaker_channels = vigor_get_speaker_channels,
};

static struct dev_ctrl_ops dops = {
	.support_opendsp = vigor_support_opendsp,
};

#endif

void __init vigor_audio_init(void)
{
	int i = 0;
	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);

#ifdef CONFIG_MSM8X60_AUDIO_LTE
	pr_aud_info("%s\n", __func__);
	htc_8x60_register_analog_ops(&ops);
	htc_8x60_register_ecodec_ops(&eops);
	htc_8x60_register_icodec_ops(&iops);
	htc_8x60_register_dev_ctrl_ops(&dops);
	acoustic_register_ops(&acoustic);
#endif

	/* PMIC GPIO Init (See board-vigor.c) */
	for (i = 0 ; i < ARRAY_SIZE(msm_snddev_gpio); i++)
		gpio_tlmm_config(msm_snddev_gpio[i], GPIO_CFG_DISABLE);

	gpio_tlmm_config(msm_spi_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_gpio[3], GPIO_CFG_ENABLE);

}
