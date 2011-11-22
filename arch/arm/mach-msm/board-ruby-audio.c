/* linux/arch/arm/mach-msm/board-ruby-audio.c
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
#include <linux/spi/spi_aic3254.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include <mach/tpa2051d3.h>
#include <mach/qdsp6v2/snddev_icodec.h>
#include <mach/qdsp6v2/snddev_ecodec.h>
#include <mach/qdsp6v2/snddev_hdmi.h>
#include <mach/htc_acoustic_8x60.h>

#include "board-ruby.h"
#include "board-ruby-audio-data.h"
#include <mach/qdsp6v2/audio_dev_ctl.h>

static struct mutex bt_sco_lock;
static struct mutex mic_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

/* function prototype */
void ruby_snddev_bmic_pamp_on(int);

static uint32_t msm_codec_reset_gpio[] = {
	/* AIC3254 Reset */
	GPIO_CFG(RUBY_AUD_CODEC_RST, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	/* Timpani Reset */
	GPIO_CFG(RUBY_AUD_QTR_RESET, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
};

void ruby_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 1);
		set_speaker_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
		mdelay(5);
	} else {
		msleep(60);
		set_speaker_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
		mdelay(5);
	}
}

void ruby_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 1);
		set_headset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
	} else {
		msleep(60);
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void ruby_snddev_hs_spk_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 1);
		set_speaker_headset_amp(1);
		if (!atomic_read(&aic3254_ctl)) {
			curr_rx_mode |= BIT_SPEAKER;
			curr_rx_mode |= BIT_HEADSET;
		}
	} else {
		set_speaker_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl)) {
			curr_rx_mode &= ~BIT_SPEAKER;
			curr_rx_mode &= ~BIT_HEADSET;
		}
	}
}

void ruby_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
}

void ruby_snddev_bt_sco_pamp_on(int en)
{
	/* to be implemented */
}

/* power on/off externnal mic bias */
static struct regulator *vreg_l2;
void ruby_mic_enable(int en, int shift)
{
	int rc = 0;
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);

	mutex_lock(&mic_lock);
	if (!vreg_l2) {
		vreg_l2 = regulator_get(NULL, "8058_l2");

		if (IS_ERR(vreg_l2)) {
			pr_aud_err("%s: vreg_get failed (%ld)\n",
					__func__, PTR_ERR(vreg_l2));
			mutex_unlock(&mic_lock);
			return;
		}
	}

	if (en) {
		rc = regulator_set_voltage(vreg_l2, 2600000, 2600000);
		if (rc) {
			pr_aud_err("%s: unable to set 8058_s4 voltage"
					" to 2.6 V\n", __func__);
			goto vreg_fail;
		}

		rc = regulator_enable(vreg_l2);
		if (rc) {
			pr_aud_err("%s: failed on vreg_enabled %d\n",
					__func__, rc);
			goto vreg_fail;
		}

	} else {
		rc = regulator_disable(vreg_l2);
		if (rc) {
			pr_aud_err("%s: failed on vreg_disable %d\n",
					__func__, rc);
			goto vreg_fail;
		}

	}

	mutex_unlock(&mic_lock);
	return;

vreg_fail:
	regulator_put(vreg_l2);
	mutex_unlock(&mic_lock);
	vreg_l2 = NULL;
}

void ruby_snddev_imic_pamp_on(int en)
{
	int ret = 0;
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

	/* power up second mic at same time becuase don't support separate scenario */
	ruby_snddev_bmic_pamp_on(en);
}

void ruby_snddev_bmic_pamp_on(int en)
{
	int ret = 0;
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);

	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);
	}
}

void ruby_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		/* pull gpio high to do input mic selection */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_MIC_SEL), 1);
	} else {
		/* pull gpio down in default which input from back mic */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_MIC_SEL), 0);
	}
}

void ruby_snddev_stereo_mic_pamp_on(int en)
{
	int ret = 0;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);

	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Disabling int mic power failed\n", __func__);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Disabling back mic power failed\n", __func__);
	}
}

void ruby_snddev_fmspk_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 1);
		set_speaker_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
	} else {
		set_speaker_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void ruby_snddev_fmhs_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 1);
		set_headset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
	} else {
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

static struct regulator *snddev_reg_ncp;
void ruby_usb_headset_on (int en)
{
	int rc;
	pr_aud_info("%s\n", __func__);

	if (!snddev_reg_ncp) {
		snddev_reg_ncp = regulator_get(NULL, "8058_ncp");
		if (IS_ERR(snddev_reg_ncp)) {
			pr_aud_err("%s: regulator_get(%s) failed (%ld)\n",
				__func__, "ncp", PTR_ERR(snddev_reg_ncp));
			return;
		}
	}

	if (en) {
		rc = regulator_set_voltage(snddev_reg_ncp, 1800000, 1800000);
		if (rc < 0) {
			pr_aud_err("%s: regulator_set_voltage(ncp) failed"
					"(%d)\n", __func__, rc);
			goto vreg_fail;
		}

		rc = regulator_enable(snddev_reg_ncp);
		if (rc < 0) {
			pr_aud_err("%s: regulator_enable(ncp) failed (%d)\n",
				__func__, rc);
			goto vreg_fail;
		}
	} else {
		rc = regulator_disable(snddev_reg_ncp);
		if (rc < 0) {
			pr_aud_err("%s: regulator_disable(ncp) failed (%d)\n",
					__func__, rc);
			goto vreg_fail;
		}
	}

vreg_fail:
	regulator_put(snddev_reg_ncp);
	snddev_reg_ncp = NULL;
}

int ruby_get_rx_vol(uint8_t hw, int network, int level)
{
	int vol = 0;

	/* to be implemented */

	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);

	return vol;
}

void ruby_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			ruby_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			ruby_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			ruby_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			ruby_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			ruby_snddev_fmhs_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

int ruby_support_aic3254(void)
{
	return 1;
}

int ruby_support_adie(void)
{
	return 1;
}

int ruby_support_back_mic(void)
{
	return 1;
}

int ruby_support_audience(void)
{
	return 0;
}

int ruby_is_msm_i2s_slave(void)
{
	/* 1 - CPU slave, 0 - CPU master */
	return 1;
}

static uint32_t msm_spi_gpio_on[] = {
	GPIO_CFG(RUBY_SPI_DO,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(RUBY_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(RUBY_SPI_CS,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(RUBY_SPI_CLK, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};
static uint32_t msm_spi_gpio_off[] = {
	GPIO_CFG(RUBY_SPI_DO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(RUBY_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(RUBY_SPI_CS,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(RUBY_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
};

void ruby_spibus_enable(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_tlmm_config(msm_spi_gpio_on[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(msm_spi_gpio_on[1], GPIO_CFG_ENABLE);
		gpio_tlmm_config(msm_spi_gpio_on[2], GPIO_CFG_ENABLE);
		gpio_tlmm_config(msm_spi_gpio_on[3], GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(msm_spi_gpio_off[0], GPIO_CFG_DISABLE);
		gpio_tlmm_config(msm_spi_gpio_off[1], GPIO_CFG_DISABLE);
		gpio_tlmm_config(msm_spi_gpio_off[2], GPIO_CFG_DISABLE);
		gpio_tlmm_config(msm_spi_gpio_off[3], GPIO_CFG_DISABLE);
	}
	mdelay(1);
}


void ruby_get_acoustic_tables(struct acoustic_tables *tb)
{
	/* clean name buffer */
	memset(tb->aic3254, 0, PROPERTY_VALUE_MAX);
	memset(tb->adie, 0, PROPERTY_VALUE_MAX);
	memset(tb->spkamp, 0, PROPERTY_VALUE_MAX);
	memset(tb->acdb, 0, PROPERTY_VALUE_MAX);
	memset(tb->tpa2051, 0, PROPERTY_VALUE_MAX);
	memset(tb->tpa2026, 0, PROPERTY_VALUE_MAX);
	memset(tb->tpa2028, 0, PROPERTY_VALUE_MAX);


	/* HW version is after XD */
	if (system_rev > 3)
		strcpy(tb->aic3254, "AIC3254_REG_DualMic.csv");
	else
		strcpy(tb->aic3254, "AIC3254_REG_DualMic_XD.csv");

	if (system_rev > 2)
		strcpy(tb->tpa2051, "TPA2051_CFG.csv");
	else if (system_rev == 2)
		strcpy(tb->tpa2051, "TPA2051_CFG_XC.csv");
	else
		strcpy(tb->tpa2051, "TPA2051_CFG_XB.csv");
}

void ruby_reset_3254(void)
{
	gpio_tlmm_config(msm_codec_reset_gpio[0], GPIO_CFG_ENABLE);
	gpio_set_value(RUBY_AUD_CODEC_RST, 0);
	mdelay(1);
	gpio_set_value(RUBY_AUD_CODEC_RST, 1);
}

static struct q6v2audio_analog_ops ops = {
	.speaker_enable	        = ruby_snddev_poweramp_on,
	.headset_enable	        = ruby_snddev_hsed_pamp_on,
	.handset_enable	        = ruby_snddev_receiver_pamp_on,
	.headset_speaker_enable	= ruby_snddev_hs_spk_pamp_on,
	.bt_sco_enable	        = ruby_snddev_bt_sco_pamp_on,
	.int_mic_enable         = ruby_snddev_imic_pamp_on,
	.back_mic_enable        = ruby_snddev_bmic_pamp_on,
	.ext_mic_enable         = ruby_snddev_emic_pamp_on,
	.stereo_mic_enable      = ruby_snddev_stereo_mic_pamp_on,
	.fm_headset_enable      = ruby_snddev_fmhs_pamp_on,
	.fm_speaker_enable      = ruby_snddev_fmspk_pamp_on,
	.usb_headset_enable     = ruby_usb_headset_on,
};

static struct q6v2audio_icodec_ops iops = {
	.support_aic3254 = ruby_support_aic3254,
	.support_adie = ruby_support_adie,
	.is_msm_i2s_slave = ruby_is_msm_i2s_slave,
};

static struct q6v2audio_ecodec_ops eops = {
	.bt_sco_enable  = ruby_snddev_bt_sco_pamp_on,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable        = ruby_rx_amp_enable,
	.reset_3254           = ruby_reset_3254,
	.spibus_enable        = ruby_spibus_enable,
	.lb_dsp_init          = &LOOPBACK_DSP_INIT_PARAM,
	.lb_receiver_imic     = &LOOPBACK_Receiver_IMIC_PARAM,
	.lb_speaker_imic      = &LOOPBACK_Speaker_IMIC_PARAM,
	.lb_headset_emic      = &LOOPBACK_Headset_EMIC_PARAM,
	.lb_receiver_bmic     = &LOOPBACK_Receiver_BMIC_PARAM,
	.lb_speaker_bmic      = &LOOPBACK_Speaker_BMIC_PARAM,
	.lb_headset_bmic      = &LOOPBACK_Headset_BMIC_PARAM,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = ruby_mic_enable,
	.support_aic3254 = ruby_support_aic3254,
	.support_back_mic = ruby_support_back_mic,
	.support_audience = ruby_support_audience,
	.get_acoustic_tables = ruby_get_acoustic_tables
};

void ruby_aic3254_set_mode(int config, int mode)
{
	aic3254_set_mode(config, mode);
}


static struct q6v2audio_aic3254_ops aops = {
       .aic3254_set_mode = ruby_aic3254_set_mode,
};

void __init ruby_audio_init(void)
{
	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);

#ifdef CONFIG_MSM8X60_AUDIO_LTE
	pr_aud_info("%s\n", __func__);
	htc_8x60_register_analog_ops(&ops);
	htc_8x60_register_ecodec_ops(&eops);
	htc_8x60_register_icodec_ops(&iops);
	acoustic_register_ops(&acoustic);
	htc_8x60_register_aic3254_ops(&aops);
	msm_set_voc_freq(8000, 8000);
#endif

	aic3254_register_ctl_ops(&cops);
	/* PMIC GPIO Init (See board-ruby.c) */
	/* Reset AIC3254 */
	ruby_reset_3254();

	/* Timpani Reset */
	gpio_tlmm_config(msm_codec_reset_gpio[1], GPIO_CFG_ENABLE);
	gpio_set_value(RUBY_AUD_QTR_RESET, 0);
	mdelay(1);
	gpio_set_value(RUBY_AUD_QTR_RESET, 1);

}
