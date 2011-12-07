/* linux/arch/arm/mach-msm/board-holiday-audio.c
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
#include <mach/qdsp6v3/snddev_icodec.h>
#include <mach/qdsp6v3/snddev_ecodec.h>
#include <mach/qdsp6v3/snddev_hdmi.h>
#include <mach/htc_acoustic_8x60.h>

#include "board-holiday.h"
#include "board-holiday-audio-data.h"
#include <mach/qdsp6v3/audio_dev_ctl.h>
#include <linux/a1026.h>

static struct mutex bt_sco_lock;
static struct mutex mic_lock;
static int curr_rx_mode;
static int support_audience;
extern unsigned skuid;

#define SKU_ATT1 0x2A800 /*AT&T 1*/
#define SKU_ATT2 0x2D002 /*AT&T 2*/
#define SKU_ATT3 0x2D004 /*AT&T 3*/
#define SKU_TELSTRA 0x2D001 /*Telstra*/
#define SKU_SKT 0x2D000 /*SKT*/

static int force_a1026_on = 0;/*Use for MFG test a1026 recording.*/

static atomic_t aic3254_ctl = ATOMIC_INIT(0);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

void holiday_snddev_bmic_pamp_on(int en);

static uint32_t msm_aic3254_reset_gpio[] = {
	GPIO_CFG(HOLIDAY_AUD_CODEC_RST, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
};

static uint32_t msm_snddev_gpio[] = {
	GPIO_CFG(108, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(109, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t msm_a1026_gpio[] = {
	GPIO_CFG(HOLIDAY_GPIO_AUD_A1026_INT, 0, GPIO_CFG_INPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(HOLIDAY_GPIO_AUD_A1026_WAKEUP, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
};

static uint32_t msm_a1026_na_gpio[] = {
	GPIO_CFG(HOLIDAY_GPIO_AUD_A1026_INT, 0, GPIO_CFG_INPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(HOLIDAY_GPIO_AUD_A1026_WAKEUP, 0, GPIO_CFG_INPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
};

void holiday_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 1);
		msleep(50);
		set_speaker_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
	} else {
		set_speaker_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void holiday_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 1);
		msleep(50);
		set_headset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
	} else {
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void holiday_snddev_hs_spk_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 1);
		msleep(50);
		set_speaker_headset_amp(1);
		if (!atomic_read(&aic3254_ctl)) {
			curr_rx_mode |= BIT_SPEAKER;
			curr_rx_mode |= BIT_HEADSET;
		}
	} else {
		set_speaker_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl)) {
			curr_rx_mode &= ~BIT_SPEAKER;
			curr_rx_mode &= ~BIT_HEADSET;
		}
	}
}

void holiday_snddev_receiver_pamp_on(int en)
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

void holiday_snddev_bt_sco_pamp_on(int en)
{
	/* to be implemented */
}

/* rules for mic selection.
			phonecall A1026	recordind 3254
			SEL1	SEL2	SEL1	SEL2
INT MIC recording		x	x	0	1
2ND MIC recording		x	x	0	1
Headset MIC recording	x	x	1	1
stereo Recording		x	x	0	1
INT MIC phonecall		0	0	x	x
Headset MIC phonecall	1	0	x	x

call state = 1 <-- start voice
              = 0 <-- end voice
*/

/* power on/off externnal mic bias */
void holiday_mic_enable(int en, int shift)
{
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);

	mutex_lock(&mic_lock);

	if (en)
		pm8058_micbias_enable(OTHC_MICBIAS_2, OTHC_SIGNAL_ALWAYS_ON);
	else
		pm8058_micbias_enable(OTHC_MICBIAS_2, OTHC_SIGNAL_OFF);

	mutex_unlock(&mic_lock);
}

void holiday_imic_pamp_on_with_audience(int en)
{
	int ret, call_state=0;
	pr_aud_info("%s %d\n", __func__, en);

	call_state = msm_get_call_state();
	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		/* select internal mic path */
		if (call_state) {
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 0);
			ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
			if (ret)
				pr_aud_err("%s: Enabling back mic power failed\n", __func__);
		} else {
			if (!force_a1026_on)
				gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 1);
		}
	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);
		if (call_state) {
			ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
			if (ret)
				pr_aud_err("%s: Enabling back mic power failed\n", __func__);
		}
	}
}

void holiday_imic_pamp_on_without_audience(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	holiday_snddev_bmic_pamp_on(en);
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

void holiday_snddev_imic_pamp_on(int en)
{
	if (support_audience)
		holiday_imic_pamp_on_with_audience(en);
	else
		holiday_imic_pamp_on_without_audience(en);
}

void holiday_snddev_bmic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s (support_audience, en)=(%d, %d)\n",
		__func__, support_audience, en);
	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);

		/* select internal mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL1), 0);
		if (support_audience && !force_a1026_on)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 1);
	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);

	}
}

void holiday_snddev_emic_pamp_on(int en)
{
	int call_state = 0;

	pr_aud_info("%s (support_audience, en)=(%d, %d)\n",
		__func__, support_audience, en);

	call_state = msm_get_call_state();
	/*
	external micbias should be controlled by headset driver with HOLIDAY_mic_enable
	turn on with headset plugged in and turn off when headset unplugged.
	*/
	if (en) {
		/* select internal mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL1), 1);
		if (support_audience) {
			if (call_state)
				gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 0);
			else
				gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 1);
		}
	}
}

void holiday_snddev_stereo_mic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);

		/* select internal mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL1), 0);
		if (support_audience)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 1);
	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Disabling int mic power failed\n", __func__);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Disabling back mic power failed\n", __func__);
	}
}

void holiday_snddev_fmspk_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 1);
		set_speaker_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
	} else {
		set_speaker_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void holiday_snddev_fmhs_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 1);
		set_headset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
	} else {
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_HANDSET_ENO), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

int holiday_get_rx_vol(uint8_t hw, int network, int level)
{
	int vol = 0;

	/* to be implemented */

	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);

	return vol;
}

void holiday_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			holiday_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			holiday_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			holiday_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			holiday_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			holiday_snddev_fmhs_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}


int holiday_support_aic3254(void)
{
	return 1;
}

int holiday_support_back_mic(void)
{
	return 1;
}

int holiday_support_audience(void)
{
	return support_audience;
}

void holiday_get_acoustic_tables(struct acoustic_tables *tb)
{
	if (support_audience)
		strcpy(tb->aic3254, "AIC3254_REG_DualMic_WA.txt");
	else
		strcpy(tb->aic3254, "AIC3254_REG_DualMic.txt");
}


int holiday_is_msm_i2s_slave(void)
{
	/* 1 - CPU slave, 0 - CPU master */
	return 1;
}

void holiday_spibus_enable(int en)
{
	uint32_t msm_spi_gpio_on[] = {
		GPIO_CFG(HOLIDAY_SPI_DO,  1, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		GPIO_CFG(HOLIDAY_SPI_DI,  1, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		GPIO_CFG(HOLIDAY_SPI_CS,  1, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		GPIO_CFG(HOLIDAY_SPI_CLK, 1, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	};

	uint32_t msm_spi_gpio_off[] = {
		GPIO_CFG(HOLIDAY_SPI_DO,  0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
		GPIO_CFG(HOLIDAY_SPI_DI,  0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
		GPIO_CFG(HOLIDAY_SPI_CS,  0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
		GPIO_CFG(HOLIDAY_SPI_CLK, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	};
	pr_debug("%s %d\n", __func__, en);
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

void holiday_a1026_hw_reset(void)
{
	/* Reset A1026 chip */
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_A1026_RST), 0);

	/* Enable A1026 clock */
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_A1026_CLK), 1);
	mdelay(1);

	/* Take out of reset */
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_A1026_RST), 1);
}

/*
	Design for MFG testing originally.
	Force turn a1026 on for recording test.
*/
int holiday_set_mic_state(char miccase)
{
	int rc = 0;
	unsigned int cmd_msg = 0;

	gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 0);
	force_a1026_on = 1;

	switch (miccase) {
	case 1: /* Mic-1 ON / Mic-2 OFF */
		cmd_msg = 0x80260007;
		break;
	case 2: /* Mic-1 OFF / Mic-2 ON */
		cmd_msg = 0x80260015;
		break;
	case 3: /* both ON */
		cmd_msg = 0x80260001;
		break;
	case 4: /* both OFF */
		cmd_msg = 0x80260006;
		break;
	default:
		pr_aud_info("%s: invalid input %d\n", __func__, miccase);
		rc = -EINVAL;
		break;
	}
	rc = execute_cmdmsg(cmd_msg);
	return rc;
}

void holiday_selmic(int en)
{
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL2), 0);
	force_a1026_on = 1;
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(HOLIDAY_AUD_MIC_SEL1), en);
	}
}

int holiday_a1026_support_bt(void)
{
	/*holiday BT did not go through audience chip.*/
	return 0;
}

void holiday_reset_3254(void)
{
	gpio_tlmm_config(msm_aic3254_reset_gpio[0], GPIO_CFG_ENABLE);
	gpio_set_value(HOLIDAY_AUD_CODEC_RST, 0);
	mdelay(1);
	gpio_set_value(HOLIDAY_AUD_CODEC_RST, 1);
}

static void __init audience_gpio_init(void)
{
	if (!support_audience) {
		pr_aud_info("Configure audio codec gpio for devices WITHOUT audience(A1026).\n");
		gpio_tlmm_config(msm_a1026_na_gpio[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(msm_a1026_na_gpio[1], GPIO_CFG_ENABLE);
	} else {
		pr_aud_info("Configure audio codec gpio for devices WITH audience(A1026).\n");
		gpio_tlmm_config(msm_a1026_gpio[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(msm_a1026_gpio[1], GPIO_CFG_ENABLE);
	}
}

static struct q6v2audio_analog_ops ops = {
	.speaker_enable	        = holiday_snddev_poweramp_on,
	.headset_enable	        = holiday_snddev_hsed_pamp_on,
	.handset_enable	        = holiday_snddev_receiver_pamp_on,
	.headset_speaker_enable	= holiday_snddev_hs_spk_pamp_on,
	.bt_sco_enable	        = holiday_snddev_bt_sco_pamp_on,
	.int_mic_enable         = holiday_snddev_imic_pamp_on,
	.back_mic_enable        = holiday_snddev_bmic_pamp_on,
	.ext_mic_enable         = holiday_snddev_emic_pamp_on,
	.stereo_mic_enable      = holiday_snddev_stereo_mic_pamp_on,
	.fm_headset_enable      = holiday_snddev_fmhs_pamp_on,
	.fm_speaker_enable      = holiday_snddev_fmspk_pamp_on,
};

static struct q6v2audio_icodec_ops iops = {
	.support_aic3254 = holiday_support_aic3254,
	.is_msm_i2s_slave = holiday_is_msm_i2s_slave,
};

static struct q6v2audio_ecodec_ops eops = {
	.bt_sco_enable  = holiday_snddev_bt_sco_pamp_on,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable        = holiday_rx_amp_enable,
	.spibus_enable        = holiday_spibus_enable,
	.reset_3254           = holiday_reset_3254,
	.lb_dsp_init          = &LOOPBACK_DSP_INIT_PARAM,
	.lb_receiver_imic     = &LOOPBACK_Receiver_IMIC_PARAM,
	.lb_speaker_imic      = &LOOPBACK_Speaker_IMIC_PARAM,
	.lb_headset_emic      = &LOOPBACK_Headset_EMIC_PARAM,
	.lb_receiver_bmic     = &LOOPBACK_Receiver_BMIC_PARAM,
	.lb_speaker_bmic      = &LOOPBACK_Speaker_BMIC_PARAM,
	.lb_headset_bmic      = &LOOPBACK_Headset_BMIC_PARAM,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = holiday_mic_enable,
	.support_aic3254 = holiday_support_aic3254,
	.support_back_mic = holiday_support_back_mic,
	.support_audience = holiday_support_audience,
	.get_acoustic_tables = holiday_get_acoustic_tables,
};

static struct audience_ctl_ops a1026ops = {
	.hw_reset = holiday_a1026_hw_reset,
	.set_mic_state = holiday_set_mic_state,
	.selmic = holiday_selmic,
	.support_bt = holiday_a1026_support_bt,
};
void holiday_aic3254_set_mode(int config, int mode)
{
	aic3254_set_mode(config, mode);
}


static struct q6v2audio_aic3254_ops aops = {
       .aic3254_set_mode = holiday_aic3254_set_mode,
};

void __init holiday_audio_init(void)
{
	int i = 0;
	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);
	pr_aud_info("%s: 0x%x\n", __func__, skuid);
	switch (skuid) {
	case SKU_ATT1:
	case SKU_ATT2:
	case SKU_ATT3:
		support_audience = 1;
		break;
	default:
		support_audience = 0;
		break;
	}

#ifdef CONFIG_MSM8X60_AUDIO_LTE
	pr_aud_info("%s\n", __func__);
	htc_8x60_register_analog_ops(&ops);
	htc_8x60_register_ecodec_ops(&eops);
	htc_8x60_register_icodec_ops(&iops);
	acoustic_register_ops(&acoustic);
	htc_8x60_register_aic3254_ops(&aops);
	a1026_register_ctl_ops(&a1026ops);

	if (!support_audience)
		/*fix voice sample rate as 8KHz for 3254 dual mic.*/
		msm_set_voc_freq(8000, 8000);
#endif

	aic3254_register_ctl_ops(&cops);
	/* PMIC GPIO Init (See board-holiday.c) */
	/* Reset AIC3254 */
	holiday_reset_3254();
	for (i=0 ; i<sizeof(msm_snddev_gpio); i++)
		gpio_tlmm_config(msm_snddev_gpio[i], GPIO_CFG_DISABLE);

	/* Configure A1026 GPIOs */
	audience_gpio_init();
}
