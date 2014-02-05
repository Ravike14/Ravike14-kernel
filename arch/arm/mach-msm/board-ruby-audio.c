/* Copyright (C) 2010-2011 HTC Corporation.
 * Copyright (c) 2013 Ravishka Fernando <rn.fernando3@gmail.com>
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
#include <linux/spi/spi_aic3254.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include <mach/tpa2051d3.h>
#include "qdsp6v2/snddev_icodec.h"
#include "qdsp6v2/snddev_ecodec.h"
#include "qdsp6v2/snddev_hdmi.h"
#include <mach/qdsp6v2/audio_dev_ctl.h>
#include <sound/apr_audio.h>
#include <sound/q6asm.h>
#include <mach/htc_acoustic_8x60.h>
#include <mach/board_htc.h>

#include "board-ruby.h"

#define PM8058_GPIO_BASE					NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)

static struct mutex mic_lock;
static atomic_t q6_effect_mode = ATOMIC_INIT(-1);
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

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

static struct regulator *vreg_l5 = NULL;

void ruby_snddev_tx_pamp_on(int en)
{
	int rc = 0;
	int call_state = 0;

	call_state = msm_get_call_state(); 

	if (!call_state)
		return;

	pr_aud_info("%s %d\n", __func__, en);

	vreg_l5 = regulator_get(NULL, "8058_l5");

	if (IS_ERR(vreg_l5)) {
		pr_aud_err("%s: vreg_get failed (%ld)\n", __func__, PTR_ERR(vreg_l5));
		return;
	}

	rc = regulator_set_voltage(vreg_l5, 2850000, 2850000);
	if (rc) {
		pr_aud_err("%s: unable to set 8058_l5 voltage\n", __func__);
		goto vreg_fail;
	}

	rc = regulator_enable(vreg_l5);
	if (rc) {
		pr_aud_err("%s: unable to enable 8058_l5\n", __func__);
		goto vreg_fail;
	}

vreg_fail:
	regulator_put(vreg_l5);
	vreg_l5 = NULL;
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

	ruby_snddev_bmic_pamp_on(en);
	ruby_snddev_tx_pamp_on(en);
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
	ruby_snddev_tx_pamp_on(en);
}

void ruby_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_MIC_SEL), 1);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_MIC_SEL), 0);
	}
	ruby_snddev_tx_pamp_on(en);
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
	ruby_snddev_tx_pamp_on(en);
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
void ruby_usb_headset_on(int en)
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

int ruby_get_speaker_channels(void)
{
	return 1;
}

int ruby_support_beats(void)
{
	return 1;
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
	return 1;
}

void ruby_enable_beats(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);
}

void ruby_reset_3254(void)
{
	gpio_tlmm_config(msm_codec_reset_gpio[0], GPIO_CFG_ENABLE);
	gpio_set_value(RUBY_AUD_CODEC_RST, 0);
	mdelay(1);
	gpio_set_value(RUBY_AUD_CODEC_RST, 1);
}

void ruby_set_q6_effect_mode(int mode)
{
	pr_aud_info("%s: mode %d\n", __func__, mode);
	atomic_set(&q6_effect_mode, mode);
}

int ruby_get_q6_effect_mode(void)
{
	int mode = atomic_read(&q6_effect_mode);
	pr_aud_info("%s: mode %d\n", __func__, mode);
	return mode;
}

static struct q6v2audio_analog_ops ops = {
	.speaker_enable	        = ruby_snddev_poweramp_on,
	.headset_enable	        = ruby_snddev_hsed_pamp_on,
	.handset_enable	        = ruby_snddev_receiver_pamp_on,
	.headset_speaker_enable	= ruby_snddev_hs_spk_pamp_on,
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

static struct q6asm_ops qops = {
	.get_q6_effect = ruby_get_q6_effect_mode,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable        = ruby_rx_amp_enable,
	.reset_3254           = ruby_reset_3254,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = ruby_mic_enable,
	.support_adie = ruby_support_adie,
	.support_aic3254 = ruby_support_aic3254,
	.support_back_mic = ruby_support_back_mic,
	.get_speaker_channels = ruby_get_speaker_channels,
	.support_beats = ruby_support_beats,
	.enable_beats = ruby_enable_beats,
	.set_q6_effect = ruby_set_q6_effect_mode,
};

void ruby_aic3254_set_mode(int config, int mode)
{
	aic3254_set_mode(config, mode);
}


static struct q6v2audio_aic3254_ops aops = {
       .aic3254_set_mode = ruby_aic3254_set_mode,
};

void ruby_audio_gpios_init(void)
{
	pr_aud_info("%s\n", __func__);
	gpio_request(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_MIC_SEL), "AUD_MIC_SEL");
	gpio_request(PM8058_GPIO_PM_TO_SYS(RUBY_AUD_HANDSET_ENO), "AUD_HANDSET_ENO");
}

void __init ruby_audio_init(void)
{
	mutex_init(&mic_lock);
	
	pr_aud_info("%s\n", __func__);
	htc_8x60_register_analog_ops(&ops);
	htc_8x60_register_icodec_ops(&iops);
	htc_register_q6asm_ops(&qops);
	acoustic_register_ops(&acoustic);
	htc_8x60_register_aic3254_ops(&aops);
	msm_set_voc_freq(8000, 8000);	
	aic3254_register_ctl_ops(&cops);
	ruby_audio_gpios_init();
	ruby_reset_3254();
	
	gpio_tlmm_config(msm_codec_reset_gpio[1], GPIO_CFG_ENABLE);
	gpio_set_value(RUBY_AUD_QTR_RESET, 0);
	mdelay(1);
	gpio_set_value(RUBY_AUD_QTR_RESET, 1);
	
}
