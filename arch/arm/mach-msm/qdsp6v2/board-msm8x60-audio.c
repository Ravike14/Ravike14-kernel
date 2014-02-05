/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2013 Sebastian Sobczyk <sebastiansobczyk@wp.pl>
 * Modified  2013 Ravishka Fernando <rn.fernando3@gmail.com>
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
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>

#include <mach/qdsp6v2/audio_dev_ctl.h>
#include <sound/apr_audio.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/board-msm8660.h>

#include "snddev_icodec.h"
#include "snddev_ecodec.h"
#include "timpani_profile_8x60.h"
#include "timpani_profile_8x60_ruby.h"
#include "snddev_hdmi.h"
#include "snddev_mi2s.h"
#include <linux/spi/spi_aic3254.h>
#include "snddev_virtual.h"

#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)

static struct q6v2audio_analog_ops default_audio_ops;
static struct q6v2audio_analog_ops *audio_ops = &default_audio_ops;

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_hsed_config;
static void snddev_hsed_config_modify_setting(int type);
static void snddev_hsed_config_restore_setting(void);
#endif

#define SNDDEV_GPIO_CLASS_D0_EN 227

#define SNDDEV_GPIO_CLASS_D1_EN 229

#define SNDDEV_GPIO_MIC2_ANCR_SEL 294
#define SNDDEV_GPIO_MIC1_ANCL_SEL 295
#define SNDDEV_GPIO_HS_MIC4_SEL 296

int speaker_enable(void)
{
	if (audio_ops->speaker_enable)
		audio_ops->speaker_enable(1);
	return 0;
}

void speaker_disable(void)
{
	if (audio_ops->speaker_enable)
		audio_ops->speaker_enable(0);
}

int headset_enable(void)
{
	if (audio_ops->headset_enable)
		audio_ops->headset_enable(1);
	return 0;
}

void headset_disable(void)
{
	if (audio_ops->headset_enable)
		audio_ops->headset_enable(0);
}

int handset_enable(void)
{
	if (audio_ops->handset_enable)
		audio_ops->handset_enable(1);
	return 0;
}

void handset_disable(void)
{
	if (audio_ops->handset_enable)
		audio_ops->handset_enable(0);
}

int headset_speaker_enable(void)
{
	if (audio_ops->headset_speaker_enable)
		audio_ops->headset_speaker_enable(1);
	return 0;
}

void headset_speaker_disable(void)
{
	if (audio_ops->headset_speaker_enable)
		audio_ops->headset_speaker_enable(0);
}

int int_mic_enable(void)
{
	if (audio_ops->int_mic_enable)
		audio_ops->int_mic_enable(1);
	return 0;
}

void int_mic_disable(void)
{
	if (audio_ops->int_mic_enable)
		audio_ops->int_mic_enable(0);
}

int back_mic_enable(void)
{
	if (audio_ops->back_mic_enable)
		audio_ops->back_mic_enable(1);
	return 0;
}

void back_mic_disable(void)
{
	if (audio_ops->back_mic_enable)
		audio_ops->back_mic_enable(0);
}

int ext_mic_enable(void)
{
	if (audio_ops->ext_mic_enable)
		audio_ops->ext_mic_enable(1);
	return 0;
}

void ext_mic_disable(void)
{
	if (audio_ops->ext_mic_enable)
		audio_ops->ext_mic_enable(0);
}

int stereo_mic_enable(void)
{
	if (audio_ops->stereo_mic_enable)
		audio_ops->stereo_mic_enable(1);
	return 0;
}

void stereo_mic_disable(void)
{
	if (audio_ops->stereo_mic_enable)
		audio_ops->stereo_mic_enable(0);
}

int usb_headset_enable(void)
{
	if (audio_ops->usb_headset_enable)
		audio_ops->usb_headset_enable(1);
	return 0;
}

void usb_headset_disable(void)
{
	if (audio_ops->usb_headset_enable)
		audio_ops->usb_headset_enable(0);
}

int fm_headset_enable(void)
{
	if (audio_ops->fm_headset_enable)
		audio_ops->fm_headset_enable(1);
	return 0;
}

void fm_headset_disable(void)
{
	if (audio_ops->fm_headset_enable)
		audio_ops->fm_headset_enable(0);
}

int fm_speaker_enable(void)
{
	if (audio_ops->fm_speaker_enable)
		audio_ops->fm_speaker_enable(1);
	return 0;
}

void fm_speaker_disable(void)
{
	if (audio_ops->fm_speaker_enable)
		audio_ops->fm_speaker_enable(0);
}

int voltage_on(void)
{
	if (audio_ops->voltage_on)
		audio_ops->voltage_on(1);
	return 0;
}

void voltage_off(void)
{
	if (audio_ops->voltage_on)
		audio_ops->voltage_on(0);
}

static struct regulator *s3;
static struct regulator *mvs;

static int msm_snddev_enable_dmic_power(void)
{
	int ret;

	pr_aud_err("%s", __func__);
	s3 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(s3)) {
		ret = -EBUSY;
		goto fail_get_s3;
	}

	ret = regulator_set_voltage(s3, 1800000, 1800000);
	if (ret) {
		pr_err("%s: error setting voltage\n", __func__);
		goto fail_s3;
	}

	ret = regulator_enable(s3);
	if (ret) {
		pr_err("%s: error enabling regulator\n", __func__);
		goto fail_s3;
	}

	mvs = regulator_get(NULL, "8901_mvs0");
	if (IS_ERR(mvs))
		goto fail_mvs0_get;

	ret = regulator_enable(mvs);
	if (ret) {
		pr_err("%s: error setting regulator\n", __func__);
		goto fail_mvs0_enable;
	}
	stereo_mic_enable();
	return ret;

fail_mvs0_enable:
	regulator_put(mvs);
	mvs = NULL;
fail_mvs0_get:
	regulator_disable(s3);
fail_s3:
	regulator_put(s3);
	s3 = NULL;
fail_get_s3:
	return ret;
}

static void msm_snddev_disable_dmic_power(void)
{
	int ret;

	pr_aud_err("%s", __func__);
	if (mvs) {
		ret = regulator_disable(mvs);
		if (ret < 0)
			pr_err("%s: error disabling vreg mvs\n", __func__);
		regulator_put(mvs);
		mvs = NULL;
	}

	if (s3) {
		ret = regulator_disable(s3);
		if (ret < 0)
			pr_err("%s: error disabling regulator s3\n", __func__);
		regulator_put(s3);
		s3 = NULL;
	}
	stereo_mic_disable();
}

static struct regulator *snddev_reg_ncp;
static struct regulator *snddev_reg_l10;

static atomic_t preg_ref_cnt;

static int msm_snddev_voltage_on(void)
{
	int rc;
	pr_debug("%s\n", __func__);

	if (atomic_inc_return(&preg_ref_cnt) > 1)
		return 0;

	snddev_reg_l10 = regulator_get(NULL, "8058_l10");
	if (IS_ERR(snddev_reg_l10)) {
		pr_err("%s: regulator_get(%s) failed (%ld)\n", __func__,
			"l10", PTR_ERR(snddev_reg_l10));
		return -EBUSY;
	}

	rc = regulator_set_voltage(snddev_reg_l10, 2600000, 2600000);
	if (rc < 0)
		pr_err("%s: regulator_set_voltage(l10) failed (%d)\n",
			__func__, rc);

	rc = regulator_enable(snddev_reg_l10);
	if (rc < 0)
		pr_err("%s: regulator_enable(l10) failed (%d)\n", __func__, rc);

	snddev_reg_ncp = regulator_get(NULL, "8058_ncp");
	if (IS_ERR(snddev_reg_ncp)) {
		pr_err("%s: regulator_get(%s) failed (%ld)\n", __func__,
			"ncp", PTR_ERR(snddev_reg_ncp));
		return -EBUSY;
	}

	rc = regulator_set_voltage(snddev_reg_ncp, 1800000, 1800000);
	if (rc < 0) {
		pr_err("%s: regulator_set_voltage(ncp) failed (%d)\n",
			__func__, rc);
		goto regulator_fail;
	}

	rc = regulator_enable(snddev_reg_ncp);
	if (rc < 0) {
		pr_err("%s: regulator_enable(ncp) failed (%d)\n", __func__, rc);
		goto regulator_fail;
	}

	return rc;

regulator_fail:
	regulator_put(snddev_reg_ncp);
	snddev_reg_ncp = NULL;
	return rc;
}

static void msm_snddev_voltage_off(void)
{
	int rc;
	pr_debug("%s\n", __func__);

	if (!snddev_reg_ncp)
		goto done;

	if (atomic_dec_return(&preg_ref_cnt) == 0) {
		rc = regulator_disable(snddev_reg_ncp);
		if (rc < 0)
			pr_err("%s: regulator_disable(ncp) failed (%d)\n",
				__func__, rc);
		regulator_put(snddev_reg_ncp);

		snddev_reg_ncp = NULL;
	}

done:
	if (!snddev_reg_l10)
		return;

	rc = regulator_disable(snddev_reg_l10);
	if (rc < 0)
		pr_err("%s: regulator_disable(l10) failed (%d)\n",
			__func__, rc);

	regulator_put(snddev_reg_l10);

	snddev_reg_l10 = NULL;
}

static int msm_snddev_enable_dmic_sec_power(void)
{
	int ret;

	pr_aud_err("%s", __func__);
	ret = msm_snddev_enable_dmic_power();
	if (ret) {
		pr_err("%s: Error: Enabling dmic power failed\n", __func__);
		return ret;
	}
#ifdef CONFIG_PMIC8058_OTHC
	ret = pm8058_micbias_enable(OTHC_MICBIAS_2, OTHC_SIGNAL_ALWAYS_ON);
	if (ret) {
		pr_err("%s: Error: Enabling micbias failed\n", __func__);
		msm_snddev_disable_dmic_power();
		return ret;
	}
#endif
	return 0;
}

static void msm_snddev_disable_dmic_sec_power(void)
{
	pr_aud_err("%s", __func__);
	msm_snddev_disable_dmic_power();

#ifdef CONFIG_PMIC8058_OTHC
	pm8058_micbias_enable(OTHC_MICBIAS_2, OTHC_SIGNAL_OFF);
#endif
}

static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
	EAR_PRI_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = handset_enable,
	.pamp_off = handset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_RECEIVER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_RECEIVER,
	.default_aic3254_id = PLAYBACK_RECEIVER,
};

static struct platform_device msm_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

static struct adie_codec_action_unit iearpiece_hac_48KHz_osr256_actions[] =
	EAR_PRI_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_hac_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_hac_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_hac_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_hac_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_hac_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_hac_settings),
};

static struct snddev_icodec_data snddev_ihac_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "hac_rx",
	.copp_id = 0,
	.profile = &iearpiece_hac_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = handset_enable,
	.pamp_off = handset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_RECEIVER,
	.aic3254_voc_id = HAC,
	.default_aic3254_id = PLAYBACK_RECEIVER,
};

static struct platform_device msm_ihac_device = {
	.name = "snddev_icodec",
	.id = 38,
	.dev = { .platform_data = &snddev_ihac_data },
};

static struct adie_codec_action_unit imic_48KHz_osr256_actions[] =
#ifdef CONFIG_MACH_RUBY
	AMIC_PRI_MONO_48000_OSR_256;
#else
	AMIC_PRI_MONO_OSR_256;
#endif

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 1,
	.profile = &imic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.pamp_off = int_mic_disable,
	.aic3254_id = VOICERECOGNITION_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_RECEIVER,
	.default_aic3254_id = VOICERECOGNITION_IMIC,
};

static struct platform_device msm_imic_device = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

static struct snddev_icodec_data snddev_nomic_headset_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "nomic_headset_tx",
	.copp_id = 1,
	.profile = &imic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.pamp_off = int_mic_disable,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_IMIC,
};

static struct platform_device msm_nomic_headset_tx_device = {
	.name = "snddev_icodec",
	.id = 40,
	.dev = { .platform_data = &snddev_nomic_headset_data },
};
static struct adie_codec_action_unit headset_ab_cpls_48KHz_osr256_actions[] =
	HEADSET_AB_CPLS_48000_OSR_256;

static struct adie_codec_hwsetting_entry headset_ab_cpls_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_ab_cpls_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_ab_cpls_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile headset_ab_cpls_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_ab_cpls_settings,
	.setting_sz = ARRAY_SIZE(headset_ab_cpls_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.profile = &headset_ab_cpls_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = headset_enable,
	.pamp_off = headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_HEADSET,
	.aic3254_voc_id = CALL_DOWNLINK_EMIC_HEADSET,
	.default_aic3254_id = PLAYBACK_HEADSET,
};

static struct platform_device msm_headset_stereo_device = {
	.name = "snddev_icodec",
	.id = 34,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data },
};

static struct snddev_icodec_data snddev_nomic_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "nomic_headset_stereo_rx",
	.copp_id = 0,
	.profile = &headset_ab_cpls_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = headset_enable,
	.pamp_off = headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_HEADSET,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_HEADSET,
	.default_aic3254_id = PLAYBACK_HEADSET,
};

static struct platform_device msm_nomic_headset_stereo_device = {
	.name = "snddev_icodec",
	.id = 39,
	.dev = { .platform_data = &snddev_nomic_ihs_stereo_rx_data },
};

static struct adie_codec_action_unit headset_anc_48KHz_osr256_actions[] =
	ANC_HEADSET_CPLS_AMIC1_AUXL_RX1_48000_OSR_256;

static struct adie_codec_hwsetting_entry headset_anc_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_anc_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_anc_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile headset_anc_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_anc_settings,
	.setting_sz = ARRAY_SIZE(headset_anc_settings),
};

static struct snddev_icodec_data snddev_anc_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_ANC),
	.name = "anc_headset_stereo_rx",
	.copp_id = PRIMARY_I2S_RX,
	.profile = &headset_anc_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.pamp_off = int_mic_disable,
};

static struct platform_device msm_anc_headset_device = {
	.name = "snddev_icodec",
	.id = 51,
	.dev = { .platform_data = &snddev_anc_headset_data },
};

static struct adie_codec_action_unit ispkr_stereo_48KHz_osr256_actions[] =
#ifdef CONFIG_MACH_RUBY
	SPEAKER_PRI_48000_OSR_256;
#else
	SPEAKER_PRI_STEREO_48000_OSR_256;
#endif

static struct adie_codec_hwsetting_entry ispkr_stereo_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispkr_stereo_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispkr_stereo_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispkr_stereo_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispkr_stereo_settings,
	.setting_sz = ARRAY_SIZE(ispkr_stereo_settings),
};

static struct snddev_icodec_data snddev_ispkr_stereo_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.profile = &ispkr_stereo_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = speaker_enable,
	.pamp_off = speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_SPEAKER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_SPEAKER,
	.default_aic3254_id = PLAYBACK_SPEAKER,
};

static struct platform_device msm_ispkr_stereo_device = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ispkr_stereo_data },
};

static struct adie_codec_action_unit idmic_mono_48KHz_osr256_actions[] =
#ifdef CONFIG_MACH_RUBY
	AMIC_PRI_MONO_48000_OSR_256;
#else
	DMIC1_PRI_MONO_OSR_256;
#endif

static struct adie_codec_hwsetting_entry idmic_mono_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idmic_mono_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idmic_mono_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idmic_mono_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idmic_mono_settings,
	.setting_sz = ARRAY_SIZE(idmic_mono_settings),
};

static struct snddev_icodec_data snddev_ispkr_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &idmic_mono_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.pamp_off = int_mic_disable,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_SPEAKER,
	.default_aic3254_id = VOICERECORD_IMIC,
};

static struct platform_device msm_ispkr_mic_device = {
	.name = "snddev_icodec",
	.id = 3,
	.dev = { .platform_data = &snddev_ispkr_mic_data },
};

#ifdef CONFIG_MACH_RUBY
static struct adie_codec_action_unit handset_dual_mic_endfire_8KHz_osr256_actions[] =
	DMIC1_PRI_STEREO_8000_OSR_256;

static struct adie_codec_action_unit spk_dual_mic_endfire_8KHz_osr256_actions[] =
	DMIC1_PRI_STEREO_8000_OSR_256;

static struct adie_codec_hwsetting_entry handset_dual_mic_endfire_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_dual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_dual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_hwsetting_entry spk_dual_mic_endfire_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = spk_dual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(spk_dual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile handset_dual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_dual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(handset_dual_mic_endfire_settings),
};

static struct adie_codec_dev_profile spk_dual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = spk_dual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(spk_dual_mic_endfire_settings),
};
#endif

#ifndef CONFIG_MACH_RUBY
static struct adie_codec_action_unit dual_mic_endfire_8KHz_osr256_actions[] =
	DMIC1_PRI_STEREO_OSR_256;

static struct adie_codec_hwsetting_entry dual_mic_endfire_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = dual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(dual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile dual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = dual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(dual_mic_endfire_settings),
};
#endif 

static struct snddev_icodec_data snddev_dual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &handset_dual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_enable_dmic_power,
	.pamp_off = msm_snddev_disable_dmic_power,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_RECEIVER,
	.default_aic3254_id = VOICERECORD_IMIC,
};

static struct platform_device msm_hs_dual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 14,
	.dev = { .platform_data = &snddev_dual_mic_endfire_data },
};

static struct snddev_icodec_data snddev_dual_mic_spkr_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &spk_dual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_enable_dmic_power,
	.pamp_off = msm_snddev_disable_dmic_power,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_SPEAKER,
	.default_aic3254_id = VOICERECORD_IMIC,
};

static struct platform_device msm_spkr_dual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 15,
	.dev = { .platform_data = &snddev_dual_mic_spkr_endfire_data },
};

static struct adie_codec_action_unit dual_mic_broadside_8osr256_actions[] =
#ifdef CONFIG_MACH_RUBY
	HS_DMIC2_STEREO_8000_OSR_256;
#else
	HS_DMIC2_STEREO_OSR_256;
#endif

static struct adie_codec_hwsetting_entry dual_mic_broadside_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = dual_mic_broadside_8osr256_actions,
		.action_sz = ARRAY_SIZE(dual_mic_broadside_8osr256_actions),
	}
};

static struct adie_codec_dev_profile dual_mic_broadside_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = dual_mic_broadside_settings,
	.setting_sz = ARRAY_SIZE(dual_mic_broadside_settings),
};

static struct snddev_icodec_data snddev_hs_dual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &dual_mic_broadside_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_enable_dmic_sec_power,
	.pamp_off = msm_snddev_disable_dmic_sec_power,
};

static struct platform_device msm_hs_dual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_hs_dual_mic_broadside_data },
};

static struct snddev_icodec_data snddev_spkr_dual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_broadside_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &dual_mic_broadside_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_enable_dmic_sec_power,
	.pamp_off = msm_snddev_disable_dmic_sec_power,
};

static struct platform_device msm_spkr_dual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_spkr_dual_mic_broadside_data },
};

static struct snddev_hdmi_data snddev_hdmi_stereo_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "hdmi_stereo_rx",
	.copp_id = HDMI_RX,
	.channel_mode = 0,
	.default_sample_rate = 48000,
};

static struct platform_device msm_snddev_hdmi_stereo_rx_device = {
	.name = "snddev_hdmi",
	.id = 0,
	.dev = { .platform_data = &snddev_hdmi_stereo_rx_data },
};

static struct snddev_mi2s_data snddev_mi2s_fm_tx_data = {
	.capability = SNDDEV_CAP_TX ,
	.name = "fmradio_stereo_tx",
	.copp_id = MI2S_TX,
	.channel_mode = 2, 
	.sd_lines = MI2S_SD3, 
	.sample_rate = 48000,
};

static struct platform_device msm_mi2s_fm_tx_device = {
	.name = "snddev_mi2s",
	.id = 0,
	.dev = { .platform_data = &snddev_mi2s_fm_tx_data },
};

static struct snddev_mi2s_data snddev_mi2s_fm_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "fmradio_stereo_rx",
	.copp_id = MI2S_RX,
	.channel_mode = 2, 
	.sd_lines = MI2S_SD3, 
	.sample_rate = 48000,
};

static struct platform_device msm_mi2s_fm_rx_device = {
	.name = "snddev_mi2s",
	.id = 1,
	.dev = { .platform_data = &snddev_mi2s_fm_rx_data },
};

static struct adie_codec_action_unit ifmradio_speaker_osr256_actions[] =
	AUXPGA_SPEAKER_RX;

static struct adie_codec_hwsetting_entry ifmradio_speaker_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ifmradio_speaker_osr256_actions,
		.action_sz = ARRAY_SIZE(ifmradio_speaker_osr256_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_speaker_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_speaker_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_speaker_settings),
};

static struct snddev_icodec_data snddev_ifmradio_speaker_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_speaker_rx",
	.copp_id = 0,
	.profile = &ifmradio_speaker_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = fm_speaker_enable,
	.pamp_off = fm_speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = FM_OUT_SPEAKER,
	.aic3254_voc_id = FM_OUT_SPEAKER,
	.default_aic3254_id = FM_OUT_SPEAKER,
};

static struct platform_device msm_ifmradio_speaker_device = {
	.name = "snddev_icodec",
	.id = 9,
	.dev = { .platform_data = &snddev_ifmradio_speaker_data },
};

static struct adie_codec_action_unit ifmradio_headset_osr256_actions[] =
	AUXPGA_HEADSET_AB_CPLS_RX_48000;

static struct adie_codec_hwsetting_entry ifmradio_headset_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ifmradio_headset_osr256_actions,
		.action_sz = ARRAY_SIZE(ifmradio_headset_osr256_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.profile = &ifmradio_headset_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = fm_headset_enable,
	.pamp_off = fm_headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = FM_OUT_HEADSET,
	.aic3254_voc_id = FM_OUT_HEADSET,
	.default_aic3254_id = FM_OUT_HEADSET,
};

static struct platform_device msm_ifmradio_headset_device = {
	.name = "snddev_icodec",
	.id = 10,
	.dev = { .platform_data = &snddev_ifmradio_headset_data },
};

static struct adie_codec_action_unit iheadset_mic_tx_osr256_actions[] =
#ifdef CONFIG_MACH_RUBY
	HS_AMIC2_MONO_48000_OSR_256;
#else
	HEADSET_AMIC2_TX_MONO_PRI_OSR_256;
#endif

static struct adie_codec_hwsetting_entry iheadset_mic_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iheadset_mic_tx_osr256_actions,
		.action_sz = ARRAY_SIZE(iheadset_mic_tx_osr256_actions),
	}
};

static struct adie_codec_dev_profile iheadset_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = iheadset_mic_tx_settings,
	.setting_sz = ARRAY_SIZE(iheadset_mic_tx_settings),
};

static struct snddev_icodec_data snddev_headset_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &iheadset_mic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
	.pamp_off = ext_mic_disable,
	.aic3254_id = VOICERECOGNITION_EMIC,
	.aic3254_voc_id = CALL_UPLINK_EMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_EMIC,
};

static struct platform_device msm_headset_mic_device = {
	.name = "snddev_icodec",
	.id = 33,
	.dev = { .platform_data = &snddev_headset_mic_data },
};

static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions[] =
	SPEAKER_HPH_AB_CPL_PRI_STEREO_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.profile = &ihs_stereo_speaker_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = headset_speaker_enable,
	.pamp_off = headset_speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = RING_HEADSET_SPEAKER,
	.aic3254_voc_id = RING_HEADSET_SPEAKER,
	.default_aic3254_id = RING_HEADSET_SPEAKER,
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 22,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data },
};

static struct snddev_ecodec_data snddev_bt_sco_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = PCM_RX,
	.channel_mode = 1,
};

static struct snddev_ecodec_data snddev_bt_sco_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = PCM_TX,
	.channel_mode = 1,
};

struct platform_device msm_bt_sco_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data },
};

struct platform_device msm_bt_sco_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 1,
	.dev = { .platform_data = &snddev_bt_sco_mic_data },
};

static struct adie_codec_action_unit itty_mono_tx_actions[] =
#ifdef CONFIG_MACH_RUBY
	TTY_HEADSET_MONO_TX_48000_OSR_256;
#else
	TTY_HEADSET_MONO_TX_OSR_256;
#endif

static struct adie_codec_hwsetting_entry itty_mono_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_mono_tx_actions,
		.action_sz = ARRAY_SIZE(itty_mono_tx_actions),
	},
};

static struct adie_codec_dev_profile itty_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = itty_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(itty_mono_tx_settings),
};

static struct snddev_icodec_data snddev_itty_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &itty_mono_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
	.pamp_off = ext_mic_disable,
	.aic3254_id = TTY_IN_FULL,
	.aic3254_voc_id = TTY_IN_FULL,
	.default_aic3254_id = TTY_IN_FULL,
};

static struct platform_device msm_itty_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 16,
	.dev = { .platform_data = &snddev_itty_mono_tx_data },
};

static struct adie_codec_action_unit itty_mono_rx_actions[] =
#ifdef CONFIG_MACH_RUBY
	TTY_HEADSET_MONO_RX_48000_OSR_256;
#else
	TTY_HEADSET_MONO_RX_8000_OSR_256;
#endif

static struct adie_codec_hwsetting_entry itty_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_mono_rx_actions,
		.action_sz = ARRAY_SIZE(itty_mono_rx_actions),
	},
};

static struct adie_codec_dev_profile itty_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = itty_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(itty_mono_rx_settings),
};

static struct snddev_icodec_data snddev_itty_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_rx",
	.copp_id = PRIMARY_I2S_RX,
	.profile = &itty_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = headset_enable,
	.pamp_off = headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = TTY_OUT_FULL,
	.aic3254_voc_id = TTY_OUT_FULL,
	.default_aic3254_id = TTY_OUT_FULL,
};

static struct platform_device msm_itty_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 17,
	.dev = { .platform_data = &snddev_itty_mono_rx_data },
};

static struct snddev_virtual_data snddev_uplink_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "uplink_rx",
	.copp_id = VOICE_PLAYBACK_TX,
};

static struct platform_device msm_uplink_rx_device = {
	.name = "snddev_virtual",
	.dev = { .platform_data = &snddev_uplink_rx_data },
};

static struct adie_codec_action_unit headset_mono_ab_cpls_48KHz_osr256_actions[] =
	HEADSET_AB_CPLS_48000_OSR_256; 

static struct adie_codec_hwsetting_entry headset_mono_ab_cpls_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = headset_mono_ab_cpls_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_mono_ab_cpls_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile headset_mono_ab_cpls_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_mono_ab_cpls_settings,
	.setting_sz = ARRAY_SIZE(headset_mono_ab_cpls_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.profile = &headset_mono_ab_cpls_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = headset_enable,
	.pamp_off = headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_HEADSET,
	.aic3254_voc_id = CALL_DOWNLINK_EMIC_HEADSET,
	.default_aic3254_id = PLAYBACK_HEADSET,
};

static struct platform_device msm_headset_mono_ab_cpls_device = {
	.name = "snddev_icodec",
	.id = 35,  
	.dev = { .platform_data = &snddev_ihs_mono_rx_data },
};

static struct adie_codec_action_unit ihs_ispk_stereo_rx_48KHz_osr256_actions[] =
	SPEAKER_HPH_AB_CPL_PRI_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ispk_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ispk_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ispk_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_ispk_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ispk_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ispk_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ispk_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_speaker_stereo_rx",
	.copp_id = 0,
	.profile = &ihs_ispk_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = headset_speaker_enable,
	.pamp_off = headset_speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = RING_HEADSET_SPEAKER,
	.aic3254_voc_id = RING_HEADSET_SPEAKER,
	.default_aic3254_id = RING_HEADSET_SPEAKER,
};

static struct platform_device msm_iheadset_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 36,  
	.dev = { .platform_data = &snddev_ihs_ispk_stereo_rx_data },
};

static struct adie_codec_action_unit bmic_tx_48KHz_osr256_actions[] =
	AMIC_SEC_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry bmic_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = bmic_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(bmic_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile bmic_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = bmic_tx_settings,
	.setting_sz = ARRAY_SIZE(bmic_tx_settings),
};

static struct snddev_icodec_data snddev_bmic_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "back_mic_tx",
	.copp_id = 1,
	.profile = &bmic_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = back_mic_enable,
	.pamp_off = back_mic_disable,
	.aic3254_id = VOICERECORD_EMIC,
	.aic3254_voc_id = VOICERECORD_EMIC,
	.default_aic3254_id = VOICERECORD_EMIC,
};

static struct platform_device msm_bmic_tx_device = {
	.name = "snddev_icodec",
	.id = 50, 
	.dev = { .platform_data = &snddev_bmic_tx_data },
};

static struct adie_codec_action_unit idual_mic_48KHz_osr256_actions[] =
	DUAL_MIC_STEREO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_settings),
};

static struct snddev_icodec_data snddev_idual_mic_endfire_real_stereo_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "dual_mic_stereo_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &idual_mic_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = stereo_mic_enable,
	.pamp_off = stereo_mic_disable,
	.aic3254_id = VIDEORECORD_IMIC,
	.aic3254_voc_id = VOICERECORD_EMIC, /* FIX ME */
	.default_aic3254_id = VIDEORECORD_IMIC,
};

static struct platform_device msm_real_stereo_tx_device = {
	.name = "snddev_icodec",
	.id = 26,  
	.dev = { .platform_data = &snddev_idual_mic_endfire_real_stereo_data },
};

static struct adie_codec_action_unit iusb_headset_stereo_rx_48KHz_osr256_actions[] =
	SPEAKER_HPH_AB_CPL_PRI_48000_OSR_256;

static struct adie_codec_hwsetting_entry iusb_headset_stereo_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iusb_headset_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iusb_headset_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iusb_headset_stereo_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iusb_headset_stereo_settings,
	.setting_sz = ARRAY_SIZE(iusb_headset_stereo_settings),
};

static struct snddev_icodec_data snddev_iusb_headset_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "usb_headset_stereo_rx",
	.copp_id = 0,
	.profile = &iusb_headset_stereo_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = usb_headset_enable,
	.pamp_off = usb_headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = USB_AUDIO,
	.aic3254_voc_id = USB_AUDIO,
	.default_aic3254_id = USB_AUDIO,
};

static struct platform_device msm_iusb_headset_rx_device = {
	.name = "snddev_icodec",
	.id = 27,  
	.dev = { .platform_data = &snddev_iusb_headset_stereo_rx_data },
};

static struct adie_codec_action_unit ispkr_mono_48KHz_osr256_actions[] =
	SPEAKER_PRI_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispkr_mono_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispkr_mono_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispkr_mono_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispkr_mono_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispkr_mono_settings,
	.setting_sz = ARRAY_SIZE(ispkr_mono_settings),
};

static struct snddev_icodec_data snddev_ispkr_mono_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_rx",
	.copp_id = 0,
	.profile = &ispkr_mono_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = speaker_enable,
	.pamp_off = speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_SPEAKER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_SPEAKER,
	.default_aic3254_id = PLAYBACK_SPEAKER,
};

static struct platform_device msm_ispkr_mono_device = {
	.name = "snddev_icodec",
	.id = 28,
	.dev = { .platform_data = &snddev_ispkr_mono_data },
};

static struct adie_codec_action_unit camcorder_imic_48KHz_osr256_actions[] =
	AMIC_PRI_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry camcorder_imic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = camcorder_imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(camcorder_imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile camcorder_imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = camcorder_imic_settings,
	.setting_sz = ARRAY_SIZE(camcorder_imic_settings),
};

static struct snddev_icodec_data snddev_camcorder_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "camcorder_mono_tx",
	.copp_id = 1,
	.profile = &camcorder_imic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = int_mic_enable,
	.pamp_off = int_mic_disable,
	.aic3254_id = VOICERECOGNITION_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_RECEIVER,
	.default_aic3254_id = VOICERECOGNITION_IMIC,
};

static struct platform_device msm_camcorder_imic_device = {
	.name = "snddev_icodec",
	.id = 53,
	.dev = { .platform_data = &snddev_camcorder_imic_data },
};

static struct adie_codec_action_unit camcorder_idual_mic_48KHz_osr256_actions[] =
	DUAL_MIC_STEREO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry camcorder_idual_mic_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = camcorder_idual_mic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(camcorder_idual_mic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile camcorder_idual_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = camcorder_idual_mic_settings,
	.setting_sz = ARRAY_SIZE(camcorder_idual_mic_settings),
};

static struct snddev_icodec_data snddev_camcorder_imic_stereo_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "camcorder_stereo_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &camcorder_idual_mic_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = stereo_mic_enable,
	.pamp_off = stereo_mic_disable,
	.aic3254_id = VIDEORECORD_IMIC,
	.aic3254_voc_id = VOICERECORD_EMIC, /* FIX ME */
	.default_aic3254_id = VIDEORECORD_IMIC,
};

static struct platform_device msm_camcorder_imic_stereo_device = {
	.name = "snddev_icodec",
	.id = 54,  
	.dev = { .platform_data = &snddev_camcorder_imic_stereo_data },
};

static struct adie_codec_action_unit camcorder_idual_mic_rev_48KHz_osr256_actions[] =
	DUAL_MIC_STEREO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry camcorder_idual_mic_rev_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = camcorder_idual_mic_rev_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(camcorder_idual_mic_rev_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile camcorder_idual_mic_rev_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = camcorder_idual_mic_rev_settings,
	.setting_sz = ARRAY_SIZE(camcorder_idual_mic_rev_settings),
};

static struct snddev_icodec_data snddev_camcorder_imic_stereo_rev_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "camcorder_stereo_rev_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &camcorder_idual_mic_rev_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = stereo_mic_enable,
	.pamp_off = stereo_mic_disable,
	.aic3254_id = VIDEORECORD_IMIC,
	.aic3254_voc_id = VOICERECORD_EMIC, /* FIX ME */
	.default_aic3254_id = VIDEORECORD_IMIC,
};

static struct platform_device msm_camcorder_imic_stereo_rev_device = {
	.name = "snddev_icodec",
	.id = 55,
	.dev = { .platform_data = &snddev_camcorder_imic_stereo_rev_data },
};

static struct adie_codec_action_unit camcorder_iheadset_mic_tx_osr256_actions[] =
	HS_AMIC2_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry camcorder_iheadset_mic_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = camcorder_iheadset_mic_tx_osr256_actions,
		.action_sz = ARRAY_SIZE(camcorder_iheadset_mic_tx_osr256_actions),
	}
};

static struct adie_codec_dev_profile camcorder_iheadset_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = camcorder_iheadset_mic_tx_settings,
	.setting_sz = ARRAY_SIZE(camcorder_iheadset_mic_tx_settings),
};

static struct snddev_icodec_data snddev_camcorder_headset_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "camcorder_headset_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &camcorder_iheadset_mic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
	.pamp_off = ext_mic_disable,
	.aic3254_id = VOICERECOGNITION_EMIC,
	.aic3254_voc_id = CALL_UPLINK_EMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_EMIC,
};

static struct platform_device msm_camcorder_headset_mic_device = {
	.name = "snddev_icodec",
	.id = 56,
	.dev = { .platform_data = &snddev_camcorder_headset_mic_data },
};

static struct adie_codec_action_unit vr_iearpiece_48KHz_osr256_actions[] =
	EAR_PRI_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry vr_iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = vr_iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(vr_iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile vr_iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = vr_iearpiece_settings,
	.setting_sz = ARRAY_SIZE(vr_iearpiece_settings),
};

static struct snddev_icodec_data snddev_vr_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "vr_handset_mono_tx",
	.copp_id = 0,
	.profile = &vr_iearpiece_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = handset_enable,
	.pamp_off = handset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_RECEIVER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_RECEIVER,
	.default_aic3254_id = PLAYBACK_RECEIVER,
};

static struct platform_device msm_vr_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 57,
	.dev = { .platform_data = &snddev_vr_iearpiece_data },
};

static struct adie_codec_action_unit vr_iheadset_mic_tx_osr256_actions[] =
	HS_AMIC2_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry vr_iheadset_mic_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = vr_iheadset_mic_tx_osr256_actions,
		.action_sz = ARRAY_SIZE(vr_iheadset_mic_tx_osr256_actions),
	}
};

static struct adie_codec_dev_profile vr_iheadset_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = vr_iheadset_mic_tx_settings,
	.setting_sz = ARRAY_SIZE(vr_iheadset_mic_tx_settings),
};

static struct snddev_icodec_data snddev_vr_headset_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "vr_headset_mono_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &vr_iheadset_mic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
	.pamp_off = ext_mic_disable,
	.aic3254_id = VOICERECOGNITION_EMIC,
	.aic3254_voc_id = CALL_UPLINK_EMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_EMIC,
};

static struct platform_device msm_vr_headset_mic_device = {
	.name = "snddev_icodec",
	.id = 58,
	.dev = { .platform_data = &snddev_vr_headset_mic_data },
};

static struct adie_codec_action_unit ispkr_mono_alt_48KHz_osr256_actions[] =
	SPEAKER_PRI_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispkr_mono_alt_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispkr_mono_alt_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispkr_mono_alt_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispkr_mono_alt_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispkr_mono_alt_settings,
	.setting_sz = ARRAY_SIZE(ispkr_mono_alt_settings),
};

static struct snddev_icodec_data snddev_ispkr_mono_alt_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_alt_rx",
	.copp_id = 0,
	.profile = &ispkr_mono_alt_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = speaker_enable,
	.pamp_off = speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_SPEAKER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_SPEAKER,
	.default_aic3254_id = PLAYBACK_SPEAKER,
};

static struct platform_device msm_ispkr_mono_alt_device = {
	.name = "snddev_icodec",
	.id = 59,
	.dev = { .platform_data = &snddev_ispkr_mono_alt_data },
};

static struct adie_codec_action_unit imic_note_48KHz_osr256_actions[] =
	AMIC_PRI_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry imic_note_settings[] = {
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_note_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_note_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_note_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_note_settings,
	.setting_sz = ARRAY_SIZE(imic_note_settings),
};

static struct snddev_icodec_data snddev_imic_note_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "imic_note_tx",
	.copp_id = 1,
	.profile = &imic_note_profile,
	.channel_mode = 2,
	.default_sample_rate = 16000,
	.pamp_on = stereo_mic_enable,
	.pamp_off = stereo_mic_disable,
	.aic3254_id = VOICERECORD_IMIC,
	.aic3254_voc_id = CALL_UPLINK_IMIC_RECEIVER,
	.default_aic3254_id = VOICERECORD_IMIC,
};

static struct platform_device msm_imic_note_device = {
	.name = "snddev_icodec",
	.id = 60,
	.dev = { .platform_data = &snddev_imic_note_data },
};

static struct adie_codec_action_unit ispkr_note_48KHz_osr256_actions[] =
	SPEAKER_PRI_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispkr_note_settings[] = {
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispkr_note_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispkr_note_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispkr_note_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispkr_note_settings,
	.setting_sz = ARRAY_SIZE(ispkr_note_settings),
};

static struct snddev_icodec_data snddev_ispkr_note_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "ispkr_note_rx",
	.copp_id = 0,
	.profile = &ispkr_note_profile,
	.channel_mode = 2,
	.default_sample_rate = 16000,
	.pamp_on = speaker_enable,
	.pamp_off = speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_SPEAKER,
	.aic3254_voc_id = CALL_DOWNLINK_IMIC_SPEAKER,
	.default_aic3254_id = PLAYBACK_SPEAKER,
};

static struct platform_device msm_ispkr_note_device = {
	.name = "snddev_icodec",
	.id = 61,
	.dev = { .platform_data = &snddev_ispkr_note_data },
};

static struct adie_codec_action_unit emic_note_16KHz_osr256_actions[] =
	AMIC_PRI_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry emic_note_settings[] = {
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = emic_note_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(emic_note_16KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile emic_note_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = emic_note_settings,
	.setting_sz = ARRAY_SIZE(emic_note_settings),
};

static struct snddev_icodec_data snddev_emic_note_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "emic_note_tx",
	.copp_id = 1,
	.profile = &emic_note_profile,
	.channel_mode = 1,
	.default_sample_rate = 16000,
	.pamp_on = ext_mic_enable,
	.pamp_off = ext_mic_disable,
	.aic3254_id = VOICERECORD_EMIC,
	.aic3254_voc_id = VOICERECORD_EMIC,
	.default_aic3254_id = VOICERECORD_EMIC,
};

static struct platform_device msm_emic_note_device = {
	.name = "snddev_icodec",
	.id = 62,
	.dev = { .platform_data = &snddev_emic_note_data },
};

static struct adie_codec_action_unit headset_note_48KHz_osr256_actions[] =
	HEADSET_AB_CPLS_48000_OSR_256;

static struct adie_codec_hwsetting_entry headset_note_settings[] = {
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = headset_note_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(headset_note_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile headset_note_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = headset_note_settings,
	.setting_sz = ARRAY_SIZE(headset_note_settings),
};

static struct snddev_icodec_data snddev_ihs_note_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_note_rx",
	.copp_id = 0,
	.profile = &headset_note_profile,
	.channel_mode = 2,
	.default_sample_rate = 16000,
	.pamp_on = headset_enable,
	.pamp_off = headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_HEADSET,
	.aic3254_voc_id = CALL_DOWNLINK_EMIC_HEADSET,
	.default_aic3254_id = PLAYBACK_HEADSET,
};

static struct platform_device msm_headset_note_device = {
	.name = "snddev_icodec",
	.id = 63,
	.dev = { .platform_data = &snddev_ihs_note_data },
};

static struct adie_codec_action_unit
	ihs_mono_speaker_mono_rx_48KHz_osr256_actions[] =
	SPEAKER_HPH_AB_CPL_PRI_STEREO_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_mono_speaker_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_speaker_mono_rx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(ihs_mono_speaker_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_speaker_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_mono_speaker_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_speaker_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_speaker_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_speaker_mono_rx",
	.copp_id = 0,
	.profile = &ihs_mono_speaker_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = headset_speaker_enable,
	.pamp_off = headset_speaker_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = RING_HEADSET_SPEAKER,
	.aic3254_voc_id = RING_HEADSET_SPEAKER,
	.default_aic3254_id = RING_HEADSET_SPEAKER,
};

static struct platform_device msm_ihs_mono_speaker_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 64,
	.dev = { .platform_data = &snddev_ihs_mono_speaker_mono_rx_data },
};

static struct adie_codec_action_unit
	fm_ihs_mono_tx_48KHz_osr256_actions[] =
	FM_HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	fm_ihs_mono_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = fm_ihs_mono_tx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(fm_ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile fm_ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = fm_ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(fm_ihs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_fm_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "fm_headset_mono_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &fm_ihs_mono_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
};

static struct platform_device msm_fm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 65,
	.dev = { .platform_data = &snddev_fm_ihs_mono_tx_data },
};

static struct adie_codec_action_unit beats_headset_ab_cpls_48KHz_osr256_actions[] =
	BEATS_HEADSET_AB_CPLS_48000_OSR_256;

static struct adie_codec_hwsetting_entry beats_headset_ab_cpls_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = beats_headset_ab_cpls_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(beats_headset_ab_cpls_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile beats_headset_ab_cpls_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = beats_headset_ab_cpls_settings,
	.setting_sz = ARRAY_SIZE(beats_headset_ab_cpls_settings),
};

static struct snddev_icodec_data snddev_beats_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "beats_headset_stereo_rx",
	.copp_id = 0,
	.profile = &beats_headset_ab_cpls_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = headset_enable,
	.pamp_off = headset_disable,
	.voltage_on = voltage_on,
	.voltage_off = voltage_off,
	.aic3254_id = PLAYBACK_HEADSET,
	.aic3254_voc_id = CALL_DOWNLINK_EMIC_HEADSET,
	.default_aic3254_id = PLAYBACK_HEADSET,
};

static struct platform_device msm_beats_headset_stereo_device = {
	.name = "snddev_icodec",
	.id = 66,
	.dev = { .platform_data = &snddev_beats_ihs_stereo_rx_data },
};

static struct adie_codec_action_unit ibeats_headset_mic_tx_osr256_actions[] =
	BEATS_HS_AMIC2_MONO_48000_OSR_256;

static struct adie_codec_hwsetting_entry ibeats_headset_mic_tx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ibeats_headset_mic_tx_osr256_actions,
		.action_sz = ARRAY_SIZE(ibeats_headset_mic_tx_osr256_actions),
	}
};

static struct adie_codec_dev_profile ibeats_headset_mic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ibeats_headset_mic_tx_settings,
	.setting_sz = ARRAY_SIZE(ibeats_headset_mic_tx_settings),
};

static struct snddev_icodec_data snddev_beats_headset_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "beats_headset_mono_tx",
	.copp_id = PRIMARY_I2S_TX,
	.profile = &ibeats_headset_mic_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = ext_mic_enable,
	.pamp_off = ext_mic_disable,
	.aic3254_id = VOICERECOGNITION_EMIC,
	.aic3254_voc_id = CALL_UPLINK_EMIC_HEADSET,
	.default_aic3254_id = VOICERECORD_EMIC,
};

static struct platform_device msm_beats_headset_mic_device = {
	.name = "snddev_icodec",
	.id = 67,
	.dev = { .platform_data = &snddev_beats_headset_mic_data },
};

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_action_unit
	ihs_stereo_rx_class_d_legacy_48KHz_osr256_actions[] =
	HPH_PRI_D_LEG_STEREO;

static struct adie_codec_hwsetting_entry
	ihs_stereo_rx_class_d_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_stereo_rx_class_d_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_stereo_rx_class_d_legacy_48KHz_osr256_actions),
	}
};

static struct adie_codec_action_unit
	ihs_stereo_rx_class_ab_legacy_48KHz_osr256_actions[] =
	HPH_PRI_AB_LEG_STEREO;

static struct adie_codec_hwsetting_entry
	ihs_stereo_rx_class_ab_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_stereo_rx_class_ab_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_stereo_rx_class_ab_legacy_48KHz_osr256_actions),
	}
};

static void snddev_hsed_config_modify_setting(int type)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_headset_stereo_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		if (type == 1) {
			icodec_data->voltage_on = NULL;
			icodec_data->voltage_off = NULL;
			icodec_data->profile->settings =
				ihs_stereo_rx_class_d_legacy_settings;
			icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_stereo_rx_class_d_legacy_settings);
		} else if (type == 2) {
			icodec_data->voltage_on = NULL;
			icodec_data->voltage_off = NULL;
			icodec_data->profile->settings =
				ihs_stereo_rx_class_ab_legacy_settings;
			icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_stereo_rx_class_ab_legacy_settings);
		}
	}
}

static void snddev_hsed_config_restore_setting(void)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_headset_stereo_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		icodec_data->voltage_on = msm_snddev_voltage_on;
		icodec_data->voltage_off = msm_snddev_voltage_off;
		icodec_data->profile->settings = headset_ab_cpls_settings;
		icodec_data->profile->setting_sz =
			ARRAY_SIZE(headset_ab_cpls_settings);
	}
}

static ssize_t snddev_hsed_config_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *lb_str = filp->private_data;
	char cmd;

	if (get_user(cmd, ubuf))
		return -EFAULT;

	if (!strcmp(lb_str, "msm_hsed_config")) {
		switch (cmd) {
		case '0':
			snddev_hsed_config_restore_setting();
			break;

		case '1':
			snddev_hsed_config_modify_setting(1);
			break;

		case '2':
			snddev_hsed_config_modify_setting(2);
			break;

		default:
			break;
		}
	}
	return cnt;
}

static int snddev_hsed_config_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations snddev_hsed_config_debug_fops = {
	.open = snddev_hsed_config_debug_open,
	.write = snddev_hsed_config_debug_write
};
#endif

static struct platform_device *snd_devices_common[] __initdata = {
/*	&msm_aux_pcm_device,
	&msm_cdcclk_ctl_device,
	&msm_mi2s_device,
    &msm_device_dspcrashd_8x60,  */	
	&msm_uplink_rx_device,
};

static struct platform_device *snd_devices_ruby[] __initdata = {
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ispkr_stereo_device,
	&msm_snddev_hdmi_stereo_rx_device,
	&msm_headset_mic_device,
	&msm_ispkr_mic_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_headset_stereo_device,
	&msm_itty_mono_tx_device,
	&msm_itty_mono_rx_device,
	&msm_mi2s_fm_tx_device,
	&msm_mi2s_fm_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_hs_dual_mic_endfire_device,
	&msm_spkr_dual_mic_endfire_device,
	&msm_hs_dual_mic_broadside_device,
	&msm_spkr_dual_mic_broadside_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_headset_mono_ab_cpls_device,
	&msm_iheadset_ispeaker_rx_device,
	&msm_bmic_tx_device,
	&msm_anc_headset_device,
	&msm_real_stereo_tx_device,
	&msm_ihac_device,
	&msm_nomic_headset_tx_device,
	&msm_nomic_headset_stereo_device,
	&msm_iusb_headset_rx_device,
	&msm_ispkr_mono_device,
	&msm_camcorder_imic_device,
	&msm_camcorder_imic_stereo_device,
	&msm_camcorder_imic_stereo_rev_device,
	&msm_camcorder_headset_mic_device,
	&msm_vr_iearpiece_device,
	&msm_vr_headset_mic_device,
	&msm_ispkr_mono_alt_device,
	&msm_imic_note_device,
	&msm_ispkr_note_device,
	&msm_emic_note_device,
	&msm_headset_note_device,
	&msm_ihs_mono_speaker_mono_rx_device,
	&msm_fm_ihs_mono_tx_device,
	&msm_beats_headset_stereo_device,
	&msm_beats_headset_mic_device,
};

void htc_8x60_register_analog_ops(struct q6v2audio_analog_ops *ops)
{
	audio_ops = ops;
}

void __init msm_snddev_init(void)
{
	int i;
	int dev_id;

	atomic_set(&preg_ref_cnt, 0);

	platform_add_devices(snd_devices_ruby, ARRAY_SIZE(snd_devices_ruby));

	for (i = 0, dev_id = 0; i < ARRAY_SIZE(snd_devices_common); i++)
		snd_devices_common[i]->id = dev_id++;

	platform_add_devices(snd_devices_common,
		ARRAY_SIZE(snd_devices_common));

#ifdef CONFIG_DEBUG_FS
	debugfs_hsed_config = debugfs_create_file("msm_hsed_config",
				S_IFREG | S_IRUGO, NULL,
		(void *) "msm_hsed_config", &snddev_hsed_config_debug_fops);
#endif
}
