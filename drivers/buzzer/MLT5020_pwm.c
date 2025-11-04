/* drivers/buzzer/MLT5020_pwm.c */
/*
 * Copyright (c) 2025 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT RP_pwm_buzzer

#include <drivers/buzzer.h>

#define LOG_LEVEL CONFIG_BUZZER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(buzzer_pwm);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>

#define PWM_MAX_VOLUME 100

struct buzzer_pwm_cfg{
    const struct pwm_dt_spec pwm;
    uint32_t freq;
    uint8_t volume;
};


static int buzzer_pwm_set_config(const struct device *dev, uint32_t freq, uint8_t volume)
{
    if (dev == NULL) {
        LOG_ERR("Device is NULL");
        return -ENODEV;
    }

    const struct buzzer_pwm_cfg *cfg = dev->config;

    if (!pwm_is_ready_dt(&cfg->pwm)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    if (freq == 0 || volume > PWM_MAX_VOLUME) {
        LOG_ERR("Invalid frequency or volume");
        return -EINVAL;
    }

    uint32_t period_ns = NSEC_PER_SEC / freq;
    uint32_t pulse_ns  = (period_ns * volume) / PWM_MAX_VOLUME;

    return pwm_set_dt(&cfg->pwm, period_ns, pulse_ns);
}

static int buzzer_pwm_play_note(const struct device *dev, const struct note_duration *note_cfg)
{
    const struct buzzer_pwm_cfg *cfg = dev->config;

    if (!pwm_is_ready_dt(&cfg->pwm)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }
    if (note_cfg == NULL || note_cfg->duration <= 0) {
        return -EINVAL;
    }

    if (note_cfg->note == 0) {
        /* 休止符：设为 0% 占空比即可 */
        (void)pwm_set_pulse_dt(&cfg->pwm, 0);
        k_msleep(note_cfg->duration);
        return 0;
    }

    /* period/pulse 单位必须是纳秒 */
    uint32_t period_ns = NSEC_PER_SEC / (uint32_t)note_cfg->note;
    uint32_t pulse_ns  = (period_ns * cfg->volume) / PWM_MAX_VOLUME;

    int ret = pwm_set_dt(&cfg->pwm, period_ns, pulse_ns);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM for note %d Hz: %d", note_cfg->note, ret);
        return ret;
    }

    k_msleep(note_cfg->duration);

    /* 播放结束，拉成 0% 占空比静音 */
    (void)pwm_set_pulse_dt(&cfg->pwm, 0);
    return 0;
}

/* tempo 换算：从 BPM + 每拍单位 + 本音符分母 得到毫秒
 * - bpm>0 时有效；beat_unit/div 负数表示附点（×1.5）
 * - bpm==0 时不使用此函数（直接用毫秒）
 */
static inline uint32_t buzzer_ms_from_tempo(uint16_t bpm, int beat_unit, int div)
{
    if (bpm == 0 || div == 0) {
        return 0U;
    }

    uint32_t beat_den = (uint32_t)((beat_unit >= 0) ? beat_unit : -beat_unit);
    uint32_t note_den = (uint32_t)((div       >= 0) ? div       : -div);
    if (beat_den == 0U) {
        beat_den = 4U; /* 默认四分为一拍 */
    }

    /* 附点系数 3/2 */
    uint32_t beat_num = (beat_unit < 0) ? 3U : 1U;
    uint32_t beat_den_fac = (beat_unit < 0) ? 2U : 1U;
    uint32_t note_num = (div < 0) ? 3U : 1U;
    uint32_t note_den_fac = (div < 0) ? 2U : 1U;

    /* ms = (60000/bpm) * (beat_den/note_den) * (note_factor/beat_factor) */
    uint64_t numerator   = 60000ULL * beat_den * note_num * beat_den_fac;
    uint64_t denominator = (uint64_t)bpm * note_den * note_den_fac * beat_num;

    return (uint32_t)(numerator / denominator);
}

static int buzzer_pwm_play_song(const struct device *dev, struct song_config *song_cfg)
{
    if (dev == NULL || song_cfg == NULL || song_cfg->notes == NULL) {
        return -EINVAL;
    }
    const struct buzzer_pwm_cfg *cfg = dev->config;
    if (!pwm_is_ready_dt(&cfg->pwm)) {
        return -ENODEV;
    }

    int first_err = 0;
    int beat_unit = (song_cfg->beat_unit != 0) ? song_cfg->beat_unit : 4; /* 默认四分为一拍 */
    uint16_t bpm = song_cfg->pace;

    for (uint32_t i = 0; i < song_cfg->length; ++i) {
        const struct note_duration *n = &song_cfg->notes[i];

        /* 计算时值（ms）：bpm>0 用乐谱分母；否则用毫秒 */
        uint32_t dur_ms = 0U;
        if (bpm > 0 && n->div != 0) {
            dur_ms = buzzer_ms_from_tempo(bpm, beat_unit, n->div);
        } else {
            dur_ms = (uint32_t)((n->duration > 0) ? n->duration : 0);
        }
        if (dur_ms == 0U) {
            continue;
        }

        if (n->note == 0) {
            /* 休止：关断并等待 */
            (void)pwm_set_pulse_dt(&cfg->pwm, 0);
            k_msleep(dur_ms);
            continue;
        }

        uint32_t period_ns = NSEC_PER_SEC / (uint32_t)n->note;
        uint32_t pulse_ns  = (period_ns * cfg->volume) / PWM_MAX_VOLUME;

        int ret = pwm_set_dt(&cfg->pwm, period_ns, pulse_ns);
        if (ret < 0 && first_err == 0) {
            first_err = ret;
        }

        k_msleep(dur_ms);
        /* 连音衔接：不在音符间强制关断；休止或曲终再关断 */
    }

    (void)pwm_set_pulse_dt(&cfg->pwm, 0);
    return first_err;
}
