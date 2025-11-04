/* drivers/buzzer/buzzer.h */
/*
 * Copyright (c) 2025 RobotPilots
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_BUZZER_BUZZER_H_
#define DRIVERS_BUZZER_BUZZER_H_

#include <zephyr/types.h>
#include <errno.h>

#include <zephyr/device.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    struct note_duration
    {
        int note;           /* Hz; 0=休止符 */
        int duration;       /* 毫秒：当 pace==0 时使用 */
        int div;            /* 乐谱分母：1=全,2=二分,4=四分,8=八分...; 负数=附点；当 pace>0 时使用 */
    };

    struct song_config
    {
        const struct note_duration *notes;
        uint32_t length;
        uint8_t pace;       /* 0=按毫秒; >0=BPM */
        int8_t  beat_unit;  /* 每拍对应的音符：正值=分母(如4)，负值=附点该分母(如-4)；0=默认4 */
    };

    /**
     * @typedef buzzer_api_set_freq
     * @brief Callback API for set the frequency and volume of the buzzer.
     *
     */
    typedef int (*buzzer_api_set_config)(const struct device *dev,
                                       uint32_t freq,
                                       uint8_t volume);

    /**
     * @typedef buzzer_api_play_note
     * @brief Callback API for play a note on the buzzer.
     *
     */
    typedef int (*buzzer_api_play_note)(const struct device *dev,
                                        const struct note_duration *note_cfg);
    /**
     * @typedef buzzer_api_play_song
     * @brief Callback API for play a song on the buzzer.
     *
     */
    typedef int (*buzzer_api_play_song)(const struct device *dev,
                                        struct song_config *song_cfg);

    struct buzzer_driver_api
    {
        buzzer_api_set_config set_config;
        buzzer_api_play_note play_note;
        buzzer_api_play_song play_song;
    };

    static inline int buzzer_set_config(const struct device *dev, uint32_t freq, uint8_t volume)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->set_config == NULL) {
            return -ENOSYS;
        }
        return api->set_config(dev, freq, volume);
    }

    static inline int buzzer_play_note(const struct device *dev, const struct note_duration *note_cfg)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->play_note == NULL) {
            return -ENOSYS;
        }
        return api->play_note(dev, note_cfg);
    }

    static inline int buzzer_play_song(const struct device *dev, struct song_config *song_cfg)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (!api || api->play_song == NULL) {
            return -ENOSYS;
        }
        return api->play_song(dev, song_cfg);
    }

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BUZZER_BUZZER_H_ */
