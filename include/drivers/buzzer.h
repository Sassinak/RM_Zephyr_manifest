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
        int note;     /* hz */
        int duration; /* msec */
    };

    struct song_config
    {
        const struct note_duration *notes;
        uint32_t length;
        uint8_t pace;
    };

    /**
     * @typedef buzzer_api_set_freq
     * @brief Callback API for set the frequency and volume of the buzzer.
     *
     */
    typedef int (*buzzer_api_set_freq)(const struct device *dev,
                                       float_t freq,
                                       uint8_t volume);

    /**
     * @typedef buzzer_api_play_note
     * @brief Callback API for play a note on the buzzer.
     *
     */
    typedef int (*buzzer_api_play_note)(const struct device *dev,
                                        note_duration *note_cfg);
    /**
     * @typedef buzzer_api_play_song
     * @brief Callback API for play a song on the buzzer.
     *
     */
    typedef int (*buzzer_api_play_song)(const struct device *dev,
                                        struct song_config *song_cfg);

    struct buzzer_driver_api
    {
        buzzer_api_set_freq set_freq;
        buzzer_api_play_note play_note;
        buzzer_api_play_song play_song;
    };

    static inline int buzzer_set_freq(const struct device *dev, float_t freq, uint8_t volume)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (api->set_freq == NULL)
        {
            return -ENOSYS;
        }
        return api->set_freq(dev, freq, volume);
    }

    static inline int buzzer_play_note(const struct device *dev, struct note_duration *note_cfg)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (api->play_note == NULL)
        {
            return -ENOSYS;
        }
        return api->play_note(dev, note_cfg);
    }

    static inline int buzzer_play_song(const struct device *dev, struct song_config *song_cfg)
    {
        const struct buzzer_driver_api *api = (const struct buzzer_driver_api *)dev->api;
        if (api->play_song == NULL)
        {
            return -ENOSYS;
        }
        return api->play_song(dev, song_cfg);
    }

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_BUZZER_BUZZER_H_ */
