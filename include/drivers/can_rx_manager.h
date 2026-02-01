/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef CAN_RX_MANAGER_H_
#define CAN_RX_MANAGER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*can_rx_handler_t)(const struct can_frame *frame, void *user_data);

/**
 * @brief Register a software RX handler inside a CAN RX manager.
 *
 * The manager owns the hardware RX filter(s) and a message queue. It receives CAN frames
 * and dispatches them to registered handlers by matching @p filter.
 *
 * @param mgr      CAN RX manager device.
 * @param filter   CAN filter used for software matching (standard/extended is controlled by flags).
 * @param handler  Called in manager RX thread context.
 * @param user_data Opaque pointer passed to handler.
 *
 * @retval >=0 Listener ID (can be used for unregister).
 * @retval -EINVAL Invalid arguments.
 * @retval -ENODEV Manager not ready.
 * @retval -ENOSPC No free listener slots.
 */
typedef int (*can_rx_manager_api_register)(const struct device *mgr, const struct can_filter *filter,
                                           can_rx_handler_t handler, void *user_data);

/**
 * @brief Unregister a previously registered listener.
 *
 * @param mgr CAN RX manager device.
 * @param listener_id Listener ID returned by can_rx_manager_api_register().
 *
 * @retval 0 on success.
 * @retval -EINVAL Invalid arguments.
 * @retval -ENOENT Listener not found.
 */
typedef int (*can_rx_manager_api_unregister)(const struct device *mgr, int listener_id);

// TODO 实现 bitrate 计算功能
typedef float (*can_rx_manager_api_calculate_bitrate)(const struct device *mgr);


struct can_rx_manager_api
{
    can_rx_manager_api_register register_listener;
    can_rx_manager_api_unregister unregister_listener;
    can_rx_manager_api_calculate_bitrate calculate_bitrate;
};


static inline int can_rx_manager_register(const struct device *mgr, const struct can_filter *filter,
                                        can_rx_handler_t handler, void *user_data)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->register_listener == NULL) {
        return -ENOSYS;
    }
    return api->register_listener(mgr, filter, handler, user_data);
}

static inline int can_rx_manager_unregister(const struct device *mgr, int listener_id)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->unregister_listener == NULL) {
        return -ENOSYS;
    }
    return api->unregister_listener(mgr, listener_id);
}

static inline float can_rx_manager_calculate_bitrate(const struct device *mgr)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->calculate_bitrate == NULL) {
        return -ENOSYS;
    }
    return api->calculate_bitrate(mgr);
} 


#ifdef __cplusplus
}
#endif

#endif /* CAN_RX_MANAGER_H_ */
