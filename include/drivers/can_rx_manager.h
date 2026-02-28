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

/**
 * @brief Calculate current CAN bus load (percentage) over the interval since last call.
 * @param mgr CAN RX manager device
 * @param nominal_bitrate_bps Bitrate used for arbitration/CRC/control (bps)
 * @param data_bitrate_bps Bitrate used for FD data phase when BRS is set (bps). If 0, FD data is assumed to use nominal rate.
 * @return load percentage in range [0.0 .. 100.0], negative on error.
 */
typedef float (*can_rx_manager_api_calculate_load)(const struct device *mgr, uint32_t nominal_bitrate_bps, uint32_t data_bitrate_bps);


struct can_rx_manager_api
{
    can_rx_manager_api_register register_listener;
    can_rx_manager_api_unregister unregister_listener;
    can_rx_manager_api_calculate_load calculate_load;
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

/**
 * @brief 计算can回路的负载率
 *
 * @param mgr 管理器设备
 * @param nominal_bitrate_bps 标称比特率 (bps)
 * @param data_bitrate_bps 数据比特率 (bps)，如果为0，则假定FD数据使用标称速率
 * @return  float 负载率百分比，错误时返回负值
 */
static inline float can_rx_manager_calculate_load(const struct device *mgr, uint32_t nominal_bitrate_bps, uint32_t data_bitrate_bps)
{
    const struct can_rx_manager_api *api = (const struct can_rx_manager_api *)mgr->api;
    if (api->calculate_load == NULL) {
        return -ENOSYS;
    }
    return api->calculate_load(mgr, nominal_bitrate_bps, data_bitrate_bps);
}


#ifdef __cplusplus
}
#endif

#endif /* CAN_RX_MANAGER_H_ */
