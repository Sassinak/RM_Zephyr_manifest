/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#ifndef CAN_TX_MANAGER_H_
#define CAN_TX_MANAGER_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef int (*rp_tx_fillbuffer_cb_t)(struct can_frame *frame, void *user_data);

/**
  * @brief Register a software TX handler inside a CAN TX manager.
 */
int can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id, rp_tx_fillbuffer_cb_t fill_buffer_cb, void *user_data);

int can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id);

int can_tx_manager_send(const struct device *mgr, k_timeout_t timeout, can_tx_callback_t callback, uint16_t tx_id, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TX_MANAGER_H_ */
