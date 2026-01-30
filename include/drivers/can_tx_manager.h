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

/**
  * @brief Register a software TX handler inside a CAN TX manager.
 */
int can_tx_manager_register(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id);


int can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id);

int can_tx_manager_send(const struct device *mgr, const struct can_frame *frame, const uint16_t tx_id, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TX_MANAGER_H_ */
