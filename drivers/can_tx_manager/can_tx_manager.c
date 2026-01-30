/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

 #define DT_DRV_COMPAT rp_can_tx_manager
// TODO 实现 CAN TX manager 的功能

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <drivers/can_tx_manager.h>

#include <string.h>

typedef struct rp_can_tx_cfg {
    const struct device *can_dev;
} rp_can_tx_cfg_t;

typedef struct device_send_cfg {
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t device_index;
    device_send_cfg_t *next_device;
} device_send_cfg_t;

typedef struct rp_can_tx_data {
    device_send_cfg_t *device_list;
} rp_can_tx_data_t;

// 新增节点
device_send_cfg_t *device_send_cfg_add(device_send_cfg_t **head, uint16_t tx_id, uint16_t rx_id, uint8_t device_index)
{
    if (head == NULL)
        return NULL;
    device_send_cfg_t *node = (device_send_cfg_t *)k_malloc(sizeof(device_send_cfg_t));
    if (!node)
        return NULL;
    node->tx_id = tx_id;
    node->rx_id = rx_id;
    node->device_index = device_index;
    node->next_device = *head;
    *head = node;
    return node;
}

/*--------------------------------------Linked List Operations start-----------------------------------------------------------------*/
// 查找节点（按tx_id和rx_id）
device_send_cfg_t *device_send_cfg_find(device_send_cfg_t *head, uint16_t tx_id, uint16_t rx_id)
{
    device_send_cfg_t *cur = head;
    while (cur)
    {
        if (cur->tx_id == tx_id && cur->rx_id == rx_id)
        {
            return cur;
        }
        cur = cur->next_device;
    }
    return NULL;
}

// 删除节点（按tx_id和rx_id）
bool device_send_cfg_delete(device_send_cfg_t **head, uint16_t tx_id, uint16_t rx_id)
{
    if (head == NULL || *head == NULL)
        return false;
    device_send_cfg_t *cur = *head, *prev = NULL;
    while (cur)
    {
        if (cur->tx_id == tx_id && cur->rx_id == rx_id)
        {
            if (prev)
            {
                prev->next_device = cur->next_device;
            }
            else
            {
                *head = cur->next_device;
            }
            k_free(cur);
            return true;
        }
        prev = cur;
        cur = cur->next_device;
    }
    return false;
}

// 修改节点（按tx_id和rx_id查找，修改device_index）
bool device_send_cfg_update(device_send_cfg_t *head, uint16_t tx_id, uint16_t rx_id, uint8_t new_device_index)
{
    device_send_cfg_t *node = device_send_cfg_find(head, tx_id, rx_id);
    if (node)
    {
        node->device_index = new_device_index;
        return true;
    }
    return false;
}

/*--------------------------------------Linked List Operations end-----------------------------------------------------------------*/

int rp_can_tx_manager_send(const struct device *mgr, const struct can_frame *frame, const uint16_t tx_id, k_timeout_t timeout,
                           can_tx_callback_t callback, void *user_data)
{
    if (mgr == NULL || frame == NULL) {
        return -EINVAL;
    }

    const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
    if (cfg == NULL || cfg->can_dev == NULL) {
        return -ENODEV;
    }

    struct can_frame tx_frame = *frame;
    tx_frame.id = tx_id;

    int ret = can_send(cfg->can_dev, &tx_frame, timeout, callback, user_data);
    return ret;
}
