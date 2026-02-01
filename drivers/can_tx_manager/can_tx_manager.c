/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

 #define DT_DRV_COMPAT rp_can_tx_manager

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <drivers/can_tx_manager.h>

#include <string.h>

#define LOG_LEVEL 3
LOG_MODULE_REGISTER(can_tx_manager);

#ifdef CONFIG_CAN_TX_MANAGER

typedef struct rp_can_tx_cfg {
    const struct device *can_dev;
} rp_can_tx_cfg_t;

typedef struct device_sender_cfg {
    uint16_t tx_id;
    uint16_t rx_id;
    bool used;
    void *device;                               // 指向具体设备的指针
    rp_tx_fillbuffer_cb_t fill_buffer_cb;       // 用于填充发送数据的回调函数
} device_sender_cfg_t;

typedef struct rp_can_tx_data {
    device_sender_cfg_t device_list[CONFIG_MAX_DEVICE_SENDERS];
    struct can_frame *frame;                    // 实际管理的can帧
    struct k_mutex lock;                         // 保护 data 的互斥体
    uint8_t frame_num;                          // 发送帧的数量
} rp_can_tx_data_t;


int rp_can_tx_manager_init(const struct device *dev)
{
    const rp_can_tx_cfg_t *cfg = dev->config;
    struct rp_can_tx_data *data = dev->data;

    if ((cfg == NULL) || (data == NULL)) {
        return -EINVAL;
    }

    /* 安全初始化：不要对包含内核对象的结构做 memset，逐项初始化 */
    (void)cfg;
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        data->device_list[i].used = false;
        data->device_list[i].tx_id = 0;
        data->device_list[i].rx_id = 0;
        data->device_list[i].fill_buffer_cb = NULL;
    }
    data->frame = k_malloc(MAX_CAN_FRAMES * sizeof(struct can_frame));      // 分配最大数量的 CAN 帧
    if (data->frame == NULL)
    {
        LOG_ERR("[can_tx_manager]Failed to allocate memory for CAN frames");
        return -ENOMEM;
    }
    data->frame_num = 0;
    k_mutex_init(&data->lock);                                         // 初始化互斥体

    return 0;
}

/**
 * @brief 注册一个 CAN 发送设备到 TX manager
 * 
 * @param mgr CAN TX manager 设备
 * @param tx_id 发送的 CAN 帧 ID
 * @param rx_id 接收的 CAN 帧 ID
 * @param fill_buffer_cb 用于填充发送数据的回调函数
 * @return int 返回注册的索引 ID，失败返回负值错误码
 */
int rp_can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id, rp_tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
{
    if (!device_is_ready(mgr))
    {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    struct rp_can_tx_data *data = mgr->data;
    if (data == NULL)
    {
        LOG_ERR("[can_tx_manager]CAN TX manager data is NULL");
        return -EINVAL;
    }
    /* 保护并发访问：保持锁直到准备返回 */
    k_mutex_lock(&data->lock, K_FOREVER);

    if (data->frame == NULL) {
        LOG_ERR("[can_tx_manager]CAN frames not initialized");
        k_mutex_unlock(&data->lock);
        return -EFAULT;
    }

    /* 检查是否已注册过相同的 tx_id */
    bool found = false;
    for (int f = 0; f < data->frame_num; f++) {
        if (data->frame[f].id == tx_id) {
            found = true;
            break;
        }
    }

    if (!found) {
        if (data->frame_num >= CONFIG_MAX_CAN_FRAMES) {
            LOG_ERR("[can_tx_manager]No space left for CAN frames");
            k_mutex_unlock(&data->lock);
            return -ENOSPC;
        }
        memset(&data->frame[data->frame_num], 0, sizeof(struct can_frame));
        data->frame[data->frame_num].id = tx_id;
        data->frame_num++;
    }
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (data->device_list[i].used) {
            continue;
        }
        data->device_list[i].used = true;
        data->device_list[i].tx_id = tx_id;
        data->device_list[i].rx_id = rx_id;
        data->device_list[i].device = user_data;
        data->device_list[i].fill_buffer_cb = fill_buffer_cb;
        k_mutex_unlock(&data->lock);
        return i; /* 返回分配到的索引 */
    }

    /* 无空槽 */
    k_mutex_unlock(&data->lock);
    return -ENOSPC;
}

/**
 * @brief 注销一个已注册的 CAN 发送设备
 * 
 * @param mgr 
 * @param tx_id 
 * @param rx_id 
 * @return int 
 */
int rp_can_tx_manager_unregister(const struct device *mgr, const uint16_t tx_id, const uint16_t rx_id)
{
    if (!device_is_ready(mgr))
    {
        LOG_ERR("[can_tx_manager]CAN TX manager device not ready");
        return -ENODEV;
    }

    struct rp_can_tx_data *data = mgr->data;
    if (data == NULL)
    {
        LOG_ERR("[can_tx_manager]CAN TX manager data is NULL");
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (!data->device_list[i].used) {
            continue;
        }
        if (data->device_list[i].tx_id == tx_id && data->device_list[i].rx_id == rx_id) {
            data->device_list[i].used = false;
            data->device_list[i].tx_id = 0;
            data->device_list[i].rx_id = 0;
            data->device_list[i].user_data = NULL;
            data->device_list[i].fill_buffer_cb = NULL;
            k_mutex_unlock(&data->lock);
            return 0; /* 成功注销 */
        }
    }

    k_mutex_unlock(&data->lock);
    return -ENOENT; /* 未找到匹配的注册项 */
}


int rp_can_tx_fillbuffer(uint16_t tx_id, struct can_frame *frame, rp_can_tx_data_t *data, void *user_datas)
{
    if (frame == NULL || data == NULL) {
        LOG_ERR("[can_tx_manager]Invalid frame or data pointer");
        return -EINVAL;
    }

    /* 收集回调指针*/
    rp_tx_fillbuffer_cb_t cbs[CONFIG_MAX_DEVICE_SENDERS];
    int cb_count = 0;

    k_mutex_lock(&data->lock, K_FOREVER);
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (!data->device_list[i].used) {
            continue;
        }
        if (data->device_list[i].tx_id != tx_id) {
            continue;
        }
        if (data->device_list[i].fill_buffer_cb == NULL) {
            continue;
        }
        cbs[cb_count] = data->device_list[i].fill_buffer_cb;
        cb_count++;
    }
    k_mutex_unlock(&data->lock);

    if (cb_count == 0) {
        LOG_ERR("[can_tx_manager]No fill buffer callback for tx_id 0x%03x", tx_id);
        return -EINVAL;
    }
    if(cb_count > 3)
    {
        LOG_ERR("[can_tx_manager]Too many fill buffer callbacks (%d) for tx_id 0x%03x", cb_count, tx_id);
        return -EINVAL;
    }

    for (int j = 0; j < cb_count; j++) {
        int ret = cbs[j](frame, user_datas);
        if (ret != 0) {
            LOG_ERR("[can_tx_manager]Fill buffer callback failed for tx_id 0x%03x, err %d", tx_id, ret);
            return ret;
        }
    }

    return 0;
}

/**
 * @brief 通过can tx manager 发送 can 帧
 * 
 * @param mgr can总线管理器设备
 * @param timeout 发送超时时间
 * @param callback 发送完成回调函数
 * @param tx_filter_id 发送过滤器ID，由注册设备时返回
 * @param user_data 用户数据
 * @return int 
 */
int rp_can_tx_manager_send(const struct device *mgr, k_timeout_t timeout, can_tx_callback_t callback, uint16_t tx_id, void *user_data)
{
    if (mgr == NULL) {
        return -EINVAL;
    }

    const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
    rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
    if (cfg == NULL || cfg->can_dev == NULL || data == NULL || data->frame == NULL) {
        LOG_ERR("[can_tx_manager]Invalid CAN TX manager configuration");
        return -ENODEV;
    }
    if(data->frame_num == 0) {
        LOG_ERR("[can_tx_manager]No CAN frames registered for transmission");
        return -EINVAL;
    }
    int frame_index = -1;
    for (int i = 0; i < data->frame_num; i++) {
        if (tx_id == data->frame[i].id) {
            frame_index = i;
            break;
        }
    }

    if (frame_index < 0) {
        LOG_ERR("[can_tx_manager]Frame for tx_id 0x%03x not found", tx_id);
        return -ENOENT;
    }

    int ret = rp_can_tx_fillbuffer(tx_id, &data->frame[frame_index], data, data->device_list[frame_index].device);
    if (ret != 0) {
        return ret;
    }

    return can_send(cfg->can_dev, &data->frame[frame_index], timeout, callback, data->device_list[frame_index].device);
}

// TODO 实现自动can发送管理任务，定频发送等功能 --- IGNORE ---


#define RP_CAN_TX_MGR_DEFINE(inst)                                                              \
    static const struct rp_can_tx_cfg rp_can_tx_mgr_cfg_##inst = {                      \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                               \
    };                                                                                          \
    static struct rp_can_tx_data rp_can_tx_mgr_data_##inst;                             \
    DEVICE_DT_INST_DEFINE(inst, rp_can_tx_manager_init, NULL, &rp_can_tx_mgr_data_##inst,       \
                          &rp_can_tx_mgr_cfg_##inst, POST_KERNEL, CONFIG_CAN_TX_MANAGER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_TX_MGR_DEFINE)

#endif /* CAN_TX_MANAGER */
