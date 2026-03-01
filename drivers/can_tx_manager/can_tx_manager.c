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

#define _RP_CAN_TX_MGR_DEV_PTR(inst) DEVICE_DT_INST_GET(inst),

typedef struct rp_can_tx_cfg {
    const struct device *can_dev;
} rp_can_tx_cfg_t;

typedef struct device_sender_cfg {
    uint16_t tx_id;
    uint16_t rx_id;
    bool used;
    void *user_data;                               // 指向具体设备的指针
    tx_fillbuffer_cb_t fill_buffer_cb;       // 用于填充发送数据的回调函数
} device_sender_cfg_t;

typedef struct rp_can_item {
    struct can_frame frame;
    uint16_t frequency;             // 发送频率，单位为 Hz，0 表示不定期发送（事件触发）
    uint16_t interval;              // 发送间隔 tick 数（注册时预计算：1000 / frequency）
    uint16_t tick_counter;          // 距离下次发送的已累计 tick 数（1 tick = 1ms）
} rp_can_item_t;

typedef struct rp_can_tx_data
{
    device_sender_cfg_t sender_list[CONFIG_MAX_DEVICE_SENDERS];
    rp_can_item_t can_items[CONFIG_MAX_CAN_FRAMES];    /* 实际管理的can帧，静态分配以避免 heap 依赖 */
    struct k_mutex lock;                         /* 保护 data 的互斥体 */
    uint8_t frame_num;                          /* 发送帧的数量 */
} rp_can_tx_data_t;

/**
 * @brief 发送管理器初始化函数，主要是初始化数据结构和互斥体
 *
 * @param dev 指向设备实例的指针
 * @return int
 */
int rp_can_tx_manager_init(const struct device *dev)
{
    const rp_can_tx_cfg_t *cfg = dev->config;
    struct rp_can_tx_data *data = dev->data;

    if ((cfg == NULL) || (data == NULL)) {
        LOG_ERR("[can_tx_manager]CAN TX manager init failed - invalid config or data");
        return -EINVAL;
    }

    /* 安全初始化：不要对包含内核对象的结构做 memset，逐项初始化 */
    (void)cfg;
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        data->sender_list[i].used = false;
        data->sender_list[i].tx_id = 0;
        data->sender_list[i].rx_id = 0;
        data->sender_list[i].fill_buffer_cb = NULL;
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
 * @param rx_id 接收的 CAN 帧 ID（预留，当前未用于接收过滤）
 * @param dlc 数据长度码（CAN 2.0 标准帧通常为 8）
 * @param flags CAN 帧标志位，参见 CAN_FRAME_FLAGS（标准帧填 0）
 * @param fill_buffer_cb 用于填充发送数据的回调函数
 * @return int 返回注册的索引 ID，失败返回负值错误码
 */
int rp_can_tx_manager_register(const struct device *mgr, uint16_t tx_id, uint16_t rx_id,
                                uint8_t dlc, uint8_t flags, uint16_t frequency,
                                tx_fillbuffer_cb_t fill_buffer_cb, void *user_data)
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

    /* 检查是否已注册过相同的 tx_id */
    bool found = false;
    for (int f = 0; f < data->frame_num; f++) {
        if (data->can_items[f].frame.id == tx_id) {
            if (data->can_items[f].frequency != frequency) {
                LOG_ERR("[can_tx_manager]Cannot register same tx_id 0x%03x with different frequency (existing %d Hz, new %d Hz)", tx_id, data->can_items[f].frequency, frequency);
                k_mutex_unlock(&data->lock);
                return -EINVAL;
            }
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
        memset(&data->can_items[data->frame_num], 0, sizeof(rp_can_item_t));
        data->can_items[data->frame_num].frame.id = tx_id;
        data->can_items[data->frame_num].frame.dlc = dlc;
        data->can_items[data->frame_num].frame.flags = flags;
        data->can_items[data->frame_num].frequency = frequency;
        data->can_items[data->frame_num].interval = (frequency > 0) ? (uint16_t)(1000U / frequency) : 0;
        if (data->can_items[data->frame_num].interval == 0 && frequency > 0) {
            data->can_items[data->frame_num].interval = 1;
        }
        data->frame_num++;
    }
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (data->sender_list[i].used) {
            continue;
        }
        data->sender_list[i].used = true;
        data->sender_list[i].tx_id = tx_id;
        data->sender_list[i].rx_id = rx_id;
        data->sender_list[i].user_data = user_data;
        data->sender_list[i].fill_buffer_cb = fill_buffer_cb;
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
        if (!data->sender_list[i].used) {
            continue;
        }
        if (data->sender_list[i].tx_id == tx_id && data->sender_list[i].rx_id == rx_id) {
            data->sender_list[i].used = false;
            data->sender_list[i].tx_id = 0;
            data->sender_list[i].rx_id = 0;
            data->sender_list[i].user_data = NULL;
            data->sender_list[i].fill_buffer_cb = NULL;
            k_mutex_unlock(&data->lock);
            return 0; /* 成功注销 */
        }
    }

    k_mutex_unlock(&data->lock);
    return -ENOENT; /* 未找到匹配的注册项 */
}


/**
 * @brief 调用注册的回调函数填充 CAN 发送缓冲区，供发送函数调用
 *
 * @param tx_id
 * @param frame
 * @param data
 * @param user_data
 * @return int
 */
static int rp_can_tx_fillbuffer(uint16_t tx_id, struct can_frame *frame, rp_can_tx_data_t *data)
{
    if (frame == NULL || data == NULL) {
        LOG_ERR("[can_tx_manager]Invalid frame or data pointer");
        return -EINVAL;
    }

    /* 调用每个匹配注册项的回调，传入该注册项保存的 user_data */
    int cb_count = 0;
    for (int i = 0; i < CONFIG_MAX_DEVICE_SENDERS; i++) {
        if (!data->sender_list[i].used) {
            continue;
        }
        if (data->sender_list[i].tx_id != tx_id) {
            continue;
        }
        if (data->sender_list[i].fill_buffer_cb == NULL) {
            continue;
        }
        cb_count++;
        int ret = data->sender_list[i].fill_buffer_cb(frame, data->sender_list[i].user_data);
        if (ret != 0) {
            LOG_ERR("[can_tx_manager]Fill buffer callback failed for tx_id 0x%03x, err %d", tx_id, ret);
            return ret;
        }
    }

    if (cb_count == 0)
    {
        LOG_ERR("[can_tx_manager]No fill buffer callback for tx_id 0x%03x", tx_id);
        return -EINVAL;
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
        LOG_ERR("[can_tx_manager]CAN TX manager device is NULL");
        return -EINVAL;
    }

    const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
    rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
    if (cfg == NULL || cfg->can_dev == NULL || data == NULL) {
        LOG_ERR("[can_tx_manager]Invalid CAN TX manager configuration");
        return -ENODEV;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    if(data->frame_num == 0) {
        LOG_ERR("[can_tx_manager]No CAN frames registered for transmission");
        k_mutex_unlock(&data->lock);
        return -EINVAL;
    }
    int frame_index = -1;
    for (int i = 0; i < data->frame_num; i++) {
        if (tx_id == data->can_items[i].frame.id) {
            frame_index = i;
            break;
        }
    }

    if (frame_index < 0) {
        LOG_ERR("[can_tx_manager]Frame for tx_id 0x%03x not found", tx_id);
        k_mutex_unlock(&data->lock);
        return -ENOENT;
    }

    int ret = rp_can_tx_fillbuffer(tx_id, &data->can_items[frame_index].frame, data);
    if (ret != 0) {
        k_mutex_unlock(&data->lock);
        return ret;
    }

    /* can_send 在锁内调用，防止定频线程并发修改同一帧数据 */
    int send_ret = can_send(cfg->can_dev, &data->can_items[frame_index].frame, timeout, callback, user_data);
    k_mutex_unlock(&data->lock);
    return send_ret;
}


static const struct can_tx_manager_api rp_can_tx_mgr_api = {
    .register_sender = rp_can_tx_manager_register,
    .unregister_sender =  rp_can_tx_manager_unregister,
    .send_frame = rp_can_tx_manager_send,
};


/* 非阻塞发送的空回调，避免阻塞周期线程 */
static void can_tx_mgr_tx_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(error);
    ARG_UNUSED(user_data);
}

/* 硬件定时器 + 信号量：定时器 ISR 给信号，线程消费 */
static K_SEM_DEFINE(s_tx_tick_sem, 0, 1);
static struct k_timer s_tx_timer;

/**
 * @brief 硬件定时器到期回调（运行在 ISR 上下文）
 *        仅给信号量，不做任何实际工作，避免在 ISR 中执行耗时操作。
 */
static void can_tx_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&s_tx_tick_sem);
}

/**
 * @brief 1000Hz 定频发送线程
 *        由硬件定时器精确触发（每 1ms），遍历所有已注册的
 *        TX manager 实例，对每个实例的每个定频帧填充并发送。
 */
static void can_tx_manager_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* 编译期静态数组，包含所有 DT 使能的 TX manager 实例 */
    static const struct device *const devs[] = {
        DT_INST_FOREACH_STATUS_OKAY(_RP_CAN_TX_MGR_DEV_PTR)
    };
    static const int dev_count = ARRAY_SIZE(devs);

    /* 启动硬件定时器，周期 1ms */
    k_timer_init(&s_tx_timer, can_tx_timer_expiry, NULL);
    k_timer_start(&s_tx_timer, K_MSEC(1), K_MSEC(1));

    while (1) {
        /* 等待定时器 ISR 触发，精度由硬件定时器保证 */
        k_sem_take(&s_tx_tick_sem, K_FOREVER);

        for (int d = 0; d < dev_count; d++) {
            const struct device *mgr = devs[d];
            const rp_can_tx_cfg_t *cfg = (const rp_can_tx_cfg_t *)mgr->config;
            rp_can_tx_data_t *data = (rp_can_tx_data_t *)mgr->data;
            if (cfg == NULL || cfg->can_dev == NULL || data == NULL) {
                continue;
            }

            k_mutex_lock(&data->lock, K_FOREVER);
            for (int f = 0; f < data->frame_num; f++) {
                rp_can_item_t *item = &data->can_items[f];

                /* frequency == 0：事件触发，不在定频线程中发送 */
                if (item->frequency == 0) {
                    continue;
                }

                item->tick_counter++;
                if (item->tick_counter < item->interval) {
                    continue;
                }
                item->tick_counter = 0;

                uint16_t tx_id = (uint16_t)item->frame.id;
                int ret = rp_can_tx_fillbuffer(tx_id, &item->frame, data);
                if (ret != 0) {
                    continue; /* 无回调或填充失败，跳过本帧 */
                }
                ret = can_send(cfg->can_dev, &item->frame, K_NO_WAIT,
                               can_tx_mgr_tx_cb, NULL);
                if (ret != 0) {
                    LOG_ERR("[can_tx_manager]Periodic can_send failed for tx_id 0x%03x, err %d", tx_id, ret);
                }
            }
            k_mutex_unlock(&data->lock);
        }
    }
}

K_THREAD_DEFINE(can_tx_mgr_thread, CONFIG_CAN_TX_MANAGER_THREAD_STACK_SIZE,
                can_tx_manager_thread, NULL, NULL, NULL,
                CONFIG_CAN_TX_MANAGER_THREAD_PRIORITY, 0, 0);


#define RP_CAN_TX_MGR_DEFINE(inst)                                                              \
    static const struct rp_can_tx_cfg rp_can_tx_mgr_cfg_##inst = {                      \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                               \
    };                                                                                          \
    static struct rp_can_tx_data rp_can_tx_mgr_data_##inst;                             \
    DEVICE_DT_INST_DEFINE(inst, rp_can_tx_manager_init, NULL, &rp_can_tx_mgr_data_##inst,       \
                          &rp_can_tx_mgr_cfg_##inst, POST_KERNEL, CONFIG_CAN_TX_MANAGER_INIT_PRIORITY, &rp_can_tx_mgr_api);

DT_INST_FOREACH_STATUS_OKAY(RP_CAN_TX_MGR_DEFINE)

