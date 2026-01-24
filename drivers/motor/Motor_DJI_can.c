/*
 * Copyright (c) 2025 Sassinak
 * SPDX-License-Identifier: Apache-2.0
 * author: Sassinak
 */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT rp_dji_can_motor

#include <drivers/motor.h>

#define LOG_LEVEL CONFIG_MOTOR_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_dji_can);

#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include <errno.h>
#include <string.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

#include <drivers/can_rx_manager.h>

/*
 * motor-id: DTS string -> const char*
 * control-mode: DTS enum -> DT_ENUM_IDX
 */
typedef struct motor_dji_cfg_t {
    uint16_t tx_id;
    uint16_t rx_id;
    const char *motor_id;
    int8_t control_mode;
    const struct device *can_dev;
    const struct device *rx_mgr;
} motor_dji_cfg_t;

typedef struct motor_dji_data_t
{
    sMotor_data_t motor_data;
    bool registered;
    int rx_filter_id;
} motor_dji_data_t;


static void motor_dji_can_rx_handler(const struct can_frame *frame, void *user_data)
{
    motor_dji_data_t *motor = (motor_dji_data_t *)user_data;
    if ((motor == NULL) || (frame == NULL)) {
        LOG_ERR("[dji_motor_err] rx handle Invalid arguments");
        return;
    }

    /* manager 已默认跳过 RTR，这里再做一次保护 */
    if ((frame->flags & CAN_FRAME_RTR) != 0U) {
        LOG_ERR("[dji_motor_err] RTR frame received");
        return;
    }

    motor->motor_data.Rx_data.angle   = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
    motor->motor_data.Rx_data.speed   = (int16_t)((frame->data[2] << 8)  | frame->data[3]);
    motor->motor_data.Rx_data.current = (int16_t)((frame->data[4] << 8)  | frame->data[5]);

    motor->motor_data.heartbeat_status.heartbeat_tick = (uint64_t)k_uptime_get();
    motor->motor_data.heartbeat_status.is_alive = true;
}


/**
 * @brief 单电机注册接口，通过 motor_id 识别电机实例，配置了can的接收过滤器
 * 
 * @param dev 
 * @return int 
 */
static int motor_dji_can_register_motor(const struct device *dev)
{
    const motor_dji_cfg_t *cfg = dev->config;
    motor_dji_data_t *data = dev->data;

    if ((cfg == NULL) || (data == NULL)) {
        LOG_ERR("[dji_motor_err] register Invalid arguments");
        return -EINVAL;
    }

    if (!device_is_ready(cfg->can_dev)) {
        LOG_ERR("[dji_motor_err] CAN device not ready");
        return -ENODEV;
    }

    if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) {
        LOG_ERR("[dji_motor_err] RX manager not ready");
        return -ENODEV;
    }

    if (data->registered) {
        return -EALREADY;
    }

    struct can_filter filter = {
        .id = cfg->rx_id & CAN_STD_ID_MASK,
        .mask = CAN_STD_ID_MASK,
        .flags = 0,
    };

    int ret = rp_can_rx_manager_register(cfg->rx_mgr, &filter, motor_dji_can_rx_handler, data);
    if (ret < 0) {
        LOG_ERR("[dji_motor_err] Failed to register CAN RX filter");
        return ret;
    }

    data->registered = true;
    data->rx_filter_id = ret;
    data->motor_data.Tx_data = 0;
    data->motor_data.interface_ptr = (void *)cfg;
    data->motor_data.heartbeat_status.is_alive = false;
    data->motor_data.heartbeat_status.heartbeat_tick = 0;

    return 0;
}


static int motor_dji_can_transfer(const struct sMotor_data_t *motor_)
{
    ARG_UNUSED(motor_);
    return -ENOSYS;
}


static int motor_dji_can_get_heartbeat_status(const char *motor_id)
{
    ARG_UNUSED(motor_id);
    return -ENOSYS;
}


static int motor_dji_can_update_receive_data(const struct sMotor_data_t *motor_)
{
    ARG_UNUSED(motor_);
    return -ENOSYS;
}


static int motor_dji_can_calculate_baudrate(const struct sMotor_data_t *motor_)
{
    ARG_UNUSED(motor_);
    return -ENOSYS;
}


static const motor_driver_api_t motor_dji_can_api = {
    .register_motor = motor_dji_can_register_motor,
    .transfer = motor_dji_can_transfer,
    .get_heartbeat_status = motor_dji_can_get_heartbeat_status,
    .update_receive_data = motor_dji_can_update_receive_data,
    .calculate_baudrate = motor_dji_can_calculate_baudrate,
};


/* ---------- Devicetree helpers ---------- */

/* 获取 control-mode enum 的索引；未配置则返回 -1 */
#define MOTOR_DJI_CONTROL_MODE(inst) \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, control_mode), (DT_INST_ENUM_IDX(inst, control_mode)), (-1))

#define MOTOR_DJI_DEFINE(inst) \
    static const motor_dji_cfg_t motor_dji_cfg_##inst = { \
        .tx_id = (uint16_t)DT_INST_PROP(inst, tx_id), \
        .rx_id = (uint16_t)DT_INST_PROP(inst, rx_id), \
        .motor_id = DT_INST_PROP(inst, motor_id), \
        .control_mode = (int8_t)MOTOR_DJI_CONTROL_MODE(inst), \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)), \
        .rx_mgr = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rx_manager), \
                      (DEVICE_DT_GET(DT_INST_PHANDLE(inst, rx_manager))), (NULL)), \
    }; \
    static motor_dji_data_t motor_dji_data_##inst; \
    static int motor_dji_can_init_##inst(const struct device *dev) \
    { \
        const motor_dji_cfg_t *cfg = dev->config; \
        motor_dji_data_t *data = dev->data; \
        if (!device_is_ready(cfg->can_dev)) { \
            return -ENODEV; \
        } \
        if ((cfg->rx_mgr == NULL) || !device_is_ready(cfg->rx_mgr)) { \
            return -ENODEV; \
        } \
        data->registered = false; \
        data->rx_filter_id = -1; \
        memset(&data->motor_data, 0, sizeof(data->motor_data)); \
        data->motor_data.interface_ptr = (void *)cfg; \
        data->motor_data.heartbeat_status.is_alive = false; \
        data->motor_data.heartbeat_status.heartbeat_tick = 0; \
        return 0; \
    } \
    DEVICE_DT_INST_DEFINE(inst, motor_dji_can_init_##inst, NULL, &motor_dji_data_##inst, \
                  &motor_dji_cfg_##inst, POST_KERNEL, CONFIG_MOTOR_INIT_PRIORITY, \
                  &motor_dji_can_api);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(MOTOR_DJI_DEFINE)
#endif
