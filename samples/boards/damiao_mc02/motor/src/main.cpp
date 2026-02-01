#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <drivers/motor.h>
#include <drivers/can_tx_manager.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

typedef struct motor_dji_cfg_t
{
    uint16_t tx_id;
    uint16_t rx_id;
    const char *motor_id;
    int8_t motor_type;
    int8_t control_mode;
    uint16_t motor_encoder;
    uint8_t transmission_ratio;
    const struct device *can_dev;
    const struct device *rx_mgr;
} motor_dji_cfg_t;

typedef struct motor_dji_data_t
{
    sMotor_data_t motor_data;
    struct k_spinlock lock; // 保护 motor_data 的自旋锁，防止接收更新和心跳检测冲突
    bool registered;
    int rx_filter_id; // CAN RX管理器 ID
    int tx_filter_id; // CAN TX管理器 ID
#if defined(CONFIG_MOTOR_DJI_HEARTBEAT_AUTOCHECK)
    const struct device *dev_self;   // 指向自身设备的指针，用于心跳自动检测
    struct k_work_delayable hb_work; // 心跳自动检测的延时工作
#endif
} motor_dji_data_t;

int main(void)
{
    LOG_INF("[app] start");
    const struct device *motor_fl = DEVICE_DT_GET(DT_NODELABEL(chassis_fl));
    const struct device *motor_fr = DEVICE_DT_GET(DT_NODELABEL(chassis_fr));


    if (!device_is_ready(motor_fl)) {
        LOG_ERR("motor FL not ready: %s", motor_fl->name);
        return -ENODEV;
    }
    if (!device_is_ready(motor_fr)) {
        LOG_ERR("motor FR not ready: %s", motor_fr->name);
        return -ENODEV;
    }

    int current = 1000;

    register_motor(motor_fl);
    register_motor(motor_fr);

    while (true) {
        const sMotor_Receive_Data_t *fl = get_motor_rxdata(motor_fl);
        const sMotor_Receive_Data_t *fr = get_motor_rxdata(motor_fr);
        const motor_dji_cfg_t *cfg_fl = (const motor_dji_cfg_t *)motor_fl->config;
        const motor_dji_cfg_t *cfg_fr = (const motor_dji_cfg_t *)motor_fr->config;
        const motor_dji_data_t *data_fl = (const motor_dji_data_t *)motor_fl->data;
        const motor_dji_data_t *data_fr = (const motor_dji_data_t *)motor_fr->data;
        
        // 接收函数检查
        LOG_INF("FL angle=%d speed=%d current=%d alive=%d temp=%d",
                    (int)fl->angle, (int)fl->speed, (int)fl->current,
                    get_motor_heartbeat_status(motor_fl) ? 1 : 0, (int)fl->specific_data.m3508.temp);
        LOG_INF("FR angle=%d speed=%d current=%d alive=%d temp=%d",
                (int)fr->angle, (int)fr->speed, (int)fr->current,
                get_motor_heartbeat_status(motor_fr) ? 1 : 0, (int)fr->specific_data.m3508.temp);
        // 发送函数检查
        motor_update_serialized(motor_fl, current);
        motor_update_serialized(motor_fr, current);
        LOG_INF("real send tx_data: %d", (data_fl->motor_data.Tx_data[0]<<8 | data_fl->motor_data.Tx_data[1]));
        can_tx_manager_send(cfg_fl->can_dev, K_MSEC(10), NULL,
                            cfg_fl->tx_id, NULL);
        

        /* Yield so the log processing thread and RTT backend can flush. */
        k_sleep(K_MSEC(500));
    }

    return 0;
}
