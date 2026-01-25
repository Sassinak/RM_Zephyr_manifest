#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <drivers/motor.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);


static const sMotor_data_t *motor_data_from_device(const struct device *dev)
{
    /*
     * Motor_DJI_can.c: dev->data 的首成员就是 sMotor_data_t motor_data;
     * 因此这里可以把 dev->data 视作 sMotor_data_t* 来读取 Rx_data。
     */
    return (const sMotor_data_t *)dev->data;
}

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

    register_motor(motor_fl);
    register_motor(motor_fr);

    while (true) {
        const sMotor_data_t *fl = motor_data_from_device(motor_fl);
        const sMotor_data_t *fr = motor_data_from_device(motor_fr);
        // LOG_INF("motor_fl: heartbeat is_alive=%d", get_motor_heartbeat_status(motor_fl));
        // LOG_INF("motor_fr: heartbeat is_alive=%d", get_motor_heartbeat_status(motor_fr));

        LOG_INF("FL angle=%d speed=%d current=%d alive=%d",
            (int)fl->Rx_data.angle, (int)fl->Rx_data.speed, (int)fl->Rx_data.current,
            fl->heartbeat_status.is_alive ? 1 : 0);
        LOG_INF("FR angle=%d speed=%d current=%d alive=%d",
            (int)fr->Rx_data.angle, (int)fr->Rx_data.speed, (int)fr->Rx_data.current,
            fr->heartbeat_status.is_alive ? 1 : 0);

        /* Yield so the log processing thread and RTT backend can flush. */
        k_sleep(K_MSEC(500));
    }

    return 0;
}
