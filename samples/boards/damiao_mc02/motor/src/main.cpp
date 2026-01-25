#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <drivers/motor.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);


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
        const sMotor_Receive_Data_t *fl = get_motor_rxdata(motor_fl);
        const sMotor_Receive_Data_t *fr = get_motor_rxdata(motor_fr);
        // LOG_INF("motor_fl: heartbeat is_alive=%d", get_motor_heartbeat_status(motor_fl));
        // LOG_INF("motor_fr: heartbeat is_alive=%d", get_motor_heartbeat_status(motor_fr));

        LOG_INF("FL angle=%d speed=%d current=%d alive=%d temp=%d",
            (int)fl->angle, (int)fl->speed, (int)fl->current,
            get_motor_heartbeat_status(motor_fl) ? 1 : 0, (int)fl->specific_data.m3508.temp);
        LOG_INF("FR angle=%d speed=%d current=%d alive=%d temp=%d",
                (int)fr->angle, (int)fr->speed, (int)fr->current,
                get_motor_heartbeat_status(motor_fr) ? 1 : 0, (int)fr->specific_data.m3508.temp);

        /* Yield so the log processing thread and RTT backend can flush. */
        k_sleep(K_MSEC(500));
    }

    return 0;
}
