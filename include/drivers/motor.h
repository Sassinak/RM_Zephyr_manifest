#include <zephyr/device.h>
#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct sMotor_Heartbeat_Status_t
    {
        uint64_t heartbeat_tick; // 心跳时间戳
        bool is_alive;           // 心跳状态
    } sMotor_Heartbeat_Status_t;

    typedef struct sMotor_Receive_Data_t
    {
        uint8_t error_code;      // 错误码
        uint16_t zero_point;     // 零点校准值
        int16_t current;         // 电流值
        int16_t voltage;         // 电压值
        int16_t torque;          // 力矩值
        int16_t speed;           // 速度值
        int32_t angle;           // 编码器原始值
        int32_t posit;           // 相对零点位置
        uint8_t temp;            // 温度值
    } sMotor_Receive_Data_t;

    typedef struct sMotor_data_t
    {
        uint8_t *Tx_data;
        sMotor_Receive_Data_t Rx_data;
        sMotor_Heartbeat_Status_t heartbeat_status;
        void *interface_ptr;                            // 指向具体接口的指针
    } sMotor_data_t;

    /**
     * @typedef motor_api_register
     * @brief Callback API for register a motor device.
     * 
     */
    typedef int (*motor_api_register)(const struct device *dev);

    /**
     * @typedef motor_api_transfer
     * @brief transifer data by unique interface
     */
    typedef int (*motor_api_transfer)(const struct device *dev);

    /**
     * @typedef motor_api_get_heartbeat_status
     * @brief application/middleware get motor heartbeat status
     * 
     */
    typedef int (*motor_api_get_heartbeat_status)(const struct device *dev);


    typedef struct motor_driver_api_t
    {
        motor_api_register register_motor;
        motor_api_transfer transfer;
        motor_api_get_heartbeat_status get_heartbeat_status;
    } motor_driver_api_t;

    static inline int register_motor(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->register_motor == NULL) {
            return -ENOSYS;
        }
        return api->register_motor(dev);
    }

    static inline int get_motor_heartbeat_status(const struct device *dev)
    {
        const struct motor_driver_api_t *api = (const struct motor_driver_api_t *)dev->api;
        if(!api || api->get_heartbeat_status == NULL) {
            return -ENOSYS;
        }
        return api->get_heartbeat_status(dev);
    }

#ifdef __cplusplus
}
#endif
