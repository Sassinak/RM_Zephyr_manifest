#include <zephyr/device.h>
#include <math.h>
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

    typedef struct sMotor_Instance_t
    {
        uint8_t *Tx_data;
        sMotor_Receive_Data_t Rx_data;
        sMotor_Heartbeat_Status_t heartbeat_status;
        void *interface_ptr;                            // 指向具体接口的指针
        char motor_type;                                // 电机类型
        char motor_id;                                  // 电机ID
        struct sMotor_Instance_t *motor_next;           // 指向下一个电机实例
    } sMotor_Instance_t;

    /**
     * @typedef motor_api_register
     * @brief Callback API for register a motor device.
     * 
     */
    typedef int (*motor_api_register)(const struct device *dev,
                                      const sMotor_Instance_t *motor_);
    

    /**
     * @typedef motor_api_transfer
     * @brief transifer data by unique interface
     */
    typedef int (*motor_api_transfer)(const struct device *dev,
                                      const sMotor_Instance_t *motor_);
    

    /**
     * @typedef motor_api_get_heartbeat_status
     * @brief application/middleware get motor heartbeat status
     * 
     */
    typedef int (*motor_api_get_heartbeat_status)(const struct device *dev,
                                                  const char motor_id);
    
    /**
     * @typedef motor_api_update_recevie_data
     * @brief  application/middleware update motor receive data when motor register motor insrance will
     *          update automatically. this api is used for update receive data hardly
     * 
     */
    typedef int (*motor_api_update_receive_data)(const struct device *dev,
                                                 sMotor_Instance_t *motor_);

    /**
     * @typedef motor_api_calculate_baudrate
     * @brief calculate motor's interface baudrate 
     *
     */
    typedef int (*motor_api_calculate_baudrate)(const struct device *dev,
                                             sMotor_Instance_t *motor_);

#ifdef __cplusplus
}
#endif
