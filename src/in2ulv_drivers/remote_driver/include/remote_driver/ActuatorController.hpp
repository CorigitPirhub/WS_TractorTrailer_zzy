#ifndef STM32_COMMUNICATOR_HPP
#define STM32_COMMUNICATOR_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include "msgs_core/car_state.h"
#include "msgs_core/angles.h"
#include "utils_core/ros_utils.hpp"

namespace in2ulv_drivers {
namespace remote_driver {
/**
 * @brief STM32通信控制器
 * 
 * 负责处理与STM32的通信，发送控制指令，控制电磁锁、电推杆、舵机等
 */
class ActuatorController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param serial_port 共享的串口对象
     */
    ActuatorController(ros::NodeHandle& nh, serial::Serial& serial_port);
    
    /**
     * @brief 初始化控制器
     */
    void initialize();
    
    /**
     * @brief 发送数据到STM32
     */
    void sendData();
    
    // 状态更新函数
    void updateAngleError(float error);
    void updateLinkState(int state);
    void updateBehindLinkState(int state);
    void updateHBehindBYaw(float yaw);
    void updatePointBYaw(float yaw);
    void updateAngleSensor(float angle);

private:
    /**
     * @brief 发送给下位机(STM32)的数据结构体
     */
    struct SendData {
        uint8_t SOF1;               //!< 帧头1
        uint8_t SOF2;               //!< 帧头2
        uint8_t lock;               //!< 电磁锁控制
        uint8_t push_lock;          //!< 电推杆控制
        uint8_t servo_back;         //!< 前轴舵机回正标志位
        uint8_t servo_push;         //!< 组合时将锁头推紧的标志位
        uint8_t behind_servo_state; //!< 后铰链舵机控制的标志位
        float motor_angle_ref;      //!< 电机转速期望
        float sensor_angle;         //!< 传感器角度
    };
    
    /**
     * @brief 计算参考角度
     * @param point_b_yaw B点偏航角
     * @param h_behind_b_yaw 后车B点相对于本车H点的偏航角
     * @return 参考角度
     */
    float calculateAngleRef(float point_b_yaw, float h_behind_b_yaw);
    
private:
    ros::NodeHandle& nh_;            // ROS节点句柄引用
    serial::Serial& serial_port_;    // 串口对象引用
    SendData send_data_;             // 要发送的数据
    
    // 状态变量
    float current_angle_error_ = 0.0;
    int link_state_ = -1;
    int behind_link_state_ = 0;
    float h_behind_b_yaw_ = 0;
    float point_b_yaw_ = 0;
    float angle_sensor_ = -999;
    float ang_dead_band_ = 0.45;
    int lock_angle_hit_ = 0;
};
 
}  // namespace remote_driver
}  // namespace in2ulv_drivers

#endif  // STM32_COMMUNICATOR_HPP