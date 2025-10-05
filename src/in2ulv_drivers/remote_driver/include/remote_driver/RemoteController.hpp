#ifndef REMOTE_CONTROLLER_HPP
#define REMOTE_CONTROLLER_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include "msgs_core/remote_info.h"
#include "msgs_core/car_state.h"
#include "msgs_core/angles.h"
#include "utils_core/ros_utils.hpp"

namespace in2ulv_drivers {
namespace remote_driver {

// 前向声明
class ActuatorController;

/**
 * @brief 飞思遥控器控制器
 * 
 * 负责处理与飞思遥控器的通信，接收遥控器数据并发布为ROS话题
 */
class RemoteController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    RemoteController(ros::NodeHandle& nh);
    
    /**
     * @brief 初始化控制器，打开串口连接
     * @return 初始化是否成功
     */
    bool initialize();
    
    /**
     * @brief 运行主循环
     */
    void run();
    
    /**
     * @brief 关闭串口连接
     */
    void shutdown();

    /**
     * @brief 获取串口对象引用
     * @return 串口对象引用
     */
    serial::Serial& getSerialPort() { return serial_port_; }
    
    /**
     * @brief 设置STM32通信器
     * @param stm32_comm STM32通信器对象
     */
    void setActuatorController(ActuatorController* stm32_comm);

private:
    /**
     * @brief 接收来自飞思遥控器的数据结构体
     */
    struct StateData {
        uint8_t SOF1;               //!< 帧头1
        uint8_t SOF2;               //!< 帧头2
        uint16_t connect;           //!< 连接状态
        int16_t ch[10];             //!< 10个通道的数据
    };
    
    /**
     * @brief 处理传感器响应数据
     */
    void processResponse();
    
    // 回调函数
    void encoderCallback(const msgs_core::angles::ConstPtr& msg);
    void carStateCallback(const msgs_core::car_state::ConstPtr& msg);
    void pointBCallback(const msgs_core::car_state::ConstPtr& msg);
    void hAndBYawCallback(const msgs_core::car_state::ConstPtr& msg);
    void behindCarStateCallback(const msgs_core::car_state::ConstPtr& msg);
    void fsRemoteInfoCallback(const msgs_core::remote_info::ConstPtr& msg);
    
private:
    ros::NodeHandle nh_;                   // ROS节点句柄
    serial::Serial serial_port_;           // 串口对象
    ros::Publisher remote_pub_;            // 遥控器信息发布者
    ros::Subscriber error_angle_sub_;      // 角度误差订阅者
    ros::Subscriber car_state_sub_;        // 车辆状态订阅者
    ros::Subscriber point_b_sub_;          // B点偏航角订阅者
    ros::Subscriber h_and_b_yaw_sub_;      // H点和B点偏航角订阅者
    ros::Subscriber behind_state_sub_;     // 后车状态订阅者
    ros::Subscriber fs_remote_info_sub_;   // 前车遥控器信息订阅者
    
    StateData state_data_;                  // 接收到的遥控器数据
    msgs_core::remote_info remote_info_msg_; // 遥控器信息消息
    
    std::string port_;                      // 串口设备路径
    int baudrate_;                          // 波特率
    float ang_dead_band_;                   // 角度死区
    
    // 状态变量
    float current_angle_error_ = 0.0;       // 当前角度误差
    int link_state_ = -1;                   // 连接状态
    int behind_link_state_ = 0;              // 后车连接状态
    float h_behind_b_yaw_ = 0;              // 后车B点相对于本车H点的偏航角
    float point_b_yaw_ = 0;                 // B点偏航角
    float angle_sensor_ = -999;             // 传感器角度
    int front_lock_state_ = 0;              // 前车锁状态

    ActuatorController* stm32_comm_;         // STM32通信器对象
};
    
}  // namespace remote_driver
}  // namespace in2ulv_drivers

#endif  // REMOTE_CONTROLLER_HPP