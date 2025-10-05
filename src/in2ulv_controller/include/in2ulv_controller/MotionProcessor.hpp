#ifndef MOTION_PROCESSOR_HPP
#define MOTION_PROCESSOR_HPP

#include "ros/ros.h"
#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <cmath>
#include <string>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <utility>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <functional>

#include "msgs_core/ctrl_cmd.h"
#include "msgs_core/torque_cmd.h"
#include "msgs_core/angle_error.h"
#include "msgs_core/ackermann_cmd.h"
#include "msgs_core/car_state.h"
#include "msgs_core/angles.h" 
#include "msgs_core/lr_wheel_fb.h"
#include "msgs_core/rr_wheel_fb.h"

#include <dynamic_reconfigure/server.h>

#include "in2ulv_controller/PIDControlConfig.h"

#include "utils_core/automotive_pid.hpp"
#include "utils_core/angle_utils.hpp"
#include "utils_core/spline_interpolator.hpp"
#include "utils_core/ros_utils.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace in2ulv_controller {

#define REFERENCE_K 1.0f  
#define PI 3.14159f

/**
 * @class MotionProcessor
 * @brief 运动处理核心类
 * 
 * 负责处理车辆运动控制的核心算法，包括：
 * - 接收运动指令和车辆状态
 * - 执行PID控制算法
 * - 生成扭矩控制指令
 */
class MotionProcessor {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit MotionProcessor(ros::NodeHandle& nh);
    
    /**
     * @brief 计算控制值
     */
    void calculateControlValues();
    
    /**
     * @brief 发布控制指令
     */
    void publishControlCommands();

private:
    /**
     * @brief 运动期望结构体
     */
    struct MotionExpectations {
        float velocity;
        float steering;
    };
    
    // 动态参数配置回调
    void reconfigureCallback(in2ulv_controller::PIDControlConfig& config, uint32_t level);
    
    // ROS消息回调
    void ackermannCmdCallback(const msgs_core::ackermann_cmd::ConstPtr& msg);
    void carStateCallback(const msgs_core::car_state::ConstPtr& msg);
    void angleCallback(const msgs_core::angles::ConstPtr& msg);
    void leftWheelCallback(const msgs_core::lr_wheel_fb::ConstPtr& msg);
    void rightWheelCallback(const msgs_core::rr_wheel_fb::ConstPtr& msg);
    
    // 更新运动期望
    void updateMotionExpectations(const MotionExpectations& expectations);
    
    // 车辆状态参数
    int link_state_;
    int manual_mode_;
    
    // 传感器反馈
    float left_wheel_velocity_;
    float right_wheel_velocity_;
    float front_axle_angle_;
    
    // 控制指令
    float cmd_velocity_;
    float cmd_velocity_last_;
    float cmd_left_velocity_;
    float cmd_right_velocity_;
    float cmd_steering_;
    int cmd_gear_;
    float cmd_left_torque_;
    float cmd_right_torque_;
    
    // 配置参数
    bool motor_positive_direction_;
    float current_angle_;
    float measured_velocity_;
    float measured_velocity_diff_;
    
    // 时间管理
    ros::Time last_update_time_;
    
    // PID控制器
    in2ulv_cores::utils_core::AutomotivePID angle_correction_pid_;
    in2ulv_cores::utils_core::AutomotivePID left_wheel_pid_;
    in2ulv_cores::utils_core::AutomotivePID right_wheel_pid_;
    
    // PID参数
    float angle_kp_;
    float angle_ki_;
    float angle_deadband_;
    float left_wheel_kp_;
    float right_wheel_kp_;
    
    // 动态参数配置
    dynamic_reconfigure::Server<in2ulv_controller::PIDControlConfig> dyn_reconf_server_;
    bool pid_tuning_enabled_;
    
    // ROS接口
    ros::Subscriber ackermann_cmd_sub_;
    ros::Subscriber car_state_sub_;
    ros::Subscriber angle_sub_;
    ros::Subscriber left_wheel_fb_sub_;
    ros::Subscriber right_wheel_fb_sub_;
    ros::Publisher ctrl_cmd_pub_;
    ros::Publisher torque_cmd_pub_;
    ros::Publisher angle_error_pub_;
};

} // namespace in2ulv_controller

#endif // MOTION_PROCESSOR_HPP