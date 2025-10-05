#include "remote_driver/ActuatorController.hpp"
#include <iomanip>

namespace in2ulv_drivers {
namespace remote_driver {
ActuatorController::ActuatorController(ros::NodeHandle& nh, serial::Serial& serial_port)
    : nh_(nh), serial_port_(serial_port) {
    ROS_INFO("Initializing ActuatorController...");
    
    // 从参数服务器获取角度死区
    if (in2ulv_cores::utils_core::getParamWithFullName(nh_, "chassis_ctl_param/ang_dead_band", ang_dead_band_)) {
        ROS_INFO("Chassis_ctl_param/ang_dead_band: %f", ang_dead_band_);
    }
    ang_dead_band_ = 5 * ang_dead_band_;
    ROS_INFO("Angle dead band set to: %.3f", ang_dead_band_);
}

void ActuatorController::initialize() {
    // 初始化发送数据结构
    send_data_.SOF1 = 0xa5;
    send_data_.SOF2 = 0x5a;
    
    ROS_INFO("ActuatorController initialized");
}

void ActuatorController::sendData() {
    ROS_DEBUG_THROTTLE(5.0, "Preparing data for STM32");
    
    // 设置电磁锁控制为后车连接状态
    send_data_.lock = behind_link_state_;
    
    // 设置后铰链舵机状态
    send_data_.behind_servo_state = std::abs(behind_link_state_);
    
    // 根据角度误差和连接状态判断是否满足锁止条件
    if (std::abs(current_angle_error_) < ang_dead_band_ && (link_state_ == 1 || link_state_ == -1)) {
        lock_angle_hit_++;
        if (lock_angle_hit_ >= 3) {
            send_data_.push_lock = 0x01; // 满足条件，设置电推杆控制为锁止
            ROS_DEBUG_THROTTLE(5.0, "Lock condition met: push_lock set to 1");
        }
    } else if (link_state_ == 0) {
        send_data_.push_lock = 0x00; // 已连接，设置电推杆控制为解锁
        lock_angle_hit_ = 0;
        ROS_DEBUG_THROTTLE(5.0, "Link state 0: push_lock set to 0");
    } else {
        lock_angle_hit_ = 0;
        ROS_DEBUG_THROTTLE(5.0, "Lock condition not met: resetting lock_angle_hit");
    }
    
    // 设置舵机推杆控制和回正标志位
    send_data_.servo_push = 1 - std::abs(link_state_);
    send_data_.servo_back = std::abs(link_state_);
    
    // 设置电机转速期望
    send_data_.motor_angle_ref = calculateAngleRef(point_b_yaw_, h_behind_b_yaw_);
    
    // 设置传感器角度
    send_data_.sensor_angle = angle_sensor_;
    
    // 打印关键数据（节流：每5秒最多打印一次）
    ROS_DEBUG_THROTTLE(5.0, "Sending to STM32: "
                      "lock=%d, push_lock=%d, servo_back=%d, servo_push=%d, "
                      "behind_servo_state=%d, motor_angle_ref=%.3f, sensor_angle=%.3f",
                      send_data_.lock, send_data_.push_lock, send_data_.servo_back,
                      send_data_.servo_push, send_data_.behind_servo_state,
                      send_data_.motor_angle_ref, send_data_.sensor_angle);
    
    // 打印原始发送数据（节流：每5秒最多打印一次）
    static ros::Time last_raw_print_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    if ((now - last_raw_print_time).toSec() > 5.0) {
        last_raw_print_time = now;
        
        std::stringstream raw_ss;
        raw_ss << "Raw TX (" << sizeof(SendData) << " bytes): ";
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&send_data_);
        for (size_t i = 0; i < sizeof(SendData); ++i) {
            raw_ss << std::hex << std::setw(2) << std::setfill('0') 
                   << static_cast<int>(data_ptr[i]) << " ";
        }
        ROS_DEBUG_STREAM(raw_ss.str());
    }
    
    // 通过串口发送数据
    if (serial_port_.isOpen()) {
        serial_port_.write(reinterpret_cast<uint8_t*>(&send_data_), sizeof(send_data_));
    }
}

float ActuatorController::calculateAngleRef(float point_b_yaw, float h_behind_b_yaw) {
    float temp_ref = h_behind_b_yaw;
    
    // 将角度限制在 -180 到 180 度之间
    while (temp_ref > 180) temp_ref -= 360;
    while (temp_ref < -180) temp_ref += 360;
    
    // 将角度进一步限制在 -60 到 60 度之间
    float result = std::clamp(temp_ref, -60.0f, 60.0f);
    
    ROS_DEBUG_THROTTLE(5.0, "Calculated angle ref: input=%.3f, output=%.3f", h_behind_b_yaw, result);
    return result;
}

// 状态更新函数
void ActuatorController::updateAngleError(float error) {
    current_angle_error_ = error;
    ROS_DEBUG_THROTTLE(5.0, "Angle error updated: %.3f", error);
}

void ActuatorController::updateLinkState(int state) {
    link_state_ = state;
    ROS_DEBUG_THROTTLE(5.0, "Link state updated: %d", state);
}

void ActuatorController::updateBehindLinkState(int state) {
    behind_link_state_ = state;
    ROS_DEBUG_THROTTLE(5.0, "Behind link state updated: %d", state);
}

void ActuatorController::updateHBehindBYaw(float yaw) {
    h_behind_b_yaw_ = yaw;
    ROS_DEBUG_THROTTLE(5.0, "H behind B yaw updated: %.3f", yaw);
}

void ActuatorController::updatePointBYaw(float yaw) {
    point_b_yaw_ = yaw;
    ROS_DEBUG_THROTTLE(5.0, "Point B yaw updated: %.3f", yaw);
}

void ActuatorController::updateAngleSensor(float angle) {
    angle_sensor_ = angle;
    ROS_DEBUG_THROTTLE(5.0, "Angle sensor updated: %.3f", angle);
}
}  // namespace remote_driver
}  // namespace in2ulv_drivers