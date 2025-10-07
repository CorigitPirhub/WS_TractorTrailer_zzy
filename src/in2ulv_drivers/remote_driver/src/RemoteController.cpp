#include "remote_driver/RemoteController.hpp"
#include "remote_driver/ActuatorController.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <iomanip>

namespace in2ulv_drivers {
namespace remote_driver {

RemoteController::RemoteController(ros::NodeHandle& nh) 
    : nh_(nh), port_("/dev/ttyACM1"), baudrate_(115200), ang_dead_band_(0.45), stm32_comm_(nullptr), 
    monitor_agent_(nh, "remote_driver") {
    
    ROS_INFO("Initializing RemoteController...");
    
    // 初始化发布者和订阅者
    remote_pub_ = nh_.advertise<msgs_core::remote_info>("FS_remote_info", 10);
    error_angle_sub_ = nh_.subscribe("angles", 5, &RemoteController::encoderCallback, this);
    car_state_sub_ = nh_.subscribe("car_state", 5, &RemoteController::carStateCallback, this);
    point_b_sub_ = nh_.subscribe("Point_B", 5, &RemoteController::pointBCallback, this);
    
    ROS_INFO("Subscribers initialized");
    
    // 动态订阅配置
    std::string behind_chassis;
    if (in2ulv_cores::utils_core::getParamWithFullName(nh_, "common/behind_chassis", behind_chassis)) {
        ROS_INFO("common/behind_chassis: %s", behind_chassis.c_str());
        std::string topic_name;
        topic_name = "/" + behind_chassis + "/H_And_B_yaw";
        h_and_b_yaw_sub_ = nh_.subscribe(topic_name, 5, &RemoteController::hAndBYawCallback, this);
        
        topic_name = "/" + behind_chassis + "/car_state";
        behind_state_sub_ = nh_.subscribe(topic_name, 5, &RemoteController::behindCarStateCallback, this);
        
        ROS_INFO("Behind chassis topics subscribed");
    }
    
    std::string car_type;
    if (in2ulv_cores::utils_core::getParamWithFullName(nh_, "common/chassis_type", car_type)) {
        ROS_INFO("common/chassis_type: %s", car_type.c_str());
        if (car_type != "Head") {
            std::string follow_chassis;
            if (in2ulv_cores::utils_core::getParamWithFullName(nh_, "common/follow_chassis", follow_chassis)) {
                ROS_INFO("follow_chassis: %s", follow_chassis.c_str());
                std::string topic_name = "/" + follow_chassis + "/FS_remote_info";
                fs_remote_info_sub_ = nh_.subscribe(topic_name, 5, &RemoteController::fsRemoteInfoCallback, this);
                ROS_INFO("Front vehicle remote info subscribed");
            }
        }
    }
    
    if(in2ulv_cores::utils_core::getParamWithFullName(nh_, "chassis_ctl_param/ang_dead_band", ang_dead_band_)) {
        ROS_INFO("Chassis_ctl_param/ang_dead_band: %f", ang_dead_band_);
    }
    ang_dead_band_ = 5 * ang_dead_band_; // 乘以10,为电推杆预留响应时间
    ROS_INFO("Angle dead band set to: %.3f", ang_dead_band_);
}

bool RemoteController::initialize() {
    ROS_INFO("Initializing serial port: %s at %d baud", port_.c_str(), baudrate_);
    
    // 设置串口参数
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial_port_.setPort(port_);
    serial_port_.setBaudrate(baudrate_);
    serial_port_.setTimeout(to);
    
    try {
        serial_port_.open();
        if (serial_port_.isOpen()) {
            ROS_INFO_STREAM("Remote controller port opened: " << port_);
            return true;
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open remote port: " << e.what());
    }
    return false;
}

void RemoteController::shutdown() {
    if (serial_port_.isOpen()) {
        ROS_INFO("Closing serial port");
        serial_port_.close();
    }
}

void RemoteController::setActuatorController(ActuatorController* stm32_comm) {
    stm32_comm_ = stm32_comm;
    ROS_INFO("ActuatorController set");
}

void RemoteController::run() {
    ros::Rate loop_rate(100); // 100 Hz
    ROS_INFO("Entering main loop");

    ros::Time last_metric_time = ros::Time::now();

    while (ros::ok()) {
        // 接收数据
        size_t num = serial_port_.available();
        if (num > 0) {
            ROS_DEBUG_THROTTLE(5.0, "Serial data available: %lu bytes", num);
            processResponse();
        }
        
        // 发送数据
        if (stm32_comm_ && angle_sensor_ > -500) {
            stm32_comm_->sendData();
        }
        
        // 定期发布监控指标（每秒一次）
        if ((ros::Time::now() - last_metric_time).toSec() >= 1.0) {
            monitor_agent_.publishMetrics();
            last_metric_time = ros::Time::now();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RemoteController::processResponse() {
    uint8_t buffer[256];
    size_t num = serial_port_.read(buffer, serial_port_.available());
    
    ROS_DEBUG_THROTTLE(5.0, "Read %lu bytes from serial", num);
    
    // 打印原始接收数据（节流：每5秒最多打印一次）
    if (num > 0) {
        static ros::Time last_raw_print_time = ros::Time::now();
        ros::Time now = ros::Time::now();
        if ((now - last_raw_print_time).toSec() > 5.0) {
            last_raw_print_time = now;
            
            std::stringstream raw_ss;
            raw_ss << "Raw RX (" << num << " bytes): ";
            for (size_t i = 0; i < num; ++i) {
                raw_ss << std::hex << std::setw(2) << std::setfill('0') 
                       << static_cast<int>(buffer[i]) << " ";
            }
            ROS_DEBUG_STREAM(raw_ss.str());
        }
    }
    
    // 处理数据堆积情况
    uint8_t* process_buffer = buffer;
    size_t process_num = num;
    
    if (num % sizeof(StateData) == 0 && num > sizeof(StateData)) {
        ROS_WARN_THROTTLE(5.0, "Received stacked data (size: %lu bytes). Processing last packet.", num);
        process_buffer = buffer + (num - sizeof(StateData));
        process_num = sizeof(StateData);
    }
    
    if (process_num == sizeof(StateData) && process_buffer[0] == 0x5A && process_buffer[1] == 0xA5) {
        memcpy(&state_data_, process_buffer, sizeof(StateData));
        remote_info_msg_.ch.resize(10);
        for (int i = 0; i < 10; i++) {
            remote_info_msg_.ch[i] = state_data_.ch[i];
        }
        remote_info_msg_.lock = state_data_.connect;
        in2ulv_cores::utils_core::safePublish(remote_pub_, remote_info_msg_);
        monitor_agent_.recordPublish("FS_remote_info"); // 记录发布事件
        
        // 使用静态变量控制打印频率，每5秒最多打印一次
        static ros::Time last_print_time = ros::Time::now();
        ros::Time now = ros::Time::now();
        if ((now - last_print_time).toSec() > 5.0) {
            last_print_time = now;
            
            ROS_DEBUG("Received valid remote data: Connect=%d", state_data_.connect);
            for (int i = 0; i < 10; i++) {
                ROS_DEBUG("  CH%d: %d", i, state_data_.ch[i]);
            }
        }
    } else {
        ROS_WARN_THROTTLE(5.0, "Received invalid data. Expected size: %lu, Received Size: %lu", 
                         sizeof(StateData), num);
    }
}

// 回调函数实现
void RemoteController::encoderCallback(const msgs_core::angles::ConstPtr& msg) {
    current_angle_error_ = -msg->angle_f;
    angle_sensor_ = msg->angle_r;
    
    ROS_DEBUG_THROTTLE(5.0, "Encoder callback: angle_error=%.3f, sensor_angle=%.3f", 
                      current_angle_error_, angle_sensor_);
    
    if (stm32_comm_) {
        stm32_comm_->updateAngleError(current_angle_error_);
        stm32_comm_->updateAngleSensor(angle_sensor_);
    }
}

void RemoteController::carStateCallback(const msgs_core::car_state::ConstPtr& msg) {
    link_state_ = msg->linked;
    
    ROS_DEBUG_THROTTLE(5.0, "Car state callback: link_state=%d", link_state_);
    
    if (stm32_comm_) {
        stm32_comm_->updateLinkState(link_state_);
    }
}

void RemoteController::pointBCallback(const msgs_core::car_state::ConstPtr& msg) {
    point_b_yaw_ = msg->yaw;
    
    ROS_DEBUG_THROTTLE(5.0, "Point B callback: yaw=%.3f", point_b_yaw_);
    
    if (stm32_comm_) {
        stm32_comm_->updatePointBYaw(point_b_yaw_);
    }
}

void RemoteController::hAndBYawCallback(const msgs_core::car_state::ConstPtr& msg) {
    h_behind_b_yaw_ = msg->H_behind_B_yaw;
    
    ROS_DEBUG_THROTTLE(5.0, "H and B yaw callback: yaw=%.3f", h_behind_b_yaw_);
    
    if (stm32_comm_) {
        stm32_comm_->updateHBehindBYaw(h_behind_b_yaw_);
    }
}

void RemoteController::behindCarStateCallback(const msgs_core::car_state::ConstPtr& msg) {
    behind_link_state_ = msg->linked;
    
    ROS_DEBUG_THROTTLE(5.0, "Behind car state callback: link_state=%d", behind_link_state_);
    
    if (stm32_comm_) {
        stm32_comm_->updateBehindLinkState(behind_link_state_);
    }
}

void RemoteController::fsRemoteInfoCallback(const msgs_core::remote_info::ConstPtr& msg) {
    front_lock_state_ = msg->lock;
    
    ROS_DEBUG_THROTTLE(5.0, "Front remote info callback: lock=%d", front_lock_state_);
}
}  // namespace remote_driver
}  // namespace in2ulv_drivers