#include "in2ulv_controller/ChassisCmdGenerator.hpp"

namespace in2ulv_controller {
TractorCommandInput::TractorCommandInput(ros::NodeHandle& nh) 
    : nh_(nh), remote_watchdog_(0), linked_mode_(-1), manual_mode_(-1), 
      speed_mode_(-1), is_follow_same_traj_(-1), remote_v0_(0.0f), 
      remote_steering_(0.0f), steering_tra_(0.0f), self_point_b_yaw_(0.0f),
      v0_(0.0f), steering_(0.0f) {
    
    // 获取参数
    remote_offset_v0_ = in2ulv_cores::utils_core::getParamWithCheck<float>(nh_, "remote_offset/offset_v0", 0.0f);
    remote_offset_steering_ = in2ulv_cores::utils_core::getParamWithCheck<float>(nh_, "remote_offset/offset_steering", 0.0f);
    
    // 初始化订阅者和发布者
    remote_info_sub_ = nh_.subscribe("FS_remote_info", 10, 
                                    &TractorCommandInput::remoteCmdCallback, this);
    ackermann_cmd_tra_sub_ = nh_.subscribe("ackermann_cmd_tra", 10, 
                                          &TractorCommandInput::ackermannCmdTraCallback, this);
    point_b_sub_ = nh_.subscribe("Point_B", 10, 
                                 &TractorCommandInput::selfPointBCallback, this);
    cmd_vel_pub_ = nh_.advertise<msgs_core::ackermann_cmd>("ackermann_cmd", 5);
    
    ROS_INFO("TractorCommandInput: All subscriptions initialized");
}

void TractorCommandInput::remoteCmdCallback(const msgs_core::remote_info::ConstPtr& msg) {
    manual_mode_ = static_cast<int>(msg->ch[6] / 783.0f);
    linked_mode_ = static_cast<int>(msg->ch[7] / 783.0f);
    speed_mode_ = static_cast<int>(msg->ch[8] / 783.0f);
    is_follow_same_traj_ = static_cast<int>(msg->ch[9] / 783.0f);
    
    remote_v0_ = msg->ch[1] / 783.0f + remote_offset_v0_;
    remote_steering_ = -msg->ch[3] / 783.0f * 27.0f + remote_offset_steering_;
    
    remote_watchdog_ = 0;
}

void TractorCommandInput::ackermannCmdTraCallback(const msgs_core::ackermann_cmd::ConstPtr& msg) {
    steering_tra_ = msg->steering;
}

void TractorCommandInput::selfPointBCallback(const msgs_core::car_state::ConstPtr& msg) {
    self_point_b_yaw_ = msg->yaw;
}

void TractorCommandInput::processData() {
    // 转向控制
    steering_ = (manual_mode_ == -1) ? remote_steering_ : steering_tra_;
    
    // 速度控制
    if (speed_mode_ == -1) {
        v0_ = -remote_v0_;
    } else if (speed_mode_ == 0) {
        v0_ = -remote_v0_ * 2.0f;
    } else if (speed_mode_ == 1) {
        const float axel_dis = 0.6f;
        const float max_lat_acc = 0.8f;
        const float steering_rad = std::abs(steering_) * M_PI / 180.0f;
        const float tan_ster = std::tan(steering_rad);
        
        float acc_R = axel_dis / (tan_ster == 0.0f ? 0.001f : tan_ster);
        float v_max = std::sqrt(max_lat_acc * acc_R);
        
        if (v_max < 1.8f) {
            v0_ = v_max + 0.5f * (1.8f - v_max);
        } else {
            v0_ = 1.8f;
        }
        
        const float adjust_angle = std::abs(in2ulv_cores::utils_core::add_angles(self_point_b_yaw_, 90.0f));
        const float k_speed = 0.8f * adjust_angle / (adjust_angle + 30.0f) + 0.2f;
        v0_ *= k_speed;
    } else {
        v0_ = -remote_v0_;
    }
    
    remote_watchdog_++;
}

void TractorCommandInput::publishCommands() {
    msgs_core::ackermann_cmd msg;
    msg.v0 = v0_;
    msg.steering = steering_;
    in2ulv_cores::utils_core::safePublish(cmd_vel_pub_, msg);
}

TrailerCommandInput::TrailerCommandInput(ros::NodeHandle& nh) 
    : nh_(nh), remote_watchdog_(0), linked_mode_(-1), manual_mode_(-1), 
      speed_mode_(-1), is_follow_same_traj_(-1), remote_v0_(0.0f), 
      remote_steering_(0.0f), follow_v0_(0.0f), steering_tra_(0.0f), 
      steering_joint_(0.0f), v0_joint_(0.0f), steering_sep_(0.0f), 
      v0_sep_(0.0f), visual_mode_(0), v0_(0.0f), steering_(0.0f) {
    
    // 获取参数
    follow_chassis_ = in2ulv_cores::utils_core::getParamWithCheck<std::string>(nh_, "common/follow_chassis", "tractor", "warn");
    remote_offset_v0_ = in2ulv_cores::utils_core::getParamWithCheck<float>(nh_, "remote_offset/offset_v0", 0.0f, "warn");
    remote_offset_steering_ = in2ulv_cores::utils_core::getParamWithCheck<float>(nh_, "remote_offset/offset_steering", 0.0f, "warn");
    
    // 初始化订阅者和发布者
    remote_info_sub_ = nh_.subscribe("FS_remote_info", 10, 
                                    &TrailerCommandInput::remoteCmdCallback, this);
    follow_v0_sub_ = nh_.subscribe("/" + follow_chassis_ + "/ackermann_cmd", 10, 
                                  &TrailerCommandInput::followV0Callback, this);
    ackermann_cmd_tra_sub_ = nh_.subscribe("ackermann_cmd_tra", 10, 
                                          &TrailerCommandInput::ackermannCmdTraCallback, this);
    ackermann_cmd_joint_sub_ = nh_.subscribe("ackermann_cmd_joint", 10, 
                                            &TrailerCommandInput::ackermannCmdJointCallback, this);
    ackermann_cmd_sep_sub_ = nh_.subscribe("ackermann_cmd_sep", 10, 
                                          &TrailerCommandInput::ackermannCmdSepCallback, this);
    car_state_sub_ = nh_.subscribe("trailer_mode", 10, 
                                  &TrailerCommandInput::carStateCallback, this);
    cmd_vel_pub_ = nh_.advertise<msgs_core::ackermann_cmd>("ackermann_cmd", 5);
    car_state_pub_ = nh_.advertise<msgs_core::car_state>("car_state", 5);
    
    ROS_INFO("TrailerCommandInput: All subscriptions initialized");
}

void TrailerCommandInput::remoteCmdCallback(const msgs_core::remote_info::ConstPtr& msg) {
    manual_mode_ = static_cast<int>(msg->ch[6] / 783.0f);
    linked_mode_ = static_cast<int>(msg->ch[7] / 783.0f);
    speed_mode_ = static_cast<int>(msg->ch[8] / 783.0f);
    is_follow_same_traj_ = static_cast<int>(msg->ch[9] / 783.0f);
    
    remote_v0_ = msg->ch[1] / 783.0f + remote_offset_v0_;
    remote_steering_ = -msg->ch[3] / 783.0f * 27.0f + remote_offset_steering_;
    
    remote_watchdog_ = 0;
}

void TrailerCommandInput::followV0Callback(const msgs_core::ackermann_cmd::ConstPtr& msg) {
    follow_v0_ = msg->v0;
}

void TrailerCommandInput::ackermannCmdTraCallback(const msgs_core::ackermann_cmd::ConstPtr& msg) {
    steering_tra_ = msg->steering;
}

void TrailerCommandInput::ackermannCmdJointCallback(const msgs_core::ackermann_cmd::ConstPtr& msg) {
    steering_joint_ = msg->steering;
    v0_joint_ = msg->v0;
}

void TrailerCommandInput::ackermannCmdSepCallback(const msgs_core::ackermann_cmd::ConstPtr& msg) {
    steering_sep_ = msg->steering;
    v0_sep_ = msg->v0;
}

void TrailerCommandInput::carStateCallback(const msgs_core::car_state::ConstPtr& msg) {
    visual_mode_ = msg->visual;
}

void TrailerCommandInput::processData() {
    // 速度控制
    if (manual_mode_ == -1) {
        v0_ = remote_v0_;
    } else {
        if (linked_mode_ == -1) {           // 靠近、对接
            v0_ = v0_joint_;
        } else if (linked_mode_ == 0) {      // 已经对接，跟前车速度
            v0_ = follow_v0_;
        } else if (linked_mode_ == 1) {      // 分离或已经分离
            v0_ = v0_sep_;
        }
    }
    
    // 转向控制
    if (manual_mode_ == -1) {
        steering_ = remote_steering_;
    } else {
        if (linked_mode_ == -1) {           // 靠近、对接
            steering_ = steering_joint_;
        } else if (linked_mode_ == 0) {      // 已经对接
            if (is_follow_same_traj_ == 1) {
                steering_ = steering_tra_;
            } else {
                steering_ = remote_steering_;
            }
        } else if (linked_mode_ == 1) {      // 分离或已经分离
            steering_ = steering_sep_;
        }
    }
    
    remote_watchdog_++;
}

void TrailerCommandInput::publishCommands() {
    // 发布控制指令
    msgs_core::ackermann_cmd cmd_msg;
    cmd_msg.v0 = v0_;
    cmd_msg.steering = steering_;
    in2ulv_cores::utils_core::safePublish(cmd_vel_pub_, cmd_msg);
    
    // 发布车辆状态
    msgs_core::car_state state_msg;
    state_msg.linked = linked_mode_;
    state_msg.manual = manual_mode_;
    in2ulv_cores::utils_core::safePublish(car_state_pub_, state_msg);
}

std::unique_ptr<ChassisCmdGenerator> CommandInputFactory::createInstance(
    const std::string& chassis_type, ros::NodeHandle& nh) {
    if (chassis_type == "Head") {
        ROS_INFO("Creating TractorCommandInput");
        return std::make_unique<TractorCommandInput>(nh);
    } else {
        ROS_INFO("Creating TrailerCommandInput");
        return std::make_unique<TrailerCommandInput>(nh);
    }
}

}  // namespace in2ulv_controller