#include "in2ulv_controller/MotionProcessor.hpp"

namespace in2ulv_controller {

using namespace in2ulv_cores::utils_core;

MotionProcessor::MotionProcessor(ros::NodeHandle& nh) 
    : link_state_(-1), manual_mode_(-1),
      left_wheel_velocity_(0.0f), right_wheel_velocity_(0.0f),
      front_axle_angle_(0.0f),
      cmd_velocity_(0.0f), cmd_velocity_last_(0.0f),
      cmd_steering_(0.0f),
      cmd_gear_(5),
      cmd_left_torque_(0.0f), cmd_right_torque_(0.0f),
      motor_positive_direction_(true),
      current_angle_(0.0f),
      measured_velocity_(0.0f), measured_velocity_diff_(0.0f),
      angle_correction_pid_(0.02f, 0.03f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f, "angle_correction_pid"),
      left_wheel_pid_(1.5f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f, "left_wheel_pid"),
      right_wheel_pid_(1.39f, 0.0f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f, "right_wheel_pid"),
      angle_kp_(0.0f), angle_ki_(0.0f), angle_deadband_(0.0f),
      left_wheel_kp_(0.0f), right_wheel_kp_(0.0f),
      pid_tuning_enabled_(false) {
    
    // 初始化订阅者和发布者
    ackermann_cmd_sub_ = nh.subscribe("ackermann_cmd", 5, &MotionProcessor::ackermannCmdCallback, this);
    car_state_sub_ = nh.subscribe("car_state", 5, &MotionProcessor::carStateCallback, this);
    angle_sub_ = nh.subscribe("angles", 5, &MotionProcessor::angleCallback, this);
    left_wheel_fb_sub_ = nh.subscribe("lr_wheel_fb", 5, &MotionProcessor::leftWheelCallback, this);
    right_wheel_fb_sub_ = nh.subscribe("rr_wheel_fb", 5, &MotionProcessor::rightWheelCallback, this);
    
    ctrl_cmd_pub_ = nh.advertise<msgs_core::ctrl_cmd>("ctrl_cmd", 5);
    torque_cmd_pub_ = nh.advertise<msgs_core::torque_cmd>("torque_cmd", 5);
    angle_error_pub_ = nh.advertise<msgs_core::angle_error>("angle_error", 5);
    
    // 获取参数
    getParamWithFullName(nh, "chassis_ctl_param/motor_positive_direction", motor_positive_direction_);
    getParamWithFullName(nh, "motion_controller/enable_pid_tuning", pid_tuning_enabled_);
    getParamWithFullName(nh, "chassis_ctl_param/ang_kp", angle_kp_);
    getParamWithFullName(nh, "chassis_ctl_param/ang_ki", angle_ki_);
    getParamWithFullName(nh, "chassis_ctl_param/ang_dead_band", angle_deadband_);
    getParamWithFullName(nh, "chassis_ctl_param/vel_l_kp", left_wheel_kp_);
    getParamWithFullName(nh, "chassis_ctl_param/vel_r_kp", right_wheel_kp_);
    
    // 初始化PID控制器
    if (pid_tuning_enabled_) {
        dynamic_reconfigure::Server<in2ulv_controller::PIDControlConfig>::CallbackType f;
        f = boost::bind(&MotionProcessor::reconfigureCallback, this, _1, _2);
        dyn_reconf_server_.setCallback(f);
    } else {
        angle_correction_pid_.setProportionalGain(angle_kp_);
        angle_correction_pid_.setIntegralGain(angle_ki_);
        angle_correction_pid_.setDeadband(angle_deadband_);
        angle_correction_pid_.setFeedforwardGain(0.0f);
        angle_correction_pid_.setDerivativeFilter(0.1f);
        angle_correction_pid_.setAntiWindupGain(0.02f);
        
        left_wheel_pid_.setProportionalGain(left_wheel_kp_);
        left_wheel_pid_.setFeedforwardGain(0.0f);
        left_wheel_pid_.setDerivativeFilter(0.1f);
        left_wheel_pid_.setAntiWindupGain(0.02f);
        left_wheel_pid_.setDeadband(0.01f);
        
        right_wheel_pid_.setProportionalGain(right_wheel_kp_);
        right_wheel_pid_.setFeedforwardGain(0.0f);
        right_wheel_pid_.setDerivativeFilter(0.1f);
        right_wheel_pid_.setAntiWindupGain(0.02f);
        right_wheel_pid_.setDeadband(0.01f);
    }
    
    last_update_time_ = ros::Time::now();
    ROS_INFO("MotionProcessor initialized");
}

void MotionProcessor::reconfigureCallback(in2ulv_controller::PIDControlConfig& config, uint32_t level) {
    if (pid_tuning_enabled_) {
        angle_correction_pid_.reset();
        left_wheel_pid_.reset();
        right_wheel_pid_.reset();
        
        left_wheel_pid_.setProportionalGain(config.left_vel_kp);
        left_wheel_pid_.setIntegralGain(config.left_vel_ki);
        left_wheel_pid_.setDerivativeGain(config.left_vel_kd);
        left_wheel_pid_.setFeedforwardGain(config.left_vel_ffg);
        left_wheel_pid_.setDerivativeFilter(config.left_vel_cdf);
        left_wheel_pid_.setAntiWindupGain(config.left_vel_kaw);
        left_wheel_pid_.setDeadband(config.left_vel_deadband);
        
        right_wheel_pid_.setProportionalGain(config.right_vel_kp);
        right_wheel_pid_.setIntegralGain(config.right_vel_ki);
        right_wheel_pid_.setDerivativeGain(config.right_vel_kd);
        right_wheel_pid_.setFeedforwardGain(config.right_vel_ffg);
        right_wheel_pid_.setDerivativeFilter(config.right_vel_cdf);
        right_wheel_pid_.setAntiWindupGain(config.right_vel_kaw);
        right_wheel_pid_.setDeadband(config.right_vel_deadband);
        
        angle_correction_pid_.setProportionalGain(config.back_ang_kp);
        angle_correction_pid_.setIntegralGain(config.back_ang_ki);
        angle_correction_pid_.setDerivativeGain(config.back_ang_kd);
        angle_correction_pid_.setFeedforwardGain(config.back_ang_ffg);
        angle_correction_pid_.setDerivativeFilter(config.back_ang_cdf);
        angle_correction_pid_.setAntiWindupGain(config.back_ang_kaw);
        angle_correction_pid_.setDeadband(config.back_ang_deadband);
        
        ROS_INFO("PID parameters reconfigured");
    }
}

void MotionProcessor::ackermannCmdCallback(const msgs_core::ackermann_cmd::ConstPtr& msg) {
    MotionExpectations expectations;
    expectations.velocity = msg->v0;
    expectations.steering = msg->steering;
    updateMotionExpectations(expectations);
}

void MotionProcessor::carStateCallback(const msgs_core::car_state::ConstPtr& msg) {
    link_state_ = msg->linked;
    manual_mode_ = msg->manual;
}

void MotionProcessor::angleCallback(const msgs_core::angles::ConstPtr& msg) {
    current_angle_ = in2ulv_cores::utils_core::normalize_angle(msg->angle_f);
}

void MotionProcessor::leftWheelCallback(const msgs_core::lr_wheel_fb::ConstPtr& msg) {
    left_wheel_velocity_ = msg->lr_wheel_fb_velocity;
}

void MotionProcessor::rightWheelCallback(const msgs_core::rr_wheel_fb::ConstPtr& msg) {
    right_wheel_velocity_ = msg->rr_wheel_fb_velocity;
}

void MotionProcessor::updateMotionExpectations(const MotionExpectations& expectations) {
    float velocity = expectations.velocity;
    
    if (!motor_positive_direction_) {
        velocity = -velocity;
    }
    
    cmd_velocity_last_ = cmd_velocity_;
    cmd_velocity_ = velocity;
    
    bool reset_pid = true;
    
    if (link_state_ == 0) {
        cmd_gear_ = 5;
        reset_pid = false;
    } else if (cmd_velocity_ > 1e-2) {
        cmd_gear_ = 4;
    } else if (cmd_velocity_ < -1e-2) {
        cmd_gear_ = 2;
        cmd_velocity_ = -cmd_velocity_;
    } else if (manual_mode_ == -1 && std::abs(current_angle_) > 2 * angle_deadband_) {
        cmd_gear_ = 5;
        reset_pid = false;
    } else {
        cmd_gear_ = 1;
    }
    
    if (reset_pid) {
        left_wheel_pid_.reset();
        right_wheel_pid_.reset();
    }
    
    cmd_steering_ = expectations.steering;
}

void MotionProcessor::calculateControlValues() {
    // 计算测量值
    measured_velocity_ = (left_wheel_velocity_ + right_wheel_velocity_) * 0.5f;
    measured_velocity_diff_ = (left_wheel_velocity_ - right_wheel_velocity_) * 0.5f;
    
    // 计算时间间隔
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;
    
    // 发布角度误差
    msgs_core::angle_error angle_error_msg;
    angle_error_msg.angle_error_p = -current_angle_;
    safePublish(angle_error_pub_, angle_error_msg);
    
    float velocity_diff_expected = 0.0f;
    const float target_angle = 0.0f;
    
    if (link_state_ == -1 || link_state_ == 1) {
        if (std::abs(measured_velocity_) < 0.05f && manual_mode_ == -1) {
            velocity_diff_expected = angle_correction_pid_.compute(
                target_angle, current_angle_, dt, 1.0f);
        } else {
            angle_correction_pid_.reset();
            velocity_diff_expected = REFERENCE_K * cmd_velocity_ * 0.305f / 0.645f * 
                                     std::tan(cmd_steering_ * PI / 180.0f);
        }
        cmd_left_velocity_ = -velocity_diff_expected;
        cmd_right_velocity_ = velocity_diff_expected;
    } else {
        cmd_left_velocity_ = cmd_velocity_;
        cmd_right_velocity_ = cmd_velocity_;
    }
    
    // 计算扭矩
    cmd_left_torque_ = left_wheel_pid_.compute(cmd_left_velocity_, measured_velocity_, dt);
    cmd_right_torque_ = right_wheel_pid_.compute(cmd_right_velocity_, measured_velocity_, dt);
    
    // PID调试信息
    if (pid_tuning_enabled_) {
        ROS_INFO("%s Output: %f, Proportional: %f, Integral: %f, Derivative: %f, Feedforward: %f, Saturation: %d",
                left_wheel_pid_.getName().c_str(),
                left_wheel_pid_.getOutput(),
                left_wheel_pid_.getProportional(),
                left_wheel_pid_.getIntegral(),
                left_wheel_pid_.getDerivative(),
                left_wheel_pid_.getFeedforward(),
                left_wheel_pid_.getOutputSaturationStatus());
        
        ROS_INFO("%s Output: %f, Proportional: %f, Integral: %f, Derivative: %f, Feedforward: %f, Saturation: %d",
                right_wheel_pid_.getName().c_str(),
                right_wheel_pid_.getOutput(),
                right_wheel_pid_.getProportional(),
                right_wheel_pid_.getIntegral(),
                right_wheel_pid_.getDerivative(),
                right_wheel_pid_.getFeedforward(),
                right_wheel_pid_.getOutputSaturationStatus());
    }
}

void MotionProcessor::publishControlCommands() {
    // 发布控制命令
    msgs_core::ctrl_cmd ctrl_msg;
    ctrl_msg.ctrl_cmd_gear = cmd_gear_;
    ctrl_msg.ctrl_cmd_steering = cmd_steering_;
    ctrl_msg.ctrl_cmd_velocity = REFERENCE_K * cmd_velocity_;
    safePublish(ctrl_cmd_pub_, ctrl_msg);
    
    // 发布扭矩命令
    msgs_core::torque_cmd torque_msg;
    torque_msg.lr_torque_cmd = cmd_left_torque_;
    torque_msg.rr_torque_cmd = cmd_right_torque_;
    safePublish(torque_cmd_pub_, torque_msg);
}

} // namespace in2ulv_controller