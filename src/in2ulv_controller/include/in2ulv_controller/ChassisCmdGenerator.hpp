#ifndef CHASSIS_COMMAND_INPUT_HPP
#define CHASSIS_COMMAND_INPUT_HPP

#include <ros/ros.h>

#include "msgs_core/remote_info.h"
#include "msgs_core/ackermann_cmd.h"
#include "msgs_core/car_state.h"

#include "utils_core/angle_utils.hpp"
#include "utils_core/ros_utils.hpp"

namespace in2ulv_controller {

/**
 * @class ChassisCmdGenerator
 * @brief 底盘控制指令输入基类
 * 
 * 提供底盘控制指令输入的基本接口
 */
class ChassisCmdGenerator {
public:
    virtual ~ChassisCmdGenerator() = default;
    
    /**
     * @brief 处理数据并生成控制指令
     */
    virtual void processData() = 0;
    
    /**
     * @brief 发布控制指令和状态信息
     */
    virtual void publishCommands() = 0;
};

/**
 * @class TractorCommandInput
 * @brief 头车控制指令输入类
 * 
 * 处理头车的控制指令输入逻辑
 */
class TractorCommandInput : public ChassisCmdGenerator {
public:
    explicit TractorCommandInput(ros::NodeHandle& nh);
    
    void processData() override;
    void publishCommands() override;

private:
    void remoteCmdCallback(const msgs_core::remote_info::ConstPtr& msg);
    void ackermannCmdTraCallback(const msgs_core::ackermann_cmd::ConstPtr& msg);
    void selfPointBCallback(const msgs_core::car_state::ConstPtr& msg);
    
    ros::NodeHandle nh_;
    ros::Subscriber remote_info_sub_;
    ros::Subscriber ackermann_cmd_tra_sub_;
    ros::Subscriber point_b_sub_;
    ros::Publisher cmd_vel_pub_;
    
    int remote_watchdog_;
    int linked_mode_;
    int manual_mode_;
    int speed_mode_;
    int is_follow_same_traj_;
    float remote_v0_;
    float remote_steering_;
    float remote_offset_v0_;
    float remote_offset_steering_;
    float steering_tra_;
    float self_point_b_yaw_;
    float v0_;
    float steering_;
};

/**
 * @class TrailerCommandInput
 * @brief 挂车控制指令输入类
 * 
 * 处理挂车的控制指令输入逻辑
 */
class TrailerCommandInput : public ChassisCmdGenerator {
public:
    explicit TrailerCommandInput(ros::NodeHandle& nh);
    
    void processData() override;
    void publishCommands() override;

private:
    void remoteCmdCallback(const msgs_core::remote_info::ConstPtr& msg);
    void followV0Callback(const msgs_core::ackermann_cmd::ConstPtr& msg);
    void ackermannCmdTraCallback(const msgs_core::ackermann_cmd::ConstPtr& msg);
    void ackermannCmdJointCallback(const msgs_core::ackermann_cmd::ConstPtr& msg);
    void ackermannCmdSepCallback(const msgs_core::ackermann_cmd::ConstPtr& msg);
    void carStateCallback(const msgs_core::car_state::ConstPtr& msg);
    
    ros::NodeHandle nh_;
    ros::Subscriber remote_info_sub_;
    ros::Subscriber follow_v0_sub_;
    ros::Subscriber ackermann_cmd_tra_sub_;
    ros::Subscriber ackermann_cmd_joint_sub_;
    ros::Subscriber ackermann_cmd_sep_sub_;
    ros::Subscriber car_state_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher car_state_pub_;
    
    int remote_watchdog_;
    int linked_mode_;
    int manual_mode_;
    int speed_mode_;
    int is_follow_same_traj_;
    float remote_v0_;
    float remote_steering_;
    float remote_offset_v0_;
    float remote_offset_steering_;
    float follow_v0_;
    float steering_tra_;
    float steering_joint_;
    float v0_joint_;
    float steering_sep_;
    float v0_sep_;
    int visual_mode_;
    float v0_;
    float steering_;
    std::string follow_chassis_;
};

/**
 * @class CommandInputFactory
 * @brief 命令输入工厂类
 * 
 * 根据底盘类型创建对应的命令输入实例
 */
class CommandInputFactory {
public:
    /**
     * @brief 创建命令输入实例
     * 
     * @param chassis_type 底盘类型 ("Head" 或 "Trailer")
     * @param nh ROS节点句柄
     * @return 命令输入实例指针
     */
    static std::unique_ptr<ChassisCmdGenerator> createInstance(
        const std::string& chassis_type, ros::NodeHandle& nh);
};

}  // namespace in2ulv_controller

#endif  // CHASSIS_COMMAND_INPUT_HPP