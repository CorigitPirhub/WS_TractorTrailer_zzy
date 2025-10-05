#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include <ros/ros.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "msgs_core/ctrl_cmd.h"
#include "msgs_core/io_cmd.h"
#include "msgs_core/torque_cmd.h"
#include "msgs_core/angles.h"
#include "msgs_core/ctrl_fb.h"
#include "msgs_core/lr_wheel_fb.h"
#include "msgs_core/rr_wheel_fb.h"
#include "msgs_core/io_fb.h"
#include "msgs_core/odo_fb.h"
#include "msgs_core/bms_Infor.h"
#include "msgs_core/bms_flag_Infor.h"
#include "msgs_core/Drive_MCUEcoder_fb.h"
#include "msgs_core/Veh_Diag_fb.h"
#include "msgs_core/torque_fb.h"
#include "msgs_core/current_fb.h"

#include "utils_core/ros_utils.hpp"
#include "utils_core/angle_utils.hpp"

namespace in2ulv_drivers {
namespace chassis_driver {

/**
 * @class ChassisController
 * @brief 底盘控制器类，负责与底盘系统的CAN通信
 * 
 * 该类处理底盘控制指令的发送和反馈数据的接收，包括：
 * - 速度控制
 * - 转向控制
 * - 扭矩控制
 * - IO控制
 * - 底盘状态反馈
 */
class ChassisController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    ChassisController(ros::NodeHandle& nh);
    
    /**
     * @brief 运行底盘控制器
     */
    void run();
    
private:
    /**
     * @brief 初始化CAN总线连接
     * @return 初始化是否成功
     */
    bool initializeCAN();
    
    /**
     * @brief IO控制回调函数
     * @param msg IO控制消息
     */
    void ioCmdCallback(const msgs_core::io_cmd::ConstPtr& msg);
    
    /**
     * @brief 速度控制回调函数
     * @param msg 速度控制消息
     */
    void ctrlCmdCallback(const msgs_core::ctrl_cmd::ConstPtr& msg);
    
    /**
     * @brief 扭矩控制回调函数
     * @param msg 扭矩控制消息
     */
    void torqueCmdCallback(const msgs_core::torque_cmd::ConstPtr& msg);
    
    /**
     * @brief 接收并处理CAN数据
     */
    void receiveData();
    
    /**
     * @brief 设置CAN设备回传模式
     */
    void setEchoMode();
    
    /**
     * @brief 设置CAN设备回传时间
     */
    void setEchoTime();
    
    /**
     * @brief 设置CAN设备ID
     */
    void setDeviceID();
    
    /**
     * @brief 设置编码器零位
     */
    void setEncoderZero();
    
    ros::NodeHandle nh_;                  // ROS节点句柄
    int can_socket_;                      // CAN套接字描述符
    struct can_frame send_frame_;         // CAN发送帧
    struct can_frame recv_frame_;         // CAN接收帧
    
    // 发布者
    ros::Publisher angles_pub_;             // 角度反馈
    ros::Publisher ctrl_fb_pub_;            // 控制反馈
    ros::Publisher lr_wheel_fb_pub_;        // 左轮反馈
    ros::Publisher rr_wheel_fb_pub_;        // 右轮反馈
    ros::Publisher io_fb_pub_;              // IO反馈
    ros::Publisher odo_fb_pub_;             // 里程计反馈
    ros::Publisher bms_infor_pub_;           // BMS信息
    ros::Publisher bms_flag_infor_pub_;      // BMS标志信息
    ros::Publisher drive_mcu_encoder_pub_;   // 驱动MCU编码器反馈
    ros::Publisher veh_diag_fb_pub_;        // 车辆诊断反馈
    ros::Publisher torque_fb_pub_;           // 扭矩反馈
    ros::Publisher current_fb_pub_;          // 电流反馈
    
    // 订阅者
    ros::Subscriber ctrl_cmd_sub_;          // 控制命令
    ros::Subscriber io_cmd_sub_;            // IO命令
    ros::Subscriber torque_cmd_sub_;        // 扭矩命令
    
    boost::mutex cmd_mutex_;                // 命令互斥锁
    
    float angle_offset_front_;              // 前编码器角度偏移
    float angle_offset_rear_;               // 后编码器角度偏移
};

}  // namespace chassis_driver
}  // namespace in2ulv_drivers

#endif  // CHASSIS_CONTROLLER_HPP