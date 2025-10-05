#ifndef FORCE_SENSOR_CONTROLLER_HPP
#define FORCE_SENSOR_CONTROLLER_HPP

#include <ros/ros.h>
#include <serial/serial.h>
#include "msgs_core/force_info.h"
#include "utils_core/modbus_crc.hpp"

namespace in2ulv_drivers {
namespace force_sensor_driver {

class ForceSensorController {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param port 串口设备路径
     * @param baudrate 波特率
     * @param force_gain 力传感器增益系数
     */
    ForceSensorController(ros::NodeHandle& nh, 
                         const std::string& port = "/dev/ttyACM0", 
                         int baudrate = 115200,
                         float force_gain = 0.02f);
    
    /**
     * @brief 初始化串口连接
     * @return 初始化是否成功
     */
    bool initialize();
    
    /**
     * @brief 主运行循环
     */
    void run();
    
    /**
     * @brief 关闭串口连接
     */
    void shutdown();

private:
    /**
     * @brief 发送读取命令到传感器
     */
    void sendReadCommand();
    
    /**
     * @brief 处理传感器响应数据
     * @return 处理是否成功
     */
    bool processResponse();
    
    /**
     * @brief 将字节数组转换为浮点数
     * @param buff 字节数组指针
     * @return 转换后的浮点数
     */
    float bytesToFloat(uint8_t* buff);
    
    /**
     * @brief 将浮点数转换为字节数组
     * @param f 浮点数
     * @param buff 字节数组指针
     */
    void floatToBytes(float f, uint8_t* buff);

private:
    ros::NodeHandle nh_;                   // ROS节点句柄
    serial::Serial serial_port_;           // 串口对象
    ros::Publisher force_info_pub_;       // 力信息发布器
    
    uint8_t read_cmd_[8];                 // 读取命令缓冲区
    uint8_t id_count_;                    // 传感器ID计数器
    float force_gain_;                     // 力传感器增益系数
    msgs_core::force_info force_info_msg_; // 力信息消息
    
    std::string port_;                    // 串口设备路径
    int baudrate_;                        // 波特率
    float force_l_;                       // 左传感器力值
    float force_r_;                       // 右传感器力值
};

}  // namespace force_sensor_driver
}  // namespace in2ulv_drivers

#endif  // FORCE_SENSOR_CONTROLLER_HPP