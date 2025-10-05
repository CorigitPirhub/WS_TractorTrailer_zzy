#include "force_sensor_driver/ForceSensorController.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_sensor_node");
    ros::NodeHandle nh("~"); // 私有节点句柄
    
    // 从参数服务器获取配置
    std::string port = "/dev/ttyACM0";
    int baudrate = 115200;
    float force_gain = 0.02f;
    
    nh.param<std::string>("port", port, port);
    nh.param<int>("baudrate", baudrate, baudrate);
    nh.param<float>("force_gain", force_gain, force_gain);
    
    // 创建控制器实例
    in2ulv_drivers::force_sensor_driver::ForceSensorController controller(
        nh, port, baudrate, force_gain
    );
    
    // 初始化并运行
    if (controller.initialize()) {
        ROS_INFO("Force sensor controller initialized successfully");
        controller.run();
    } else {
        ROS_ERROR("Failed to initialize force sensor controller");
        return 1;
    }
    
    // 关闭连接
    controller.shutdown();
    return 0;
}