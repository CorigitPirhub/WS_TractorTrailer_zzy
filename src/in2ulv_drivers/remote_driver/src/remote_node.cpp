#include "remote_driver/RemoteController.hpp"
#include "remote_driver/ActuatorController.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "remote_control_node");
    ros::NodeHandle nh;
    
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Starting remote control node...");
    
    // 创建遥控控制器
    in2ulv_drivers::remote_driver::RemoteController remote_controller(nh);
    if (!remote_controller.initialize()) {
        ROS_ERROR("Failed to initialize remote controller");
        return 1;
    }
    
    // 创建STM32通信器
    in2ulv_drivers::remote_driver::ActuatorController stm32_communicator(nh, remote_controller.getSerialPort());
    stm32_communicator.initialize();
    
    // 设置回调更新函数
    remote_controller.setActuatorController(&stm32_communicator);
    
    ROS_INFO("Starting main loop...");
    
    // 运行主循环
    remote_controller.run();
    
    ROS_INFO("Shutting down remote control node");
    return 0;
}