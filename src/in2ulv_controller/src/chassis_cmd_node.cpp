#include "in2ulv_controller/ChassisCmdGenerator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_cmd_node");
    ros::NodeHandle nh;
    
    in2ulv_cores::utils_core::global_monitor_agent = 
        std::make_unique<in2ulv_cores::supervisor_core::MonitoringAgent>(nh, ros::this_node::getName());
    // 创建定时器，每秒发布一次监控数据
    ros::Timer metrics_timer = nh.createTimer(ros::Duration(1.0),
        [](const ros::TimerEvent&) {
            in2ulv_cores::utils_core::global_monitor_agent->publishMetrics();
        });

    // 获取底盘类型参数
    std::string chassis_type;
    try {
        // 使用无默认值版本，如果参数不存在会抛出异常
        chassis_type = in2ulv_cores::utils_core::getParamWithCheck<std::string>(
            nh, "common/chassis_type", "info");
    } catch (const std::runtime_error& e) {
        ROS_FATAL("Failed to get parameter 'common/chassis_type': %s", e.what());
        return 1;
    }
    
    ROS_INFO("Chassis type: %s", chassis_type.c_str());
    
    // 创建命令输入实例
    auto command_input = in2ulv_controller::CommandInputFactory::createInstance(chassis_type, nh);
    
    ros::Rate rate(100);  // 100Hz
    while (ros::ok()) {
        command_input->processData();
        command_input->publishCommands();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}