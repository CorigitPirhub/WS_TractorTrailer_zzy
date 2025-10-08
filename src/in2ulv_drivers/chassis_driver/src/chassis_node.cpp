#include "chassis_driver/ChassisController.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_control_node");
    ros::NodeHandle nh;
    
    in2ulv_cores::utils_core::global_monitor_agent = 
        std::make_unique<in2ulv_cores::supervisor_core::MonitoringAgent>(nh, ros::this_node::getName());
    // 创建定时器，每秒发布一次监控数据
    ros::Timer metrics_timer = nh.createTimer(ros::Duration(1.0),
        [](const ros::TimerEvent&) {
            in2ulv_cores::utils_core::global_monitor_agent->publishMetrics();
        });

    // 设置CAN接口（可选，可以在系统启动时配置）
    // system("sudo ip link set can0 type can bitrate 500000");
    // system("sudo ip link set can0 up");
    
    in2ulv_drivers::chassis_driver::ChassisController controller(nh);
    controller.run();
    
    return 0;
}