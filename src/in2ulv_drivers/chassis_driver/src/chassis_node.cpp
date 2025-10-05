#include "chassis_driver/ChassisController.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_control_node");
    ros::NodeHandle nh;
    
    // 设置CAN接口（可选，可以在系统启动时配置）
    // system("sudo ip link set can0 type can bitrate 500000");
    // system("sudo ip link set can0 up");
    
    in2ulv_drivers::chassis_driver::ChassisController controller(nh);
    controller.run();
    
    return 0;
}