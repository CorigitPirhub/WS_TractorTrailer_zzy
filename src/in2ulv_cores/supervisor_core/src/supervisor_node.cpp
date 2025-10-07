#include <ros/ros.h>
#include <ros/package.h>
#include "supervisor_core/InterfaceContract.hpp"
#include "supervisor_core/CentralMonitor.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "supervisor_node");
    ros::NodeHandle nh("~");
    
    // 获取配置文件路径
    std::string package_path = ros::package::getPath("supervisor_core");
    std::string config_path = package_path + "/config/interfaces.yaml";
    
    // 加载接口契约
    in2ulv_cores::supervisor_core::InterfaceManager interface_manager(nh);
    if (!interface_manager.loadContracts(config_path)) {
        ROS_ERROR("Failed to load interface contracts.");
        return 1;
    }
    
    // 创建中央监控器
    in2ulv_cores::supervisor_core::CentralMonitor central_monitor(nh, interface_manager);
    
    ros::spin();
    return 0;
}