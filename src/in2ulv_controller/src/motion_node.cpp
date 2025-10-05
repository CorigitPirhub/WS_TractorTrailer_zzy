#include "in2ulv_controller/MotionProcessor.hpp"

int main(int argc, char *argv[]) {
    // 初始化ROS节点
    ros::init(argc, argv, "motion_processor_node");
    ros::NodeHandle nh;
    ROS_INFO("Motion Processor node starting");
    
    // 创建运动处理器实例
    in2ulv_controller::MotionProcessor processor(nh);
    
    // 获取处理频率参数
    int processing_rate = 100; // 默认值
    if (!in2ulv_cores::utils_core::getParamWithFullName(nh, "fps_main", processing_rate)) {
        ROS_WARN("Parameter 'fps_main' not found, using default: %d Hz", processing_rate);
    } else {
        ROS_INFO("Motion processing rate: %d Hz", processing_rate);
    }
    
    ros::Rate loop_rate(processing_rate);
    
    while (ros::ok()) {
        // 计算控制值
        processor.calculateControlValues();
        
        // 发布控制指令
        processor.publishControlCommands();
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("Motion Processor node shutting down");
    return 0;
}