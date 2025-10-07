#ifndef INTERFACECONTRACT_HPP
#define INTERFACECONTRACT_HPP

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>

namespace in2ulv_cores {
namespace supervisor_core {
/**
 * @brief 频率约束结构体
 * @details 用于描述话题的频率约束
 * - expected_hz: 期望频率
 * - min_hz: 最小允许频率
 * - max_hz: 最大允许频率
 */
struct FrequencyConstraints {
    double expected_hz;
    double min_hz;
    double max_hz;
};

/**
 * @brief 接口契约结构体
 * @details 用于描述ROS话题的接口契约，包括话题名称、消息类型、频率约束、允许的发布者和订阅者等
 * - topic: 话题名称
 * - msg_type: 消息类型
 * - description: 描述
 * - frequency: 频率约束
 * - allowed_publishers: 允许的发布者
 * - allowed_subscribers: 允许的订阅者
 */
struct InterfaceContract {
    std::string topic;
    std::string msg_type;
    std::string description;
    
    FrequencyConstraints frequency;
    std::vector<std::string> allowed_publishers;
    std::vector<std::string> allowed_subscribers;
    
    // 验证频率是否在允许范围内
    bool validateFrequency(double actual_hz) const {
        // 如果实际频率为0，可能是节点未启动，暂时不报警
        if (actual_hz == 0) return true;
        return actual_hz >= frequency.min_hz && actual_hz <= frequency.max_hz;
    }
};

/**
 * @brief 接口管理器类
 * @details 用于管理ROS话题的接口契约，包括加载契约、验证节点行为是否符合契约等
 */
class InterfaceManager {
public:
    InterfaceManager(ros::NodeHandle& nh) : nh_(nh) {}
    
    // 获取所有契约
    const std::map<std::string, InterfaceContract>& getAllContracts() const { return contracts_; }

    // 加载接口契约
    bool loadContracts(const std::string& config_path);
    
    // 验证节点行为是否符合契约
    bool validatePublisher(const std::string& node_name, const std::string& topic);
    bool validateSubscriber(const std::string& node_name, const std::string& topic);
    
    // 获取话题的契约
    const InterfaceContract* getContract(const std::string& topic) const;

private:
    ros::NodeHandle nh_;
    std::map<std::string, InterfaceContract> contracts_;

    // 解析接口契约
    bool parseInterfaceContract(const YAML::Node& node, const std::string& contract_name);
};
}  // namespace supervisor_core
}  // namespace in2ulv_cores

#endif  // INTERFACECONTRACT_HPP