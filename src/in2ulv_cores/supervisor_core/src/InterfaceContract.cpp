#include "supervisor_core/InterfaceContract.hpp"
#include <fstream>
#include <ros/package.h>

namespace in2ulv_cores {
namespace supervisor_core {
bool InterfaceManager::loadContracts(const std::string& config_path) {
    try {
        // 解析YAML配置
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 加载每个接口契约
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            std::string contract_name = it->first.as<std::string>();
            if (contract_name == "interface_version" || contract_name == "last_updated") {
                continue; // 跳过元数据
            }
            
            if (!parseInterfaceContract(it->second, contract_name)) {
                ROS_ERROR("Failed to parse interface contract: %s", contract_name.c_str());
                return false;
            }
        }
        
        ROS_INFO("Successfully loaded %lu interface contracts", contracts_.size());
        return true;
        
    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
        return false;
    }
}

bool InterfaceManager::parseInterfaceContract(const YAML::Node& node, const std::string& contract_name) {
    InterfaceContract contract;
    
    try {
        // 检查必须字段
        if (!node["topic"]) {
            ROS_ERROR("Missing 'topic' field in contract: %s", contract_name.c_str());
            return false;
        }
        contract.topic = node["topic"].as<std::string>();
        
        if (!node["msg_type"]) {
            ROS_ERROR("Missing 'msg_type' field in contract: %s", contract_name.c_str());
            return false;
        }
        contract.msg_type = node["msg_type"].as<std::string>();
        
        if (!node["description"]) {
            ROS_ERROR("Missing 'description' field in contract: %s", contract_name.c_str());
            return false;
        }
        contract.description = node["description"].as<std::string>();
        
        // 解析频率约束
        if (!node["constraints"] || !node["constraints"]["frequency"]) {
            ROS_ERROR("Missing 'constraints/frequency' in contract: %s", contract_name.c_str());
            return false;
        }
        YAML::Node freq_node = node["constraints"]["frequency"];
        if (!freq_node["expected"]) {
            ROS_ERROR("Missing 'expected' in frequency constraints: %s", contract_name.c_str());
            return false;
        }
        contract.frequency.expected_hz = freq_node["expected"].as<double>();
        if (!freq_node["min"]) {
            ROS_ERROR("Missing 'min' in frequency constraints: %s", contract_name.c_str());
            return false;
        }
        contract.frequency.min_hz = freq_node["min"].as<double>();
        if (!freq_node["max"]) {
            ROS_ERROR("Missing 'max' in frequency constraints: %s", contract_name.c_str());
            return false;
        }
        contract.frequency.max_hz = freq_node["max"].as<double>();
        
        // 解析发布者约束
        if (!node["constraints"]["publishers"]) {
            ROS_ERROR("Missing 'publishers' in constraints: %s", contract_name.c_str());
            return false;
        }
        YAML::Node pub_node = node["constraints"]["publishers"];
        for (size_t i = 0; i < pub_node.size(); i++) {
            contract.allowed_publishers.push_back(pub_node[i].as<std::string>());
        }
        
        // 解析订阅者约束
        if (!node["constraints"]["subscribers"]) {
            ROS_ERROR("Missing 'subscribers' in constraints: %s", contract_name.c_str());
            return false;
        }
        YAML::Node sub_node = node["constraints"]["subscribers"];
        for (size_t i = 0; i < sub_node.size(); i++) {
            contract.allowed_subscribers.push_back(sub_node[i].as<std::string>());
        }
        
        contracts_[contract.topic] = contract;
        return true;
        
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Error parsing contract %s: %s", contract_name.c_str(), e.what());
        return false;
    }
}

bool InterfaceManager::validatePublisher(const std::string& node_name, const std::string& topic) {
    auto it = contracts_.find(topic);
    if (it == contracts_.end()) {
        ROS_WARN("No contract found for topic: %s", topic.c_str());
        return true; // 未定义契约的话题允许访问
    }
    
    const auto& contract = it->second;
    bool allowed = false;
    for (const auto& allowed_name : contract.allowed_publishers) {
        // 允许节点名称部分匹配（考虑命名空间）
        if (node_name == allowed_name || node_name.find("/" + allowed_name) != std::string::npos) {
            allowed = true;
            break;
        }
    }
    
    if (!allowed) {
        ROS_ERROR("Node %s is not allowed to publish to topic %s", 
                 node_name.c_str(), topic.c_str());
    }
    
    return allowed;
}

bool InterfaceManager::validateSubscriber(const std::string& node_name, const std::string& topic) {
    auto it = contracts_.find(topic);
    if (it == contracts_.end()) {
        ROS_WARN("No contract found for topic: %s", topic.c_str());
        return true; // 未定义契约的话题允许访问
    }
    
    const auto& contract = it->second;
    bool allowed = false;
    for (const auto& allowed_name : contract.allowed_subscribers) {
        if (node_name == allowed_name || node_name.find("/" + allowed_name) != std::string::npos) {
            allowed = true;
            break;
        }
    }
    
    if (!allowed) {
        ROS_WARN("Node %s is not allowed to subscribe to topic %s", 
                node_name.c_str(), topic.c_str());
    }
    
    return allowed;
}

const InterfaceContract* InterfaceManager::getContract(const std::string& topic) const {
    auto it = contracts_.find(topic);
    if (it != contracts_.end()) {
        return &it->second;
    }
    return nullptr;
}

}  // namespace supervisor_core
}  // namespace in2ulv_cores