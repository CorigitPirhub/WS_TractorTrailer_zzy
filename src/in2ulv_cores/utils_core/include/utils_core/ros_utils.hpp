#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_msgs/Header.h>
#include <map>
#include <mutex>
#include <XmlRpcValue.h>
#include <stdexcept>

namespace in2ulv_cores {
namespace utils_core {

/**
 * @brief ROS实用工具函数集合
 * 
 * 提供与ROS相关的常用工具函数，包括：
 * - 参数获取
 * - ROS状态检查
 * - 消息头创建
 * - 安全消息发布
 * - 参数解析
 */

/**
 * @brief 获取ROS参数并添加检测机制，确保参数获取成功
 * 
 * @tparam T 参数类型
 * @param nh ROS节点句柄
 * @param param_name 参数名称
 * @param default_value 参数不存在时的默认值
 * @param log_level 日志级别 ("debug", "info", "warn", "error")
 * @return 获取的参数值或默认值
 * 
 * @throws std::runtime_error 如果参数不存在且未提供默认值
 */
template <typename T>
T getParamWithCheck(ros::NodeHandle& nh, const std::string& param_name, 
                    const T& default_value, const std::string& log_level = "info") {
    T value;
    bool has_param = nh.getParam(param_name, value);
    
    if (!has_param) {
        if (log_level == "warn") {
            ROS_WARN("Parameter '%s' not found, using default: %s", 
                     param_name.c_str(), boost::lexical_cast<std::string>(default_value).c_str());
        } else if (log_level == "error") {
            ROS_ERROR("Parameter '%s' not found, using default: %s", 
                      param_name.c_str(), boost::lexical_cast<std::string>(default_value).c_str());
        } else {
            ROS_INFO("Parameter '%s' not found, using default: %s", 
                     param_name.c_str(), boost::lexical_cast<std::string>(default_value).c_str());
        }
        return default_value;
    }
    
    if (log_level == "debug") {
        ROS_DEBUG("Got parameter '%s' = %s", param_name.c_str(), 
                  boost::lexical_cast<std::string>(value).c_str());
        
        // 显示相关参数以帮助调试
        std::vector<std::string> param_names;
        nh.getParamNames(param_names);
        
        std::string base_name = param_name.substr(param_name.find_last_of('/') + 1);
        std::vector<std::string> matching_params;
        
        for (const auto& name : param_names) {
            if (name.find(base_name) != std::string::npos) {
                matching_params.push_back(name);
            }
        }
        
        if (!matching_params.empty()) {
            ROS_DEBUG("Related parameters for '%s':", base_name.c_str());
            for (const auto& name : matching_params) {
                ROS_DEBUG("  %s", name.c_str());
            }
        }
    } else {
        ROS_INFO("Got parameter '%s' = %s", param_name.c_str(), 
                 boost::lexical_cast<std::string>(value).c_str());
    }
    
    return value;
}

/**
 * @brief 获取ROS参数并添加检测机制（无默认值版本）
 * 
 * @tparam T 参数类型
 * @param nh ROS节点句柄
 * @param param_name 参数名称
 * @param log_level 日志级别 ("debug", "info", "warn", "error")
 * @return 获取的参数值
 * 
 * @throws std::runtime_error 如果参数不存在
 */
template <typename T>
T getParamWithCheck(ros::NodeHandle& nh, const std::string& param_name, 
                    const std::string& log_level = "info") {
    T value;
    bool has_param = nh.getParam(param_name, value);
    
    if (!has_param) {
        std::string error_msg = "Parameter '" + param_name + "' not found and no default provided!";
        if (log_level == "warn") {
            ROS_WARN("%s", error_msg.c_str());
        } else {
            ROS_ERROR("%s", error_msg.c_str());
        }
        throw std::runtime_error(error_msg);
    }
    
    if (log_level == "debug") {
        ROS_DEBUG("Got parameter '%s' = %s", param_name.c_str(), 
                  boost::lexical_cast<std::string>(value).c_str());
        
        // 显示相关参数以帮助调试
        std::vector<std::string> param_names;
        nh.getParamNames(param_names);
        
        std::string base_name = param_name.substr(param_name.find_last_of('/') + 1);
        std::vector<std::string> matching_params;
        
        for (const auto& name : param_names) {
            if (name.find(base_name) != std::string::npos) {
                matching_params.push_back(name);
            }
        }
        
        if (!matching_params.empty()) {
            ROS_DEBUG("Related parameters for '%s':", base_name.c_str());
            for (const auto& name : matching_params) {
                ROS_DEBUG("  %s", name.c_str());
            }
        }
    } else {
        ROS_INFO("Got parameter '%s' = %s", param_name.c_str(), 
                 boost::lexical_cast<std::string>(value).c_str());
    }
    
    return value;
}

/**
 * @brief 获取带有完整名称的参数，并在失败时打印参数名称。
 * 
 * @param nh NodeHandle。
 * @param relative_param_name 相对参数名。
 * @param value 用于存储参数值的变量。
 * @return 如果成功获取参数则返回 true，否则返回 false。
 */
template <typename T>
bool getParamWithFullName(ros::NodeHandle& nh, const std::string& relative_param_name, T& value) {
    std::string full_param_name = nh.resolveName(relative_param_name);
    if (nh.getParam(full_param_name, value)) {
        return true;
    } else {
        ROS_ERROR("Failed to get param '%s'", full_param_name.c_str());
        return false;
    }
}

/**
 * @brief 检查ROS是否仍在运行，如果ROS已关闭则打印错误消息并退出程序。
 *        这是一个主动的检查，比被动等待ROS事件更可靠。
 */
inline void checkIfRosIsRunning() {
    if (!ros::ok()) {
        ROS_FATAL("ROS is not running! Exiting...");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief 创建一个带有时间戳的 ROS 消息头。
 * 
 * @param frame_id 消息的 frame_id。
 * @return 创建的 std_msgs::Header 对象。
 */
inline std_msgs::Header createHeader(const std::string& frame_id = "") {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    return header;
}

/**
 * @brief 安全地发布 ROS 消息，并在发布失败时打印错误消息。
 *        现在会在订阅者状态变化时打印信息，并在没有订阅者时不发布消息。
 *
 * @param publisher ROS 发布者对象。
 * @param msg 要发布的消息。
 */
template <typename T>
void safePublish(const ros::Publisher& publisher, const T& msg) {
    static std::map<std::string, std::pair<bool, int>> pub_status;
    static std::mutex status_mutex;
    std::string topic_name = publisher.getTopic();
    int current_subscribers = publisher.getNumSubscribers();
    bool is_publishing = false;
    int last_subscriber_count = 0;

    {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (pub_status.find(topic_name) == pub_status.end()) {
            pub_status[topic_name] = std::make_pair(false, 0);
        }
        is_publishing = pub_status[topic_name].first;
        last_subscriber_count = pub_status[topic_name].second;
        pub_status[topic_name].second = current_subscribers;
    }

    if (current_subscribers > 0) {
        if (!is_publishing) {
            ROS_INFO("Start publishing topic '%s', subscribers count: %d", 
                    topic_name.c_str(), current_subscribers);
            {
                std::lock_guard<std::mutex> lock(status_mutex);
                pub_status[topic_name].first = true;
            }
        }
        try {
            publisher.publish(msg);
        } catch (const ros::serialization::StreamOverrunException& e) {
            ROS_ERROR_STREAM("Serialization error while publishing message on topic '" 
                            << topic_name << "': " << e.what());
        }
    } else {
        if (is_publishing) {
            ROS_WARN("No subscribers for topic '%s', stop publishing.", topic_name.c_str());
            {
                std::lock_guard<std::mutex> lock(status_mutex);
                pub_status[topic_name].first = false;
            }
        }
    }
}

/**
 * @brief 从参数向量创建字符串向量。
 *
 * @param nh NodeHandle。
 * @param param_name 参数名称。
 * @return 包含参数向量元素的 std::vector<std::string>。如果参数不存在或不是字符串向量，则返回空向量。
 */
inline std::vector<std::string> getStringVectorFromParam(ros::NodeHandle& nh, 
                                                        const std::string& param_name) {
    std::vector<std::string> result;
    XmlRpc::XmlRpcValue param_list;

    if (getParamWithFullName(nh, param_name, param_list)) {
        if (param_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int32_t i = 0; i < param_list.size(); ++i) {
                if (param_list[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                    result.push_back(static_cast<std::string>(param_list[i]));
                } else {
                    ROS_WARN("Element %d of parameter '%s' is not a string.", 
                            i, nh.resolveName(param_name).c_str());
                    return std::vector<std::string>();
                }
            }
        } else {
            ROS_ERROR("Parameter '%s' is not a list.", nh.resolveName(param_name).c_str());
        }
    }
    return result;
}

}  // namespace utils_core
}  // namespace in2ulv_cores

#endif  // ROS_UTILS_HPP