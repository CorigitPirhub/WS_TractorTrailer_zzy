#ifndef MONITORINGAGENT_HPP
#define MONITORINGAGENT_HPP

#include <ros/ros.h>
#include <string>
#include <map>
#include <boost/thread.hpp>
#include "supervisor_core/InterfaceContract.hpp"
#include "msgs_core/MonitoringMetric.h"

namespace in2ulv_cores {
namespace supervisor_core {
/**
 * @brief 监控代理类
 * @details 用于节点内部监控，包括记录发布/订阅频率、节点状态等，并定期发布监控指标
 */
class MonitoringAgent {
public:
    MonitoringAgent(ros::NodeHandle& nh, const std::string& node_name);
    
    // 记录一次消息发布/订阅
    void recordPublish(const std::string& topic);
    void recordSubscribe(const std::string& topic);
    
    // 设置节点状态
    void setNodeStatus(double status);
    
    // 获取节点名称
    std::string getNodeName() const { return node_name_; }
    
    // 发布监控指标
    void publishMetrics();

private:
    ros::NodeHandle nh_;
    std::string node_name_;
    ros::Publisher metrics_pub_;
    
    // 用于频率统计
    std::map<std::string, int> publish_counts_;
    std::map<std::string, int> subscribe_counts_;
    double node_status_;
    ros::Time last_publish_time_;
    
    // 互斥锁
    boost::mutex mutex_;
};

}  // namespace supervisor_core
}  // namespace in2ulv_cores

#endif  // MONITORINGAGENT_HPP