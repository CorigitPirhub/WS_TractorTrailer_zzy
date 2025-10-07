#include "supervisor_core/CentralMonitor.hpp"
#include "msgs_core/AlertMessage.h"
#include <ros/time.h>

namespace in2ulv_cores {
namespace supervisor_core {

CentralMonitor::CentralMonitor(ros::NodeHandle& nh, InterfaceManager& interface_manager)
    : nh_(nh), interface_manager_(interface_manager) {
    metric_sub_ = nh_.subscribe("/monitoring_metrics", 100, &CentralMonitor::metricCallback, this);
    alert_pub_ = nh_.advertise<msgs_core::AlertMessage>("/system_alerts", 10);
    check_timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&CentralMonitor::checkAnomalies, this));
}

void CentralMonitor::metricCallback(const msgs_core::MonitoringMetric::ConstPtr& msg) {
    // 存储指标
    latest_metrics_[msg->node_name][msg->topic][msg->metric_name] = *msg;
    
    // 如果是节点状态，记录时间
    if (msg->metric_name == "node_status") {
        last_status_time_[msg->node_name] = ros::Time::now();
    }
}

void CentralMonitor::checkAnomalies() {
    ros::Time now = ros::Time::now();
    
    // 检查节点状态超时
    for (auto& node_time : last_status_time_) {
        if ((now - node_time.second).toSec() > 5.0) { // 5秒超时
            msgs_core::AlertMessage alert;
            alert.severity = msgs_core::AlertMessage::ERROR;
            alert.source = node_time.first;
            alert.message = "Node status timeout";
            alert.error_code = 1001; // 自定义错误码
            alert_pub_.publish(alert);
        }
    }
    
    // 遍历所有节点
    for (auto& node : latest_metrics_) {
        std::string node_name = node.first;
        
        // 遍历该节点的所有话题
        for (auto& topic : node.second) {
            std::string topic_name = topic.first;
            
            // 遍历该话题下的所有指标
            for (auto& metric : topic.second) {
                const msgs_core::MonitoringMetric& msg = metric.second;
                
                // 检查节点状态
                if (metric.first == "node_status") {
                    double status = msg.value;
                    if (status > 0.0) {
                        msgs_core::AlertMessage alert;
                        alert.severity = (status == 1.0) ? msgs_core::AlertMessage::WARNING : msgs_core::AlertMessage::ERROR;
                        alert.source = node_name;
                        alert.message = "Node status abnormal: " + std::to_string(status);
                        alert.error_code = (status == 1.0) ? 2001 : 2002;
                        alert_pub_.publish(alert);
                    }
                }
                
                // 检查话题频率
                if (metric.first.find("frequency") != std::string::npos) {
                    double frequency = msg.value;
                    const InterfaceContract* contract = interface_manager_.getContract(topic_name);
                    if (contract) {
                        if (!contract->validateFrequency(frequency)) {
                            msgs_core::AlertMessage alert;
                            alert.severity = msgs_core::AlertMessage::WARNING;
                            alert.source = node_name;
                            alert.message = "Frequency violation on topic " + topic_name + 
                                           ": actual=" + std::to_string(frequency) + 
                                           ", min=" + std::to_string(contract->frequency.min_hz) +
                                           ", max=" + std::to_string(contract->frequency.max_hz);
                            alert.error_code = 3001;
                            alert_pub_.publish(alert);
                        }
                    }
                }
            }
        }
    }
}

}  // namespace supervisor_core
}  // namespace in2ulv_cores