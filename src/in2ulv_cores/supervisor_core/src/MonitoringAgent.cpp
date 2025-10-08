#include "supervisor_core/MonitoringAgent.hpp"
#include "msgs_core/MonitoringMetric.h"

namespace in2ulv_cores {
namespace supervisor_core {

MonitoringAgent::MonitoringAgent(ros::NodeHandle& nh, const std::string& node_name)
    : nh_(nh), node_name_(node_name), node_status_(0.0) {
    metrics_pub_ = nh_.advertise<msgs_core::MonitoringMetric>("/monitoring_metrics", 10);
    last_publish_time_ = ros::Time::now();
}

void MonitoringAgent::recordPublish(const std::string& topic) {
    boost::mutex::scoped_lock lock(mutex_);
    publish_counts_[topic]++;
}

void MonitoringAgent::recordSubscribe(const std::string& topic) {
    boost::mutex::scoped_lock lock(mutex_);
    subscribe_counts_[topic]++;
}

void MonitoringAgent::setNodeName(const std::string& node_name) {
    boost::mutex::scoped_lock lock(mutex_);
    node_name_ = node_name;
}

void MonitoringAgent::setMessageType(const std::string& msg_type) {
    boost::mutex::scoped_lock lock(mutex_);
    msg_type_ = msg_type;
}

void MonitoringAgent::setNodeStatus(double status) {
    boost::mutex::scoped_lock lock(mutex_);
    node_status_ = status;
}

void MonitoringAgent::publishMetrics() {
    // 复制数据，减少锁持有时间
    std::map<std::string, int> publish_copy;
    std::map<std::string, int> subscribe_copy;
    double status_copy;
    ros::Time now = ros::Time::now();
    double duration;
    {
        boost::mutex::scoped_lock lock(mutex_);
        duration = (now - last_publish_time_).toSec();
        last_publish_time_ = now;
        publish_copy = std::move(publish_counts_);
        subscribe_copy = std::move(subscribe_counts_);
        status_copy = node_status_;
        publish_counts_.clear();
        subscribe_counts_.clear();
    }
    
    // 发布节点状态
    msgs_core::MonitoringMetric status_metric;
    status_metric.node_name = node_name_;
    status_metric.topic = "";
    status_metric.msg_type = "";
    status_metric.metric_name = "node_status";
    status_metric.metric_type = "";
    status_metric.value = status_copy;
    status_metric.stamp = now; // 设置时间戳
    metrics_pub_.publish(status_metric);
    
    // 发布每个话题的发布频率
    for (const auto& pub : publish_copy) {
        msgs_core::MonitoringMetric metric;
        metric.node_name = node_name_;
        metric.topic = pub.first;
        metric.msg_type = msg_type_;
        metric.metric_name = "publish_frequency";
        metric.metric_type = "publish";
        metric.value = (duration > 0) ? (pub.second / duration) : 0;
        metric.stamp = now;
        metrics_pub_.publish(metric);
    }
    
    // 发布每个话题的订阅频率
    for (const auto& sub : subscribe_copy) {
        msgs_core::MonitoringMetric metric;
        metric.node_name = node_name_;
        metric.topic = sub.first;
        metric.msg_type = msg_type_;
        metric.metric_name = "subscribe_frequency";
        metric.metric_type = "subscribe";
        metric.value = (duration > 0) ? (sub.second / duration) : 0;
        metric.stamp = now;
        metrics_pub_.publish(metric);
    }
}

}  // namespace supervisor_core
}  // namespace in2ulv_cores