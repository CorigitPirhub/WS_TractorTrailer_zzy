#ifndef CENTRALMONITOR_HPP
#define CENTRALMONITOR_HPP

#include <ros/ros.h>
#include <map>
#include <string>
#include "supervisor_core/InterfaceContract.hpp"
#include "msgs_core/MonitoringMetric.h"
#include "msgs_core/AlertMessage.h"

namespace in2ulv_cores {
namespace supervisor_core {

class CentralMonitor {
public:
    CentralMonitor(ros::NodeHandle& nh, InterfaceManager& interface_manager);
    
private:
    void metricCallback(const msgs_core::MonitoringMetric::ConstPtr& msg);
    void checkAnomalies();
    
    ros::NodeHandle nh_;
    InterfaceManager& interface_manager_;
    ros::Subscriber metric_sub_;
    ros::Timer check_timer_;
    ros::Publisher alert_pub_;
    
    // 存储最近一次接收到的指标（按节点、话题、指标名）
    std::map<std::string, std::map<std::string, std::map<std::string, msgs_core::MonitoringMetric>>> latest_metrics_;
    // 记录每个节点状态最后更新时间
    std::map<std::string, ros::Time> last_status_time_;
};

}  // namespace supervisor_core
}  // namespace in2ulv_cores

#endif  // CENTRALMONITOR_HPP