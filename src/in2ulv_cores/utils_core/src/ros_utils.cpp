// utils_core/src/ros_utils.cpp
#include "utils_core/ros_utils.hpp"

namespace in2ulv_cores {
namespace utils_core {

// 实际定义全局变量
std::unique_ptr<supervisor_core::MonitoringAgent> global_monitor_agent = nullptr;

}  // namespace utils_core
}  // namespace in2ulv_cores