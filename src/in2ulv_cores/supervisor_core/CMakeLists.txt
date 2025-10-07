cmake_minimum_required(VERSION 3.0.2)
project(supervisor_core)

# 使用C++17标准
add_compile_options(-std=c++17)

# 查找catkin和依赖项
find_package(catkin REQUIRED COMPONENTS
  msgs_core
  roscpp
  rospy
  roslib
  std_msgs
)

find_package(yaml-cpp REQUIRED)

# 设置catkin包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    InterfaceContract
    MonitoringAgent
    CentralMonitor
  CATKIN_DEPENDS 
    msgs_core 
    roscpp 
    rospy 
    roslib
    std_msgs
  DEPENDS yaml-cpp
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# 声明库
add_library(InterfaceContract
  src/InterfaceContract.cpp
)

add_library(MonitoringAgent
  src/MonitoringAgent.cpp
)

add_library(CentralMonitor
  src/CentralMonitor.cpp
)

# 添加库依赖
add_dependencies(InterfaceContract
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(MonitoringAgent
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(CentralMonitor
  ${catkin_EXPORTED_TARGETS}
)

# 声明可执行文件
add_executable(supervisor_node
  src/supervisor_node.cpp
)

# 添加可执行文件依赖
add_dependencies(supervisor_node
  ${catkin_EXPORTED_TARGETS}
)

# 链接库到可执行文件
target_link_libraries(supervisor_node
  InterfaceContract
  MonitoringAgent
  CentralMonitor
  ${catkin_LIBRARIES}
  yaml-cpp
)

# 安装库文件
install(TARGETS 
  InterfaceContract
  MonitoringAgent
  CentralMonitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装可执行文件
install(TARGETS supervisor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/supervisor_core/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)