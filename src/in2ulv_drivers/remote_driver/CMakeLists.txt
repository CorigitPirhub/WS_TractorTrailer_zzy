cmake_minimum_required(VERSION 3.0.2)
project(remote_driver)

# 使用C++17标准
add_compile_options(-std=c++17)

# 查找catkin和依赖项
find_package(catkin REQUIRED COMPONENTS
  msgs_core
  supervisor_core
  roscpp
  rospy
  serial
  std_msgs
  message_runtime
  utils_core
)

# 设置catkin包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    RemoteController
    ActuatorController
  CATKIN_DEPENDS 
    msgs_core
    supervisor_core 
    roscpp 
    rospy 
    serial 
    std_msgs
    utils_core
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 声明库
add_library(RemoteController
  src/RemoteController.cpp
)

add_library(ActuatorController
  src/ActuatorController.cpp
)

# 添加库依赖
add_dependencies(RemoteController
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(ActuatorController
  ${catkin_EXPORTED_TARGETS}
)

# 声明可执行文件
add_executable(remote_node
  src/remote_node.cpp
)

# 添加可执行文件依赖
add_dependencies(remote_node
  ${catkin_EXPORTED_TARGETS}
)

# 链接库到可执行文件
target_link_libraries(remote_node
  RemoteController
  ActuatorController
  ${catkin_LIBRARIES}
)

# 安装库文件
install(TARGETS 
  RemoteController
  ActuatorController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装可执行文件
install(TARGETS remote_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/remote_driver/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)