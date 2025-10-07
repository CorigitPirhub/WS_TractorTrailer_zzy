cmake_minimum_required(VERSION 3.0.2)
project(in2ulv_controller)

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
  message_filters
  message_runtime
  dynamic_reconfigure
  utils_core
)

generate_dynamic_reconfigure_options(
  config/PIDControl.cfg
)

# 设置catkin包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES in2ulv_controller
  CATKIN_DEPENDS 
    msgs_core 
    supervisor_core
    roscpp 
    rospy 
    serial 
    std_msgs
    utils_core
  DEPENDS 
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 声明库
add_library(${PROJECT_NAME}
  src/ChassisCmdGenerator.cpp
  src/MotionProcessor.cpp
)

# 添加库依赖
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# 声明可执行文件
add_executable(chassis_cmd_node
  src/chassis_cmd_node.cpp
)

add_executable(motion_node
  src/motion_node.cpp
)

# 添加可执行文件依赖
add_dependencies(chassis_cmd_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(motion_node
  ${catkin_EXPORTED_TARGETS}
)

# 链接库到可执行文件
target_link_libraries(chassis_cmd_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(motion_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装可执行文件
install(TARGETS chassis_cmd_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
