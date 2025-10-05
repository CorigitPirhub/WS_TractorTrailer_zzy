cmake_minimum_required(VERSION 3.0.2)
project(utils_core)

# 使用C++17标准
add_compile_options(-std=c++17)

# 查找catkin和依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# 设置catkin包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_core
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS 
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 声明库
add_library(${PROJECT_NAME}
  src/modbus_crc.cpp
  src/automotive_pid.cpp
  src/spline_interpolator.cpp
  src/filter.cpp
)

# 目标链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装库文件
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# 安装导出文件
install(DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
