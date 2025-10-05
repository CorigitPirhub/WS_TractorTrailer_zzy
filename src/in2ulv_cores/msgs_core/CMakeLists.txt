cmake_minimum_required(VERSION 3.0.2)
project(msgs_core)

# 查找catkin和依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation  # 添加消息生成依赖
)

# 添加消息文件
add_message_files(
  FILES
  DrivingInfo.msg
  LocationsInfo.msg
  ChassisInfoToHMI.msg
  remote_info.msg
  force_info.msg
  ctrl_fb.msg
  INSStdMsg.msg
  bms_Infor.msg
  ActionType.msg
  torque_cmd.msg
  InNavDev.msg
  Trajectory.msg
  Veh_Diag_fb.msg
  InNavDev_ConvData.msg
  angle_error.msg
  ModuleInfo.msg
  TrajectoryPoint.msg
  torque_fb.msg
  io_cmd.msg
  current_fb.msg
  car_state.msg
  TboxMsg.msg
  GisInfo.msg
  bms_flag_Infor.msg
  InNavDev_SatData.msg
  vision_trailer.msg
  Drive_MCUEcoder_fb.msg
  angles.msg
  rr_wheel_fb.msg
  UCURequestMsg.msg
  odo_fb.msg
  HMIStepReport.msg
  io_fb.msg
  ctrl_cmd.msg
  ackermann_cmd.msg
  LocalizationInfo.msg
  InNavDev_ImuData.msg
  LatticeDebug.msg
  lr_wheel_fb.msg
  ins570.msg
)

# 添加服务文件（如果有）
# file(GLOB srv_files "srv/*.srv")
# add_service_files(FILES ${srv_files})

# 生成消息和服务
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 设置catkin包信息
catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    roscpp 
    rospy 
    std_msgs
  DEPENDS 
)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)
