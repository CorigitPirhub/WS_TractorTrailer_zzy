# 帧头，包含时间戳信息
std_msgs/Header header

# 任务步骤反馈
# FULL_SUCCESS 或 PART_SUCCESS
uint8 step_report

# 如果完成的是带有 station id 的 HEAD_FOR_GOAL 任务，则返回正确的 station id
uint32 station_id