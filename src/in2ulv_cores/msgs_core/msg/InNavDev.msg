Header header

float32   PowerTime    # 上电时间，0.001s
float32   WorkTime     # 工作时间，0.001s
uint8    WorkStatus    # 工作状态
uint8    NaviStatus    # 导航状态
uint32   SysDevStatus  # 故障字

InNavDev_ConvData ConvData  # 组合导航信息
InNavDev_ImuData  ImuData   # IMU导航信息
InNavDev_SatData  SatData   # 卫星导航信息