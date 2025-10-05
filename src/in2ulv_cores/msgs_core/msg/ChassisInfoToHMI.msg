# 帧头，包含时间戳信息
std_msgs/Header header

string vin # 字符串类型, 16 ~ 32个字符， 读取配置文件获取，列如: LK6ADAE21MB100250
string current_stop # 字符串类型, 16 ~ 32个字符, 列如: d0a0d333931203a,
string route_stop # 字符串类型,16 ~ 32个字符, 列如: d0a0d333931203a

uint64 route_line # 路线

uint8 control_mode # 车辆控制模式: 0为自动模式， 1为手动模式

uint16 veh_status  # 0: 无服务请求, 1: 未准备完毕, 2: 准备完毕, 3: 正常行驶, 4: 到达目的地, 5: 服务失败(未到达目的地), 6: 车辆已经暂停, 7: 避障停车

float64[2] gps_veh # 车辆位置经纬度，2个dobule类型的数组, 第一个是车辆位置的纬度， 第二个是车辆位置的经度
float64[2] gps_stop # 车辆目标站点经纬度，2个dobule类型的数组, 第一个是车辆位置的纬度， 第二个是车辆位置的经度

float64 trip_distance     # 行程总里程, 距离站点总里程, 单位: m
float64 trip_eta          # 行程剩余时间, 到达站点预计剩余时间, 单位: ms 
float64 trip_left_mileage # 行程剩余里程, 到达站点剩余里程, 单位: m

uint8 SysPowMode  # 整车电源模式, ACAN的0x565, 0x0: OFF, 0x1: ACC, 0x2: ON
uint8 HzrdLtIO    # 双闪状态, 0: 关闭, 1: 开启
uint8 WindscenWipSt  # 雨刮状态, 0: 关闭, 1: 开启 
uint8 Highbeem       # 远关灯状态, 0: 关闭, 1: 开启
uint8 Lowbeem        # 近光灯状态, 0: 关闭, 1: 开启
uint8 Windows        # 车窗状态, 0: 升起, 1: 落下
uint8 Doorlock       # 门锁状态, 0: 锁车, 1: 解锁
uint8 DoorSwitch     # 门锁开关, 0: 关闭, 1: 未关闭
uint8 EmergencyButton# 按钮状态: 按下为1， 不按下为0

float64 speed	      # 车辆当前速度, IDU从0x36A获取到的车速, 单位m/s
float64 angle          # 车辆转向角度, IDU从0x1E5获取到的转角值, 单位度
float64 soc            # IDU从0x36A获取到的SOC电池值, 单位%
uint16 event         # 故障事件上报, 具体故障事件定义和含义由算法给出

