# UCU_Requestment_message_4 (0x53E)

# 帧头，包含时间戳信息
std_msgs/Header header
# 模块类型
uint8 module_type

# error_code
uint32 error_code

# UCU数据有效性
bool ucu_validity
string route_encoder

# 版本号
uint8 Version
# 地区信息：省份、城市、区县、街道、校区
uint8 Province
uint8 City
uint8 District
uint8 Street
uint8 Community
# 功能请求
uint8 FeatureRequest
# 车头朝向
uint8 CarHeading
# 类型
uint8 XXType
# 地图编号
uint16 MapNumber
# 站点编号
uint8 StationNumber
# 远程控制命令
uint8 RemoteCtrlCommand
