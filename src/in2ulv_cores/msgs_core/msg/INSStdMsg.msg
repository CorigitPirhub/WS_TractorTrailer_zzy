# 帧头，包含时间戳信息
std_msgs/Header header

# 模块类型
uint8 module_type

# error_code
uint32 error_code

# 模块信息
string ins_data

# ROS 标准信息
sensor_msgs/Imu imu
sensor_msgs/NavSatFix nav_sat_fix

# INS_Acc (0X500)
float64 ACC_X
float64 ACC_Y
float64 ACC_Z

# INS_GYRO (0X501)
float64 GYRO_X
float64 GYRO_Y
float64 GYRO_Z

# INS_HeadingPitchRoll (0X502)
float64 INS_PitchAngle
float64 INS_RollAngle
float64 INS_HeadingAngle

# INS_HeightAndTime (0X503)
float64 INS_LocatHeight
uint32 INS_Time

# INS_LatitudeLongitude (0X504)
float64 INS_Latitude
float64 INS_Longitude

# INS_Speed (0X505)
float64 INS_NorthSpd
float64 INS_EastSpd
float64 INS_ToGroundSpd

# INS_DataInfo (0X506)
uint8 INS_GpsFlag_Pos
uint8 INS_NumSV     #星数
uint8 INS_GpsFlag_Heading
uint8 INS_Gps_Age
uint8 INS_Car_Status
uint8 INS_Status 

# INS_Std (0X507)
float32 INS_Std_Lat
float32 INS_Std_Lon
float32 INS_Std_LocatHeight
float32 INS_Std_Heading
