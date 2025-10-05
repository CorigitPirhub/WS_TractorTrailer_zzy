float32 acc_x #g
float32 acc_y
float32 acc_z


float32 gyro_x #deg/s
float32 gyro_y
float32 gyro_z

float32 pitch  #deg
float32 roll
float32 heading

uint32 time    #ms
float32 height #m
float64 lat    #deg
float64 lon    #deg 

float32 north_spd   #m/s
float32 east_spd    #m/s
float32 ground_spd  #m/s

uint8 gps_flag_pos
uint8 sat_num        #卫星数

uint8 gps_flag_heading
uint8 gps_age

uint8 car_status #有轮速为1
uint8 ins_status #0:none  1、姿态初始化 2、组合导航

float32 std_lat #标准差
float32 std_lon
float32 std_height
float32 std_heading