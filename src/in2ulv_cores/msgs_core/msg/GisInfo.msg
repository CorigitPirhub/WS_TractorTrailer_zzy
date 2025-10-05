std_msgs/Header header
#行驶模式标志,0停止，1行驶
uint8 status
#转向灯,0不闪，1右转向灯闪，2左转向灯闪
uint8 indicator_status
#档位，0空挡，1为D档，2为R档，3为P档
uint8 gear
#GIS局部路径导航点，输出原点为（1500，1000）
float64[301] x
float64[301] y
#-PI左偏到PI右偏
float64[301] yaw
#对应导航点期望速度
float64[301] speed

