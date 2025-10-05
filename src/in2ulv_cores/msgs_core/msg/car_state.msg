float32 x
float32 y

float32 yaw

float32 v0
float32 steer

float32 H_behind_B_yaw #后车B点与前车H点之间的夹角，以前车H点为坐标原点

int8 multi
int8 manual #-1:手动； 1：自动
int8 linked #-1：卫星靠近-视觉对接；  0：已对接；  1：分离中-已分离
int8 visual #visual=0为路径跟踪，visual=1为近距离视觉引导对接，visual=2为解体过程