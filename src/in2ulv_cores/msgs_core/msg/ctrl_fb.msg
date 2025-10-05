# 控制状态反馈信息
uint8    ctrl_fb_gear       # 档位命令：1-驻车P档；2-后退档R；3-空档N；4-前进档D；5-电脑控制
float32  ctrl_fb_velocity   # 速度反馈，单位：m/s
float32  ctrl_fb_steering   # 转向角反馈，单位：度
uint8    ctrl_fb_Brake      # 制动状态反馈
uint8    ctrl_fb_mode       # 控制模式反馈：0-手动模式，1-自动模式
