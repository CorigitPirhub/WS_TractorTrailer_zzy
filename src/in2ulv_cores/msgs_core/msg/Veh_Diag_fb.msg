# 车辆诊断反馈信息
uint8 Veh_fb_FaultLevel                # 车辆故障等级
bool Veh_fb_AutoCANCtrlCmd             # 自动CAN控制命令状态
bool Veh_fb_AutoIOCANCmd               # 自动IO CAN命令状态
bool Veh_fb_EPSDisOnline               # EPS(电子助力转向)通信状态
bool Veh_fb_EPSfault                   # EPS故障标志
bool Veh_fb_EPSMosfetOT                # EPS MOSFET过温标志
bool Veh_fb_EPSWarning                 # EPS预警标志
bool Veh_fb_EPSDisWork                 # EPS工作状态
bool Veh_fb_EPSOverCurrent             # EPS过流标志
bool Veh_fb_EHBecuFault                # EHB(电子液压制动)ECU故障
bool Veh_fb_EHBDisOnline               # EHB通信状态
bool Veh_fb_EHBWorkModelFault          # EHB工作模式故障
bool Veh_fb_EHBDisEn                   # EHB使能状态
bool Veh_fb_EHBAnguleFault             # EHB角度故障
bool Veh_fb_EHBOT                      # EHB过温标志
bool Veh_fb_EHBPowerFault              # EHB电源故障
bool Veh_fb_EHBsensorAbnomal           # EHB传感器异常
bool Veh_fb_EHBMotorFault              # EHB电机故障
bool Veh_fb_EHBOilPressSensorFault     # EHB油压传感器故障
bool Veh_fb_EHBOilFault                # EHB油液故障
uint8 Veh_fb_LDrvMCUFault              # 左驱动电机控制单元故障状态
uint8 Veh_fb_RDrvMCUFault              # 右驱动电机控制单元故障状态
bool Veh_fb_AUXBMSDisOnline            # 辅助BMS通信状态
bool Veh_fb_AuxScram                   # 辅助急停状态
bool Veh_fb_AuxRemoteClose             # 辅助远程关闭状态
bool Veh_fb_AuxRemoteDisOnline         # 辅助远程通信状态