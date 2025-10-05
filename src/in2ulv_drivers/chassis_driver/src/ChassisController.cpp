#include "chassis_driver/ChassisController.hpp"

namespace in2ulv_drivers {
namespace chassis_driver {

ChassisController::ChassisController(ros::NodeHandle& nh) 
    : nh_(nh), angle_offset_front_(0.0), angle_offset_rear_(0.0) {
    
    // 从参数服务器获取角度偏移
    if (!in2ulv_cores::utils_core::getParamWithFullName(nh_, "chassis_model_param/angle_offsetparam/offset_f", angle_offset_front_)) {
        ROS_FATAL("Failed to get critical parameter offset_f. Exiting.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Angle_offsetparam/offset_f: %f", angle_offset_front_);

    if (!in2ulv_cores::utils_core::getParamWithFullName(nh_, "chassis_model_param/angle_offsetparam/offset_r", angle_offset_rear_)) {
        ROS_FATAL("Failed to get critical parameter offset_r. Exiting.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Angle_offsetparam/offset_r: %f", angle_offset_rear_);
    
    // 初始化发布者和订阅者
    angles_pub_ = nh_.advertise<msgs_core::angles>("angles", 5);
    ctrl_fb_pub_ = nh_.advertise<msgs_core::ctrl_fb>("ctrl_fb", 5);
    lr_wheel_fb_pub_ = nh_.advertise<msgs_core::lr_wheel_fb>("lr_wheel_fb", 5);
    rr_wheel_fb_pub_ = nh_.advertise<msgs_core::rr_wheel_fb>("rr_wheel_fb", 5);
    io_fb_pub_ = nh_.advertise<msgs_core::io_fb>("io_fb", 5);
    odo_fb_pub_ = nh_.advertise<msgs_core::odo_fb>("odo_fb", 5);
    bms_infor_pub_ = nh_.advertise<msgs_core::bms_Infor>("bms_Infor", 5);
    bms_flag_infor_pub_ = nh_.advertise<msgs_core::bms_flag_Infor>("bms_flag_Infor", 5);
    drive_mcu_encoder_pub_ = nh_.advertise<msgs_core::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb", 5);
    veh_diag_fb_pub_ = nh_.advertise<msgs_core::Veh_Diag_fb>("Veh_Diag_fb", 5);
    torque_fb_pub_ = nh_.advertise<msgs_core::torque_fb>("torque_fb", 5);
    current_fb_pub_ = nh_.advertise<msgs_core::current_fb>("current_fb", 5);
    
    ctrl_cmd_sub_ = nh_.subscribe("ctrl_cmd", 5, &ChassisController::ctrlCmdCallback, this);
    io_cmd_sub_ = nh_.subscribe("io_cmd", 5, &ChassisController::ioCmdCallback, this);
    torque_cmd_sub_ = nh_.subscribe("torque_cmd", 5, &ChassisController::torqueCmdCallback, this);
}

bool ChassisController::initializeCAN() {
    // 创建CAN套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
        return false;
    }
    
    // 设置CAN接口
    struct ifreq ifr;
    std::string can_interface = "can0";
    strcpy(ifr.ifr_name, can_interface.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        ROS_ERROR("Failed to get CAN interface index: %s", strerror(errno));
        close(can_socket_);
        return false;
    }
    
    // 绑定套接字
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Failed to bind CAN socket: %s", strerror(errno));
        close(can_socket_);
        return false;
    }
    
    ROS_INFO("CAN interface initialized successfully");
    return true;
}

void ChassisController::ioCmdCallback(const msgs_core::io_cmd::ConstPtr& msg) {
    boost::lock_guard<boost::mutex> lock(cmd_mutex_);
    
    unsigned char send_data[8] = {0};
    send_data[0] = msg->io_cmd_enable;
    send_data[1] = 0xff;
    
    // 设置灯光状态
    if (!msg->io_cmd_lower_beam_headlamp) send_data[1] &= 0xfe;
    if (!msg->io_cmd_upper_beam_headlamp) send_data[1] &= 0xfd;
    send_data[1] |= msg->io_cmd_turn_lamp << 2;
    if (!msg->io_cmd_braking_lamp) send_data[1] &= 0xef;
    if (!msg->io_cmd_clearance_lamp) send_data[1] &= 0xdf;
    if (!msg->io_cmd_fog_lamp) send_data[1] &= 0xcf;
    
    send_data[2] = msg->io_cmd_speaker;
    
    // 计算校验和
    static unsigned char count = 0;
    count = (count + 1) % 16;
    send_data[6] = count << 4;
    send_data[7] = send_data[0] ^ send_data[1] ^ send_data[2] ^ send_data[3] ^ 
                   send_data[4] ^ send_data[5] ^ send_data[6];
    
    // 设置发送帧
    send_frame_.can_id = 0x98C4D7D0;
    send_frame_.can_dlc = 8;
    memcpy(send_frame_.data, send_data, 8);
    
    // 发送数据
    if (write(can_socket_, &send_frame_, sizeof(send_frame_)) <= 0) {
        ROS_ERROR("Failed to send IO command: %s", strerror(errno));
    }
}

void ChassisController::ctrlCmdCallback(const msgs_core::ctrl_cmd::ConstPtr& msg) {
    boost::lock_guard<boost::mutex> lock(cmd_mutex_);
    
    unsigned short velocity = static_cast<unsigned short>(msg->ctrl_cmd_velocity * 1000);
    short steering = static_cast<short>(msg->ctrl_cmd_steering * 100);
    unsigned char send_data[8] = {0};
    
    // 设置控制数据
    send_data[0] = (0x0f & msg->ctrl_cmd_gear) | ((velocity & 0x0f) << 4);
    send_data[1] = (velocity >> 4) & 0xff;
    send_data[2] = (0x0f & (velocity >> 12)) | ((steering & 0x0f) << 4);
    send_data[3] = (steering >> 4) & 0xff;
    send_data[4] = ((msg->ctrl_cmd_Brake & 0x0f) << 4) | (0x0f & (steering >> 12));
    
    // 计算校验和
    static unsigned char count = 0;
    count = (count + 1) % 16;
    send_data[6] = count << 4;
    send_data[7] = send_data[0] ^ send_data[1] ^ send_data[2] ^ send_data[3] ^ 
                   send_data[4] ^ send_data[5] ^ send_data[6];
    
    // 设置发送帧
    send_frame_.can_id = 0x98C4D2D0;
    send_frame_.can_dlc = 8;
    memcpy(send_frame_.data, send_data, 8);
    
    // 发送数据
    if (write(can_socket_, &send_frame_, sizeof(send_frame_)) <= 0) {
        ROS_ERROR("Failed to send control command: %s", strerror(errno));
    }
}

void ChassisController::torqueCmdCallback(const msgs_core::torque_cmd::ConstPtr& msg) {
    boost::lock_guard<boost::mutex> lock(cmd_mutex_);
    
    short torque_lr = static_cast<short>(msg->lr_torque_cmd * 1000);
    short torque_rr = static_cast<short>(msg->rr_torque_cmd * 1000);
    unsigned char send_data[8] = {0};
    
    // 设置扭矩数据
    send_data[0] = torque_lr & 0xff;
    send_data[1] = (torque_lr >> 8) & 0xff;
    send_data[2] = torque_rr & 0xff;
    send_data[3] = (torque_rr >> 8) & 0xff;
    
    // 计算校验和
    static unsigned char count = 0;
    count = (count + 1) % 16;
    send_data[6] = count << 4;
    send_data[7] = send_data[0] ^ send_data[1] ^ send_data[2] ^ send_data[3] ^ 
                   send_data[4] ^ send_data[5] ^ send_data[6];
    
    // 设置发送帧
    send_frame_.can_id = 0x98C4D4D0;
    send_frame_.can_dlc = 8;
    memcpy(send_frame_.data, send_data, 8);
    
    // 发送数据
    if (write(can_socket_, &send_frame_, sizeof(send_frame_)) <= 0) {
        ROS_ERROR("Failed to send torque command: %s", strerror(errno));
    }
}

void ChassisController::receiveData() {
    msgs_core::angles angles_msg;
    
    while (ros::ok()) {
        ssize_t bytes_read = read(can_socket_, &recv_frame_, sizeof(recv_frame_));
        if (bytes_read < 0) {
            ROS_ERROR("Error reading from CAN: %s", strerror(errno));
            ros::Duration(0.1).sleep();
            continue;
        }
        
        switch (recv_frame_.can_id) {
            // 前编码器反馈
            case 0x01: 
                if (recv_frame_.data[0] == 0x07 && recv_frame_.data[2] == 0x01 && recv_frame_.data[1] == 0x01) {
                    uint32_t data = 0;
                    memcpy(&data, recv_frame_.data + 3, 4);
                    float angle = static_cast<float>(data) / 4096.0f * 0.5f * 360.0f + angle_offset_front_;
                    angles_msg.angle_f = in2ulv_cores::utils_core::normalize_angle(angle);
                }
                break;
                
            // 后编码器反馈
            case 0x02: 
                if (recv_frame_.data[0] == 0x07 && recv_frame_.data[2] == 0x01 && recv_frame_.data[1] == 0x02) {
                    uint32_t data = 0;
                    memcpy(&data, recv_frame_.data + 3, 4);
                    float angle = static_cast<float>(data) / 4096.0f * 0.75f * 360.0f + angle_offset_rear_;
                    angles_msg.angle_r = in2ulv_cores::utils_core::normalize_angle(angle);
                    in2ulv_cores::utils_core::safePublish(angles_pub_, angles_msg);
                }
                break;
                
            // 速度控制反馈
            case 0x98C4D2EF:
            {
                msgs_core::ctrl_fb msg;
                msg.ctrl_fb_gear = 0x0f & recv_frame_.data[0];
                msg.ctrl_fb_velocity = static_cast<float>((static_cast<unsigned short>(
                    (recv_frame_.data[2] & 0x0f) << 12 | 
                    recv_frame_.data[1] << 4 | 
                    (recv_frame_.data[0] & 0xf0) >> 4
                ))) / 1000.0f;
                msg.ctrl_fb_steering = static_cast<float>((static_cast<short>(
                    (recv_frame_.data[4] & 0x0f) << 12 | 
                    recv_frame_.data[3] << 4 | 
                    (recv_frame_.data[2] & 0xf0) >> 4
                ))) / 100.0f;
                msg.ctrl_fb_Brake = (recv_frame_.data[4] & 0x30) >> 4;
                msg.ctrl_fb_mode = (recv_frame_.data[4] & 0xc0) >> 6;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(ctrl_fb_pub_, msg);
                }
                break;
            }
            
            // 左轮反馈
            case 0x98C4D7EF:
            {
                msgs_core::lr_wheel_fb msg;
                msg.lr_wheel_fb_velocity = static_cast<float>(static_cast<short>(
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                )) / 1000.0f;
                msg.lr_wheel_fb_pulse = static_cast<int>(
                    recv_frame_.data[5] << 24 | recv_frame_.data[4] << 16 | 
                    recv_frame_.data[3] << 8 | recv_frame_.data[2]
                );
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(lr_wheel_fb_pub_, msg);
                }
                break;
            }
            
            // 右轮反馈
            case 0x98C4D8EF:
            {
                msgs_core::rr_wheel_fb msg;
                msg.rr_wheel_fb_velocity = static_cast<float>(static_cast<short>(
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                )) / 1000.0f;
                msg.rr_wheel_fb_pulse = static_cast<int>(
                    recv_frame_.data[5] << 24 | recv_frame_.data[4] << 16 | 
                    recv_frame_.data[3] << 8 | recv_frame_.data[2]
                );
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(rr_wheel_fb_pub_, msg);
                }
                break;
            }
            
            // IO反馈
            case 0x98C4DAEF:
            {
                msgs_core::io_fb msg;
                msg.io_fb_enable = (0x01 & recv_frame_.data[0]) ? true : false;
                msg.io_fb_turn_lamp = (0x0c & recv_frame_.data[1]) >> 2;
                msg.io_fb_braking_lamp = (0x10 & recv_frame_.data[1]) ? true : false;
                msg.io_fb_fm_impact_sensor = (0x02 & recv_frame_.data[3]) ? true : false;
                msg.io_fb_rm_impact_sensor = (0x10 & recv_frame_.data[3]) ? true : false;
                msg.io_fb_disCharge = (0x01 & recv_frame_.data[5]) ? true : false;
                msg.io_fb_chargeEn = (0x02 & recv_frame_.data[5]) ? true : false;
                msg.io_fb_ScramSt = (0x10 & recv_frame_.data[5]) ? true : false;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(io_fb_pub_, msg);
                }
                break;
            }
            
            // 里程计反馈
            case 0x98C4DEEF:
            {
                msgs_core::odo_fb msg;
                msg.odo_fb_accumulative_mileage = static_cast<float>(static_cast<int>(
                    recv_frame_.data[3] << 24 | recv_frame_.data[2] << 16 | 
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                )) / 1000.0f;
                msg.odo_fb_accumulative_angular = static_cast<float>(static_cast<int>(
                    recv_frame_.data[7] << 24 | recv_frame_.data[6] << 16 | 
                    recv_frame_.data[5] << 8 | recv_frame_.data[4]
                )) / 1000.0f;
                in2ulv_cores::utils_core::safePublish(odo_fb_pub_, msg);
                break;
            }
            
            // BMS信息反馈
            case 0x98C4E1EF:
            {
                msgs_core::bms_Infor msg;
                msg.bms_Infor_voltage = static_cast<float>(static_cast<unsigned short>(
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                )) / 100.0f;
                msg.bms_Infor_current = static_cast<float>(static_cast<short>(
                    recv_frame_.data[3] << 8 | recv_frame_.data[2]
                )) / 100.0f;
                msg.bms_Infor_remaining_capacity = static_cast<float>(static_cast<unsigned short>(
                    recv_frame_.data[5] << 8 | recv_frame_.data[4]
                )) / 100.0f;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(bms_infor_pub_, msg);
                }
                break;
            }
            
            // BMS标志信息反馈
            case 0x98C4E2EF:
            {
                msgs_core::bms_flag_Infor msg;
                msg.bms_flag_Infor_soc = recv_frame_.data[0];
                msg.bms_flag_Infor_single_ov = (0x01 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_single_uv = (0x02 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_ov = (0x04 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_uv = (0x08 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_charge_ot = (0x10 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_charge_ut = (0x20 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_discharge_ot = (0x40 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_discharge_ut = (0x80 & recv_frame_.data[1]) ? true : false;
                msg.bms_flag_Infor_charge_oc = (0x01 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_discharge_oc = (0x02 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_short = (0x04 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_ic_error = (0x08 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_lock_mos = (0x10 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_charge_flag = (0x20 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_SOCWarning = (0x40 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_SOCLowProtection = (0x80 & recv_frame_.data[2]) ? true : false;
                msg.bms_flag_Infor_hight_temperature = static_cast<float>(static_cast<short>(
                    recv_frame_.data[4] << 4 | recv_frame_.data[3] >> 4
                )) / 10.0f;
                msg.bms_flag_Infor_low_temperature = static_cast<float>(static_cast<short>(
                    (recv_frame_.data[6] & 0x0f) << 8 | recv_frame_.data[5]
                )) / 10.0f;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(bms_flag_infor_pub_, msg);
                }
                break;
            }
            
            // 驱动MCU编码器反馈
            case 0x98C4DCEF:
            {
                msgs_core::Drive_MCUEcoder_fb msg;
                msg.Drive_fb_MCUEcoder = static_cast<int>(
                    recv_frame_.data[3] << 24 | recv_frame_.data[2] << 16 | 
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                );
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(drive_mcu_encoder_pub_, msg);
                }
                break;
            }
            
            // 车辆诊断反馈
            case 0x98C4EAEF:
            {
                msgs_core::Veh_Diag_fb msg;
                msg.Veh_fb_FaultLevel = 0x0f & recv_frame_.data[0];
                msg.Veh_fb_AutoCANCtrlCmd = (0x10 & recv_frame_.data[0]) ? true : false;
                msg.Veh_fb_AutoIOCANCmd = (0x20 & recv_frame_.data[0]) ? true : false;
                msg.Veh_fb_EPSDisOnline = (0x01 & recv_frame_.data[1]) ? true : false;
                msg.Veh_fb_EPSfault = (0x02 & recv_frame_.data[1]) ? true : false;
                msg.Veh_fb_EPSMosfetOT = (0x04 & recv_frame_.data[1]) ? true : false;
                msg.Veh_fb_EPSWarning = (0x08 & recv_frame_.data[1]) ? true : false;
                msg.Veh_fb_EPSDisWork = (0x10 & recv_frame_.data[1]) ? true : false;
                msg.Veh_fb_EPSOverCurrent = (0x20 & recv_frame_.data[1]) ? true : false;
                msg.Veh_fb_EHBecuFault = (0x10 & recv_frame_.data[2]) ? true : false;
                msg.Veh_fb_EHBDisOnline = (0x20 & recv_frame_.data[2]) ? true : false;
                msg.Veh_fb_EHBWorkModelFault = (0x40 & recv_frame_.data[2]) ? true : false;
                msg.Veh_fb_EHBDisEn = (0x80 & recv_frame_.data[2]) ? true : false;
                msg.Veh_fb_EHBAnguleFault = (0x01 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_EHBOT = (0x02 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_EHBPowerFault = (0x04 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_EHBsensorAbnomal = (0x08 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_EHBMotorFault = (0x10 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_EHBOilPressSensorFault = (0x20 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_EHBOilFault = (0x40 & recv_frame_.data[3]) ? true : false;
                msg.Veh_fb_LDrvMCUFault = 0x3f & recv_frame_.data[4];
                msg.Veh_fb_RDrvMCUFault = (recv_frame_.data[5] & 0x0f << 2) | (recv_frame_.data[4] >> 6);
                msg.Veh_fb_AUXBMSDisOnline = (0x10 & recv_frame_.data[5]) ? true : false;
                msg.Veh_fb_AuxScram = (0x20 & recv_frame_.data[5]) ? true : false;
                msg.Veh_fb_AuxRemoteClose = (0x40 & recv_frame_.data[5]) ? true : false;
                msg.Veh_fb_AuxRemoteDisOnline = (0x80 & recv_frame_.data[5]) ? true : false;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(veh_diag_fb_pub_, msg);
                }
                break;
            }
            
            // 扭矩反馈
            case 0x98C4EBFF:
            {
                msgs_core::torque_fb msg;
                msg.lr_wheel_torque = static_cast<float>(static_cast<short>(
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                )) / 1000.0f;
                msg.rr_wheel_torque = static_cast<float>(static_cast<short>(
                    recv_frame_.data[3] << 8 | recv_frame_.data[2]
                )) / 1000.0f;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(torque_fb_pub_, msg);
                }
                break;
            }
            
            // 电流反馈
            case 0x98C4ECEF:
            {
                msgs_core::current_fb msg;
                msg.lr_wheel_current = static_cast<float>(static_cast<short>(
                    recv_frame_.data[1] << 8 | recv_frame_.data[0]
                )) / 1000.0f;
                msg.rr_wheel_current = static_cast<float>(static_cast<short>(
                    recv_frame_.data[3] << 8 | recv_frame_.data[2]
                )) / 1000.0f;
                
                // 计算校验和
                unsigned char crc = recv_frame_.data[0] ^ recv_frame_.data[1] ^ recv_frame_.data[2] ^ 
                                    recv_frame_.data[3] ^ recv_frame_.data[4] ^ recv_frame_.data[5] ^ recv_frame_.data[6];
                
                if (crc == recv_frame_.data[7]) {
                    in2ulv_cores::utils_core::safePublish(current_fb_pub_, msg);
                }
                break;
            }
            
            default:
                // 可选：记录未知CAN ID
                ROS_DEBUG_THROTTLE(5.0, "Received unknown CAN ID: 0x%08X", recv_frame_.can_id);
                break;
        }
    }
}

void ChassisController::run() {
    if (!initializeCAN()) {
        ROS_FATAL("Failed to initialize CAN interface");
        ros::shutdown();
        return;
    }
    
    // 启动数据接收线程
    boost::thread recv_thread(boost::bind(&ChassisController::receiveData, this));
    
    ros::spin();
    
    // 清理
    close(can_socket_);
    recv_thread.join();
}

}  // namespace chassis_driver
}  // namespace in2ulv_drivers