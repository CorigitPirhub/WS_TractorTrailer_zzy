#include "force_sensor_driver/ForceSensorController.hpp"
#include "utils_core/modbus_crc.hpp"
#include <iomanip> // 用于十六进制输出

namespace in2ulv_drivers {
namespace force_sensor_driver {

ForceSensorController::ForceSensorController(ros::NodeHandle& nh, 
                                           const std::string& port, 
                                           int baudrate,
                                           float force_gain)
    : nh_(nh), 
      port_(port), 
      baudrate_(baudrate), 
      force_gain_(force_gain),
      id_count_(1),
      force_l_(0.0f),
      force_r_(0.0f) {
    
    ROS_INFO("[ForceSensor] Initializing controller");
    ROS_INFO("[ForceSensor] - Port: %s", port_.c_str());
    ROS_INFO("[ForceSensor] - Baudrate: %d", baudrate_);
    ROS_INFO("[ForceSensor] - Force gain: %.4f", force_gain_);
    
    // 初始化读取命令
    read_cmd_[0] = 0x01;
    read_cmd_[1] = 0x03;
    read_cmd_[2] = 0x9c;
    read_cmd_[3] = 0x40;
    read_cmd_[4] = 0x00;
    read_cmd_[5] = 0x02;
    read_cmd_[6] = 0x00;
    read_cmd_[7] = 0x00;
    
    ROS_DEBUG("[ForceSensor] Initial command: %02X %02X %02X %02X %02X %02X %02X %02X",
              read_cmd_[0], read_cmd_[1], read_cmd_[2], read_cmd_[3],
              read_cmd_[4], read_cmd_[5], read_cmd_[6], read_cmd_[7]);
}

bool ForceSensorController::initialize() {
    ROS_INFO("[ForceSensor] Initializing connection");
    
    // 创建发布器
    force_info_pub_ = nh_.advertise<msgs_core::force_info>("force_info_f", 20);
    ROS_INFO("[ForceSensor] Created publisher on topic: force_info_f");
    
    // 设置串口参数
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    serial_port_.setPort(port_);
    serial_port_.setBaudrate(baudrate_);
    serial_port_.setTimeout(to);
    
    try {
        ROS_INFO("[ForceSensor] Opening port: %s", port_.c_str());
        serial_port_.open();
        
        if (serial_port_.isOpen()) {
            ROS_INFO_STREAM("[ForceSensor] Port opened successfully: " << port_);
            return true;
        } else {
            ROS_WARN("[ForceSensor] Port opened but isOpen() returned false");
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("[ForceSensor] Unable to open port: " << e.what());
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("[ForceSensor] Unexpected error: " << e.what());
    }
    
    ROS_ERROR("[ForceSensor] Failed to open serial port");
    return false;
}

void ForceSensorController::shutdown() {
    if (serial_port_.isOpen()) {
        ROS_INFO("[ForceSensor] Closing serial port");
        serial_port_.close();
    }
}

void ForceSensorController::run() {
    ROS_INFO("[ForceSensor] Starting main loop");
    ros::Rate loop_rate(200);
    
    int loop_count = 0;
    int success_count = 0;
    
    while (ros::ok()) {
        loop_count++;
        
        // 每100次循环打印一次状态
        if (loop_count % 100 == 0) {
            ROS_INFO("[ForceSensor] Loop %d (Success rate: %.1f%%)", 
                     loop_count, 
                     (success_count * 100.0) / loop_count);
            success_count = 0;
        }
        
        sendReadCommand();
        
        if (processResponse()) {
            success_count++;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ROS_INFO("[ForceSensor] Exiting main loop");
}

void ForceSensorController::sendReadCommand() {
    // 更新传感器ID
    id_count_ = (id_count_ + 1) % 2;
    read_cmd_[0] = id_count_ + 1;
    
    // 计算CRC
    uint16_t crc_code = in2ulv_cores::utils_core::ModbusCRC::calculate(read_cmd_, 6);
    read_cmd_[6] = crc_code & 0xFF;
    read_cmd_[7] = crc_code >> 8 & 0xFF;
    
    // 调试输出命令
    ROS_DEBUG("[ForceSensor] Sending command: %02X %02X %02X %02X %02X %02X %02X %02X",
              read_cmd_[0], read_cmd_[1], read_cmd_[2], read_cmd_[3],
              read_cmd_[4], read_cmd_[5], read_cmd_[6], read_cmd_[7]);
    
    // 发送命令
    try {
        size_t bytes_written = serial_port_.write(read_cmd_, 8);
        if (bytes_written != 8) {
            ROS_WARN("[ForceSensor] Only wrote %zu bytes out of 8", bytes_written);
        }
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("[ForceSensor] Write error: " << e.what());
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("[ForceSensor] Unexpected write error: " << e.what());
    }
}

bool ForceSensorController::processResponse() {
    size_t num = serial_port_.available();
    if (num == 0) {
        ROS_DEBUG_THROTTLE(1.0, "[ForceSensor] No data available");
        return false;
    }
    
    uint8_t buffer[256];
    try {
        num = serial_port_.read(buffer, num);
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("[ForceSensor] Read error: " << e.what());
        return false;
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("[ForceSensor] Unexpected read error: " << e.what());
        return false;
    }
    
    size_t len = num;
    ROS_DEBUG("[ForceSensor] Received %zu bytes", len);
    
    // 调试输出接收到的原始数据
    if (len > 0) {
        std::stringstream ss;
        ss << "[ForceSensor] Raw data: ";
        for (size_t i = 0; i < len; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(buffer[i]) << " ";
        }
        ROS_DEBUG_STREAM(ss.str());
    }
    
    if (len != 9) {
        ROS_WARN_THROTTLE(1.0, "[ForceSensor] Unexpected data length: %zu (expected 9)", len);
        return false;
    }
    
    // 计算CRC校验
    uint16_t crc_check = in2ulv_cores::utils_core::ModbusCRC::calculate(buffer, 7);
    uint8_t crc1 = crc_check & 0xFF;
    uint8_t crc2 = crc_check >> 8 & 0xFF;
    
    // 验证响应格式和CRC
    if (buffer[1] != 0x03) {
        ROS_WARN_THROTTLE(1.0, "[ForceSensor] Invalid function code: %02X (expected 0x03)", buffer[1]);
        return false;
    }
    
    if (buffer[2] != 0x04) {
        ROS_WARN_THROTTLE(1.0, "[ForceSensor] Invalid byte count: %02X (expected 0x04)", buffer[2]);
        return false;
    }
    
    if (buffer[7] != crc1 || buffer[8] != crc2) {
        ROS_WARN_THROTTLE(1.0, "[ForceSensor] CRC mismatch: Calculated %02X%02X, Received %02X%02X",
                          crc1, crc2, buffer[7], buffer[8]);
        return false;
    }
    
    // 处理有效响应
    if (buffer[0] == 0x01) {
        int32_t force_tmp = static_cast<int32_t>(
            buffer[3] << 24 | 
            buffer[4] << 16 | 
            buffer[5] << 8 | 
            buffer[6]
        );
        force_l_ = static_cast<float>(force_tmp) * force_gain_;
        ROS_DEBUG("[ForceSensor] Left force: %.2f (raw: %d)", force_l_, force_tmp);
    } 
    else if (buffer[0] == 0x02) {
        int32_t force_tmp = static_cast<int32_t>(
            buffer[3] << 24 | 
            buffer[4] << 16 | 
            buffer[5] << 8 | 
            buffer[6]
        );
        force_r_ = static_cast<float>(force_tmp) * force_gain_;
        ROS_DEBUG("[ForceSensor] Right force: %.2f (raw: %d)", force_r_, force_tmp);
    }
    else {
        ROS_WARN_THROTTLE(1.0, "[ForceSensor] Unknown sensor ID: %02X", buffer[0]);
        return false;
    }
    
    // 填充消息并发布
    force_info_msg_.fx = -(force_l_ + force_r_);
    force_info_msg_.fy = force_l_ - force_r_;
    force_info_msg_.fl = force_l_;
    force_info_msg_.fr = force_r_;
    
    force_info_pub_.publish(force_info_msg_);
    
    ROS_DEBUG("[ForceSensor] Published: fx=%.2f, fy=%.2f, fl=%.2f, fr=%.2f",
              force_info_msg_.fx, force_info_msg_.fy, 
              force_info_msg_.fl, force_info_msg_.fr);
    
    return true;
}

float ForceSensorController::bytesToFloat(uint8_t* buff) {
    uint8_t tmpbuff[4];
    memcpy(tmpbuff, buff, 4);
    return *reinterpret_cast<float*>(tmpbuff);
}

void ForceSensorController::floatToBytes(float f, uint8_t* buff) {
    uint8_t tmpbuff[4];
    *reinterpret_cast<float*>(tmpbuff) = f;
    memcpy(buff, tmpbuff, 4);
}

} // namespace force_sensor_driver
} // namespace in2ulv_drivers