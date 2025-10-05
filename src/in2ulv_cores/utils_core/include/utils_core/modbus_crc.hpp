#ifndef MODBUS_CRC_HPP
#define MODBUS_CRC_HPP

#include <cstdint>

namespace in2ulv_cores {
namespace utils_core {
/**
 * @brief Modbus CRC-16 计算器
 * 
 * 提供符合Modbus协议的CRC-16校验计算
 */
class ModbusCRC {
public:
    /**
     * @brief 计算Modbus CRC16校验码
     * @param data 数据指针
     * @param length 数据长度
     * @return CRC16校验码
     */
    static uint16_t calculate(uint8_t* data, uint16_t length);
    
private:
    ModbusCRC() = delete; // 禁止实例化
};

}  // namespace utils_core
}  // namespace in2ulv_cores

#endif  // MODBUS_CRC_HPP