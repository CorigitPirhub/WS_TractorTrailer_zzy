#ifndef ANGLE_UTILS_HPP
#define ANGLE_UTILS_HPP

#include <cmath>

namespace in2ulv_cores {
namespace utils_core {

/**
 * @brief 角度工具函数集合
 * 
 * 提供角度相关的常用工具函数，包括：
 * - 角度归一化
 * - 角度加法
 * - 角度减法
 */

/**
 * @brief 将角度归一化到 [-180, 180] 度范围
 * 
 * @param angle 输入角度（度）
 * @return 归一化后的角度（度）
 */
inline double normalize_angle(double angle) {
    // 使用模运算实现高效的角度归一化
    angle = std::fmod(angle, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

/**
 * @brief 将两个角度相加并归一化结果
 * 
 * @param angle1 第一个角度（度）
 * @param angle2 第二个角度（度）
 * @return 归一化后的角度和（度）
 */
inline double add_angles(double angle1, double angle2) {
    return normalize_angle(angle1 + angle2);
}

/**
 * @brief 从第一个角度减去第二个角度并归一化结果
 * 
 * @param angle1 被减角度（度）
 * @param angle2 减角度（度）
 * @return 归一化后的角度差（度）
 */
inline double subtract_angles(double angle1, double angle2) {
    return normalize_angle(angle1 - angle2);
}

}  // namespace utils_core
}  // namespace in2ulv_cores

#endif  // ANGLE_UTILS_HPP