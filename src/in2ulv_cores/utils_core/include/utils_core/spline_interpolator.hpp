#ifndef SPLINE_INTERPOLATOR_HPP
#define SPLINE_INTERPOLATOR_HPP

#include <vector>
#include <stdexcept>
#include <algorithm>

namespace in2ulv_cores {
namespace utils_core {
class SplineInterpolator {
public:
    /**
     * @brief 构造函数
     * 
     * @param breaks 分段点
     * @param coefs 多项式系数矩阵 (每行对应一个分段，从高次到低次排列)
     */
    SplineInterpolator(const std::vector<double>& breaks, const std::vector<std::vector<double>>& coefs);

    /**
     * @brief 计算给定x值对应的插值结果
     * 
     * @param x 输入值
     * @return 插值结果
     */
    double evaluate(double x) const;

    /**
     * @brief 获取分段数
    */
    size_t getNumSegments() const { return breaks_.size() - 1; }

private:
    std::vector<double> breaks_;  // 分段点
    std::vector<std::vector<double>> coefs_;  // 多项式系数
};
}  // namespace utils_core
}  // namespace in2ulv_cores

#endif // SPLINE_INTERPOLATOR_HPP