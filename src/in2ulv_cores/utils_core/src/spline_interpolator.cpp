#include "utils_core/spline_interpolator.hpp"

namespace in2ulv_cores {
namespace utils_core {

SplineInterpolator::SplineInterpolator(const std::vector<double>& breaks, const std::vector<std::vector<double>>& coefs)
    : breaks_(breaks), coefs_(coefs) {
    // 检查输入数据的有效性
    if (breaks_.size() != coefs_.size() + 1) {
        throw std::invalid_argument("Number of breaks must be one more than the number of coefficient sets.");
    }
    for (const auto& coef : coefs_) {
        if (coef.size() != 4) {
            throw std::invalid_argument("Each coefficient set must have 4 elements (for cubic splines).");
        }
    }
}

double SplineInterpolator::evaluate(double x) const {
    // 找到x所在的区间
    auto it = std::upper_bound(breaks_.begin(), breaks_.end(), x);
    if (it == breaks_.begin() || it == breaks_.end()) {
        if (it == breaks_.begin()) {
            it = breaks_.begin() + 1;
        } else {
            it = breaks_.end() - 1;
        }
    }
    size_t segment_index = std::distance(breaks_.begin(), it) - 1;
    // 计算插值结果
    double dx = x - breaks_[segment_index];
    const std::vector<double>& coef = coefs_[segment_index];
    return coef[0] * dx * dx * dx + coef[1] * dx * dx + coef[2] * dx + coef[3];
}

}  // namespace utils_core
}  // namespace in2ulv_cores