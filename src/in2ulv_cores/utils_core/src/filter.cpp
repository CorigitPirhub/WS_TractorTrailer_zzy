#include "utils_core/filter.hpp"

namespace in2ulv_cores {
namespace utils_core {

// SlidingAverageFilter 实现
SlidingAverageFilter::SlidingAverageFilter(int N)
    : N_(N), i_(0), sum_(0), value_buff_(N, 0) {}

float SlidingAverageFilter::apply(float new_data) {
    // 只有当新数据与上次数据差异较大时才更新滑动平均
    if (fabs(new_data - prev_data_) > 0.01) {
        sum_ -= value_buff_[i_];
        value_buff_[i_] = new_data;
        sum_ += new_data;
        i_ = (i_ + 1) % N_;
    }
    prev_data_ = sum_/N_;
    return prev_data_;
}

// ExponentialFilter 实现
ExponentialFilter::ExponentialFilter(float alpha)
    : alpha_(alpha), prev_data_(0) {}

float ExponentialFilter::apply(float new_data) {
    prev_data_ = alpha_ * new_data + (1 - alpha_) * prev_data_;
    return prev_data_;
}

}  // namespace utils_core
}  // namespace in2ulv_cores
