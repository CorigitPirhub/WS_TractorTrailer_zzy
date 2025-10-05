#ifndef FILTER_HPP
#define FILTER_HPP

#include <vector>
#include <cmath>

namespace in2ulv_cores {
namespace utils_core {

class Filter {
public:
    virtual float apply(float new_data) = 0; // 纯虚函数
    virtual ~Filter() = default; // 虚析构函数
};

class SlidingAverageFilter : public Filter {
public:
    SlidingAverageFilter(int N);
    virtual float apply(float new_data) override;

private:
    int N_;                      // Number of samples to average
    int i_;                      // Index for circular buffer
    float sum_;                  // Sum of the buffer values
    float prev_data_;            // Previous data for comparison
    std::vector<float> value_buff_; // Buffer to store values
};

class ExponentialFilter : public Filter {
public:
    ExponentialFilter(float alpha);
    virtual float apply(float new_data) override;

private:
    float alpha_;
    float prev_data_;
};

} // namespace utils_core
} // namespace in2ulv_cores

#endif // FILTER_HPP
