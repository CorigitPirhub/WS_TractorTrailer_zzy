#include "utils_core/automotive_pid.hpp"

namespace in2ulv_cores {
namespace utils_core {
double AutomotivePID::compute(double setpoint, double measurement, double dt, 
                              double feedforward_value) {
    if (dt <= -1e-6) return prev_output_;

    // 计算误差
    double error = setpoint - measurement;

    // 应用死区
    if (std::abs(error) <= deadband_) {
        error = 0.0;
        integral_ = integral_ * 0.5;
        if(std::abs(setpoint) <= 0.01) reset();
    }

    // 根据误差选择增益（如果启用了增益调度）
    if (use_gain_scheduling_) {
        updateGainsFromSchedule(std::abs(error));
    }

    // 计算比例项
    proportional_ = kp_ * error;
    
    // 计算微分项
    double derivative = 0;
    if (dt > 0.0) {
        derivative = (error - prev_error_) / dt;

        // 应用微分滤波
        if (derivative_filter_coeff_ > 0.0) {
            derivative = derivative_filter_coeff_ * prev_derivative_ + (1.0 - derivative_filter_coeff_) * derivative;
        }
    }
    derivative_term_ = kd_ * derivative;

    // 计算前馈项
    feedforward_term_ = feedforward_gain_ * feedforward_value;
    
    // 使用后向计算抗饱和法
    // 计算未饱和的控制输出
    double unsaturated_output;
    if (first_run_ == true) unsaturated_output = proportional_ + feedforward_term_;
    else unsaturated_output = proportional_ + integral_ + derivative_term_ + feedforward_term_;
    
    // 计算饱和后的输出
    double saturated_output = std::max(output_min_, std::min(output_max_, unsaturated_output));
    
    // 计算抗饱和项
    double aw_term = saturated_output - unsaturated_output;
    
    // 更新饱和状态
    if (aw_term > 1e-6) {
        output_saturation_status_ = -1;  // 下限饱和
    } else if (aw_term < -1e-6) {
        output_saturation_status_ = 1;   // 上限饱和
    } else {
        output_saturation_status_ = 0;   // 未饱和
    }
    // 计算积分项（梯形积分法）
    double integral_term;
    if (first_run_ == true){
        integral_term = ki_ * dt * error;
        first_run_ = false;
    } else {
        integral_term = 0.5 * ki_ * dt * (error + prev_error_);
    }
    // 更新积分项（添加误差积分和后向计算修正项）
    integral_ += integral_term + kaw_ * aw_term;
    
    // 应用积分限幅
    integral_ = std::max(integral_min_, std::min(integral_max_, integral_));
    
    // 计算最终输出
    output_ = saturated_output;

    // 更新状态
    prev_error_ = error;
    prev_derivative_ = derivative;
    prev_output_ = output_;

    return output_;
}
}  // namespace utils_core
}  // namespace in2ulv_cores