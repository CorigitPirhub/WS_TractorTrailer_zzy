/**
 * 汽车控制系统专用PID控制器
 * 特点：
 * - 支持比例增益调度（根据误差大小动态调整增益）
 * - 积分限幅和后向计算抗饱和
 * - 支持微分项滤波
 * - 支持前馈控制
 * - 专门为车辆速度、转向等控制任务设计
 */
#ifndef AUTOMOTIVE_PID_HPP
#define AUTOMOTIVE_PID_HPP

#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

namespace in2ulv_cores {
namespace utils_core {
class AutomotivePID {
public:
    /**
     * @brief 增益调度结构体
     * 用于根据误差大小动态调整PID增益
     */
    struct GainSchedule {
        double error_threshold;  // 误差阈值
        double kp;               // 比例增益
        double ki;               // 积分增益
        double kd;               // 微分增益
    };
    
    /**
     * @brief 默认构造函数
     */
    AutomotivePID() :
        kp_(1.0), ki_(0.0), kd_(0.0),
        prev_error_(0.0), integral_(0.0), prev_derivative_(0.0),
        output_(0.0), prev_output_(0.0), first_run_(true),
        output_min_(-1.0), output_max_(1.0),
        integral_min_(-1.0), integral_max_(1.0),
        derivative_filter_coeff_(0.0),
        feedforward_gain_(0.0),
        use_gain_scheduling_(false),
        kaw_(0.0),                       // 默认不使用后向计算抗饱和
        output_saturation_status_(0),
        controller_name_("Default") {}
    
    /**
     * @brief 带参数的构造函数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param output_min 输出下限
     * @param output_max 输出上限
     * @param integral_min 积分下限
     * @param integral_max 积分上限
     * @param controller_name 控制器名称（用于调试）
     */
    AutomotivePID(double kp, double ki, double kd,
                 double output_min, double output_max,
                 double integral_min, double integral_max,
                 const std::string& controller_name = "PID") :
        kp_(kp), ki_(ki), kd_(kd),
        prev_error_(0.0), integral_(0.0), prev_derivative_(0.0),
        output_(0.0), prev_output_(0.0), first_run_(true),
        output_min_(output_min), output_max_(output_max),
        integral_min_(integral_min), integral_max_(integral_max),
        derivative_filter_coeff_(0.0),
        feedforward_gain_(0.0),
        use_gain_scheduling_(false),
        kaw_(0.0),                       // 默认不使用后向计算抗饱和
        output_saturation_status_(0),
        deadband_(0.045),
        controller_name_(controller_name) {}
    
    /**
     * @brief 初始化增益调度表
     * @param schedules 增益调度表
     */
    void initGainScheduling(const std::vector<GainSchedule>& schedules) {
        if (!schedules.empty()) {
            gain_schedules_ = schedules;
            use_gain_scheduling_ = true;
        }
    }

    /**
     * @brief 设置比例增益
     * @param kp 比例增益
     */
    void setProportionalGain(double kp) {
        kp_ = kp;
    }

    /**
     * @brief 设置积分增益
     * @param ki 积分增益
     */
    void setIntegralGain(double ki) {
        ki_ = ki;
    }

    /**
     * @brief 设置微分增益
     * @param kd 微分增益
     */
    void setDerivativeGain(double kd) {
        kd_ = kd;
    }
    
    /**
     * @brief 设置微分滤波系数
     * @param coefficient 滤波系数(0-1)
     */
    void setDerivativeFilter(double coefficient) {
        derivative_filter_coeff_ = std::max(0.0, std::min(1.0, coefficient));
    }
    
    /**
     * @brief 设置前馈增益
     * @param gain 前馈增益
     */
    void setFeedforwardGain(double gain) {
        feedforward_gain_ = gain;
    }
    
    /**
     * @brief 设置积分限制
     * @param min 最小值
     * @param max 最大值
     */
    void setIntegralLimits(double min, double max) {
        integral_min_ = min;
        integral_max_ = max;
    }

    /**
     * @brief 设置后向计算抗饱和参数
     * @param kaw 后向计算抗饱和参数
     */
    void setAntiWindupGain(double kaw) {
        kaw_ = kaw;
    }
    
    /**
     * @brief 设置死区
     * @param deadband 死区大小
     */
    void setDeadband(double deadband) {
        deadband_ = deadband;
    }

    /**
     * @brief 计算控制输出
     * @param setpoint 设定值
     * @param measurement 当前测量值
     * @param dt 时间间隔(秒)
     * @param feedforward_value 前馈值(可选)
     * @return 控制输出
     */
    double compute(double setpoint, double measurement, double dt, double feedforward_value = 0.0);
    
    /**
     * @brief 重置控制器状态
     */
    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
        prev_derivative_ = 0.0;
        output_ = 0.0;
        prev_output_ = 0.0;
        output_saturation_status_ = 0;
    }
    
    /**
     * @brief 获取控制器名称
     */
    const std::string& getName() const { return controller_name_; }
    
    /**
     * @brief 获取当前比例增益
     */
    double getKp() const { return kp_; }
    
    /**
     * @brief 获取当前积分增益
     */
    double getKi() const { return ki_; }
    
    /**
     * @brief 获取当前微分增益
     */
    double getKd() const { return kd_; }

    /**
     * @brief 获取当前比例项
     */
    double getProportional() const { return proportional_; }

    /**
     * @brief 获取当前微分项
     */
    double getDerivative() const { return derivative_term_; }

    /**
     * @brief 获取当前积分项
     */
    double getIntegral() const { return integral_; }

    /**
     * @brief 获取当前前馈项
     */
    double getFeedforward() const { return feedforward_term_; }
    
    /**
     * @brief 获取输出饱和状态
     * @return 饱和状态 (-1:下限饱和, 0:未饱和, 1:上限饱和)
     */
    int getOutputSaturationStatus() const { return output_saturation_status_; }
    
    /**
     * @brief 获取当前输出
     */
    double getOutput() const { return output_; }

private:
    // 基本PID参数
    double kp_;                    // 比例增益
    double ki_;                    // 积分增益
    double kd_;                    // 微分增益
    
    // 状态变量
    double prev_error_;            // 上一次误差
    double proportional_;           // 比例项
    double derivative_term_;            // 微分项
    double integral_;              // 积分项
    double feedforward_term_;      // 前馈项
    double prev_derivative_;       // 上一次微分值
    double output_;                // 当前输出
    double prev_output_;           // 上一次输出
    bool first_run_;               // 是否第一次运行,关乎微分项和积分项的具体形式
    
    // 限制参数
    double output_min_;            // 输出下限
    double output_max_;            // 输出上限
    double integral_min_;          // 积分下限
    double integral_max_;          // 积分上限
    // 后向计算抗饱和参数
    double kaw_;                    // 抗饱和增益系数
    int output_saturation_status_;  // 输出饱和状态指示器(-1:下限饱和, 0:未饱和, 1:上限饱和)
    // 死区
    double deadband_;              // 误差死区
    
    // 特殊特性参数
    double derivative_filter_coeff_;  // 微分滤波系数
    double feedforward_gain_;      // 前馈增益
    
    // 增益调度
    bool use_gain_scheduling_;     // 是否使用增益调度
    std::vector<GainSchedule> gain_schedules_; // 增益调度表
    
    // 调试信息
    std::string controller_name_;  // 控制器名称
    
    /**
     * @brief 根据误差大小更新增益
     * @param abs_error 误差绝对值
     */
    void updateGainsFromSchedule(double abs_error) {
        if (gain_schedules_.empty()) return;
        
        // 默认使用第一组增益
        kp_ = gain_schedules_[0].kp;
        ki_ = gain_schedules_[0].ki;
        kd_ = gain_schedules_[0].kd;
        
        // 查找适用的增益设置
        for (const auto& schedule : gain_schedules_) {
            if (abs_error <= schedule.error_threshold) {
                kp_ = schedule.kp;
                ki_ = schedule.ki;
                kd_ = schedule.kd;
                break;
            }
        }
    }
};

}  // namespace utils_core
}  // namespace in2ulv_cores

#endif  // AUTOMOTIVE_PID_H