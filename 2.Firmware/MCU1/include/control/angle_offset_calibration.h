#pragma once
#include "comm/comm.h"
#include "comm/time.h"

namespace CubliMini {
namespace Control {

class AngleOffsetAutoCalibration
{
   public:
    using CalibratorFinishCallback = std::function<void(float &)>;

    AngleOffsetAutoCalibration(
        float &initial_offset,
        float min_speed         = 20,
        float kp_speed          = 1.0f,
        float ki_speed          = 10.0f,
        float max_offset_change = 1.2f,
        float dt                = 0.002f)
        : kp_speed_(kp_speed),
          ki_speed_(ki_speed),
          max_offset_change_(max_offset_change),
          dt_(dt),
          angle_offset_(initial_offset),
          speed_integral_(0.0f),
          is_calibrating_(false),
          min_speed_(min_speed),
          max_integral_(max_offset_change_ * dt_ * 10)
    {}

    void StartCalibration(std::function<void()> end_callback = nullptr)
    {
        if (end_callback)
            end_callback_ = std::move(end_callback);
        is_calibrating_ = true;
        speed_integral_ = 0.0f;
    }

    bool GetCalibrationStatus() const
    {
        return is_calibrating_;
    }

    void StopCalibration() { is_calibrating_ = false; }

    void RegisterFinishCallback(CalibratorFinishCallback callback)
    {
        finish_callback_ = std::move(callback);
    }

    void Update(float motor_speed)
    {
        if (!is_calibrating_)
            return;

        if (fabs(motor_speed) <= min_speed_)
        {
            is_calibrating_ = false;
            if (finish_callback_)
                finish_callback_(angle_offset_);
            if (end_callback_)
                end_callback_();
            return;
        }

        float speed_error = 0.0f - motor_speed;

        speed_integral_ += speed_error * dt_;

        if (speed_integral_ > max_integral_)
        {
            speed_integral_ = max_integral_;
        }
        else if (speed_integral_ < -max_integral_)
        {
            speed_integral_ = -max_integral_;
        }

        float offset_adjust = kp_speed_ * speed_error + ki_speed_ * speed_integral_;
        offset_adjust = clamp(offset_adjust, -max_offset_change_ * dt_, max_offset_change_ * dt_);

        angle_offset_ += offset_adjust;
    }

    float GetCalibratedOffset() const { return angle_offset_; }

    float kp_speed_;           // 速度误差比例增益
    float ki_speed_;           // 速度误差积分增益
    float dt_;                 // 控制周期（秒）
    float max_offset_change_;  // 每秒最大偏移调整量

   private:
    float clamp(float value, float min_value, float max_value)
    {
        return max(min_value, min(value, max_value));
    }
    CalibratorFinishCallback finish_callback_ = nullptr;
    std::function<void()> end_callback_       = nullptr;
    float max_integral_;    // 积分项上限（根据系统动态调整）
    float &angle_offset_;   // 当前校准后的角度偏移（含初始值）
    float speed_integral_;  // 速度误差积分项
    bool is_calibrating_;   // 校准模式标志
    float min_speed_;
};

}  // namespace Control
}  // namespace CubliMini