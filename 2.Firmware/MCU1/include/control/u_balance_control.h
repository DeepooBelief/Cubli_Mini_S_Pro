#pragma once

#include "comm/comm.h"
#include "config/config.h"
#include "control/angle_offset_calibration.h"
#include "control/control_base.h"

namespace CubliMini {
namespace Control {
using namespace Config;

#define U_BALABCE_OUTPUT_LIMIT 10

class UBalanceControl
{
   public:
    UBalanceControl(AxisParam_t &lqr_param)
        : lqr_param_(lqr_param),
          cube_is_upside_dowm_(false),
          output_limit_(U_BALABCE_OUTPUT_LIMIT),
          angle_protection_threahold_(U_BALABCE_PROTECTION_THRESHOLD),
          u_auto_calibration_(lqr_param.angle_offset),
          is_balance_(false)
    {
        u_auto_calibration_.RegisterFinishCallback(
            [](float &angle_offset) { Param::GetInstance()->SaveUBalanceParam(); });
    }

    float Loop(const AxisSensor_t &sensor, bool is_static)
    {
        float value = 0.0f;
        if (is_static)
        {
            cube_is_upside_dowm_ = false;
        }

        if (fabs(sensor.angle - lqr_param_.angle_offset) < fabs(angle_protection_threahold_) &&
            cube_is_upside_dowm_ == false)
        {
            // value = AxisLqr(sensor, lqr_param_);da
            static float alpha = 0.0022f;
            static float angle_offset = lqr_param_.angle_offset;
            angle_offset = (1 - alpha) * angle_offset + alpha * sensor.angle;
            value = lqr_param_.kp * (sensor.angle - angle_offset + 0.23f) + sensor.gyro * lqr_param_.kv + sensor.speed * lqr_param_.ks;
            u_auto_calibration_.Update(sensor.speed);
            is_balance_ = true;
        }
        else
        {
            cube_is_upside_dowm_ = true;
            is_balance_ = false;
        }
        return Comm::Limit(value, output_limit_);
    }

    bool IsBalance() const
    {
        return is_balance_;
    }

    AngleOffsetAutoCalibration u_auto_calibration_;
   public:
    bool is_balance_;
    AxisParam_t &lqr_param_;
    float output_limit_;
    float angle_protection_threahold_;

   private:
    bool cube_is_upside_dowm_;
};

}  // namespace Control
}  // namespace CubliMini