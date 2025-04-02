#pragma once

#include "comm/comm.h"
#include "control/angle_offset_calibration.h"
#include "control/control_base.h"
#include "control/param.h"

namespace CubliMini {
namespace Control {

#define P_BALABCE_PROTECTION_THRESHOLD 20
#define P_BALABCE_OUTPUT_LIMIT         4  // 15

class PBalanceControl
{
   public:
    using AxisControl  = AxisSpeed_t;
    using MotorControl = MotorSpeed_t;

    PBalanceControl(PAxisParam_t &lqr_param)
        : lqr_param_(lqr_param),
          cube_is_upside_dowm_(false),
          output_limit_(P_BALABCE_OUTPUT_LIMIT),
          angle_protection_threahold_(P_BALABCE_PROTECTION_THRESHOLD),
          p_x_auto_calibration_(lqr_param.x.angle_offset, 50),
          p_y_auto_calibration_(lqr_param.y.angle_offset, 50),
          is_balance_(false)
    {
        p_y_auto_calibration_.RegisterFinishCallback([](float &angle_offset) {
            Param::GetInstance()->SavePBalanceParam();
        });
        p_x_auto_calibration_.RegisterFinishCallback([](float &angle_offset) {
            Param::GetInstance()->SavePBalanceParam();
        });
    }

    MotorControl Loop(
        PSensor_t &sensor, const MotorSpeed_t &motor_speed, bool is_static, float yaw_control)
    {
        MotorControl motor_control = {0, 0, 0};
        if (is_static)
        {
            cube_is_upside_dowm_ = false;
        }

        if (fabs(sensor.x.angle - lqr_param_.x.angle_offset) < fabs(angle_protection_threahold_) &&
            fabs(sensor.y.angle - lqr_param_.y.angle_offset) < fabs(angle_protection_threahold_) &&
            cube_is_upside_dowm_ == false)
        {
            AxisSpeed_t axis_speed;
            GetAxisMoveSpeed(motor_speed, axis_speed);
            sensor.x.speed = axis_speed.x;
            sensor.y.speed = axis_speed.y;
            sensor.z.speed = axis_speed.z;

            AxisControl axis_control;
            axis_control.x = AxisLqr(sensor.x, lqr_param_.x);
            axis_control.y = AxisLqr(sensor.y, lqr_param_.y);
            axis_control.z = AxisLqr(sensor.z, lqr_param_.z) + yaw_control;
            GetMotorMoveSpeed(axis_control, motor_control);

            p_x_auto_calibration_.Update(axis_speed.x);
            p_y_auto_calibration_.Update(-axis_speed.y);
            is_balance_ = true;
        }
        else
        {
            motor_control.ch1    = 0;
            motor_control.ch2    = 0;
            motor_control.ch3    = 0;
            cube_is_upside_dowm_ = true;
            is_balance_ = false;
        }

        motor_control.ch1 = Comm::Limit(motor_control.ch1, output_limit_);
        motor_control.ch2 = Comm::Limit(motor_control.ch2, output_limit_);
        motor_control.ch3 = Comm::Limit(motor_control.ch3, output_limit_);
        return motor_control;
    }
    bool IsBalance() const
    {
        return is_balance_;
    }

   private:
    void GetAxisMoveSpeed(const MotorSpeed_t &_get_speed, AxisSpeed_t &_axis_speed)
    {
        _axis_speed.x = _get_speed.ch2 - _get_speed.ch3 * sin((30) / 57.3f) -
                        _get_speed.ch1 * sin((30) / 57.3f);
        _axis_speed.y = (_get_speed.ch3 - _get_speed.ch1) * cos((30) / 57.3f);
        _axis_speed.z = (_get_speed.ch2 + _get_speed.ch3 + _get_speed.ch1);
    }

    void GetMotorMoveSpeed(const AxisControl &axis_control, MotorControl &motor_control)
    {
        motor_control.ch1 = -axis_control.y * cos((30) / 57.3f) -
                            axis_control.x * sin((30) / 57.3f) + axis_control.z / 3.0f;
        motor_control.ch2 = axis_control.x + axis_control.z / 3.0f;
        motor_control.ch3 = -axis_control.x * sin((30) / 57.3f) +
                            axis_control.y * cos((30) / 57.3f) + axis_control.z / 3.0f;
    }

   public:
    bool is_balance_;
    PAxisParam_t &lqr_param_;
    float output_limit_;
    float angle_protection_threahold_;
    AngleOffsetAutoCalibration p_x_auto_calibration_;
    AngleOffsetAutoCalibration p_y_auto_calibration_;

   private:
    bool cube_is_upside_dowm_;
};

}  // namespace Control
}  // namespace CubliMini