#pragma once
#include "config/config.h"
#include "control/p_balance_control.h"
#include "control/param.h"
#include "control/pid.h"
#include "control/u_balance_control.h"
#include "state/sub_state.h"

namespace CubliMini {
namespace Control {

enum class JumpState_e
{
    U_JUMP_IDLE,
    U_JUMP_MODE,
    U_JUMPING_MODE,
    U_BALANCE_MODE,
};

class JumpControl
{
   public:
    JumpControl(PBalanceControl &p_balance_control, UBalanceControl &u_balance_control)
        : p_balance_control_(p_balance_control), u_balance_control_(u_balance_control)
    {}

    void Loop(
        AxisSensor_t &u_sensor, const MotorSpeed_t &p_motor_speed, bool is_static)
    {
        MotorTransMode motor_trans_mode                  = {TOUCH_e, TOUCH_e, TOUCH_e};
        PBalanceControl::MotorControl send_motor_control = {0, 0, 0, 0};

        if (state_ == JumpState_e::U_JUMP_MODE)
        {
            motor_trans_mode.motor3_mode = VELOCITY_e;
            send_motor_control.ch3       = Param::GetInstance()->GetJumpParam().set_ch3_speed;
            if (jump_up_acceleration_time.GetTimeS() >= 0.4f ||
                fabs(p_motor_speed.ch3) >=
                    fabs(Param::GetInstance()->GetJumpParam().set_ch3_speed) - 20)
            {
                motor_trans_mode.motor3_mode = VELOCITY_e;
                jump_up_acceleration_time.reset();
                servo_protection_time.reset();
                send_motor_control.ch3 = -Param::GetInstance()->GetJumpParam().set_ch3_speed;
                state_                 = JumpState_e::U_JUMPING_MODE;
            }
        }
        if (state_ == JumpState_e::U_JUMPING_MODE)
        {
            motor_trans_mode.motor3_mode = VELOCITY_e;
            send_motor_control.ch3       = -Param::GetInstance()->GetJumpParam().set_ch3_speed;
            if (servo_protection_time.GetTimeS() >= 1.5f)
            {
                send_motor_control.ch3 = 0;
                state_                 = JumpState_e::U_JUMP_IDLE;
            }

            if (fabs(u_sensor.angle - Param::GetInstance()->GetUAxisParam().angle_offset) < 15.0f)
            {
                motor_trans_mode.motor3_mode = TOUCH_e;
                send_motor_control.ch3       = 0;
                state_                       = JumpState_e::U_BALANCE_MODE;
            }
        }
        if (state_ == JumpState_e::U_BALANCE_MODE)
        {
            send_motor_control.ch3 = u_balance_control_.Loop(u_sensor, true);
            send_motor_control.ch1 = 0;
            send_motor_control.ch2 = 0;
            if (fabs(u_sensor.angle - Param::GetInstance()->GetUAxisParam().angle_offset) > 15)
            {
                state_ = JumpState_e::U_JUMP_IDLE;
            }
        }
    }

   private:
    CubliMini::Comm::CumulativeTime servo_protection_time;
    CubliMini::Comm::CumulativeTime jump_up_acceleration_time;
    PBalanceControl &p_balance_control_;
    UBalanceControl &u_balance_control_;
    JumpState_e state_;
};

}  // namespace Control
}  // namespace CubliMini