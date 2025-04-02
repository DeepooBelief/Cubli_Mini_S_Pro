#pragma once
#include "BSP/bsp.h"
#include "control/control_base.h"
#include "control/p_balance_control.h"
#include "control/param.h"
#include "control/u_balance_control.h"

namespace CubliMini {
namespace Control {

using namespace Comm;
using namespace Bsp;

enum RunMode_e
{
    IDLE_MODE = 0x00,    // 普通模式
    U_JUMP_MODE,         // 边起跳模式
    U_JUMPING_MODE,      // 起跳中
    U_BALANCE_MODE,      // 边平衡模式
    POINT_JUMP_MODE,     // 点起跳模式
    POINT_JUMPING_MODE,  // 点起跳中
    POINT_BALANCE_MODE,  // 点平衡
    MOTOR_TEST_MODE,     // 电机测试模式
    MOTOR_TEST_MODE2     // 电机测试模式
};



class ControlComp
{
   public:
    ControlComp()
        : contorl_mode_(IDLE_MODE),
          U_balance_control_(Param::GetInstance()->GetUAxisParam()),
          p_balance_control_(Param::GetInstance()->GetPAxisParam_t())
    {
        
    }
    ~ControlComp() = default;

    PBalanceControl::MotorControl Loop(
        PSensor_t &p_sensor,
        const MotorSpeed_t &p_motor_speed,
        const AxisSensor_t &u_sensor,
        float is_static)
    {
        PBalanceControl::MotorControl send_motor_control = {0, 0, 0, 0};
        MotorTransMode motor_trans_mode                  = {TOUCH_e, TOUCH_e, TOUCH_e};

        if (contorl_mode_ == IDLE_MODE)
        {
            send_motor_control.ch1          = 0;
            send_motor_control.ch2          = 0;
            send_motor_control.ch3          = 0;
            send_motor_control.control_mode = 0;
            jump_up_acceleration_time_.reset();
            do_point_jump_flag_ = false;
        }

        if (contorl_mode_ == U_JUMP_MODE)
        {
            motor_trans_mode.motor3_mode = VELOCITY_e;
            send_motor_control.ch3       = Param::GetInstance()->GetJumpParam().set_ch3_speed;
            if (jump_up_acceleration_time_.GetTimeS() >= 0.4f ||
                fabs(p_motor_speed.ch2) >=
                    fabs(Param::GetInstance()->GetJumpParam().set_ch3_speed) - 10)
            {
                motor_trans_mode.motor3_mode = TOUCH_e;
                jump_up_acceleration_time_.reset();
                servo_protection_time_.reset();
                send_motor_control.ch3 = -Param::GetInstance()->GetJumpParam().set_ch3_speed;
                contorl_mode_          = U_JUMPING_MODE;
            }
        }
        if (contorl_mode_ == U_JUMPING_MODE)
        {
            motor_trans_mode.motor3_mode = TOUCH_e;
            send_motor_control.ch3       = -Param::GetInstance()->GetJumpParam().set_ch3_speed;

            if (servo_protection_time_.GetTimeS() >= 0.6f)
            {
                send_motor_control.ch3 = 0;
                contorl_mode_          = IDLE_MODE;
            }

            if (u_sensor.angle - Param::GetInstance()->GetUAxisParam().angle_offset <= 10)
            {
                motor_trans_mode.motor3_mode = TOUCH_e;
                send_motor_control.ch3       = 0;
                contorl_mode_                = U_BALANCE_MODE;
            }
        }
        if (contorl_mode_ == U_BALANCE_MODE || contorl_mode_ == POINT_JUMP_MODE)
        {
            if (is_fast_runing_)
                send_motor_control.ch3 = U_balance_control_.Loop(u_sensor, true);
            else
                send_motor_control.ch3 = U_balance_control_.Loop(u_sensor, is_static);

            send_motor_control.ch1 = 0;
            send_motor_control.ch2 = 0;
        }
        if (contorl_mode_ == POINT_JUMP_MODE)
        {
            if (do_point_jump_flag_ == false)
            {
                do_point_jump_flag_ = true;
                jump_up_acceleration_time_.reset();
            }
            send_motor_control.control_mode = 1;
            motor_trans_mode.motor2_mode    = VELOCITY_e;
            motor_trans_mode.motor1_mode    = VELOCITY_e;
            send_motor_control.ch2          = Param::GetInstance()->GetJumpParam().set_ch2_speed;
            send_motor_control.ch1          = Param::GetInstance()->GetJumpParam().set_ch1_speed;

            if (jump_up_acceleration_time_.GetTimeS() >= 0.6f ||
                (fabs(send_motor_control.ch1 - p_motor_speed.ch1) < 10 &&
                 fabs(send_motor_control.ch2 - p_motor_speed.ch3) < 10))
            {
                send_motor_control.control_mode = 0;
                motor_trans_mode.motor2_mode    = TOUCH_e;
                motor_trans_mode.motor1_mode    = TOUCH_e;
                send_motor_control.ch3          = 0;
                send_motor_control.ch2 = -Param::GetInstance()->GetJumpParam().set_ch2_speed;
                send_motor_control.ch1 = -Param::GetInstance()->GetJumpParam().set_ch1_speed;
                jump_up_acceleration_time_.reset();
                contorl_mode_    = POINT_JUMPING_MODE;
                point_jump_flag_ = true;
            }
        }
        if (contorl_mode_ == POINT_JUMPING_MODE)
        {
            motor_trans_mode.motor2_mode = VELOCITY_e;
            motor_trans_mode.motor1_mode = VELOCITY_e;
            send_motor_control.ch2       = -Param::GetInstance()->GetJumpParam().set_ch2_speed;
            send_motor_control.ch1       = -Param::GetInstance()->GetJumpParam().set_ch1_speed;

            do_point_jump_flag_ = false;
            if (jump_up_acceleration_time_.GetTimeS() > 0.7f)
            {
                motor_trans_mode.motor2_mode = TOUCH_e;
                motor_trans_mode.motor1_mode = TOUCH_e;
                send_motor_control.ch2       = 0;
                send_motor_control.ch1       = 0;
            }

            float y_delta_angle =
                p_sensor.y.angle - Param::GetInstance()->GetPAxisParam_t().y.angle_offset;

            if (fabs(p_sensor.y.gyro) > 200 && y_delta_angle > -3)
            {
                motor_trans_mode.motor2_mode = TOUCH_e;
                motor_trans_mode.motor1_mode = TOUCH_e;
                send_motor_control.ch2       = 0;
                send_motor_control.ch1       = 0;
                contorl_mode_                = POINT_BALANCE_MODE;
            }
            if (fabs(p_sensor.y.gyro) < 10 && fabs(y_delta_angle) < 6)
            {
                motor_trans_mode.motor2_mode = TOUCH_e;
                motor_trans_mode.motor1_mode = TOUCH_e;
                send_motor_control.ch2       = 0;
                send_motor_control.ch1       = 0;
                contorl_mode_                = POINT_BALANCE_MODE;
            }
            if (fabs(p_sensor.x.angle - Param::GetInstance()->GetPAxisParam_t().x.angle_offset) <
                    6 &&
                fabs(p_sensor.y.angle - Param::GetInstance()->GetPAxisParam_t().y.angle_offset) < 6)
            {
                motor_trans_mode.motor2_mode = TOUCH_e;
                motor_trans_mode.motor1_mode = TOUCH_e;
                send_motor_control.ch2       = 0;
                send_motor_control.ch1       = 0;
                contorl_mode_                = POINT_BALANCE_MODE;
            }
        }
        if (contorl_mode_ == POINT_BALANCE_MODE)
        {
            send_motor_control.control_mode = 2;
            PBalanceControl::MotorControl motor_control;
            if (is_fast_runing_)
            {
                motor_control =
                    p_balance_control_.Loop(p_sensor, p_motor_speed, true, yaw_control_);
            }
            else
            {
                motor_control =
                    p_balance_control_.Loop(p_sensor, p_motor_speed, is_static, yaw_control_);
            }
            send_motor_control.ch1 = motor_control.ch1;
            send_motor_control.ch2 = motor_control.ch3;
            send_motor_control.ch3 = motor_control.ch2;
        }
        if (contorl_mode_ == MOTOR_TEST_MODE)
        {
            send_motor_control.control_mode = 3;
            motor_trans_mode.motor1_mode    = VELOCITY_e;
            motor_trans_mode.motor2_mode    = VELOCITY_e;
            motor_trans_mode.motor3_mode    = VELOCITY_e;

            send_motor_control.ch1 = 20;
            send_motor_control.ch2 = 20;
            send_motor_control.ch3 = 20;
        }
        if (contorl_mode_ == MOTOR_TEST_MODE2)
        {
            send_motor_control.control_mode = 3;
            motor_trans_mode.motor1_mode    = VELOCITY_e;
            motor_trans_mode.motor2_mode    = VELOCITY_e;
            motor_trans_mode.motor3_mode    = VELOCITY_e;

            motor_test_time_.Task(2, [&send_motor_control, this]() { ++motor_test_count_; });
            if (motor_test_count_ % 6 == 0)
                send_motor_control.ch1 = 20;
            else if (motor_test_count_ % 6 == 1)
                send_motor_control.ch2 = 20;
            else if (motor_test_count_ % 6 == 2)
                send_motor_control.ch3 = 20;
            else if (motor_test_count_ % 6 == 3)
                send_motor_control.ch1 = -20;
            else if (motor_test_count_ % 6 == 4)
                send_motor_control.ch2 = -20;
            else if (motor_test_count_ % 6 == 5)
                send_motor_control.ch3 = -20;
        }
        send_motor_control.value = MotorTransMode::GetValue(motor_trans_mode);
        return send_motor_control;
    }

    void StartUAutoCalibration(std::function<void()> finsh_callback = nullptr)
    {
        U_balance_control_.u_auto_calibration_.StartCalibration(finsh_callback);
    }
    void StopUAutoCalibration() { U_balance_control_.u_auto_calibration_.StopCalibration(); }
    void StartPAutoCalibration(std::function<void()> finsh_callback = nullptr)
    {
        p_balance_control_.p_x_auto_calibration_.StartCalibration(finsh_callback);
        p_balance_control_.p_y_auto_calibration_.StartCalibration(finsh_callback);
    }
    void StopPAutoCalibration()
    {
        p_balance_control_.p_x_auto_calibration_.StopCalibration();
        p_balance_control_.p_y_auto_calibration_.StopCalibration();
    }

    bool is_fast_runing_             = true;
    uint8_t contorl_mode_;
    float yaw_control_ = 0;

    PBalanceControl p_balance_control_;
    UBalanceControl U_balance_control_;

   private:
    bool cubli_is_b_blance_  = false;
    bool do_point_jump_flag_ = false;
    bool point_jump_flag_    = false;
    CubliMini::Comm::CumulativeTime servo_protection_time_;
    CubliMini::Comm::CumulativeTime jump_up_acceleration_time_;
    CubliMini::Comm::CumulativeTime motor_test_time_;
    uint8_t motor_test_count_ = 0;
};

extern ControlComp g_control_comp;

}  // namespace Control
}  // namespace CubliMini