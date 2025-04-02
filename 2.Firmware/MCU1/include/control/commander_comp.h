#pragma once

#include <string>

#include "control/control.h"
#include "control/param.h"
#include "control/serial_commander.h"
#include "imu/imu_comp.h"
#include "state/sub_state.h"

namespace CubliMini {
namespace Control {

using namespace Imu;
using namespace State;

class CommanderComp
{
   public:
    void RegisterInit()
    {
        serial_commander_.Register("M", [&](const std::string &value) {
            uint16_t modeVariable        = std::stof(value);
            g_control_comp.contorl_mode_ = modeVariable;
            printf("M: %d\n", modeVariable);
            if (g_control_comp.contorl_mode_ == U_BALANCE_MODE)
                g_machine->pushEvent(static_cast<int>(StateType::U_BALANCE));
            if (g_control_comp.contorl_mode_ == POINT_BALANCE_MODE)
                g_machine->pushEvent(static_cast<int>(StateType::P_BALANCE));
            if (g_control_comp.contorl_mode_ == U_JUMP_MODE)
            {
                g_machine->is_remote_control = true;
                g_machine->pushEvent(static_cast<int>(StateType::U_BALANCE));
            }
            if (g_control_comp.contorl_mode_ == POINT_JUMP_MODE)
                g_machine->pushEvent(static_cast<int>(StateType::P_BALANCE));
            if (g_control_comp.contorl_mode_ == IDLE_MODE)
                g_machine->pushEvent(static_cast<int>(StateType::IDLE));
            if (g_control_comp.contorl_mode_ == MOTOR_TEST_MODE2)
                g_machine->pushEvent(static_cast<int>(StateType::MOTOR_TEST));
            if (g_control_comp.contorl_mode_ == MOTOR_TEST_MODE)
                g_machine->pushEvent(static_cast<int>(StateType::MOTOR_TEST));
        });

        AxisParam_t &u_param = Param::GetInstance()->GetUAxisParam();
        REGISTER_COMMAND_FLOAT(serial_commander_, "SUA", u_param.angle_offset);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SUP", u_param.kp);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SUV", u_param.kv);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SUS", u_param.ks);

        PAxisParam_t &p_param = Param::GetInstance()->GetPAxisParam_t();
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPXA", p_param.x.angle_offset);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPXP", p_param.x.kp);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPXV", p_param.x.kv);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPXS", p_param.x.ks);

        REGISTER_COMMAND_FLOAT(serial_commander_, "SPYA", p_param.y.angle_offset);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPYP", p_param.y.kp);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPYV", p_param.y.kv);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPYS", p_param.y.ks);

        REGISTER_COMMAND_FLOAT(serial_commander_, "SPZA", p_param.z.angle_offset);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPZP", p_param.z.kp);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPZV", p_param.z.kv);
        REGISTER_COMMAND_FLOAT(serial_commander_, "SPZS", p_param.z.ks);

        REGISTER_COMMAND_FLOAT(
            serial_commander_, "M1_JUMP_SPEED", Param::GetInstance()->GetJumpParam().set_ch1_speed);
        REGISTER_COMMAND_FLOAT(
            serial_commander_, "M2_JUMP_SPEED", Param::GetInstance()->GetJumpParam().set_ch2_speed);
        REGISTER_COMMAND_FLOAT(
            serial_commander_, "M3_JUMP_SPEED", Param::GetInstance()->GetJumpParam().set_ch3_speed);

        REGISTER_COMMAND_FLOAT(serial_commander_, "YAW_CONTROL", g_control_comp.yaw_control_);

        REGISTER_COMMAND_FLOAT(serial_commander_, "LOW_POWER", Param::GetInstance()->GetVbusParam().vbus_protected_v);
        REGISTER_COMMAND_FLOAT(serial_commander_, "NTC_OPT", Param::GetInstance()->GetVbusParam().ntc_protected_temp);

        serial_commander_.Register("IMU_ROTATION_Y", [&](const std::string &value) {
            float modeVariable = std::stof(value);
            g_imu_comp_.SetGyroYConvAngle(modeVariable);
            printf("SetGyroYConvAngle: %f\n", modeVariable);
        });

        serial_commander_.Register("GU", [](const std::string &value) {
            AxisParam_t &param = Param::GetInstance()->GetUAxisParam();
            printf(
                "GU angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f]\n",
                param.angle_offset,
                param.kp,
                param.kv,
                param.ks);
        });

        serial_commander_.Register("GP", [](const std::string &value) {
            PAxisParam_t &param = Param::GetInstance()->GetPAxisParam_t();
            printf(
                "GP\n x angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n y "
                "angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n z angle_offset:[%0.3f] "
                "kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n",
                param.x.angle_offset,
                param.x.kp,
                param.x.kv,
                param.x.ks,
                param.y.angle_offset,
                param.y.kp,
                param.y.kv,
                param.y.ks,
                param.z.angle_offset,
                param.z.kp,
                param.z.kv,
                param.z.ks);
        });

        serial_commander_.Register("SAVE_P", [](const std::string &value) {
            PAxisParam_t param = Param::GetInstance()->GetPAxisParam_t();
            printf(
                "SAVE\nP\n x angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n y "
                "angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n z angle_offset:[%0.3f] "
                "kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n",
                param.x.angle_offset,
                param.x.kp,
                param.x.kv,
                param.x.ks,
                param.y.angle_offset,
                param.y.kp,
                param.y.kv,
                param.y.ks,
                param.z.angle_offset,
                param.z.kp,
                param.z.kv,
                param.z.ks);
            Param::GetInstance()->SavePBalanceParam();
        });

        serial_commander_.Register("SAVE_U", [](const std::string &value) {
            AxisParam_t u_param = Param::GetInstance()->GetUAxisParam();
            printf(
                "u\n angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f]\n",
                u_param.angle_offset,
                u_param.kp,
                u_param.kv,
                u_param.ks);
            Param::GetInstance()->SaveUBalanceParam();
        });

        serial_commander_.Register("GJ", [](const std::string &value) {
            JumpParam_t jump_param = Param::GetInstance()->GetJumpParam();
            printf(
                "GJ jump param\n (ch1: %0.3f, ch2: %0.3f, ch3: %0.3f)\n",
                jump_param.set_ch1_speed,
                jump_param.set_ch2_speed,
                jump_param.set_ch3_speed);
        });

        serial_commander_.Register("SAVE_J", [](const std::string &value) {
            Param::GetInstance()->SaveJumpParam();
            JumpParam_t jump_param = Param::GetInstance()->GetJumpParam();
            printf(
                "SAVE jump param\n (ch1: %0.3f, ch2: %0.3f, ch3: %0.3f)\n",
                jump_param.set_ch1_speed,
                jump_param.set_ch2_speed,
                jump_param.set_ch3_speed);
        });

        serial_commander_.Register("GV", [](const std::string &value) {
            VbusParam_t param = Param::GetInstance()->GetVbusParam();
            printf(
                "GJ vbus param\n (ntc opt: %0.3f ℃, low power: %0.3f)\n",
                param.ntc_protected_temp,
                param.vbus_protected_v);
        });

        serial_commander_.Register("SAVE_V", [](const std::string &value) {
            Param::GetInstance()->SaveVbusParam();
            VbusParam_t param = Param::GetInstance()->GetVbusParam();
            printf(
                "GJ vbus param\n (ntc opt: %0.3f ℃, low power: %0.3f)\n",
                param.ntc_protected_temp,
                param.vbus_protected_v);
        });

        serial_commander_.Register("RESET", [](const std::string &value) {
            Param::GetInstance()->FirstWriteParamToEeprom();
            Param::GetInstance()->LoadParam();
            PAxisParam_t param = Param::GetInstance()->GetPAxisParam_t();
            printf(
                "SAVE\nP\n x angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n y "
                "angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n z angle_offset:[%0.3f] "
                "kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n",
                param.x.angle_offset,
                param.x.kp,
                param.x.kv,
                param.x.ks,
                param.y.angle_offset,
                param.y.kp,
                param.y.kv,
                param.y.ks,
                param.z.angle_offset,
                param.z.kp,
                param.z.kv,
                param.z.ks);
            AxisParam_t u_param = Param::GetInstance()->GetUAxisParam();
            printf(
                "u\n angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f]\n",
                u_param.angle_offset,
                u_param.kp,
                u_param.kv,
                u_param.ks);
        });

        serial_commander_.Register("CALIBRATION_U_ANGLE", [&](const std::string &value) {
            Param::GetInstance()->GetUAxisParam().angle_offset =
                g_imu_comp_.GetImuData().angle.pitch;
            Param::GetInstance()->SaveUBalanceParam();
            printf(
                "CALIBRATION_U_ANGLE angle: %0.2f\n",
                Param::GetInstance()->GetUAxisParam().angle_offset);
        });

        serial_commander_.Register("CALIBRATION_P_ANGLE", [&](const std::string &value) {
            Param::GetInstance()->GetPAxisParam_t().x.angle_offset =
                g_imu_comp_.GetImuData().angle.pitch;
            Param::GetInstance()->GetPAxisParam_t().y.angle_offset =
                g_imu_comp_.GetImuData().angle.roll;
            Param::GetInstance()->SavePBalanceParam();
            printf(
                "CALIBRATION_P_ANGLE x angle: %0.2f y angle: %0.2f\n",
                Param::GetInstance()->GetPAxisParam_t().x.angle_offset,
                Param::GetInstance()->GetPAxisParam_t().y.angle_offset);
        });

        serial_commander_.Register("AUTO_U_START", [&](const std::string &value) {
            g_control_comp.StartUAutoCalibration();
            printf("AUTO_U_START\n");
        });
    
        serial_commander_.Register("AUTO_P_START", [&](const std::string &value) {
            g_control_comp.StartPAutoCalibration();
            printf("AUTO_P_START\n");
        });

        // U Calibration
        // REGISTER_COMMAND_FLOAT(
        //     serial_commander_, "AUTO_KP", g_control_comp.u_auto_calibration_.kp_speed_);
        // REGISTER_COMMAND_FLOAT(
        //     serial_commander_, "AUTO_KI", g_control_comp.u_auto_calibration_.ki_speed_);
        // REGISTER_COMMAND_FLOAT(
        //     serial_commander_,
        //     "AUTO_MAX",
        //     g_control_comp.u_auto_calibration_.max_offset_change_);
        // REGISTER_COMMAND_FLOAT(
        //     serial_commander_, "AUTO_DT", g_control_comp.u_auto_calibration_.dt_);
    }

    SerialCommander &GetSerialCommander() { return serial_commander_; }

   private:
    SerialCommander serial_commander_;
};

extern CommanderComp *g_commander_comp;
}  // namespace Control
}  // namespace CubliMini