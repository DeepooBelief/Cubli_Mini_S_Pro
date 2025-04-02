#pragma once
#include "BSP/bsp.h"
#include "Comm/time.h"
#include "control/control.h"
#include "control/param.h"
#include "state/sub_state.h"

namespace CubliMini {
namespace Control {
using namespace Bsp;
using namespace State;
using namespace Comm;

#define MOTOR1_ID 0
#define MOTOR2_ID 1
#define MOTOR3_ID 2

enum FOCMotorInitStatus
{
    motor_sense_ok           = 0x00,
    motor_encoder_fail       = 0x01,
    motor_current_sense_fail = 0x02
};

void GetMotorStatuses(uint8_t rawValue, uint8_t &m1, uint8_t &m2, uint8_t &m3);

uint8_t SetMotorStatuses(FOCMotorInitStatus m1, FOCMotorInitStatus m2, FOCMotorInitStatus m3);

class ErrorMonitor
{
   public:
    ErrorMonitor()  = default;
    ~ErrorMonitor() = default;

    void Loop()
    {
        if (g_vbus.GetVbusV() <= Param::GetInstance()->GetVbusParam().vbus_protected_v)
        {
            if (vbat_save_time_.GetTimeS() >= 3)
            {
                g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::LOW_P));
            }
        }
        else
        {
            vbat_save_time_.reset();
        }

        if (g_ntc_motor1_mos.IsOverTemp(Param::GetInstance()->GetVbusParam().ntc_protected_temp))
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M1_OPT));
        }
        else
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M1_OPT));
        }

        if (g_ntc_motor2_mos.IsOverTemp(Param::GetInstance()->GetVbusParam().ntc_protected_temp))
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M2_OPT));
        }
        else
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M2_OPT));
        }

        if (g_ntc_motor3_mos.IsOverTemp(Param::GetInstance()->GetVbusParam().ntc_protected_temp))
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M3_OPT));
        }
        else
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M3_OPT));
        }
        if (g_ntc_power_mos.IsOverTemp(Param::GetInstance()->GetVbusParam().ntc_protected_temp))
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::P_OPT));
        }
        else
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::P_OPT));
        }
        if (g_can.CanIsOnline() == eOFF_LINE)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::NO_MCU2));
        }
        else
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::NO_MCU2));
        }

        motor1_status_ = (g_can.motor_status_ >> (MOTOR1_ID * 2)) & 0x03;
        motor2_status_ = (g_can.motor_status_ >> (MOTOR2_ID * 2)) & 0x03;
        motor3_status_ = (g_can.motor_status_ >> (MOTOR3_ID * 2)) & 0x03;
        // printf("g_can.motor_status_: %d (1: %d, 2: %d, 3: %d)\n", g_can.motor_status_,
        //     motor1_status_, motor2_status_, motor3_status_);

        if (motor1_status_ == motor_encoder_fail)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M1_E_E));
        }
        else if (motor1_status_ == motor_current_sense_fail)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M1_C_E));
        }
        else if (motor1_status_ == motor_sense_ok)
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M1_C_E));
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M1_E_E));
        }

        if (motor2_status_ == motor_encoder_fail)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M2_E_E));
        }
        else if (motor2_status_ == motor_current_sense_fail)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M2_C_E));
        }
        else if (motor2_status_ == motor_sense_ok)
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M2_C_E));
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M2_E_E));
        }

        if (motor3_status_ == motor_encoder_fail)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M3_E_E));
        }
        else if (motor3_status_ == motor_current_sense_fail)
        {
            g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::M3_C_E));
        }
        else if (motor3_status_ == motor_sense_ok)
        {
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M3_C_E));
            g_error_id.ResetBit(static_cast<int>(ErrorState::ERROR_MSG::M3_E_E));
        }

        if (g_error_id.GetValue() != 0 && g_machine->getCurrentStateType() != StateType::ERROR)
        {
            g_machine->pushEvent(static_cast<int>(StateType::ERROR));
        }
        if (g_error_id.GetValue() == 0 && g_machine->getCurrentStateType() == StateType::ERROR)
        {
            g_machine->pushEvent(static_cast<int>(StateType::IDLE));
        }
    }

   private:
    CumulativeTime vbat_save_time_;
    uint8_t motor1_status_;
    uint8_t motor2_status_;
    uint8_t motor3_status_;
};

extern ErrorMonitor g_error_monitor;
}  // namespace Control
}  // namespace CubliMini