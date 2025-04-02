#pragma once

#include <Arduino.h>

namespace CubliMini {
namespace State {

enum class StateType
{
    None = 0,
    INIT,
    IDLE,
    U_BALANCE,
    P_BALANCE,
    MOTOR_TEST,
    U_MANUAL_CALIBRATION,
    P_MANUAL_CALIBRATION,
    U_AUTO_CALIBRATION,
    P_AUTO_CALIBRATION,
    U_JUMP,
    P_JUMP,
    ERROR,
    MainC
};

}
}  // namespace CubliMini