#include "control/error_monitor.h"

namespace CubliMini {
namespace Control {
ErrorMonitor g_error_monitor;

void GetMotorStatuses(uint8_t rawValue, uint8_t &m1, uint8_t &m2, uint8_t &m3)
{
    uint8_t value1 = (rawValue >> (MOTOR1_ID * 2)) & 0x03;
    uint8_t value2 = (rawValue >> (MOTOR2_ID * 2)) & 0x03;
    uint8_t value3 = (rawValue >> (MOTOR3_ID * 2)) & 0x03;
    m1 = value1;
    m2 = value2;
    m3 = value3;
}

uint8_t SetMotorStatuses(FOCMotorInitStatus m1, FOCMotorInitStatus m2, FOCMotorInitStatus m3)
{
    uint8_t value = 0;
    value |= (m1 & 0x03) << (MOTOR1_ID * 2);
    value |= (m2 & 0x03) << (MOTOR2_ID * 2);
    value |= (m3 & 0x03) << (MOTOR3_ID * 2);
    return value;
}

}  // namespace Control
}  // namespace CubliMini