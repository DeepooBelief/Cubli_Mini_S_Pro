#include "bsp/ntc.h"

#include "config/config.h"

namespace CubliMini {
namespace Bsp {

NtcClass g_ntc_motor1_mos(NTC_MOTOR1_MOS_PIN);
NtcClass g_ntc_motor2_mos(NTC_MOTOR2_MOS_PIN);
NtcClass g_ntc_motor3_mos(NTC_MOTOR3_MOS_PIN);
NtcClass g_ntc_power_mos(NTC_POWER_MOS_PIN);

}  // namespace Bsp
}  // namespace CubliMini