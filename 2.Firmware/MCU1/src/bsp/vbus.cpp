#include "bsp/vbus.h"
namespace CubliMini {
namespace Bsp {
Vbus g_vbus(VBUS_VOLTAGE_PIN, VBUS_NUMERATOR_RESISTOR, VBUS_DENOMINATOR_RESISTOR);
}
}  // namespace CubliMini