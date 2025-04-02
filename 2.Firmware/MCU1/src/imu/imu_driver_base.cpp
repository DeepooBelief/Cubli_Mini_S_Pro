#include "imu/imu_driver_base.h"

namespace CubliMini {
namespace ImuDriver {

SemaphoreHandle_t g_x_semaphore_imu_base;
SemaphoreHandle_t *GetImuBaseSemaphore() { return &g_x_semaphore_imu_base; }
void ImuSemaphoreInit() { g_x_semaphore_imu_base = xSemaphoreCreateBinary(); }
void setFlag(void)
{
    // printf("imu xSemaphoreGive\n");
    xSemaphoreGive(*GetImuBaseSemaphore());
}
}  // namespace ImuDriver
}  // namespace CubliMini
