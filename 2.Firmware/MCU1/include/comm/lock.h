#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
namespace CubliMini {
namespace Comm {
class Mutex
{
   public:
    Mutex()
    {
        handle_ = xSemaphoreCreateMutex();
        configASSERT(handle_ != NULL);
    }

    ~Mutex() { vSemaphoreDelete(handle_); }

    void lock() { xSemaphoreTake(handle_, portMAX_DELAY); }

    void unlock() { xSemaphoreGive(handle_); }

    bool tryLock(TickType_t timeout = 0) { return xSemaphoreTake(handle_, timeout) == pdTRUE; }

   private:
    SemaphoreHandle_t handle_;
};
class MutexGuard
{
   public:
    explicit MutexGuard(Mutex &mutex) : mutex_(mutex) { mutex_.lock(); }

    ~MutexGuard() { mutex_.unlock(); }

   private:
    Mutex &mutex_;
};
}  // namespace Comm
}  // namespace CubliMini