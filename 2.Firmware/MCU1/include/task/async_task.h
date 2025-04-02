#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Task {

class AsyncTask
{
   public:
    static QueueHandle_t async_task_quque;
    using Task = std::function<void()>;

    static void Run(void *parameter)
    {
        for (;;)
        {
            Task task;
            if (xQueueReceive(async_task_quque, &task, portMAX_DELAY) == pdTRUE)  // wait max 2000ms
            {
                task();
            }
        }
    }

    static void Init()
    {
        async_task_quque = xQueueCreate(1, sizeof(Task));
        xTaskCreatePinnedToCore(Run, "cmd task", 1024 * 3, NULL, 2, NULL, 1);
    }

    static void AddTask(Task task) { xQueueOverwrite(async_task_quque, &task); }
};
}  // namespace Task
}  // namespace CubliMini
