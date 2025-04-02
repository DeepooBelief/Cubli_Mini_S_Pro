#pragma once
#include "bsp/bsp.h"
namespace CubliMini {
namespace Control {
using namespace Bsp;
class MotorLedControl
{
    struct LedColor
    {
        uint16_t r;
        uint16_t g;
        uint16_t b;
    };

   public:
    MotorLedControl() = default;
    MotorLedControl(LedCom::LED_TYPE led_type) : led_type_(led_type) {}

    void RuningControl(float motor_speed)
    {
        LedColor rgb {0, 0, 0};
        const int total_leds   = g_led_com.Num(led_type_);
        const bool is_negative = motor_speed < 0;

        // 计算速度绝对值对应的LED数量
        int index       = static_cast<int>(fabs(motor_speed)) / 5;
        int active_leds = std::min(index, total_leds);

        // 根据速度方向计算起始位置
        int start_pos = is_negative ? total_leds - 1 : 0;
        int end_pos   = is_negative ? total_leds - active_leds : active_leds;
        int step      = is_negative ? -1 : 1;

        // 计算颜色梯度（保持原有逻辑）
        int temp = 50 + index * 10;
        if (temp <= 255)
        {
            rgb.r = temp;
        }
        else if (temp <= 510)
        {  // 255 * 2=510
            rgb.r = 220;
            rgb.b = std::min(255, 100 + temp % 255);
        }
        else
        {
            rgb.r = 220;
            rgb.b = 220;
            rgb.g = std::min(255, 100 + temp % 255);
        }

        // 点亮有效LED（正反向逻辑）
        for (int i = start_pos; is_negative ? (i >= end_pos) : (i < end_pos); i += step)
        {
            g_led_com.SetPixelColor(led_type_, i, LedCom::Color(rgb.r, rgb.g, rgb.b));
        }

        // 熄灭剩余LED（正反向逻辑）
        int unlit_start = is_negative ? end_pos - 1 : active_leds;
        int unlit_end   = is_negative ? -1 : total_leds;
        step            = is_negative ? -1 : 1;

        for (int i = unlit_start; is_negative ? (i > unlit_end) : (i < unlit_end); i += step)
        {
            g_led_com.SetPixelColor(led_type_, i, 0);
        }
    }

   private:
    int offset = 0;
    LedCom::LED_TYPE led_type_;
};
}  // namespace Control
}  // namespace CubliMini