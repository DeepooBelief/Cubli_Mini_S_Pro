#include "bsp/led.h"

#include <Arduino.h>
namespace CubliMini {
namespace Bsp {

void LedDriver::Init(int _led_pin)
{
    led_pin_ = _led_pin;
    pinMode(led_pin_, OUTPUT);
    digitalWrite(led_pin_, LED_OFF);
}

void LedDriver::AlwaysOn() { digitalWrite(led_pin_, LED_ON); }

void LedDriver::AlwaysOff() { digitalWrite(led_pin_, LED_OFF); }

/*
 *  输入1：_flashes_interval_time， 闪烁的间隔时间 , ms
 *  输入2：_number_of_flashes，闪烁的次数, ms
 *  输入3：_control_cycle，一个控制周期, ms
 */
void LedDriver::Control(int _flashes_interval_time, int _number_of_flashes, int _control_cycle)
{
    float time_interval = time_.GetTimeS() * 1000.0f;  // ms
    control_cycle_time_count_ += time_interval;
    flashes_interva_time_count_ += time_interval;

    if (control_cycle_time_count_ <= _control_cycle)
    {
        if (flashes_interva_time_count_ >= _flashes_interval_time)
        {
            if (number_of_flashes_count_ < _number_of_flashes * 2)
            {
                flashes_interva_time_count_ = 0;
                number_of_flashes_count_++;
                inversion_status_ = !inversion_status_;
                if (inversion_status_)
                {
                    digitalWrite(led_pin_, LED_ON);
                }
                else
                {
                    digitalWrite(led_pin_, LED_OFF);
                }
            }
            else
            {
                digitalWrite(led_pin_, LED_OFF);
                inversion_status_ = false;
            }
        }
    }
    else
    {
        control_cycle_time_count_   = 0;
        flashes_interva_time_count_ = 0;
        number_of_flashes_count_    = 0;
        digitalWrite(led_pin_, LED_OFF);
        inversion_status_ = false;
    }
}

void LedDriver::Control(
    std::function<void()> action1,
    std::function<void()> action2,
    int _flashes_interval_time,
    int _number_of_flashes,
    int _control_cycle)
{
    float time_interval = time_.GetTimeS() * 1000.0f;  // ms
    control_cycle_time_count_ += time_interval;
    flashes_interva_time_count_ += time_interval;

    if (control_cycle_time_count_ <= _control_cycle)
    {
        if (flashes_interva_time_count_ >= _flashes_interval_time)
        {
            if (number_of_flashes_count_ < _number_of_flashes * 2)
            {
                flashes_interva_time_count_ = 0;
                number_of_flashes_count_++;
                inversion_status_ = !inversion_status_;
                if (inversion_status_)
                {
                    if (action1)
                        action1();
                }
                else
                {
                    if (action2)
                        action2();
                }
            }
            else
            {
                if (action2)
                    action2();
                inversion_status_ = false;
            }
        }
    }
    else
    {
        control_cycle_time_count_   = 0;
        flashes_interva_time_count_ = 0;
        number_of_flashes_count_    = 0;
        if (action2)
            action2();
        inversion_status_ = false;
    }
}

void LedCom::Init()
{
    led_param_[INIT_LED]         = LedParam {0, 1};
    led_param_[BALANCESTATE_LED] = LedParam {1, 4};
    led_param_[SYSTEM_MODE_LED]  = LedParam {5, 2};
    led_param_[MOTOR3_LED]       = LedParam {7, 36};
    led_param_[MOTOR1_LED]       = LedParam {7 + 36, 36};
    led_param_[MOTOR2_LED]       = LedParam {7 + 36 * 2, 36};
    led_param_[ALL_LED]          = LedParam {0, SYSTEM_LED_NUM};

    g_system_strip.begin();
    g_system_strip.setBrightness(50);
    g_system_strip.setPixelColor(0, g_system_strip.Color(0, 0, 0));
    g_system_strip.show();
}

void LedCom::SetPixelColor(LED_TYPE type, uint16_t n, uint32_t c)
{
    g_system_strip.setPixelColor(led_param_[type].offset + n, c);
}

uint16_t LedCom::Num(LED_TYPE type) { return led_param_[type].num; }

void LedCom::Control(
    std::function<void()> action1,
    std::function<void()> action2,
    int _flashes_interval_time,
    int _number_of_flashes,
    int _control_cycle = 2000)
{
    led_driver_.Control(
        action1, action2, _flashes_interval_time, _number_of_flashes, _control_cycle);
}

void LedCom::Show() {
    g_system_strip.show();
}

LedCom g_led_com;

}  // namespace Bsp
}  // namespace CubliMini