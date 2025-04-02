#pragma once
#include <Arduino.h>
#include "comm/time.h"
#include <Adafruit_NeoPixel.h>
#include "config/config.h"

namespace CubliMini {
namespace Bsp {

#define LED_ON LOW
#define LED_OFF HIGH

class LedDriver
{
   public:
    LedDriver()
    {
        control_cycle_time_count_ = 0.0f;
        flashes_interva_time_count_ = 0.0f;
        number_of_flashes_count_ = 0;
        inversion_status_ = false;
    }
    void Init(int _led_pin);
    void Control(int _flashes_interval_time, int _number_of_flashes, int _control_cycle);
    void Control(std::function<void()> action1, std::function<void()> action2, int _flashes_interval_time, int _number_of_flashes, int _control_cycle);

    void AlwaysOn();
    void AlwaysOff();

   private:
    int led_pin_;
    Comm::Time time_;
    // 累计一个周期内的时间
    float control_cycle_time_count_;
    // 一个闪烁周期内累积的时间
    float flashes_interva_time_count_;
    // 累计闪烁的次数，一个闪烁等于两次电平翻转
    int number_of_flashes_count_;
    // 电平状态
    bool inversion_status_;
};

class LedCom
{
    public:
        static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
            return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }

        struct Color_t
        {
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };

        enum LED_TYPE
        {
            MOTOR1_LED = 0,
            MOTOR2_LED,
            MOTOR3_LED,
            INIT_LED,
            SYSTEM_MODE_LED,
            BALANCESTATE_LED,
            ALL_LED,
            COUNT // 不要使用
        };

        struct LedParam
        {
            uint16_t offset;
            uint16_t num;
        };
        
        void Init();
        uint16_t Num(LED_TYPE type);
        void SetPixelColor(LED_TYPE type, uint16_t n, uint32_t c);
        void Control(
            std::function<void()> action1, 
            std::function<void()> action2, 
            int _flashes_interval_time,
            int _number_of_flashes,
            int _control_cycle);

        void Show();
        void AllTrueOff(LED_TYPE type)
        {
            for(uint8_t i = 0; i < Num(type); ++i){
                SetPixelColor(type, i, 0);
            }
        }
        void AllTrueOn(LED_TYPE type, uint32_t color)
        {
            for(uint8_t i = 0; i < Num(type); ++i){
                SetPixelColor(type, i, color);
            }
        }

    private:
        LedParam led_param_[COUNT];
        Adafruit_NeoPixel g_system_strip =
            Adafruit_NeoPixel(SYSTEM_LED_NUM, SYSTEM_LED_PIN, NEO_GRB + NEO_KHZ800);
        LedDriver led_driver_;
};

extern LedCom g_led_com;

} // namespace CubliMini
} // namespace Bsp