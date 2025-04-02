#pragma once
#include <Arduino.h>
namespace CubliMini
{
namespace Bsp
{

class NtcClass
{
public:
    NtcClass()
    {
    }
    NtcClass(int ntc_pin)
    {
        ntc_pin_ = ntc_pin;
        pinMode(ntc_pin_, INPUT);
    }

    ~NtcClass() = default;

    float Ntc2TempV(float adc_voltage)
    {
        float rThermistor = base_r * ((max_adc_value / adc_voltage) - 1);
        float tKelvin = (beta * roomTemp) / (beta + (roomTemp * log(rThermistor / roomTempR)));
        float tCelsius = tKelvin - 273.15; // 将开尔文转换为摄氏温度
        return tCelsius;
    }

    float Ntc2TempV()
    {
        if(ntc_pin_ == -1) 
        {
            return 0;
        }
        int ntc_raw_value = analogRead(ntc_pin_);
        return Ntc2TempV(ntc_raw_value);
    }

    bool IsOverTemp(float over_temp)
    {
        return Ntc2TempV() >= over_temp;
    }

private:
    const float beta = 3435.0;       // 商家给出的电阻对应25°C下的bata值
    const float roomTemp = 298.15;   // 以开尔文为单位的室温25°C
    const float roomTempR = 10000.0; // NTC热敏电阻在室温25°C下具有典型的电阻
    const float base_r = 4700;
    const float max_adc_value = 4096.0f;
    int ntc_pin_ = -1;
};

extern NtcClass g_ntc_motor1_mos;
extern NtcClass g_ntc_motor2_mos;
extern NtcClass g_ntc_motor3_mos;
extern NtcClass g_ntc_power_mos;
}
}