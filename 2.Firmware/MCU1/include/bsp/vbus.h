#pragma once
#include <Arduino.h>

#include "bsp/adc.h"
#include "config/config.h"
namespace CubliMini {
namespace Bsp {

class Vbus
{
   public:
    const int kInvalidParameter = -1;
    Vbus(int vbus_pin, float numerator_resistor, float denominator_resistor, float measured_v = -1)
        : numerator_resistor_(numerator_resistor),
          denominator_resistor_(denominator_resistor),
          measured_v_(measured_v)
    {
        vbus_voltage_.Init(vbus_pin);
    }

    void SetMeasured(float measured_v)
    {
        measured_v_ = measured_v;
    }

    float GetVbusV()
    {
        if (measured_v_ == kInvalidParameter)
        {
            printf("VBUS: fail, please calibrate the VBUS voltage\n");
            return 0;
        }
        return GetVbusVoltageV() * measured_v_;
    }

    float VbusCalibration(float real_voltage)
    {
        float vbus_sum          = 0;
        uint32_t sampling_count = 500;
        for (uint32_t i = 0; i < sampling_count; ++i)
        {
            float read_vbus = GetVbusVoltageV();
            vbus_sum += read_vbus;
            delay(1);
        }
        float mean_vbus = vbus_sum / sampling_count;
        measured_v_     = real_voltage / mean_vbus;
        printf("VBUS: measured_v: %0.3f\n", measured_v_);
        return measured_v_;
    }

    const float GetMeasuredV() const {return measured_v_; }

   private:
    float ResistorVoltageDividerRatio()
    {
        if (numerator_resistor_ == kInvalidParameter && denominator_resistor_ == kInvalidParameter)
        {
            return 0;
        }
        return numerator_resistor_ / (numerator_resistor_ + denominator_resistor_);
    }

    float GetVbusVoltageV()
    {
        int adc_data = vbus_voltage_.GetAdcValue();
        return (float)adc_data / 4096.0f * 3.3f / ResistorVoltageDividerRatio();
    }
    AdcDriver vbus_voltage_;
    float measured_v_           = kInvalidParameter;
    float numerator_resistor_   = kInvalidParameter;  // 分子电阻
    float denominator_resistor_ = kInvalidParameter;  // 分母电阻
};

extern Vbus g_vbus;
}  // namespace Bsp
}  // namespace CubliMini