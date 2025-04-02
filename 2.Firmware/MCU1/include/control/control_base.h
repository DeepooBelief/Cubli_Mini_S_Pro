#pragma once

#include <math.h>

namespace CubliMini {
namespace Control {

enum MotorControlMode
{
    TOUCH_e    = 0,
    VELOCITY_e = 1,
    BRAKE_e    = 2,
};

struct MotorTransMode
{
    uint8_t motor1_mode;
    uint8_t motor2_mode;
    uint8_t motor3_mode;

    static MotorTransMode Get(uint8_t value)
    {
        MotorTransMode this_mode;
        this_mode.motor1_mode = value & 0x03;
        this_mode.motor2_mode = (value >> 2) & 0x03;
        this_mode.motor3_mode = (value >> 4) & 0x03;
        return this_mode;
    }

    static uint8_t GetValue(MotorTransMode &other)
    {
        uint8_t value;
        value = other.motor1_mode | (other.motor2_mode << 2) | (other.motor3_mode << 4);
        return value;
    }
};

struct AxisSensor_t
{
    float angle;
    float gyro;
    float speed;
};

struct AxisParam_t
{
    float kp;
    float kv;
    float ks;
    float angle_offset;
};

struct JumpParam_t
{
    float set_ch1_speed;
    float set_ch2_speed;
    float set_ch3_speed;
};

struct VbusParam_t
{
    float measured_v;
    float ntc_protected_temp;
    float vbus_protected_v;
};

struct PAxisParam_t
{
    AxisParam_t x;
    AxisParam_t y;
    AxisParam_t z;
};

struct PSensor_t
{
    AxisSensor_t x;
    AxisSensor_t y;
    AxisSensor_t z;
};

struct MotorSpeed_t
{
    float ch1;
    float ch2;
    float ch3;
    uint8_t value;
    uint8_t control_mode;
};

struct AxisSpeed_t
{
    float x;
    float y;
    float z;
};

inline float AxisLqr(const AxisSensor_t &_sensor, const AxisParam_t &_param)
{
    return (_sensor.angle - _param.angle_offset) * _param.kp + _sensor.gyro * _param.kv +
           _sensor.speed * _param.ks;
}

}  // namespace Control
}  // namespace CubliMini