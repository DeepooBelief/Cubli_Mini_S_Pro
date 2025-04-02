#pragma once

#include <Arduino.h>

#include "comm/time.h"
#include "comm/comm.h"
#include "imu/imu_driver_base.h"
#include "config/config.h"

using namespace CubliMini::ImuDriver;
using namespace CubliMini::Comm;

namespace CubliMini {
namespace Imu {

// #define Kp 5.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
// #define Ki 0.01f  // integral gain governs rate of convergence of gyroscope biases

struct Q_t
{
    volatile float q0;
    volatile float q1;
    volatile float q2;
    volatile float q3;
};

class AHRS
{
   public:
    AHRS(ImuDriverBase *imu_driver);

    void AhrsLoop();
    bool IsStatic() const { return is_static_; }
    float ConvAngle(float raw_angle);
    void SetGyroYConvAngle(float angle = IMU_GYRO_Y_CONVERT_ANGLE_DEFAULT)
    { gyro_y_conv_angle_ = angle; }

   public:
    ImuData_t imu_data_;

   private:
    ImuDriverBase *imu_driver_;
    Q_t q_;
    CubliMini::Comm::Time time_;
    float kp_;
    float ki_;
    bool is_static_;

   private:
    void RunImuIsStatic();
    float invSqrt(float x);
    void ImuAHRSUpdate(Q_t &_q, const ImuData_t &_imu_data);
    void ConvertToEulerAmgleByQ(EulerAngle_t &_angle, const Q_t &_q);
    ConvertAngularVelocity gyro_conv_;
    float gyro_y_conv_angle_;
    float imu_static_gyro_threshold_;
};

}  // namespace Imu
}  // namespace CubliMini