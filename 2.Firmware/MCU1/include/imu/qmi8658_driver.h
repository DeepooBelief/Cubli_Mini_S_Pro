#pragma once
#include <Arduino.h>

#include "SensorQMI8658.hpp"
#include "config/config.h"
#include "imu/imu_driver_base.h"

namespace CubliMini {
namespace ImuDriver {

/**
 * 优先SPI，中断1K hz
 */
class Qmi8658Driver : public ImuDriverBase
{
   public:
    // Qmi8658Driver(
    //     TwoWire *write,
    //     uint8_t addr = QMI8658_H_SLAVE_ADDRESS,
    //     int sda      = IMU_SPI_CS_PIN,
    //     int scl      = IMU_SPI_SCK_PIN,
    //     int int1     = IMU_SPI_MOSI_PIN,
    //     int int2     = IMU_SPI_MISO_PIN)
    //     : ImuDriverBase(IMU_INIT_GYRO_THRESHOLD, IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME),
    //       wire_ptr_(write),
    //       addr_(addr),
    //       sda_(sda),
    //       scl_(scl),
    //       int1_(int1),
    //       int2_(int2),
    //       is_spi_(false)
    // {}

    Qmi8658Driver(
        int spi_cs   = IMU_SPI_CS_PIN,
        int spi_sck  = IMU_SPI_SCK_PIN,
        int spi_mosi = IMU_SPI_MOSI_PIN,
        int spi_miso = IMU_SPI_MISO_PIN,
        int int2     = IMU_INTERRUPT_2_PIN,
        int pwm_pin  = IMU_PWM_PIN)
        : ImuDriverBase(
              IMU_INIT_GYRO_THRESHOLD,
              IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_MS,
              IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNTS,
              IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUTS,
              IMU_TEMPERATURE_CONTROL_TARGETS,
              IMU_STATIC_GYRO_THRESHOLD,
              IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIMES),
          spi_cs_(spi_cs),
          spi_sck_(spi_sck),
          spi_mosi_(spi_mosi),
          spi_miso_(spi_miso),
          int2_(int2),
          pwm_pin_(pwm_pin),
          is_spi_(true)
    {}

    Qmi8658Driver(
        TwoWire *write,
        int sda     = IMU_IIC_SDA,
        int scl     = IMU_IIC_SCL,
        int int2    = IMU_INTERRUPT_2_PIN,
        int pwm_pin = IMU_PWM_PIN)
        : ImuDriverBase(
              IMU_INIT_GYRO_THRESHOLD,
              IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_MS,
              IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNTS,
              IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUTS,
              IMU_TEMPERATURE_CONTROL_TARGETS,
              IMU_STATIC_GYRO_THRESHOLD,
              IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIMES),
          wire_ptr_(write),
          sda_(sda),
          scl_(scl),
          int2_(int2),
          pwm_pin_(pwm_pin),
          addr_(QMI8658_H_SLAVE_ADDRESS),
          is_spi_(false)
    {}

    ~Qmi8658Driver() = default;

    int Init() override
    {
        printf("IMU: start initialization\n");
        if (is_spi_)
        {
            if (!qmi_.begin(spi_cs_, spi_mosi_, spi_miso_, spi_sck_))
            {
                return -1;
            }
        }
        else
        {
            if (!qmi_.begin(*wire_ptr_, addr_, sda_, scl_))
            {
                return -1;
            }
        }
        int id = qmi_.whoAmI();
        printf("IMU: id: %x\n", id);
        if (id != 0x05)
        {
            return -2;
        }

        if (pwm_pin_)
        {
            pinMode(pwm_pin_, OUTPUT);
        }
        pinMode(int2_, INPUT_PULLUP);
        attachInterrupt(int2_, setFlag, RISING);

        qmi_.configAccelerometer(
            SensorQMI8658::ACC_RANGE_4G,
            SensorQMI8658::ACC_ODR_500Hz,
            SensorQMI8658::LPF_MODE_0,
            true);
        qmi_.configGyroscope(
            SensorQMI8658::GYR_RANGE_1024DPS,
            SensorQMI8658::GYR_ODR_448_4Hz,
            SensorQMI8658::LPF_MODE_3,
            true);

        qmi_.enableGyroscope();
        qmi_.enableAccelerometer();
        qmi_.enableINT(SensorQMI8658::IntPin2);
        qmi_.enableDataReadyINT();
        qmi_.dumpCtrlRegister();
        init_ = true;
        printf("IMU: IMU initialization successful\n");
        delay(1000);
        int res = CalGyroOffset();
        printf("res: %d\n", res);
        return res;
    }
    bool Ready(TickType_t ms = portMAX_DELAY) override
    {
        if (int2_)
        {
            return xSemaphoreTake(*GetImuBaseSemaphore(), ms) == pdTRUE;
        }
        else
        {
            return qmi_.getDataReady();
        }
    }

    bool HasTempControl() override { return pwm_pin_ != -1; }
    void TempControl() override
    {
        TempControl(imu_raw_data_.temp, IMU_TEMPERATURE_CONTROL_TARGET_);
    }

   protected:
    void TempControl(float rt_temperature, uint8_t target_temperature_c)
    {
        // static uint32_t debug_count = 0;
        // ++debug_count;
        if (pwm_pin_ == -1)
        {
            return;
        }
        float temp_d = ((float)target_temperature_c - rt_temperature);
        if (temp_d < 0)
            temp_d = 0;
        int pwm = temp_d * 25;
        if (pwm > 255)
            pwm = 255;
        analogWrite(pwm_pin_, pwm);
        // if(debug_count % 200 == 0)
        // {
        //     printf("IMU: rt_temp: %0.3f target: %d pwm: %d\n", rt_temperature,
        //     target_temperature_c, pwm);
        // }
    }

    bool ReadRawData(ImuData_t &raw_data) override
    {
        float ax, ay, az;
        bool is_read_data = false;
        if (qmi_.getAccelerometer(ax, ay, az))
        {
            raw_data.acc.ax = ax;
            raw_data.acc.ay = ay;
            raw_data.acc.az = az;
            is_read_data    = true;
        }
        float gx, gy, gz;
        if (qmi_.getGyroscope(gx, gy, gz))
        {
            raw_data.gyro.gx = gx;
            raw_data.gyro.gy = gy;
            raw_data.gyro.gz = gz;
            is_read_data     = true;
        }
        raw_data.temp = qmi_.getTemperature_C();
        return is_read_data;
    }

   private:
    TwoWire *wire_ptr_ = NULL;
    uint8_t addr_;
    int sda_;
    int scl_;
    int int1_;
    int int2_ = -1;

    int spi_sck_;
    int spi_cs_;
    int spi_miso_;
    int spi_mosi_;
    int pwm_pin_ = -1;
    SensorQMI8658 qmi_;
    bool is_spi_ = false;
};
}  // namespace ImuDriver
}  // namespace CubliMini