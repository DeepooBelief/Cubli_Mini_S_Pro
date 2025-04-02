#pragma once
#include <Arduino.h>

#include "comm/time.h"

namespace CubliMini {
namespace ImuDriver {

struct RawGyro_t
{
    volatile int16_t gx;
    volatile int16_t gy;
    volatile int16_t gz;
};

struct RawAcc_t
{
    volatile int16_t ax;
    volatile int16_t ay;
    volatile int16_t az;
};

struct ImuRawData_t
{
    volatile int16_t temp;
    RawGyro_t gyro;
    RawAcc_t acc;
};

struct EulerAngle_t
{
    float roll;
    float pitch;
    float yaw;
};

struct Gyro_t
{
    float gx;
    float gy;
    float gz;
};

struct Acc_t
{
    float ax;
    float ay;
    float az;
};

struct ImuData_t
{
    bool is_static;
    float temp;
    Acc_t acc;
    Gyro_t gyro;
    EulerAngle_t angle;
};

SemaphoreHandle_t *GetImuBaseSemaphore();
void ImuSemaphoreInit();
void setFlag(void);

// 行
#define MEAN_FILTER_ROWS 6
// 列
#define MEAN_FILTER_COLS 8

class ImuDriverBase
{
   public:
    ImuDriverBase(
        float imu_init_gyro_threshold,
        uint32_t IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME       = 500,
        uint32_t IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT = 2000,
        uint32_t IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUT    = 10,
        float IMU_TEMPERATURE_CONTROL_TARGET                = 50,
        float imu_static_gyro_threshold                     = 150,
        float IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIME         = 10,
        int mean_filter_rows                                = MEAN_FILTER_ROWS,
        int mean_filter_cols                                = MEAN_FILTER_COLS)
        : imu_init_gyro_threshold_(imu_init_gyro_threshold),
          init_(false),
          mean_filter_rows_(mean_filter_rows),
          mean_filter_cols_(mean_filter_cols),
          IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_(IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME),
          IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_(IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT),
          IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUT_(IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUT),
          IMU_TEMPERATURE_CONTROL_TARGET_(IMU_TEMPERATURE_CONTROL_TARGET),
          imu_static_gyro_threshold_(imu_static_gyro_threshold),
          IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIME_(IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIME)
    {
        memset(&mean_imu_raw_data_, 0, sizeof(mean_imu_raw_data_));
        memset(&imu_raw_data_, 0, sizeof(imu_raw_data_));
        memset(&imu_data_, 0, sizeof(imu_data_));
        memset(&gyro_offset_, 0, sizeof(gyro_offset_));
        memset(&mean_filter_fifo_, 0, sizeof(mean_filter_fifo_));
        ImuSemaphoreInit();
    }

    virtual ~ImuDriverBase()                          = default;
    virtual int Init()                                = 0;
    virtual bool Ready(TickType_t ms = portMAX_DELAY) = 0;
    virtual void TempControl()                        = 0;
    virtual bool HasTempControl()                     = 0;

    void Loop()
    {
        if (Ready())
        {
            ReadData();
            TempControl();
            RunIsStatic();
            imu_data_.is_static = is_static_;
            // AutoGyroZeroOffsetCalibration();
        }
    }

    ImuData_t GetImuData() const { return imu_data_; }
    ImuData_t GetImuRawData() const { return imu_raw_data_; }
    Gyro_t GetImuOffset() const { return gyro_offset_; }

    const bool IsStatic() const { return is_static_; }
    void Reset()
    {
        auto_zero_offset_cal_time_.reset();
        imu_static_time_.reset();
    }

   protected:
    void AutoGyroZeroOffsetCalibration()
    {
        static Gyro_t gyro_sum;
        static int count = 0;
        if (fabs(imu_data_.gyro.gx) < 3 && fabs(imu_data_.gyro.gy) < 3 &&
            fabs(imu_data_.gyro.gz) < 3)
        {
            if (auto_zero_offset_cal_time_.GetTimeS() >= IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIME_)
            {
                ++count;
                gyro_sum.gx += imu_raw_data_.gyro.gx;
                gyro_sum.gy += imu_raw_data_.gyro.gy;
                gyro_sum.gz += imu_raw_data_.gyro.gz;
                if (count >= IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_)
                {
                    gyro_offset_.gx = gyro_sum.gx / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                    gyro_offset_.gy = gyro_sum.gy / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                    gyro_offset_.gz = gyro_sum.gz / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                    count           = 0;
                    gyro_sum.gx     = 0;
                    gyro_sum.gy     = 0;
                    gyro_sum.gz     = 0;
                    auto_zero_offset_cal_time_.reset();
                }
            }
        }
        else
        {
            count       = 0;
            gyro_sum.gx = 0;
            gyro_sum.gy = 0;
            gyro_sum.gz = 0;
            auto_zero_offset_cal_time_.reset();
        }
    }
    void ReadData()
    {
        if (ReadRawData(imu_raw_data_))
        {
            MeanFilter(imu_raw_data_);
        }
        imu_data_.acc.ax  = mean_imu_raw_data_.acc.ax;
        imu_data_.acc.ay  = mean_imu_raw_data_.acc.ay;
        imu_data_.acc.az  = mean_imu_raw_data_.acc.az;
        imu_data_.gyro.gx = mean_imu_raw_data_.gyro.gx - gyro_offset_.gx;
        imu_data_.gyro.gy = mean_imu_raw_data_.gyro.gy - gyro_offset_.gy;
        imu_data_.gyro.gz = mean_imu_raw_data_.gyro.gz - gyro_offset_.gz;
        imu_data_.temp    = imu_raw_data_.temp;
    }

    void RunIsStatic()
    {
        if (fabs(imu_data_.gyro.gx) < imu_static_gyro_threshold_ &&
            fabs(imu_data_.gyro.gy) < imu_static_gyro_threshold_ &&
            fabs(imu_data_.gyro.gz) < imu_static_gyro_threshold_)
        {
            if (is_static_ == false)
            {
                if (imu_static_time_.GetTimeS() > 1.5f)
                {
                    imu_static_time_.reset();
                    is_static_ = true;
                }
            }
        }
        else
        {
            is_static_ = false;
            imu_static_time_.reset();
        }
    }

    virtual bool ReadRawData(ImuData_t &) = 0;

    int CalGyroOffset()
    {
        Comm::CumulativeTime zero_calibration_time;
        bool calibrate_gyro_zero_offset_seccess = false;
        Gyro_t gyro_sum;
        uint32_t count                 = 0;
        uint16_t read_timeout_attempts = 0;
        int last_printed_progress      = -1;
        uint32_t number_of_runs        = 0;
        if (!init_)
            return -3;

        if (HasTempControl())
        {
            uint32_t temp_control_count = 0;
            printf("IMU: has temperature control, start heating\n");
            for (;;)
            {
                if (!Ready(IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_))
                {
                    printf(
                        "IMU: Unable to read data!, Please check the IMU hardware interrupt pin\n");
                    ++read_timeout_attempts;
                    if (read_timeout_attempts >= 3)
                    {
                        printf("IMU: heating failed\n");
                        return -4;
                    }
                    continue;
                }
                ++temp_control_count;
                ReadData();
                TempControl();
                if (imu_raw_data_.temp >= IMU_TEMPERATURE_CONTROL_TARGET_ - 1)
                {
                    printf(
                        "IMU: heating completed, rt temp: %0.1f target: %0.1f\n",
                        imu_raw_data_.temp,
                        IMU_TEMPERATURE_CONTROL_TARGET_);
                    break;
                }
                if (zero_calibration_time.GetTimeS() > IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUT_)
                {
                    printf("IMU: heating fail, timeout!\n");
                    return -5;
                }
                if (temp_control_count % 400 == 0)
                {
                    printf(
                        "IMU: rt_temp: %0.3f target: %0.3f\n",
                        imu_raw_data_.temp,
                        IMU_TEMPERATURE_CONTROL_TARGET_);
                }
            }
            delay(2000);
        }

        zero_calibration_time.reset();
        read_timeout_attempts = 0;
        printf("IMU: Start zero offset calibration\n");
        while (calibrate_gyro_zero_offset_seccess == false)
        {
            if (!Ready(IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_))
            {
                printf("IMU: Unable to read data!, Please check the IMU hardware interrupt pin\n");
                ++read_timeout_attempts;
                if (read_timeout_attempts >= 3)
                {
                    printf("IMU: Zero offset initialization calibration failed\n");
                    return -6;
                }
                continue;
            }

            ReadData();
            TempControl();
            if (fabs(imu_raw_data_.gyro.gx) < imu_init_gyro_threshold_ &&
                fabs(imu_raw_data_.gyro.gy) < imu_init_gyro_threshold_ &&
                fabs(imu_raw_data_.gyro.gz) < imu_init_gyro_threshold_)
            {
                count++;
                gyro_sum.gx += imu_raw_data_.gyro.gx;
                gyro_sum.gy += imu_raw_data_.gyro.gy;
                gyro_sum.gz += imu_raw_data_.gyro.gz;

                int progress = count * 100.0f / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                if (progress % 25 == 0 && progress != last_printed_progress)
                {
                    printf(" %d%%", (int)progress);
                    last_printed_progress = (int)progress;
                }
                if (count >= IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_)
                {
                    calibrate_gyro_zero_offset_seccess = true;
                    gyro_offset_.gx = gyro_sum.gx / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                    gyro_offset_.gy = gyro_sum.gy / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                    gyro_offset_.gz = gyro_sum.gz / IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
                    printf(
                        "\nIMU: zero calibrate offset seccess, gx: %f gy: %f gz: %f\n",
                        gyro_offset_.gx,
                        gyro_offset_.gy,
                        gyro_offset_.gz);
                    return 0;
                }
            }
            else
            {
                if (number_of_runs % 200 == 0)
                {
                    printf(
                        "IMU: zero calibrate offset fail, please keep imu static! rt gx: %f gy: %f "
                        "gz: %f\n",
                        imu_raw_data_.gyro.gx,
                        imu_raw_data_.gyro.gy,
                        imu_raw_data_.gyro.gz);
                }
                count       = 0;
                gyro_sum.gx = 0;
                gyro_sum.gy = 0;
                gyro_sum.gz = 0;
            }

            if (zero_calibration_time.GetTimeS() > IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUT_)
            {
                printf("IMU: zero caloffset fail, timeout!\n");
                gyro_offset_.gx = 0;
                gyro_offset_.gy = 0;
                gyro_offset_.gz = 0;
                return -7;
            }
            ++number_of_runs;
        }

        return 0;
    }

    void MeanFilter(ImuData_t &_imu_raw_data)
    {
        for (int rows = 0; rows < mean_filter_rows_; ++rows)
        {
            for (int cols = 1; cols < mean_filter_cols_; ++cols)
            {
                mean_filter_fifo_[rows][cols - 1] = mean_filter_fifo_[rows][cols];
            }
        }

        mean_filter_fifo_[0][mean_filter_cols_ - 1] = _imu_raw_data.acc.ax;
        mean_filter_fifo_[1][mean_filter_cols_ - 1] = _imu_raw_data.acc.ay;
        mean_filter_fifo_[2][mean_filter_cols_ - 1] = _imu_raw_data.acc.az;
        mean_filter_fifo_[3][mean_filter_cols_ - 1] = _imu_raw_data.gyro.gx;
        mean_filter_fifo_[4][mean_filter_cols_ - 1] = _imu_raw_data.gyro.gy;
        mean_filter_fifo_[5][mean_filter_cols_ - 1] = _imu_raw_data.gyro.gz;

        for (int rows = 0; rows < mean_filter_rows_; ++rows)
        {
            float sum = 0;
            for (int cols = 0; cols < mean_filter_cols_; ++cols)
            {
                sum += mean_filter_fifo_[rows][cols];
            }
            mean_filter_fifo_[rows][mean_filter_cols_] = sum / mean_filter_cols_;
        }

        mean_imu_raw_data_.acc.ax  = mean_filter_fifo_[0][mean_filter_cols_];
        mean_imu_raw_data_.acc.ay  = mean_filter_fifo_[1][mean_filter_cols_];
        mean_imu_raw_data_.acc.az  = mean_filter_fifo_[2][mean_filter_cols_];
        mean_imu_raw_data_.gyro.gx = mean_filter_fifo_[3][mean_filter_cols_];
        mean_imu_raw_data_.gyro.gy = mean_filter_fifo_[4][mean_filter_cols_];
        mean_imu_raw_data_.gyro.gz = mean_filter_fifo_[5][mean_filter_cols_];
    }

    bool init_;

   private:
    std::function<void(void)> intRoutine_;
    ImuData_t mean_imu_raw_data_;
    Gyro_t gyro_offset_;
    float mean_filter_fifo_[6][15];
    float imu_init_gyro_threshold_;
    int mean_filter_rows_;
    int mean_filter_cols_;
    uint32_t static_frame_num_ = 0;
    bool is_static_;
    Comm::CumulativeTime auto_zero_offset_cal_time_;
    Comm::CumulativeTime imu_static_time_;

   protected:
    uint32_t IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_;
    uint32_t IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNT_;
    uint32_t IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUT_;
    float IMU_TEMPERATURE_CONTROL_TARGET_;
    ImuData_t imu_raw_data_;
    ImuData_t imu_data_;
    float imu_static_gyro_threshold_;
    float IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIME_;
};

}  // namespace ImuDriver
}  // namespace CubliMini
