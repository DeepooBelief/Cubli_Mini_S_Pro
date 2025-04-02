#pragma once

#include "imu/ahrs.h"
#include "imu/qmi8658_driver.h"

namespace CubliMini {
namespace Imu {
using namespace ImuDriver;

class ImuComp
{
   public:
    ImuComp()
    {
        imu_driver_ptr_ = new Qmi8658Driver();
        ahrs_ptr_       = new CubliMini::Imu::AHRS(imu_driver_ptr_);
    }
    void Init(std::function<void(int res)> res_callback)
    {
        int res = imu_driver_ptr_->Init();
        if (res != 0)
        {
            res_callback(res);
            return;
        }
        x_queue_imu_ = xQueueCreate(1, sizeof(ImuData_t));
        res_callback(res);
    }

    /*
        阻塞型等待数据，需要单独创建一个task
    */
    void Loop()
    {
        ahrs_ptr_->AhrsLoop();
        imu_data_ = ahrs_ptr_->imu_data_;
        xQueueOverwrite(x_queue_imu_, &ahrs_ptr_->imu_data_);
    }

    void ReadImuData(ImuData_t *data, TickType_t xTicksToWait = 0)
    {
        xQueueReceive(x_queue_imu_, data, xTicksToWait);
    }

    ImuData_t GetImuRawData() const { return imu_driver_ptr_->GetImuRawData(); }

    Gyro_t GetImuOffset() const { return imu_driver_ptr_->GetImuOffset(); }

    ImuData_t GetImuData() const { return imu_data_; }

    void SetGyroYConvAngle(float angle = IMU_GYRO_Y_CONVERT_ANGLE_DEFAULT)
    {
        ahrs_ptr_->SetGyroYConvAngle(angle);
    }

   private:
    AHRS *ahrs_ptr_                = nullptr;
    ImuDriverBase *imu_driver_ptr_ = nullptr;
    QueueHandle_t x_queue_imu_;
    ImuData_t imu_data_;
};

extern ImuComp g_imu_comp_;
}  // namespace Imu
}  // namespace CubliMini