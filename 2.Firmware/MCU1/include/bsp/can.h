#pragma once
#include <Arduino.h>
#include <CAN_config.h>
#include <ESP32CAN.h>
#include "comm/type.h"
#include "config/config.h"

using namespace CubliMini::Comm;

namespace CubliMini {
namespace Bsp {

#define CAN_OFFLINE_COUNT 500  // 250次,0.5s

enum CanFrameId_e
{
    eCAN_SEND_MOTOR_SPEED_FRAME = 0x101,
    eCAN_GET_MOTOR_SPEED_FRAME  = 0x100,
    eCAN_GET_MOTOR_HEALT_FRAME  = 0x102,
    eCAN_GET_SERVO_MOTOR_FRAME  = 0x103
};

struct MotorSpeed_t_t
{
    float ch1;
    float ch2;
    float ch3;
};

class CanDriver
{
   public:
    CanDriver()
    {
        rx_frame_count_ = 0;
        can_is_online_  = eOFF_LINE;
    }

    void Init(int _can_txd_pin = CAN_TXD_PIN, int _can_rxd_pin = CAN_RXD_PIN, CAN_speed_t _can_speed = CAN_SPEED_1000KBPS);

    void CanSendMotorSpeed_t(
        float &_set_ch1_speed, float &_set_ch2_speed, float &_set_ch3_speed, uint8_t& motor_mode, uint8_t control);

    bool CanGetMotorSpeed_t(float &_get_ch1_speed, float &_get_ch2_speed, float &_get_ch3_speed);

    CanStatus_e CanIsOnline();
    CanStatus_e can_is_online_;

    MotorSpeed_t_t GetMotorSpeed_t() const 
    {
        return motor_speed_;
    }
    
    uint8_t motor_status_;
   private:
    void SendMessage(uint32_t msg_id, uint8_t msggage[8]);
    uint32_t rx_frame_count_;
    MotorSpeed_t_t motor_speed_;
};

extern CanDriver g_can;

}  // namespace Bsp
}  // namespace CubliMini