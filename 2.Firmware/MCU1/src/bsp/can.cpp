#include "bsp/can.h"

CAN_device_t CAN_cfg;

namespace CubliMini {
namespace Bsp {

CanDriver g_can;

void CanDriver::Init(int _can_txd_pin, int _can_rxd_pin, CAN_speed_t _can_speed)
{
    CAN_cfg.tx_pin_id = (gpio_num_t)_can_txd_pin;
    CAN_cfg.rx_pin_id = (gpio_num_t)_can_rxd_pin;
    CAN_cfg.speed     = _can_speed;
    CAN_cfg.rx_queue  = xQueueCreate(5, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
}

void CanDriver::SendMessage(uint32_t msg_id, uint8_t message[8])
{
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF  = CAN_frame_std;
    tx_frame.FIR.B.RTR = CAN_no_RTR;
    tx_frame.MsgID     = msg_id;
    tx_frame.FIR.B.DLC = 8;
    memcpy(tx_frame.data.u8, message, sizeof(tx_frame.data.u8));
    ESP32Can.CANWriteFrame(&tx_frame);
}

void CanDriver::CanSendMotorSpeed_t(
    float &_set_ch1_speed,
    float &_set_ch2_speed,
    float &_set_ch3_speed,
    uint8_t &motor_mode,
    uint8_t control)
{
    uint8_t message[8];
    message[0] = (int16_t)(_set_ch1_speed * 54.0f) & 0xff;
    message[1] = (int16_t)(_set_ch1_speed * 54.0f) >> 8;
    message[2] = (int16_t)(_set_ch2_speed * 54.0f) & 0xff;
    message[3] = (int16_t)(_set_ch2_speed * 54.0f) >> 8;
    message[4] = (int16_t)(_set_ch3_speed * 54.0f) & 0xff;
    message[5] = (int16_t)(_set_ch3_speed * 54.0f) >> 8;
    message[6] = motor_mode;
    message[7] = control;
    SendMessage(eCAN_GET_MOTOR_SPEED_FRAME, message);
}

void EncodeSpeed(uint8_t *message, float ch1, float ch2, float ch3)
{
    message[0]    = (uint16_t)(fabs(ch1) * 100.0f) & 0xff;
    message[1]    = (uint16_t)(fabs(ch1) * 100.0f) >> 8;
    message[2]    = (uint16_t)(fabs(ch2) * 100.0f) & 0xff;
    message[3]    = (uint16_t)(fabs(ch2) * 100.0f) >> 8;
    message[4]    = (uint16_t)(fabs(ch3) * 100.0f) & 0xff;
    message[5]    = (uint16_t)(fabs(ch3) * 100.0f) >> 8;
    uint8_t value = 0;
    if (ch1 < 0)
        value = value | 0x01;
    if (ch2 < 0)
        value = value | 0x02;
    if (ch3 < 0)
        value = value | 0x04;
    message[6] = value;
}

void DecodeSpeed(float &ch1, float &ch2, float &ch3, const uint8_t *message)
{
    ch1 = (uint16_t)(message[0] | message[1] << 8) / 100.0f;
    ch2 = (uint16_t)(message[2] | message[3] << 8) / 100.0f;
    ch3 = (uint16_t)(message[4] | message[5] << 8) / 100.0f;
    if (message[6] & 0x01)
    {
        ch1 = -ch1;
    }
    if (message[6] & 0x02)
    {
        ch2 = -ch2;
    }
    if (message[6] & 0x04)
    {
        ch3 = -ch3;
    }
}

bool CanDriver::CanGetMotorSpeed_t(
    float &_get_ch1_speed, float &_get_ch2_speed, float &_get_ch3_speed)
{
    CAN_frame_t rx_frame;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, portMAX_DELAY) == pdTRUE)
    {
        // 标准帧和数据帧
        if (rx_frame.FIR.B.FF == CAN_frame_std && rx_frame.FIR.B.RTR == CAN_no_RTR)
        {
            if (rx_frame.MsgID == eCAN_SEND_MOTOR_SPEED_FRAME)
            {
                // DecodeSpeed(_get_ch1_speed, _get_ch2_speed, _get_ch3_speed, rx_frame.data.u8);
                _get_ch1_speed = (int16_t)(rx_frame.data.u8[0] | rx_frame.data.u8[1] << 8) / 60.0f;
                _get_ch2_speed = (int16_t)(rx_frame.data.u8[2] | rx_frame.data.u8[3] << 8) / 60.0f;
                _get_ch3_speed = (int16_t)(rx_frame.data.u8[4] | rx_frame.data.u8[5] << 8) / 60.0f;
                motor_speed_.ch1 = _get_ch1_speed;
                motor_speed_.ch2 = _get_ch2_speed;
                motor_speed_.ch3 = _get_ch3_speed;
                motor_status_    = rx_frame.data.u8[7];
                rx_frame_count_  = 0;
                can_is_online_   = eON_LINE;
                return true;
            }
        }
    }

    return false;
}

CanStatus_e CanDriver::CanIsOnline()
{
    rx_frame_count_++;
    if (rx_frame_count_ > CAN_OFFLINE_COUNT)
    {
        can_is_online_ = eOFF_LINE;
    }
    return can_is_online_;
}

}  // namespace Bsp
}  // namespace CubliMini