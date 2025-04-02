#pragma once
#include <Arduino.h>

namespace CubliMini {
namespace Config {

// WIFI comm
#define WIFI_DEFAULT_SSID            "ssid"
#define WIFI_DEFAULT_PASSWORD        "password"
#define WIFI_DEFAULT_TCP_SERVER_PORT 1312

// AP
#define WIFI_AP_SSID            "cubli_mini_s_pro"
#define WIFI_AP_PASSWORD        "88888888"
#define WEB_PORT                80

// PARAM CONFIG
#define P_BALANCE_X_P 0.8      // 2
#define P_BALANCE_X_V 0.08f  // 0.25
#define P_BALANCE_X_S 0.015f  // 0.02
#define P_BALANCE_X_A 5.500f

#define P_BALANCE_Y_P -0.8f
#define P_BALANCE_Y_V -0.07f
#define P_BALANCE_Y_S 0.015f
#define P_BALANCE_Y_A -58.000f

#define P_BALANCE_Z_P 0
#define P_BALANCE_Z_V 0.03f    // 0.1
#define P_BALANCE_Z_S -0.005f  // 0.02
#define P_BALANCE_Z_A 0

#define U_BALANCE_CH3_P 0.8f  // 3
#define U_BALANCE_CH3_V 0.11f
#define U_BALANCE_CH3_S 0.015f
#define U_BALANCE_CH3_A 3.0f
#define U_BALABCE_PROTECTION_THRESHOLD 20

// JUMP
#define JUNP_SET_CH1_SPEED  260
#define JUNP_SET_CH2_SPEED -260
#define JUNP_SET_CH3_SPEED -470

// LED
#define SYSTEM_LED_PIN 10
#define SYSTEM_LED_NUM (int)(36 * 3 + 7)

// KEY
#define KEY_MODE_PIN (int)26
#define KEY_ENTER_QUIT_PIN (int)27

// CAN
#define CAN_TXD_PIN (int)15
#define CAN_RXD_PIN (int)13

// OLED
#define OLED_IIC_SDA (int)9
#define OLED_IIC_SCL (int)4
#define OLED_IIC_RST (int)21

// NTC
#define NTC_MOTOR1_MOS_PIN (int)34
#define NTC_MOTOR2_MOS_PIN (int)35
#define NTC_MOTOR3_MOS_PIN (int)32
#define NTC_POWER_MOS_PIN  (int)33
#define NTC_PROTECTED_TEMP (int)60

// IMU
#define IMU_SPI_SCK_PIN                             (int)5
#define IMU_SPI_CS_PIN                              (int)18
#define IMU_SPI_MOSI_PIN                            (int)22
#define IMU_SPI_MISO_PIN                            (int)25

#define IMU_IIC_SDA                                 (int)5
#define IMU_IIC_SCL                                 (int)18
#define IMU_INTERRUPT_2_PIN                         (int)23
#define IMU_PWM_PIN                                 (int)19
#define IMU_INIT_GYRO_THRESHOLD                     150
#define IMU_STATIC_GYRO_THRESHOLD                   8
#define IMU_ZERO_OFFSET_SINGLE_READ_MAX_TIME_MS     500   // 零偏单次读取时间，ms
#define IMU_ZERO_OFFSET_CALIBRATION_SAMPLING_COUNTS 1000  // 零偏采样次数
#define IMU_ZERO_OFFSET_CALIBRATION_MAX_TIMEOUTS    10    // 零偏最大校准时间，s
#define IMU_TEMPERATURE_CONTROL_TARGETS             44    // 目标温度
#define IMU_ZERO_OFFSET_AUTO_CALIBRATION_TIMES      10    // 10S自动校准一次
#define IMU_GYRO_Y_CONVERT_ANGLE_DEFAULT            -35   // IMU Y轴角速度矫正

// VBUS
#define VBUS_VOLTAGE_PIN          (int)36
#define VBUS_NUMERATOR_RESISTOR   10    // 分子电阻
#define VBUS_DENOMINATOR_RESISTOR 42.2  // 分母电阻
#define VBUS_CALIBRATION_V        12.0  // 默认校准电压为12V
#define VBUS_PROTECTED_V          11.0  // 低电压阈值为11V

}  // namespace Config
}  // namespace CubliMini