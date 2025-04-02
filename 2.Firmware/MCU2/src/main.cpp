#include <Adafruit_NeoPixel.h>
#include <SimpleFOC.h>

#include "SPI.h"
#include "bsp/can.h"
#include "comm/bit32.h"
#include "comm/comm.h"
#include "comm/time.h"
#include "config/config.h"
#include "driver/timer.h"
#include "mt6701/MagneticSensorMT6701SSI.h"

using namespace Cubli::Comm;
using namespace Cubli::Bsp;

#define MOTOR_MAX_TORQUE 20.0f   // rad/s
#define MOTOR_MAX_SPEED  600.0f  // rad/s

#define MOTOR_DEFAULT_VOLTAGE_LIMIT 5.0f
#define MOTOR_SPEED_LIMIT_START     550  // rad/s
#define MOTOR_SPEED_LIMIT_END       600  // rad/s

#define SYSTEM_LED_NUM 1
#define SYSTEM_LED_PIN 12

Adafruit_NeoPixel system_strip =
    Adafruit_NeoPixel(SYSTEM_LED_NUM, SYSTEM_LED_PIN, NEO_GRB + NEO_KHZ800);

BLDCMotor motor1       = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTOR1_PWM1_PIN, MOTOR1_PWM2_PIN, MOTOR1_PWM3_PIN);

BLDCMotor motor2       = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTOR2_PWM1_PIN, MOTOR2_PWM2_PIN, MOTOR2_PWM3_PIN);

BLDCMotor motor3       = BLDCMotor(MOTOR_PP, MOTOR_PHASE_RESISTANCE, MOTOR_KV);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(MOTOR3_PWM1_PIN, MOTOR3_PWM2_PIN, MOTOR3_PWM3_PIN);

struct SendMotorSpeed_t
{
    float ch1;
    float ch2;
    float ch3;
};

enum ControlMode
{
    NORMAL_MODE    = 0,
    P_JUMPING_MODE = 1,
    P_BALANCE_MODE = 2,
    TEST_MODE      = 3,
    U_BALANCE_MODE = 4
};

struct GetMotorCmd_t
{
    float ch1;
    float ch2;
    float ch3;
    Cubli::Bsp::MotorTransMode trans_mode;
    uint8_t control_mode;
};

enum FOCInit
{
    M2_INIT_STATUS = 0,
    M3_INIT_STATUS,
};

SendMotorSpeed_t g_set_motor_speed;
SendMotorSpeed_t g_send_motor_speed;
CubliMini::Comm::Bit32 g_foc_init_status;

hw_timer_t *g_foc_timer = NULL;
SemaphoreHandle_t g_xISR_Semaphore_foc;

float set_ch3_debug_motor_speed = 0;
Commander g_command               = Commander(Serial);

void MotorInputCal(GetMotorCmd_t &get_motor_cmd);
void CanReceiveTask(void *parameter);
void CanSendTask(void *parameter);
void MotorControlTask(void *parameter);
void MotorDriverLoop();

// 电流倍数，电阻，入参没有作用，构造已经指定
#define mvPa (float)100
InlineCurrentSense current_sense1 =
    InlineCurrentSense(0.0009f / 2, mvPa, MOTOR1_LIN_PINA, MOTOR1_LIN_PINB);
InlineCurrentSense current_sense2 =
    InlineCurrentSense(0.0009f / 2, mvPa, MOTOR2_LIN_PINA, MOTOR2_LIN_PINB);
InlineCurrentSense current_sense3 =
    InlineCurrentSense(0.0009f / 2, mvPa, MOTOR3_LIN_PINA, MOTOR3_LIN_PINB);

void doMotor1(char *cmd) { g_command.motor(&motor3, cmd); }
void doCh3Speed(char *cmd) { g_command.scalar(&set_ch3_debug_motor_speed, cmd); }

MagneticSensorMT6701SSI g_sensor1(MOTOR1_CS1);
MagneticSensorMT6701SSI g_sensor2(MOTOR2_CS2);
MagneticSensorMT6701SSI g_sensor3(MOTOR3_CS3);
SPIClass g_motor_spi(VSPI);
Cubli::Bsp::CanDriver g_can;

#define MOTOR1_ID 0
#define MOTOR2_ID 1
#define MOTOR3_ID 2

uint8_t SetMotorStatuses(FOCMotorInitStatus m1, FOCMotorInitStatus m2, FOCMotorInitStatus m3)
{
    uint8_t value = 0;
    value |= (m1 & 0x03) << (MOTOR1_ID * 2);
    value |= (m2 & 0x03) << (MOTOR2_ID * 2);
    value |= (m3 & 0x03) << (MOTOR3_ID * 2);
    return value;
}

struct MotorInitStatus
{
    std::string name;
    int res;
};

struct MotorInit
{
    MotorInitStatus init[3];
};

MotorInit motor_init_status;

// motor3
void UMotorFocInit(
    MagneticSensorMT6701SSI *sensor,
    BLDCDriver3PWM *driver,
    BLDCMotor *motor,
    InlineCurrentSense *current_sense,
    MotorInitStatus &motor_status)
{
    motor->linkSensor(sensor);
    // 驱动器设置
    driver->voltage_power_supply = 11.1;
    driver->init();
    motor->linkDriver(driver);

    current_sense->linkDriver(driver);
    current_sense->init();

    current_sense->gain_a *= -1;
    motor->linkCurrentSense(current_sense);

    motor->torque_controller = TorqueControlType::foc_current;
    motor->controller        = MotionControlType::torque;
    motor->motion_downsample = 0.0;

    // velocity loop PID
    motor->PID_velocity.P           = 0.2;
    motor->PID_velocity.I           = 10.0;
    motor->PID_velocity.D           = 0.0;
    motor->PID_velocity.output_ramp = 1000.0;
    motor->PID_velocity.limit       = 10;
    // Low pass filtering time constant
    motor->LPF_velocity.Tf = 0.005;

    // angle loop PID
    motor->P_angle.P           = 20.0;
    motor->P_angle.I           = 0.0;
    motor->P_angle.D           = 0.0;
    motor->P_angle.output_ramp = 0.0;
    motor->P_angle.limit       = 20.0;
    // Low pass filtering time constant
    motor->LPF_angle.Tf = 0.0;
    // current q loop PID
    motor->PID_current_q.P           = 2;
    motor->PID_current_q.I           = 150.0;
    motor->PID_current_q.D           = 0.0;
    motor->PID_current_q.output_ramp = 0.0;
    motor->PID_current_q.limit       = 5.8f;
    // Low pass filtering time constant
    motor->LPF_current_q.Tf = 0.002;
    // current d loop PID
    motor->PID_current_d.P           = 2;
    motor->PID_current_d.I           = 150.0;
    motor->PID_current_d.D           = 0.0;
    motor->PID_current_d.output_ramp = 0.0;
    motor->PID_current_d.limit       = 5.8;
    // Low pass filtering time constant
    motor->LPF_current_d.Tf = 0.005;

    // Limits
    motor->voltage_sensor_align = 0.7f;
    motor->velocity_limit       = 20.0;
    motor->voltage_limit        = 5;
    motor->current_limit        = 10;

    motor->useMonitoring(Serial);

    motor->monitor_downsample = 0;
    motor->monitor_variables  = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

    // 电机初始化
    motor->init();
    if (motor_status.res = motor->initFOC(), motor_status.res == 0)
    {
        motor_status.name = "Fail";
    }
    else
    {
        motor_status.name = "SUCCESS";
    }
}
void PointMotorFocInit(
    MagneticSensorMT6701SSI *sensor,
    BLDCDriver3PWM *driver,
    BLDCMotor *motor,
    InlineCurrentSense *current_sense,
    MotorInitStatus &motor_status)
{
    motor->linkSensor(sensor);
    // 驱动器设置
    driver->voltage_power_supply = 11.1;
    driver->init();
    motor->linkDriver(driver);

    current_sense->linkDriver(driver);
    current_sense->init();

    current_sense->gain_a *= -1;
    motor->linkCurrentSense(current_sense);

    motor->torque_controller = TorqueControlType::foc_current;
    motor->controller        = MotionControlType::torque;
    motor->motion_downsample = 0.0;

    // velocity loop PID
    motor->PID_velocity.P           = 0.2;
    motor->PID_velocity.I           = 10.0;
    motor->PID_velocity.D           = 0.0;
    motor->PID_velocity.output_ramp = 1000.0;
    motor->PID_velocity.limit       = 9;
    // Low pass filtering time constant
    motor->LPF_velocity.Tf = 0.005;

    // angle loop PID
    motor->P_angle.P           = 20.0;
    motor->P_angle.I           = 0.0;
    motor->P_angle.D           = 0.0;
    motor->P_angle.output_ramp = 0.0;
    motor->P_angle.limit       = 20.0;
    // Low pass filtering time constant
    motor->LPF_angle.Tf = 0.0;
    // current q loop PID
    motor->PID_current_q.P           = 2;
    motor->PID_current_q.I           = 150.0;
    motor->PID_current_q.D           = 0.0;
    motor->PID_current_q.output_ramp = 0.0;
    motor->PID_current_q.limit       = 6.0f;
    // Low pass filtering time constant
    motor->LPF_current_q.Tf = 0.002;
    // current d loop PID
    motor->PID_current_d.P           = 2;
    motor->PID_current_d.I           = 150.0;
    motor->PID_current_d.D           = 0.0;
    motor->PID_current_d.output_ramp = 0.0;
    motor->PID_current_d.limit       = 6.0;
    // Low pass filtering time constant
    motor->LPF_current_d.Tf = 0.005;

    // Limits
    motor->voltage_sensor_align = 0.7f;
    motor->velocity_limit       = 20.0;
    motor->voltage_limit        = 5;
    motor->current_limit        = 9;

    motor->useMonitoring(Serial);

    motor->monitor_downsample = 0;
    motor->monitor_variables  = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

    // 电机初始化
    motor->init();
    if (motor_status.res = motor->initFOC(), motor_status.res == 0)
    {
        motor_status.name = "Fail";
    }
    else
    {
        motor_status.name = "SUCCESS";
    }
}

void HandleTimer()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_xISR_Semaphore_foc, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void setup()
{
    Serial.begin(115200);
    delay(1000);
    pinMode(MOS_POWER_PIN, OUTPUT);
    digitalWrite(MOS_POWER_PIN, HIGH);

    system_strip.begin();
    system_strip.setBrightness(50);
    system_strip.setPixelColor(0, system_strip.Color(120, 0, 0));
    system_strip.show();

    g_sensor1.init(&g_motor_spi);
    g_sensor2.init(&g_motor_spi, true);
    g_sensor3.init(&g_motor_spi, true);

    // motor2 init
    xTaskCreatePinnedToCore(
        [](void *parameter) {
            UMotorFocInit(
                &g_sensor2, &driver2, &motor2, &current_sense2, motor_init_status.init[1]);
            g_foc_init_status.SetBit(FOCInit::M2_INIT_STATUS);
            vTaskDelete(NULL);
        },
        "Motor2 Init task", 1024 * 5, NULL, 4, NULL, 1);

    // motor3 init
    xTaskCreatePinnedToCore(
        [](void *parameter) {
            UMotorFocInit(
                &g_sensor3, &driver3, &motor3, &current_sense3, motor_init_status.init[2]);
            g_foc_init_status.SetBit(FOCInit::M3_INIT_STATUS);
            vTaskDelete(NULL);
        }, "Motor3 Init task", 1024 * 5, NULL, 5, NULL, 1);

    // motor1 init
    PointMotorFocInit(&g_sensor1, &driver1, &motor1, &current_sense1, motor_init_status.init[0]);
    g_can.Init(CAN_TXD_PIN, CAN_RXD_PIN, CAN_SPEED_1000KBPS);

    for (;;)
    {
        if (g_foc_init_status.GetValue() != 0x03)
        {
            delay(100);
            continue;
        }
        // check init status
        if (motor_init_status.init[0].res && motor_init_status.init[1].res &&
            motor_init_status.init[2].res)
        {
            break;
        }
        uint8_t motor_status = SetMotorStatuses(
            motor1.sensor_init_res, motor2.sensor_init_res, motor3.sensor_init_res);

        for (;;)
        {
            printf(
                "init status motor1 %s motor2 %s motor3 %s  int_res: %d %d %d\n",
                motor_init_status.init[0].name.c_str(),
                motor_init_status.init[1].name.c_str(),
                motor_init_status.init[2].name.c_str(),
                motor1.sensor_init_res,
                motor2.sensor_init_res,
                motor3.sensor_init_res);
            g_can.CanSendMotorSpeed(
                g_send_motor_speed.ch1,
                g_send_motor_speed.ch2,
                g_send_motor_speed.ch3,
                motor_status);
            delay(100);
        }
    }

    motor1.target = 0;
    motor2.target = 0;
    motor3.target = 0;

    xTaskCreatePinnedToCore(CanReceiveTask, "g_can receive task", 1024 * 5, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(CanSendTask, "g_can send task", 1024 * 5, NULL, 5, NULL, 1);

    g_command.add('M', doMotor1, "motor 3");
    g_command.add('T', doCh3Speed, "motor 3 speed");
    _delay(100);

    g_xISR_Semaphore_foc = xSemaphoreCreateBinary();
    g_foc_timer        = timerBegin(0, 80, true);
    timerAttachInterrupt(g_foc_timer, &HandleTimer, true);
    timerAlarmWrite(g_foc_timer, 300, true);  // 3.3K
    timerAlarmEnable(g_foc_timer);
    xTaskCreatePinnedToCore(MotorControlTask, "g_can send task", 1024 * 5, NULL, 7, NULL, 1);
    system_strip.setPixelColor(0, system_strip.Color(0, 120, 0));
    system_strip.show();
}

void CanReceiveTask(void *parameter)
{
    uint8_t get_motor_trans_mode;
    GetMotorCmd_t get_motor_cmd;
    for (;;)
    {
        if (g_can.CanGetMotorSpeed(
                get_motor_cmd.ch1,
                get_motor_cmd.ch2,
                get_motor_cmd.ch3,
                get_motor_trans_mode,
                get_motor_cmd.control_mode))
        {
            get_motor_cmd.trans_mode = MotorTransMode::Get(get_motor_trans_mode);
            MotorInputCal(get_motor_cmd);
        }
    }
}

void CanSendTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t motor_status =
        SetMotorStatuses(motor1.sensor_init_res, motor2.sensor_init_res, motor3.sensor_init_res);
    for (;;)
    {
        g_can.CanSendMotorSpeed(
            g_send_motor_speed.ch1, g_send_motor_speed.ch2, g_send_motor_speed.ch3, motor_status);
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}

float SetSpeedLimit(
    float set_motor_speed,
    float motor_speed,
    float speed_limit_start = MOTOR_SPEED_LIMIT_START,
    float speed_limit_end   = MOTOR_SPEED_LIMIT_END)
{
    float temp_set_spped                   = set_motor_speed;
    bool temp_set_spped_is_positive_number = true;
    if (temp_set_spped < 0)
    {
        temp_set_spped_is_positive_number = false;
    }

    if (fabs(motor_speed) <= speed_limit_start)
    {
        return temp_set_spped;
    }
    else if (fabs(motor_speed) > speed_limit_start && fabs(motor_speed) < speed_limit_end)
    {
        temp_set_spped =
            ((speed_limit_end - fabs(motor_speed)) * (speed_limit_end - fabs(motor_speed))) /
            (100.0f * 100.0f) * temp_set_spped;
        if (temp_set_spped_is_positive_number == false && temp_set_spped > 0)
        {
            temp_set_spped = -temp_set_spped;
        }
    }
    else if (fabs(motor_speed) > speed_limit_end)
    {
        temp_set_spped = 0;
    }
    return temp_set_spped;
}

void ModeControl(BLDCMotor &motor, uint8_t get_motor_mode, float &control_value)
{
    switch (get_motor_mode)
    {
    case TOUCH_e:
        motor.controller = MotionControlType::torque;
        control_value    = Limit(control_value, MOTOR_MAX_TORQUE);
        break;
    case VELOCITY_e:
        motor.controller = MotionControlType::velocity;
        control_value    = Limit(control_value, MOTOR_MAX_SPEED);
        break;
    default:
        break;
    }
}

void MotorInputCal(GetMotorCmd_t &get_motor_cmd)
{
    static uint8_t motor1_last_mode = 0;
    static uint8_t motor2_last_mode = 0;
    static uint8_t motor3_last_mode = 0;

    if (get_motor_cmd.control_mode == ControlMode::P_JUMPING_MODE)
    {
        motor1.PID_velocity.limit = 1.6f;
        motor2.PID_velocity.limit = 1.6f;
    }
    else if (get_motor_cmd.control_mode == ControlMode::P_BALANCE_MODE)
    {
        motor1.voltage_limit = 4.0f;
        motor2.voltage_limit = 4.0f;
        motor3.voltage_limit = 4.0f;

        motor1.current_limit = 6;
        motor2.current_limit = 6;
        motor3.current_limit = 6;

        motor1.PID_velocity.limit = 6;
        motor2.PID_velocity.limit = 6;
        motor2.PID_velocity.limit = 6;
    }
    else if (get_motor_cmd.control_mode == ControlMode::TEST_MODE)
    {
        motor1.voltage_limit = 0.7f;
        motor2.voltage_limit = 0.7f;
        motor3.voltage_limit = 0.7f;

        motor1.current_limit = 0.8;
        motor2.current_limit = 0.8;
        motor3.current_limit = 0.8;

        motor1.PID_velocity.limit = 1.0f;
        motor2.PID_velocity.limit = 1.0f;
        motor3.PID_velocity.limit = 1.0f;
    }
    else
    {
        motor1.voltage_limit = MOTOR_DEFAULT_VOLTAGE_LIMIT;
        motor2.voltage_limit = MOTOR_DEFAULT_VOLTAGE_LIMIT;
        motor3.voltage_limit = MOTOR_DEFAULT_VOLTAGE_LIMIT;

        motor1.current_limit = 9;
        motor2.current_limit = 9;
        motor3.current_limit = 10;

        motor1.PID_velocity.limit = 9;
        motor2.PID_velocity.limit = 9;
        motor3.PID_velocity.limit = 10;
    }

    ModeControl(motor1, get_motor_cmd.trans_mode.motor1_mode, get_motor_cmd.ch1);
    ModeControl(motor2, get_motor_cmd.trans_mode.motor2_mode, get_motor_cmd.ch2);
    ModeControl(motor3, get_motor_cmd.trans_mode.motor3_mode, get_motor_cmd.ch3);

    if (motor1.controller == MotionControlType::torque)
    {
        motor1_last_mode = 0;
    }
    if (motor1.controller == MotionControlType::velocity)
    {
        if (motor1_last_mode == 0)
        {
            motor1_last_mode = 1;
            motor1.PID_current_q.reset();
            motor1.PID_current_d.reset();
            motor1.PID_velocity.reset();
        }
    }

    if (motor2.controller == MotionControlType::torque)
    {
        motor2_last_mode = 0;
    }
    if (motor2.controller == MotionControlType::velocity)
    {
        if (motor2_last_mode == 0)
        {
            motor2_last_mode = 1;
            motor2.PID_current_q.reset();
            motor2.PID_current_d.reset();
            motor2.PID_velocity.reset();
        }
    }

    if (motor3.controller == MotionControlType::torque)
    {
        motor3_last_mode = 0;
    }
    if (motor3.controller == MotionControlType::velocity)
    {
        if (motor3_last_mode == 0)
        {
            motor3_last_mode = 1;
            motor3.PID_current_q.reset();
            motor3.PID_current_d.reset();
            motor3.PID_velocity.reset();
        }
    }

    if (motor1.controller == MotionControlType::torque)
    {
        if (get_motor_cmd.control_mode == P_BALANCE_MODE)
        {
            get_motor_cmd.ch1 = SetSpeedLimit(get_motor_cmd.ch1, motor1.shaft_velocity, 400, 450);
        }
        else
        {
            get_motor_cmd.ch1 = SetSpeedLimit(get_motor_cmd.ch1, motor1.shaft_velocity);
        }
    }

    if (motor2.controller == MotionControlType::torque)
    {
        if (get_motor_cmd.control_mode == P_BALANCE_MODE)
        {
            get_motor_cmd.ch2 = SetSpeedLimit(get_motor_cmd.ch2, motor2.shaft_velocity, 400, 450);
        }
        else
        {
            get_motor_cmd.ch2 = SetSpeedLimit(get_motor_cmd.ch2, motor2.shaft_velocity);
        }
    }

    if (motor3.controller == MotionControlType::torque)
    {
        if (get_motor_cmd.control_mode == P_BALANCE_MODE)
        {
            get_motor_cmd.ch3 = SetSpeedLimit(get_motor_cmd.ch3, motor3.shaft_velocity, 400, 450);
        }
        else
        {
            get_motor_cmd.ch3 = SetSpeedLimit(get_motor_cmd.ch3, motor3.shaft_velocity);
        }
    }

    if (motor2.sensor_direction == Direction::CCW)
    {
        get_motor_cmd.ch2 = -get_motor_cmd.ch2;
    }
    if (motor1.sensor_direction == Direction::CCW)
    {
        get_motor_cmd.ch1 = -get_motor_cmd.ch1;
    }
    if (motor3.sensor_direction == Direction::CCW)
    {
        get_motor_cmd.ch3 = -get_motor_cmd.ch3;
    }

    g_set_motor_speed.ch1 = get_motor_cmd.ch1;
    g_set_motor_speed.ch2 = get_motor_cmd.ch2;
    g_set_motor_speed.ch3 = get_motor_cmd.ch3;
}

void MotorControlTask(void *parameter)
{
    for (;;)
    {
        if (xSemaphoreTake(g_xISR_Semaphore_foc, portMAX_DELAY) == pdTRUE)
        {
            MotorDriverLoop();
        }
    }
}

void MotorDriverLoop()
{
    static uint64_t count = 0;
    ++count;
    motor1.loopFOC();
    if (count % 5 == 0)
    {
        motor1.move(g_set_motor_speed.ch1);
    }
    if (motor1.sensor_direction == Direction::CCW)
    {
        g_send_motor_speed.ch1 = -motor1.shaft_velocity;
    }
    else
    {
        g_send_motor_speed.ch1 = motor1.shaft_velocity;
    }

    motor2.loopFOC();
    if (count % 5 == 0)
    {
        motor2.move(g_set_motor_speed.ch2);
    }
    if (motor2.sensor_direction == Direction::CCW)
    {
        g_send_motor_speed.ch2 = -motor2.shaft_velocity;
    }
    else
    {
        g_send_motor_speed.ch2 = motor2.shaft_velocity;
    }

    motor3.loopFOC();
    if (count % 5 == 0)
    {
        motor3.move(g_set_motor_speed.ch3);
    }
    if (motor3.sensor_direction == Direction::CCW)
    {
        g_send_motor_speed.ch3 = -motor3.shaft_velocity;
    }
    else
    {
        g_send_motor_speed.ch3 = motor3.shaft_velocity;
    }
}

void loop()
{
    if (g_can.CanIsOnline() == eOFF_LINE)
    {
        g_set_motor_speed.ch1 = 0;
        g_set_motor_speed.ch1 = 0;
        g_set_motor_speed.ch1 = 0;
    }
    // motor3.monitor();
    // g_command.run();
    delay(100);
}

// TODO: sensor test
// void setup()
// {
//     Serial.begin(115200);
//     delay(300);
//     pinMode(MOS_POWER_PIN, OUTPUT);
//     digitalWrite(MOS_POWER_PIN, LOW);

//     g_sensor1.init(&g_motor_spi);
//     g_sensor2.init(&g_motor_spi, true);
//     g_sensor3.init(&g_motor_spi, true);

//     current_sense1.init();
//     current_sense1.gain_a *= -1;

//     current_sense2.init();
//     current_sense2.gain_a *= -1;

//     current_sense3.init();
//     current_sense3.gain_a *= -1;
// }

// void loop()
// {
//     g_sensor1.update();
//     g_sensor2.update();
//     g_sensor3.update();
//     printf("motor1 angle: %0.3f vel: %0.3f motor2 angle: %0.3f vel: %0.3f motor3 angle: %0.3f"
//     "vel: %0.3f\r\n",
//         g_sensor1.getAngle(),
//         g_sensor1.getVelocity(),
//         g_sensor2.getAngle(),
//         g_sensor2.getVelocity(),
//         g_sensor3.getAngle(),
//         g_sensor3.getVelocity());

//     PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
//     float current_magnitude1 = current_sense1.getDCCurrent();
//     PhaseCurrent_s currents2 = current_sense2.getPhaseCurrents();
//     float current_magnitude2 = current_sense2.getDCCurrent();
//     PhaseCurrent_s currents3 = current_sense3.getPhaseCurrents();
//     float current_magnitude3 = current_sense3.getDCCurrent();

//     // printf("motor1 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//     //     (currents1.a*-1000),
//     //     (currents1.b*1000),
//     //     (currents1.c*1000),
//     //     (current_magnitude1*1000));

//     //  printf("motor2 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//     //     (currents2.a*-1000),
//     //     (currents2.b*1000),
//     //     (currents2.c*1000),
//     //     (current_magnitude2*1000));

//     //  printf("motor3 a[%0.3f] mA b[%0.3f] mA c[%0.3f] mA cur[%0.3f] mA\n",
//     //     (currents3.a*-1000),
//     //     (currents3.b*1000),
//     //     (currents3.c*1000),
//     //     (current_magnitude3*1000));
//     delay(10);
// }