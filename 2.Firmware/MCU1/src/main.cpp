#include <Arduino.h>

#include "bsp/bsp.h"
#include "config/config.h"
#include "control/angle_offset_calibration.h"
#include "control/commander_comp.h"
#include "control/control.h"
#include "control/error_monitor.h"
#include "control/param.h"
#include "control/serial_commander.h"
#include "imu/imu_comp.h"
#include "state/sub_state.h"
#include "task/async_task.h"
#include "web/web_comp.h"

using namespace CubliMini::Bsp;
using namespace CubliMini::Control;
using namespace CubliMini::State;
using namespace CubliMini::Imu;
using namespace CubliMini::Web;
using namespace CubliMini::Task;

void CmdTask(void *parameter);
void ImuTask(void *parameter);
void CanRecTask(void *parameter);
void ControlTask(void *parameter);
void CanSendTask(void *parameter);
void StateTask(void *parameter);
void OledTask(void *parameter);
void WebCompTask(void *parameter);

QueueHandle_t xQueueCanSend;
QueueHandle_t xQueueCanGet;

#define BIT_0    (1 << 0)  // IMU
#define BIT_1    (1 << 1)  // MOTOR SPEED
#define ALL_BITS (BIT_0 | BIT_1)
EventGroupHandle_t xEventGroup;

TaskHandle_t task_handle[8];

void setup()
{
    Serial.begin(115200);
    delay(200);

    Param::GetInstance()->Load();
    g_vbus.SetMeasured(Param::GetInstance()->GetVbusParam().measured_v);
    g_oled_control->Init();
    g_led_com.Init();
    AsyncTask::Init();
    xTaskCreatePinnedToCore(OledTask, "oled task", 1024 * 3, NULL, 2, &task_handle[0], 1);
    delay(100);
    g_machine = new StateMachine();
    g_machine->Init();
    xTaskCreatePinnedToCore(StateTask, "state task", 1024 * 5, NULL, 9, &task_handle[1], 1);
    delay(100);

    for (;;)
    {
        // wait imu init
        if (g_machine->getCurrentStateType() == StateType::IDLE)
            break;
        delay(200);
    }

    xEventGroup   = xEventGroupCreate();
    xQueueCanSend = xQueueCreate(1, sizeof(PBalanceControl::MotorControl));
    xQueueCanGet  = xQueueCreate(1, sizeof(MotorSpeed_t));
    g_can.Init();

    xTaskCreatePinnedToCore(ImuTask, "imu task", 1024 * 2, NULL, 8, &task_handle[2], 1);
    xTaskCreatePinnedToCore(CanRecTask, "can task", 1024 * 2, NULL, 6, &task_handle[3], 1);
    xTaskCreatePinnedToCore(CanSendTask, "can1 task", 1024 * 2, NULL, 4, &task_handle[4], 1);
    xTaskCreatePinnedToCore(ControlTask, "control task", 1024 * 5, NULL, 7, &task_handle[5], 1);
    xTaskCreatePinnedToCore(WebCompTask, "WebCompTask", 1024 * 8, NULL, 7, &task_handle[6], 1);
    delay(200);
    xTaskCreatePinnedToCore(CmdTask, "cmd task", 1024 * 5, NULL, 3, &task_handle[7], 1);
}

void CanSendTask(void *parameter)
{
    for (;;)
    {
        PBalanceControl::MotorControl send_motor_control;
        if (xQueueReceive(xQueueCanSend, &send_motor_control, portMAX_DELAY) == pdTRUE)
        {
            g_can.CanSendMotorSpeed_t(
                send_motor_control.ch1,
                send_motor_control.ch2,
                send_motor_control.ch3,
                send_motor_control.value,
                send_motor_control.control_mode);
        }
    }
}

void CanRecTask(void *parameter)
{
    for (;;)
    {
        MotorSpeed_t get_motor_speed;
        if (g_can.CanGetMotorSpeed_t(get_motor_speed.ch1, get_motor_speed.ch2, get_motor_speed.ch3))
        {
            xQueueOverwrite(xQueueCanGet, &get_motor_speed);
            xEventGroupSetBits(xEventGroup, BIT_1);
        }
    }
}

void ImuTask(void *parameter)
{
    for (;;)
    {
        g_imu_comp_.Loop();
        xEventGroupSetBits(xEventGroup, BIT_0);
    }
}

void WebCompTask(void *parameter)
{
    g_web_comp = new WebComp(WIFI_AP_SSID, WIFI_AP_PASSWORD, WEB_PORT);
    g_web_comp->Init();
    for (;;)
    {
        g_web_comp->Loop();
    }
}

void CmdTask(void *parameter)
{
    g_commander_comp = new CommanderComp;
    g_commander_comp->RegisterInit();
    printf("commander is ready\n");
    for (;;)
    {
        g_commander_comp->GetSerialCommander().Run(Serial);
        g_error_monitor.Loop();
        delay(20);
    }
}

void StateTask(void *parameter)
{
    for (;;)
    {
        g_machine->update();
        delay(40);
    }
}

void OledTask(void *parameter)
{
    for (;;)
    {
        g_oled_control->Loop();
    }
}

void ControlTask(void *parameter)
{
    EventBits_t uxBits;
    ImuData_t rx_imu_data;
    MotorSpeed_t rx_motor_speed;
    PBalanceControl::MotorControl send_motor_control = {0, 0, 0, 0};
    for (;;)
    {
        uxBits = xEventGroupWaitBits(xEventGroup, ALL_BITS, pdTRUE, pdTRUE, portMAX_DELAY);

        if ((uxBits & ALL_BITS) == ALL_BITS)
        {
            g_imu_comp_.ReadImuData(&rx_imu_data, 0);
            xQueueReceive(xQueueCanGet, &rx_motor_speed, 0);

            PSensor_t p_sensor = {
                {rx_imu_data.angle.pitch, rx_imu_data.gyro.gy, 0},
                {rx_imu_data.angle.roll,  rx_imu_data.gyro.gx, 0},
                {rx_imu_data.angle.yaw,   rx_imu_data.gyro.gz, 0}
            };

            MotorSpeed_t p_motor_speed {
                rx_motor_speed.ch1, rx_motor_speed.ch3, rx_motor_speed.ch2};

            AxisSensor_t u_sensor = {
                rx_imu_data.angle.pitch, rx_imu_data.gyro.gz, rx_motor_speed.ch3};

            send_motor_control =
                g_control_comp.Loop(p_sensor, p_motor_speed, u_sensor, rx_imu_data.is_static);
            xQueueOverwrite(xQueueCanSend, &send_motor_control);
        }
    }
}

void loop()
{
    // printf(
    //     "u(a: %0.2f) p(xa: %0.2f, ya: %0.2f) speed: (m1: %0.2f, m2: %0.2f, m3: %0.2f)\n",
    //     Param::GetInstance()->GetUAxisParam().angle_offset,
    //     Param::GetInstance()->GetPAxisParam_t().x.angle_offset,
    //     Param::GetInstance()->GetPAxisParam_t().y.angle_offset,
    //     g_can.GetMotorSpeed_t().ch1,
    //     g_can.GetMotorSpeed_t().ch2,
    //     g_can.GetMotorSpeed_t().ch3);
    delay(100);
}