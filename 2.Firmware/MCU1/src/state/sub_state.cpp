#include "state/sub_state.h"

#include "bsp/bsp.h"
#include "control/control.h"
#include "control/param.h"
#include "imu/imu_comp.h"
#include "task/async_task.h"
namespace CubliMini {
namespace State {

using namespace Bsp;
using namespace Task;
using namespace Imu;
using namespace Control;

std::map<int, std::string> g_error_msg = {
    {static_cast<int>(ErrorState::ERROR_MSG::NO_IMU),   "NO IMU"  },
    {static_cast<int>(ErrorState::ERROR_MSG::NO_IMU_T), "NO IMU T"},
    {static_cast<int>(ErrorState::ERROR_MSG::M1_OPT),   "M1 OTP"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M2_OPT),   "M2 OTP"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M3_OPT),   "M3 OTP"  },
    {static_cast<int>(ErrorState::ERROR_MSG::P_OPT),    "P OTP"   },
    {static_cast<int>(ErrorState::ERROR_MSG::M1_C_E),   "M1 C E"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M2_C_E),   "M2 C E"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M3_C_E),   "M3 C E"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M1_E_E),   "M1 E E"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M2_E_E),   "M2 E E"  },
    {static_cast<int>(ErrorState::ERROR_MSG::M3_E_E),   "M3 E E"  },
    {static_cast<int>(ErrorState::ERROR_MSG::NO_MCU2),  "NO MCU2" },
    {static_cast<int>(ErrorState::ERROR_MSG::LOW_P),    "LOW P"   }
};

Bit32 g_error_id;
StateMachine *g_machine = nullptr;

void InitState::Enter(StateMachine &machine)
{
    time_.reset();
    AsyncTask::AddTask([&machine]() {
        g_imu_comp_.Init([&machine](int res) {
            if (res == 0)
            {
                AsyncTask::AddTask([&machine]() {
                    delay(2000);  // UI显示imu的offset
                    machine.pushEvent(static_cast<int>(StateType::IDLE));
                    g_led_com.SetPixelColor(LedCom::INIT_LED, 0, LedCom::Color(0, 100, 0));
                    g_led_com.Show();
                });
            }
            else
            {
                g_led_com.SetPixelColor(LedCom::INIT_LED, 0, LedCom::Color(100, 0, 0));
                g_led_com.Show();
                if (res == -2 || res == -1)
                    g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::NO_IMU));
                if (res == -5)
                    g_error_id.SetBit(static_cast<int>(ErrorState::ERROR_MSG::NO_IMU_T));
                if (g_error_id.GetValue() != 0 && machine.getCurrentStateType() != StateType::ERROR)
                {
                    machine.pushEvent(static_cast<int>(StateType::ERROR));
                }
            }
        });
    });
}

void InitState::Execute(StateMachine &machine)
{
    time_.Task(0.1f, []() {
        g_led_com.SetPixelColor(LedCom::INIT_LED, 0, LedCom::Color(255, 0, 0));
        g_led_com.Show();
        OLED_SEND([]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(5, "初始化");
            U8G2->setFont(u8g2_font_6x10_tf);
            OLED->DrawUTF8(0, 17, TO_STRING("V: %.1f V", g_vbus.GetVbusV()).c_str());
            OLED->DrawUTF8(13, 27, "IMU");
            OLED->DrawUTF8(
                0, 37, TO_STRING("T: %0.1f C", g_imu_comp_.GetImuRawData().temp).c_str());
            OLED->DrawUTF8(5, 47, "offset");
            OLED->DrawUTF8(0, 57, TO_STRING("gx: %.1f", g_imu_comp_.GetImuOffset().gx).c_str());
            OLED->DrawUTF8(0, 67, TO_STRING("gy: %.1f", g_imu_comp_.GetImuOffset().gy).c_str());
            OLED->DrawUTF8(0, 77, TO_STRING("gz: %.1f", g_imu_comp_.GetImuOffset().gz).c_str());
        });
    });
}

void IdleState::Enter(StateMachine &machine) {}

void IdleState::Execute(StateMachine &machine)
{
    static uint32_t count    = 0;
    static bool base_color_f = true;
    uint8_t base_color       = true;
    ++count;
    if (count % 80 == 0)
    {
        base_color_f = !base_color_f;
    }
    if (base_color_f)
        base_color = count % 80;
    else
        base_color = 79 - count % 80;

    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::U_BALANCE));
    });

    for (uint8_t i = 0; i < g_led_com.Num(LedCom::LED_TYPE::SYSTEM_MODE_LED); ++i)
    {
        g_led_com.SetPixelColor(
            LedCom::LED_TYPE::SYSTEM_MODE_LED, i, LedCom::Color(0, 0, base_color));
    }
    g_led_com.Show();

    time_.Task(0.1f, []() {
        OLED_SEND([]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(5, "空闲");
            U8G2->setFont(u8g2_font_6x10_tf);
            OLED->DrawUTF8(0, 20, TO_STRING("V: %.1f", g_vbus.GetVbusV()).c_str());
            OLED->DrawUTF8(
                0, 30, TO_STRING("x: %.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                0, 40, TO_STRING("y: %.1f", g_imu_comp_.GetImuData().angle.roll).c_str());
            OLED->DrawNumbleInCenter(50, TO_STRING("%d: %0.1fC", 1, g_ntc_motor1_mos.Ntc2TempV()));
            OLED->DrawNumbleInCenter(60, TO_STRING("%d: %0.1fC", 2, g_ntc_motor2_mos.Ntc2TempV()));
            OLED->DrawNumbleInCenter(70, TO_STRING("%d: %0.1fC", 3, g_ntc_motor3_mos.Ntc2TempV()));
            OLED->DrawNumbleInCenter(80, TO_STRING("%d: %0.1fC", 4, g_ntc_power_mos.Ntc2TempV()));
        });
    });
}

void IdleState::InExit(StateMachine &machine)
{
    for (uint8_t i = 0; i < g_led_com.Num(LedCom::LED_TYPE::SYSTEM_MODE_LED); ++i)
    {
        g_led_com.SetPixelColor(LedCom::LED_TYPE::SYSTEM_MODE_LED, i, LedCom::Color(0, 0, 0));
    }
    g_led_com.Show();
}

void UBalanceState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::P_BALANCE));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() {
        is_runing_ = !is_runing_;
        if (is_runing_)
        {
            g_control_comp.is_fast_runing_ = false;
            g_control_comp.contorl_mode_   = U_BALANCE_MODE;
        }
        else
        {
            g_control_comp.is_fast_runing_ = true;
            g_control_comp.contorl_mode_   = IDLE_MODE;
        }
    });

    if (fabs(
            g_imu_comp_.GetImuData().angle.pitch -
            Param::GetInstance()->GetUAxisParam().angle_offset) > 15.0f &&
        !is_runing_)
    {
        if (!machine.is_remote_control)
            g_control_comp.contorl_mode_ = IDLE_MODE;
    }

    motor_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch3);
    g_led_com.Show();

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(5, "边平衡");
            U8G2->setFont(u8g2_font_6x10_tf);
            OLED->DrawUTF8(0, 20, TO_STRING("V: %.1f", g_vbus.GetVbusV()).c_str());
            OLED->DrawUTF8(
                0, 30, TO_STRING("A: %.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                0,
                40,
                TO_STRING("SA: %.1f", Param::GetInstance()->GetUAxisParam().angle_offset).c_str());
            OLED->DrawUTF8(0, 50, TO_STRING("S: %.1f", g_can.GetMotorSpeed_t().ch3).c_str());

            OLED->DrawNumbleInCenter(60, TO_STRING("%d: %0.1fC", 3, g_ntc_motor3_mos.Ntc2TempV()));
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            if (is_runing_)
                OLED->DrawUTF8(14, 75, "RUN");
            else
                OLED->DrawUTF8(11, 75, "STOP");
        });
    });
}

void UBalanceState::InExit(StateMachine &machine)
{
    machine.is_remote_control      = false;
    g_control_comp.is_fast_runing_ = true;
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void PBalanceState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::MOTOR_TEST));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() {
        is_runing_ = !is_runing_;
        if (is_runing_)
        {
            g_control_comp.is_fast_runing_ = false;
            g_control_comp.contorl_mode_   = POINT_BALANCE_MODE;
        }
        else
        {
            g_control_comp.is_fast_runing_ = true;
            g_control_comp.contorl_mode_   = IDLE_MODE;
        }
    });

    if (fabs(
            g_imu_comp_.GetImuData().angle.pitch -
            Param::GetInstance()->GetPAxisParam_t().x.angle_offset) > 15.0f &&
        fabs(
            g_imu_comp_.GetImuData().angle.roll -
            Param::GetInstance()->GetPAxisParam_t().y.angle_offset) > 15.0f &&
        !is_runing_)
    {
        g_control_comp.contorl_mode_ = IDLE_MODE;
    }

    motor1_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch1);
    motor2_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch2);
    motor3_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch3);
    g_led_com.Show();

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(5, "点平衡");
            U8G2->setFont(u8g2_font_6x10_tf);
            OLED->DrawUTF8(0, 20, TO_STRING("V: %.1f", g_vbus.GetVbusV()).c_str());
            OLED->DrawUTF8(
                0, 30, TO_STRING("X: %.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                0, 40, TO_STRING("Y: %.1f", g_imu_comp_.GetImuData().angle.roll).c_str());

            OLED->DrawUTF8(0, 50, TO_STRING("S1: %.1f", g_can.GetMotorSpeed_t().ch1).c_str());
            OLED->DrawUTF8(0, 60, TO_STRING("S2: %.1f", g_can.GetMotorSpeed_t().ch2).c_str());
            OLED->DrawUTF8(0, 70, TO_STRING("S3: %.1f", g_can.GetMotorSpeed_t().ch3).c_str());

            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            if (is_runing_)
                OLED->DrawUTF8(14, 80, "RUN");
            else
                OLED->DrawUTF8(11, 80, "STOP");
        });
    });
}

void PBalanceState::InExit(StateMachine &machine)
{
    g_control_comp.is_fast_runing_ = true;
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void MotorTestState::Execute(StateMachine &machine)
{
    static uint8_t wait_time_s = 3;
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::U_MANUAL_CALIBRATION));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() {
        is_runing_ = !is_runing_;
        if (is_runing_)
            g_control_comp.contorl_mode_ = MOTOR_TEST_MODE;
        else
        {
            g_control_comp.contorl_mode_ = IDLE_MODE;
        }
    });

    if(is_runing_)
    {
        time_calibration_.Task(wait_time_s, [](){
            static bool change = true;
            change = !change; 
            if(change)
            {
                wait_time_s = 3;
                g_control_comp.contorl_mode_ = MOTOR_TEST_MODE;
            }
            else
            {
                wait_time_s = 7;
                g_control_comp.contorl_mode_ = MOTOR_TEST_MODE2;
            }
        });
    }

    motor1_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch1);
    motor2_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch2);
    motor3_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch3);
    g_led_com.Show();

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(3, "电机测试");
            U8G2->setFont(u8g2_font_6x10_tf);
            if (is_runing_)
                OLED->DrawUTF8(17, 17, "RUN");
            else
                OLED->DrawUTF8(15, 17, "STOP");

            OLED->DrawUTF8(0, 27, TO_STRING("s1:%.1f", g_can.GetMotorSpeed_t().ch1).c_str());
            OLED->DrawUTF8(0, 37, TO_STRING("s2:%.1f", g_can.GetMotorSpeed_t().ch2).c_str());
            OLED->DrawUTF8(0, 47, TO_STRING("s3:%.1f", g_can.GetMotorSpeed_t().ch3).c_str());
            OLED->DrawNumbleInCenter(57, TO_STRING("%d: %0.1fC", 1, g_ntc_motor1_mos.Ntc2TempV()));
            OLED->DrawNumbleInCenter(67, TO_STRING("%d: %0.1fC", 2, g_ntc_motor2_mos.Ntc2TempV()));
            OLED->DrawNumbleInCenter(77, TO_STRING("%d: %0.1fC", 3, g_ntc_motor3_mos.Ntc2TempV()));
        });
    });
}

void MotorTestState::InExit(StateMachine &machine)
{
    g_control_comp.contorl_mode_ = IDLE_MODE;
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void UCalibrationState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::P_MANUAL_CALIBRATION));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() {
        is_runing_ = !is_runing_;
        if (!is_runing_)
        {
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
            g_led_com.Show();
        }
    });

    if (is_runing_)
    {
        g_led_com.Control(
            []() {
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR3_LED, LedCom::Color(120, 120, 120));
                g_led_com.Show();
            },
            []() {
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            },
            100,
            1,
            count_ * 100);
    }

    if (is_runing_)
    {
        calibration_countdown_.Task(1, [this]() {
            --count_;
            if (count_ == 0)
            {
                is_runing_ = false;
                count_     = 3;
                Param::GetInstance()->GetUAxisParam().angle_offset =
                    g_imu_comp_.GetImuData().angle.pitch;
                Param::GetInstance()->SaveUBalanceParam();
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            }
        });
    }
    else
    {
        calibration_countdown_.reset();
    }

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "手动校准");
            OLED->DrawChineseInCenter(22, "边");
            OLED->DrawUTF8(
                5, 45, TO_STRING("A: %.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                5,
                55,
                TO_STRING("SA:%.1f", Param::GetInstance()->GetUAxisParam().angle_offset).c_str());
            if (is_runing_)
            {
                OLED->DrawUTF8(15, 70, TO_STRING(" %d", count_).c_str());
            }
            else
            {
                OLED->DrawChineseInCenter(70, "就绪");
            }
        });
    });
}

void UCalibrationState::InExit(StateMachine &machine)
{
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void PCalibrationState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::U_AUTO_CALIBRATION));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() {
        is_runing_ = !is_runing_;
        if (!is_runing_)
        {
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
            g_led_com.Show();
        }
    });

    if (is_runing_)
    {
        g_led_com.Control(
            []() {
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR1_LED, LedCom::Color(120, 120, 120));
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR2_LED, LedCom::Color(120, 120, 120));
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR3_LED, LedCom::Color(120, 120, 120));
                g_led_com.Show();
            },
            []() {
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            },
            100,
            1,
            count_ * 100);
    }

    if (is_runing_)
    {
        calibration_countdown_.Task(1, [this]() {
            --count_;
            if (count_ == 0)
            {
                is_runing_ = false;
                count_     = 3;
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
                Param::GetInstance()->GetPAxisParam_t().x.angle_offset =
                    g_imu_comp_.GetImuData().angle.pitch;
                Param::GetInstance()->GetPAxisParam_t().y.angle_offset =
                    g_imu_comp_.GetImuData().angle.roll;
                Param::GetInstance()->SavePBalanceParam();
            }
        });
    }
    else
    {
        calibration_countdown_.reset();
    }

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "手动校准");
            OLED->DrawChineseInCenter(22, "点");
            U8G2->setFont(u8g2_font_6x10_tf);
            OLED->DrawUTF8(
                5, 35, TO_STRING("XA:%.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                5, 45, TO_STRING("YA:%.1f", g_imu_comp_.GetImuData().angle.roll).c_str());
            OLED->DrawUTF8(
                5,
                55,
                TO_STRING("SX:%.1f", Param::GetInstance()->GetPAxisParam_t().x.angle_offset)
                    .c_str());
            OLED->DrawUTF8(
                5,
                65,
                TO_STRING("SY:%.1f", Param::GetInstance()->GetPAxisParam_t().y.angle_offset)
                    .c_str());
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            if (is_runing_)
            {
                OLED->DrawUTF8(15, 80, TO_STRING(" %d", count_).c_str());
            }
            else
            {
                OLED->DrawChineseInCenter(80, "就绪");
            }
        });
    });
}

void PCalibrationState::InExit(StateMachine &machine)
{
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void UAutoCalibrationState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::P_AUTO_CALIBRATION));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this, &machine]() {
        is_runing_ = !is_runing_;
        if (is_runing_)
        {
            g_control_comp.is_fast_runing_ = false;
            g_control_comp.contorl_mode_   = U_BALANCE_MODE;
            is_start_calibrationstate_     = true;
        }
        else
        {
            is_start_calibrationstate_     = false;
            g_control_comp.is_fast_runing_ = true;
            g_control_comp.contorl_mode_   = IDLE_MODE;
            g_control_comp.StopUAutoCalibration();
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
            g_led_com.Show();
        }
    });

    if (is_runing_)
    {
        g_led_com.Control(
            []() {
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR3_LED, LedCom::Color(120, 120, 120));
                g_led_com.Show();
            },
            []() {
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            },
            100,
            1,
            500);
        if (g_control_comp.U_balance_control_.IsBalance() && is_start_calibrationstate_)
        {
            time_calibration_.Task(1, [this, &machine]() {
                is_start_calibrationstate_ = false;
                g_control_comp.StartUAutoCalibration([this, &machine]() {
                    is_runing_ = false;
                    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                    g_led_com.Show();
                    g_control_comp.is_fast_runing_ = true;
                    machine.pushEvent(static_cast<int>(StateType::U_BALANCE));
                });
            });
        }
        else
        {
            time_calibration_.reset();
        }
    }

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "自动校准");
            OLED->DrawChineseInCenter(25, "边");
            OLED->DrawUTF8(
                5, 45, TO_STRING("A: %.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                5,
                58,
                TO_STRING("SA:%.1f", Param::GetInstance()->GetUAxisParam().angle_offset).c_str());

            if (is_runing_)
            {
                OLED->DrawChineseInCenter(75, "运行中...");
            }
            else
            {
                OLED->DrawChineseInCenter(75, "就绪");
            }
        });
    });
}

void UAutoCalibrationState::InExit(StateMachine &machine)
{
    g_control_comp.is_fast_runing_ = true;
    g_control_comp.StopUAutoCalibration();
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void PAutoCalibrationState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::U_JUMP));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this, &machine]() {
        is_runing_ = !is_runing_;
        if (is_runing_)
        {
            g_control_comp.is_fast_runing_ = false;
            g_control_comp.contorl_mode_   = POINT_BALANCE_MODE;
            is_start_calibrationstate_     = true;
        }
        else
        {
            g_control_comp.StopPAutoCalibration();
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
            g_led_com.Show();
            g_control_comp.is_fast_runing_ = false;
            g_control_comp.contorl_mode_   = IDLE_MODE;
        }
    });

    if (is_runing_)
    {
        g_led_com.Control(
            []() {
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR1_LED, LedCom::Color(120, 120, 120));
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR2_LED, LedCom::Color(120, 120, 120));
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR3_LED, LedCom::Color(120, 120, 120));
                g_led_com.Show();
            },
            []() {
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            },
            100,
            1,
            500);
        if (g_control_comp.p_balance_control_.IsBalance() && is_start_calibrationstate_)
        {
            time_calibration_.Task(1, [this, &machine]() {
                is_start_calibrationstate_ = false;
                g_control_comp.StartPAutoCalibration([this, &machine]() {
                    is_runing_ = false;
                    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
                    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
                    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                    g_led_com.Show();
                    g_control_comp.is_fast_runing_ = true;
                    machine.pushEvent(static_cast<int>(StateType::P_BALANCE));
                });
            });
        }
        else
        {
            time_calibration_.reset();
        }
    }

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "自动校准");
            OLED->DrawChineseInCenter(22, "点");
            U8G2->setFont(u8g2_font_6x10_tf);

            OLED->DrawUTF8(
                5, 35, TO_STRING("XA:%.1f", g_imu_comp_.GetImuData().angle.pitch).c_str());
            OLED->DrawUTF8(
                5, 45, TO_STRING("YA:%.1f", g_imu_comp_.GetImuData().angle.roll).c_str());
            OLED->DrawUTF8(
                5,
                55,
                TO_STRING("SX:%.1f", Param::GetInstance()->GetPAxisParam_t().x.angle_offset)
                    .c_str());
            OLED->DrawUTF8(
                5,
                65,
                TO_STRING("SY:%.1f", Param::GetInstance()->GetPAxisParam_t().y.angle_offset)
                    .c_str());

            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            if (is_runing_)
            {
                OLED->DrawChineseInCenter(80, "运行中...");
            }
            else
            {
                OLED->DrawChineseInCenter(80, "就绪");
            }
        });
    });
}

void PAutoCalibrationState::InExit(StateMachine &machine)
{
    g_control_comp.StopUAutoCalibration();
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void UJumpState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::P_JUMP));
    });

    g_key_enter_quit.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() {
        is_runing_ = !is_runing_;
        if (!is_runing_)
        {
            g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
            g_led_com.Show();
        }
    });

    if (is_runing_)
    {
        g_led_com.Control(
            []() {
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR3_LED, LedCom::Color(120, 120, 120));
                g_led_com.Show();
            },
            []() {
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            },
            100,
            1,
            (count_ + 1) * 100);
    }

    if (is_runing_)
    {
        calibration_countdown_.Task(1, [this, &machine]() {
            --count_;
            if (count_ == 0)
            {
                is_runing_ = false;
                count_     = 3;
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
                sub_status_                  = UJumpSubStates::JUMP_U;
                g_control_comp.contorl_mode_ = RunMode_e::U_JUMP_MODE;
            }
        });
    }
    else
    {
        calibration_countdown_.reset();
    }

    if (sub_status_ == UJumpSubStates::JUMP_U)
    {
        motor3_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch3);
        g_led_com.Show();

        if (g_control_comp.contorl_mode_ == RunMode_e::IDLE_MODE)
        {
            sub_status_ = UJumpSubStates::IDLE;
            is_runing_  = false;
        }
        if (g_control_comp.contorl_mode_ == RunMode_e::U_BALANCE_MODE)
        {
            machine.pushEvent(static_cast<int>(StateType::U_BALANCE));
        }
    }

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "起跳");
            OLED->DrawChineseInCenter(25, "边");
            OLED->DrawUTF8(
                4,
                45,
                TO_STRING("s:%0.1f", Param::GetInstance()->GetJumpParam().set_ch3_speed).c_str());
            if (is_runing_)
            {
                OLED->DrawUTF8(15, 70, TO_STRING(" %d", count_).c_str());
            }
            else
            {
                OLED->DrawChineseInCenter(70, "就绪");
            }
        });
    });
}

void UJumpState::InExit(StateMachine &machine)
{
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void PJumpState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::IDLE));
    });

    g_key_enter_quit.Task(
        KeyAction_e::eKEY_ACTION_SHORT_PRESS, [this]() { is_runing_ = !is_runing_; });

    if (is_runing_)
    {
        calibration_countdown_.Task(1, [this]() {
            --count_;
            if (count_ == 0)
            {
                is_runing_  = false;
                count_      = 3;
                sub_status_ = PJumpSubStates::JUMP_U;
                p_jump_timer_.reset();
                g_control_comp.contorl_mode_ = RunMode_e::U_JUMP_MODE;
            }
        });
    }
    else
    {
        calibration_countdown_.reset();
    }

    if (is_runing_ && sub_status_ == PJumpSubStates::IDLE)
    {
        g_led_com.Control(
            []() {
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR1_LED, LedCom::Color(120, 120, 120));
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR2_LED, LedCom::Color(120, 120, 120));
                g_led_com.AllTrueOn(LedCom::LED_TYPE::MOTOR3_LED, LedCom::Color(120, 120, 120));
                g_led_com.Show();
            },
            []() {
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
                g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
                g_led_com.Show();
            },
            100,
            1,
            (count_ + 1) * 100);
    }

    if (sub_status_ == PJumpSubStates::JUMP_U)
    {
        motor1_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch1);
        motor2_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch2);
        motor3_led_control_.RuningControl(g_can.GetMotorSpeed_t().ch3);
        g_led_com.Show();

        if (g_control_comp.contorl_mode_ == RunMode_e::IDLE_MODE)
        {
            sub_status_ = PJumpSubStates::IDLE;
            is_runing_  = false;
        }
        if (g_control_comp.contorl_mode_ == RunMode_e::U_BALANCE_MODE)
        {
            if ((fabs(g_can.GetMotorSpeed_t().ch3) < 30 && p_jump_timer_.GetTimeS() >= 2.0f) ||
                p_jump_timer_.GetTimeS() > 5.0f)
            {
                g_control_comp.contorl_mode_ = RunMode_e::POINT_JUMP_MODE;
            }
        }
        if (g_control_comp.contorl_mode_ == RunMode_e::POINT_BALANCE_MODE)
        {
            machine.pushEvent(static_cast<int>(StateType::P_BALANCE));
        }
    }

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "起跳");
            OLED->DrawChineseInCenter(22, "点");
            OLED->DrawUTF8(
                4,
                35,
                TO_STRING("s1:%0.1f", Param::GetInstance()->GetJumpParam().set_ch1_speed).c_str());
            OLED->DrawUTF8(
                4,
                47,
                TO_STRING("s2:%0.1f", Param::GetInstance()->GetJumpParam().set_ch2_speed).c_str());
            OLED->DrawUTF8(
                4,
                59,
                TO_STRING("s3:%0.1f", Param::GetInstance()->GetJumpParam().set_ch3_speed).c_str());
            if (is_runing_)
            {
                OLED->DrawUTF8(15, 73, TO_STRING(" %d", count_).c_str());
            }
            else
            {
                OLED->DrawChineseInCenter(73, "就绪");
            }
        });
    });
}

void PJumpState::InExit(StateMachine &machine)
{
    sub_status_ = PJumpSubStates::IDLE;
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR1_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR2_LED);
    g_led_com.AllTrueOff(LedCom::LED_TYPE::MOTOR3_LED);
    g_led_com.Show();
}

void ErrorState::Enter(StateMachine &machine)
{
    g_led_com.SetPixelColor(LedCom::INIT_LED, 0, LedCom::Color(100, 0, 0));
    g_led_com.Show();
}

void ErrorState::Execute(StateMachine &machine)
{
    g_key_mode.Task(KeyAction_e::eKEY_ACTION_SHORT_PRESS, [&machine]() {
        machine.pushEvent(static_cast<int>(StateType::IDLE));
    });

    g_control_comp.contorl_mode_ = IDLE_MODE;

    time_.Task(0.1f, [this]() {
        OLED_SEND([this]() {
            U8G2->setFont(u8g2_font_wqy12_t_gb2312);
            OLED->DrawChineseInCenter(6, "错误");
            U8G2->setFont(u8g2_font_6x10_tf);
            int find_bit = 0;
            int id       = 0;
            for (int i = 0; i < g_error_id.Len(); ++i)
            {
                if (i > g_error_msg.size())
                    break;
                if (g_error_id.GetBit(i))
                {
                    if (id >= 7)
                        break;
                    OLED->DrawUTF8(0, 17 + id * 10, g_error_msg[i].c_str());
                    ++id;
                }
            }
        });
    });
}

void ErrorState::InExit(StateMachine &machine)
{
    g_error_id.SetValue(0);
    g_led_com.SetPixelColor(LedCom::INIT_LED, 0, LedCom::Color(0, 100, 0));
    g_led_com.Show();
}
}  // namespace State
}  // namespace CubliMini
