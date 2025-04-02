#pragma once

#include "bsp/bsp.h"
#include "comm/bit32.h"
#include "comm/time.h"
#include "control/control.h"
#include "control/motor_led_control.h"
#include "state/state_base.h"

using namespace CubliMini::Comm;
namespace CubliMini {
namespace State {
using namespace Control;
using namespace Bsp;

class SubState : public StateBase
{
   public:
    SubState() : count_(3), is_runing_(false) { time_.reset(); }
    virtual ~SubState() = default;
    virtual void Exit(StateMachine &machine) override
    {
        if (is_runing_)
            g_control_comp.contorl_mode_ = IDLE_MODE;
        is_runing_ = false;
        count_     = 3;
        InExit(machine);
    }
    virtual void InExit(StateMachine &machine) = 0;

   protected:
    uint8_t count_;
    bool is_runing_;
    CumulativeTime time_;
    CumulativeTime time_calibration_;
};

class InitState : public SubState
{
   public:
    StateType GetType() const override { return StateType::INIT; }
    void Enter(StateMachine &machine) override;
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override {}
};

class IdleState : public SubState
{
   public:
    StateType GetType() const override { return StateType::IDLE; }
    void Enter(StateMachine &machine) override;
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;
};

class UBalanceState : public SubState
{
   public:
    UBalanceState() : motor_led_control_(LedCom::LED_TYPE::MOTOR3_LED) {}
    StateType GetType() const override { return StateType::U_BALANCE; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;

   private:
    MotorLedControl motor_led_control_;
};

class PBalanceState : public SubState
{
   public:
    PBalanceState()
        : motor1_led_control_(LedCom::LED_TYPE::MOTOR1_LED),
          motor2_led_control_(LedCom::LED_TYPE::MOTOR2_LED),
          motor3_led_control_(LedCom::LED_TYPE::MOTOR3_LED)
    {}
    StateType GetType() const override { return StateType::P_BALANCE; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;

   private:
    MotorLedControl motor1_led_control_;
    MotorLedControl motor2_led_control_;
    MotorLedControl motor3_led_control_;
};

class MotorTestState : public SubState
{
   public:
    MotorTestState()
        : motor1_led_control_(LedCom::LED_TYPE::MOTOR1_LED),
          motor2_led_control_(LedCom::LED_TYPE::MOTOR2_LED),
          motor3_led_control_(LedCom::LED_TYPE::MOTOR3_LED)
    {}
    StateType GetType() const override { return StateType::MOTOR_TEST; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;

   private:
    MotorLedControl motor1_led_control_;
    MotorLedControl motor2_led_control_;
    MotorLedControl motor3_led_control_;
};

class UCalibrationState : public SubState
{
   public:
    StateType GetType() const override { return StateType::U_MANUAL_CALIBRATION; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;

   private:
    CumulativeTime calibration_countdown_;
};

class PCalibrationState : public SubState
{
   public:
    StateType GetType() const override { return StateType::P_MANUAL_CALIBRATION; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;
   private:
    CumulativeTime calibration_countdown_;
};

class UAutoCalibrationState : public SubState
{
   public:
    StateType GetType() const override { return StateType::U_AUTO_CALIBRATION; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;
   private:
    bool is_start_calibrationstate_ = false;
    CumulativeTime calibration_countdown_;
};

class PAutoCalibrationState : public SubState
{
   public:
    StateType GetType() const override { return StateType::P_AUTO_CALIBRATION; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;
private:
    bool is_start_calibrationstate_ = false;
    CumulativeTime calibration_countdown_;
};

class UJumpState : public SubState
{
   public:
    enum class UJumpSubStates
    {
        IDLE = 0,
        JUMP_U
    };
    UJumpState()
        : sub_status_(UJumpSubStates::IDLE), motor3_led_control_(LedCom::LED_TYPE::MOTOR3_LED)
    {}
    StateType GetType() const override { return StateType::U_JUMP; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;

   private:
    CumulativeTime calibration_countdown_;
    UJumpSubStates sub_status_;
    MotorLedControl motor3_led_control_;
};

class PJumpState : public SubState
{
   public:
    enum class PJumpSubStates
    {
        IDLE = 0,
        JUMP_U
    };

    PJumpState()
        : sub_status_(PJumpSubStates::IDLE),
          motor1_led_control_(LedCom::LED_TYPE::MOTOR1_LED),
          motor2_led_control_(LedCom::LED_TYPE::MOTOR2_LED),
          motor3_led_control_(LedCom::LED_TYPE::MOTOR3_LED)
    {}
    StateType GetType() const override { return StateType::P_JUMP; }
    void Enter(StateMachine &machine) override {}
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;

   private:
    CumulativeTime calibration_countdown_;
    CumulativeTime p_jump_timer_;
    PJumpSubStates sub_status_;
    MotorLedControl motor1_led_control_;
    MotorLedControl motor2_led_control_;
    MotorLedControl motor3_led_control_;
};

class ErrorState : public SubState
{
   public:
    enum class ERROR_MSG
    {
        NO_IMU = 0,
        NO_IMU_T,
        M1_OPT,
        M2_OPT,
        M3_OPT,
        P_OPT,
        M1_C_E,
        M2_C_E,
        M3_C_E,
        M1_E_E,
        M2_E_E,
        M3_E_E,
        NO_MCU2,
        LOW_P
    };

    StateType GetType() const override { return StateType::ERROR; }
    void Enter(StateMachine &machine) override;
    void Execute(StateMachine &machine) override;
    void InExit(StateMachine &machine) override;
};

extern Bit32 g_error_id;
extern StateMachine *g_machine;
}  // namespace State
}  // namespace CubliMini