#include "state/state_base.h"

#include "state/state_factory.h"
#include "state/sub_state.h"

namespace CubliMini {
namespace State {
StateMachine::StateMachine()
{
    StateFactory::instance().registerState<InitState>(StateType::INIT);
    StateFactory::instance().registerState<IdleState>(StateType::IDLE);
    StateFactory::instance().registerState<UBalanceState>(StateType::U_BALANCE);
    StateFactory::instance().registerState<PBalanceState>(StateType::P_BALANCE);
    StateFactory::instance().registerState<MotorTestState>(StateType::MOTOR_TEST);
    StateFactory::instance().registerState<UCalibrationState>(StateType::U_MANUAL_CALIBRATION);
    StateFactory::instance().registerState<PCalibrationState>(StateType::P_MANUAL_CALIBRATION);
    StateFactory::instance().registerState<UAutoCalibrationState>(StateType::U_AUTO_CALIBRATION);
    StateFactory::instance().registerState<PAutoCalibrationState>(StateType::P_AUTO_CALIBRATION);
    StateFactory::instance().registerState<UJumpState>(StateType::U_JUMP);
    StateFactory::instance().registerState<PJumpState>(StateType::P_JUMP);
    StateFactory::instance().registerState<ErrorState>(StateType::ERROR);
}

void StateMachine::Init() { changeState(new InitState); }

void StateMachine::update()
{
    while (!eventQueue.empty())
    {
        StateType event;
        {
            MutexGuard lock(mutex_);
            event = static_cast<StateType>(eventQueue.front());
            eventQueue.pop();
        }

        if (event == getCurrentStateType())
            continue;

        if (StateBase *newState = StateFactory::instance().create(event))
        {
            safeDeleteState(currentState);
            currentState   = newState;
            stateStartTime = millis();
            currentState->Enter(*this);
        }
    }

    if (currentState)
    {
        currentState->Execute(*this);
    }
}

void StateMachine::changeState(StateBase *newState)
{
    if (!newState)
        return;

    // 双重状态检查
    if (newState->GetType() == getCurrentStateType())
    {
        delete newState;
        return;
    }

    safeDeleteState(currentState);
    currentState   = newState;
    stateStartTime = millis();
    if (currentState)
    {
        currentState->Enter(*this);
    }
}

void StateMachine::pushEvent(int event)
{
    MutexGuard lock(mutex_);
    if (eventQueue.size() < 5)
        eventQueue.push(event);
}

unsigned long StateMachine::getStateDuration() const { return millis() - stateStartTime; }

StateType StateMachine::getCurrentStateType() const
{
    return currentState ? currentState->GetType() : StateType::None;
}

StateMachine::~StateMachine() { safeDeleteState(currentState); }

}  // namespace State
}  // namespace CubliMini