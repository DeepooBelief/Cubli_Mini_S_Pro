#pragma once

#include <queue>

#include "state/state_base.h"
#include "state/state_type.h"
#include "comm/lock.h"

namespace CubliMini {
namespace State {
class StateMachine;
using namespace Comm;

class StateBase
{
   public:
    virtual StateType GetType() const           = 0;
    virtual void Enter(StateMachine &machine)   = 0;
    virtual void Execute(StateMachine &machine) = 0;
    virtual void Exit(StateMachine &machine)    = 0;
    virtual ~StateBase()                        = default;
};

class StateMachine
{
   private:
    StateBase *currentState = nullptr;
    std::queue<int> eventQueue;
    unsigned long stateStartTime;

    void safeDeleteState(StateBase *state)
    {
        if (state)
        {
            state->Exit(*this);
            delete state;
        }
    }

   public:
    void Init();
    StateMachine();
    void update();
    void changeState(StateBase *newState);
    void pushEvent(int event);
    unsigned long getStateDuration() const;
    StateType getCurrentStateType() const;
    ~StateMachine();

    bool is_remote_control = false;
    Mutex mutex_;
};
}  // namespace State
}  // namespace CubliMini