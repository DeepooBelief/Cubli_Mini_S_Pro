#pragma once
#include <Arduino.h>

#include "comm/time.h"

using namespace CubliMini::Comm;
namespace CubliMini {
namespace Bsp {

#define KEY_UP   1
#define KEY_DOWM 0

enum KeyAction_e
{
    eKEY_ACTION_NO_PRESS    = 0x00,
    eKEY_ACTION_SHORT_PRESS = 0x01,
    eKEY_ACTION_LONG_PRESS  = 0x02,
};

enum KeyPressStatus_e
{
    eKEY_STATUS_NOMAL         = 0,
    eKEY_STATUS_UP            = 1,
    eKEY_STATUS_DOWM          = 2,
    eKEY_STATUS_IS_STILL_DOWM = 3,
};

class KeyDriver
{
   public:
    KeyDriver(int _key) : key_pin_(_key)
    {
        key_press_status_        = eKEY_STATUS_NOMAL;
        key_is_press_start_time_ = 0;
        pinMode(key_pin_, INPUT_PULLUP);
    }

    KeyDriver()
    {
        key_press_status_        = eKEY_STATUS_NOMAL;
        key_is_press_start_time_ = 0;
    }
    void Init(int _key);
    KeyAction_e GetKeyAction();
    void Task(KeyAction_e action, std::function<void()> task);
    int ReadRawValue();

   private:
    int key_pin_;
    Time key_time_;
    KeyPressStatus_e key_press_status_;
    unsigned long key_is_press_start_time_;
};


extern KeyDriver g_key_mode;
extern KeyDriver g_key_enter_quit;

}  // namespace Bsp
}  // namespace CubliMini