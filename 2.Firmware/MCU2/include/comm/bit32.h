#pragma once
#include <cstdint>
namespace CubliMini {
namespace Comm {

class Bit32 
{
public:
    Bit32() : value_(0) {}

    void SetBit(uint8_t index) {
        if (index >= 32) return;
        value_ |= (1U << index);
    }

    void ResetBit(uint8_t index) {
        if (index >= 32) return;
        value_ &= ~(1U << index);
    }

    bool GetBit(uint8_t index) const {
        return (value_ >> index) & 1U;
    }

    void ToggleBit(uint8_t index) {
        if (index >= 32) return;
        value_ ^= (1U << index);
    }

    uint32_t GetValue() const { return value_; }

    void SetValue(uint32_t value) { value_ = value; }

    uint8_t Len() const { return 32; }

private:
    uint32_t value_;
};
}
}