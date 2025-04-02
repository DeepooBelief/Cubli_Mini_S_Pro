#include "bsp/eeprom.h"

void EepromInit() { EEPROM.begin(1024); }

void Read(uint32_t _addr, uint8_t *_data, uint32_t _len)
{
    if (_len > 1024)
    {
        return;
    }
    for (uint32_t i = 0; i < _len; ++i)
    {
        _data[i] = EEPROM.read(i + _addr);
    }
}

bool Write(uint32_t _addr, uint8_t *_data, uint32_t _len)
{
    if (_len > 1024)
    {
        return false;
    }
    for (uint32_t i = 0; i < _len; ++i)
    {
        EEPROM.write(_addr + i, _data[i]);
    }
    return EEPROM.commit();
}