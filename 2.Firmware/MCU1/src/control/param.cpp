#include "control/param.h"

#include "bsp/vbus.h"

namespace CubliMini {
namespace Control {
using namespace Bsp;

bool Param::Load()
{
    EepromInit();
    uint32_t flag = 0xffffffff;
    ReadFirstDownload(flag);
    if (flag != 0)
    {
        flag = 0;
        SaveFirstDownload(flag);
        FirstWriteParamToEeprom();
        printf("first write\r\n");
    }
    LoadParam();
    return 0;
}

void Param::LoadParam()
{
    ReadPBalanceParam(p_axis_param_);
    ReadUBalanceParam(u_param_);
    ReadVbusParam(vbus_param_);
    ReadJumpParam(jump_param_);
}
void Param::FirstWriteParamToEeprom()
{
    PAxisParam_t p_axis_param {
        {P_BALANCE_X_P, P_BALANCE_X_V, P_BALANCE_X_S, P_BALANCE_X_A},
        {P_BALANCE_Y_P, P_BALANCE_Y_V, P_BALANCE_Y_S, P_BALANCE_Y_A},
        {P_BALANCE_Z_P, P_BALANCE_Z_V, P_BALANCE_Z_S, P_BALANCE_Z_A}
    };

    AxisParam_t u_param {U_BALANCE_CH3_P, U_BALANCE_CH3_V, U_BALANCE_CH3_S, U_BALANCE_CH3_A};
    SavePBalanceParam(p_axis_param);
    SaveUBalanceParam(u_param);

    vbus_param_.measured_v         = g_vbus.VbusCalibration(VBUS_CALIBRATION_V);
    vbus_param_.ntc_protected_temp = NTC_PROTECTED_TEMP;
    vbus_param_.vbus_protected_v   = VBUS_PROTECTED_V;
    SaveVbusParam(vbus_param_);

    JumpParam_t jump_param =
        JumpParam_t {JUNP_SET_CH1_SPEED, JUNP_SET_CH2_SPEED, JUNP_SET_CH3_SPEED};
    SaveJumpParam(jump_param);
}

void Param::SaveAllParam()
{
    SavePBalanceParam(p_axis_param_);
    SaveUBalanceParam(u_param_);
    SaveVbusParam(vbus_param_);
    SaveJumpParam(jump_param_);
}

bool Param::SaveFirstDownload(uint32_t first)
{
    return Write(FIRST_DOWNLOAD_ADDR, (uint8_t *)&first, sizeof(first));
}

void Param::ReadFirstDownload(uint32_t &first)
{
    Read(FIRST_DOWNLOAD_ADDR, (uint8_t *)&first, sizeof(first));
}

bool Param::SavePBalanceParam(const PAxisParam_t &_param)
{
    return Write(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void Param::ReadPBalanceParam(PAxisParam_t &_param)
{
    Read(P_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool Param::SaveUBalanceParam(const AxisParam_t &_param)
{
    return Write(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void Param::ReadUBalanceParam(AxisParam_t &_param)
{
    Read(U_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool Param::SaveVbusParam(VbusParam_t &_param)
{
    return Write(VBUS_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void Param::ReadVbusParam(VbusParam_t &_param)
{
    Read(VBUS_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

bool Param::SaveJumpParam(JumpParam_t &_param)
{
    return Write(JUMP_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

void Param::ReadJumpParam(JumpParam_t &_param)
{
    Read(JUMP_PARAM_ADDR, (uint8_t *)&_param, sizeof(_param));
}

}  // namespace Control
}  // namespace CubliMini