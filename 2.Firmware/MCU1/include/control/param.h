#pragma once
#include "bsp/eeprom.h"
#include "config/config.h"
#include "control/control_base.h"

namespace CubliMini {
namespace Control {

#define FIRST_DOWNLOAD_ADDR 0
#define P_PARAM_ADDR        4
#define U_PARAM_ADDR        P_PARAM_ADDR + sizeof(PAxisParam_t)
#define JUMP_PARAM_ADDR     U_PARAM_ADDR + sizeof(AxisParam_t)
#define VBUS_PARAM_ADDR     JUMP_PARAM_ADDR + sizeof(JumpParam_t)

class Param
{
   public:
    static Param *GetInstance()
    {
        static Param *param = new Param;
        return param;
    }

    void SaveAllParam();
    void SavePBalanceParam() { SavePBalanceParam(p_axis_param_); }
    void SaveUBalanceParam() { SaveUBalanceParam(u_param_); }
    void SaveVbusParam() { SaveVbusParam(vbus_param_); }
    void SaveJumpParam() { SaveJumpParam(jump_param_); }

    bool Load();

    PAxisParam_t &GetPAxisParam_t() { return p_axis_param_; }
    AxisParam_t &GetUAxisParam() { return u_param_; }
    VbusParam_t &GetVbusParam() { return vbus_param_; }
    JumpParam_t &GetJumpParam() { return jump_param_; }

    void FirstWriteParamToEeprom();
    void LoadParam();

   private:
    bool SavePBalanceParam(const PAxisParam_t &_param);
    bool SaveUBalanceParam(const AxisParam_t &_param);
    bool SaveFirstDownload(uint32_t first);
    void ReadFirstDownload(uint32_t &first);
    void ReadPBalanceParam(PAxisParam_t &_param);
    void ReadUBalanceParam(AxisParam_t &_param);
    bool SaveVbusParam(VbusParam_t &_param);
    void ReadVbusParam(VbusParam_t &_param);
    bool SaveJumpParam(JumpParam_t &_param);
    void ReadJumpParam(JumpParam_t &_param);

   private:
    PAxisParam_t p_axis_param_;
    AxisParam_t u_param_;
    JumpParam_t jump_param_;
    VbusParam_t vbus_param_;
};

}  // namespace Control
}  // namespace CubliMini