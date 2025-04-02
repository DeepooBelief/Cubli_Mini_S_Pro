#include "web/web_comp.h"

#include "config/config.h"
#include "control/control.h"
#include "control/param.h"
#include "imu/imu_comp.h"
#include "state/sub_state.h"
namespace CubliMini {
namespace Web {

using namespace Config;
using namespace Control;
using namespace State;
using namespace Imu;

WebComp *g_web_comp;
void WebComp::Init()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid_.c_str(), password_.c_str());
    // 信号增强
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    printf("\nAP IP: %s\n", WiFi.softAPIP().toString());
    printf("SSID: %s\n", ssid_.c_str());
    printf("Password: %s\n", password_.c_str());

    server_.on("/get_params", HTTP_GET, [this]() { handleGetParams(); });
    server_.on("/save_params", HTTP_POST, [this]() { handleSaveParams(); });
    server_.on("/setParam", HTTP_POST, [this]() { SetParam(); });
    server_.on("/setCmd", HTTP_POST, [this]() { SetCmd(); });
    server_.on("/", [this]() { handleRoot(); });
    server_.onNotFound([this]() { handleNotFound(); });
    server_.begin();
}

void WebComp::SetParam()
{
    if (server_.hasArg("param") && server_.hasArg("value"))
    {
        JumpParam_t &jump_param = Param::GetInstance()->GetJumpParam();
        PAxisParam_t &p_param   = Param::GetInstance()->GetPAxisParam_t();
        AxisParam_t &u_param    = Param::GetInstance()->GetUAxisParam();
        VbusParam_t &vbus_param = Param::GetInstance()->GetVbusParam();

        String param = server_.arg("param");
        float value  = server_.arg("value").toFloat();

        printf("debug rec msg: %s %f\n", param, value);

        // control
        if (param == "rotate_speed")
        {
            g_control_comp.yaw_control_ = value;
            printf("rotate_speed: %f\n", g_control_comp.yaw_control_);
        }
        // jump
        else if (param == "u_jump_speed")
        {
            jump_param.set_ch3_speed = value;
            printf("jump_param u_speed: %f\n", jump_param.set_ch3_speed);
        }
        else if (param == "p_jump_speed_ch1")
        {
            jump_param.set_ch1_speed = value;
            printf(
                "jump_param p_speed ch1: %0.2f\n",
                jump_param.set_ch1_speed);
        }
        else if (param == "p_jump_speed_ch2")
        {
            jump_param.set_ch2_speed = value;
            printf(
                "jump_param p_speed ch2: %0.2f\n",
                jump_param.set_ch2_speed);
        }

        // u blan
        else if (param == "u_ka")
        {
            u_param.angle_offset = value;
            printf("u_param u_ka: %0.2f\n", u_param.angle_offset);
        }
        else if (param == "u_kp")
        {
            u_param.kp = value;
            printf("u_param u_ka: %0.2f\n", u_param.kp);
        }
        else if (param == "u_kv")
        {
            u_param.kv = value;
            printf("u_param u_kv: %0.2f\n", u_param.kv);
        }
        else if (param == "u_ks")
        {
            u_param.ks = value;
            printf("u_param u_ks: %0.2f\n", u_param.ks);
        }

        // p blan
        else if (param == "p_xa")
        {
            p_param.x.angle_offset = value;
            printf("p_param p_xa: %0.2f\n", p_param.x.angle_offset);
        }
        else if (param == "p_ya")
        {
            p_param.y.angle_offset = value;
            printf("p_param p_ya: %0.2f\n", p_param.y.angle_offset);
        }
        else if (param == "p_xka")
        {  // 需要修改
            p_param.x.kp = value;
            printf("p_param p_xkp: %0.2f\n", p_param.x.kp);
        }
        else if (param == "p_xkv")
        {
            p_param.x.kv = value;
            printf("p_param p_xkv: %0.2f\n", p_param.x.kv);
        }
        else if (param == "p_xks")
        {
            p_param.x.ks = value;
            printf("p_param p_xks: %0.2f\n", p_param.x.ks);
        }
        else if (param == "p_ykp")
        {
            p_param.y.kp = value;
            printf("p_param p_ykp: %0.2f\n", p_param.y.kp);
        }
        else if (param == "p_ykv")
        {
            p_param.y.kv = value;
            printf("p_param p_ykv: %0.2f\n", p_param.y.kv);
        }
        else if (param == "p_yks")
        {
            p_param.y.ks = value;
            printf("p_param p_yks: %0.2f\n", p_param.y.ks);
        }
        else if (param == "p_zkp")
        {
            p_param.z.kp = value;
            printf("p_param p_zkp: %0.2f\n", p_param.z.kp);
        }
        else if (param == "p_zkv")
        {
            p_param.z.kv = value;
            printf("p_param p_zkv: %0.2f\n", p_param.z.kv);
        }
        else if (param == "p_zks")
        {
            p_param.z.ks = value;
            printf("p_param p_zks: %0.2f\n", p_param.z.ks);
        }

        // vbus
        else if (param == "low_power")
        {
            vbus_param.vbus_protected_v = value;
            printf("vbus_param low_power: %0.2f\n", vbus_param.vbus_protected_v);
        }
        else if (param == "opt")
        {
            vbus_param.ntc_protected_temp = value;
            printf("vbus_param opt: %0.2f\n", vbus_param.ntc_protected_temp);
        }

        server_.send(200, "text/plain", "OK");
    }
    else
    {
        server_.send(400, "text/plain", "Missing parameters");
    }
}

void WebComp::SetCmd()
{
    if (server_.hasArg("param"))
    {
        String param = server_.arg("param");
        // printf("debug rec cmd: %s\n", param);

        // control
        if (param == "idle")
        {
            g_control_comp.contorl_mode_ = IDLE_MODE;
            g_machine->pushEvent(static_cast<int>(StateType::IDLE));
        }
        else if (param == "u_balance")
        {
            g_control_comp.contorl_mode_ = U_BALANCE_MODE;
            g_machine->pushEvent(static_cast<int>(StateType::U_BALANCE));
        }
        else if (param == "point_balance")
        {
            g_control_comp.contorl_mode_ = POINT_BALANCE_MODE;
            g_machine->pushEvent(static_cast<int>(StateType::P_BALANCE));
        }
        else if (param == "edge_jump")
        {
            g_control_comp.contorl_mode_ = U_JUMP_MODE;
            g_machine->is_remote_control = true;
            g_machine->pushEvent(static_cast<int>(StateType::U_BALANCE));
        }
        else if (param == "point_jump")
        {
            g_control_comp.contorl_mode_ = POINT_JUMP_MODE;
            g_machine->pushEvent(static_cast<int>(StateType::P_BALANCE));
        }

        // calibrate
        if (param == "calibrate_edge")
        {
            Param::GetInstance()->GetUAxisParam().angle_offset =
                g_imu_comp_.GetImuData().angle.pitch;
            Param::GetInstance()->SaveUBalanceParam();
        }
        else if (param == "calibrate_point")
        {
            Param::GetInstance()->GetPAxisParam_t().x.angle_offset =
                g_imu_comp_.GetImuData().angle.pitch;
            Param::GetInstance()->GetPAxisParam_t().y.angle_offset =
                g_imu_comp_.GetImuData().angle.roll;
            Param::GetInstance()->SavePBalanceParam();
        }
        else if (param == "autoCalibrate_edge")
        {
            g_control_comp.StartUAutoCalibration();
        }
        else if (param == "autoCalibrate_point")
        {
            g_control_comp.StartPAutoCalibration();
        }
        server_.send(200, "text/plain", "OK");
    }
    else
    {
        server_.send(400, "text/plain", "Missing parameters");
    }
}

void WebComp::handleRoot() { server_.send(200, "text/html; charset=utf-8", html); }

void WebComp::handleGetParams()
{
    if (server_.method() != HTTP_GET)
    {
        server_.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    if (!server_.hasArg("type"))
    {
        server_.send(400, "text/plain", "Missing type parameter");
        return;
    }

    String type = server_.arg("type");
    DynamicJsonDocument doc(512);

    JumpParam_t &jump_param = Param::GetInstance()->GetJumpParam();
    PAxisParam_t &p_param   = Param::GetInstance()->GetPAxisParam_t();
    AxisParam_t &u_param    = Param::GetInstance()->GetUAxisParam();
    VbusParam_t &vbus_param = Param::GetInstance()->GetVbusParam();

    if (type == "u_balance")
    {
        doc["ka"] = u_param.angle_offset;
        doc["kp"] = u_param.kp;
        doc["kv"] = u_param.kv;
        doc["ks"] = u_param.ks;
        Serial.printf(
            "GetParams u_param\n(angle, kp, kv, ks)\n(%0.1f, %0.1f, %0.1f, %0.1f)\n",
            u_param.angle_offset,
            u_param.kp,
            u_param.kv,
            u_param.ks);
    }
    else if (type == "jump")
    {
        doc["u_jump_speed"] = jump_param.set_ch3_speed;
        doc["p_jump_speed_ch1"] = jump_param.set_ch1_speed;
        doc["p_jump_speed_ch2"] = jump_param.set_ch2_speed;
        printf(
            "GetParams jump_param:\n(u, ch1, ch2)\n(%0.1f, %0.1f, %0.1f)\n",
            jump_param.set_ch3_speed,
            jump_param.set_ch1_speed);
    }
    else if (type == "p_balance")
    {
        doc["xa"] = p_param.x.angle_offset;
        doc["ya"] = p_param.y.angle_offset;

        doc["xkp"] = p_param.x.kp;
        doc["xkv"] = p_param.x.kv;
        doc["xks"] = p_param.x.ks;

        doc["ykp"] = p_param.y.kp;
        doc["ykv"] = p_param.y.kv;
        doc["yks"] = p_param.y.ks;

        doc["zkp"] = p_param.z.kp;
        doc["zkv"] = p_param.z.kv;
        doc["zks"] = p_param.z.ks;
        printf(
            "GetParams p_param\n(xa: %0.2f ya: %0.2f)\n(xkp: %0.2f xkv: %0.2f xks: %0.2f)\n(ykp: "
            "%0.2f ykv: %0.2f yks: %0.2f)\n(zkp: %0.2f zkv: %0.2f zks: %0.2f)\n",
            p_param.x.angle_offset,
            p_param.y.angle_offset,
            p_param.x.kp,
            p_param.x.kv,
            p_param.x.ks,
            p_param.y.kp,
            p_param.y.kv,
            p_param.y.ks,
            p_param.z.kp,
            p_param.z.kv,
            p_param.z.ks);
    }
    else if (type == "protected")
    {
        doc["low_power"] = vbus_param.vbus_protected_v;
        doc["opt"]       = vbus_param.ntc_protected_temp;
        Serial.printf(
            "GetParams vbus_param\n(low_power, opt)\n(%0.1f, %0.1f)\n",
            vbus_param.vbus_protected_v,
            vbus_param.ntc_protected_temp);
    }
    else
    {
        server_.send(400, "text/plain", "Invalid type");
        return;
    }

    String json;
    serializeJson(doc, json);
    server_.send(200, "application/json", json);
}

void WebComp::handleSaveParams()
{
    if (server_.method() != HTTP_POST)
    {
        server_.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    String json = server_.arg("plain");
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, json);

    if (error)
    {
        server_.send(400, "text/plain", "Invalid JSON");
        return;
    }

    String type          = doc["type"];
    JsonObject paramsObj = doc["params"];

    JumpParam_t &jump_param = Param::GetInstance()->GetJumpParam();
    PAxisParam_t &p_param   = Param::GetInstance()->GetPAxisParam_t();
    AxisParam_t &u_param    = Param::GetInstance()->GetUAxisParam();
    VbusParam_t &vbus_param = Param::GetInstance()->GetVbusParam();

    if (type == "u_balance")
    {
        // u_param.angle_offset = paramsObj["ka"] | u_param.angle_offset;
        // u_param.kp = paramsObj["kp"] | u_param.kp;
        // u_param.kv = paramsObj["kv"] | u_param.kv;
        // u_param.ks = paramsObj["ks"] | u_param.ks;
        Param::GetInstance()->SaveUBalanceParam();
        printf(
            "SaveParams u_param\n(ka: %0.2f kp: %0.2f kv: %0.2f ks: %0.2f)\n",
            u_param.angle_offset,
            u_param.kp,
            u_param.kv,
            u_param.ks);
    }
    else if (type == "jump")
    {
        // jump_param.set_ch3_speed = paramsObj["u_jump_speed"] | jump_param.set_ch3_speed;
        // jump_param.set_ch1_speed = paramsObj["p_jump_speed"] | jump_param.set_ch1_speed;
        // jump_param.set_ch2_speed = jump_param.set_ch1_speed;
        Param::GetInstance()->SaveJumpParam();
        Serial.printf(
            "SaveParams jump_param\n(u_jump_speed: %0.1f p_jump_speed: %0.1f)\n(s1: %0.1f, s2: "
            "%0.1f, s3: %0.1f)\n",
            jump_param.set_ch1_speed,
            jump_param.set_ch3_speed,
            jump_param.set_ch1_speed,
            jump_param.set_ch2_speed,
            jump_param.set_ch3_speed);
    }
    else if (type == "p_balance")
    {
        // p_param.x.angle_offset = paramsObj["xa"] | p_param.x.angle_offset;
        // p_param.y.angle_offset = paramsObj["ya"] | p_param.y.angle_offset;
        // p_param.x.kp = paramsObj["xkp"] | p_param.x.kp;
        // p_param.x.kv = paramsObj["xkv"] | p_param.x.kv;
        // p_param.x.ks = paramsObj["xks"] | p_param.x.ks;
        // p_param.y.kp = paramsObj["ykp"] | p_param.y.kp;
        // p_param.y.kv = paramsObj["ykv"] | p_param.y.kv;
        // p_param.y.ks = paramsObj["yks"] | p_param.y.ks;
        // p_param.z.kp = paramsObj["zkp"] | p_param.z.kp;
        // p_param.z.kv = paramsObj["zkv"] | p_param.z.kv;
        // p_param.z.ks = paramsObj["zks"] | p_param.z.ks;
        Param::GetInstance()->SavePBalanceParam();

        printf(
            "SaveParams p_param\n(xa: %0.2f ya: %0.2f)\n(xkp: %0.2f xkv: %0.2f xks: %0.2f)\n(ykp: "
            "%0.2f ykv: %0.2f yks: %0.2f)\n(zkp: %0.2f zkv: %0.2f zks: %0.2f)\n",
            p_param.x.angle_offset,
            p_param.y.angle_offset,
            p_param.x.kp,
            p_param.x.kv,
            p_param.x.ks,
            p_param.y.kp,
            p_param.y.kv,
            p_param.y.ks,
            p_param.z.kp,
            p_param.z.kv,
            p_param.z.ks);
    }
    else if (type == "protected")
    {
        // vbus_param.vbus_protected_v = paramsObj["low_power"] | vbus_param.vbus_protected_v;
        // vbus_param.ntc_protected_temp = paramsObj["opt"] | vbus_param.ntc_protected_temp;
        printf(
            "SaveParams vbus_param\n(low_power: %0.1f opt: %0.1f)\n",
            vbus_param.vbus_protected_v,
            vbus_param.ntc_protected_temp);
        Param::GetInstance()->SaveVbusParam();
    }

    server_.send(200, "text/plain", "OK");
}

// 详细的404处理
void WebComp::handleNotFound()
{
    String message = "Path not found\n\n";
    message += "URI: " + server_.uri() + "\n";
    message += "Method: " + (String)(server_.method() == HTTP_GET ? "GET" : "POST") + "\n";
    message += "Arguments: " + String(server_.args()) + "\n";

    for (uint8_t i = 0; i < server_.args(); i++)
    {
        message += " " + server_.argName(i) + ": " + server_.arg(i) + "\n";
    }

    server_.send(404, "text/plain", message);
    Serial.println("Unhandled request:\n" + message);
}
}  // namespace Web
}  // namespace CubliMini