#pragma once

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <WiFi.h>

#include <string>

#include "web/html.h"

namespace CubliMini {
namespace Web {

class WebComp
{
   public:
    WebComp(const std::string &ssid, const std::string &password, int port = 80)
        : ssid_(ssid), password_(password), server_(80)
    {}
    void Init();

    void Loop() { server_.handleClient(); }
    void SetParam();
    void SetCmd();
    void handleRoot();
    void handleGetParams();
    void handleSaveParams();
    void handleNotFound();
    wl_status_t Status() const 
    {
        return WiFi.status();
    }

   private:
    WebServer server_;
    std::string ssid_;
    std::string password_;
    bool wifi_is_connect = false;
};

extern WebComp *g_web_comp;

}  // namespace Web
}  // namespace CubliMini