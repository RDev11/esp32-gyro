#pragma once
#include "esp_event.h"

// Настройки Wi-Fi
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASS      "YOUR_WIFI_PASSWORD"
#define MAX_RETRY      5

// Настройки сервера
#define BROADCAST_PORT 12345
#define BROADCAST_IP   "255.255.255.255"
#define SERVER_PORT    12345
#define CLIENT_PORT    12345
#define DISCOVERY_MSG  "DISCOVER_SERVER"
#define RESPONSE_MSG   "SERVER_RESPONSE"

#define MAX_CMD_LEN 256


static int retry_num = 0;
static char server_ip[16] = {0};

static bool wifi_bConnected = false;

using wifi_cmd_handler_t = void (*)(char*, size_t len);

#define gWifi() Wifi::Instance()

inline void wifi_cmd_handler(char* _data, size_t len) {

}

//init_sta - начинает бродлкастить всем DISCOVERY_MSG, пока сервер не ответит, тогда соединение считается установленным.
//принятые сообщенния вызывают _handler из setHandler
class Wifi {
public:
    static Wifi& Instance() {
        static Wifi inst{};
        inst.setHandler(&wifi_cmd_handler);
        return inst;
    }

    void init_sta();//Connect
    bool isConnected();
    void sendData(char* _data, size_t _len);
    void setHandler(wifi_cmd_handler_t _handler);
private:
    ~Wifi() = default;
    wifi_cmd_handler_t m_handler;
};

