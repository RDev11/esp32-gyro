#include "wifi.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
//#include "lwip/sys.h"
#include "lwip/sockets.h"
//#include "lwip/netdb.h"
//
const char *TAG = "UDP_CLIENT";


void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                        int32_t event_id, void* event_data);
void wifi_discover_server_task(void *pvParameters);
void wifi_send_hello_task(void *pvParameters);
void wifi_recv_commands_task(void *pvParameters);


void Wifi::init_sta()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_got_ip));

    wifi_config_t wifi_config = {};
    wifi_config.sta = wifi_sta_config_t{};
    memcpy( wifi_config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy( wifi_config.sta.password, WIFI_PASS, sizeof(WIFI_PASS));
    //wifi_config.sta.ssid = WIFI_SSID;
    //wifi_config.sta.password = WIFI_PASS;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA/*ESP_IF_WIFI_STA*/, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            ESP_LOGI(TAG, "Connect to the AP fail");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xTaskCreate(wifi_discover_server_task, "discover_server", 4096, NULL, 5, NULL);
    }
}

void wifi_discover_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting server discovery...");
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    // Разрешить broadcast
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // Настройка адреса для broadcast
    sockaddr_in broadcast_addr{};
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(BROADCAST_PORT);
    broadcast_addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);

    // Настройка таймаута
    timeval tv = {
        .tv_sec = 2,
        .tv_usec = 0
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Отправка discovery запроса
    sendto(sock, DISCOVERY_MSG, strlen(DISCOVERY_MSG), 0,
          (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

    ESP_LOGI(TAG, "Discovery message sent, waiting for response...");

    // Ожидание ответа
    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                      (struct sockaddr *)&source_addr, &socklen);

    if (len > 0) {
        rx_buffer[len] = '\0';
        if (strstr(rx_buffer, RESPONSE_MSG)) {
            inet_ntoa_r(source_addr.sin_addr, server_ip, sizeof(server_ip));
            ESP_LOGI(TAG, "Server found at %s", server_ip);
            close(sock);
            wifi_bConnected = true;
            xTaskCreate(wifi_send_hello_task, "send_hello", 4096, NULL, 5, NULL);
            xTaskCreate(wifi_recv_commands_task, "discover_server", 4096, NULL, 5, NULL);
            vTaskDelete(NULL);
        }
    }

    ESP_LOGW(TAG, "Server not found, retrying...");
    close(sock);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    xTaskCreate(wifi_discover_server_task, "discover_server", 4096, NULL, 5, NULL);
    vTaskDelete(NULL);
}

void wifi_send_hello_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting hello world sender to %s:%d", server_ip, SERVER_PORT);
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(server_ip);

    while (1) {
        int err = sendto(sock, "hello world", strlen("hello world"), 0,
                        (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error sending message: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Message sent to server");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    
    close(sock);
    vTaskDelete(NULL);
}

//чтение 
void wifi_recv_commands_task(void *pvParameters)
{
    char rx_buffer[MAX_CMD_LEN];
    struct sockaddr_in server_addr, client_addr;
    socklen_t socklen = sizeof(client_addr);

    // Создаем UDP сокет
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    // Настраиваем сервер
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(CLIENT_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    // Привязываем сокет
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr))) {
        ESP_LOGE(TAG, "Failed to bind socket: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "UDP server started on port %d", CLIENT_PORT);

    while (1) {
        // Получаем данные
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                        (struct sockaddr *)&client_addr, &socklen);

        if (len > 0) {
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "Received %d bytes from %s: %s", len, 
                    inet_ntoa(client_addr.sin_addr), rx_buffer);
            
            // Обрабатываем команду
            /*
            handle_command(rx_buffer);
            
            // Отправляем подтверждение
            char reply[] = "CMD_RECEIVED";
            sendto(sock, reply, strlen(reply), 0, 
                 (struct sockaddr *)&client_addr, socklen);
            */
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}



bool Wifi::isConnected() {
    return wifi_bConnected;
}

void Wifi::sendData(char* _data, size_t _len)
{

}

void Wifi::setHandler(wifi_cmd_handler_t _handler) {

}