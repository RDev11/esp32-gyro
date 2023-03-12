#pragma once

#include "esp_spp_api.h"

namespace bluetooth{
    void bluetooth_start_accept();
    void onConnect();
    void onDisconnect();
    void onReceiveData(const esp_spp_cb_param_t::spp_data_ind_evt_param& data_ind);//esp_spp_status_t status, uint32_t handle, const uint8_t *data, uint16_t len
    void sendData();



};