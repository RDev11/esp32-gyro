#include "bluetooth/bluetooth_serial.h"
#include "esp_log.h"

extern void bt_test();

namespace bluetooth{
    void bluetooth_start_accept(){
        bt_test();
    }
    void onConnect(){

    }
    void onDisconnect(){

    }
    void onReceiveData(const esp_spp_cb_param_t::spp_data_ind_evt_param& data_ind)//esp_spp_status_t status, uint32_t handle, const uint8_t *data, uint16_t len
    {
        esp_log_buffer_hex("btr", data_ind.data, data_ind.len);
        
        esp_spp_write( data_ind.handle, sizeof("Response")-1, (uint8_t *)"Response") ;
    }
    void sendData(){
        //esp_spp_write( param->data_ind.handle, sizeof("Response")-1, (uint8_t *)"Response") ;
    }



};