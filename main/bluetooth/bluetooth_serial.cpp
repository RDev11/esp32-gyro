#include "stdafx.h"
#include "bluetooth/bluetooth_serial.h"
#include "esp_log.h"

#include "SPI.h"
extern void bt_test();
extern spi::Gyro dev2;

namespace bluetooth{
    static uint32_t HBtChannel_last;
    void bluetooth_start_accept(){
        bt_test();
    }
    void onConnect(const esp_spp_cb_param_t::spp_srv_open_evt_param& param){
        //param.status
        HBtChannel_last = param.handle;
    }
    void onDisconnect(const esp_spp_cb_param_t::spp_close_evt_param& param){
        HBtChannel_last = 0;
    }
    void onReceiveData(const esp_spp_cb_param_t::spp_data_ind_evt_param& param)//esp_spp_status_t status, uint32_t handle, const uint8_t *data, uint16_t len
    {
        esp_log_buffer_hex("btRead", param.data, param.len);
        if(param.len==4+4*4 && param.data[2]==0x01 && param.data[3]==0xFF){
            int P = *reinterpret_cast<int32_t*>(&param.data[4]);
            int I = *reinterpret_cast<int32_t*>(&param.data[8]);
            int D = *reinterpret_cast<int32_t*>(&param.data[12]);
            int Offset = *reinterpret_cast<int32_t*>(&param.data[16]);
            swapbytes((uint8_t*)&P, 4);
            swapbytes((uint8_t*)&I, 4);
            swapbytes((uint8_t*)&D, 4);
            swapbytes((uint8_t*)&Offset, 4);
            //I/MyLog: sendMessage: 00  12  01  ff  00  00  10  00  00  00  00  00  00  00  00  00  00  00  80  00 
            ESP_LOGE("bluetooth::onReceiveData", "Set PID 0x: P:%x I:%x, D:%x, Offest:%x", P, I, D, Offset );
            dev2.bal.P = P/4096.0f;
            dev2.bal.I = I/4096.0f;
            dev2.bal.D = D/4096.0f;
            dev2.bal.angle_offset = Offset/4096.0f;
            ESP_LOGE("bluetooth::onReceiveData", "Set PID dec: P:%d I:%d, D:%d, Offest:%d"
                , (int)dev2.bal.P, (int)dev2.bal.I, (int)dev2.bal.D, (int)dev2.bal.angle_offset );
            dev2.bal.Ei = 0;
            dev2.bal.steps = 0;
            //dev2.bal.K = 0;
        }
        //esp_spp_write( param.handle, sizeof("Response")-1, (uint8_t *)"Response") ;
    }
    void sendData(uint32_t HBtChannel, uint8_t* data, size_t size){
        if(HBtChannel==0){
            HBtChannel = HBtChannel_last;
        }
        if(HBtChannel==0){
            return;
        }
        data[0]=size>>8;
        data[1]=size&0xFF;
        esp_spp_write( HBtChannel, size, data) ;
    }



};