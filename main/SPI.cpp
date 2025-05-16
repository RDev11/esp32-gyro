#include "stdafx.h"
#include "SPI.h"

//#define ILI9341
//#include "ili9341_d.h"
//#include "text2.h"

#include <cmath>
//#include "bluetooth/bluetooth_serial.h"
namespace spi{ //FUNCTIONS
esp_err_t default_init(spi_host_device_t spihost /*= HSPI_HOST*/)
{
    esp_err_t ret=ESP_OK;
    spi_bus_config_t cfg={
        .mosi_io_num=CONFIG_PIN_NUM_MOSI,
        .miso_io_num=CONFIG_PIN_NUM_MISO,
        .sclk_io_num=CONFIG_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0,
        .intr_flags = 0
    };
    ret = spi_bus_initialize(spihost, &cfg, 1);
    return ret;
}
}


namespace spi{ //spi::device 
    device::device(spi_host_device_t host, gpio_num_t cs_pin)
        : m_spi_cfg{}
        , m_spi_dev{}
        , m_host(host)
    {
        m_spi_cfg.spics_io_num = cs_pin;
    }

    void device::transfer(const uint8_t* data, size_t size)
    {
        spi_transaction_t t; 							
        memset(&t, 0, sizeof(t));						
        t.length = size*8;							
        t.tx_buffer = data;								
        spi_device_transmit(m_spi_dev, &t);        
    }

    void device::transfer(const uint16_t* data, size_t size)
    {
    // transfer((const uint8_t*)data, size);
        spi_transaction_t t; 							
        memset(&t, 0, sizeof(t));						
        t.length = size*8;							
        t.tx_buffer = (const uint8_t*)data;								
        spi_device_transmit(m_spi_dev, &t);     
    }
    /*
    template<typename... Args>
    void device::transfer(Args... args)
    {
        uint8_t buf[] = { ((uint8_t)args)... };
        transfer(buf, sizeof(buf));
    }*/



    /*
    void device::transfer( std::initializer_list<uint8_t> list)
    {
        std::array<uint8_t> data(list);
        data.
        uint8_t buf[] = { args... }; 				
        spi_transaction_t t; 							
        memset(&t, 0, sizeof(t));						
        t.length=sizeof(buf)*8;							
        t.tx_buffer=buf;								
        spi_device_transmit(m_spi_dev, &t);        
    }*/
}

namespace spi{ //spi::ili9341 


#define GYRO_READ 0x80
#define GYRO_WRITE 0x00
#define ADDR_MASK 0x7F

    void ReadRegisters(spi::device* dev, uint8_t addr, uint8_t *data, uint8_t count)
    {
        {
        spi_transaction_ext_t t;
        memset(&t, 0, sizeof(t));
        t.command_bits = 8;
        t.base.cmd = GYRO_READ | (addr & ADDR_MASK),
        t.base.length = count*8;
        t.base.rxlength = count*8;
        t.base.flags |= SPI_TRANS_VARIABLE_CMD;
        t.base.rx_buffer = data;
        spi_device_transmit(dev->m_spi_dev, reinterpret_cast<spi_transaction_t*>(&t));

        //char* str = new char[count*2+1];
        //char* str = new char[32];
        //char* str = (char*)malloc(count*2+1);
        char str[100];//какая-то дичь при выделении памяти в куче при работе блютус, хотя если выделять 1024+ то не проявляется
        //]char* str = (char*)heap_caps_malloc(count*2+1, MALLOC_CAP_DEFAULT);
        for(int i = 0; i<count; i++)
        {
            sprintf(str+i*3, "%.2x,",data[i]);
        }
//        ESP_LOGI(__FUNCTION__, " R[%.2x+%.2x]=%s", addr, count, str);
        //delete[] str;
        //free(str);
        //heap_caps_free(str);
        }
    }

    uint8_t ReadRegister(spi::device* dev, uint8_t addr)
    {
        {
        spi_transaction_ext_t t;
        memset(&t, 0, sizeof(t));
        t.command_bits = 8;
        t.base.cmd = GYRO_READ | (addr & ADDR_MASK),
        t.base.length = 8;
        t.base.rxlength = 8;
        t.base.flags |= SPI_TRANS_VARIABLE_CMD;
        t.base.flags |= SPI_TRANS_USE_RXDATA;
        //t.base.rx_buffer;
        spi_device_transmit(dev->m_spi_dev, reinterpret_cast<spi_transaction_t*>(&t));
        ESP_LOGI(__FUNCTION__, " R[0x%.2x]=0x%.2x", addr, t.base.rx_data[0]);
        return t.base.rx_data[0];
        }
    }

    void WriteRegisters(spi::device* dev, uint8_t addr, uint8_t *data, uint8_t count)
    {
        spi_transaction_ext_t t;
        memset(&t, 0, sizeof(t));
        t.command_bits = 8;
        t.base.cmd = GYRO_WRITE | (addr & ADDR_MASK),
        t.base.length = 8*count;
        t.base.flags |= SPI_TRANS_VARIABLE_CMD;
        t.base.tx_buffer = data;
        spi_device_transmit(dev->m_spi_dev, reinterpret_cast<spi_transaction_t*>(&t));
        
        char str[100];//какая-то дичь при выделении памяти в куче при работе блютус, хотя если выделять 1024+ то не проявляется
        for(int i = 0; i<count; i++)
        {
            sprintf(str+i*3, "%.2x,",data[i]);
        }
        ESP_LOGE(__FUNCTION__, " W[%.2x+%.2x]=%s", addr, count, str);
    }


    void WriteRegister(spi::device* dev, uint8_t addr,  uint8_t data)
    {
        spi_transaction_ext_t t;
        memset(&t, 0, sizeof(t));
        t.command_bits = 8;
        t.base.cmd = GYRO_WRITE | (addr & ADDR_MASK),
        t.base.length = 8;
        t.base.flags |= SPI_TRANS_VARIABLE_CMD;
        t.base.flags |= SPI_TRANS_USE_TXDATA;
        t.base.tx_data[0] = data;
        spi_device_transmit(dev->m_spi_dev, reinterpret_cast<spi_transaction_t*>(&t));
        ESP_LOGE(__FUNCTION__, "W[0x%.2x]=0x%.2x", addr, data);
    }

    Gyro::Gyro(spi_host_device_t host, gpio_num_t cs_pin)
        : device(host, cs_pin)
    {
        //bal={};
        bal.P = 10;
        bal.I = 60;
        bal.D = 0.03;
        bal.angle_offset = 0.00;
        bal.K=0.99;
    }

    void Gyro::testGyro(device* display)
    {
        ESP_LOGI(__FUNCTION__, "IN");
        {//возможно это не нужно и возможно стоит это делать до spi_bus_add_device
            /* Toggle CS pin to lock in SPI mode */
        /*	gpio_set_level(dev->spi_cfg.spics_io_num, 0);
            usleep(1);
            gpio_set_level(dev->spi_cfg.spics_io_num, 1);
            usleep(1);*/
        }
        //Either queue all transactions by calling the function spi_device_queue_trans() and, at a later time, query the result using the function spi_device_get_trans_result(), or

        ReadRegister(this, 0x6B);//6b=PWR_MGMT_1  register
        WriteRegister(this, 0x6B, 0x80);//reset
        usleep(1000);
        ReadRegister(this, 0x6B);
        WriteRegister(this, 0x6B, 0x00);
        usleep(1000);
        WriteRegister(this, 0x6B, 0x01);
        usleep(2000);
        ReadRegister(this, 0x6B);


        ReadRegister(this, 0x75);
    #define MPU6500_MEM_REV_ADDR    (0x17)
        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        ReadRegister(this, 0x1A);
        WriteRegister(this, 0x1A, 0x03);//MPU_CONFIG  = 41HZ
        ReadRegister(this, 0x1A);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        ReadRegister(this, 0x19);
        WriteRegister(this, 0x19, 0x04);//e Use a 200 Hz rate; a rate consistent with the filter update rate
        ReadRegister(this, 0x19);		// determined inset in CONFIG above

        ReadRegister(this, 0x3A);//INT_STATUS - data available?
        //if(display){
        //    display->setWindowFullScreen();
        //    display->setWindowXX(0,49);
        //}

        bool bCalibrate = false;
        if (bCalibrate) {
            int16_t gyro_offsets[3] = {};
            ReadRegisters(this, 0x13, (uint8_t*)gyro_offsets, 6);
            ESP_LOGI(__FUNCTION__, "Gyro offsets:%x, %x, %x", swapbytes(gyro_offsets[0]), swapbytes(gyro_offsets[1]), swapbytes(gyro_offsets[2]) );
            
            WriteRegisters(this, 0x13, (uint8_t*)gyro_offsets, 6);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            int16_t data[7];
            int32_t sum_data[7];
            
            memset(&sum_data, 0, sizeof(sum_data));
            int read_count=400;
            for(int i = 0 ; i < read_count; i++){
                memset(&data, 0, sizeof(data));
                ReadRegisters(this, 0x3B, (uint8_t*)data, 14/*ACCEL_XOUT_H*/);
                for(int j = 0; j<7; j++) {
                    sum_data[j] += swapbytes(data[j]);
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }                
            for(int j = 0; j<7; j++) {
                data[j]=swapbytes(-(int16_t)(sum_data[j]/read_count/4));//деление на 4 скорее всего из-за выбора scale range ±250, ±500, ±1000, and ±2000°/sec
            }

       
            gyro_offsets[0]=data[4];
            gyro_offsets[1]=data[5];
            gyro_offsets[2]=data[6];
            //E (41591) WriteRegisters:  W[13+06]=00,49,00,c2,ff,e3,
            //E (41581) WriteRegisters:  W[13+06]=ff,b7,ff,3e,00,1d,
            //                           W[13+06]=ff,b7,ff,3b,00,1e,
         //   I (41691) testGyro: avg  offsets:004a, 00c3, ffe3
         //   I (41691) testGyro: Gyro offsets:ffb6, ff3d, 1d
         //   E (41691) WriteRegisters:  W[13+06]=ff,b6,ff,3d,00,1d,
            ESP_LOGI(__FUNCTION__, "avg  offsets:%x, %x, %x", (int16_t)(sum_data[4]/read_count), (int16_t)(sum_data[5]/read_count), (int16_t)(sum_data[6]/read_count) );
            ESP_LOGI(__FUNCTION__, "Gyro offsets:%x, %x, %x", swapbytes(gyro_offsets[0]), swapbytes(gyro_offsets[1]), swapbytes(gyro_offsets[2]) );
            WriteRegisters(this, 0x13, (uint8_t*)gyro_offsets, 6);

            ReadRegisters(this, 0x13, (uint8_t*)gyro_offsets, 6);
            ESP_LOGI(__FUNCTION__, "Gyro offsets:%x, %x, %x", swapbytes(gyro_offsets[0]), swapbytes(gyro_offsets[1]), swapbytes(gyro_offsets[2]) );
        } else {
            //uint16_t gyro_offsets[3] = {0,0,0};
            uint16_t gyro_offsets[3] = {swapbytes(0x3ffb6>>2), swapbytes(0x3ff3d>>2), swapbytes(0x001d>>2)};
            WriteRegisters(this, 0x13, (uint8_t*)gyro_offsets, 6);
            
            ReadRegisters(this, 0x13, (uint8_t*)gyro_offsets, 6);
            ESP_LOGI(__FUNCTION__, "Gyro offsets:%x, %x, %x", swapbytes(gyro_offsets[0]), swapbytes(gyro_offsets[1]), swapbytes(gyro_offsets[2]) );
        }

        uint16_t data[7];

        //    constexpr size_t length=8;//stack size =2kB
        for(;;)//for(int i=0;i<1000;i++)
        {
            
            memset(&data, 0, sizeof(data));
            ReadRegisters(this, 0x3B, (uint8_t*)data, 14/*ACCEL_XOUT_H*/);

                //ESP_LOGI(__FUNCTION__, "mem free: %d, total: %d", heap_caps_get_free_size(MALLOC_CAP_DEFAULT), heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
                //heap_caps_check_integrity(MALLOC_CAP_DEFAULT, true);
            
            ax = swapbytes(data[0]);
            ay = swapbytes(data[1]);
            az = swapbytes(data[2]);
            temp = swapbytes(data[3]);
            rx = swapbytes(data[4]);
            ry = swapbytes(data[5]);
            rz = swapbytes(data[6]);
            /*
            float xangle;//
            float xangle_rot;//
            float xangle_acc;//
            float P;//
            float I;
            float D;
            int16_t delay1;
            int16_t delay2;
            */
            //bal.xangle_rot = 
            const float gyro_scale_to_deg = 250.0f;//degree per second 
            
            float axf = ax/32768.0f;
            float ayf = ay/32768.0f;
            float azf = az/32768.0f;
            float rxf = rx/32768.0f*gyro_scale_to_deg;
            float ryf = ry/32768.0f*gyro_scale_to_deg;
            float rzf = rz/32768.0f*gyro_scale_to_deg;


            if(ayf!=0){
                bal.xangle_acc = -std::atan(azf/ayf)*180/M_PI ;
            }else {
                bal.xangle_acc = 90;//+-
            }
            uint32_t time = esp_log_timestamp();
            float time_since_last_frame = (time - bal.timestamp)/1000.0f;
            bal.timestamp = time;
            float K = bal.K;
            if(time<2000){
                K = 0;//
                bal.Ei = 0;
            }
            bal.xangle_rot = bal.xangle_rot + rxf * time_since_last_frame; //100ms since last frame
            bal.xangle = K*(bal.xangle + rxf * time_since_last_frame) + (1-K)*(bal.xangle_acc); //080ms since last frame

            bal.Ep = bal.xangle + 8.2f + bal.speed/5000.0f;// 8.2f=bal.angle_offset + bal.steps/5000.0f;
            bal.Ei += bal.Ep*time_since_last_frame;
            bal.Ei = clamp(bal.Ei, -500, 500);
            //bal.Ed = (bal.Ep-bal.Elast)/time_since_last_frame;
            float K_Ed = 0.2;
            bal.Ed = (K_Ed*(bal.Ep-bal.Elast)/time_since_last_frame)+(1-K_Ed)*bal.Ed;
            bal.Elast = bal.Ep;

            static float speed_last=0; 
            float speed = bal.P*bal.Ep + (bal.I+(bal.angle_offset * bal.speed/5000.0f))*bal.Ei + bal.D*bal.Ed;
            bal.speed = (speed*0.2)+(speed_last*0.8);
            speed_last = bal.speed;
            
            bal.delay1=10000/(bal.speed!=0?bal.speed:1);// + or - = direction
            if (bal.state == 1) {
                bal.Ei = clamp(bal.Ei*0.98, -50, 50);
                bal.delay1=10000;
                speed_last = 0;
                if(std::abs(bal.speed)<10 &&  std::abs(bal.Ep)<10 ){
                    bal.state = 0;
                    bal.Ei = 0;
                    bal.steps = 0;
                }
            } else if(std::abs(bal.speed)>250){
                speed_last = 0;
                bal.state = 1;//stop. wait for stabilization
                bal.delay1=10000;
                bal.steps = 0;
            } 
            uint8_t btLogMsg[12];
            *(uint16_t*)&btLogMsg[0] = 0x0000;//MsgSize
            *(uint16_t*)&btLogMsg[2] = 0x0001;//MsgType
            *(int*)&btLogMsg[4] = bal.Ep*1024;
            *(int*)&btLogMsg[8] = bal.speed*1024;
            swapbytes(&btLogMsg[4], 4);
            swapbytes(&btLogMsg[8], 4);
            //bluetooth::sendData(0, btLogMsg, 12);
           // bal.delay2=1000/speed2;
            static int every100 = 0;
            if(!every100--) {
                every100 = 100;
                ESP_LOGI(__FUNCTION__, "Ep: %3d, Ei:%3d, Ed10:%4d, Speed: %5d, Delay1: %3d, steps:%8lld"
                , (int)bal.Ep, (int)bal.Ei, (int)(bal.Ed*10), (int)bal.speed, bal.delay1, bal.steps);
            }
            /*if(display) {
                

                std::string sax = float_to_str(axf, 1, 4);
                std::string say = float_to_str(ayf, 1, 4);
                std::string saz = float_to_str(azf, 1, 4);
                std::string stemp  = "t:"+float_to_str((temp/ 333.87f + 21.0f), 2, 2);//16.5 ~=36.6?; 6 ~= 22/// ???
                std::string srx = float_to_str(rxf, 1, 4);
                std::string sry = float_to_str(ryf, 1, 4);
                std::string srz = float_to_str(rzf, 1, 4);

                ili9341_text_attr_t attr{};
                attr.font = &ili9341_font_16x26;
                attr.fg_color = RGB(255,0,0);
                ili9341_draw_string(display, attr, sax);

                attr.fg_color = RGB(0,255,0);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, say);

                attr.fg_color = RGB(0,0,255);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, saz);

                attr.fg_color = RGB(255,0,0);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, srx);

                attr.fg_color = RGB(0,255,0);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, sry);

                attr.fg_color = RGB(0,0,255);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, srz);

                attr.fg_color = RGB(255,255,255);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, stemp);


                std::string atanzy_s = float_to_str(bal.xangle_acc, 1, 4);  //PI/2 = 90deg 
                attr.fg_color = RGB(255,255,255);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, atanzy_s);

                std::string xrot_s = float_to_str(bal.xangle_rot, 1, 4); //~26 = 90deg
                attr.fg_color = RGB(255,255,255);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, xrot_s);

                std::string xangle_s = float_to_str(bal.xangle, 1, 4);
                attr.fg_color = RGB(255,255,255);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, xangle_s);

                std::string speed_s = float_to_str(bal.speed, 1, 4);
                attr.fg_color = RGB(255,255,0);
                attr.origin_y += attr.font->height;
                ili9341_draw_string(display, attr, speed_s);

            }*/
            //usleep(150*1000);
            vTaskDelay(10 / portTICK_PERIOD_MS);//portTICK_PERIOD_MS=10
        }


    /*
        WriteRegisterCheck

        SPI_TRANSFER_DATA(0x6B, 0x80);
        usleep(100*1000);
        SPI_TRANSFER_DATA(0x6B, 0x00);
        usleep(100*1000);
        SPI_TRANSFER_DATA(0x6B, 0x01);
        usleep(100*1000);
        SPI_TRANSFER_DATA(0x1A, 0x03);//DLPF_41HZ

        SPI_TRANSFER_DATA(0x19, 0x04);//SMPL_200HZ

        uint8_t c = read_byte(mpu_i2c_addr, GYRO_CONFIG)*/
        ESP_LOGI(__FUNCTION__, "OUT");
    }

}