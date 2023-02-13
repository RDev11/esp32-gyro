#include "stdafx.h"
#include "SPI.h"

#define ILI9341
#include "ili9341_d.h"

#include "text2.h"

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
        .max_transfer_sz = 0,
        .flags = 0
    };
    ret = spi_bus_initialize(spihost, &cfg, 1);
    return ret;
}
}


namespace spi{ //spi::device 
    device::device(spi_host_device_t host, gpio_num_t cs_pin)
        : m_spi_cfg{.spics_io_num = cs_pin}
        , m_spi_dev{}
        , m_host(host)
    {
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

    ili9341::ili9341(spi_host_device_t host, gpio_num_t cs_pin, gpio_num_t dcrs_pin)
        : SUPER(host, cs_pin)
        , m_dcrs_pin(dcrs_pin)
    {

        ESP_LOGI(__FUNCTION__, "IN");
        memset(&m_spi_cfg, 0, sizeof(m_spi_cfg));
        m_spi_cfg.spics_io_num = cs_pin;
        m_spi_cfg.clock_speed_hz = SPI_MASTER_FREQ_40M;//CLOCK_SPEED_HZ;
        m_spi_cfg.mode = 0;
        m_spi_cfg.queue_size = 1;
        m_spi_cfg.flags = SPI_DEVICE_NO_DUMMY;
        spi_bus_add_device(host, &m_spi_cfg, &m_spi_dev);

        //buf[0] = 0x0f | 0x00<<8;//отключим режим тестирования
        //send(dev, buf, 1);

        transfer_cmd(0x01/*Software Reset*/);
        usleep(5*1000);
        transfer_cmd(0x28/*Display OFF*/);
        // The following appear in ILI9341 Data Sheet v1.11 (2011/06/10), but not in older v1.02 (2010/12/06).
        transfer_cmd(0xCB/*Power Control A*/, {0x39/*Reserved*/, 0x2C/*Reserved*/, 0x00/*Reserved*/, 0x34/*REG_VD=1.6V*/, 0x02/*VBC=5.6V*/}); // These are the same as power on.
        transfer_cmd(0xCF/*Power Control B*/, {0x00/*Always Zero*/, 0xC1/*Power Control=0,DRV_ena=0,PCEQ=1*/, 0x30/*DC_ena=1*/}); // Not sure what the effect is, set to default as per ILI9341 Application Notes v0.6 (2011/03/11) document (which is not apparently same as default at power on).
        transfer_cmd(0xE8/*Driver Timing Control A*/, {0x85, 0x00, 0x78}); // Not sure what the effect is, set to default as per ILI9341 Application Notes v0.6 (2011/03/11) document (which is not apparently same as default at power on).
        transfer_cmd(0xEA/*Driver Timing Control B*/, {0x00, 0x00}); // Not sure what the effect is, set to default as per ILI9341 Application Notes v0.6 (2011/03/11) document (which is not apparently same as default at power on).
        transfer_cmd(0xED/*Power On Sequence Control*/, {0x64, 0x03, 0x12, 0x81}); // Not sure what the effect is, set to default as per ILI9341 Application Notes v0.6 (2011/03/11) document (which is not apparently same as default at power on).
        //#if ILI9341_UPDATE_FRAMERATE == ILI9341_FRAMERATE_119_HZ // Setting pump ratio if update rate is less than 119 Hz does not look good but produces shimmering in panning motion.
        transfer_cmd(0xF7/*Pump Ratio Control*/, {ILI9341_PUMP_CONTROL});
        //#endif

        // The following appear also in old ILI9341 Data Sheet v1.02 (2010/12/06).
        transfer_cmd(0xC0/*Power Control 1*/, {0x23/*VRH=4.60V*/}); // Set the GVDD level, which is a reference level for the VCOM level and the grayscale voltage level.
        transfer_cmd(0xC1/*Power Control 2*/, {0x10/*AVCC=VCIx2,VGH=VCIx7,VGL=-VCIx4*/}); // Sets the factor used in the step-up circuits. To reduce power consumption, set a smaller factor.
        transfer_cmd(0xC5/*VCOM Control 1*/, {0x3e/*VCOMH=4.250V*/, 0x28/*VCOML=-1.500V*/}); // Adjusting VCOM 1 and 2 can control display brightness
        transfer_cmd(0xC7/*VCOM Control 2*/, {0x86/*VCOMH=VMH-58,VCOML=VML-58*/});
        // POWER CONTROL,VRH[5:0]



    #define MADCTL_BGR_PIXEL_ORDER (1<<3)
    #define MADCTL_ROW_COLUMN_EXCHANGE (1<<5)
    #define MADCTL_COLUMN_ADDRESS_ORDER_SWAP (1<<6)
    #define MADCTL_ROW_ADDRESS_ORDER_SWAP (1<<7)
    #define MADCTL_ROTATE_180_DEGREES (MADCTL_COLUMN_ADDRESS_ORDER_SWAP | MADCTL_ROW_ADDRESS_ORDER_SWAP)

        uint8_t madctl = 0;
    #ifndef DISPLAY_SWAP_BGR
        madctl |= MADCTL_BGR_PIXEL_ORDER;
    #endif
    #if defined(DISPLAY_FLIP_ORIENTATION_IN_HARDWARE)
        madctl |= MADCTL_ROW_COLUMN_EXCHANGE;
    #endif
    #ifdef DISPLAY_ROTATE_180_DEGREES
        madctl ^= MADCTL_ROTATE_180_DEGREES;
    #endif

        madctl |= MADCTL_ROW_ADDRESS_ORDER_SWAP;
        transfer_cmd(0x36/*MADCTL: Memory Access Control*/, {madctl});


    #ifdef DISPLAY_INVERT_COLORS
        transfer_cmd(0x21/*Display Inversion ON*/);
    #else
        transfer_cmd(0x20/*Display Inversion OFF*/);
    #endif
        transfer_cmd(0x3A/*COLMOD: Pixel Format Set*/, {0x55/*DPI=16bits/pixel,DBI=16bits/pixel*/});

        // According to spec sheet, display frame rate in 4-wire SPI "internal clock mode" is computed with the following formula:
        // frameRate = 615000 / [ (pow(2,DIVA) * (320 + VFP + VBP) * RTNA ]
        // where
        // - DIVA is clock division ratio, 0 <= DIVA <= 3; so pow(2,DIVA) is either 1, 2, 4 or 8.
        // - RTNA specifies the number of clocks assigned to each horizontal scanline, and must follow 16 <= RTNA <= 31.
        // - VFP is vertical front porch, number of idle sleep scanlines before refreshing a new frame, 2 <= VFP <= 127.
        // - VBP is vertical back porch, number of idle sleep scanlines after refreshing a new frame, 2 <= VBP <= 127.

        // Max refresh rate then is with DIVA=0, VFP=2, VBP=2 and RTNA=16:
        // maxFrameRate = 615000 / (1 * (320 + 2 + 2) * 16) = 118.63 Hz

        // To get 60fps, set DIVA=0, RTNA=31, VFP=2 and VBP=2:
        // minFrameRate = 615000 / (8 * (320 + 2 + 2) * 31) = 61.23 Hz

        // It seems that in internal clock mode, horizontal front and back porch settings (HFP, BFP) are ignored(?)

        transfer_cmd(0xB1/*Frame Rate Control (In Normal Mode/Full Colors)*/,{
            0x00/*DIVA=fosc*/,
            ILI9341_FRAMERATE_119_HZ/*RTNA(Frame Rate)*/});
        transfer_cmd(0xB6/*Display Function Control*/,{
            0x08/*PTG=Interval Scan,PT=V63/V0/VCOML/VCOMH*/,
            0x82/*REV=1(Normally white),ISC(Scan Cycle)=5 frames*/,
            0x27/*LCD Driver Lines=320*/});
        transfer_cmd(0xF2/*Enable 3G*/, {0x02/*False*/}); // This one is present only in ILI9341 Data Sheet v1.11 (2011/06/10) // 3GAMMA FUNCTION DISABLE
        transfer_cmd(0x26/*Gamma Set*/, {0x01/*Gamma curve 1 (G2.2)*/});
        transfer_cmd(0xE0/*Positive Gamma Correction*/, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00});
        transfer_cmd(0xE1/*Negative Gamma Correction*/, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F});
        transfer_cmd(0x11/*Sleep Out*/);
        usleep(120 * 1000);
        transfer_cmd(/*Display ON*/0x29);
        ESP_LOGI(__FUNCTION__, "OUT");
    }

    /*
    template<typename... Args>
    void ili9341::transfer_cmd(uint8_t cmd, Args... args)
    {
        gpio_set_level(m_dcrs_pin, 0);
        //transfer(cmd, 1);	
        gpio_set_level(m_dcrs_pin, 1);
        //if (sizeof(std::tuple<Args...>)>0)
        {				
            //transfer(args...);
        }		    
    }*/

    /*void ili9341::transfer_cmd(uint8_t cmd)
    {
        gpio_set_level(m_dcrs_pin, 0);
        transfer(&cmd, 1);	
        gpio_set_level(m_dcrs_pin, 1);
        //if (sizeof(std::tuple<Args...>)>0)
        {				
            //transfer(args...);
        }		    
    }*/
    void ili9341::transfer_cmd(uint8_t cmd, std::initializer_list<uint8_t> data)
    {
        

        uint8_t buf[data.size()];
        int i=0;
        for(auto iter = data.begin();iter!=data.end();++iter)
        {
            buf[i++] = *iter;
        }
        gpio_set_level(m_dcrs_pin, 0);
        transfer(&cmd, 1);
        gpio_set_level(m_dcrs_pin, 1);
        if (data.size()>0)
        {				
            transfer(buf, data.size());
        }		    
    }
    void ili9341::transfer_cmd(uint8_t cmd, uint8_t* data, size_t size)
    {
        gpio_set_level(m_dcrs_pin, 0);
        transfer(&cmd, 1);
        gpio_set_level(m_dcrs_pin, 1);
        if (size>0)
        {				
            transfer(data, size);
        }		    
    }
    /*void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0){}
    void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1){}
    void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t){}
    void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t){}
    void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t){}
    void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t, uint8_t){}*/
    //void ili9341::transfer_cmd(std::initializer_list<uint8_t> data){}
    //void ili9341::transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t){}

    void ili9341::setWindowXX(uint16_t x1, uint16_t x2)
    {
        transfer_cmd(DISPLAY_SET_CURSOR_X, {HIGH_BYTE(x1), LOW_BYTE(x1), HIGH_BYTE(x2), LOW_BYTE(x2)});
        transfer_cmd(0x2C);
    }
    void ili9341::setWindowYY(uint16_t y1, uint16_t y2)
    {
        transfer_cmd(DISPLAY_SET_CURSOR_Y, {HIGH_BYTE(y1), LOW_BYTE(y1), HIGH_BYTE(y2), LOW_BYTE(y2)});
        transfer_cmd(0x2C);
    }
    void ili9341::setWindow(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
    {
        /*#ifdef DISPLAY_SPI_BUS_IS_16BITS_WIDE
        transfer_cmd(DISPLAY_SET_CURSOR_X, 0, 0, 0, 0, 0, (DISPLAY_WIDTH-1) >> 8, 0, (DISPLAY_WIDTH-1) & 0xFF);
        transfer_cmd(DISPLAY_SET_CURSOR_Y, 0, (uint8_t)(y >> 8), 0, (uint8_t)(y & 0xFF), 0, (DISPLAY_HEIGHT-1) >> 8, 0, (DISPLAY_HEIGHT-1) & 0xFF);
        #elif defined(DISPLAY_SET_CURSOR_IS_8_BIT)
        transfer_cmd(DISPLAY_SET_CURSOR_X, 0, DISPLAY_WIDTH-1);
        transfer_cmd(DISPLAY_SET_CURSOR_Y, (uint8_t)y, DISPLAY_HEIGHT-1);
        #else*/
        setWindowXX(x1,x2);
        setWindowYY(y1,y2);
        //#endif
    }
    void ili9341::setWindowFullScreen()
    {
        setWindow(0, DISPLAY_NATIVE_WIDTH-1, 0, DISPLAY_NATIVE_HEIGHT-1);
    }
    /*
    template<>
    void device::transfer(uint8_t);
    template<>
    void device::transfer(uint8_t, uint8_t);
    template<>
    void device::transfer(uint8_t, uint8_t, uint8_t);
    template<>
    void device::transfer(uint8_t, uint8_t, uint8_t, uint8_t);
    template<>
    void device::transfer(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    template<>
    void device::transfer(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    */
   

    uint16_t ili9341::getScreenWidth() { return DISPLAY_NATIVE_WIDTH; }//240
    uint16_t ili9341::getScreenHeight() { return DISPLAY_NATIVE_HEIGHT; }//360

    void ili9341::ClearScreen()
    {

        ESP_LOGI(__FUNCTION__, "IN");
        spi_device_acquire_bus(m_spi_dev, portMAX_DELAY);
        setWindowFullScreen();
        transfer_cmd(0x2C);
        for(int y = 0; y < DISPLAY_NATIVE_HEIGHT; ++y)
        {
            //uint8_t buf[DISPLAY_NATIVE_HEIGHT*SPI_BYTESPERPIXEL]={0};
            //memset(clearLine->data, 0, clearLine->size);
            //spi_transaction_t t;

            //setWindowYY(dev, y, y);
            //SPI_TRANSFER(0x2C);
            gpio_set_level(m_dcrs_pin, 1);//это data секция предыдущей команды
            for(uint32_t x=0; x<DISPLAY_NATIVE_WIDTH;x+=24)
            {//HIGH(RGB(0xFF,0,0)):HIGH(RGB(0,0,0xFF)
                uint16_t buf[24]{};
                transfer(buf,24*2);
                /*if((y/8+x/8)%2)
                {

                }*/
                /*transfer(
                    //(y/8+x/8)%2==0?HIGH_BYTE(color):(0xFF - HIGH_BYTE(color)),
                    //(y/8+x/8)%2==0? LOW_BYTE(color):(0xFF - LOW_BYTE(color)),
                    (y/8+x/8)%2==0?HIGH_BYTE(RGB(x,x,0)):(HIGH_BYTE(RGB(0,0,x))),
                    (y/8+x/8)%2==0? LOW_BYTE(RGB(x,x,0)):(LOW_BYTE(RGB(0,0,x)))
                );*/
            }
            //buf[0]=0x2C;
            //t.tx_buffer=&buf[0];
            //spi_device_transmit(dev->m_spi_dev, &t);

            /*memset(&t, 0, sizeof(t));
            for(uint32_t i=0; i<sizeof(buf);i++)
            {
            //	SPI_TRANSFER(0x2C, 1, i);
                / *t.length=8;
                buf[i]=i;
                buf[i]=0x2C;
                t.tx_buffer=buf+i;
                spi_device_transmit(dev->m_spi_dev, &t);* /
            }*/
            //	vTaskDelay(1);
        } 
        spi_device_release_bus(m_spi_dev);
        ESP_LOGI(__FUNCTION__, "OUT");
    }



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

        char* str = new char[count*2+1];
        for(int i = 0; i<count; i++)
        {
            sprintf(str+i*3, "%.2x,",data[i]);
        }
        ESP_LOGE(__FUNCTION__, " R[%.2x+%.2x]=%s", addr, count, str);
        delete[] str;
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
        ESP_LOGE(__FUNCTION__, " R[0x%.2x]=0x%.2x", addr, t.base.rx_data[0]);
        return t.base.rx_data[0];
        }
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
        
    }

    void Gyro::testGyro(ili9341* dev)
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


        ReadRegister(this, 0x6B);
        WriteRegister(this, 0x6B, 0x80);
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
        dev->setWindowFullScreen();
        dev->setWindowXX(0,49);
        uint16_t data[7];

            constexpr size_t length=8;//stack size =2kB
        for(;;)//for(int i=0;i<1000;i++)
        {
            memset(&data, 0, sizeof(data));
            ReadRegisters(this, 0x3B, (uint8_t*)data, 14/*ACCEL_XOUT_H*/);
            
            //ReadRegisters:  R[3b+0e]=ff,80,00,6c,49,48,05,69,00,5e,00,ae,ff,f0,

            //std::string ax = std::to_string(0.01);
            //std::string ax = std::to_string((int)((int16_t)swapbytes(data[0])/32768.0f*1000));

            /*ESP_LOGI(__FUNCTION__, "wtf %d", 123);
            float asdf =  ((int16_t)swapbytes(data[0]))/32768.0f;
            ESP_LOGI(__FUNCTION__, "float ok");
            double asd =  ((int16_t)swapbytes(data[0]))/32768.0;
            ESP_LOGI(__FUNCTION__, "double ok");
            ESP_LOGI(__FUNCTION__, "test float=%2.4f ", asdf);
            ESP_LOGI(__FUNCTION__, "test float=%2.4lf ", asd);*/
            //double asd =  ((int16_t)swapbytes(data[0]))/32768.0;
            //snprintf(strbuf, 1023, "%1.5lf ", asd);
            //ESP_LOGI(__FUNCTION__,  strbuf);

            std::string ax = float_to_str(((int16_t)swapbytes(data[0])/32768.0f), 2, 4);
            std::string ay = float_to_str(((int16_t)swapbytes(data[1])/32768.0f), 2, 4);
            std::string az = float_to_str(((int16_t)swapbytes(data[2])/32768.0f), 2, 4);
            std::string t  = float_to_str(((int16_t)swapbytes(data[3])/32768.0f), 2, 4);
            std::string gx = float_to_str(((int16_t)swapbytes(data[4])/32768.0f), 2, 4);
            std::string gy = float_to_str(((int16_t)swapbytes(data[5])/32768.0f), 2, 4);
            std::string gz = float_to_str(((int16_t)swapbytes(data[6])/32768.0f), 2, 4);

            ili9341_text_attr_t attr{};
            attr.font = &ili9341_font_16x26;
            attr.fg_color = RGB(255,0,0);
            ili9341_draw_string(dev, attr, ax);

            attr.fg_color = RGB(0,255,0);
            attr.origin_y += attr.font->height;
            ili9341_draw_string(dev, attr, ay);

            attr.fg_color = RGB(0,0,255);
            attr.origin_y += attr.font->height;
            ili9341_draw_string(dev, attr, az);

            attr.fg_color = RGB(255,0,0);
            attr.origin_y += attr.font->height;
            ili9341_draw_string(dev, attr, gx);

            attr.fg_color = RGB(0,255,0);
            attr.origin_y += attr.font->height;
            ili9341_draw_string(dev, attr, gy);

            attr.fg_color = RGB(0,0,255);
            attr.origin_y += attr.font->height;
            ili9341_draw_string(dev, attr, gz);

            attr.fg_color = RGB(255,255,255);
            attr.origin_y += attr.font->height;
            ili9341_draw_string(dev, attr, t);
            
            usleep(250*1000);
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