#include "ili9341.h"

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