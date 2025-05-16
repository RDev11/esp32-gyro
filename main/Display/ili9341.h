#pragma once
#include "SPI.h"

//Display device
class ili9341 : public device
{
    using SUPER = device;
public:
    ili9341(spi_host_device_t host, gpio_num_t cs_pin, gpio_num_t dcrs_pin);
    //template<typename... Args>
    //void transfer_cmd(uint8_t cmd, Args... args);
    //void transfer_cmd(uint8_t cmd);
    ////void transfer_cmd(uint8_t cmd, uint8_t d0);
    ////void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1);
    ////void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t);
    ////void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t);
    ////void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t);
    ////void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t, uint8_t);
    //void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    void transfer_cmd(uint8_t cmd, std::initializer_list<uint8_t> data={});
    void transfer_cmd(uint8_t cmd, uint8_t* data, size_t size);

    void setWindowXX(uint16_t x1, uint16_t x2);
    //void setWindowXW(uint16_t x1, uint16_t x2);
    void setWindowYY(uint16_t y1, uint16_t y2);
    //void setWindowYW(uint16_t y1, uint16_t y2);
    void setWindow(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);
    void setWindowFullScreen();
    
    void ClearScreen();
    void DrawDoomImg();

    uint16_t getScreenWidth();//240
    uint16_t getScreenHeight();//360

public:
    gpio_num_t m_dcrs_pin;//0-cmd, 1-data
};