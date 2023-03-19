#pragma once
#include <driver/spi_master.h>
#include "sdkconfig.h"
#include <initializer_list>
		

namespace spi
{
    static const spi_host_device_t default_spi_host = HSPI_HOST;
    //spi_host_device_t SPI_BUS = ;HSPI_HOST
    esp_err_t default_init(spi_host_device_t spihost = HSPI_HOST);


    /*
    #define SPI_TRANSFER(...) do { 						\
        uint8_t buf[] = { __VA_ARGS__ }; 				\
        gpio_set_level(DCRS_PIN, 0);					\
        send(dev, &buf[0], 1);							\
        gpio_set_level(DCRS_PIN, 1);					\
        spi_transaction_t t; 							\
        memset(&t, 0, sizeof(t));						\
        for(uint8_t i=1; i<sizeof(buf); i++)			\
        {												\
            t.length=8;									\
            t.tx_buffer=buf+i;							\
            spi_device_transmit(dev->spi_dev, &t);      \
        }												\
    } while(0)

    #define SPI_TRANSFER_DATA(...) do { 				\
        uint8_t buf[] = { __VA_ARGS__ }; 				\
        spi_transaction_t t; 							\
        memset(&t, 0, sizeof(t));						\
        t.length=sizeof(buf)*8;							\
        t.tx_buffer=buf;								\
        spi_device_transmit(dev->spi_dev, &t);          \
    } while(0)

    //отправляет 1 байт-адрес регистра
    #define SPI_REQUIRE_DATA(...) do { 			        \
        uint8_t buf[] = { __VA_ARGS__ }; 				\
        spi_transaction_t t; 							\
        memset(&t, 0, sizeof(t));						\
        t.length=sizeof(buf)*8;							\
        t.tx_buffer=buf;								\
        spi_device_transmit(dev->spi_dev, &t);          \
    } while(0)
    */
    
    class device
    {
    public:
        //device();
        device(spi_host_device_t host, gpio_num_t cs_pin);
        //virtual ~device(){}
        void transfer(const uint8_t* data, size_t size);
        void transfer(const uint16_t* data, size_t size);
        //template<typename... Args>
        //void transfer(Args... args);
        //template<typename >
    // void transfer( std::initializer_list<uint8_t> list);

        spi_device_interface_config_t m_spi_cfg;
        spi_device_handle_t m_spi_dev;
        spi_host_device_t m_host;
    };

    //Display device
    class ili9341 : public device
    {
        using SUPER = device;
    public:
        ili9341(spi_host_device_t host, gpio_num_t cs_pin, gpio_num_t dcrs_pin);
        //template<typename... Args>
        //void transfer_cmd(uint8_t cmd, Args... args);
        //void transfer_cmd(uint8_t cmd);
        /*void transfer_cmd(uint8_t cmd, uint8_t d0);
        void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1);
        void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t);
        void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t);
        void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t);
        void transfer_cmd(uint8_t cmd, uint8_t d0, uint8_t d1, uint8_t, uint8_t, uint8_t, uint8_t);*/
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


    class Gyro : public device
    {
    public:
        Gyro(spi_host_device_t host, gpio_num_t cs_pin);
        void testGyro(ili9341* dev);
    public:
        int16_t ax, ay, az;
        int16_t rx, ry, rz;
        int16_t temp;
        
        
        struct Balancing{
            uint32_t timestamp;
            float xangle;//
            float xangle_rot;//
            float xangle_acc;//
            float P;//
            float I;
            float D;
            float K;//коэффицент комплиментарного фильтра, угол вычисляется: 0.0 - по аккселерометру .. 1.0 -по гироскопу
            float angle_offset;
            float Ep=0;
            float Ei=0;
            float Ed=0;
            float Elast=0;
            int16_t delay1;
            int16_t delay2;
            float speed=0;
            int64_t steps;
            uint8_t state;
        } bal;
    };


};