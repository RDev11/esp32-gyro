//#pragma warning( disable : 4101) -fpermissive
#include "stdafx.h"
#include "spi.h"

//#define ILI9341
//#include "ili9341_d.h"
//#include "text.h"
//==============================================================
#define TAG "main"
#define DCRS_PIN  static_cast<gpio_num_t>(25)
// 16 bits per pixel
#define SPI_BYTESPERPIXEL 2
//#include "img_doom_v.h"
uint16_t img_doom[]={0};
//

uint16_t color=0;
using namespace spi;
using mydevice_t = spi::device;








void DrawDoomImg(ili9341* dev, uint8_t* img_doom, size_t size)
{
    ESP_LOGI(__FUNCTION__, "IN");
    
    //setWindow(dev, 0, DISPLAY_NATIVE_WIDTH-1, 0, DISPLAY_NATIVE_WIDTH-1);
    
    //dev->transfer(img_doom, size);
    for(size_t i = 0; i < size; i+=4092)
    {
        //gpio_set_level(m_dcrs_pin, 1);
        dev->transfer(&img_doom[i], std::min((size_t)4092, size-i));
        //vTaskDelay(1);
    }
    ESP_LOGI(__FUNCTION__, "OUT");
}

void app_main_cpp(void);
extern "C"{
    void app_main(void);
}
void app_main(void)
{
	app_main_cpp();
}
void app_main_cpp(void)
{
	ESP_LOGI(TAG, "BEGIN");
	gpio_num_t led_gpio = (gpio_num_t)CONFIG_BLINK_GPIO;
	gpio_num_t button_gpio = (gpio_num_t)CONFIG_BUTTON_GPIO;
	{//
	  gpio_reset_pin(led_gpio);
	  gpio_reset_pin(button_gpio);
	  gpio_reset_pin(DCRS_PIN);
	  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
	  gpio_set_direction(button_gpio, GPIO_MODE_INPUT);
	  gpio_set_pull_mode(button_gpio, GPIO_PULLUP_ONLY);
	  gpio_set_direction(DCRS_PIN, GPIO_MODE_OUTPUT);
	  gpio_set_level(led_gpio, 1);
	}
	//ESP_LOGI(TAG, "test float=%2.4f", 0.4);
	/*esp_err_t ret=ESP_OK;
	spi_bus_config_t cfg={
			.mosi_io_num=CONFIG_PIN_NUM_MOSI,
			.miso_io_num=CONFIG_PIN_NUM_MISO,
			.sclk_io_num=CONFIG_PIN_NUM_CLK,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 0,
			.flags = 0
	};

	ret = spi_bus_initialize(HSPI_HOST, &cfg, 1);*/

    spi::default_init(HSPI_HOST);
	spi::ili9341 dev(HSPI_HOST, (gpio_num_t)CONFIG_PIN_NUM_CS, (gpio_num_t)DCRS_PIN);//display device

	spi::Gyro dev2(HSPI_HOST, (gpio_num_t)26);//accelerometer/gyroscope
	{
		memset(&dev2.m_spi_cfg, 0, sizeof(dev2.m_spi_cfg));
		dev2.m_spi_cfg.spics_io_num = 26;
		dev2.m_spi_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M;//CLOCK_SPEED_HZ;
		dev2.m_spi_cfg.mode = 3;
		dev2.m_spi_cfg.queue_size = 1;
		dev2.m_spi_cfg.flags = SPI_DEVICE_NO_DUMMY;
		//dev2.spi_cfg.address_bits =
		spi_bus_add_device(HSPI_HOST, &dev2.m_spi_cfg, &dev2.m_spi_dev);
	}


	swapbytes((uint8_t*)img_doom, sizeof(img_doom));
	vTaskDelay(1);
	bool bImg=false;
	dev.setWindowFullScreen();
	DrawDoomImg(&dev, (uint8_t*)img_doom, sizeof(img_doom));
	//dev.setWindowFullScreen();
	//DrawDoomImg(&dev, ((uint8_t*)img_doom)+sizeof(img_doom)/2, sizeof(img_doom)/2);
	
	vTaskDelay(1);
	dev.ClearScreen();
	vTaskDelay(1);
	dev2.testGyro(&dev);
	color=random();
  while (1) {
    if(gpio_get_level(button_gpio)) //кнопка НЕ нажата
	{
		gpio_set_level(led_gpio, 0);
		
    	ESP_LOGI(__FUNCTION__, ".");
	}
    else
    	{
    	gpio_set_level(led_gpio, 1);
    	color=random();
    	/*if(bImg)
			DrawDoomImg(&dev, (uint8_t*)img_doom, sizeof(img_doom));
    		//DrawDoomImg(&dev);
    	else
    		ClearScreen(&dev);*/
    	bImg=!bImg;
    	}
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

