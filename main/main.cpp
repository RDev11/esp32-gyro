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
void bt_test();

uint16_t color=0;
using namespace spi;
using mydevice_t = spi::device;



class MyMotor {
public:
	MyMotor() {
		dir_gpio = (gpio_num_t)33;
		step_gpio = (gpio_num_t)32;
		_step = 0;
		dir = 0;
	}
	void init() {
		
		gpio_reset_pin(dir_gpio);
		gpio_reset_pin(step_gpio);
		gpio_set_direction(dir_gpio, GPIO_MODE_OUTPUT);
		gpio_set_direction(step_gpio, GPIO_MODE_OUTPUT);
		gpio_set_level(dir_gpio, dir);
	}
	void step() {
		_step = 1 - _step;
		gpio_set_level(step_gpio, _step);
	}
	void setDir(int dir){
		this->dir = dir;
		gpio_set_level(dir_gpio, dir);
	}
public:
	gpio_num_t dir_gpio;
	gpio_num_t step_gpio;
	int _step;
	int dir;
};
MyMotor myMotor;
spi::Gyro dev2(HSPI_HOST, (gpio_num_t)26);
bool IRAM_ATTR onTimer(void* args){
	//digitalWrite(LED, !digitalRead(LED));
	
	
	int32_t delay = 65536*50 ;
	myMotor.setDir(dev2.ax>0?1:0);
	if (dev2.ax!=0){
		delay = delay/std::abs(dev2.ax);
	} else {
	}
	if(delay>20000) {
		delay = 20000;
	} else {
		myMotor.step();
	}
	delay = std::max(std::abs(delay), 10);
	//delay = 100;
 	//ESP_DRAM_LOGI(__FUNCTION__, "ON TIMER %d", delay);
	//timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, delay);
	return false;
}



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
	

	timer_group_t group_num;
	timer_idx_t timer_num;
	timer_config_t config{};
		config.divider = 80; // 80MHz / 80 = 1MHz
		config.counter_dir = TIMER_COUNT_UP;
		config.counter_en = TIMER_PAUSE; // pause after init
		config.alarm_en = TIMER_ALARM_EN; //interrupt enable
		config.auto_reload = TIMER_AUTORELOAD_EN;// TIMER_AUTORELOAD_EN; //loop 

	esp_err_t err = timer_init(TIMER_GROUP_0, TIMER_0, &config);

	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 3e4);

	timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, onTimer, nullptr, 0);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);

	myMotor.init();
	timer_start(TIMER_GROUP_0, TIMER_0);

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

	//spi::Gyro dev2(HSPI_HOST, (gpio_num_t)26);//accelerometer/gyroscope
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

