//#pragma warning( disable : 4101) -fpermissive
#include "stdafx.h"
#include "spi.h"

//#define ILI9341
//#include "ili9341_d.h"
//#include "text.h"
//==============================================================
#define TAG_LOG "main"
// 16 bits per pixel
#define SPI_BYTESPERPIXEL 2
//#include "img_doom_v.h"
uint16_t img_doom[]={0};
//
//#include "bluetooth/bluetooth_serial.h"
#include "wifi.h"
///
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
///
//using namespace spi;

class MyMotor {
public:
	MyMotor(gpio_num_t dir_pin, gpio_num_t step_pin, bool inverse_dir = false) {
		dir_gpio = dir_pin;
		step_gpio = step_pin;
		_step = 0;
		dir = 1;
		this->inverse_dir = inverse_dir;
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
		gpio_set_level(dir_gpio, (inverse_dir ? 1-dir : dir));
	}
public:
	gpio_num_t dir_gpio;
	gpio_num_t step_gpio;
	int _step;
	int dir;
	bool inverse_dir;
};
MyMotor myMotor1((gpio_num_t)CONFIG_PIN_NUM_MOT1_DIR, (gpio_num_t)CONFIG_PIN_NUM_MOT1_STEP);
MyMotor myMotor2((gpio_num_t)CONFIG_PIN_NUM_MOT2_DIR, (gpio_num_t)CONFIG_PIN_NUM_MOT2_STEP, true);

spi::Gyro dev2(HSPI_HOST, (gpio_num_t)26);

static bool example_timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
	/*	
		typedef struct {
			uint64_t event_count;
		} example_queue_element_t;

	    QueueHandle_t queue = (QueueHandle_t)user_ctx;
		// Retrieve the count value from event data
		example_queue_element_t ele = {
			.event_count = edata->count_value
		};
	
		// Optional: send the event data to other task by OS queue
		// Do not introduce complex logics in callbacks
		// Suggest dealing with event data in the main loop, instead of in this callback
    	xQueueSendFromISR(queue, &ele, &high_task_awoken);
	*/
	//high_task_awoken = pdTRUE;
	
	myMotor1.setDir(dev2.bal.delay1>0?1:0);
	myMotor2.setDir(dev2.bal.delay1>0?1:0);
	if(dev2.bal.delay1!=10000)
	{
		myMotor1.step();
		myMotor2.step();
		dev2.bal.steps += dev2.bal.delay1>0?1:-1;
	}
	//ESP_DRAM_LOGI(__FUNCTION__, "ON TIMER %d %d", delay, edata->alarm_value);
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + std::max(std::abs(dev2.bal.delay1), 20), 
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    // return whether we need to yield at the end of ISR
    return high_task_awoken == pdTRUE;
}

gptimer_alarm_config_t alarm_config = {
    .alarm_count = 1000000, // period = 1s @resolution 1MHz
  //  .reload_count = 0, // counter will reload with 0 on alarm event
  //  .flags = {.auto_reload_on_alarm=true}, // enable auto-reload
};

//моторы управляются через dev2.bal
void init_motors() {
	gptimer_handle_t gptimer = NULL;
	gptimer_config_t timer_config = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

	ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

	gptimer_event_callbacks_t cbs = {
		.on_alarm = example_timer_on_alarm_cb, // register user callback
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, nullptr));
	myMotor1.init();
	myMotor2.init();
	ESP_ERROR_CHECK(gptimer_enable(gptimer));
	ESP_ERROR_CHECK(gptimer_start(gptimer));
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
	ESP_LOGI(TAG_LOG, "BEGIN");
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
	dev2.bal.P = 10;
	dev2.bal.I = 60;
	dev2.bal.D = 0.03;
	dev2.bal.angle_offset = 0.00;
	dev2.bal.K=0.99;
/*
#include "nvs_flash.h"
#include "nvs.h"

// Инициализация NVS
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	ESP_ERROR_CHECK(nvs_flash_erase());
	ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);
// Запись
nvs_handle_t handle;
nvs_open("storage", NVS_READWRITE, &handle);
nvs_set_i32(handle, "boot_count", 123);  // Сохраняем число
nvs_commit(handle);  // Фиксируем изменения
nvs_close(handle);

// Чтение
int32_t boot_count = 0;
nvs_open("storage", NVS_READONLY, &handle);
nvs_get_i32(handle, "boot_count", &boot_count);
nvs_close(handle);
*/

    ESP_LOGI(TAG_LOG, "Initializing WiFi");
    gWifi().init_sta();


	gpio_num_t led_gpio = (gpio_num_t)CONFIG_BLINK_GPIO;
	gpio_num_t button_gpio = (gpio_num_t)CONFIG_BUTTON_GPIO;
	gpio_num_t dcrs_gpio = (gpio_num_t)CONFIG_PIN_NUM_DCRS;
	{//
	  gpio_reset_pin(led_gpio);
	  gpio_reset_pin(button_gpio);
	  gpio_reset_pin(dcrs_gpio);
	  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
	  gpio_set_direction(button_gpio, GPIO_MODE_INPUT);
	  gpio_set_pull_mode(button_gpio, GPIO_PULLUP_ONLY);
	  gpio_set_direction(dcrs_gpio, GPIO_MODE_OUTPUT);
	  gpio_set_level(led_gpio, 1);
	}
	

	init_motors();

	ESP_LOGI(TAG_LOG, "after timers");
	//ESP_LOGI(TAG_LOG, "test float=%2.4f", 0.4);


    spi::default_init(HSPI_HOST);

	ESP_LOGI(TAG_LOG, "after init display");

	//spi::Gyro dev2(HSPI_HOST, (gpio_num_t)CONFIG_PIN_NUM_CS_GYRO);//accelerometer/gyroscope
	{
		memset(&dev2.m_spi_cfg, 0, sizeof(dev2.m_spi_cfg));
		dev2.m_spi_cfg.spics_io_num = (gpio_num_t)CONFIG_PIN_NUM_CS_GYRO;
		dev2.m_spi_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M;//CLOCK_SPEED_HZ;
		dev2.m_spi_cfg.mode = 3;
		dev2.m_spi_cfg.queue_size = 1;
		dev2.m_spi_cfg.flags = SPI_DEVICE_NO_DUMMY;
		//dev2.spi_cfg.address_bits =
		spi_bus_add_device(HSPI_HOST, &dev2.m_spi_cfg, &dev2.m_spi_dev);
	}

	//bluetooth::bluetooth_start_accept();

	dev2.testGyro(nullptr);
	/*
	while (1) {
		if(gpio_get_level(button_gpio)) //кнопка НЕ нажата
		{
			gpio_set_level(led_gpio, 0);
			
			ESP_LOGI(__FUNCTION__, ".");
		}
		else
			{
			gpio_set_level(led_gpio, 1);
			}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}*/
}

