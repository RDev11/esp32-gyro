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


void main () {

#define USE_DISPLAY false
#if(USE_DISPLAY)
	spi::ili9341 dev(HSPI_HOST, (gpio_num_t)CONFIG_PIN_NUM_CS_DISPLAY, dcrs_gpio);//display device
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
#endif

    /*if(bImg)
        DrawDoomImg(&dev, (uint8_t*)img_doom, sizeof(img_doom));
        //DrawDoomImg(&dev);
    else
        ClearScreen(&dev);*/
    //bImg=!bImg;

}