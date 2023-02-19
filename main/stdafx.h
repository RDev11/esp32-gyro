#pragma once
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sstream>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"



//#include "driver/spi_master.h"
//#define min(a,b) ((a)<(b)?(a):(b)) //min,max объявляются в <string>
//#define max(a,b) ((a)>(b)?(a):(b))

inline void swapbytes (uint8_t* buf, uint32_t size)//
{
	ESP_LOGI(__FUNCTION__, "IN");
	uint8_t t;
	for(uint32_t i=0;i<size-1;i+=2)//SPI_SWAP_DATA_TX can help
	{
		t = buf[i];
		buf[i] = buf[i+1];
		buf[i+1] = t;
	}
	ESP_LOGI(__FUNCTION__, "OUT");
}
inline uint16_t swapbytes (const uint16_t& v)//
{
	return (v>>8) | (v<<8);
}

#define HIGH_BYTE(word) ((uint8_t)(word>>8))
#define LOW_BYTE(word) ((uint8_t)(word&0xFF))


inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b)//3 байта сжимает в 2
{
	return swapbytes((((uint16_t)r & 0xF8)<<8) | (((uint16_t)g & 0xFC)<<3) | (((uint16_t)b)>>3));
}
inline uint16_t RGB(uint32_t rgb)//3 байта сжимает в 2
{
	return swapbytes((rgb>>16 & 0xF8)<<8 | (rgb>>8 & 0xFC)<<3 | (rgb&0xF8)>>3);
}

//workaround, to_string(float) и подобные функции по какой-то причине выдают ошибку
inline std::string float_to_str(double v, int intwidth, int prec){
	/*char strbuf[1024];
	char fmtbuf[50];
	snprintf(fmtbuf, 50, "%%0%d.%df", intwidth, prec);//ex: "%2d.%4d"
	
	snprintf(strbuf, 1023, fmtbuf, v);
	return std::string(strbuf);*/
	bool positive = v>0;
	int intpart = std::abs(v);
	auto _floatpart = std::abs(v)-std::floor(std::abs(v));
	//auto _floatpart = std::abs(v)-(int)(std::abs(v));
	for(int i =0;i<prec;i++){
		_floatpart = _floatpart*10.0;
	}
	int floatpart = _floatpart;
	//ESP_LOGI(__FUNCTION__, "%hhx%hhx%hhx%hhx=%d.%d", *((char*)&v+3),*((char*)&v+2),*((char*)&v+1),*((char*)&v+0), intpart, floatpart);

	char result[50];
	char fmtbuf[20];
	snprintf(fmtbuf, 20, "%c%%0.%dd.%%0.%dd", positive?'+':'-', intwidth, prec);
	//ESP_LOGI(__FUNCTION__, "fmt:\"%s\"", fmtbuf);
	snprintf(result, 50, fmtbuf, intpart, floatpart);
	return result;
}

/*
// convert value at addr to little-endian (16-bit)
#define __LEu16(addr)                                      \
    ( ( (((uint16_t)(*(((uint8_t *)(addr)) + 1)))      ) | \
        (((uint16_t)(*(((uint8_t *)(addr)) + 0))) << 8U) ) )
*/
