#pragma once 
#include "ili9341_font.h"

namespace spi{
  class ili9341;
}

struct ili9341_text_attr_t {
  ili9341_font_t const *font;
  uint16_t fg_color;
  uint16_t bg_color;
  uint16_t origin_x;
  uint16_t origin_y;
};



void ili9341_draw_char(spi::ili9341 *lcd, ili9341_text_attr_t attr, char ch);

void ili9341_draw_string(spi::ili9341 *lcd, ili9341_text_attr_t attr, const std::string& str);