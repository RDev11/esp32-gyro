#include "stdafx.h"
#include "text2.h"

#include "SPI.h"




void ili9341_draw_char(spi::ili9341 *lcd, ili9341_text_attr_t attr, char ch)
{
    // verify we have something within screen dimensions to be drawn
    int16_t  _x = attr.origin_x;
    int16_t  _y = attr.origin_y;
    uint16_t _w = attr.font->width;
    uint16_t _h = attr.font->height;
    //if (ibNOT(ili9341_clip_rect(lcd, &_x, &_y, &_w, &_h)))
    //  { return; }

    // 16-bit color, so need 2 bytes for each pixel being filled
    uint32_t num_pixels = attr.font->width * attr.font->height;
    uint32_t rect_wc    = num_pixels;

    uint16_t fg_le = attr.fg_color;
    uint16_t bg_le = attr.bg_color;

    uint32_t block_wc = rect_wc;
    /*
    if (block_wc > __SPI_TX_BLOCK_MAX__)[]
    { block_wc = __SPI_TX_BLOCK_MAX__; }*/
    uint16_t buffer[16*26];

    // initialize the buffer with glyph from selected font
    uint8_t ch_index = glyph_index(ch);
    for (uint32_t yi = 0; yi < attr.font->height; ++yi) {
        uint32_t gl = (uint32_t)attr.font->glyph[ch_index * attr.font->height + yi];
        for (uint32_t xi = 0; xi < attr.font->width; ++xi) {
            if ((gl << xi) & 0x8000)
            { buffer[yi * attr.font->width + xi] = fg_le; }
            else
            { buffer[yi * attr.font->width + xi] = bg_le; }
        }
    }

    // select target region
    lcd->setWindow(
        attr.origin_x,  attr.origin_x + attr.font->width - 1,
        attr.origin_y, attr.origin_y + attr.font->height - 1);

    lcd->transfer_cmd(0x2C, (uint8_t*)&buffer[0], attr.font->height*attr.font->width*2);
  
}

void ili9341_draw_string(spi::ili9341 *lcd, ili9341_text_attr_t attr, const std::string& str)
{
    int16_t curr_x = attr.origin_x;
    int16_t curr_y = attr.origin_y;
    int16_t start_x = attr.origin_x;
    int idx = 0;
    while ('\0' != str[idx]) {
        if('\r' == str[idx])
        {
            curr_x = start_x;
        } 
        else if('\n' == str[idx])
        {
            curr_y += attr.font->height;
            curr_x = start_x;
        }
        else
        {
            if ( (curr_x > lcd->getScreenWidth()) ||
                (curr_y > lcd->getScreenHeight()) )
            { break; }

            attr.origin_x = curr_x;
            attr.origin_y = curr_y;

            ili9341_draw_char(lcd, attr, str[idx]);

            curr_x += attr.font->width;
        }
        ++idx;
    }
}
