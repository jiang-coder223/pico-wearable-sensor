#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "SSD1306.h"
#include "global_defines.h"
#include "string.h"


uint8_t buffer[1024]; // 128x64

void ssd1306_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(I2C0_PORT, SSD1306_ADDR, buf, 2, false);
}

void ssd1306_data(uint8_t* data, size_t len) {
    uint8_t temp[1025];
    temp[0] = 0x40;
    memcpy(&temp[1], data, len);
    i2c_write_blocking(I2C0_PORT, SSD1306_ADDR, temp, len + 1, false);
}

void ssd1306_init() {
    sleep_ms(100);

    ssd1306_cmd(0xAE);
    ssd1306_cmd(0x20); ssd1306_cmd(0x00);
    ssd1306_cmd(0xB0);
    ssd1306_cmd(0xC8);
    ssd1306_cmd(0x00);
    ssd1306_cmd(0x10);
    ssd1306_cmd(0x40);
    ssd1306_cmd(0x81); ssd1306_cmd(0xFF);
    ssd1306_cmd(0xA1);
    ssd1306_cmd(0xA6);
    ssd1306_cmd(0xA8); ssd1306_cmd(0x3F);
    ssd1306_cmd(0xA4);
    ssd1306_cmd(0xD3); ssd1306_cmd(0x00);
    ssd1306_cmd(0xD5); ssd1306_cmd(0xF0);
    ssd1306_cmd(0xD9); ssd1306_cmd(0x22);
    ssd1306_cmd(0xDA); ssd1306_cmd(0x12);
    ssd1306_cmd(0xDB); ssd1306_cmd(0x20);
    ssd1306_cmd(0x8D); ssd1306_cmd(0x14);
    ssd1306_cmd(0xAF);
}

void ssd1306_clear() {
    memset(buffer, 0x00, sizeof(buffer));
}

void ssd1306_update() {
    for (int page = 0; page < 8; page++) {
        ssd1306_cmd(0xB0 + page);
        ssd1306_cmd(0x00);
        ssd1306_cmd(0x10);
        ssd1306_data(&buffer[page * 128], 128);
    }
}

const uint8_t font[][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
};

void draw_char(int x, int page, char c) {
    if (c < 'A' || c > 'G') return;
    int index = c - 'A';

    for (int i = 0; i < 5; i++) {
        buffer[page * 128 + x + i] = font[index][i];
    }
}