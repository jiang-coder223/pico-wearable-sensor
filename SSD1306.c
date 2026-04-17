#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "SSD1306.h"
#include "global_defines.h"
#include "string.h"


uint8_t buffer[1024]; // 128x64

void SSD1306_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(I2C0_PORT, SSD1306_ADDR, buf, 2, false);
}

void SSD1306_data(uint8_t* data, size_t len) {
    uint8_t temp[1025];
    temp[0] = 0x40;
    memcpy(&temp[1], data, len);
    i2c_write_blocking(I2C0_PORT, SSD1306_ADDR, temp, len + 1, false);
}

void SSD1306_init() {
    sleep_ms(100);

    SSD1306_cmd(0xAE);
    SSD1306_cmd(0x20); SSD1306_cmd(0x00);
    SSD1306_cmd(0xB0);
    SSD1306_cmd(0xC8);
    SSD1306_cmd(0x00);
    SSD1306_cmd(0x10);
    SSD1306_cmd(0x40);
    SSD1306_cmd(0x81); SSD1306_cmd(0xFF);
    SSD1306_cmd(0xA1);
    SSD1306_cmd(0xA6);
    SSD1306_cmd(0xA8); SSD1306_cmd(0x3F);
    SSD1306_cmd(0xA4);
    SSD1306_cmd(0xD3); SSD1306_cmd(0x00);
    SSD1306_cmd(0xD5); SSD1306_cmd(0xF0);
    SSD1306_cmd(0xD9); SSD1306_cmd(0x22);
    SSD1306_cmd(0xDA); SSD1306_cmd(0x12);
    SSD1306_cmd(0xDB); SSD1306_cmd(0x20);
    SSD1306_cmd(0x8D); SSD1306_cmd(0x14);
    SSD1306_cmd(0xAF);
}

void SSD1306_clear() {
    memset(buffer, 0x00, sizeof(buffer));
}

void SSD1306_update() {
    for (int page = 0; page < 8; page++) {
        SSD1306_cmd(0xB0 + page);
        SSD1306_cmd(0x00);
        SSD1306_cmd(0x10);
        SSD1306_data(&buffer[page * 128], 128);
    }
}

// 5x7 ASCII font (只放常用字)
static const uint8_t font5x7_full[][5] = {
    // SPACE (32)
    {0x00,0x00,0x00,0x00,0x00},

    // '0' - '9' (48–57)
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9

    // ':' (58)
    {0x00,0x36,0x36,0x00,0x00},

    // '%' (37)
    {0x62,0x64,0x08,0x13,0x23},

    // 'A' - 'Z' (65–90)
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x04,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
};

void SSD1306_draw_pixel(int x, int y, int color) {
    if (x < 0 || x >= 128 || y < 0 || y >= 64) return;

    if (color)
        buffer[x + (y / 8) * 128] |= (1 << (y % 8));
    else
        buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
}

void SSD1306_draw_char(int x, int y, char c) {

    const uint8_t *bitmap = NULL;

    if (c == ' ') {
        bitmap = font5x7_full[0];
    }
    else if (c >= '0' && c <= '9') {
        bitmap = font5x7_full[1 + (c - '0')];
    }
    else if (c == ':') {
        bitmap = font5x7_full[11];
    }
    else if (c == '%') {
        bitmap = font5x7_full[12];
    }
    else if (c >= 'A' && c <= 'Z') {
        bitmap = font5x7_full[13 + (c - 'A')];
    }
    else {
        return;
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 7; j++) {
            if (bitmap[i] & (1 << j))
                SSD1306_draw_pixel(x + i, y + j, 1);
            else
                SSD1306_draw_pixel(x + i, y + j, 0);
        }
    }
}

void display_update(int bpm, int spo2, int state, int trend, int steps) {
    SSD1306_clear();

    char line1[20];
    char line2[20];
    char line3[20];
    char line4[20];

    const char* trend_str[] = {
        "STABLE",
        "UP",
        "DOWN"
    };
    const char* state_str[] = {
        "REST",
        "MOVE",
        "ACTIVE"
    };

    if (state < 0 || state > 2) state = 0;
    if (trend < 0 || trend > 2) trend = 0;  // 防呆

    snprintf(line1, sizeof(line1), "BPM: %d", bpm);
    snprintf(line2, sizeof(line2), "SPO2: %d%%", spo2);
    snprintf(line3, sizeof(line3), "ST:%s %d", state_str[state], steps);
    snprintf(line4, sizeof(line4), "TREND: %s", trend_str[trend]);

    SSD1306_draw_string(0, 0, line1);
    SSD1306_draw_string(0, 16, line2);
    SSD1306_draw_string(0, 32, line3);
    SSD1306_draw_string(0, 48, line4);

    SSD1306_update();
}

void SSD1306_draw_string(int x, int y, const char *str) {
    while (*str) {
        SSD1306_draw_char(x, y, *str);
        x += 6;  // 字距
        str++;
    }
}
