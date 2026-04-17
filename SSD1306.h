#ifndef SSD1306_H
#define SSD1306_H

// I2C address
#define SSD1306_ADDR 0x3C

extern uint8_t buffer[1024];

// function
void SSD1306_init();
void SSD1306_clear();
void SSD1306_update();
void SSD1306_draw_pixel(int x, int y, int color);
void SSD1306_draw_char(int x, int y, char c);
void display_update(int bpm, int spo2, int state, int trend);
void SSD1306_draw_string(int x, int y, const char *str);

#endif