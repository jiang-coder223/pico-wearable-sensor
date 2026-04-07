#ifndef SSD1306_H
#define SSD1306_H

// I2C address
#define SSD1306_ADDR 0x3C

// register


extern uint8_t buffer[1024];

// function
void ssd1306_init();
void ssd1306_clear();
void ssd1306_update();
void draw_char(int x, int y, char c);

#endif