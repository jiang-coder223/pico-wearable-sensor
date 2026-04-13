#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "hardware/i2c.h"

// I2C 共同配置
#define I2C0_PORT i2c0
#define I2C0_SDA 8
#define I2C0_SCL 9

// 如果之後有 LED 或按鈕，也寫在這裡
#define HEART_RATE_LED_PIN 25 
#endif