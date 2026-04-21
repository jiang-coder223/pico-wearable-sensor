#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// ===== 參數 =====
#define ADC_MAX 4095.0f
#define VREF    3.3f
#define DIVIDER 2.0f

// ===== 兩點校正 =====
#define GAIN    1.04f
#define OFFSET  0.0f

// ===== 狀態 =====
static float filtered_voltage = 0;
static int last_percent = -1;


// ===== 初始化 =====
void battery_init() {
    adc_init();
    adc_gpio_init(28);
    adc_select_input(2);
}

// ===== 讀電壓（單次）=====
float read_battery_voltage_raw() {

    sleep_us(50);
    adc_read();
    sleep_us(10);

    uint16_t raw = adc_read();

    float v_adc = raw * VREF / ADC_MAX;
    
    float v_bat = v_adc * DIVIDER;
    v_bat = v_bat * GAIN + OFFSET;

    return v_bat;
}

// ===== 平均 =====
float read_battery_voltage_avg() {
    float sum = 0;
    int samples = 10;

    for (int i = 0; i < samples; i++) {
        sum += read_battery_voltage_raw();
        sleep_ms(2);
    }

    return sum / samples;
}

// ===== 低通濾波 =====
float low_pass_filter(float v) {
    if (filtered_voltage == 0) {
        filtered_voltage = v;
    } else {
        filtered_voltage = 0.85f * filtered_voltage + 0.15f * v;
    }
    return filtered_voltage;
}

// ===== 電壓轉百分比（鋰電池曲線）=====
int voltage_to_percent(float v) {

    if (v >= 4.20f) return 100;
    if (v >= 4.10f) return 95;
    if (v >= 4.00f) return 90;
    if (v >= 3.95f) return 85;
    if (v >= 3.90f) return 80;
    if (v >= 3.85f) return 70;
    if (v >= 3.80f) return 60;
    if (v >= 3.75f) return 50;
    if (v >= 3.70f) return 40;
    if (v >= 3.65f) return 30;
    if (v >= 3.60f) return 20;
    if (v >= 3.55f) return 10;
    return 5;
}

// ===== 主讀取 API =====
void battery_update(float *voltage, int *percent) {

    float v = read_battery_voltage_avg();
    v = low_pass_filter(v);

    int p = voltage_to_percent(v);

    // ⭐ 抗跳動（避免%亂跳）
    if (last_percent != -1 && abs(p - last_percent) > 5) {
        p = last_percent;
    }

    last_percent = p;

    *voltage = v;
    *percent = p;
}