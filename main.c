#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU6050.h"
#include "MAX30102.h"
#include "SSD1306.h"
#include "global_defines.h"
#include "MQTT.h"

#define DEBUG_RAW 0
#define DEBUG_HR  0
#define DEBUG_SPO2 1

/* int main() {
    stdio_init_all();

    // I2C 初始化
    i2c_init(I2C0_PORT, 400 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    // MPU6050 初始化與校準
    mpu6050_init();
    sleep_ms(1000);
    mpu6050_calibrate();

    while (true) {
        int total_activity_sum = 0;
        for (int i = 0; i < 50; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu6050_read_accel(&ax, &ay, &az);
            mpu6050_read_gyro(&gx, &gy, &gz);

            // 扣除 offset
            ax -= ax_offset; ay -= ay_offset; az -= az_offset;
            gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

            int acc_act = abs(ax) + abs(ay) + abs(az);
            int gyro_act = abs(gx) + abs(gy) + abs(gz);

            if (acc_act < 500) acc_act = 0;
            if (gyro_act < 300) gyro_act = 0;

            total_activity_sum += (acc_act + (gyro_act / 4));
            sleep_ms(20);
        }

        printf("SUM=%d -> ", total_activity_sum);
        if (total_activity_sum < 50000) printf("STATIC\n");
        else printf("ACTIVE\n");
    }
} */

/* int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }

    printf("Start Autocorrelation Heart Rate Monitor\n");

    i2c_init(I2C0_PORT, 400 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    max30102_init();
    max30102_setup();

    // 變數初始化
    float data_buffer[WINDOW_SIZE] = {0};
    int data_idx = 0;
    int averaged_bpm = 0;
    float lp = 0;
    uint32_t last_print_time = 0;

    while (true) {
        uint32_t red, ir;
        max30102_read_fifo(&red, &ir);
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (red < 80000) { 
            // 沒放手指：重置所有狀態
            data_idx = 0;
            averaged_bpm = 0;
            lp = 0; 
            
            // 限制印出頻率：每 500ms 報一次，避免洗板
            if (now - last_print_time >= 500) {
                printf("Status: [ - ] Waiting for finger... (RED: %u)\n", red);
                last_print_time = now;
            }
        } 
        else {
            // --- 有放手指：進入正常運算邏輯 ---
            
            //進入DSP 1. 預處理：去直流
            if (lp == 0) lp = (float)red; 
            lp = (lp * 0.96f) + (red * 0.04f); 
            float ac_val = (float)red - lp;

            // 2. 存入緩衝區
            data_buffer[data_idx++] = ac_val;

            // 3. 緩衝區滿了才計算 (2.56秒)
            if (data_idx >= WINDOW_SIZE) {
                float max_correlation = -1.0f;
                int best_lag = -1;
                float zero_lag_corr = 0;

                for (int i = 0; i < WINDOW_SIZE; i++) 
                    zero_lag_corr += data_buffer[i] * data_buffer[i];

                for (int lag = MIN_LAG; lag <= MAX_LAG; lag++) {
                    float correlation = 0;
                    for (int i = 0; i < (WINDOW_SIZE - lag); i++) {
                        correlation += data_buffer[i] * data_buffer[i + lag];
                    }
                    if (correlation > max_correlation) {
                        max_correlation = correlation;
                        best_lag = lag;
                    }
                }

                // 4. 判定與顯示
                if (best_lag > 0 && max_correlation > (zero_lag_corr * 0.3)) { 
                    int current_bpm = (SAMPLE_RATE * 60) / best_lag;
                    
                    if (averaged_bpm == 0) averaged_bpm = current_bpm;
                    else averaged_bpm = (averaged_bpm * 0.6f) + (current_bpm * 0.4f);
                    
                    printf("BPM: %d | Confidence: High | Lag: %d\n", averaged_bpm, best_lag);
                } 
                else {
                    // 分析中狀態也限制印出頻率
                    if (now - last_print_time >= 500) {
                        printf("Status: [ ? ] Analyzing... Keep still\n");
                        last_print_time = now;
                    }
                }

                // 5. 滑動窗口
                for (int i = 0; i < 128; i++) {
                    data_buffer[i] = data_buffer[i + 128];
                }
                data_idx = 128; 
            }
        }
        
        // 核心：維持 100Hz 穩定採樣，不要隨便改這個 sleep
        sleep_ms(10); 
    }
} */

int main() {
    // init all
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    sleep_ms(500);

    printf("Start Autocorrelation Heart Rate Monitor\n");

    i2c_init(I2C0_PORT, 400 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    max30102_init();
    max30102_setup();
    max30102_hr_init();
    max30102_spo2_init();
    mqtt_init();

    uint32_t last_print = 0;
    uint32_t last_mqtt = 0;

    while (1) {
        mqtt_loop();

        uint32_t red, ir;
        max30102_read_fifo(&red, &ir);

        max30102_hr_update(red, ir);
        max30102_spo2_update(red, ir);

        int bpm = max30102_get_bpm();
        int spo2 = max30102_get_spo2();

        uint32_t now = to_ms_since_boot(get_absolute_time());
        
         // MQTT 1 Hz (Every 1000ms)
        if (now - last_mqtt >= 1000) {

            if (bpm > 0) {
                mqtt_publish_bpm(bpm);
            }

            last_mqtt = now;
        }

        #if DEBUG_RAW
            if (now - last_print >= 1000) {
            printf("RAW RED=%u IR=%u\n", red, ir);
            last_print = now;
        }
        #endif

        #if DEBUG_HR
            if (now - last_print >= 1000) {
            printf("BPM=%d\n", bpm);
            last_print = now;
        }
        #endif

        #if DEBUG_SPO2
            if (now - last_print >= 1000) {
            printf("SpO2=%d\n", spo2);
            last_print = now;
        }
        #endif

        if (now - last_print >= 1000) {

            int lag = max30102_get_lag();
            float conf = max30102_get_confidence();

            if (bpm > 0) {
                printf("BPM: %d | Lag: %d | Conf: %.2f\n",
                    bpm, lag, conf);
            } else {
                printf("No valid signal | Lag: %d | Conf: %.2f\n",
                    lag, conf);
            }
            last_print = now;
        }

        

        sleep_ms(10);
    }
}

/* int main() {
    stdio_init_all();

    i2c_init(I2C0_PORT, 400 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    ssd1306_init();
    ssd1306_clear();

    for (int i = 0; i < 1024; i++) {
        buffer[i] = 0xFF;
    }


    ssd1306_update();

    while (1) {
        sleep_ms(1000);
    }
} */