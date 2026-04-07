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
#define DEBUG_SPO2 0

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
        float R = max30102_get_R();
        float ratio_r = max30102_get_ratio_r();
        float ratio_i = max30102_get_ratio_i();

        uint32_t now = to_ms_since_boot(get_absolute_time());
        
         // MQTT 1 Hz (Every 1000ms)
        if (now - last_mqtt >= 1000) {
            if (bpm > 0 && spo2 > 0) {
                mqtt_publish_data(bpm, spo2);
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

            if (bpm > 0 || spo2 > 0) {
                printf("BPM: %d | Lag: %d | Conf: %.2f\n"
                        "SpO2: %d | R=%.3f ACr/DC=%.4f ACi/DC=%.4f\n",
                        bpm, lag, conf, spo2, R, ratio_r, ratio_i);
            } else {
                printf("No valid signal | Lag: %d | Conf: %.2f\n"
                        "R=%.3f ACr/DC=%.4f ACi/DC=%.4f\n",
                        lag, conf, R, ratio_r, ratio_i);
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