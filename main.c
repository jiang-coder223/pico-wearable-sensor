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
#define DEBUG_IMU 0
#define DEBUG_ALL 1

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
    mpu6050_init();
    mpu6050_calibrate();
    SSD1306_init();
    SSD1306_clear();
    mqtt_init();

    uint32_t last_print = 0;
    uint32_t last_mqtt = 0;
    uint32_t last_oled = 0;
    static int no_finger_count = 0;

    SSD1306_update();

    while (1) {
        mqtt_loop();
        mpu6050_get_activity_step();

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
        
        if (ratio_i < 0.00003f) {
            no_finger_count++;
        } else {
            no_finger_count = 0;
        }

        if (no_finger_count > 200) {
            bpm = 0;
            spo2 = 0;
        }

            int activity = mpu6050_get_activity();
            int state    = mpu6050_get_state();

         // MQTT 1 Hz (Every 1000ms)
        if (now - last_mqtt >= 1000) {

            static int last_bpm = 0;
            static int last_spo2 = 0;
            static int last_state = -1;

            int changed = 0;

            // HR / SpO2
            if (bpm > 0 && spo2 > 0) {
                if (abs(bpm - last_bpm) >= 1 || abs(spo2 - last_spo2) >= 1) {
                    changed = 1;
                    last_bpm = bpm;
                    last_spo2 = spo2;
                }
            }

            if (state != last_state) {
                changed = 1;
                last_state = state;
            }

            if (changed) {
                mqtt_publish_data(bpm, spo2, state);
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

        #if DEBUG_IMU
            if (now - last_print >= 1000) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu6050_read_accel(&ax, &ay, &az);
            mpu6050_read_gyro(&gx, &gy, &gz);

            printf("IMU: ACT=%d | STATE=%s\n"
                "ACC: %d %d %d | GYRO: %d %d %d\n",
                activity,
                (state == 0) ? "STATIC" : "ACTIVE",
                ax, ay, az, gx, gy, gz);
            }
        #endif

        #if DEBUG_ALL
        if (now - last_print >= 1000) {

            int lag = max30102_get_lag();
            float conf = max30102_get_confidence();

            const char *state_str = (state == 0) ? "STATIC" : "ACTIVE";

            if (bpm > 0 || spo2 > 0) {
                printf("State: %s \n"
                    "BPM: %d | Lag: %d | Conf: %.2f \n"
                    "SpO2: %d | R=%.3f ACr/DC=%.4f ACi/DC=%.4f\n",
                    state_str, 
                    bpm, lag, conf,
                    spo2, R, ratio_r, ratio_i);
            } else {
                printf("No valid signal\n"
                    "BPM: %d | Lag: %d | Conf: %.2f \n"
                    "SpO2: %d | R=%.3f ACr/DC=%.4f ACi/DC=%.4f\n", 
                    bpm, lag, conf,
                    spo2, R, ratio_r, ratio_i);
            }

            last_print = now;
        }
        #endif

        if (now - last_oled >= 500) {
            display_update(bpm, spo2, state);
            last_oled = now;
        }
        

        sleep_ms(10);
    }
}

