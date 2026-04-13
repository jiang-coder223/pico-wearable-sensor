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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define DEBUG_RAW 0
#define DEBUG_HR  0
#define DEBUG_SPO2 0
#define DEBUG_IMU 0
#define DEBUG_ALL 1

QueueHandle_t sensorQueue;

typedef struct {
    uint32_t red;
    uint32_t ir;
} sensor_data_t;

void SensorTask(void *param);
void ImuTask(void *param);
void DisplayTask(void *param);
void MqttTask(void *param);


int main() {

    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    sleep_ms(500);

    printf("Start RTOS System\n");

    // I2C init
    i2c_init(I2C0_PORT, 400 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    // init modules
    max30102_init();
    max30102_setup();
    max30102_hr_init();
    max30102_spo2_init();
    mpu6050_init();
    mpu6050_calibrate();
    SSD1306_init();
    SSD1306_clear();
    mqtt_init();

    // 建 queue
    sensorQueue = xQueueCreate(1, sizeof(sensor_data_t));

    // 建 task
    xTaskCreate(SensorTask, "Sensor", 2048, NULL, 3, NULL);
    xTaskCreate(ImuTask, "IMU", 1024, NULL, 2, NULL);
    xTaskCreate(MqttTask, "MQTT", 2048, NULL, 1, NULL);
    xTaskCreate(DisplayTask, "Display", 1024, NULL, 1, NULL);

    // 啟動 RTOS
    vTaskStartScheduler();

    while (1); 
}

void SensorTask(void *param) {
    TickType_t last = xTaskGetTickCount();

    while (1) {
        sensor_data_t data;

        max30102_read_fifo(&data.red, &data.ir);

        max30102_hr_update(data.red, data.ir);
        max30102_spo2_update(data.red, data.ir);

        xQueueOverwrite(sensorQueue, &data);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10)); // 100Hz
    }
}

void ImuTask(void *param) {
    while (1) {
        mpu6050_get_activity_step();
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

void DisplayTask(void *param) {
    while (1) {

        int bpm = max30102_get_bpm();
        int spo2 = max30102_get_spo2();
        int state = mpu6050_get_state();

        display_update(bpm, spo2, state);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void MqttTask(void *param) {
    sensor_data_t data;
    TickType_t lastSend = 0;

    while (1) {
        mqtt_loop();

        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY)) {

            TickType_t now = xTaskGetTickCount();

            if (now - lastSend >= pdMS_TO_TICKS(1000)) {

                int bpm = max30102_get_bpm();
                int spo2 = max30102_get_spo2();
                int state = mpu6050_get_state();

                mqtt_publish_data(bpm, spo2, state);

                lastSend = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("Stack overflow: %s\n", pcTaskName);
    while (1);
}

void vApplicationMallocFailedHook(void) {
    printf("Malloc failed\n");
    while (1);
}