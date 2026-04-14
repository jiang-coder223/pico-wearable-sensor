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
#include "semphr.h"

#define DEBUG_RAW 0
#define DEBUG_HR  0
#define DEBUG_SPO2 0
#define DEBUG_IMU 0
#define DEBUG_ALL 1

QueueHandle_t sensorQueue;
QueueHandle_t mqttQueue;
SemaphoreHandle_t i2cMutex;

typedef struct {
    uint32_t red;
    uint32_t ir;
    int bpm;
    int spo2;
} sensor_data_t;

void SensorTask(void *param);
void ImuTask(void *param);
void DisplayTask(void *param);
void NetworkTask(void *param);


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
    
    //解決資源互搶問題
    i2cMutex = xSemaphoreCreateMutex();

    // 建 queue
    sensorQueue = xQueueCreate(1, sizeof(sensor_data_t));
    mqttQueue = xQueueCreate(1, sizeof(sensor_data_t));

    // 建 task
    xTaskCreate(SensorTask, "Sensor", 2048, NULL, 3, NULL);
    xTaskCreate(ImuTask, "IMU", 1024, NULL, 1, NULL);
    xTaskCreate(NetworkTask, "NET", 4096, NULL, 2, NULL);
    xTaskCreate(DisplayTask, "Display", 1024, NULL, 1, NULL);

    // 啟動 RTOS
    vTaskStartScheduler();

    while (1); 
}

void SensorTask(void *param) {

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    max30102_init();
    max30102_setup();
    max30102_hr_init();
    max30102_spo2_init();
    mpu6050_init();
    mpu6050_calibrate();
    xSemaphoreGive(i2cMutex);

    TickType_t last = xTaskGetTickCount();

    /* static TickType_t prev = 0;
    static int count = 0;
    static int sum = 0; */

    while (1) {

        /* TickType_t now = xTaskGetTickCount();

        if (prev != 0) {
            int dt = now - prev;
            sum += dt;
            count++;

            if (count == 20) {
                printf("avg dt = %d ms\n", sum / 20);
                count = 0;
                sum = 0;
            }
        }
        prev = now; */

        sensor_data_t data;

        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        max30102_read_fifo(&data.red, &data.ir);
        xSemaphoreGive(i2cMutex);

        max30102_hr_update(data.red, data.ir);
        max30102_spo2_update(data.red, data.ir);

        data.bpm  = max30102_get_bpm();
        data.spo2 = max30102_get_spo2();

        xQueueOverwrite(sensorQueue, &data);
        xQueueOverwrite(mqttQueue, &data);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10)); // 100Hz
    }
}

void ImuTask(void *param) {
    while (1) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        mpu6050_get_activity_step();
        xSemaphoreGive(i2cMutex);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

void DisplayTask(void *param) {

    sensor_data_t data;
    int bpm = 0, spo2 = 0;
    int state;

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    SSD1306_init();
    SSD1306_clear();
    xSemaphoreGive(i2cMutex);

    while (1) {

        if (xQueueReceive(sensorQueue, &data, 0)) {
        bpm  = data.bpm;
        spo2 = data.spo2;
        }

        state = mpu6050_get_state();

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        display_update(bpm, spo2, state);

        xSemaphoreGive(i2cMutex);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void NetworkTask(void *param) {

    printf("NetworkTask start\n");

    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        while (1);
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting WiFi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASS,
            CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connect failed\n");
    } else {
        printf("WiFi connected\n");
    }
    mqtt_init();

    while (1) {
        cyw43_arch_poll();
        sensor_data_t data;

        if (xQueueReceive(mqttQueue, &data, 0)) {
            
            int state = mpu6050_get_state();

            mqtt_publish_data(data.bpm, data.spo2, state);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
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
