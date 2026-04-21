#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <math.h>
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "MPU6050.h"
#include "MAX30102.h"
#include "SSD1306.h"
#include "global_defines.h"
#include "MQTT.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bettery.h"



#define DEBUG_RAW 0
#define DEBUG_HR  0
#define DEBUG_SPO2 0
#define DEBUG_IMU 0
#define DEBUG_ALL 0

QueueHandle_t sensorQueue;
QueueHandle_t batteryQueue;
QueueHandle_t mqttQueue;
QueueHandle_t sampleQueue;
SemaphoreHandle_t i2cMutex;


typedef struct {
    uint32_t red;
    uint32_t ir;
    int bpm;
    int spo2;
    int state;
    int hr_trend;
    int steps;
    float battery_v;   
    int   battery_p;
} sensor_data_t;

typedef struct {
    float v;
    int p;
} battery_data_t;

typedef struct {
    uint32_t red;
    uint32_t ir;
    uint32_t t;   // timestamp（之後用）
} sample_t;

typedef enum {
    HR_STABLE = 0,
    HR_RISING,
    HR_FALLING
} hr_trend_t;

void DisplayTask(void *param);
void NetworkTask(void *param);
void ReadTask(void *param);
void ProcessTask(void *param);
void BatteryTask(void *param);
hr_trend_t calc_hr_trend(int current, int prev);

    int main() {

        stdio_init_all();
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

        max30102_init();
        max30102_setup();
        max30102_hr_init();
        max30102_spo2_init();

        SSD1306_init();
        SSD1306_clear();

        mpu6050_init();
        mpu6050_calibrate();

        printf("ALL INIT DONE\n");

        // 建 queue
        sensorQueue = xQueueCreate(1, sizeof(sensor_data_t));
        batteryQueue = xQueueCreate(1, sizeof(battery_data_t));
        mqttQueue = xQueueCreate(1, sizeof(sensor_data_t));
        sampleQueue = xQueueCreate(128, sizeof(sample_t));

        // 建 task
        xTaskCreate(ReadTask,    "read",    1024, NULL, 2, NULL);
        xTaskCreate(ProcessTask, "proc",    2048, NULL, 3, NULL);
        /* xTaskCreate(NetworkTask, "NET",     2048, NULL, 1, NULL); */
        xTaskCreate(DisplayTask, "Display", 512, NULL, 1, NULL);
        xTaskCreate(BatteryTask, "BAT", 1024, NULL, 1, NULL);

        // 啟動 RTOS
        vTaskStartScheduler();

        while (1); 
    }


void DisplayTask(void *param)
{
    sensor_data_t data;
    int bpm = 0, spo2 = 0;
    int trend = 0;
    int state = 0;
    int steps = 0;
    int battery = 0;


    while (1)
    {
        if (xQueueReceive(sensorQueue, &data, 0)) {
            bpm   = data.bpm;
            spo2  = data.spo2;
            trend = data.hr_trend;
            state = data.state;    
            steps = data.steps; 
            battery = data.battery_p;
        }

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        display_update(bpm, spo2, state, trend, steps, battery);

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

    sensor_data_t data;
    static sensor_data_t last_data = {0};

    int counter = 0;

    while (1) {

        cyw43_arch_poll();

        // ⭐ 非阻塞取資料
        if (xQueueReceive(mqttQueue, &data, 0)) {
            last_data = data;
        }

        // ⭐ 固定輸出
        mqtt_publish_data(
            last_data.bpm,
            last_data.spo2,
            last_data.state,
            last_data.hr_trend,
            last_data.battery_p
        );

        counter++;
        if (counter >= 25) {
            mqtt_publish_heartbeat();
            counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void ReadTask(void *param)
{
    sample_t s;

    TickType_t lastWake = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10)); // 100Hz

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        if (max30102_read_fifo(&s.red, &s.ir))
        {
            s.t = time_us_64();

            // 保證不丟資料（重要）
            xQueueSend(sampleQueue, &s, portMAX_DELAY);
        }

        xSemaphoreGive(i2cMutex);
    }
}

void ProcessTask(void *param)
{
    sample_t s;
    sensor_data_t data = {0};
    battery_data_t bat;

    TickType_t lastWake = xTaskGetTickCount();

    static int prev_bpm = -1;
    static int current_trend = HR_STABLE;

    while (1)
    {
        // ✅ 固定 100Hz（核心）
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

        // =====================================================
        // 🔴 1. IMU（永遠跑，不管 HR）
        // =====================================================
        xSemaphoreTake(i2cMutex, portMAX_DELAY);   // 🔴 一定要拿到
        mpu6050_update();
        xSemaphoreGive(i2cMutex);

        mpu6050_activity_update();
        mpu6050_step_counter_update();

        data.state = mpu6050_get_state();
        data.steps = mpu6050_get_steps();

        // =====================================================
        // 🔴 2. HR（有資料才更新）
        // =====================================================
        if (xQueueReceive(sampleQueue, &s, 0))
        {
            max30102_hr_update(s.red, s.ir);
            max30102_spo2_update(s.red, s.ir);

            int bpm  = max30102_get_bpm();
            int spo2 = max30102_get_spo2();

            if (!max30102_has_signal()) {
                prev_bpm = -1;
                current_trend = HR_STABLE;
            } else {
                if (prev_bpm == -1) prev_bpm = bpm;

                if (bpm != prev_bpm)
                    current_trend = calc_hr_trend(bpm, prev_bpm);

                prev_bpm = bpm;
            }

            data.red  = s.red;
            data.ir   = s.ir;
            data.bpm  = bpm;
            data.spo2 = spo2;
            data.hr_trend = current_trend;
        }

        // =====================================================
        // 🔴 3. battery
        // =====================================================
        static battery_data_t last_bat = {0};

        if (xQueueReceive(batteryQueue, &bat, 0)) {
            last_bat = bat;   // ⭐ 關鍵：更新 cache
        }

        data.battery_v = last_bat.v;
        data.battery_p = last_bat.p;

        // =====================================================
        // 🔴 4. output（固定節奏）
        // =====================================================
        xQueueOverwrite(sensorQueue, &data);
        xQueueOverwrite(mqttQueue, &data);
    }
}

void BatteryTask(void *param) {

    float v;
    int p;

    battery_data_t bat;

    battery_init();

    while (1) {

        battery_update(&v, &p);

        bat.v = v;
        bat.p = p;

        // ✅ 寫自己的 queue
        xQueueOverwrite(batteryQueue, &bat);

        printf("[BAT] %.2f V (%d%%)\n", v, p);

        vTaskDelay(pdMS_TO_TICKS(2000));
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

hr_trend_t calc_hr_trend(int current, int prev) {
    int diff = current - prev;

    if (diff > 0) return HR_RISING;     
    if (diff < 0) return HR_FALLING;
    return HR_STABLE;
}