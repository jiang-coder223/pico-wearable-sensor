#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include "global_defines.h"

static int last_activity = 0;
static int state = 0;
static int16_t prev_ax = 0, prev_ay = 0, prev_az = 0;

// 定義變數實體
int32_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int32_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

void mpu6050_init(void) {
    uint8_t buf[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C0_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buf[6];
    i2c_write_blocking(I2C0_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, MPU6050_ADDR, buf, 6, false);
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t reg = MPU6050_REG_GYRO_XOUT_H;
    uint8_t buf[6];
    i2c_write_blocking(I2C0_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, MPU6050_ADDR, buf, 6, false);
    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

void mpu6050_calibrate(void) {
    int32_t ax_s = 0, ay_s = 0, az_s = 0;
    int32_t gx_s = 0, gy_s = 0, gz_s = 0;
    printf("Calibrating... Keep device still\n");
    for (int i = 0; i < 100; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050_read_accel(&ax, &ay, &az);
        mpu6050_read_gyro(&gx, &gy, &gz);
        ax_s += ax; ay_s += ay; az_s += az;
        gx_s += gx; gy_s += gy; gz_s += gz;
        sleep_ms(10);
    }
    ax_offset = ax_s / 100;
    ay_offset = ay_s / 100;
    az_offset = (az_s / 100) - 16384; // 扣除 Z 軸重力
    gx_offset = gx_s / 100;
    gy_offset = gy_s / 100;
    gz_offset = gz_s / 100;
    printf("Calibrate Done!\n");
}

int mpu6050_get_activity_step(void) {

    static uint32_t last_sample_time = 0;
    static int active_count = 0;
    static int static_count = 0;

    //  sliding window + EMA
    static int activity = 0;

    uint32_t now = to_ms_since_boot(get_absolute_time());

    last_sample_time = now;

    int16_t ax, ay, az, gx, gy, gz;

    mpu6050_read_accel(&ax, &ay, &az);
    mpu6050_read_gyro(&gx, &gy, &gz);

    ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

    //  核心：瞬時能量
    int acc = abs(ax - prev_ax) + abs(ay - prev_ay) + abs(az - prev_az);

    prev_ax = ax;
    prev_ay = ay;
    prev_az = az;
    int gyro = abs(gx) + abs(gy) + abs(gz);

    //  去噪
    if (acc < 300) acc = 0;
    if (gyro < 200) gyro = 0;

    int instant = acc + gyro / 4;

    // EMA

    if (instant > activity)
        activity = (activity * 8 + instant * 2) / 10;
    else
        activity = (activity * 5 + instant * 5) / 10;  // 快速下降

    last_activity = activity;

    //  hysteresis
        if (activity > 600) {
            active_count++;
            static_count = 0;
        } 
        else if (activity < 200) {
            static_count++;
            active_count = 0;
        }

        // 狀態切換（防抖）
        if (state == 0 && active_count > 5)
            state = 1;

        if (state == 1 && static_count > 10)
            state = 0;


    return activity;
}

int mpu6050_get_state(void) {
    return state;
}

int mpu6050_get_activity(void) {
    return last_activity;
}

void mpu6050_reset(void) {
    last_activity = 0;
    state = 0;
    prev_ax = prev_ay = prev_az = 0;
}