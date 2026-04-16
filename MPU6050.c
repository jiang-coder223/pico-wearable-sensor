#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include "global_defines.h"
#include "math.h"

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

    for (int i = 0; i < 200; i++) {

        int16_t ax, ay, az, gx, gy, gz;

        mpu6050_read_accel(&ax, &ay, &az);
        mpu6050_read_gyro(&gx, &gy, &gz);

        ax_s += ax;
        ay_s += ay;
        az_s += az;

        gx_s += gx;
        gy_s += gy;
        gz_s += gz;

        sleep_ms(5);
    }

    ax_offset = ax_s / 200;
    ay_offset = ay_s / 200;
    az_offset = az_s / 200;

    gx_offset = gx_s / 200;
    gy_offset = gy_s / 200;
    gz_offset = gz_s / 200;

    printf("Offset: ax=%d ay=%d az=%d\n", ax_offset, ay_offset, az_offset);
    printf("Calibrate Done!\n");
}

int mpu6050_get_activity_step(void) {

    static float energy = 0.0f;
    static float baseline = 0.02f;   // 初始靜止噪聲
    static int state = 0;            // 0=REST,1=MOVE,2=ACTIVE

    int16_t ax, ay, az;
    mpu6050_read_accel(&ax, &ay, &az);

    // ===== normalize =====
    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    // ===== magnitude =====
    float mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

    // ===== dynamic（去重力）=====
    float dynamic = fabsf(mag - 1.0f);

    // ===== energy（EMA）=====
    energy = 0.8f * energy + 0.2f * dynamic;

    // ===== baseline（只在靜止時更新）=====
    if (energy < 0.05f) {
        baseline = 0.99f * baseline + 0.01f * energy;
    }

    // ===== 自適應 threshold =====
    float th_move   = baseline * 3.0f;
    float th_active = baseline * 8.0f;

    // ===== hysteresis（避免抖動）=====
    switch (state) {

        case 0: // REST
            if (energy > th_active)
                state = 2;
            else if (energy > th_move)
                state = 1;
            break;

        case 1: // MOVE
            if (energy > th_active)
                state = 2;
            else if (energy < th_move * 0.7f)
                state = 0;
            break;

        case 2: // ACTIVE
            if (energy < th_move)
                state = 0;
            else if (energy < th_active * 0.7f)
                state = 1;
            break;
    }

    // ===== debug =====
    printf("[IMU] dyn=%.3f E=%.3f base=%.3f | state=%d\n",
        dynamic, energy, baseline, state);

    return state;
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