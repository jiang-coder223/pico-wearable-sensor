#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include "global_defines.h"

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