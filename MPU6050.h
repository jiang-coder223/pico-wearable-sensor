#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"


// 暫存器定義
#define MPU6050_ADDR            0x68
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

// 外部變數宣告，讓 main.c 可以存取
extern int32_t ax_offset, ay_offset, az_offset;
extern int32_t gx_offset, gy_offset, gz_offset;

// 函式原型
void mpu6050_init(void);
void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);
void mpu6050_calibrate(void);
int mpu6050_get_activity_step(void);
int mpu6050_get_state(void);
int mpu6050_get_activity(void);
void mpu6050_reset(void);

#endif