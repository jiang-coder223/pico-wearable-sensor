#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"


// 暫存器定義
#define MPU6050_ADDR            0x68
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

// 函式原型
void mpu6050_init(void);
void mpu6050_update(void);
void mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
void mpu6050_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);
void mpu6050_calibrate(void);
int mpu6050_get_state(void);
void mpu6050_activity_update(void);
int mpu6050_get_steps(void);
void mpu6050_step_counter_update(void);
float mpu6050_get_activity(void);

#endif