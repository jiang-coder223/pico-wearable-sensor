#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>
#include "global_defines.h"
#include "math.h"

#define FS_HZ              100.0f

#define HPF_CUTOFF_HZ      0.5f
#define LPF_CUTOFF_HZ      3.0f

#define INIT_THRESHOLD     0.12f
#define TH_FACTOR          1.5f

#define MIN_STEP_MS        300
#define MAX_STEP_MS        2000

typedef struct {
    float y;
    float x_prev;
} HPF;

typedef struct {
    float y;
} LPF;

static int state = 0;
static int steps = 0;
static float g_ax, g_ay, g_az;
static float g_gx, g_gy, g_gz;

// 定義變數實體
int32_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int32_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

void mpu6050_init(void) {
    uint8_t buf[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C0_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_update(void) {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu6050_read_accel(&ax, &ay, &az);
    mpu6050_read_gyro(&gx, &gy, &gz);

    // accel 校正
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;

    // gyro 校正
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    g_ax = ax / 16384.0f;
    g_ay = ay / 16384.0f;
    g_az = az / 16384.0f;

    g_gx = gx / 131.0f;   // deg/s（±250dps）
    g_gy = gy / 131.0f;
    g_gz = gz / 131.0f;
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

    /* printf("Offset: ax=%d ay=%d az=%d\n", ax_offset, ay_offset, az_offset);
    printf("Calibrate Done!\n"); */
}

int mpu6050_get_state(void){
    return state;
}

void mpu6050_activity_update(void)
{
    static float energy = 0.0f;
    static float baseline = 0.02f;

    float ax_g = g_ax;
    float ay_g = g_ay;
    float az_g = g_az;

    float mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    static float gravity = 1.0f;

    // 慢速追蹤重力（時間常數約 1~2 秒）
    gravity = 0.995f * gravity + 0.005f * mag;

    float dynamic = fabsf(mag - gravity);

    energy = 0.8f * energy + 0.2f * dynamic;

    if (energy < 0.05f) {
        baseline = 0.99f * baseline + 0.01f * energy;
    }

    float th_move   = baseline * 3.0f;
    float th_active = baseline * 8.0f;

    switch (state) {
        case 0:
            if (energy > th_active) state = 2;
            else if (energy > th_move) state = 1;
            break;

        case 1:
            if (energy > th_active) state = 2;
            else if (energy < th_move * 0.7f) state = 0;
            break;

        case 2:
            if (energy < th_move) state = 0;
            else if (energy < th_active * 0.7f) state = 1;
            break;
    }

    static int dbg_count = 0;

    if (++dbg_count >= 20) {   // 每 200ms 印一次
        dbg_count = 0;
    }
}

static float hpf_alpha(float fc) {
    float dt = 1.0f / FS_HZ;
    float RC = 1.0f / (2.0f * 3.1415926f * fc);
    return RC / (RC + dt);
}

static float lpf_alpha(float fc) {
    float dt = 1.0f / FS_HZ;
    float RC = 1.0f / (2.0f * 3.1415926f * fc);
    return dt / (RC + dt);
}

static float hpf_update(HPF *f, float x, float a) {
    f->y = a * (f->y + x - f->x_prev);
    f->x_prev = x;
    return f->y;
}

static float lpf_update(LPF *f, float x, float a) {
    f->y = f->y + a * (x - f->y);
    return f->y;
}

int mpu6050_get_steps(void){
    return steps;
}
void mpu6050_step_counter_update(void) {

    // ---- static state ----
    static HPF hpf = {0};
    static LPF lpf = {0};
    static float a_hpf_a = 0, a_lpf_a = 0;
    static int init = 0;

    static float prev = 0, prev2 = 0;
    static float threshold = INIT_THRESHOLD;

    static uint32_t last_step_ms = 0;

    static float mean = 0, var = 0;   // for adaptive threshold

    // ---- init ----
    if (!init) {
        a_hpf_a = hpf_alpha(HPF_CUTOFF_HZ);
        a_lpf_a = lpf_alpha(LPF_CUTOFF_HZ);
        init = 1;
    }

    // ---- magnitude ----
    float mag = sqrtf(g_ax*g_ax + g_ay*g_ay + g_az*g_az);

    // ---- band-pass ----
    float x = hpf_update(&hpf, mag, a_hpf_a);
    x = lpf_update(&lpf, x, a_lpf_a);

    // ---- 更新統計（rolling）----
    mean = 0.99f * mean + 0.01f * x;
    float diff = x - mean;
    var = 0.99f * var + 0.01f * diff * diff;
    float std = sqrtf(var);

    // ---- adaptive threshold ----
    threshold = fmaxf(INIT_THRESHOLD, mean + TH_FACTOR * std);

    // ---- peak detection ----
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (prev > prev2 && prev > x && prev > threshold) {

        uint32_t dt = now - last_step_ms;

        if (dt > MIN_STEP_MS && dt < MAX_STEP_MS) {
            steps++;
            last_step_ms = now;
        }
    }

    prev2 = prev;
    prev = x;
}