#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MAX30102.h"
#include <stdio.h>
#include "global_defines.h"
#include "stdlib.h"
#include "math.h"

#define SAMPLE_RATE 100
#define WINDOW_SIZE 256
#define SLIDE_SIZE 128
#define MIN_LAG 60
#define MAX_LAG 100

#define SPO2_BUF 100

#define DEBUG_RAW 0
#define DEBUG_HR  0
#define DEBUG_SPO2 0


static float data_buffer[WINDOW_SIZE];
static int data_idx = 0;
static int averaged_bpm = 0;
static float lp = 0;
static float smooth = 0;
static int last_lag = 0;
static int init_count = 0;
static int has_signal = 0;
static int prev_red = 0;
static int stable_count = 0;
static int last_best_lag = 0;
static float last_confidence = 0;
static int bad_count = 0;

static float dc_red = 0;
static float dc_ir  = 0;
static float red_buf[SPO2_BUF];
static float ir_buf[SPO2_BUF];
static int spo2_idx = 0;
static int spo2_value = 0;


// driver
static int write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(I2C0_PORT, MAX30102_ADDR, buf, 2, false, 1000);
}

static int read_reg(uint8_t reg, uint8_t *value) {
    int ret = i2c_write_timeout_us(I2C0_PORT, MAX30102_ADDR, &reg, 1, true, 1000);
    if (ret < 0) return ret;

    return i2c_read_timeout_us(I2C0_PORT, MAX30102_ADDR, value, 1, false, 1000);
}

void max30102_init(void) {
    uint8_t data;

    printf("Init MAX30102...\n");

    // 讀 PART ID
    int ret = read_reg(MAX30102_PART_ID, &data);
    if (ret < 0) {
        printf("Read PART_ID failed\n");
        return;
    }

    printf("PART_ID = 0x%02X\n", data);

    // reset
    ret = write_reg(MAX30102_MODE_CONFIG, 0x40);
    if (ret < 0) {
        printf("Reset failed\n");
        return;
    }

    sleep_ms(100);

    printf("Init done\n");
}

void max30102_read_fifo(uint32_t *red, uint32_t *ir) {
    uint8_t reg = MAX30102_FIFO_DATA;
    uint8_t data[6];

    // 使用阻塞模式讀取 6 bytes (3 bytes Red + 3 bytes IR)
    i2c_write_blocking(I2C0_PORT, MAX30102_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, MAX30102_ADDR, data, 6, false);

    // 數據解析 (將三個 8-bit 組合成一個 18-bit/24-bit 數值)
    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *red &= 0x03FFFF; // 屏蔽多餘位元

    *ir = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *ir &= 0x03FFFF; 
}

// config
void max30102_setup(void) {
    // 1. FIFO Configuration (暫存器 0x08)
    // SMP_AVE (bit 7-5): 000 = No averaging (不平均，反應最快)
    // FIFO_ROLLOVER_EN (bit 4): 1 = 滿了自動覆蓋舊數據
    // FIFO_A_FULL (bit 3-0): 0xF = 剩下 15 格時觸發中斷 (我們目前沒用到中斷可忽略)
    write_reg(MAX30102_FIFO_CONFIG, 0x10); 

    // 2. Mode Config (暫存器 0x09)
    // 0x03 為 SpO2 模式 (同時開啟 Red + IR)
    write_reg(MAX30102_MODE_CONFIG, 0x03); 
    
    // 3. SpO2 Config (暫存器 0x0A)
    // SPO2_ADC_RGE (bit 6-5): 01 = 4096nA 範圍
    // SPO2_SR (bit 4-2): 001 = 100Hz 採樣率
    // LED_PW (bit 1-0): 11 = 411us 脈衝寬度 (18-bit 解析度)
    write_reg(MAX30102_SPO2_CONFIG, 0x47); 
    
    // 4. LED 電流設定 (暫存器 0x0C, 0x0D)
    // 0x24 約為 7.2mA。如果覺得數據太小可以調高到 0x3F (12.5mA)
    write_reg(MAX30102_LED1_PA, 0x3F); 
    write_reg(MAX30102_LED2_PA, 0x3F); 
    
    // 5. 重要：重設 FIFO 指標 (清空緩衝區)
    // 確保一開始讀取就是最新的數據，不會讀到重置前的殘留值
    write_reg(MAX30102_FIFO_WR_PTR, 0x00);
    write_reg(MAX30102_OVF_COUNTER, 0x00);
    write_reg(MAX30102_FIFO_RD_PTR, 0x00);
    
    printf("MAX30102 Optimized Setup Done.\n");
}


// heart rate
void max30102_hr_init(void) {
    data_idx = 0;
    averaged_bpm = 0;
    lp = 0;
    smooth = 0;
    last_lag = 0;
    init_count = 0;
    has_signal = 0;
    prev_red = 0;
    stable_count = 0;
}

int max30102_get_bpm(void) {
    return averaged_bpm;
}

void max30102_hr_update(uint32_t red, uint32_t ir) {

    uint32_t diff = (red > prev_red) ? (red - prev_red) : (prev_red - red);
    prev_red = red;

    if (diff < 2000) stable_count++;
    else stable_count = 0;

    int valid = (stable_count >= 5 && diff < 8000);

    // finger detect
    if (!has_signal) {
        if (red > 8000) has_signal = 1;
    } else {
        if (red < 4000) has_signal = 0;
    }

    if (!has_signal) {
        max30102_hr_init();
        return;
    }

    // DC removal
    if (lp == 0) lp = (float)red;
    lp = (lp * 0.92f) + (red * 0.08f);

    float ac_val = (float)red - lp;

    // smooth

    float alpha = 0.18f;
    smooth = alpha * ac_val + (1 - alpha) * smooth;

    static float hp = 0;
    hp = smooth - 0.95f * hp;

    data_buffer[data_idx++] = hp;

    if (data_idx >= WINDOW_SIZE) {

        float max_corr = -1.0f;
        int best_lag = -1;
        float zero_corr = 0;

        for (int i = 0; i < WINDOW_SIZE; i++)
            zero_corr += data_buffer[i] * data_buffer[i];

        for (int lag = MIN_LAG; lag <= MAX_LAG; lag++) {
            float corr = 0;
            for (int i = 0; i < (WINDOW_SIZE - lag); i++) {
                corr += data_buffer[i] * data_buffer[i + lag];
            }
            if (corr > max_corr) {
                max_corr = corr;
                best_lag = lag;
            }
        }

        float confidence = max_corr / zero_corr;

        

        if (confidence < 0.4f && last_lag != 0) {
            best_lag = last_lag;
        }

        last_best_lag = best_lag;
        last_confidence = confidence;

        if (best_lag <= 0) goto skip;

        if (last_lag != 0) {
            int diff = abs(best_lag - last_lag);

            if (confidence < 0.2f || diff > 12) {
                bad_count++;
            } else {
                bad_count = 0;
            }
        }

        if (bad_count > 10) {
                bad_count = 5;
        }

        if (best_lag > 0 && confidence > 0.4f) {

            bad_count = 0;
            if (last_lag != 0 && abs(best_lag - last_lag) > 12)
                best_lag = last_lag;

            if (last_lag != 0) {
                int diff = abs(best_lag - last_lag);

                if (diff < 12) {
                        // ✔ 正常範圍 → 做平滑追蹤
                        best_lag = (int)(0.7f * last_lag + 0.3f * best_lag);
                }
            }

            if (best_lag < 50 || best_lag > 100)
                goto skip;

            int bpm = (SAMPLE_RATE * 60) / best_lag;

            if (bpm < 40 || bpm > 180)
                goto skip;

            if (averaged_bpm == 0) {
                init_count++;
                if (init_count < 2) goto skip;

                averaged_bpm = bpm;
                last_lag = best_lag;
                return;
            }

            if (bpm > averaged_bpm + 8) bpm = averaged_bpm + 8;
            if (bpm < averaged_bpm - 8) bpm = averaged_bpm - 8;

            averaged_bpm = (averaged_bpm * 0.5f) + (bpm * 0.5f);
            last_lag = best_lag;
        }

skip:
        for (int i = 0; i < WINDOW_SIZE - SLIDE_SIZE; i++) {
            data_buffer[i] = data_buffer[i + SLIDE_SIZE];
        }
        data_idx = WINDOW_SIZE - SLIDE_SIZE;
    }
}

int max30102_get_lag(void) {
    return last_best_lag;
}

float max30102_get_confidence(void) {
    return last_confidence;
}

// SpO2
void max30102_spo2_init(void) {
    dc_red = 0;
    dc_ir  = 0;
    spo2_idx = 0;
    spo2_value = 0;
}

void max30102_spo2_update(uint32_t red, uint32_t ir) {

    // DC tracking
    dc_red = 0.95f * dc_red + 0.05f * red;
    dc_ir  = 0.95f * dc_ir  + 0.05f * ir;

    // finger detect（共用）
    if (dc_red < 50000) {
        spo2_idx = 0;
        return;
    }

    float ac_red = red - dc_red;
    float ac_ir  = ir  - dc_ir;

    red_buf[spo2_idx] = ac_red;
    ir_buf[spo2_idx]  = ac_ir;
    spo2_idx++;

    if (spo2_idx < SPO2_BUF) return;

    // RMS
    float sum_r = 0, sum_i = 0;
    for (int i = 0; i < SPO2_BUF; i++) {
        sum_r += red_buf[i] * red_buf[i];
        sum_i += ir_buf[i]  * ir_buf[i];
    }

    float ac_r = sqrtf(sum_r / SPO2_BUF);
    float ac_i = sqrtf(sum_i / SPO2_BUF);

    // ===== [PATCH 1] Signal gating =====
    float ratio_r = ac_r / dc_red;
    float ratio_i = ac_i / dc_ir;

    if (ratio_r < 0.003f || ratio_i < 0.003f) {
        spo2_idx = 0;
        return;
    }
    // ===== [PATCH 2] R calculation =====
    float R = (ac_r / dc_red) / (ac_i / dc_ir);

    // ===== [PATCH 3] R range limit =====
    if (R < 0.3f || R > 1.5f) {
        spo2_idx = 0;
        return;
    }

    int spo2 = (int)(110 - 25 * R);

    if (spo2 > 100) spo2 = 100;
    if (spo2 < 70)  spo2 = 70;

    // ===== [PATCH 4] smoothing =====
    spo2_value = (int)(0.8f * spo2_value + 0.2f * spo2);

    spo2_idx = 0;

    printf("R=%.3f ACr/DC=%.4f ACi/DC=%.4f\n",
    R, ratio_r, ratio_i);
}

int max30102_get_spo2(void) {
    return spo2_value;
}