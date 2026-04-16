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
#define MIN_LAG 50
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
static float last_R = 0;
static float g_R = 0;
static float g_ratio_r = 0;
static float g_ratio_i = 0;
static int reject_count = 0;
static float R_history[5] = {0};
static int R_idx = 0;
static uint32_t last_peak_time = 0;
static int peak_bpm = 0;
static float peak_threshold = 50.0f;
static float prev_hp_for_peak = 0;
static int refractory = 0;


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
    write_reg(MAX30102_LED2_PA, 0x10); 
    
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

            /* static int dbg_count = 0;
            dbg_count++;
            int print_now = (dbg_count % 50 == 0); */

    uint32_t diff = (red > prev_red) ? (red - prev_red) : (prev_red - red);
    prev_red = red;

    if (diff > 2000) {
        stable_count = 0;
        return;
    }

    if (diff < 2000) stable_count++;
    else stable_count = 0;

    int valid = (stable_count >= 5 && diff < 8000);

    // ===== finger detect =====
    if (!has_signal) {
        if (red > 8000) has_signal = 1;
    } else {
        if (red < 4000) has_signal = 0;
    }

    if (!has_signal) {
        max30102_hr_init();
        averaged_bpm = 0;
        return;
    }

            /* if (print_now) {
                printf("[RAW] red=%lu diff=%lu stable=%d\n", red, diff, stable_count);
            } */

    // ===== DC removal =====
    if (lp == 0) lp = (float)red;
    lp = (lp * 0.92f) + (red * 0.08f);

    // ===== AC Amplifier =====
    float ac_val = ((float)red - lp) * 5.0f;

    if (fabs(ac_val) > 5000.0f) {
        return;
    }

    // ===== smoothing =====
    smooth = 0.4f * ac_val + 0.6f * smooth;

    static float hp = 0;
    static float prev_hp = 0;

    hp = smooth - 0.8f * hp;

    // ===== spike limiter（限制最大幅度）=====
    if (hp > 1000.0f) hp = 1000.0f;
    if (hp < -1000.0f) hp = -1000.0f;

    // ===== jump filter（忽略瞬間爆衝）=====
    if (fabs(hp - prev_hp) > 2000.0f) {
        hp = prev_hp;
    }

    prev_hp = hp;

            /* if (print_now) {
                printf("[FILTER] ac=%.1f smooth=%.1f hp=%.1f\n", ac_val, smooth, hp);
            } */

    
    // ===== validity filter =====
    if (fabs(hp) > 700.0f) {
        goto skip;   // 被 clamp 的爆訊號，無效
    }

    data_buffer[data_idx++] = hp;

    if (data_idx >= WINDOW_SIZE) {

        float max_corr = -1.0f;
        int best_lag = -1;
        float zero_corr = 0;

        for (int i = 0; i < WINDOW_SIZE; i++)
            zero_corr += data_buffer[i] * data_buffer[i];

        float best_score = -1e9;

        for (int lag = MIN_LAG; lag <= MAX_LAG; lag++) {
            float corr = 0;

            for (int i = 0; i < (WINDOW_SIZE - lag); i++) {
                corr += data_buffer[i] * data_buffer[i + lag];
            }

            if (corr > max_corr) {
                max_corr = corr;
            }

            float penalty = 0;
            if (last_lag != 0) {
                penalty = fabs(lag - last_lag) * 0.01f;
            }

            float score = corr - penalty;

            if (score > best_score) {
                best_score = score;
                best_lag = lag;
            }
        }


        // ===== confidence =====
        float confidence = max_corr / zero_corr;

                /* if (print_now) {
                    printf("[LAG] best=%d conf=%.2f\n", best_lag, confidence);
                } */

        if (confidence < 0.35f) {
            goto skip;
        }

        last_best_lag = best_lag;
        last_confidence = confidence;

        if (best_lag <= 0) goto skip;

        if (last_lag != 0) {
            int diff = abs(best_lag - last_lag);

            if (confidence < 0.35f || diff > 12) {
                bad_count++;
            } else {
                bad_count = 0;
            }
        }

        if (bad_count > 10) {
                bad_count = 5;
        }

        if (best_lag > 0 && confidence > 0.35f) {

            bad_count = 0;
            if (last_lag != 0 && abs(best_lag - last_lag) > 12)
                best_lag = last_lag;

            // 半週修正（非常關鍵）
            if (best_lag * 2 <= MAX_LAG) {
                int lag2 = best_lag * 2;

                float corr2 = 0;
                for (int i = 0; i < WINDOW_SIZE - lag2; i++) {
                    corr2 += data_buffer[i] * data_buffer[i + lag2];
                }

                if (corr2 > max_corr * 0.75f) {
                    best_lag = lag2;
                }
            }

            if (last_lag != 0) {
                int diff = abs(best_lag - last_lag);

                if (diff < 12) {
                        // ✔ 正常範圍 → 做平滑追蹤
                        best_lag = (int)(0.5f * last_lag + 0.5f * best_lag);
                }
            }

            if (best_lag < 50 || best_lag > 120)
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

void max30102_spo2_update(uint32_t red, uint32_t ir)
{
    static int init_count = 0;

    // ===== 初始化 =====
    if (init_count < 50) {
        dc_red = red;
        dc_ir  = ir;
        init_count++;
        return;
    }

    // ===== DC tracking =====
    dc_red = 0.95f * dc_red + 0.05f * red;
    dc_ir  = 0.95f * dc_ir  + 0.05f * ir;

    // ===== finger detect =====
    if (dc_red < 10000) {
        spo2_value = 0;
        return;
    }

    // ===== AC =====
    float ac_red = red - dc_red;
    float ac_ir  = ir  - dc_ir;

    // ===== buffer =====
    red_buf[spo2_idx] = ac_red;
    ir_buf[spo2_idx]  = ac_ir;
    spo2_idx++;

    if (spo2_idx < SPO2_BUF) return;
    spo2_idx = 0;

    // ===== RMS (差分版，抗漂移) =====
    float ac_r = 0;
    float ac_i = 0;

    for (int i = 1; i < SPO2_BUF; i++) {
        float dr = red_buf[i] - red_buf[i-1];
        float di = ir_buf[i]  - ir_buf[i-1];
        ac_r += dr * dr;
        ac_i += di * di;
    }

    ac_r = sqrtf(ac_r / SPO2_BUF);
    ac_i = sqrtf(ac_i / SPO2_BUF);

    // ===== ratio =====
    float ratio_r = ac_r / dc_red;
    float ratio_i = ac_i / dc_ir;

    // ===== 品質 gating（關鍵）=====
    float signal_strength = ratio_r + ratio_i;

    // 太弱（雜訊）
    if (signal_strength < 0.0001f) return;

    // 太強（晃動 / 壓太緊）
    if (signal_strength > 0.01f) return;

    g_ratio_r = ratio_r;
    g_ratio_i = ratio_i;

    // ===== 結構檢查（避免假R）=====
    float balance = ratio_r / ratio_i;

    // 避免兩者太接近（典型 noise 特徵）
    if (balance > 0.95f && balance < 1.05f && signal_strength < 0.0003f) {
    return;
    }

    // ===== 基本 gating（極簡版）=====
    if (ratio_r < 0.00003f || ratio_i < 0.00003f) return;
    if (ratio_r > 0.02f   || ratio_i > 0.02f)   return;

    // ===== R =====
    float R = ratio_r / ratio_i;

    // 合理範圍
    if (R < 0.3f || R > 0.9f) return;

    g_R = R;
    static float last_R = 0;

    if (last_R != 0 && fabs(R - last_R) > 0.1f) {
        return;
    }

    // ===== SpO2 =====
    last_R = R;
    int spo2 = (int)(106 - 18 * R);

    if (spo2 > 100) spo2 = 100;
    if (spo2 < 70)  spo2 = 70;

    // ===== smoothing =====
    if (spo2_value == 0)
        spo2_value = spo2;
    else
        spo2_value = 0.6f * spo2_value + 0.4f * spo2;
}

int max30102_get_spo2(void) {
    return spo2_value;
}

float max30102_get_R(void) {
    return g_R;
}

float max30102_get_ratio_r(void) {
    return g_ratio_r;
}

float max30102_get_ratio_i(void) {
    return g_ratio_i;
}