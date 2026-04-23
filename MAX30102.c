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

#define SPO2_BUF WINDOW_SIZE

#define DEBUG_RAW 0
#define DEBUG_HR  0
#define DEBUG_SPO2 0


static float    data_buffer[WINDOW_SIZE];
static int      data_idx = 0;
static int      averaged_bpm = 0;
static float    lp = 0;
static float    smooth = 0;
static int      last_lag = 0;
static int      init_count = 0;
static int      has_signal = 0;
static int      prev_red = 0;
static int      stable_count = 0;
static int      last_best_lag = 0;
static float    last_confidence = 0;
static int      bad_count = 0;

static float    red_buf[SPO2_BUF];
static float    ir_buf[SPO2_BUF];
static int      spo2_idx = 0;
static int      spo2_value = 0;
static float    last_R = 0;
static float    g_R = 0;
static float    g_ratio_r = 0;
static float    g_ratio_i = 0;
static int      reject_count = 0;
static float    R_history[5] = {0};
static int      R_idx = 0;
static uint32_t last_peak_time = 0;
static int      peak_bpm = 0;
static float    peak_threshold = 50.0f;
static float    prev_hp_for_peak = 0;
static int      refractory = 0;
static float    hp = 0;
static float    prev_hp = 0;
static float    prev_smooth = 0;
static int      prev_has_signal = 0;
static int      spo2_init_count = 0;
static float    spo2_last_R = 0;
static int      spo2_stable_count = 0;
static int      spo2_ready = 0;


static float hr_ir_buf[WINDOW_SIZE];
static float hr_red_buf[WINDOW_SIZE];

static float raw_red_buf[WINDOW_SIZE];
static float raw_ir_buf[WINDOW_SIZE];

static int rf_idx = 0;

static int last_period = 60;   // 初始假設 ~100 BPM


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


int max30102_read_fifo(uint32_t *red, uint32_t *ir) {

    uint8_t wr = 0, rd = 0;

    if (read_reg(MAX30102_FIFO_WR_PTR, &wr) < 0) return -1;
    if (read_reg(MAX30102_FIFO_RD_PTR, &rd) < 0) return -1;

    int available = (wr - rd) & 0x1F;

    if (available <= 0) {
        return 0;   // 沒資料
    }

    uint8_t reg = MAX30102_FIFO_DATA;
    uint8_t data[6];

    int ret = i2c_write_timeout_us(I2C0_PORT, MAX30102_ADDR, &reg, 1, true, 1000);
    if (ret < 0) return -1;

    // ✅ 只讀一筆
    ret = i2c_read_timeout_us(I2C0_PORT, MAX30102_ADDR, data, 6, false, 1000);
    if (ret < 0) return -1;

    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *red &= 0x03FFFF;

    *ir  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *ir &= 0x03FFFF;

    return 1;   // ✅ 固定回傳1
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
    hp = 0;
    prev_hp = 0;
    prev_smooth = 0;
    bad_count = 0;        
    last_best_lag = 0;     
    last_confidence = 0;
    rf_idx = 0;
    last_period = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        data_buffer[i] = 0;
    }
}

int max30102_get_bpm(void) {
    return averaged_bpm;
}

void max30102_hr_update(uint32_t red, uint32_t ir)
{
    // ===== finger detect =====
    if (!has_signal) {
        if (red > 8000) has_signal = 1;
    } else {
        if (red < 4000) has_signal = 0;
    }

    if (!has_signal && prev_has_signal) {
        max30102_hr_init();
        max30102_spo2_init();
        return;
    }

    prev_has_signal = has_signal;
    if (!has_signal) return;

    // ===== buffer =====
    raw_red_buf[rf_idx] = (float)red;
    raw_ir_buf[rf_idx]  = (float)ir;

    hr_ir_buf[rf_idx]  = (float)ir;
    hr_red_buf[rf_idx] = (float)red;
    rf_idx++;

    if (rf_idx < WINDOW_SIZE) return;
    rf_idx = 0;

    // ===== 1. DC removal =====
    float ir_mean = 0, red_mean = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        ir_mean  += hr_ir_buf[i];
        red_mean += hr_red_buf[i];
    }

    ir_mean /= WINDOW_SIZE;
    red_mean /= WINDOW_SIZE;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        hr_ir_buf[i]  -= ir_mean;
        hr_red_buf[i] -= red_mean;
    }

    printf("[DC] ir=%.1f red=%.1f\n", ir_mean, red_mean);

    // ===== 2. detrend =====
    float beta_ir = 0, beta_red = 0;
    float sum_x2 = 0;
    float x;

    float mean_x = (WINDOW_SIZE - 1) / 2.0f;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        x = i - mean_x;
        beta_ir  += x * hr_ir_buf[i];
        beta_red += x * hr_red_buf[i];
        sum_x2   += x * x;
    }

    beta_ir  /= sum_x2;
    beta_red /= sum_x2;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        x = i - mean_x;
        hr_ir_buf[i]  -= beta_ir  * x;
        hr_red_buf[i] -= beta_red * x;
    }

    __asm volatile("" ::: "memory");

    spo2_ready = 1;
    
    // ===== 3. RMS =====
    float ir_sumsq = 0, red_sumsq = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        ir_sumsq  += hr_ir_buf[i]  * hr_ir_buf[i];
        red_sumsq += hr_red_buf[i] * hr_red_buf[i];
    }

    ir_sumsq  /= WINDOW_SIZE;
    red_sumsq /= WINDOW_SIZE;

    float ir_rms  = sqrtf(ir_sumsq);
    float red_rms = sqrtf(red_sumsq);

    printf("[RMS] ir=%.3f red=%.3f\n", ir_rms, red_rms);

    // ===== 4. correlation =====
    float corr = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        corr += hr_ir_buf[i] * hr_red_buf[i];
    }

    corr /= WINDOW_SIZE;
    corr /= (sqrtf(ir_sumsq * red_sumsq) + 1e-6f);

    printf("[CORR] %.3f\n", corr);

    if (corr < 0.7f) {
        printf("[SKIP] low corr\n");
        return;
    }

    // ===== 5. autocorr (RF-style) =====

    int search_min, search_max;

    if (last_period == 0) {
        search_min = 50;
        search_max = 120;
    } else {
        search_min = last_period - 20;
        search_max = last_period + 20;
    }

    if (search_min < 30) search_min = 30;
    if (search_max > 120) search_max = 120;

    int best_lag = 0;
    float best_corr = 0;

    float lag0 = ir_sumsq * WINDOW_SIZE;
    float threshold = 0.4f * lag0;

    // ===== Step 1：找第一個合理 lag =====
    for (int lag = search_min; lag <= search_max; lag++) {

        float ac = 0;

        for (int i = 0; i < WINDOW_SIZE - lag; i++) {
            ac += hr_ir_buf[i] * hr_ir_buf[i + lag];
        }

        if (ac > threshold) {
            best_lag = lag;
            best_corr = ac;
            break;
        }
    }

    if (best_lag == 0) {
        printf("[RESET] no valid lag\n");
        return;
    }

    // ===== Step 2：local peak refine =====

    while (best_lag > 50) {
        float curr = 0, left = 0;

        for (int i = 0; i < WINDOW_SIZE - best_lag; i++)
            curr += hr_ir_buf[i] * hr_ir_buf[i + best_lag];

        for (int i = 0; i < WINDOW_SIZE - (best_lag - 1); i++)
            left += hr_ir_buf[i] * hr_ir_buf[i + best_lag - 1];

        if (left <= curr) break;
        best_lag--;
    }

    while (best_lag < 120) {
        float curr = 0, right = 0;

        for (int i = 0; i < WINDOW_SIZE - best_lag; i++)
            curr += hr_ir_buf[i] * hr_ir_buf[i + best_lag];

        for (int i = 0; i < WINDOW_SIZE - (best_lag + 1); i++)
            right += hr_ir_buf[i] * hr_ir_buf[i + best_lag + 1];

        if (right <= curr) break;
        best_lag++;
    }

    // ⭐ 重新計算 best_corr（很重要）
    best_corr = 0;
    for (int i = 0; i < WINDOW_SIZE - best_lag; i++) {
        best_corr += hr_ir_buf[i] * hr_ir_buf[i + best_lag];
    }

    float ratio = best_corr / lag0;

    printf("[AC] lag=%d ratio=%.3f\n", best_lag, ratio);

    if (ratio < 0.15f) {
        printf("[SKIP] low periodic\n");
        return;
    }

    // ===== 6. lag stabilize =====
    if (last_period != 0) {
        int diff = abs(best_lag - last_period);

        if (diff > 25) {
            printf("[RESET] lag jump (%d)\n", diff);
            return;
        }

        best_lag = (int)(0.7f * last_period + 0.3f * best_lag);
    }

    last_period = best_lag;

    int bpm = (SAMPLE_RATE * 60) / best_lag;

    if (bpm < 40 || bpm > 180) {
        printf("[SKIP] bpm out\n");
        return;
    }

    // ===== 7. smoothing =====
    if (averaged_bpm == 0)
        averaged_bpm = bpm;
    else
        averaged_bpm = 0.7f * averaged_bpm + 0.3f * bpm;

    printf("[BPM] %d\n", averaged_bpm);
}

int max30102_get_lag(void) {
    return last_best_lag;
}

float max30102_get_confidence(void) {
    return last_confidence;
}

// SpO2
void max30102_spo2_init(void) {

    spo2_idx = 0;
    spo2_value = 0;
    spo2_last_R = 0;
    spo2_stable_count = 0;

    last_R = 0;
    g_R = 0;
    g_ratio_r = 0;
    g_ratio_i = 0;

    reject_count = 0;
    R_idx = 0;

    spo2_init_count = 0;

    //  buffer 清空
    for (int i = 0; i < SPO2_BUF; i++) {
        red_buf[i] = 0;
        ir_buf[i]  = 0;
    }

    // R history 清空
    for (int i = 0; i < 5; i++) {
        R_history[i] = 0;
    }

    // DC 初始化狀態
    // （會重新 warm-up）
}

void max30102_spo2_update(uint32_t red, uint32_t ir)
{
    if (!spo2_ready) return;
    spo2_ready = 0;

    printf("\n==== SpO2 WINDOW ====\n");

    // ===== DC（raw，同一窗口）=====
    float dc_red = 0, dc_ir = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        dc_red += raw_red_buf[i];
        dc_ir  += raw_ir_buf[i];
    }

    dc_red /= WINDOW_SIZE;
    dc_ir  /= WINDOW_SIZE;

    printf("[SpO2-DC] red=%.1f ir=%.1f\n", dc_red, dc_ir);

    if (dc_red < 50000 || dc_ir < 50000) {
        printf("[SpO2] reject: no finger\n");
        spo2_value = 0;
        return;
    }

    // ===== AC（HR clean）=====
    float sum_r = 0, sum_i = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum_r += hr_red_buf[i] * hr_red_buf[i];
        sum_i += hr_ir_buf[i]  * hr_ir_buf[i];
    }

    float ac_r = sqrtf(sum_r / WINDOW_SIZE);
    float ac_i = sqrtf(sum_i / WINDOW_SIZE);

    printf("[SpO2-AC] red=%.1f ir=%.1f\n", ac_r, ac_i);

    // ===== corr =====
    float corr = 0;
    float sum_r2 = 0;
    float sum_i2 = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        float r = raw_red_buf[i] - dc_red;
        float i_ = raw_ir_buf[i] - dc_ir;

        corr   += r * i_;
        sum_r2 += r * r;
        sum_i2 += i_ * i_;
    }

    corr /= WINDOW_SIZE;
    corr /= (sqrtf(sum_r2 * sum_i2) + 1e-6f);

    printf("[SpO2-CORR] %.3f\n", corr);

    // ===== R =====
    float R = (ac_r * dc_ir) / (ac_i * dc_red);

    printf("[SpO2-R] %.3f\n", R);

    if (R < 0.3f || R > 0.9f) return;

    float spo2 = (-45.060f * R + 30.354f) * R + 94.845f;

    if (spo2 > 100) spo2 = 100;
    if (spo2 < 80)  spo2 = 80;

    spo2_value = spo2;

    printf("[SpO2-OK] %.1f\n", spo2);
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

int max30102_has_signal(void) {
    return has_signal;
}