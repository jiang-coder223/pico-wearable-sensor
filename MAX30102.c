#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MAX30102.h"
#include <stdio.h>
#include "global_defines.h"
#include "stdlib.h"
#include "math.h"
#include "pico/time.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════
 * Tunables
 * ═══════════════════════════════════════════════════════════════ */
#define SAMPLE_RATE     100         /* Hz                           */
#define WINDOW_SIZE     200         /* samples  (2 s @ 100 Hz)      */
#define STEP_SIZE       (WINDOW_SIZE / 2)   /* 50 % overlap         */

#define AC_THRESHOLD    0.05f       /* autocorr / lag0 gate         */
#define RATIO_MIN       0.08f       /* periodicity ratio floor      */
#define CORR_MIN        0.70f       /* IR-RED cross-corr floor      */
#define PI_MIN          0.0001f     /* perfusion index floor        */
#define PI_MAX          0.05f       /* motion-artifact ceiling      */
#define QUALITY_GOOD    50.0f       /* quality score for conf++     */
#define RMS_FROZEN      5.0f        /* below this = dead signal     */
#define CONF_MAX        5
#define LAG_JUMP_MAX    25          /* samples — hard lag reset     */
#define NO_LAG_RESET    3           /* streak count — reopen search */
#define LOG_FAIL_EVERY  10          /* print fail msg every N times */

/* ═══════════════════════════════════════════════════════════════
 * HR state  (static — no VLA on Pico stack)
 * ═══════════════════════════════════════════════════════════════ */

/* ===== HR (band-pass + peak) ===== */

static float lp = 0, hp = 0, prev_lp = 0;
static float prev1 = 0, prev2 = 0;
static float threshold = 1000;

static uint32_t last_peak_time = 0;

static float averaged_bpm = 0;
static int has_signal = 0;
static int prev_has_signal = 0;
static int peak_count = 0;

/* ═══════════════════════════════════════════════════════════════
 * SpO2 state
 * ═══════════════════════════════════════════════════════════════ */
static float spo2_red_buf[WINDOW_SIZE];
static float spo2_ir_buf [WINDOW_SIZE];
static int   spo2_idx          = 0;
static int   spo2_value        = 0;
static float g_R               = 0.0f;
static float last_valid_R      = 0.0f;

/* ═══════════════════════════════════════════════════════════════
 * Low-level I²C helpers
 * ═══════════════════════════════════════════════════════════════ */
static int write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(I2C0_PORT, MAX30102_ADDR, buf, 2, false, 1000);
}

static int read_reg(uint8_t reg, uint8_t *value)
{
    int ret = i2c_write_timeout_us(I2C0_PORT, MAX30102_ADDR, &reg, 1, true, 1000);
    if (ret < 0) return ret;
    return i2c_read_timeout_us(I2C0_PORT, MAX30102_ADDR, value, 1, false, 1000);
}

/* ═══════════════════════════════════════════════════════════════
 * Hardware init / setup
 * ═══════════════════════════════════════════════════════════════ */
void max30102_init(void)
{
    uint8_t data;
    printf("Init MAX30102...\n");

    int ret = read_reg(MAX30102_PART_ID, &data);
    if (ret < 0) { printf("Read PART_ID failed\n"); return; }
    printf("PART_ID = 0x%02X\n", data);

    ret = write_reg(MAX30102_MODE_CONFIG, 0x40);   /* soft-reset */
    if (ret < 0) { printf("Reset failed\n"); return; }
    sleep_ms(100);
    printf("Init done\n");
}

void max30102_setup(void)
{
    /* FIFO: no averaging, roll-over enabled */
    write_reg(MAX30102_FIFO_CONFIG, 0x10);

    /* SpO2 mode (Red + IR), 4096 nA range, 100 Hz, 411 µs pulse */
    write_reg(MAX30102_MODE_CONFIG, 0x03);
    write_reg(MAX30102_SPO2_CONFIG, 0x47);

    /* LED current ~18.8 mA  (0x5F).  Raise to 0x7F (~25 mA) for wrist. */
    write_reg(MAX30102_LED1_PA, 0x24);
    write_reg(MAX30102_LED2_PA, 0x24);

    /* Clear FIFO */
    write_reg(MAX30102_FIFO_WR_PTR, 0x00);
    write_reg(MAX30102_OVF_COUNTER, 0x00);
    write_reg(MAX30102_FIFO_RD_PTR, 0x00);

    printf("MAX30102 setup done.\n");
}

int max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t wr = 0, rd = 0;
    if (read_reg(MAX30102_FIFO_WR_PTR, &wr) < 0) return -1;
    if (read_reg(MAX30102_FIFO_RD_PTR, &rd) < 0) return -1;

    if (((wr - rd) & 0x1F) <= 0) return 0;

    uint8_t reg  = MAX30102_FIFO_DATA;
    uint8_t data[6];

    if (i2c_write_timeout_us(I2C0_PORT, MAX30102_ADDR, &reg, 1, true,  1000) < 0) return -1;
    if (i2c_read_timeout_us (I2C0_PORT, MAX30102_ADDR, data, 6, false, 1000) < 0) return -1;

    *red = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]) & 0x03FFFF;
    *ir  = (((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5]) & 0x03FFFF;
    return 1;
}

/* ═══════════════════════════════════════════════════════════════
 * HR — init / getters
 * ═══════════════════════════════════════════════════════════════ */
void max30102_hr_init(void)
{
    averaged_bpm = 0.0f;

    has_signal = 0;
    prev_has_signal = 0;

    lp = hp = prev_lp = 0.0f;
    prev1 = prev2 = 0.0f;

    threshold = 0.0f;

    last_peak_time = 0;
}

int   max30102_get_bpm      (void) { return (int)(averaged_bpm + 0.5f); }
int   max30102_has_signal   (void) { return has_signal; }

/* ───────────────────────────────────────────────────────────────
 * Inline helper: suppress repeated failure spam.
 * Prints the message only on the 1st failure and every
 * LOG_FAIL_EVERY-th one after that.
 * ─────────────────────────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════
 * HR — main update  (call once per sample, i.e. at 100 Hz)
 * ═══════════════════════════════════════════════════════════════ */
void max30102_hr_update(uint32_t red, uint32_t ir)
{
    /* ── 0. finger detect ── */
    if (!has_signal) {
        if (ir > 20000) has_signal = 1;
    } else {
        if (ir < 10000) has_signal = 0;
    }

    if (!has_signal && prev_has_signal) {
        max30102_hr_init();
        max30102_spo2_init();
        last_peak_time = 0;
        printf("[RESET] finger removed\n");
    }

    prev_has_signal = has_signal;
    if (!has_signal) return;

    /* ── 1. Band-pass ── */
    lp = 0.9f * lp + 0.1f * ir;
    hp = lp - prev_lp + 0.95f * hp;
    prev_lp = lp;

    float signal = hp;

    /* ── 2. threshold（稍微加快反應） ── */
    float abs_sig = fabsf(signal);
    threshold = 0.85f * threshold + 0.15f * abs_sig;

    /* ── 3. slope peak detect ── */
    float slope1 = prev1 - prev2;
    float slope2 = signal - prev1;

    static float expected_interval = 0.0f;
    static float last_peak_value = 0.0f;

    float slope_ratio = prev1 / (prev2 + 1e-6f);

    if (slope1 > 0 && slope2 < 0 &&
        prev1 > threshold * 1.25f &&
        slope_ratio > 1.02f)   // ⭐ 關鍵修正（通用）
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (last_peak_time == 0) {
            last_peak_time = now;
            last_peak_value = prev1;
            goto shift;
        }

        uint32_t interval = now - last_peak_time;

        /* ── 4. interval 限制 ── */
        if (interval < 600 || interval > 1500) {
            last_peak_time = now;
            goto shift;
        }

        /* ── 5. peak 強度過濾（砍次波） ── */
        if (last_peak_value > 0) {
            if (prev1 < last_peak_value * 0.4f) {
                last_peak_time = now;
                goto shift;
            }
        }

        /* ── 6. 節奏一致性 ── */
        if (expected_interval > 0)
        {
            float ratio = interval / expected_interval;

            if (ratio < 0.75f) {
                last_peak_time = now;
                goto shift;
            }

            if (ratio > 1.6f) {
                interval *= 0.5f;
            }

        }

        float bpm = 60000.0f / interval;

        /* ── 7. BPM 範圍 ── */
        if (bpm < 50 || bpm > 120) {
            last_peak_time = now;
            goto shift;
        }

        /* ── 8. 抗跳動 ── */
        if (peak_count > 3 && averaged_bpm > 0 && fabsf(bpm - averaged_bpm) > 15){
            last_peak_time = now;
            goto shift;
        }

        /* ── 9. 平滑 ── */
        if (averaged_bpm <= 2)
            averaged_bpm = bpm;
        else
            averaged_bpm = 0.6f * averaged_bpm + 0.4f * bpm;

        /* ── 10. 更新節奏 ── */
        float cur_interval = 60000.0f / averaged_bpm;

        peak_count++;

        if (peak_count > 3)   // ⭐ 前3拍不建立節奏
        {
            if (expected_interval == 0)
                expected_interval = cur_interval;
            else
                expected_interval = 0.9f * expected_interval + 0.1f * cur_interval;
        }

        printf("[PEAK] %d ms BPM=%.0f\n", interval, bpm);
        printf("[BPM] %d\n", (int)averaged_bpm);

        last_peak_time = now;
        last_peak_value = prev1;
    }

shift:
    prev2 = prev1;
    prev1 = signal;
}

/* ═══════════════════════════════════════════════════════════════
 * SpO2
 * ═══════════════════════════════════════════════════════════════ */
void max30102_spo2_init(void)
{
    spo2_value   = 0;
    spo2_idx     = 0;
    g_R          = 0.0f;
    last_valid_R = 0.0f;
    memset(spo2_red_buf, 0, sizeof(spo2_red_buf));
    memset(spo2_ir_buf,  0, sizeof(spo2_ir_buf));
}

void max30102_spo2_update(uint32_t red, uint32_t ir)
{
    if (!has_signal) { spo2_idx = 0; return; }

    spo2_red_buf[spo2_idx] = (float)red;
    spo2_ir_buf [spo2_idx] = (float)ir;
    spo2_idx++;
    if (spo2_idx < WINDOW_SIZE) return;
    spo2_idx = 0;

    /* DC */
    float red_mean = 0.0f, ir_mean = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        red_mean += spo2_red_buf[i];
        ir_mean  += spo2_ir_buf[i];
    }
    red_mean /= WINDOW_SIZE;
    ir_mean  /= WINDOW_SIZE;

    if (red_mean < 1000.0f || ir_mean < 1000.0f) return;

    /* AC RMS */
    float red_ac = 0.0f, ir_ac = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float r = spo2_red_buf[i] - red_mean;
        float v = spo2_ir_buf[i]  - ir_mean;
        red_ac += r * r;
        ir_ac  += v * v;
    }
    red_ac = sqrtf(red_ac / WINDOW_SIZE);
    ir_ac  = sqrtf(ir_ac  / WINDOW_SIZE);

    if (red_ac < 30.0f || ir_ac < 30.0f) return;

    float ratio_r = red_ac / red_mean;
    float ratio_i = ir_ac  / ir_mean;

    /* Perfusion gate — same logic as HR */
    if (ratio_r < 0.0008f || ratio_i < 0.0008f) return;
    if (ratio_i > PI_MAX)  return;   /* motion artifact */

    /* R = (AC_red/DC_red) / (AC_ir/DC_ir) */
    float R = ratio_r / ratio_i;
    if (R < 0.4f || R > 3.4f) return;   /* physically impossible range */

    /* Reject large R jumps */
    if (g_R != 0.0f && fabsf(R - g_R) > 0.3f) return;

    /* Smooth R with slow EMA (changes slowly with SpO2) */
    g_R = (g_R == 0.0f) ? R : (0.95f * g_R + 0.05f * R);

    /*
     * Linear empirical approximation:
     *   SpO2 ≈ 110 − 25×R   (valid for R ≈ 0.4 – 1.0, SpO2 95–100 %)
     * For lower saturation extend with:
     *   SpO2 ≈ 104 − 17×R   (R 1.0 – 2.0, SpO2 80–95 %)
     */
    int spo2;
    if (g_R <= 1.0f)
        spo2 = (int)(110.0f - 25.0f * g_R + 0.5f);
    else
        spo2 = (int)(104.0f - 17.0f * g_R + 0.5f);

    if (spo2 > 100) spo2 = 100;
    if (spo2 < 70)  spo2 = 70;

    spo2_value   = spo2;
    last_valid_R = g_R;
}

int   max30102_get_spo2(void) { return spo2_value; }
float max30102_get_R   (void) { return g_R; }