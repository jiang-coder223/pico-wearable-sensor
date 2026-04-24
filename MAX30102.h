#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>

// =============================
// I2C address
// =============================
#define MAX30102_ADDR 0x57

// =============================
// register
// =============================
#define MAX30102_FIFO_WR_PTR     0x04
#define MAX30102_OVF_COUNTER     0x05
#define MAX30102_FIFO_RD_PTR     0x06
#define MAX30102_FIFO_DATA       0x07
#define MAX30102_FIFO_CONFIG     0x08  
#define MAX30102_MODE_CONFIG     0x09
#define MAX30102_SPO2_CONFIG     0x0A
#define MAX30102_LED1_PA         0x0C  // Red LED
#define MAX30102_LED2_PA         0x0D  // IR LED
#define MAX30102_PART_ID         0xFF

// =============================
// init & config
// =============================
void max30102_init(void);
void max30102_setup(void);

// =============================
// driver
// =============================
int max30102_read_fifo(uint32_t *red, uint32_t *ir);

// =============================
// Heart Rate
// =============================
void max30102_hr_init(void);
void max30102_hr_update(uint32_t red, uint32_t ir);
int  max30102_get_bpm(void);
int max30102_has_signal(void);

// =============================
// SpO2
// =============================
void max30102_spo2_init(void);
void max30102_spo2_update(uint32_t red, uint32_t ir);
int  max30102_get_spo2(void);

// =============================
// (Optional) Debug / tuning
// =============================
// 建議未來可以加
// void max30102_set_led_current(uint8_t red, uint8_t ir);

#endif