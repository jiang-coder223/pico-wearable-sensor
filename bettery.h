#ifndef BETTERY_H
#define BETTERY_H

void battery_init();
float read_battery_voltage_raw();
float read_battery_voltage_avg();
float low_pass_filter(float v);
int voltage_to_percent(float v);
void battery_update(float *voltage, int *percent);

#endif