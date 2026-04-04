#ifndef MQTT_H
#define MQTT_H

void mqtt_init(void);
void mqtt_loop(void);
void mqtt_publish_bpm(int bpm);

#endif