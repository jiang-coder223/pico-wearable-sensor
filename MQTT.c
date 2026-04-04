#include "MQTT.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include <stdio.h>

static mqtt_client_t *client;
static int mqtt_ready = 0;

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

void mqtt_init(void) {
    if (cyw43_arch_init()) {
    printf("cyw43 init failed\n");
    return;
    }
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(
            "34n2f-5G",
            "29842804",
            CYW43_AUTH_WPA2_AES_PSK,
            30000)) {
        printf("WiFi failed\n");
        return;
    }

    printf("WiFi connected\n");

    client = mqtt_client_new();

    ip_addr_t broker_ip;
    ipaddr_aton("192.168.0.106", &broker_ip);

    struct mqtt_connect_client_info_t ci = {
    .client_id = "pico",
    };

    err_t err = mqtt_client_connect(client,
                                    &broker_ip,
                                    1883,
                                    mqtt_connection_cb,
                                    NULL,
                                    &ci);
    printf("mqtt_client_connect err = %d\n", err);
}

void mqtt_publish_bpm(int bpm) {

    if (!mqtt_ready) return;

    char msg[32];
    snprintf(msg, sizeof(msg),
            "{\"bpm\":%d}", bpm);

    mqtt_publish(client,
                "sensor/heart_rate",
                msg,
                strlen(msg),
                0, 0,
                NULL, NULL);
}

void mqtt_loop(void) {
    cyw43_arch_poll();
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_ready = 1;
        printf("MQTT REAL connected\n");
    } else {
        mqtt_ready = 0;
        printf("MQTT connect failed: %d\n", status);
    }
}