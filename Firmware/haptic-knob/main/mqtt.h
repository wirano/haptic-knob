//
// Created by wirano on 23-5-5.
//

#ifndef MQTT_H
#define MQTT_H

#include "mqtt_client.h"


extern esp_mqtt_client_handle_t mqtt_client;

void mqtt_app_start(void);

#endif //MQTT_H
