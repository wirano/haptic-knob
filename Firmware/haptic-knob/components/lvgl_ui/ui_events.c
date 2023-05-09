// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.2.3
// LVGL version: 8.3.4
// Project name: haptic_knob

#include "core/lv_group.h"
#include "core/lv_obj.h"
#include "ui.h"
#include "lv_port/include/lv_port_indev.h"
#include "knob.h"
#include "knob_task.h"
#include "mqtt.h"
#include "widgets/lv_arc.h"
#include "widgets/lv_label.h"
#include <stdio.h>

void init_action(lv_event_t *e) {
    lv_group_t *g = lv_group_create();
    lv_group_set_default(g);
    lv_indev_set_group(indev_encoder, g);
    lv_group_add_obj(g, ui_Arc1);
    lv_group_add_obj(g, ui_Arc2);
    lv_group_add_obj(g, ui_Arc4);
    lv_group_add_obj(g, ui_Arc5);
}

void encoder_cb(lv_event_t *e) {
    knob_set_mode(knob, MODE_ENCODER);
}

void damped_cb(lv_event_t *e) {
    knob_set_zero(knob);
    knob_set_mode(knob, MODE_DAMPED);
}

void light_cb(lv_event_t *e) {
    knob_set_mode(knob, MODE_ENCODER);
}

void light_tuning(lv_event_t *e) {
    char data[5];
    lv_obj_t *arc = lv_event_get_target(e);
    int brightness = lv_arc_get_value(arc);
    int size = snprintf(data, sizeof(data), "%d", brightness);

    esp_mqtt_client_publish(mqtt_client, "homeassistant/light/yeelink_lamp22_5b33_light/set", data, size, 0, 0);
}

void swich_cb(lv_event_t *e) {
    knob_set_mode(knob, MODE_SWITCH);
}

void swich_status(lv_event_t *e) {
    lv_obj_t *arc = lv_event_get_target(e);
    int val = lv_arc_get_value(arc);

    switch (val) {
        case 0:
            lv_label_set_text(ui_Label15,"OFF");
            break;
        case 1:
            lv_label_set_text(ui_Label15,"ON");
            break;
    }
}
