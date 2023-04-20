// MIT License
//
// Copyright (c) 2023 wirano
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

//
// Created by wirano on 23-4-5.
//

#include "mt6701_driver.h"
#include "esp_log.h"
#include <math.h>
#include <stdint.h>
#include <string.h>


#define _2PI 6.28318530718f

#define TAG "mt6701"


typedef union {
    struct __attribute__((__packed__)) {
        struct {
            uint8_t crc : 6;
            uint8_t mg_state_strength : 2;
        };
        struct {
            uint16_t mg_state_btn : 1;
            uint16_t mg_state_speed : 1;
            uint16_t angle : 14;
        };
    };
    uint8_t bytes[3];
} mt6701_rec_data_t;

#define CRC6_POLY 0x0c // 0x0c = 0x03 << (8 - 6)

static uint8_t crc6_check(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;

    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC6_POLY;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void mt6701_read_data(mt6701_handle_t handle) {
    uint8_t swap;
    mt6701_rec_data_t *temp;

    handle->ssi_read(handle->data_raw, sizeof(handle->data_raw));

    // check crc in big-endian
    uint8_t crc = crc6_check(handle->data_raw, sizeof(handle->data_raw));
    if (crc) {
        ESP_LOGE(TAG, "crc check failed");
        memset(handle->data_raw, 0, sizeof(handle->data_raw));
        return;
    }

    // swap data to fit bit order
    swap = handle->data_raw[0];
    handle->data_raw[0] = handle->data_raw[2];
    handle->data_raw[2] = swap;

    temp = (mt6701_rec_data_t *) handle->data_raw;

    if (temp->mg_state_speed) {
        ESP_LOGW(TAG, "over speed!");
        if (handle->over_speed_cb) {
            handle->over_speed_cb();
        }
    }

    if (temp->mg_state_btn) {
        ESP_LOGI(TAG, "btn pressed!");
        if (handle->btn_pressed_cb) {
            handle->btn_pressed_cb();
        }
    }
}

void mt6701_init(mt6701_handle_t *handle, void (*ssi_read)(uint8_t *rec_buffer, uint8_t rec_len)) {
    mt6701_instance_t *dev = malloc(sizeof(mt6701_instance_t));

    dev->ssi_read = ssi_read;
    dev->over_speed_cb = NULL;
    dev->btn_pressed_cb = NULL;

    *handle = dev;
}

float mt6701_get_angle_deg(mt6701_handle_t handle) {
    mt6701_rec_data_t *temp;

    mt6701_read_data(handle);

    temp = (mt6701_rec_data_t *) handle->data_raw;

    return (float) temp->angle / 16384.f * 360.f;
}

float mt6701_get_angle_rad(mt6701_handle_t handle) {
    mt6701_rec_data_t *temp;

    mt6701_read_data(handle);

    temp = (mt6701_rec_data_t *) handle->data_raw;

    return (float) temp->angle / 16384.f * _2PI;
}
