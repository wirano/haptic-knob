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
#include <stdint.h>

#define TAG "mt6701"

typedef union {
    struct __attribute__((__packed__)) {
        struct {
            uint16_t angle : 14;
            uint16_t mg_state_speed : 1;
            uint16_t mg_state_btn : 1;
        };
        struct {
            uint8_t mg_state_strength : 2;
            uint8_t crc : 6;
        };
    };
    uint8_t bytes[3];
} mt6701_rec_data_t;

static void mt6701_read(mt6701_handle_t handle){
    handle->ssi_read(handle->data_raw, sizeof(handle->data_raw));
}

float mt6701_get_angle(mt6701_handle_t handle){
    mt6701_rec_data_t *temp;

    handle->ssi_read(handle->data_raw, sizeof(handle->data_raw));

    temp = (mt6701_rec_data_t *)handle->data_raw;

    if(handle->crc6 != NULL){
        uint8_t crc = handle->crc6(temp->bytes,2);

        if(crc != temp->crc){
            ESP_LOGE(TAG,"crc check failed");
            return -1;
        }
    }

    if(temp->mg_state_speed){
        //todo: over speed
    }

    if(temp->mg_state_btn){
        //todo: btn pressed
    }

    return (float)temp->angle / 16384 * 360;
}

