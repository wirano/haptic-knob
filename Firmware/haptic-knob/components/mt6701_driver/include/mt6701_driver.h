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

#ifndef MT6701_DRIVER_H
#define MT6701_DRIVER_H

#include <stdint.h>

typedef struct {
    uint8_t data_raw[3];

    void (*over_speed_cb)(void);

    void (*btn_pressed_cb)(void);

    void (*ssi_read)(uint8_t *rec_buffer, uint8_t rec_len);
} mt6701_instance_t;

typedef mt6701_instance_t *mt6701_handle_t;

void mt6701_init(mt6701_handle_t *handle, void (*ssi_read)(uint8_t *rec_buffer, uint8_t rec_len));

float mt6701_get_angle(mt6701_handle_t handle);

#endif //MT6701_DRIVER_H
