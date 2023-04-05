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
// Created by wirano on 23-4-4.
//

#ifndef DRV8311_DRIVER_H
#define DRV8311_DRIVER_H

#include "drv8311_reg.h"
#include <stdint.h>

typedef enum {
    SPI = 0x00,
    tSPI = 0x01
} drv8311_protal_e;

typedef struct {
    drv8311_protal_e protel;
    uint8_t devicd_id;

    void
    (*spi_trans)(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data,
                 uint8_t rec_len);
} drv8311_instance_t;

typedef drv8311_instance_t *drv8311_handle_t;

void
drv8311_init(drv8311_handle_t *handle, drv8311_protal_e portal, uint8_t device_id,
             void (*spi_trans)(uint8_t *send_data, uint8_t send_len,
                               uint8_t *rec_data, uint8_t rec_len));

drv8311_dev_sts1_t drv8311_get_status(drv8311_handle_t handle);

#endif //DRV8311_DRIVER_H
