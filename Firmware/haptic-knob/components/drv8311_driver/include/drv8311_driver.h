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
    SPI = 0x00, tSPI = 0x01
} drv8311_protal_t;

typedef struct {
    uint8_t inited;
    struct {
        DRV8311_PWMCNTR_MODE_t mode;
        uint16_t period;
    } pwm_gen;

    struct {
        uint8_t en;
        DRV8311_CSA_GAIN_t gain;
    } csa;

    struct {
        drv8311_protal_t protal;
        uint8_t devicd_id;
        uint8_t parity_check;

        void (*spi_trans)(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len);

        void (*nsleep_set)(uint8_t level);
    } interface;
} drv8311_instance_t;

typedef struct {
    DRV8311_PWMCNTR_MODE_t pwmcnt_mode;
    DRV8311_PWM_OSC_SYNC_t sync_mode;
    DRV8311_SPICLK_FREQ_SYNC_t spi_clk;
    DRV8311_SPISYNC_ACRCY_t spi_sync_clks;
    drv8311_protal_t portal;
    DRV8311_CSA_GAIN_t csa_gain;

    uint16_t pwm_period;
    uint8_t use_csa;
    uint8_t dev_id;
    uint8_t parity_check;

    void (*spi_trans)(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len);

    void (*nsleep_set)(uint8_t level);
} drv8311_cfg_t;

typedef drv8311_instance_t *drv8311_handle_t;


void drv8311_init(drv8311_handle_t *handle, drv8311_cfg_t *cfg);

void drv8311_deinit(drv8311_handle_t *handle);

void drv8311_nsleep_ctrl(drv8311_handle_t handle, uint8_t level);

drv8311_dev_sts1_t drv8311_get_status(drv8311_handle_t handle);

uint16_t drv8311_update_synced_period(drv8311_handle_t handle);

void drv8311_csa_ctrl(drv8311_handle_t handle, uint8_t en);

void drv8311_csa_set_gain(drv8311_handle_t handle, DRV8311_CSA_GAIN_t gain);

void drv8311_calc_current(drv8311_handle_t handle, float v_ref,
                          float v_soa, float v_sob, float v_soc,
                          float *i_a, float *i_b, float *i_c);

void drv8311_phase_ctrl(drv8311_handle_t handle,
                        DRV8311_PHASE_MODE_t phase_a, DRV8311_PHASE_MODE_t phase_b, DRV8311_PHASE_MODE_t phase_c);

void drv8311_out_ctrl(drv8311_handle_t handle, uint8_t en);

void drv8311_set_period(drv8311_handle_t handle, uint16_t period);

void drv8311_set_duty(drv8311_handle_t handle, float a, float b, float c);

void drv8311_clear_fault(drv8311_handle_t handle);

#endif //DRV8311_DRIVER_H
