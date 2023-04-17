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

#include <stdlib.h>
#include "drv8311_driver.h"
#include "drv8311_reg.h"

#define RW_CTRL_READ 0x01
#define RW_CTRL_WRITE 0x00

#define SWAP(x, y) do { (x) ^= (y); (y) ^= (x); (x) ^= (y); } while (0)


typedef struct {
    uint8_t parity: 1;
    uint8_t addr: 6;
    uint8_t rw_ctrl: 1;
} drv8311_spi_send_header_t;

typedef struct {
    uint16_t parity: 1;
    uint16_t zero_1: 2;
    uint16_t addr: 8;
    uint16_t device_id: 2;
    uint16_t zero_0: 2;
    uint16_t rw_ctrl: 1;
} drv8311_tspi_send_header_t;

typedef struct {
    uint16_t data: 15;
    uint16_t parity: 1;
} drv8311_send_data_t;

typedef union {
    struct __attribute__ ((__packed__)) {
        drv8311_send_data_t data;
        drv8311_spi_send_header_t header;
    };
    uint8_t bytes[3];
} drv8311_spi_send_pkg_t;

typedef union {
    struct __attribute__ ((__packed__)) {
        drv8311_send_data_t data;
        drv8311_tspi_send_header_t header;
    };
    uint8_t bytes[4];
} drv8311_tspi_send_pkg_t;

typedef union {
    struct __attribute__ ((__packed__)) {
        uint16_t data;
        uint8_t status;
    };
    uint8_t bytes[3];
} drv8311_recv_pkg_t;


static inline uint16_t parity_even_calc(uint16_t data) {
    uint16_t parity = 0;

    while (data) {
        parity ^= data & 1;
        data >>= 1;
    }

    return parity & 1;
}

static inline drv8311_spi_send_header_t drv8311_spi_header_gen(uint8_t rw_mode, uint8_t addr) {
    drv8311_spi_send_header_t header;
    uint16_t *temp;

    header.rw_ctrl = rw_mode;
    header.addr = addr;

    temp = (uint16_t *) &header;

    header.parity = parity_even_calc(*temp);

    return header;
}

static inline drv8311_tspi_send_header_t drv8311_tspi_header_gen(uint8_t rw_mode, uint8_t addr, uint8_t device_id) {
    drv8311_tspi_send_header_t header;
    uint16_t *temp;

    header.rw_ctrl = rw_mode;
    header.device_id = device_id;
    header.addr = addr;

    temp = (uint16_t *) &header;

    header.parity = parity_even_calc(*temp);

    return header;
}

static void
drv8311_transmit(drv8311_handle_t handle, uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len) {
    // swap to big-endian
    for (int i = 0; i < send_len / 2; ++i) {
        SWAP(send_data[i], send_data[send_len - 1 - i]);
    }

    handle->interface.spi_trans(send_data, send_len, rec_data, rec_len);

    // swap to little-endian
    SWAP(rec_data[0], rec_data[2]);
}

static void drv8311_write(drv8311_handle_t handle, uint8_t reg, uint16_t data) {
    drv8311_recv_pkg_t rec;

    if (handle->interface.protel == SPI) {
        drv8311_spi_send_pkg_t data_pkg;
        data_pkg.header = drv8311_spi_header_gen(RW_CTRL_WRITE, reg);
        data_pkg.data.data = data;
        data_pkg.data.parity = parity_even_calc(data);

        drv8311_transmit(handle, data_pkg.bytes, sizeof(data_pkg), rec.bytes, sizeof(rec));
    } else if (handle->interface.protel == tSPI) {
        drv8311_tspi_send_pkg_t data_pkg;
        data_pkg.header = drv8311_tspi_header_gen(RW_CTRL_WRITE, reg, handle->interface.devicd_id);
        data_pkg.data.data = data;
        data_pkg.data.parity = parity_even_calc(data);

        drv8311_transmit(handle, data_pkg.bytes, sizeof(data_pkg), rec.bytes, sizeof(rec));
    }
}

static uint16_t drv8311_read(drv8311_handle_t handle, uint8_t reg) {
    drv8311_recv_pkg_t rec;

    if (handle->interface.protel == SPI) {
        drv8311_spi_send_pkg_t data_pkg;
        data_pkg.header = drv8311_spi_header_gen(RW_CTRL_READ, reg);
        data_pkg.data.data = 0xffff >> 1; // send dummy bits
        data_pkg.data.parity = parity_even_calc(data_pkg.data.data);

        drv8311_transmit(handle, data_pkg.bytes, sizeof(data_pkg), rec.bytes, sizeof(rec));
    } else if (handle->interface.protel == tSPI) {
        drv8311_tspi_send_pkg_t data_pkg;
        data_pkg.header = drv8311_tspi_header_gen(RW_CTRL_READ, reg, handle->interface.devicd_id);
        data_pkg.data.data = 0xffff >> 1; // send dummy bits
        data_pkg.data.parity = parity_even_calc(data_pkg.data.data);

        drv8311_transmit(handle, data_pkg.bytes, sizeof(data_pkg), rec.bytes, sizeof(rec));
    }

    if (handle->interface.parity_check) {
        //todo
    }

    return rec.data;
}

void drv8311_init(drv8311_handle_t *handle, drv8311_cfg_t *cfg) {
    drv8311_reg_t reg;
    drv8311_instance_t *dev = malloc(sizeof(drv8311_instance_t));

    dev->interface.protel = cfg->portal;
    dev->interface.devicd_id = cfg->dev_id;
    dev->interface.parity_check = cfg->parity_check;
    dev->interface.spi_trans = cfg->spi_trans;
    dev->interface.nsleep_set = cfg->nsleep_set;

    drv8311_nsleep_ctrl(dev, 1);

    reg.half_word = 0;
    dev->pwm_gen.mode = cfg->pwmcnt_mode;
    reg.pwmg_ctrl.pwmcntr_mode = cfg->pwmcnt_mode;
    reg.pwmg_ctrl.pwm_osc_sync = cfg->sync_mode;
    if (cfg->sync_mode == USE_SPI_CLK) {
        reg.pwmg_ctrl.spiclk_freq_sync = cfg->spi_clk;
        reg.pwmg_ctrl.spisync_acrcy = cfg->spi_sync_clks;
    }
    drv8311_write(dev, DRV8311_PWMG_CTRL_ADDR, reg.half_word);

    reg.half_word = 0;
    dev->pwm_gen.period = cfg->pwm_period;
    reg.pwmg_period.pwm_prd_out = cfg->pwm_period;
    drv8311_write(dev, DRV8311_PWMG_PERIOD_ADDR, reg.half_word);

    reg.half_word = 0;
    if (cfg->use_csa) {
        reg.csa_ctrl.csa_en = 1;
        reg.csa_ctrl.csa_gain = cfg->csa_gain;
        drv8311_write(dev, DRV8311_CSA_CTRL_ADDR, reg.half_word);
    }

    reg.half_word = 0;
    if (cfg->parity_check) {
        //todo
    }

    *handle = dev;
}

void drv8311_nsleep_ctrl(drv8311_handle_t handle, uint8_t level) {
    handle->interface.nsleep_set(level);
}

drv8311_dev_sts1_t drv8311_get_status(drv8311_handle_t handle) {
    drv8311_reg_t rec;
    rec.half_word = drv8311_read(handle, DRV8311_DEV_STS1_ADDR);

    return rec.dev_sts1;
}

uint16_t drv8311_get_sync_period(drv8311_handle_t handle) {
    drv8311_reg_t rec;
    rec.half_word = drv8311_read(handle, DRV8311_PWM_SYNC_PRD_ADDR);

    return rec.pwm_sync_prd.pwm_sync_prd;
}

void drv8311_csa_ctrl(drv8311_handle_t handle, uint8_t en) {
    drv8311_reg_t csa_ctrl;

    csa_ctrl.half_word = drv8311_read(handle, DRV8311_PWMG_CTRL_ADDR);

    if (en) {
        csa_ctrl.csa_ctrl.csa_en = 1;
    } else {
        csa_ctrl.csa_ctrl.csa_en = 0;
    }

    drv8311_write(handle, DRV8311_CSA_CTRL_ADDR, csa_ctrl.half_word);
}

void drv8311_csa_set_gain(drv8311_handle_t handle, DRV8311_CSA_GAIN_t gain) {
    drv8311_reg_t csa_ctrl;

    csa_ctrl.half_word = drv8311_read(handle, DRV8311_PWMG_CTRL_ADDR);

    csa_ctrl.csa_ctrl.csa_gain = gain;

    drv8311_write(handle, DRV8311_CSA_CTRL_ADDR, csa_ctrl.half_word);
}

void drv8311_phase_ctrl(drv8311_handle_t handle, DRV8311_PHASE_MODE_t phase_a, DRV8311_PHASE_MODE_t phase_b,
                        DRV8311_PHASE_MODE_t phase_c) {
    drv8311_reg_t phase_ctrl;

    phase_ctrl.pwm_state.pwma_state = phase_a;
    phase_ctrl.pwm_state.pwmb_state = phase_b;
    phase_ctrl.pwm_state.pwmc_state = phase_c;

    drv8311_write(handle, DRV8311_PWM_STATE_ADDR, phase_ctrl.half_word);
}

void drv8311_out_ctrl(drv8311_handle_t handle, uint8_t en) {
    drv8311_reg_t pwmg_ctrl;

    pwmg_ctrl.half_word = drv8311_read(handle, DRV8311_PWMG_CTRL_ADDR);

    if (en) {
        pwmg_ctrl.pwmg_ctrl.pwm_en = 1;
    } else {
        pwmg_ctrl.pwmg_ctrl.pwm_en = 0;
    }

    drv8311_write(handle, DRV8311_PWMG_CTRL_ADDR, pwmg_ctrl.half_word);
}

void drv8311_set_period(drv8311_handle_t handle, uint16_t period) {
    drv8311_reg_t period_reg;

    period_reg.pwmg_period.pwm_prd_out = period;
    drv8311_write(handle, DRV8311_PWMG_PERIOD_ADDR, period_reg.half_word);
}

void drv8311_set_duty(drv8311_handle_t handle, float a, float b, float c) {
    drv8311_reg_t pwmg_duty;
    uint16_t cmp_a, cmp_b, cmp_c;

    cmp_a = (uint16_t) ((float) handle->pwm_gen.period * a);
    cmp_b = (uint16_t) ((float) handle->pwm_gen.period * b);
    cmp_c = (uint16_t) ((float) handle->pwm_gen.period * c);

    pwmg_duty.pwmg_x_duty.pwm_duty_outx = cmp_a;
    drv8311_write(handle, DRV8311_PWMG_A_DUTY_ADDR, pwmg_duty.half_word);
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = cmp_b;
    drv8311_write(handle, DRV8311_PWMG_B_DUTY_ADDR, pwmg_duty.half_word);
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = cmp_c;
    drv8311_write(handle, DRV8311_PWMG_C_DUTY_ADDR, pwmg_duty.half_word);
}
