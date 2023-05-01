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

#ifndef FOC_H
#define FOC_H

#include "pid.h"
#include "foc_utils.h"
#include <stdint.h>

typedef struct foc_hal_t foc_hal_t;

typedef struct foc_instance_t foc_instance_t;
typedef foc_instance_t *foc_handle_t;

typedef enum {
    FOC_MODE_POS,
    FOC_MODE_VEL,
    FOC_MODE_TOR,
} foc_mode_t;

struct foc_hal_t {
    void (*delay)(uint32_t ms);

    void (*update_sensors)(foc_handle_t handler);

    void (*driver_enable)(uint8_t en);

    void (*set_pwm)(float duty_a, float duty_b, float duty_c);

    uint32_t (*micros)(void);
};

typedef struct {
    uint8_t pole_pairs;
    float motor_volt;
    foc_mode_t mode;
    pid_controller_t *current_q;
    pid_controller_t *current_d;
    pid_controller_t *velocity_loop;
    pid_controller_t *angle_loop;
    foc_hal_t hal;
} foc_config_t;

struct foc_instance_t {
    foc_hal_t hal;
    struct {
        uint8_t enabled;
        uint32_t vel_prev_time;
        foc_mode_t mode;
    } status;
    struct {
        uint8_t pole_pairs;
        float volt;
    } motor;
    struct {
        float angle_abs; // absolut angle in radian
        float i_a;
        float i_b;
        float i_c;
    } sensors;
    struct {
        float i_q;
        float i_d;
        float u_q;
        float u_d;
        float angle_zero_offset;
        float angle_prev;
        float angle_mech;
        float angle_elec;
        int32_t full_rotations;
        int32_t full_rotations_prev;
        float velocity; // rad/s
    } data;
    struct {
        pid_controller_t *current_q;
        pid_controller_t *current_d;
        pid_controller_t *velocity_loop;
        pid_controller_t *angle_loop;
    } pid_ctrl;
    struct {
        float current;
        float velocity;
        float angle;
    } target;
    struct {
        foc_lpf_t i_q;
        foc_lpf_t i_d;
        foc_lpf_t angle;
    }lpf;
};

void foc_ctrl_loop(foc_handle_t handle);

void foc_init(foc_handle_t *handle, foc_config_t *cfg);

void foc_enable(foc_handle_t handle, uint8_t en);

void foc_angle_auto_zeroing(foc_handle_t handle);

#endif //FOC_H
