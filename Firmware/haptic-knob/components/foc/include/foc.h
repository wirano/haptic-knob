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
#include <stdint.h>

typedef struct foc_hal_t foc_hal_t;

typedef struct foc_instance_t foc_instance_t;
typedef foc_instance_t *foc_handler_t;

typedef struct {
    uint8_t pole_pairs;
    float motor_volt;
    pid_incremental_t *current_q;
    pid_incremental_t *current_d;
    pid_incremental_t *velocity_loop;
    pid_incremental_t *angle_loop;

    void (*delay)(uint32_t ms);

    void (*update_sensors)(foc_handler_t handler);

    void (*driver_enable)(uint8_t en);

    void (*set_pwm)(float duty_a, float duty_b, float duty_c);
} foc_config_t;

struct foc_hal_t {
    void (*delay)(uint32_t ms);

    void (*update_sensors)(foc_handler_t handler);

    void (*driver_enable)(uint8_t en);

    void (*set_pwm)(float duty_a, float duty_b, float duty_c);
};

struct foc_instance_t {
    foc_hal_t hal;
    struct {
        uint8_t enabled;
    } status;
    struct {
        uint8_t pole_pairs;
        float volt;
    } motor;
    struct {
        float angle_abs; // absolut angle in radian
        float angle_zero_offset;
        float i_a;
        float i_b;
        float i_c;
    } sensors;
    struct {
        float i_q;
        float i_d;
        float u_q;
        float u_d;
        float angle_mech;
        float angle_elec;
    } data;
    struct {
        pid_incremental_t *current_q;
        pid_incremental_t *current_d;
        pid_incremental_t *velocity_loop;
        pid_incremental_t *angle_loop;
    } pid_ctrl;
    struct {
        float current;
        float velocity;
        float angle;
    } target;
};

void foc_ctrl_loop(foc_handler_t handler);

#endif //FOC_H
