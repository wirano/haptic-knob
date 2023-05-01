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

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "foc.h"
#include "foc_utils.h"
#include "math_table.h"
#include "pid.h"


#define CALI_CURRENT 0.3f


void static dqz_trans(foc_handle_t handler) {
    float i_a, i_b;
    float i_alpha, i_beta;

    // Clark transform
    if (!handler->sensors.i_a) {
        i_a = -handler->sensors.i_b - handler->sensors.i_c;
        i_b = handler->sensors.i_b;
    } else if (!handler->sensors.i_b) {
        i_a = handler->sensors.i_a;
        i_b = -handler->sensors.i_a - handler->sensors.i_c;
    } else if (!handler->sensors.i_c) {
        i_a = handler->sensors.i_a;
        i_b = handler->sensors.i_b;
    } else {
        float err = (1.f / 3) * (handler->sensors.i_a + handler->sensors.i_b + handler->sensors.i_c);
        i_a = handler->sensors.i_a - err;
        i_b = handler->sensors.i_b - err;
    }

    i_alpha = i_a;
    i_beta = _1_SQRT3 * i_a + _2_SQRT3 * i_b;

    float si = sinf(handler->data.angle_elec);
    float co = cosf(handler->data.angle_elec);

    // Park transform
    handler->data.i_d = co * i_alpha + si * i_beta;
    handler->data.i_q = co * i_beta - si * i_alpha;
}

void static svpwm_output(foc_handle_t handler) {
    float angle_elec;
    float u_ref;

    if (handler->data.u_d != 0) {
        u_ref = handler->data.u_q * handler->data.u_q + handler->data.u_d * handler->data.u_d;
        u_ref = sqrtf(u_ref) / handler->motor.volt;

        angle_elec = angle_normalize(handler->data.angle_elec + atan2f(handler->data.u_q, handler->data.u_d));
    } else {
        u_ref = handler->data.u_q / handler->motor.volt;

        angle_elec = angle_normalize(handler->data.angle_elec + _PI_2);
    }

//    u_ref = _limit(u_ref, 0, _1_SQRT3);

    uint8_t sector = (uint8_t) floorf(angle_elec / _PI_3) + 1;

    float alpha = angle_elec - (float) (sector - 1) * _PI_3;
    float T1 = _SQRT3 * u_ref * sinf(_PI_3 - alpha);
    float T2 = _SQRT3 * u_ref * sinf(alpha);
    float T0 = 1.f - T1 - T2;

    float Ta, Tb, Tc;
    switch (sector) {
        case 1:
            Ta = T1 + T2 + T0 / 2.f;
            Tb = T2 + T0 / 2.f;
            Tc = T0 / 2.f;
            break;
        case 2:
            Ta = T1 + T0 / 2.f;
            Tb = T1 + T2 + T0 / 2.f;
            Tc = T0 / 2.f;
            break;
        case 3:
            Ta = T0 / 2.f;
            Tb = T1 + T2 + T0 / 2.f;
            Tc = T2 + T0 / 2.f;
            break;
        case 4:
            Ta = T0 / 2.f;
            Tb = T1 + T0 / 2.f;
            Tc = T1 + T2 + T0 / 2.f;
            break;
        case 5:
            Ta = T2 + T0 / 2.f;
            Tb = T0 / 2.f;
            Tc = T1 + T2 + T0 / 2.f;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2.f;
            Tb = T0 / 2.f;
            Tc = T1 + T0 / 2.f;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }

    handler->hal.set_pwm(Ta, Tb, Tc);
}

void foc_update_sensor(foc_handle_t handle) {
    uint32_t time_now = handle->hal.micros();
    float Ts = (float) (time_now - handle->status.vel_prev_time) * 1e-6f;
    handle->status.vel_prev_time = time_now;

    handle->hal.update_sensors(handle);

    float angle = handle->sensors.angle_abs - handle->data.angle_zero_offset;
    angle = angle > 0 ? angle : angle + _2PI;

    /* get velocity */
    float angle_delta = angle - handle->data.angle_prev;
    if (fabsf(angle_delta) > 0.8f * _2PI) handle->data.full_rotations += angle_delta > 0 ? -1 : 1;

    float real_delta = (float) (handle->data.full_rotations - handle->data.full_rotations_prev) * _2PI + angle_delta;
    handle->data.velocity = real_delta / Ts;
    handle->data.full_rotations_prev = handle->data.full_rotations;
    /* end get velocity */

    handle->data.angle_mech += real_delta;

    handle->data.angle_prev = angle;
    handle->data.angle_elec = angle * handle->motor.pole_pairs;
}

void foc_current_loop(foc_handle_t handler) {
    dqz_trans(handler);

    handler->data.i_d = lpf(&handler->lpf.i_d, handler->data.i_d);
    handler->data.i_q = lpf(&handler->lpf.i_q, handler->data.i_q);

    handler->data.u_q = pid_calc(handler->pid_ctrl.current_q, handler->data.i_q,
                                                handler->target.current);
    handler->data.u_d = pid_calc(handler->pid_ctrl.current_d, handler->data.i_d, 0);

    svpwm_output(handler);
}

void foc_velocity_loop(foc_handle_t handler) {
    float speed = handler->data.velocity;
    float target = handler->target.velocity;

    handler->target.current = pid_calc(handler->pid_ctrl.velocity_loop, speed, target);
}

void foc_angle_loop(foc_handle_t handler) {
    float angle = handler->data.angle_mech;
    float target = handler->target.angle;

    handler->target.current = pid_calc(handler->pid_ctrl.angle_loop, angle, target);
}

void foc_ctrl_loop(foc_handle_t handle) {
    static uint8_t cnt = 0;

    foc_update_sensor(handle);

    if (!handle->status.enabled) return;

    switch (handle->status.mode) {
        case FOC_MODE_POS:
            if (cnt % 10 == 0) {
                foc_angle_loop(handle);
            }
            break;
        case FOC_MODE_VEL:
            if (cnt % 10 == 0) {
                foc_velocity_loop(handle);
            }
            break;
        default:
            break;
    }

    foc_current_loop(handle);

    if (cnt % 100 == 0) cnt = 0;

    cnt++;
}

void foc_init(foc_handle_t *handle, foc_config_t *cfg) {
    foc_instance_t *foc = malloc(sizeof(foc_instance_t));
    memset(foc, 0, sizeof(foc_instance_t));

    foc->status.mode = cfg->mode;

    foc->motor.pole_pairs = cfg->pole_pairs;
    foc->motor.volt = cfg->motor_volt;

    foc->pid_ctrl.current_d = cfg->current_d;
    foc->pid_ctrl.current_q = cfg->current_q;
    foc->pid_ctrl.velocity_loop = cfg->velocity_loop;
    foc->pid_ctrl.angle_loop = cfg->angle_loop;

    foc->hal.set_pwm = cfg->hal.set_pwm;
    foc->hal.update_sensors = cfg->hal.update_sensors;
    foc->hal.driver_enable = cfg->hal.driver_enable;
    foc->hal.delay = cfg->hal.delay;
    foc->hal.micros = cfg->hal.micros;

    foc->lpf.i_d.micros = cfg->hal.micros;
    foc->lpf.i_q.micros = cfg->hal.micros;
    foc->lpf.angle.micros = cfg->hal.micros;

    foc->lpf.i_d.Tf = 0.01f;
    foc->lpf.i_q.Tf = 0.01f;
    foc->lpf.angle.Tf = 0.01f;

    *handle = foc;

    foc->status.enabled = 0;
    foc->hal.driver_enable(0);
}

void foc_enable(foc_handle_t handle, uint8_t en) {
    if (en) {
        // clear old data
        handle->data.velocity = 0;
        pid_reset(handle->pid_ctrl.current_d);
        pid_reset(handle->pid_ctrl.current_q);
        pid_reset(handle->pid_ctrl.velocity_loop);
        pid_reset(handle->pid_ctrl.angle_loop);

        handle->status.enabled = 1;
        handle->hal.driver_enable(1);
    } else {
        handle->status.enabled = 0;
        handle->hal.driver_enable(0);
    }
}

void foc_angle_auto_zeroing(foc_handle_t handle) {
    uint8_t en = handle->status.enabled;

    // stop foc ctrl loop
    handle->status.enabled = 0;
    handle->hal.driver_enable(1);

    handle->data.u_q = CALI_CURRENT;
    handle->data.u_d = 0;
    handle->data.angle_elec = -_PI_2; // -90 + 90 = 0
    svpwm_output(handle);
    handle->hal.delay(300);

    handle->hal.update_sensors(handle);

    handle->data.angle_zero_offset = handle->sensors.angle_abs;

    handle->data.u_q = 0;
    handle->data.u_d = 0;
    handle->data.angle_elec = 0;
    handle->data.angle_mech = 0;
    svpwm_output(handle);

    // reset speed
    handle->data.velocity = 0;

    // restore foc ctrl state
    handle->status.enabled = en;
}
