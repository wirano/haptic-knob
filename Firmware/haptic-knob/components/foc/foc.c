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
#include <stdint.h>
#include <stdio.h>
#include "foc.h"
#include "math_table.h"

void static dqz_trans(foc_handler_t handler) {
    float i_alpha, i_beta;

    // Clark transform
    if (!handler->sensors.i_a) {
        float i_a = -handler->sensors.i_b - handler->sensors.i_c;

        i_alpha = _SQRT2_SQRT3 * handler->sensors.i_a;
        i_beta = _1_SQRT2 * i_a + _SQRT2 * handler->sensors.i_b;
    } else if (!handler->sensors.i_b) {
        float i_b = -handler->sensors.i_a - handler->sensors.i_c;

        i_alpha = _SQRT2_SQRT3 * handler->sensors.i_a;
        i_beta = _1_SQRT2 * handler->sensors.i_a + _SQRT2 * i_b;
    } else if (!handler->sensors.i_c) {
        i_alpha = _SQRT2_SQRT3 * handler->sensors.i_a;
        i_beta = _1_SQRT2 * handler->sensors.i_a + _SQRT2 * handler->sensors.i_b;
    } else {
        float err = (1.f / 3) * (handler->sensors.i_a + handler->sensors.i_b + handler->sensors.i_c);
        float i_a = handler->sensors.i_a - err;
        float i_b = handler->sensors.i_b - err;

        i_alpha = _SQRT2_SQRT3 * i_a;
        i_beta = _1_SQRT2 * i_a + _SQRT2 * i_b;
    }

    float si = sinf(handler->data.angle_elec);
    float co = cosf(handler->data.angle_elec);

    // Park transform
    handler->data.i_d = co * i_alpha + si * i_beta;
    handler->data.i_q = co * i_alpha - si * i_beta;
}

void static svpwm_output(foc_handler_t handler){
    uint8_t sector = 0;
    float u_a,u_b,u_c,u_vec;

    float u_2 = handler->data.u_alpha * handler->data.u_alpha + handler->data.u_beta * handler->data.u_beta;
    u_vec = sqrtf(u_2) / (float)handler->motor.volt;

    switch (sector) {
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
        case 6:
            break;
        default:
            u_a = 0;
            u_b = 0;
            u_c = 0;
    }

    handler->hal.set_pwm(u_a, u_b, u_c);
}

void foc_loop(foc_handler_t handler) {
    handler->hal.update_sensors(handler);

    dqz_trans(handler);

    svpwm_output(handler);
}
