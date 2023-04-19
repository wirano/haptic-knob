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
#include "foc.h"
#include "foc_utils.h"
#include "math_table.h"
#include "pid.h"


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

void static svpwm_output(foc_handler_t handler) {
    float angle_elec;
    float u_ref;

    if (handler->data.u_alpha != 0) {
        u_ref = handler->data.u_alpha * handler->data.u_alpha + handler->data.u_beta * handler->data.u_beta;
        u_ref = sqrtf(u_ref) / handler->motor.volt;

        angle_elec = angle_normalize(handler->data.angle_elec) + atan2f(handler->data.u_alpha, handler->data.u_beta);
    } else {
        u_ref = handler->data.u_beta / handler->motor.volt;

        angle_elec = angle_normalize(handler->data.angle_elec + _PI_2);
    }

    uint8_t sector = (uint8_t) floorf(angle_elec / _PI_3) + 1;

    float alpha = angle_elec - (float) sector * _PI_3;
    float T1 = _SQRT3 * u_ref * sinf(_PI_3 - alpha);
    float T2 = _SQRT3 * u_ref * sinf(alpha);
    float T0 = 1.f - T1 - T2;

    float Ta, Tb, Tc;
    switch (sector) {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }

    handler->hal.set_pwm(Ta, Tb, Tc);
}

void foc_loop(foc_handler_t handler) {
    handler->hal.update_sensors(handler);

    dqz_trans(handler);

    svpwm_output(handler);
}
