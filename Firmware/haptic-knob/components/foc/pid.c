// MIT License
//
// Copyright (c) 2020-2023 wirano
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
// Created by wirano on 20-5-17.
//

#include "pid.h"
#include <math.h>


float PID_IncrementalRealize(pid_incremental_t *PID, float inVal, float target) {
    uint8_t Ki_enable = 1;
    uint32_t time_now = PID->get_micros();
    float Ts = (float) (time_now - PID->time_prev) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5) Ts = 1e-6f;

    PID->err = target - inVal;

    if (fabsf(PID->err) > PID->Ki_separation) Ki_enable = 0;

    float proportional = PID->Kp * (PID->err - PID->err_prev);

    float integral = 0;
    if (Ki_enable) integral = PID->Ki * Ts * PID->err;

    float derivative = PID->Kd * (PID->err - 2 * PID->err_prev + PID->err_last) / Ts;

    float increment = proportional + integral + derivative;

    PID->err_last = PID->err_prev;
    PID->err_prev = PID->err;

    return increment;
}

float PID_PositionalRealize(pid_positional_t *PID, float inVal, float target) {
    uint32_t time_now = PID->get_micros();
    float Ts = (float) (time_now - PID->time_prev) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5) Ts = 1e-3f;

    PID->err = target - inVal;
//    if (PID->err < PID->Ki_Separation && PID->err > -PID->Ki_Separation)
//        PID->integral += PID->err;
//    else
//        PID->integral = PID->integral;

    if (PID->integral > PID->Ki_Limit)
        PID->integral = PID->Ki_Limit;
    else if (PID->integral < -PID->Ki_Limit)
        PID->integral = -PID->Ki_Limit;


    float proportional = PID->Kp * PID->err;

    float integral_new = PID->integral + (PID->err + PID->err_prev) / 2.f;
    float integral = PID->Ki * Ts * integral_new;

    float derivative = PID->Kd * (PID->err - PID->err_prev) / Ts;

    float output = proportional + integral + derivative;

    PID->err_prev = PID->err;
    PID->integral = integral_new;

    return output;
}

