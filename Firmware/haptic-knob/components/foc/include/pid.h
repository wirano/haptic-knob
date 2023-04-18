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

#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ki_separation;

    float err;
    float err_prev;
    float err_last;
    uint32_t time_prev;

    uint32_t (*get_micros)(void);
} pid_incremental_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ki_Limit;
    float Ki_Separation;

    float err;
    float err_prev;
    float integral;
    uint32_t time_prev;

    uint32_t (*get_micros)(void);
} pid_positional_t;


float PID_IncrementalRealize(pid_incremental_t *PID, float inVal, float target);

float PID_PositionalRealize(pid_positional_t *PID, float inVal, float target);


#endif //PID_H
