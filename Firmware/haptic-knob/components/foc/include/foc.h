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

#include <stdint.h>

typedef struct foc_hal_t foc_hal_t;

typedef struct foc_instance_t foc_instance_t;
typedef foc_instance_t *foc_handler_t;

struct foc_hal_t {
    void (*delay)(uint32_t ms);

    void (*update_sensors)(foc_handler_t handler);

    void (*set_pwm)(float u_a, float u_b, float u_c);
};

struct foc_instance_t {
    foc_hal_t hal;
    struct {
        uint8_t pole_pairs;
        uint8_t volt;
    }motor;
    struct {
        float angle_abs;
        float i_a;
        float i_b;
        float i_c;
    } sensors;
    struct {
        float i_q;
        float i_d;
        float u_alpha;
        float u_beta;
        float angle_mech;
        float angle_elec;
    } data;
};

void foc_loop(foc_handler_t handler);
