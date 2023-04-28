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
// Created by wirano on 23-4-27.
//

#include "hk_cmd.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "foc_platform.h"

static struct {
    struct arg_int *loop;
    struct arg_dbl *kp;
    struct arg_end *end;
} set_kp_args;

static int set_kp_cmd(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **) &set_kp_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_kp_args.end, argv[0]);
        return 1;
    }
    assert(set_kp_args.loop->count == 1);
    assert(set_kp_args.kp->count == 1);

    int loop = set_kp_args.loop->ival[0];
    double kp = set_kp_args.kp->dval[0];

    switch (loop) {
        case 0:
            foc->pid_ctrl.current_d->Kp = kp;
            break;
        case 1:
            foc->pid_ctrl.current_q->Kp = kp;
            break;
        case 2:
            foc->pid_ctrl.velocity_loop->Kp = kp;
            break;
        case 3:
            foc->pid_ctrl.angle_loop->Kp = kp;
            break;
    }

    return 0;
}

static struct {
    struct arg_int *loop;
    struct arg_dbl *ki;
    struct arg_end *end;
} set_ki_args;

static int set_ki_cmd(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **) &set_ki_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_ki_args.end, argv[0]);
        return 1;
    }
    assert(set_ki_args.loop->count == 1);
    assert(set_ki_args.ki->count == 1);

    int loop = set_ki_args.loop->ival[0];
    double ki = set_ki_args.ki->dval[0];

    switch (loop) {
        case 0:
            foc->pid_ctrl.current_d->Ki = ki;
            break;
        case 1:
            foc->pid_ctrl.current_q->Ki = ki;
            break;
        case 2:
            foc->pid_ctrl.velocity_loop->Ki = ki;
            break;
        case 3:
            foc->pid_ctrl.angle_loop->Ki = ki;
            break;
    }

    return 0;
}

static struct {
    struct arg_int *loop;
    struct arg_dbl *kd;
    struct arg_end *end;
} set_kd_args;

static int set_kd_cmd(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **) &set_kd_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_kd_args.end, argv[0]);
        return 1;
    }
    assert(set_kd_args.loop->count == 1);
    assert(set_kd_args.kd->count == 1);

    int loop = set_kd_args.loop->ival[0];
    double kd = set_kd_args.kd->dval[0];

    switch (loop) {
        case 0:
            foc->pid_ctrl.current_d->Kd = kd;
            break;
        case 1:
            foc->pid_ctrl.current_q->Kd = kd;
            break;
        case 2:
            foc->pid_ctrl.velocity_loop->Kd = kd;
            break;
        case 3:
            foc->pid_ctrl.angle_loop->Kd = kd;
            break;
    }

    return 0;
}

void register_hk_cmd(void) {
    set_kp_args.loop = arg_int1(NULL, NULL, "<i>", "which loop to set: 0:id 1:iq 2:speed 3:pos");
    set_kp_args.kp = arg_dbl1(NULL, NULL, "<d>", "kp value");
    set_kp_args.end = arg_end(2);

    esp_console_cmd_t kp_cmd = {
            .command = "set_kp",
            .help = "set pid kp",
            .func = &set_kp_cmd,
            .argtable = &set_kp_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&kp_cmd));

    set_ki_args.loop = arg_int1(NULL, NULL, "<i>", "which loop to set: 0:id 1:iq 2:speed 3:pos");
    set_ki_args.ki = arg_dbl1(NULL, NULL, "<d>", "ki value");
    set_ki_args.end = arg_end(2);

    esp_console_cmd_t ki_cmd = {
            .command = "set_ki",
            .help = "set pid ki",
            .func = &set_ki_cmd,
            .argtable = &set_ki_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ki_cmd));

    set_kd_args.loop = arg_int1(NULL, NULL, "<i>", "which loop to set: 0:id 1:iq 2:speed 3:pos");
    set_kd_args.kd = arg_dbl1(NULL, NULL, "<d>", "kd value");
    set_kd_args.end = arg_end(2);

    esp_console_cmd_t kd_cmd = {
            .command = "set_kd",
            .help = "set pid kd",
            .func = &set_kd_cmd,
            .argtable = &set_kd_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&kd_cmd));
}
