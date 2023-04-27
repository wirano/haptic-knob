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

static struct {
    struct arg_str *loop;
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

    const char *loop = set_kp_args.loop->sval[0];
    double kp = set_kp_args.kp->dval[0];

    switch (*loop) {

    }

    return 0;
}

void register_hk_cmd(void) {
    set_kp_args.loop = arg_str1(NULL, NULL, "<s>", "which loop to set");
    set_kp_args.kp = arg_dbl1(NULL, NULL, "<d>", "kp value");
    set_kp_args.end = arg_end(2);

    esp_console_cmd_t command = {
            .command = "set_kp",
            .help = "set pid kp",
            .func = &set_kp_cmd,
            .argtable = &set_kp_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&command));

    //todo
}
