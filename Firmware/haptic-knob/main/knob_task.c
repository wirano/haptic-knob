//
// Created by wirano on 23-5-2.
//

#include "knob_task.h"
#include "foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "knob.h"
#include "foc_platform.h"

knob_handle_t knob = NULL;

static void knob_task(void *args) {

    while (1) {
        knob_loop(knob);
//        printf("/*%f,%f,%f,%f,%f,%f,%f,%f*/\n", foc->data.i_d, foc->data.i_q, foc->data.u_d, foc->data.u_q,
//               foc->data.angle_mech, foc->target.current, foc->target.velocity, foc->target.angle);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelete(NULL);
}

void knob_task_init(void) {
    platform_foc_init();

    knob_init(&knob, foc);

    knob_set_mode(knob, MODE_ENCODER);
//    knob->foc->target.current = 0.5;
//    foc->status.mode = FOC_MODE_TOR;
//    foc_enable(knob->foc, 1);

    xTaskCreatePinnedToCore(knob_task, "knob", 4096, NULL, 32, NULL, 1);
}
