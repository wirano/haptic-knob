#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "lvgl/lvgl.h"

#include "lv_port_disp.h"
#include "lv_port_indev.h"

#include "lv_demos.h"

#include "ui.h"

#define LV_TICK_PERIOD_MS 10

#define TAG "lv_port"


TaskHandle_t lvgl = NULL;

//static void lv_tick_task(void *args);

static void lvgl_task(void *args);

static void lvgl_task(void *args) {

    ESP_LOGI(TAG, "Initializing lv");
    lv_init();

    ESP_LOGI(TAG, "Initializing lv_disp");
    lv_port_disp_init();

    lv_port_indev_init();

    // use LV_TICK_CUSTOM instead, in lv_conf.h

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
//    const esp_timer_create_args_t periodic_timer_args = {
//            .callback = &lv_tick_task,
//            .name = "periodic_gui"
//    };
//    esp_timer_handle_t periodic_timer;
//    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
//    ESP_ERROR_CHECK(
//            esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

//    lv_demo_benchmark();
//    lv_demo_music();
    ui_init();
    while (1) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(LV_TICK_PERIOD_MS));
    }

    vTaskDelete(NULL);
}

void lvgl_init() {

    ESP_LOGI(TAG, "Creating lvgl task");
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 8192, NULL, 0, &lvgl, 0);
}

// use LV_TICK_CUSTOM instead, in lv_conf.h
//static void lv_tick_task(void *arg) {
//    (void) arg;
//
//    lv_tick_inc(LV_TICK_PERIOD_MS);
//}
