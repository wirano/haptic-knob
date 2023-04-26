#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"

#include "lv_port.h"
#include "mt6701_driver.h"
#include "foc_platform.h"
#include "foc.h"
#include <stdio.h>

#define CFG1    17U
#define CFG2    18U
#define CFG3    8U

#define TAG "main"

void app_main(void) {
    gpio_config_t gpio_conf;

    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pull_up_en = false;
    gpio_conf.pull_down_en = false;
    gpio_conf.pin_bit_mask = 1 << CFG1;
    gpio_config(&gpio_conf);

    gpio_conf.pin_bit_mask = 1 << CFG2;
    gpio_config(&gpio_conf);

    gpio_conf.pin_bit_mask = 1 << CFG3;
    gpio_config(&gpio_conf);

    lvgl_init();

    spi_dev_init(); // init drv8311 before query for 12V

    // 12V
    gpio_set_level(CFG2, 0);
    gpio_set_level(CFG3, 1);
    gpio_set_level(CFG1, 0);

    platform_foc_init();

//    drv8311_clear_fault(drv8311);
//    drv8311_out_ctrl(drv8311,1);
//    float d = 0.3f;
    while (1) {
//        ESP_LOGI(TAG,"%f", mt6701_get_angle_deg(mt6701));
//        drv8311_ge

////        drv8311_set_duty(drv8311,d,0,d);
//        vTaskDelay(pdMS_TO_TICKS(10));
//        drv8311_set_duty(drv8311,0,d,0);
        vTaskDelay(pdMS_TO_TICKS(10));
//        d += 0.05f;
//        if(d >= 1.f) d = 0;
    }
}
