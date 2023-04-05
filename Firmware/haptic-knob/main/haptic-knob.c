#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lv_port.h"
#include "drv8311_driver.h"
#include "spi.h"

#define CFG1    18U
#define CFG2    8U
#define CFG3    17U

#define TAG "main"

void app_main(void) {
    gpio_config_t gpio_conf;

    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pull_up_en = true;
    gpio_conf.pull_down_en = false;
    gpio_conf.pin_bit_mask = 1 << CFG1;
    gpio_config(&gpio_conf);

    gpio_conf.pull_up_en = false;
    gpio_conf.pin_bit_mask = 1 << CFG2;
    gpio_config(&gpio_conf);

    gpio_conf.pin_bit_mask = 1 << CFG3;
    gpio_config(&gpio_conf);

    gpio_set_level(CFG2, 0);
    gpio_set_level(CFG3, 0);
    gpio_set_level(CFG1, 1);

//    lvgl_init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    gpio_set_level(CFG2, 0);
    gpio_set_level(CFG3, 1);
    gpio_set_level(CFG1, 0);

    spi_dev_init();

    drv8311_dev_sts1_t rec = drv8311_get_status(drv8311);

    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));

//    gpio_conf.pin_bit_mask = (uint64_t) 1 << 35;
//    gpio_config(&gpio_conf);
//    gpio_conf.pin_bit_mask = (uint64_t) 1 << 36;
//    gpio_config(&gpio_conf);
//    gpio_conf.pin_bit_mask = (uint64_t) 1 << 37;
//    gpio_config(&gpio_conf);
//    gpio_conf.pin_bit_mask = (uint64_t) 1 << 39;
//    gpio_config(&gpio_conf);
//    gpio_conf.pin_bit_mask = (uint64_t) 1 << 40;
//    gpio_config(&gpio_conf);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
//        gpio_set_level(35, 1);
//        gpio_set_level(36, 1);
//        gpio_set_level(37, 1);
//        gpio_set_level(39, 1);
//        gpio_set_level(40, 1);
//        vTaskDelay(pdMS_TO_TICKS(500));
//        gpio_set_level(35, 0);
//        gpio_set_level(36, 0);
//        gpio_set_level(37, 0);
//        gpio_set_level(39, 0);
//        gpio_set_level(40, 0);
    }
}
