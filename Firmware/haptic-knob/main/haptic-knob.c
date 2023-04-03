#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lv_port.h"

#define CFG1    18U
#define CFG2    8U
#define CFG3    17U

void app_main(void)
{
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

    gpio_set_level(CFG2,0);
    gpio_set_level(CFG3,0);
    gpio_set_level(CFG1,1);

    lvgl_init();

    vTaskDelay(pdMS_TO_TICKS(5000));

    gpio_set_level(CFG2,0);
    gpio_set_level(CFG3,1);
    gpio_set_level(CFG1,0);

    while (1){
       vTaskDelay(pdMS_TO_TICKS(100));
    }
}
