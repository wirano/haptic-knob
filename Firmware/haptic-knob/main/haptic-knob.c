#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "lv_port.h"
#include "drv8311_driver.h"
#include "mt6701_driver.h"
#include "foc_peripherals.h"

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

    spi_dev_init(); // init drv8311 before query for 12V

    // 12V
    gpio_set_level(CFG2, 0);
    gpio_set_level(CFG3, 1);
    gpio_set_level(CFG1, 0);

    ESP_LOGI(TAG, "%f\n", mt6701_get_angle_deg(mt6701));

    drv8311_out_ctrl(drv8311, 1);

    float duty = 0.1f;
    drv8311_set_duty(drv8311, duty, 0, 0);

    while (1) {
        ESP_LOGI(TAG, "%d %d %d", adc_raw[0], adc_raw[1], adc_raw[2]);
        vTaskDelay(pdMS_TO_TICKS(100));
//        ESP_LOGI(TAG, "%f\n", mt6701_get_angle(mt6701));
    }
}
