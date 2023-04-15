#include "driver/gpio.h"
#include "drv8311_reg.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "lv_port.h"
#include "drv8311_driver.h"
#include "mt6701_driver.h"
#include "spi.h"

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

    drv8311_reg_t rec;
    rec.half_word = drv8311_read(drv8311, DRV8311_PWMG_PERIOD_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311, DRV8311_PWMG_A_DUTY_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    drv8311_write(drv8311, DRV8311_PWMG_PERIOD_ADDR, 400);
    drv8311_write(drv8311,DRV8311_PWMG_A_DUTY_ADDR,200);
    rec.half_word = drv8311_read(drv8311, DRV8311_PWMG_PERIOD_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311, DRV8311_PWMG_A_DUTY_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    drv8311_write(drv8311, DRV8311_PWMG_CTRL_ADDR, 1U << 10);
    rec.half_word = drv8311_read(drv8311, DRV8311_PWMG_CTRL_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));


    ESP_LOGI(TAG, "%f\n", mt6701_get_angle(mt6701));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        rec.half_word = drv8311_read(drv8311, DRV8311_PWM_STATE_ADDR);
        ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    }
}
