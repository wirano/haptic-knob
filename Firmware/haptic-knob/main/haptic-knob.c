#include "driver/gpio.h"
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

    // 12V
    gpio_set_level(CFG2, 0);
    gpio_set_level(CFG3, 1);
    gpio_set_level(CFG1, 0);

    gpio_conf.pin_bit_mask = 1ULL << 38U;  // mt6701 cs
    gpio_config(&gpio_conf);
    gpio_set_level(38,0);

    gpio_conf.pin_bit_mask = 1ULL << 2U;  // drv8311 cs
    gpio_config(&gpio_conf);

    gpio_conf.pin_bit_mask = 1ULL << 39U; // drv8311 nsleep
    gpio_config(&gpio_conf);
    gpio_set_level(39, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(39, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(39, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    spi_dev_init();
//    drv8311_get_status(drv8311);

    drv8311_reg_t rec;
    rec.half_word = drv8311_read(drv8311,DRV8311_DEV_STS1_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_OT_STS_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_SUP_STS_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_DRV_STS_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_SYS_STS_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_PWM_SYNC_PRD_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_FLT_MODE_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_SYSF_CTRL_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_DRVF_CTRL_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_FLT_TCTRL_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_FLT_CLR_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    rec.half_word = drv8311_read(drv8311,DRV8311_PWMG_PERIOD_ADDR);
    ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));

    ESP_LOGI(TAG, "%f\n", mt6701_get_angle(mt6701));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        rec.half_word = drv8311_read(drv8311,DRV8311_DEV_STS1_ADDR);
        ESP_LOG_BUFFER_HEX(TAG, &rec, sizeof(rec));
    }
}
