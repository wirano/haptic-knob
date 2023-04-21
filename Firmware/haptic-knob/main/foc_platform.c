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
// Created by wirano on 23-4-5.
//

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_private/adc_private.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "foc_platform.h"
#include "hal/adc_types.h"
#include "mt6701_driver.h"
#include "drv8311_driver.h"
#include "foc.h"

#define SPI_BUS SPI3_HOST
#define SPI_FREQ SPI_MASTER_FREQ_10M
#define SPI_CLK_PIN 40U
#define MOSI_PIN 41U
#define MISO_PIN 42U

#define MT6701_CS_PIN 38U
#define DRV8311_CS_PIN 2U
#define DRV8311_NSLEEP_PIN 39U
#define DRV8311_PWM_SYNC_PIN 1U
#define DRV8311_VREF 3.261010526f

#define PHASE_A_CH ADC_CHANNEL_5
#define PHASE_B_CH ADC_CHANNEL_4
#define PHASE_C_CH ADC_CHANNEL_3

#define SWAP(x, y) do { (x) ^= (y); (y) ^= (x); (x) ^= (y); } while (0)

#define TAG "foc_platform"


foc_handle_t foc;

spi_device_handle_t drv8311_dev;
spi_device_handle_t mt6701_dev;

mcpwm_timer_handle_t mcpwm_timer;
mcpwm_oper_handle_t mcpwm_oper;
mcpwm_cmpr_handle_t mcpwm_cmp;
mcpwm_gen_handle_t mcpwm_gen;

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle;
int adc_raw[3];

drv8311_handle_t drv8311;
mt6701_handle_t mt6701;

uint32_t pid_get_micros(void) {
    return esp_timer_get_time();
}

pid_incremental_t i_d = {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .get_micros = pid_get_micros,
};

pid_incremental_t i_q = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .get_micros = pid_get_micros,
};

pid_incremental_t speed = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .get_micros = pid_get_micros,
};

pid_incremental_t angle = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,
        .get_micros = pid_get_micros,
};

//void drv8311_cs_low(spi_transaction_t *trans) {
//    gpio_set_level(DRV8311_CS_PIN, 0);
//}
//
//void drv8311_cs_high(spi_transaction_t *trans) {
//    gpio_set_level(DRV8311_CS_PIN, 1);
//}

void drv8311_spi_trans(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len) {
    union {
        uint8_t bytes[4];
        uint32_t word;
    } rec_buffer;

    spi_transaction_t t = {
            .tx_buffer = send_data,
            .length = send_len * 8,
            .rx_buffer = rec_buffer.bytes,
            .rxlength = (rec_len + 1) * 8,
    };

    spi_device_acquire_bus(drv8311_dev, portMAX_DELAY);
    ESP_ERROR_CHECK(spi_device_transmit(drv8311_dev, &t));
    spi_device_release_bus(drv8311_dev);
//    ESP_LOG_BUFFER_HEX(TAG, rec_buffer.bytes, rec_len + 1);

    // workaround for SDO pin output ahead one bit
    // swap to fit little-endian bit order
    for (int i = 0; i < sizeof(rec_buffer) / 2; ++i) {
        SWAP(rec_buffer.bytes[i], rec_buffer.bytes[3 - i]);
    }
    // shift right
    rec_buffer.word >>= 1U;
    // swap back to SPI MSB order
    for (int i = 0; i < sizeof(rec_buffer) / 2; ++i) {
        SWAP(rec_buffer.bytes[i], rec_buffer.bytes[3 - i]);
    }
    memcpy(rec_data, rec_buffer.bytes + 1, rec_len);
//    ESP_LOG_BUFFER_HEX(TAG, send_data, send_len);
//    ESP_LOG_BUFFER_HEX(TAG, rec_buffer.bytes, rec_len + 1);
}

void mt6701_spi_trans(uint8_t *rec_data, uint8_t rec_len) {
    spi_transaction_t t = {
            .tx_buffer = NULL,
            .length = rec_len * 8,
            .rxlength = rec_len * 8,
            .rx_buffer = rec_data,
    };

    spi_device_acquire_bus(mt6701_dev, portMAX_DELAY);
    ESP_ERROR_CHECK(spi_device_transmit(mt6701_dev, &t));
//    ESP_LOG_BUFFER_HEX(TAG, rec_data, rec_len);

    spi_device_release_bus(mt6701_dev);
}

void drv8311_nsleep_set(uint8_t level) {
    gpio_set_level(DRV8311_NSLEEP_PIN, level);
}

bool IRAM_ATTR
current_oneshot(mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t *edata, void *user_ctx) {
    adc_oneshot_read_isr(adc1_handle, PHASE_A_CH, &adc_raw[0]);
    adc_oneshot_read_isr(adc1_handle, PHASE_B_CH, &adc_raw[1]);
    adc_oneshot_read_isr(adc1_handle, PHASE_C_CH, &adc_raw[2]);

    return true;
}

void foc_delay(uint32_t delay) {
//    vTaskDelay(pdMS_TO_TICKS(delay));
}

void foc_setpwm(float duty_a, float duty_b, float duty_c) {
    drv8311_set_duty(drv8311, duty_a, duty_b, duty_c);
}

void foc_drver_enable(uint8_t en) {
    if (en) {
        drv8311_out_ctrl(drv8311, 1);
    } else {
        drv8311_set_duty(drv8311, 0, 0, 0);
        drv8311_out_ctrl(drv8311, 0);
    }
}

void foc_update_sensors(foc_handle_t handler) {
    float angle = mt6701_get_angle_rad(mt6701);
    int volt_a, volt_b, volt_c;

    handler->sensors.angle_abs = angle;

    adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0], &volt_a);
    adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[1], &volt_b);
    adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[2], &volt_c);

    drv8311_calc_current(drv8311, DRV8311_VREF,
                         volt_a / 1000.f, volt_b / 1000.f, volt_c / 1000.f,
                         &handler->sensors.i_a, &handler->sensors.i_b, &handler->sensors.i_c);
}

bool IRAM_ATTR foc_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    return true;
}

static void sync_pwm_init(void) {
    // generate 20kHZ PWM for DRV8311 PWM SYNC
    mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .period_ticks = (10 * 1000 * 1000) / (10 * 1000),
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &mcpwm_timer));

    mcpwm_operator_config_t operator_config = {
            .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &mcpwm_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_oper, mcpwm_timer));

    mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_oper, &comparator_config, &mcpwm_cmp));

    mcpwm_comparator_set_compare_value(mcpwm_cmp, 500);

    mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = DRV8311_PWM_SYNC_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_oper, &generator_config, &mcpwm_gen));

    mcpwm_generator_set_actions_on_timer_event(mcpwm_gen,
                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                            MCPWM_TIMER_EVENT_EMPTY,
                                                                            MCPWM_GEN_ACTION_HIGH),
                                               MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(mcpwm_gen,
                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_cmp,
                                                                                MCPWM_GEN_ACTION_LOW),
                                                 MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    // sampling current on falling edge (low side FET on)
    mcpwm_comparator_event_callbacks_t cmp_cb = {
            .on_reach = current_oneshot
    };
    mcpwm_comparator_register_event_callbacks(mcpwm_cmp, &cmp_cb, NULL);

    ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));
}

static void adc_init(void) {
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHASE_A_CH, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHASE_B_CH, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHASE_C_CH, &config));

    //-------------ADC1 Calibration Init---------------//
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));
}

static void spi_dev_gpio_init(void) {
    gpio_config_t gpio_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pull_down_en = false,
            .pull_up_en = false,
    };

//    gpio_conf.pin_bit_mask = 1ULL << MT6701_CS_PIN;  // mt6701 cs
//    gpio_config(&gpio_conf);

//    gpio_conf.pin_bit_mask = 1ULL << DRV8311_CS_PIN;  // drv8311 cs
//    gpio_config(&gpio_conf);
//    gpio_set_level(DRV8311_CS_PIN,1);

    gpio_conf.pin_bit_mask = 1ULL << DRV8311_NSLEEP_PIN; // drv8311 nsleep
    gpio_config(&gpio_conf);
    gpio_set_level(DRV8311_NSLEEP_PIN, 0);
}

void spi_dev_init(void) {
    spi_dev_gpio_init();
    adc_init();
    sync_pwm_init();

    spi_bus_config_t buscfg = {
            .sclk_io_num = SPI_CLK_PIN,
            .mosi_io_num = MOSI_PIN,
            .miso_io_num = MISO_PIN,
            .quadhd_io_num = -1,
            .quadwp_io_num = -1,
            .max_transfer_sz = 4,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BUS, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t spi_dev_cfg = {
            .clock_speed_hz = SPI_FREQ,
            .mode = 1,
            .cs_ena_pretrans = 1,
            .spics_io_num = DRV8311_CS_PIN,
            .queue_size = 4,
//            .pre_cb = drv8311_cs_low,
//            .post_cb = drv8311_cs_high,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &spi_dev_cfg, &drv8311_dev));

    spi_dev_cfg.mode = 3;
    spi_dev_cfg.spics_io_num = MT6701_CS_PIN;
    spi_dev_cfg.pre_cb = NULL;
    spi_dev_cfg.post_cb = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &spi_dev_cfg, &mt6701_dev));

    drv8311_cfg_t drv8311_cfg = {
            .pwmcnt_mode = UP_DOWN,
            .sync_mode = SET_PWM_PERIOD,
            .portal = tSPI,
            .csa_gain = CSA_GAIN_2000MV,

//            .pwm_period = 400,
            .use_csa = 1,
            .dev_id = 0x0,
            .parity_check = 0,

            .spi_trans = drv8311_spi_trans,
            .nsleep_set = drv8311_nsleep_set
    };

    drv8311_init(&drv8311, &drv8311_cfg);
    mt6701_init(&mt6701, mt6701_spi_trans);

    // update drv8311 pwm period (used to calculate duty)
    drv8311_update_synced_period(drv8311);
}

void platform_foc_init(void) {
    foc_config_t foc_cfg = {
            .hal = {
                    .update_sensors = foc_update_sensors,
                    .set_pwm = foc_setpwm,
                    .driver_enable = foc_drver_enable,
                    .delay = foc_delay,
            },
            .mode = FOC_MODE_TOR,
            .current_q = &i_q,
            .current_d = &i_d,
            .velocity_loop = &speed,
            .angle_loop = &angle,
            .motor_volt = 5,
            .pole_pairs = 7,
    };

    foc_init(&foc, &foc_cfg);
    foc_angle_auto_zeroing(foc);

//    gptimer_handle_t gptimer = NULL;
//    gptimer_config_t timer_config = {
//            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
//            .direction = GPTIMER_COUNT_UP,
//            .resolution_hz = 1000000, // 1MHz, 1 tick=1us
//    };
//    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
//
//    gptimer_alarm_config_t alarm_config = {
//            .reload_count = 0, // counter will reload with 0 on alarm event
//            .alarm_count = 1000000, // period = 0.001s @resolution 1MHz
//            .flags.auto_reload_on_alarm = true, // enable auto-reload
//    };
//    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
//
//    gptimer_event_callbacks_t cbs = {
//            .on_alarm = foc_timer_cb, // register user callback
//    };
//    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
////    ESP_ERROR_CHECK(gptimer_enable(gptimer));
//    ESP_ERROR_CHECK(gptimer_start(gptimer));

    foc_enable(foc, 1);
}
