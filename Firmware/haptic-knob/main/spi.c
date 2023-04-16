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

#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_log.h"
#include "spi.h"
#include "mt6701_driver.h"
#include "drv8311_driver.h"

#define SPI_BUS SPI3_HOST
#define SPI_FREQ SPI_MASTER_FREQ_8M
#define SPI_CLK_PIN 40U
#define MOSI_PIN 41U
#define MISO_PIN 42U

#define MT6701_CS_PIN 38U
#define DRV8311_CS_PIN 2U
#define DRV8311_NSLEEP_PIN 39U
#define DRV8311_PWM_SYNC_PIN 1U

#define SWAP(x, y) do { (x) ^= (y); (y) ^= (x); (x) ^= (y); } while (0)

#define TAG "spi"

spi_device_handle_t drv8311_dev;
spi_device_handle_t mt6701_dev;

mcpwm_timer_handle_t mcpwm_timer;
mcpwm_oper_handle_t mcpwm_oper;
mcpwm_cmpr_handle_t mcpwm_cmp;
mcpwm_gen_handle_t mcpwm_gen;

drv8311_handle_t drv8311;
mt6701_handle_t mt6701;

//void drv8311_cs_low(spi_transaction_t *trans) {
//    gpio_set_level(DRV8311_CS_PIN, 0);
//}
//
//void drv8311_cs_high(spi_transaction_t *trans) {
//    gpio_set_level(DRV8311_CS_PIN, 1);
//}

void drv8311_spi_trans(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len) {
    spi_device_acquire_bus(drv8311_dev, portMAX_DELAY);
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

    ESP_ERROR_CHECK(spi_device_transmit(drv8311_dev, &t));
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

    spi_device_release_bus(drv8311_dev);
}

void mt6701_spi_trans(uint8_t *rec_data, uint8_t rec_len) {
    spi_device_acquire_bus(mt6701_dev, portMAX_DELAY);

    spi_transaction_t t = {
            .tx_buffer = NULL,
            .length = rec_len * 8,
            .rxlength = rec_len * 8,
            .rx_buffer = rec_data,
    };

    ESP_ERROR_CHECK(spi_device_transmit(mt6701_dev, &t));
//    ESP_LOG_BUFFER_HEX(TAG, rec_data, rec_len);

    spi_device_release_bus(mt6701_dev);
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
    gpio_set_level(DRV8311_NSLEEP_PIN, 1);

    // generate PWM for DRV8311 PWM SYNC
    mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .period_ticks = (10 * 1000 * 1000) / (20 * 1000),
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

    mcpwm_comparator_set_compare_value(mcpwm_cmp, 250);

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

    ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));
}

void spi_dev_init(void) {
    spi_dev_gpio_init();

    spi_bus_config_t buscfg = {
            .sclk_io_num = SPI_CLK_PIN,
            .mosi_io_num = MOSI_PIN,
            .miso_io_num = MISO_PIN,
            .quadhd_io_num = -1,
            .quadwp_io_num = -1,
//            .max_transfer_sz = 4,
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

    drv8311_init(&drv8311, tSPI, 0x0, drv8311_spi_trans);
    mt6701_init(&mt6701, mt6701_spi_trans);
}
