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

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "spi.h"
#include "drv8311_driver.h"

#define SPI_BUS SPI3_HOST
#define SPI_CLK_PIN 40
#define MOSI_PIN 36
#define MISO_PIN 37
#define DRV8311_CS_PIN 39
#define MT6701_CS_PIN 35
#define SPI_FREQ SPI_MASTER_FREQ_10M

#define TAG "spi"

spi_device_handle_t drv8311_dev;
spi_device_handle_t mt6701_dev;

drv8311_handle_t drv8311;

void drv8311_spi_trans(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data,
               uint8_t rec_len) {
//    spi_device_acquire_bus(drv8311_dev,portMAX_DELAY);

    spi_transaction_t t = {
            .tx_buffer = send_data,
            .length = send_len * 8,
            .rx_buffer = rec_data,
            .rxlength = rec_len
    };

    ESP_ERROR_CHECK(spi_device_polling_transmit(drv8311_dev,&t));

//    spi_device_release_bus(drv8311_dev);
}

void spi_dev_init() {
    spi_bus_config_t buscfg = {
            .sclk_io_num = SPI_CLK_PIN,
            .mosi_io_num = MOSI_PIN,
            .miso_io_num = MISO_PIN,
            .quadhd_io_num = -1,
            .quadwp_io_num = -1,
            .max_transfer_sz = 4,
//            .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t spi_dev_cfg = {
            .clock_speed_hz = SPI_MASTER_FREQ_8M,
            .mode = 1,
            .spics_io_num = DRV8311_CS_PIN,
            .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spi_dev_cfg, &drv8311_dev));

    spi_dev_cfg.spics_io_num = MT6701_CS_PIN;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spi_dev_cfg, &mt6701_dev));

    drv8311_init(&drv8311,tSPI,0x00,drv8311_spi_trans);
}
