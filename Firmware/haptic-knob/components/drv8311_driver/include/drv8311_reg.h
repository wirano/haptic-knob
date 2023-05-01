// MIT License
//
// Copyright (c) 2023-2023 wirano
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
// Created by wirano on 23-4-4.
//

#ifndef DRV8311_REG_H
#define DRV8311_REG_H

#include <stdint.h>


typedef enum {
    DRV8311_DEV_STS1_ADDR = 0x00,
    DRV8311_OT_STS_ADDR = 0x04,
    DRV8311_SUP_STS_ADDR = 0x05,
    DRV8311_DRV_STS_ADDR = 0x06,
    DRV8311_SYS_STS_ADDR = 0x07,
    DRV8311_PWM_SYNC_PRD_ADDR = 0x0c,
    DRV8311_FLT_MODE_ADDR = 0x10,
    DRV8311_SYSF_CTRL_ADDR = 0x12,
    DRV8311_DRVF_CTRL_ADDR = 0x13,
    DRV8311_FLT_TCTRL_ADDR = 0x16,
    DRV8311_FLT_CLR_ADDR = 0x17,
    DRV8311_PWMG_PERIOD_ADDR = 0x18,
    DRV8311_PWMG_A_DUTY_ADDR = 0x19,
    DRV8311_PWMG_B_DUTY_ADDR = 0x1a,
    DRV8311_PWMG_C_DUTY_ADDR = 0x1b,
    DRV8311_PWM_STATE_ADDR = 0x1c,
    DRV8311_PWMG_CTRL_ADDR = 0x1d,
    DRV8311_PWM_CTRL1_ADDR = 0x20,
    DRV8311_DRV_CTRL_ADDR = 0x22,
    DRV8311_CSA_CTRL_ADDR = 0x23,
    DRV8311_SYS_CTRL_ADDR = 0x3f
} DRV8311_REG_ADDR_t;

/**
 * @brief Phase Driver Output control
 */
typedef enum {
    PHASE_OFF = 0x0,
    PHASE_SET_LOW = 0x01,
    PHASE_SET_HIGH = 0x02,
    PHASE_LOW_PWM = 0x5,
    PHASE_HIGH_PWM = 0x6,
    PHASE_CMP_PWM = 0x7
} DRV8311_PHASE_MODE_t;

/**
 * @brief PWM GEN counter mode
 */
typedef enum {
    UP_DOWN = 0x0,
    UP = 0x1,
    DOWN = 0x2,
    NO_ACTION = 0x3
} DRV8311_PWMCNTR_MODE_t;

/**
 * @brief Oscillator synchronization and PWM_SYNC control
 */
typedef enum {
    SYNC_DISABLE = 0x0,
    USE_PWM_SYNC_PERIOD = 0x01,
    SET_PWM_PERIOD = 0x02,
    OSC_SYNC = 0x05,
    OSC_SYNC_PWM_PERIOD = 0x06,
    USE_SPI_CLK = 0x7
} DRV8311_PWM_OSC_SYNC_t;

/**
 * @brief SPI Clock Frequency for synchronizing the Oscillator
 */
typedef enum {
    SPI_FREQ_1M = 0x0,
    SPI_FREQ_1_25M = 0x1,
    SPI_FREQ_2M = 0x2,
    SPI_FREQ_2_5M = 0x3,
    SPI_FREQ_4M = 0x4,
    SPI_FREQ_5M = 0x5,
    SPI_FREQ_8M = 0x6,
    SPI_FREQ_10M = 0x7,
} DRV8311_SPICLK_FREQ_SYNC_t;

/**
 * @brief Number of SPI Clock Cycle require for synchronizing the Oscillator
 */
typedef enum {
    CLOCKS_512 = 0x0,
    CLOCKS_256 = 0x1,
    CLOCKS_128 = 0x2,
    CLOCKS_64 = 0x3
} DRV8311_SPISYNC_ACRCY_t;

/**
 * @brief Current Sense Amplifier Gain setting
 */
typedef enum {
    CSA_GAIN_250MV = 0x0,
    CSA_GAIN_500MV = 0x1,
    CSA_GAIN_1000MV = 0x2,
    CSA_GAIN_2000MV = 0x3
} DRV8311_CSA_GAIN_t;

/**
 * @brief Device Status 1 Register
 */
typedef struct {
    uint16_t fault: 1;
    uint16_t ot: 1;
    uint16_t uvp: 1;
    uint16_t reserved_1: 2;
    uint16_t ocp: 1;
    uint16_t spi_flt: 1;
    uint16_t reset: 1;
    uint16_t opt_flt: 1;
    uint16_t reserved_0: 6;
    uint16_t parity_bit: 1;
} drv8311_dev_sts1_t;

/**
 * @brief Over Temperature Status Register
 */
typedef struct {
    uint16_t otsd: 1;
    uint16_t otw: 1;
    uint16_t ots_avdd: 1;
    uint16_t reserved_0: 12;
    uint16_t parity_bit: 1;
} drv8311_ot_sts_t;

/**
 * @brief Supply Status Register
 */
typedef struct {
    uint16_t vinavdd_uv: 1;
    uint16_t reserved_2: 1;
    uint16_t avdd_uv: 1;
    uint16_t reserved_1: 1;
    uint16_t cp_uv: 1;
    uint16_t csaref_uv: 1;
    uint16_t reserved_0: 9;
    uint16_t parity_bit: 1;
} drv8311_sup_sts_t;

/**
 * @brief Over Temperature Status Register
 */
typedef struct {
    uint16_t ocpa_ls: 1;
    uint16_t ocpb_ls: 1;
    uint16_t ocpc_ls: 1;
    uint16_t reserved_1: 1;
    uint16_t ocpa_hs: 1;
    uint16_t ocpb_hs: 1;
    uint16_t ocpc_hs: 1;
    uint16_t reserved_0: 8;
    uint16_t parity_bit: 1;
} drv8311_drv_sts_t;

/**
 * @brief System Status Register
 */
typedef struct {
    uint16_t frm_err: 1;
    uint16_t bus_cnt: 1;
    uint16_t spi_parity: 1;
    uint16_t resvered_1: 1;
    uint16_t otpld_err: 1;
    uint16_t reserved_0: 10;
    uint16_t parity_bit: 1;
} drv8311_sys_sts_t;

/**
 * @brief PWM Sync Period Register
 */
typedef struct {
    uint16_t pwm_sync_prd: 12;
    uint16_t reserved_0: 3;
    uint16_t parity_bit: 1;
} drv8311_pwm_sync_prd_t;


/**
 * @brief Fault Mode Register
 */
typedef struct {
    uint16_t otsd_mode: 2;
    uint16_t uvp_mode: 2;
    uint16_t ocp_mode: 3;
    uint16_t spiflt_mode: 1;
    uint16_t otpflt_mode: 1;
    uint16_t reserved_0: 6;
    uint16_t parity_bit: 1;
} drv8311_flt_mode_t;

/**
 * @brief System Fault Control Register
 */
typedef struct {
    uint16_t resverd_1: 5;
    uint16_t csarefuv_en: 1;
    uint16_t reserved_1: 3;
    uint16_t otw_en: 1;
    uint16_t otavdd_en: 1;
    uint16_t reserved_0: 4;
    uint16_t parity_bit: 1;
} drv8311_sysf_ctrl_t;

/**
 * @brief Driver Fault Control Register
 */
typedef struct {
    uint16_t ocp_lvl: 1;
    uint16_t reserved_1: 1;
    uint16_t ocp_tblank: 2;
    uint16_t ocp_deg: 2;
    uint16_t reserved_0: 9;
    uint16_t parity_bit: 1;
} drv8311_drvf_ctrl_t;

/**
 * @brief Fault Timing Control Register
 */
typedef struct {
    uint16_t fast_tretry: 2;
    uint16_t slow_tretry: 2;
    uint16_t reserved_0: 11;
    uint16_t parity_bit: 1;
} drv8311_flt_tctrl_t;

/**
 * @brief Fault Clear Register
 */
typedef struct {
    uint16_t flt_clr: 1;
    uint16_t reserved_0: 14;
    uint16_t parity_bit: 1;
} drv8311_flt_ctrl_t;

/**
 * @brief PWM_GEN Period Register
 */
typedef struct {
    uint16_t pwm_prd_out: 12;
    uint16_t reserved_0: 3;
    uint16_t parity_bit: 1;
} drv8311_pwmg_period_t;

/**
 * @brief PWM_GEN x Duty Registe
 * x can be A, B,C
 */
typedef struct {
    uint16_t pwm_duty_outx: 12;
    uint16_t reserved_0: 3;
    uint16_t parity_bit: 1;
} drv8311_pwmg_x_duty_t;

/**
 * @brief PWM State Register
 */
typedef struct {
    uint16_t pwma_state: 3;
    uint16_t reserved_2: 1;
    uint16_t pwmb_state: 3;
    uint16_t reserved_1: 1;
    uint16_t pwmc_state: 3;
    uint16_t reserved_0: 4;
    uint16_t parity_bit: 1;
} drv8311_pwm_state_t;

/**
 * @brief PWM_GEN Control Register
 */
typedef struct {
    uint16_t spisync_acrcy: 2;
    uint16_t spiclk_freq_sync: 3;
    uint16_t pwm_osc_sync: 3;
    uint16_t pwmcntr_mode: 2;
    uint16_t pwm_en: 1;
    uint16_t reserved_0: 4;
    uint16_t parity_bit: 1;
} drv8311_pwmg_ctrl_t;

/**
 * @brief PWM Control Register 1
 */
typedef struct {
    uint16_t pwm_mode: 2;
    uint16_t ssc_dis: 1;
    uint16_t reserved_0: 12;
    uint16_t parity_bit: 1;
} drv8311_pwm_ctrl1_t;

/**
 * @brief Predriver control Register
 */
typedef struct {
    uint16_t slew_rate: 2;
    uint16_t reserved_1: 2;
    uint16_t tdead_ctrl: 3;
    uint16_t dlycmp_en: 1;
    uint16_t reserved_0: 7;
    uint16_t parity_bit: 1;
} drv8311_drv_ctrl_t;

/**
 * @brief CSA Control Register
 */
typedef struct {
    uint16_t csa_gain: 2;
    uint16_t reserved_1: 1;
    uint16_t csa_en: 1;
    uint16_t reserved_0: 11;
    uint16_t parity_bit: 1;
} drv8311_csa_ctrl_t;

/**
 * @brief System Control Registe
 */
typedef struct {
    uint16_t reserved_1: 6;
    uint16_t spi_pen: 1;
    uint16_t reg_lock: 1;
    uint16_t reserved_0: 4;
    uint16_t write_key: 3;
    uint16_t parity_bit: 1;
} drv8311_sys_ctrl_t;

typedef union {
    drv8311_dev_sts1_t dev_sts1;
    drv8311_ot_sts_t ot_sts;
    drv8311_sup_sts_t sup_sts;
    drv8311_drv_sts_t drv_sts;
    drv8311_sys_sts_t sys_sts;
    drv8311_pwm_sync_prd_t pwm_sync_prd;
    drv8311_flt_mode_t flt_mode;
    drv8311_sysf_ctrl_t sysf_ctrl;
    drv8311_drvf_ctrl_t drvf_ctrl;
    drv8311_flt_tctrl_t flt_tctrl;
    drv8311_flt_ctrl_t flt_ctrl;
    drv8311_pwmg_period_t pwmg_period;
    drv8311_pwmg_x_duty_t pwmg_x_duty;
    drv8311_pwm_state_t pwm_state;
    drv8311_pwmg_ctrl_t pwmg_ctrl;
    drv8311_pwm_ctrl1_t pwm_ctrl1;
    drv8311_drv_ctrl_t drv_ctrl;
    drv8311_csa_ctrl_t csa_ctrl;
    drv8311_sys_ctrl_t sys_ctrl;
    uint16_t half_word;
    uint8_t bytes[2];
} drv8311_reg_t;

#endif //DRV8311_REG_H
