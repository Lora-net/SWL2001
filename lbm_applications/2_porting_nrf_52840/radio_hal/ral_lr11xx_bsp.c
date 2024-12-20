/*!
 * \file      ral_lr11xx_bsp.c
 *
 * \brief     Implements the BSP (BoardSpecificPackage) HAL functions for LR11XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "ral_lr11xx_bsp.h"
#include "lr11xx_pa_pwr_cfg.h"
#include "smtc_hal_mcu.h"
#include "radio_utilities.h"
#include "smtc_modem_hal.h"
#include "lr11xx_radio.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// TODO: check values
#define LR11XX_GFSK_RX_CONSUMPTION_DCDC 5400
#define LR11XX_GFSK_RX_BOOSTED_CONSUMPTION_DCDC 7500

#define LR11XX_GFSK_RX_CONSUMPTION_LDO 5400
#define LR11XX_GFSK_RX_BOOSTED_CONSUMPTION_LDO 7500

#define LR11XX_LORA_RX_CONSUMPTION_DCDC 5700
#define LR11XX_LORA_RX_BOOSTED_CONSUMPTION_DCDC 7800

#define LR11XX_LORA_RX_CONSUMPTION_LDO 5700
#define LR11XX_LORA_RX_BOOSTED_CONSUMPTION_LDO 7800

#define LR11XX_LP_MIN_OUTPUT_POWER -17
#define LR11XX_LP_MAX_OUTPUT_POWER 15

#define LR11XX_HP_MIN_OUTPUT_POWER -9
#define LR11XX_HP_MAX_OUTPUT_POWER 22

#define LR11XX_HF_MIN_OUTPUT_POWER -18
#define LR11XX_HF_MAX_OUTPUT_POWER 13

#define LR11XX_LP_CONVERT_TABLE_INDEX_OFFSET 17
#define LR11XX_HP_CONVERT_TABLE_INDEX_OFFSET 9
#define LR11XX_HF_CONVERT_TABLE_INDEX_OFFSET 18

#define LR11XX_PWR_VREG_VBAT_SWITCH 8

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum lr11xx_pa_type_s
{
    LR11XX_WITH_LF_LP_PA,
    LR11XX_WITH_LF_HP_PA,
    LR11XX_WITH_LF_LP_HP_PA,
    LR11XX_WITH_HF_PA,
} lr11xx_pa_type_t;

typedef struct lr11xx_pa_pwr_cfg_s
{
    int8_t  power;
    uint8_t pa_duty_cycle;
    uint8_t pa_hp_sel;
} lr11xx_pa_pwr_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const lr11xx_pa_pwr_cfg_t pa_lp_cfg_table[LR11XX_LP_MAX_OUTPUT_POWER - LR11XX_LP_MIN_OUTPUT_POWER + 1] = LR11XX_PA_LP_LF_CFG_TABLE;
const lr11xx_pa_pwr_cfg_t pa_hp_cfg_table[LR11XX_HP_MAX_OUTPUT_POWER - LR11XX_HP_MIN_OUTPUT_POWER + 1] = LR11XX_PA_HP_LF_CFG_TABLE;

const lr11xx_pa_pwr_cfg_t pa_hf_cfg_table[LR11XX_HF_MAX_OUTPUT_POWER - LR11XX_HF_MIN_OUTPUT_POWER + 1] = LR11XX_PA_HF_CFG_TABLE;

static const uint32_t ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_dcdc_lp_vreg[] = {
    10820,  // -17 dBm
    10980,  // -16 dBm
    11060,  // -15 dBm
    11160,  // -14 dBm
    11300,  // -13 dBm
    11430,  // -12 dBm
    11550,  // -11 dBm
    11680,  // -10 dBm
    11930,  //  -9 dBm
    12170,  //  -8 dBm
    12420,  //  -7 dBm
    12650,  //  -6 dBm
    12900,  //  -5 dBm
    13280,  //  -4 dBm
    13600,  //  -3 dBm
    14120,  //  -2 dBm
    14600,  //  -1 dBm
    15090,  //   0 dBm
    15780,  //   1 dBm
    16490,  //   2 dBm
    17250,  //   3 dBm
    17850,  //   4 dBm
    18720,  //   5 dBm
    19640,  //   6 dBm
    20560,  //   7 dBm
    21400,  //   8 dBm
    22620,  //   9 dBm
    23720,  //  10 dBm
    25050,  //  11 dBm
    26350,  //  12 dBm
    27870,  //  13 dBm
    28590,  //  14 dBm
    37820,  //  15 dBm
};

static const uint32_t ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_ldo_lp_vreg[] = {
    14950,  // -17 dBm
    15280,  // -16 dBm
    15530,  // -15 dBm
    15770,  // -14 dBm
    16020,  // -13 dBm
    16290,  // -12 dBm
    16550,  // -11 dBm
    16760,  // -10 dBm
    17280,  //  -9 dBm
    17770,  //  -8 dBm
    18250,  //  -7 dBm
    18750,  //  -6 dBm
    19250,  //  -5 dBm
    19960,  //  -4 dBm
    20710,  //  -3 dBm
    21620,  //  -2 dBm
    22570,  //  -1 dBm
    23570,  //   0 dBm
    24990,  //   1 dBm
    26320,  //   2 dBm
    27830,  //   3 dBm
    29070,  //   4 dBm
    30660,  //   5 dBm
    32490,  //   6 dBm
    34220,  //   7 dBm
    35820,  //   8 dBm
    38180,  //   9 dBm
    40220,  //  10 dBm
    42800,  //  11 dBm
    45030,  //  12 dBm
    47900,  //  13 dBm
    51220,  //  14 dBm
    66060,  //  15 dBm
};

static const uint32_t ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_dcdc_hp_vbat[] = {
    27750,   //  -9 dBm
    29100,   //  -8 dBm
    30320,   //  -7 dBm
    31650,   //  -6 dBm
    34250,   //  -5 dBm
    35550,   //  -4 dBm
    36770,   //  -3 dBm
    39250,   //  -2 dBm
    41480,   //  -1 dBm
    43820,   //   0 dBm
    46000,   //   1 dBm
    49020,   //   2 dBm
    50900,   //   3 dBm
    54200,   //   4 dBm
    56330,   //   5 dBm
    59050,   //   6 dBm
    62210,   //   7 dBm
    65270,   //   8 dBm
    68600,   //   9 dBm
    71920,   //  10 dBm
    75500,   //  11 dBm
    79500,   //  12 dBm
    84130,   //  13 dBm
    88470,   //  14 dBm
    92200,   //  15 dBm
    94340,   //  16 dBm
    96360,   //  17 dBm
    98970,   //  18 dBm
    102220,  //  19 dBm
    106250,  //  20 dBm
    111300,  //  21 dBm
    113040,  //  22 dBm
};

static const uint32_t ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_ldo_hp_vbat[] = {
    31310,   //  -9 dBm
    32700,   //  -8 dBm
    33970,   //  -7 dBm
    35270,   //  -6 dBm
    37900,   //  -5 dBm
    39140,   //  -4 dBm
    40380,   //  -3 dBm
    42860,   //  -2 dBm
    45150,   //  -1 dBm
    47400,   //   0 dBm
    49600,   //   1 dBm
    52600,   //   2 dBm
    54460,   //   3 dBm
    57690,   //   4 dBm
    59840,   //   5 dBm
    62550,   //   6 dBm
    65750,   //   7 dBm
    68520,   //   8 dBm
    72130,   //   9 dBm
    75230,   //  10 dBm
    78600,   //  11 dBm
    82770,   //  12 dBm
    87450,   //  13 dBm
    91700,   //  14 dBm
    95330,   //  15 dBm
    97520,   //  16 dBm
    99520,   //  17 dBm
    102080,  //  18 dBm
    105140,  //  19 dBm
    109300,  //  20 dBm
    114460,  //  21 dBm
    116530,  //  22 dBm
};

static const uint32_t ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_dcdc_hf_vreg[] = {
    11800,  // -18 dBm
    11800,  // -17 dBm
    11800,  // -16 dBm
    11900,  // -15 dBm
    12020,  // -14 dBm
    12120,  // -13 dBm
    12230,  // -12 dBm
    12390,  // -11 dBm
    12540,  // -10 dBm
    12740,  //  -9 dBm
    12960,  //  -8 dBm
    13150,  //  -7 dBm
    13460,  //  -6 dBm
    13770,  //  -5 dBm
    14070,  //  -4 dBm
    14460,  //  -3 dBm
    15030,  //  -2 dBm
    15440,  //  -1 dBm
    16030,  //   0 dBm
    16980,  //   1 dBm
    17590,  //   2 dBm
    18270,  //   3 dBm
    19060,  //   4 dBm
    19900,  //   5 dBm
    20740,  //   6 dBm
    21610,  //   7 dBm
    22400,  //   8 dBm
    23370,  //   9 dBm
    24860,  //  10 dBm
    26410,  //  11 dBm
    26430,  //  12 dBm
    27890,  //  13 dBm
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get the tx output param configuration given the type of power amplifier and expected output power
 *
 * @param [in] pa_type Power Amplifier type
 * @param [in] expected_output_pwr_in_dbm TX output power in dBm
 * @param [out] output_params The tx config output params
 */
void lr11xx_get_tx_cfg( lr11xx_pa_type_t pa_type, int8_t expected_output_pwr_in_dbm,
                        ral_lr11xx_bsp_tx_cfg_output_params_t* output_params );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void ral_lr11xx_bsp_get_rf_switch_cfg( const void* context, lr11xx_system_rfswitch_cfg_t* rf_switch_cfg )
{
    rf_switch_cfg->enable =
        LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH | LR11XX_SYSTEM_RFSW3_HIGH;
    rf_switch_cfg->standby = 0;
    rf_switch_cfg->rx      = LR11XX_SYSTEM_RFSW0_HIGH;
    rf_switch_cfg->tx      = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH;
    rf_switch_cfg->tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH;
    rf_switch_cfg->tx_hf   = 0;
    rf_switch_cfg->gnss    = LR11XX_SYSTEM_RFSW2_HIGH;
    rf_switch_cfg->wifi    = LR11XX_SYSTEM_RFSW3_HIGH;
}

void ral_lr11xx_bsp_get_tx_cfg( const void* context, const ral_lr11xx_bsp_tx_cfg_input_params_t* input_params,
                                ral_lr11xx_bsp_tx_cfg_output_params_t* output_params )
{
    // get board tx power offset
    int8_t board_tx_pwr_offset_db = radio_utilities_get_tx_power_offset( );

    int16_t power = input_params->system_output_pwr_in_dbm + board_tx_pwr_offset_db;

    lr11xx_pa_type_t pa_type;

    // check frequency band first to choose LF of HF PA
    if( input_params->freq_in_hz >= 2400000000 )
    {
        pa_type = LR11XX_WITH_HF_PA;
    }
    else
    {
        // Modem is acting in subgig band: use LP/HP PA (both LP and HP are connected on lr11xx evk board)
        pa_type = LR11XX_WITH_LF_LP_HP_PA;
    }

    // call the configuration function
    lr11xx_get_tx_cfg( pa_type, power, output_params );
}

void ral_lr11xx_bsp_get_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
    *reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;
}

void ral_lr11xx_bsp_get_xosc_cfg( const void* context, ral_xosc_cfg_t* xosc_cfg,
                                  lr11xx_system_tcxo_supply_voltage_t* supply_voltage, uint32_t* startup_time_in_tick )
{
#if !defined( LR1121 )
    // Get startup value defined in modem_hal to avoid mis-alignment
    uint32_t startup_time_ms = smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    *xosc_cfg                = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
    *supply_voltage          = LR11XX_SYSTEM_TCXO_CTRL_1_8V;
    // tick is 30.52µs
    *startup_time_in_tick = lr11xx_radio_convert_time_in_ms_to_rtc_step( startup_time_ms );
#endif  // !defined( LR1121 )
}

void ral_lr11xx_bsp_get_crc_state( const void* context, bool* crc_is_activated )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    SMTC_HAL_TRACE_INFO( "LR11XX CRC over spi is activated\n" );
    *crc_is_activated = true;
#else
    *crc_is_activated = false;
#endif
}

void ral_lr11xx_bsp_get_rssi_calibration_table( const void* context, const uint32_t freq_in_hz,
                                                lr11xx_radio_rssi_calibration_table_t* rssi_calibration_table )
{
    if( freq_in_hz <= 600000000 )
    {
        rssi_calibration_table->gain_offset      = 0;
        rssi_calibration_table->gain_tune.g4     = 12;
        rssi_calibration_table->gain_tune.g5     = 12;
        rssi_calibration_table->gain_tune.g6     = 14;
        rssi_calibration_table->gain_tune.g7     = 0;
        rssi_calibration_table->gain_tune.g8     = 1;
        rssi_calibration_table->gain_tune.g9     = 3;
        rssi_calibration_table->gain_tune.g10    = 4;
        rssi_calibration_table->gain_tune.g11    = 4;
        rssi_calibration_table->gain_tune.g12    = 3;
        rssi_calibration_table->gain_tune.g13    = 6;
        rssi_calibration_table->gain_tune.g13hp1 = 6;
        rssi_calibration_table->gain_tune.g13hp2 = 6;
        rssi_calibration_table->gain_tune.g13hp3 = 6;
        rssi_calibration_table->gain_tune.g13hp4 = 6;
        rssi_calibration_table->gain_tune.g13hp5 = 6;
        rssi_calibration_table->gain_tune.g13hp6 = 6;
        rssi_calibration_table->gain_tune.g13hp7 = 6;
    }
    else if( ( 600000000 <= freq_in_hz ) && ( freq_in_hz <= 2000000000 ) )
    {
        rssi_calibration_table->gain_offset      = 0;
        rssi_calibration_table->gain_tune.g4     = 2;
        rssi_calibration_table->gain_tune.g5     = 2;
        rssi_calibration_table->gain_tune.g6     = 2;
        rssi_calibration_table->gain_tune.g7     = 3;
        rssi_calibration_table->gain_tune.g8     = 3;
        rssi_calibration_table->gain_tune.g9     = 4;
        rssi_calibration_table->gain_tune.g10    = 5;
        rssi_calibration_table->gain_tune.g11    = 4;
        rssi_calibration_table->gain_tune.g12    = 4;
        rssi_calibration_table->gain_tune.g13    = 6;
        rssi_calibration_table->gain_tune.g13hp1 = 5;
        rssi_calibration_table->gain_tune.g13hp2 = 5;
        rssi_calibration_table->gain_tune.g13hp3 = 6;
        rssi_calibration_table->gain_tune.g13hp4 = 6;
        rssi_calibration_table->gain_tune.g13hp5 = 6;
        rssi_calibration_table->gain_tune.g13hp6 = 7;
        rssi_calibration_table->gain_tune.g13hp7 = 6;
    }
    else  // freq_in_hz > 2000000000
    {
        rssi_calibration_table->gain_offset      = 2030;
        rssi_calibration_table->gain_tune.g4     = 6;
        rssi_calibration_table->gain_tune.g5     = 7;
        rssi_calibration_table->gain_tune.g6     = 6;
        rssi_calibration_table->gain_tune.g7     = 4;
        rssi_calibration_table->gain_tune.g8     = 3;
        rssi_calibration_table->gain_tune.g9     = 4;
        rssi_calibration_table->gain_tune.g10    = 14;
        rssi_calibration_table->gain_tune.g11    = 12;
        rssi_calibration_table->gain_tune.g12    = 14;
        rssi_calibration_table->gain_tune.g13    = 12;
        rssi_calibration_table->gain_tune.g13hp1 = 12;
        rssi_calibration_table->gain_tune.g13hp2 = 12;
        rssi_calibration_table->gain_tune.g13hp3 = 12;
        rssi_calibration_table->gain_tune.g13hp4 = 8;
        rssi_calibration_table->gain_tune.g13hp5 = 8;
        rssi_calibration_table->gain_tune.g13hp6 = 9;
        rssi_calibration_table->gain_tune.g13hp7 = 9;
    }
}

void ral_lr11xx_bsp_get_lora_cad_det_peak( const void *context, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                           ral_lora_cad_symbs_t nb_symbol, uint8_t* in_out_cad_det_peak )
{
    // Function used to fine tune the cad detection peak, update if needed
}

void ral_lr11xx_bsp_get_rx_boost_cfg( const void* context, bool* rx_boost_is_activated )
{
    *rx_boost_is_activated = false;
}

void ral_lr11xx_bsp_get_lfclk_cfg_in_sleep( const void* context, bool* lfclk_is_running )
{
#if defined( ADD_APP_GEOLOCATION )
    *lfclk_is_running = true;
#else
    *lfclk_is_running = false;
#endif
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr11xx_get_tx_cfg( lr11xx_pa_type_t pa_type, int8_t expected_output_pwr_in_dbm,
                        ral_lr11xx_bsp_tx_cfg_output_params_t* output_params )
{
    int8_t power = expected_output_pwr_in_dbm;

    // Ramp time is the same for any config
    output_params->pa_ramp_time = LR11XX_RADIO_RAMP_48_US;

    switch( pa_type )
    {
    case LR11XX_WITH_LF_LP_PA:
    {
        // Check power boundaries for LP LF PA: The output power must be in range [ -17 , +15 ] dBm
        if( power < LR11XX_LP_MIN_OUTPUT_POWER )
        {
            power = LR11XX_LP_MIN_OUTPUT_POWER;
        }
        else if( power > LR11XX_LP_MAX_OUTPUT_POWER )
        {
            power = LR11XX_LP_MAX_OUTPUT_POWER;
        }
        output_params->pa_cfg.pa_sel                     = LR11XX_RADIO_PA_SEL_LP;
        output_params->pa_cfg.pa_reg_supply              = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        output_params->pa_cfg.pa_duty_cycle              = pa_lp_cfg_table[power - LR11XX_LP_MIN_OUTPUT_POWER].pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = pa_lp_cfg_table[power - LR11XX_LP_MIN_OUTPUT_POWER].pa_hp_sel;
        output_params->chip_output_pwr_in_dbm_configured = pa_lp_cfg_table[power - LR11XX_LP_MIN_OUTPUT_POWER].power;
        output_params->chip_output_pwr_in_dbm_expected   = power;
        break;
    }
    case LR11XX_WITH_LF_HP_PA:
    {
        // Check power boundaries for HP LF PA: The output power must be in range [ -9 , +22 ] dBm
        if( power < LR11XX_HP_MIN_OUTPUT_POWER )
        {
            power = LR11XX_HP_MIN_OUTPUT_POWER;
        }
        else if( power > LR11XX_HP_MAX_OUTPUT_POWER )
        {
            power = LR11XX_HP_MAX_OUTPUT_POWER;
        }
        output_params->pa_cfg.pa_sel                   = LR11XX_RADIO_PA_SEL_HP;
        output_params->chip_output_pwr_in_dbm_expected = power;

        if( power <= LR11XX_PWR_VREG_VBAT_SWITCH )
        {
            // For powers below 8dBm use regulated supply for HP PA for a better efficiency.
            output_params->pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        }
        else
        {
            output_params->pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT;
        }

        output_params->pa_cfg.pa_duty_cycle              = pa_hp_cfg_table[power - LR11XX_HP_MIN_OUTPUT_POWER].pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = pa_hp_cfg_table[power - LR11XX_HP_MIN_OUTPUT_POWER].pa_hp_sel;
        output_params->chip_output_pwr_in_dbm_configured = pa_hp_cfg_table[power - LR11XX_HP_MIN_OUTPUT_POWER].power;
        break;
    }
    case LR11XX_WITH_LF_LP_HP_PA:
    {
        // Check power boundaries for LP/HP LF PA: The output power must be in range [ -17 , +22 ] dBm
        if( power < LR11XX_LP_MIN_OUTPUT_POWER )
        {
            power = LR11XX_LP_MIN_OUTPUT_POWER;
        }
        else if( power > LR11XX_HP_MAX_OUTPUT_POWER )
        {
            power = LR11XX_HP_MAX_OUTPUT_POWER;
        }
        output_params->chip_output_pwr_in_dbm_expected = power;

        if( power <= LR11XX_LP_MAX_OUTPUT_POWER )
        {
            output_params->pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_LP;
            output_params->pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
            output_params->pa_cfg.pa_duty_cycle = pa_lp_cfg_table[power - LR11XX_LP_MIN_OUTPUT_POWER].pa_duty_cycle;
            output_params->pa_cfg.pa_hp_sel     = pa_lp_cfg_table[power - LR11XX_LP_MIN_OUTPUT_POWER].pa_hp_sel;
            output_params->chip_output_pwr_in_dbm_configured = pa_lp_cfg_table[power - LR11XX_LP_MIN_OUTPUT_POWER].power;
        }
        else
        {
            output_params->pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_HP;
            output_params->pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT;
            output_params->pa_cfg.pa_duty_cycle = pa_hp_cfg_table[power - LR11XX_HP_MIN_OUTPUT_POWER].pa_duty_cycle;
            output_params->pa_cfg.pa_hp_sel     = pa_hp_cfg_table[power - LR11XX_HP_MIN_OUTPUT_POWER].pa_hp_sel;
            output_params->chip_output_pwr_in_dbm_configured = pa_hp_cfg_table[power - LR11XX_HP_MIN_OUTPUT_POWER].power;
        }
        break;
    }
    case LR11XX_WITH_HF_PA:
    {
        // Check power boundaries for HF PA: The output power must be in range [ -18 , +13 ] dBm
        if( power < LR11XX_HF_MIN_OUTPUT_POWER )
        {
            power = LR11XX_HF_MIN_OUTPUT_POWER;
        }
        else if( power > LR11XX_HF_MAX_OUTPUT_POWER )
        {
            power = LR11XX_HF_MAX_OUTPUT_POWER;
        }
        output_params->pa_cfg.pa_sel                     = LR11XX_RADIO_PA_SEL_HF;
        output_params->pa_cfg.pa_reg_supply              = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        output_params->pa_cfg.pa_duty_cycle              = pa_hf_cfg_table[power - LR11XX_HF_MIN_OUTPUT_POWER].pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = pa_hf_cfg_table[power - LR11XX_HF_MIN_OUTPUT_POWER].pa_hp_sel;
        output_params->chip_output_pwr_in_dbm_configured = pa_hf_cfg_table[power - LR11XX_HF_MIN_OUTPUT_POWER].power;
        output_params->chip_output_pwr_in_dbm_expected   = power;
        break;
    }
    }
}

ral_status_t ral_lr11xx_bsp_get_instantaneous_tx_power_consumption( const void *context,
                                                                    const ral_lr11xx_bsp_tx_cfg_output_params_t* tx_cfg,
                                                                    lr11xx_system_reg_mode_t radio_reg_mode,
                                                                    uint32_t*                pwr_consumption_in_ua )
{
    if( tx_cfg->pa_cfg.pa_sel == LR11XX_RADIO_PA_SEL_LP )
    {
        if( tx_cfg->pa_cfg.pa_reg_supply == LR11XX_RADIO_PA_REG_SUPPLY_VREG )
        {
            uint8_t index = 0;

            if( tx_cfg->chip_output_pwr_in_dbm_expected > LR11XX_LP_MAX_OUTPUT_POWER )
            {
                index = LR11XX_LP_MAX_OUTPUT_POWER + LR11XX_LP_CONVERT_TABLE_INDEX_OFFSET;
            }
            else if( tx_cfg->chip_output_pwr_in_dbm_expected < LR11XX_LP_MIN_OUTPUT_POWER )
            {
                index = LR11XX_LP_MIN_OUTPUT_POWER + LR11XX_LP_CONVERT_TABLE_INDEX_OFFSET;
            }
            else
            {
                index = tx_cfg->chip_output_pwr_in_dbm_expected + LR11XX_LP_CONVERT_TABLE_INDEX_OFFSET;
            }

            if( radio_reg_mode == LR11XX_SYSTEM_REG_MODE_DCDC )
            {
                *pwr_consumption_in_ua = ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_dcdc_lp_vreg[index];
            }
            else
            {
                *pwr_consumption_in_ua = ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_ldo_lp_vreg[index];
            }
        }
        else
        {
            return RAL_STATUS_UNSUPPORTED_FEATURE;
        }
    }
    else if( tx_cfg->pa_cfg.pa_sel == LR11XX_RADIO_PA_SEL_HP )
    {
        if( tx_cfg->pa_cfg.pa_reg_supply == LR11XX_RADIO_PA_REG_SUPPLY_VBAT )
        {
            uint8_t index = 0;

            if( tx_cfg->chip_output_pwr_in_dbm_expected > LR11XX_HP_MAX_OUTPUT_POWER )
            {
                index = LR11XX_HP_MAX_OUTPUT_POWER + LR11XX_HP_CONVERT_TABLE_INDEX_OFFSET;
            }
            else if( tx_cfg->chip_output_pwr_in_dbm_expected < LR11XX_HP_MIN_OUTPUT_POWER )
            {
                index = LR11XX_HP_MIN_OUTPUT_POWER + LR11XX_HP_CONVERT_TABLE_INDEX_OFFSET;
            }
            else
            {
                index = tx_cfg->chip_output_pwr_in_dbm_expected + LR11XX_HP_CONVERT_TABLE_INDEX_OFFSET;
            }

            if( radio_reg_mode == LR11XX_SYSTEM_REG_MODE_DCDC )
            {
                *pwr_consumption_in_ua = ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_dcdc_hp_vbat[index];
            }
            else
            {
                *pwr_consumption_in_ua = ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_ldo_hp_vbat[index];
            }
        }
        else
        {
            return RAL_STATUS_UNSUPPORTED_FEATURE;
        }
    }
    else if( tx_cfg->pa_cfg.pa_sel == LR11XX_RADIO_PA_SEL_HF )
    {
        if( tx_cfg->pa_cfg.pa_reg_supply == LR11XX_RADIO_PA_REG_SUPPLY_VREG )
        {
            uint8_t index = 0;

            if( tx_cfg->chip_output_pwr_in_dbm_expected > LR11XX_HF_MAX_OUTPUT_POWER )
            {
                index = LR11XX_HF_MAX_OUTPUT_POWER + LR11XX_HF_CONVERT_TABLE_INDEX_OFFSET;
            }
            else if( tx_cfg->chip_output_pwr_in_dbm_expected < LR11XX_HF_MIN_OUTPUT_POWER )
            {
                index = LR11XX_HF_MIN_OUTPUT_POWER + LR11XX_HF_CONVERT_TABLE_INDEX_OFFSET;
            }
            else
            {
                index = tx_cfg->chip_output_pwr_in_dbm_expected + LR11XX_HF_CONVERT_TABLE_INDEX_OFFSET;
            }

            if( radio_reg_mode == LR11XX_SYSTEM_REG_MODE_DCDC )
            {
                *pwr_consumption_in_ua = ral_lr11xx_convert_tx_dbm_to_ua_reg_mode_dcdc_hf_vreg[index];
            }
            else
            {
                return RAL_STATUS_UNSUPPORTED_FEATURE;
            }
        }
        else
        {
            return RAL_STATUS_UNSUPPORTED_FEATURE;
        }
    }
    else
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_bsp_get_instantaneous_gfsk_rx_power_consumption( const void *context,
                                                                         lr11xx_system_reg_mode_t radio_reg_mode,
                                                                         bool                     rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua )
{
    if( radio_reg_mode == LR11XX_SYSTEM_REG_MODE_DCDC )
    {
        *pwr_consumption_in_ua =
            ( rx_boosted ) ? LR11XX_GFSK_RX_BOOSTED_CONSUMPTION_DCDC : LR11XX_GFSK_RX_CONSUMPTION_DCDC;
    }
    else
    {
        // TODO: find the good values
        *pwr_consumption_in_ua =
            ( rx_boosted ) ? LR11XX_GFSK_RX_BOOSTED_CONSUMPTION_LDO : LR11XX_GFSK_RX_CONSUMPTION_LDO;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_bsp_get_instantaneous_lora_rx_power_consumption( const void *context,
                                                                         lr11xx_system_reg_mode_t radio_reg_mode,
                                                                         const bool               rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua )
{
    if( radio_reg_mode == LR11XX_SYSTEM_REG_MODE_DCDC )
    {
        *pwr_consumption_in_ua =
            ( rx_boosted ) ? LR11XX_LORA_RX_BOOSTED_CONSUMPTION_DCDC : LR11XX_LORA_RX_CONSUMPTION_DCDC;
    }
    else
    {
        // TODO: find the good values
        *pwr_consumption_in_ua =
            ( rx_boosted ) ? LR11XX_LORA_RX_BOOSTED_CONSUMPTION_LDO : LR11XX_LORA_RX_CONSUMPTION_LDO;
    }

    return RAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
