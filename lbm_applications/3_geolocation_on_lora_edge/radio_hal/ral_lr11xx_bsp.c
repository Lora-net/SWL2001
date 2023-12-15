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

#define LR11XX_PWR_VREG_VBAT_SWITCH 8

#define LR11XX_MIN_PWR_LP_LF -17
#define LR11XX_MAX_PWR_LP_LF 15

#define LR11XX_MIN_PWR_HP_LF -9
#define LR11XX_MAX_PWR_HP_LF 22

#define LR11XX_MIN_PWR_PA_HF -18
#define LR11XX_MAX_PWR_PA_HF 13

const lr11xx_pa_pwr_cfg_t pa_lp_cfg_table[LR11XX_MAX_PWR_LP_LF - LR11XX_MIN_PWR_LP_LF + 1] = LR11XX_PA_LP_LF_CFG_TABLE;
const lr11xx_pa_pwr_cfg_t pa_hp_cfg_table[LR11XX_MAX_PWR_HP_LF - LR11XX_MIN_PWR_HP_LF + 1] = LR11XX_PA_HP_LF_CFG_TABLE;

const lr11xx_pa_pwr_cfg_t pa_hf_cfg_table[LR11XX_MAX_PWR_PA_HF - LR11XX_MIN_PWR_PA_HF + 1] = LR11XX_PA_HF_CFG_TABLE;

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
    // Get startup value defined in modem_hal to avoid mis-alignment
    uint32_t startup_time_ms = smtc_modem_hal_get_radio_tcxo_startup_delay_ms( );
    *xosc_cfg                = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
    *supply_voltage          = LR11XX_SYSTEM_TCXO_CTRL_1_8V;
    // tick is 30.52µs
    *startup_time_in_tick = lr11xx_radio_convert_time_in_ms_to_rtc_step( startup_time_ms );
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

void ral_lr11xx_bsp_get_lora_cad_det_peak( ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol,
                                           uint8_t* in_out_cad_det_peak )
{
    // Function used to fine tune the cad detection peak, update if needed
}

void ral_lr11xx_bsp_get_rx_boost_cfg( const void* context, bool* rx_boost_is_activated )
{
    *rx_boost_is_activated = false;
}

void ral_lr11xx_bsp_get_lfclk_cfg_in_sleep( const void* context, bool* lfclk_is_running )
{
    *lfclk_is_running = true;
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
        if( power < LR11XX_MIN_PWR_LP_LF )
        {
            power = LR11XX_MIN_PWR_LP_LF;
        }
        else if( power > LR11XX_MAX_PWR_LP_LF )
        {
            power = LR11XX_MAX_PWR_LP_LF;
        }
        output_params->pa_cfg.pa_sel                     = LR11XX_RADIO_PA_SEL_LP;
        output_params->pa_cfg.pa_reg_supply              = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        output_params->pa_cfg.pa_duty_cycle              = pa_lp_cfg_table[power - LR11XX_MIN_PWR_LP_LF].pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = pa_lp_cfg_table[power - LR11XX_MIN_PWR_LP_LF].pa_hp_sel;
        output_params->chip_output_pwr_in_dbm_configured = pa_lp_cfg_table[power - LR11XX_MIN_PWR_LP_LF].power;
        output_params->chip_output_pwr_in_dbm_expected   = power;
        break;
    }
    case LR11XX_WITH_LF_HP_PA:
    {
        // Check power boundaries for HP LF PA: The output power must be in range [ -9 , +22 ] dBm
        if( power < LR11XX_MIN_PWR_HP_LF )
        {
            power = LR11XX_MIN_PWR_HP_LF;
        }
        else if( power > LR11XX_MAX_PWR_HP_LF )
        {
            power = LR11XX_MAX_PWR_HP_LF;
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

        output_params->pa_cfg.pa_duty_cycle              = pa_hp_cfg_table[power - LR11XX_MIN_PWR_HP_LF].pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = pa_hp_cfg_table[power - LR11XX_MIN_PWR_HP_LF].pa_hp_sel;
        output_params->chip_output_pwr_in_dbm_configured = pa_hp_cfg_table[power - LR11XX_MIN_PWR_HP_LF].power;
        break;
    }
    case LR11XX_WITH_LF_LP_HP_PA:
    {
        // Check power boundaries for LP/HP LF PA: The output power must be in range [ -17 , +22 ] dBm
        if( power < LR11XX_MIN_PWR_LP_LF )
        {
            power = LR11XX_MIN_PWR_LP_LF;
        }
        else if( power > LR11XX_MAX_PWR_HP_LF )
        {
            power = LR11XX_MAX_PWR_HP_LF;
        }
        output_params->chip_output_pwr_in_dbm_expected = power;

        if( power <= LR11XX_MAX_PWR_LP_LF )
        {
            output_params->pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_LP;
            output_params->pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
            output_params->pa_cfg.pa_duty_cycle = pa_lp_cfg_table[power - LR11XX_MIN_PWR_LP_LF].pa_duty_cycle;
            output_params->pa_cfg.pa_hp_sel     = pa_lp_cfg_table[power - LR11XX_MIN_PWR_LP_LF].pa_hp_sel;
            output_params->chip_output_pwr_in_dbm_configured = pa_lp_cfg_table[power - LR11XX_MIN_PWR_LP_LF].power;
        }
        else
        {
            output_params->pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_HP;
            output_params->pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT;
            output_params->pa_cfg.pa_duty_cycle = pa_hp_cfg_table[power - LR11XX_MIN_PWR_HP_LF].pa_duty_cycle;
            output_params->pa_cfg.pa_hp_sel     = pa_hp_cfg_table[power - LR11XX_MIN_PWR_HP_LF].pa_hp_sel;
            output_params->chip_output_pwr_in_dbm_configured = pa_hp_cfg_table[power - LR11XX_MIN_PWR_HP_LF].power;
        }
        break;
    }
    case LR11XX_WITH_HF_PA:
    {
        // Check power boundaries for HF PA: The output power must be in range [ -18 , +13 ] dBm
        if( power < LR11XX_MIN_PWR_PA_HF )
        {
            power = LR11XX_MIN_PWR_PA_HF;
        }
        else if( power > LR11XX_MAX_PWR_PA_HF )
        {
            power = LR11XX_MAX_PWR_PA_HF;
        }
        output_params->pa_cfg.pa_sel                     = LR11XX_RADIO_PA_SEL_HF;
        output_params->pa_cfg.pa_reg_supply              = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        output_params->pa_cfg.pa_duty_cycle              = pa_hf_cfg_table[power - LR11XX_MIN_PWR_PA_HF].pa_duty_cycle;
        output_params->pa_cfg.pa_hp_sel                  = pa_hf_cfg_table[power - LR11XX_MIN_PWR_PA_HF].pa_hp_sel;
        output_params->chip_output_pwr_in_dbm_configured = pa_hf_cfg_table[power - LR11XX_MIN_PWR_PA_HF].power;
        output_params->chip_output_pwr_in_dbm_expected   = power;
        break;
    }
    }
}

/* --- EOF ------------------------------------------------------------------ */
