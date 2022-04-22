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
#include "smtc_board_pa_pwr_cfg.h"
#include "smtc_hal_mcu.h"
#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

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
    int8_t modem_tx_offset;

    // get modem_configured tx power offset
    if( smtc_modem_get_tx_power_offset_db( 0, &modem_tx_offset ) != SMTC_MODEM_RC_OK )
    {
        // in case rc code is not RC_OK, this function will not return the offset and we need to use no offset (in test
        // mode for example)
        modem_tx_offset = 0;
    }

    int16_t power = input_params->system_output_pwr_in_dbm + modem_tx_offset;

    // check frequency band first
    if( input_params->freq_in_hz >= 2400000000 )
    {
        // Modem is acting in 2g4 band: use HF PA
        // Check power boundaries for HF PA: The output power must be in range [ -18 , +13 ] dBm
        if( power < -18 )
        {
            power = -18;
        }
        else if( power > 13 )
        {
            power = 13;
        }
    }
    else
    {
        // Modem is acting in subgig band: use LP/HP PA
        // Check power boundaries for LP/HP PA: The output power must be in range [ -17 , +22 ] dBm
        if( power < -17 )
        {
            power = -17;
        }
        else if( power > 22 )
        {
            power = 22;
        }
    }
    // get the pa configuration given the frequency and expected output power
    const smtc_board_pa_pwr_cfg_t* pa_pwr_cfg = smtc_board_get_pa_pwr_cfg( input_params->freq_in_hz, ( int8_t ) power );

    if( pa_pwr_cfg == NULL )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    // Fill the output_params structure
    output_params->pa_cfg.pa_duty_cycle = pa_pwr_cfg->pa_config.pa_duty_cycle;
    output_params->pa_cfg.pa_hp_sel     = pa_pwr_cfg->pa_config.pa_hp_sel;
    output_params->pa_cfg.pa_reg_supply = pa_pwr_cfg->pa_config.pa_reg_supply;
    output_params->pa_cfg.pa_sel        = pa_pwr_cfg->pa_config.pa_sel;

    output_params->chip_output_pwr_in_dbm_configured = pa_pwr_cfg->power;
    output_params->chip_output_pwr_in_dbm_expected   = ( int8_t ) power;

    output_params->pa_ramp_time = LR11XX_RADIO_RAMP_48_US;
}

void ral_lr11xx_bsp_get_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
    // TODO: manage reg mode context saving, for the moment assume that LR11XX is in DCDC reg mode
    *reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;
}

void ral_lr11xx_bsp_get_xosc_cfg( const void* context, bool* tcxo_is_radio_controlled,
                                  lr11xx_system_tcxo_supply_voltage_t* supply_voltage, uint32_t* startup_time_in_tick )
{
    // Radio control TCXO 1.8V and 5 ms of startup time
    *tcxo_is_radio_controlled = true;
    *supply_voltage           = LR11XX_SYSTEM_TCXO_CTRL_1_8V;
    *startup_time_in_tick     = 164;  // 5ms in 30.52µs ticks
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
    // Workaround for lr11xx_driver v2.1.0 that contains a bug in command bytes management
    // rssi_calibration_table structure members are not written in good order during spi transaction
    // g4 gain value has to be put in g10, g5 in g11, g6 in g8, g7 in g9, g8 in g6, g9 in g7, g10 in g4, g11 in g5, g12
    // in g13hp5, g13 in g13hp6, g13hp1 in g13hp3, g13hp2 in g13hp4, g13hp3 in g13hp1, g13hp4 in g13hp2, g13hp5 in g12,
    // g13hp6 in g13

    if( freq_in_hz <= 600000000 )
    {
        rssi_calibration_table->gain_offset      = 0;
        rssi_calibration_table->gain_tune.g10    = 12;
        rssi_calibration_table->gain_tune.g11    = 12;
        rssi_calibration_table->gain_tune.g8     = 14;
        rssi_calibration_table->gain_tune.g9     = 0;
        rssi_calibration_table->gain_tune.g6     = 1;
        rssi_calibration_table->gain_tune.g7     = 3;
        rssi_calibration_table->gain_tune.g4     = 4;
        rssi_calibration_table->gain_tune.g5     = 4;
        rssi_calibration_table->gain_tune.g13hp5 = 3;
        rssi_calibration_table->gain_tune.g13hp6 = 6;
        rssi_calibration_table->gain_tune.g13hp3 = 6;
        rssi_calibration_table->gain_tune.g13hp4 = 6;
        rssi_calibration_table->gain_tune.g13hp1 = 6;
        rssi_calibration_table->gain_tune.g13hp2 = 6;
        rssi_calibration_table->gain_tune.g12    = 6;
        rssi_calibration_table->gain_tune.g13    = 6;
        rssi_calibration_table->gain_tune.g13hp7 = 6;
    }
    else if( ( 600000000 <= freq_in_hz ) && ( freq_in_hz <= 2000000000 ) )
    {
        rssi_calibration_table->gain_offset      = 0;
        rssi_calibration_table->gain_tune.g10    = 2;
        rssi_calibration_table->gain_tune.g11    = 2;
        rssi_calibration_table->gain_tune.g8     = 2;
        rssi_calibration_table->gain_tune.g9     = 3;
        rssi_calibration_table->gain_tune.g6     = 3;
        rssi_calibration_table->gain_tune.g7     = 4;
        rssi_calibration_table->gain_tune.g4     = 5;
        rssi_calibration_table->gain_tune.g5     = 4;
        rssi_calibration_table->gain_tune.g13hp5 = 4;
        rssi_calibration_table->gain_tune.g13hp6 = 6;
        rssi_calibration_table->gain_tune.g13hp3 = 5;
        rssi_calibration_table->gain_tune.g13hp4 = 5;
        rssi_calibration_table->gain_tune.g13hp1 = 6;
        rssi_calibration_table->gain_tune.g13hp2 = 6;
        rssi_calibration_table->gain_tune.g12    = 6;
        rssi_calibration_table->gain_tune.g13    = 7;
        rssi_calibration_table->gain_tune.g13hp7 = 6;
    }
    else  // freq_in_hz > 2000000000
    {
        rssi_calibration_table->gain_offset      = 2030;
        rssi_calibration_table->gain_tune.g10    = 6;
        rssi_calibration_table->gain_tune.g11    = 7;
        rssi_calibration_table->gain_tune.g8     = 6;
        rssi_calibration_table->gain_tune.g9     = 4;
        rssi_calibration_table->gain_tune.g6     = 3;
        rssi_calibration_table->gain_tune.g7     = 4;
        rssi_calibration_table->gain_tune.g4     = 14;
        rssi_calibration_table->gain_tune.g5     = 12;
        rssi_calibration_table->gain_tune.g13hp5 = 14;
        rssi_calibration_table->gain_tune.g13hp6 = 12;
        rssi_calibration_table->gain_tune.g13hp3 = 12;
        rssi_calibration_table->gain_tune.g13hp4 = 12;
        rssi_calibration_table->gain_tune.g13hp1 = 12;
        rssi_calibration_table->gain_tune.g13hp2 = 8;
        rssi_calibration_table->gain_tune.g12    = 8;
        rssi_calibration_table->gain_tune.g13    = 9;
        rssi_calibration_table->gain_tune.g13hp7 = 9;
    }
}

/* --- EOF ------------------------------------------------------------------ */
