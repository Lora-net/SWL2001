/**
 * @file      ral_sx128x_bsp.c
 *
 * @brief     Board Support Package for the SX128x-specific Radio Abstraction Layer.
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

#include <stdint.h>

#include "smtc_modem_hal.h"
#include "ral_sx128x_bsp.h"
#include "radio_utilities.h"

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
void ral_sx128x_bsp_get_reg_mode( const void* context, sx128x_reg_mod_t* reg_mode )
{
    *reg_mode = SX128X_REG_MODE_DCDC;
}

void ral_sx128x_bsp_get_tx_cfg( const void* context, const ral_sx128x_bsp_tx_cfg_input_params_t* input_params,
                                ral_sx128x_bsp_tx_cfg_output_params_t* output_params )
{
    // get board tx power offset
    int8_t board_tx_pwr_offset_db = radio_utilities_get_tx_power_offset( );

    int16_t power = input_params->system_output_pwr_in_dbm + board_tx_pwr_offset_db;

    if( power > 13 )
    {
        output_params->chip_output_pwr_in_dbm_configured = 13;
        output_params->chip_output_pwr_in_dbm_expected   = 13;
    }
    else if( power < -18 )
    {
        output_params->chip_output_pwr_in_dbm_configured = -18;
        output_params->chip_output_pwr_in_dbm_expected   = -18;
    }
    else
    {
        output_params->chip_output_pwr_in_dbm_configured = ( int8_t ) power;
        output_params->chip_output_pwr_in_dbm_expected   = ( int8_t ) power;
    }

    output_params->pa_ramp_time = SX128X_RAMP_20_US;
}

void ral_sx128x_bsp_get_lora_cad_det_peak( ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol,
                                           uint8_t* in_out_cad_det_peak )
{
    // Function used to fine tune the cad detection peak, update if needed
}