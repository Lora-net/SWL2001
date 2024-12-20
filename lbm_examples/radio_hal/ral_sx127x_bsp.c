/*!
 * \file      ral_sx127x_bsp.c
 *
 * \brief     Implements the HAL functions for SX127X
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

#include "ral_sx127x_bsp.h"
#include "radio_utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#define SX1276MB1LAS 0
#define SX1276MB1MAS 1

#define SX1276_MBED_SHIELD SX1276MB1MAS

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

void ral_sx127x_bsp_get_tx_cfg( const void* context, const ral_sx127x_bsp_tx_cfg_input_params_t* input_params,
                                ral_sx127x_bsp_tx_cfg_output_params_t* output_params )

{
    // get board tx power offset
    int8_t board_tx_pwr_offset_db = radio_utilities_get_tx_power_offset( );

    int16_t power = input_params->system_output_pwr_in_dbm + board_tx_pwr_offset_db;

#if defined( SX1272 )  // SX1272MB2DAS
    output_params->pa_cfg.pa_select           = SX127X_PA_SELECT_RFO;
    output_params->pa_cfg.is_20_dbm_output_on = false;
#elif defined( SX1276 )
#if( SX1276_MBED_SHIELD == SX1276MB1LAS )
    if( input_params->freq_in_hz > RF_FREQUENCY_MID_BAND_THRESHOLD )
    {
        output_params->pa_cfg.pa_select           = SX127X_PA_SELECT_BOOST;
        output_params->pa_cfg.is_20_dbm_output_on = true;
    }
    else
    {
        output_params->pa_cfg.pa_select           = SX127X_PA_SELECT_RFO;
        output_params->pa_cfg.is_20_dbm_output_on = false;
    }
#elif( SX1276_MBED_SHIELD == SX1276MB1MAS )
    output_params->pa_cfg.pa_select           = SX127X_PA_SELECT_RFO;
    output_params->pa_cfg.is_20_dbm_output_on = false;
#else
#error "Please define the mbed shield to be used"
#endif
#else
#error "Please define the radio to be used"
#endif

    output_params->chip_output_pwr_in_dbm_configured = power;
    output_params->chip_output_pwr_in_dbm_expected   = power;

    if( ( output_params->pa_cfg.pa_select == SX127X_PA_SELECT_BOOST ) )
    {  // PA_BOOST
        if( output_params->pa_cfg.is_20_dbm_output_on == true )
        {
            if( power < 5 )
            {
                output_params->chip_output_pwr_in_dbm_configured = 5;
                output_params->chip_output_pwr_in_dbm_expected   = 5;
            }
            if( power > 20 )
            {
                output_params->chip_output_pwr_in_dbm_configured = 20;
                output_params->chip_output_pwr_in_dbm_expected   = 20;
            }
        }
        else
        {
            if( power < 2 )
            {
                output_params->chip_output_pwr_in_dbm_configured = 2;
                output_params->chip_output_pwr_in_dbm_expected   = 2;
            }
            if( power > 17 )
            {
                output_params->chip_output_pwr_in_dbm_configured = 17;
                output_params->chip_output_pwr_in_dbm_expected   = 17;
            }
        }
    }
    else
    {  // RFO
#if defined( SX1272 )
        if( power < -1 )
        {
            output_params->chip_output_pwr_in_dbm_configured = -1;
            output_params->chip_output_pwr_in_dbm_expected   = -1;
        }
        if( power > 14 )
        {
            output_params->chip_output_pwr_in_dbm_configured = 14;
            output_params->chip_output_pwr_in_dbm_expected   = 14;
        }
#else  // SX1276
        if( power < -4 )
        {
            output_params->chip_output_pwr_in_dbm_configured = -4;
            output_params->chip_output_pwr_in_dbm_expected   = -4;
        }
        if( power > 15 )
        {
            output_params->chip_output_pwr_in_dbm_configured = 15;
            output_params->chip_output_pwr_in_dbm_expected   = 15;
        }
#endif
    }

    output_params->pa_ramp_time = SX127X_RAMP_40_US;
}

void ral_sx127x_bsp_get_ocp_value( const void* context, uint8_t* ocp_trim_value )
{
    // Do nothing, let the driver choose the default values
}

ral_status_t ral_sx127x_bsp_get_instantaneous_tx_power_consumption( const void *context,
    const ral_sx127x_bsp_tx_cfg_output_params_t* tx_cfg_output_params, uint32_t* pwr_consumption_in_ua )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx127x_bsp_get_instantaneous_gfsk_rx_power_consumption( const void *context,
                                                                         bool      rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx127x_bsp_get_instantaneous_lora_rx_power_consumption( const void *context,
                                                                         bool      rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
