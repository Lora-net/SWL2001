/**
 * @file      ralf_lr11xx.c
 *
 * @brief     Radio abstraction layer feature definition
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

#include "ralf_lr11xx.h"
#include "ral.h"

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

ral_status_t ralf_lr11xx_setup_gfsk( const ralf_t* radio, const ralf_params_gfsk_t* params )
{
    ral_status_t status = ral_stop_timer_on_preamble( &radio->ral, false );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_pkt_type( &radio->ral, RAL_PKT_TYPE_GFSK );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_rf_freq( &radio->ral, params->rf_freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_tx_cfg( &radio->ral, params->output_pwr_in_dbm, params->rf_freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_gfsk_mod_params( &radio->ral, &params->mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_gfsk_pkt_params( &radio->ral, &params->pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    if( params->pkt_params.crc_type != RAL_GFSK_CRC_OFF )
    {
        status = ral_set_gfsk_crc_params( &radio->ral, params->crc_seed, params->crc_polynomial );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }
    status =
        ral_set_gfsk_sync_word( &radio->ral, params->sync_word, ( params->pkt_params.sync_word_len_in_bits + 7 ) / 8 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    if( params->dc_free_is_on == true )
    {
        status = ral_set_gfsk_whitening_seed( &radio->ral, params->whitening_seed );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }
    return status;
}

ral_status_t ralf_lr11xx_setup_lora( const ralf_t* radio, const ralf_params_lora_t* params )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_set_pkt_type( &radio->ral, RAL_PKT_TYPE_LORA );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_stop_timer_on_preamble( &radio->ral, false );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_lora_symb_nb_timeout( &radio->ral, params->symb_nb_timeout );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_rf_freq( &radio->ral, params->rf_freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_tx_cfg( &radio->ral, params->output_pwr_in_dbm, params->rf_freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_lora_mod_params( &radio->ral, &params->mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_lora_pkt_params( &radio->ral, &params->pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ral_set_lora_sync_word( &radio->ral, params->sync_word );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    return status;
}

ral_status_t ralf_lr11xx_setup_flrc( const ralf_t* radio, const ralf_params_flrc_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
