/*!
 * @file      sx126x_bpsk.c
 *
 * @brief     SX126x BPSK radio driver implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#include "sx126x_bpsk.h"
#include "sx126x_hal.h"
#include "sx126x.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ 32000000UL

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum sx126x_commands_e
{
    SX126X_SET_MODULATION_PARAMS = 0x8B,
    SX126X_SET_PKT_PARAMS        = 0x8C,
} sx126x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    SX126X_SIZE_SET_MODULATION_PARAMS_BPSK = 5,
    SX126X_SIZE_SET_PKT_PARAMS_BPSK        = 2,
} sx126x_commands_size_t;

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

sx126x_status_t sx126x_set_bpsk_mod_params( const void* context, const sx126x_mod_params_bpsk_t* params )
{
    const uint32_t bitrate = ( uint32_t )( 32 * SX126X_XTAL_FREQ / params->br_in_bps );

    const uint8_t buf[SX126X_SIZE_SET_MODULATION_PARAMS_BPSK] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ),
    };

    sx126x_status_t status = sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_BPSK, 0, 0 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    return sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_BPSK, ( sx126x_lora_bw_t ) 0 );
    // WORKAROUND END
}

sx126x_status_t sx126x_set_bpsk_pkt_params( const void* context, const sx126x_pkt_params_bpsk_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_BPSK] = {
        SX126X_SET_PKT_PARAMS,
        params->pld_len_in_bytes,
    };

    sx126x_status_t status =
        ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_BPSK, 0, 0 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    const uint8_t buf2[] = {
        ( uint8_t )( params->ramp_up_delay >> 8 ),   ( uint8_t )( params->ramp_up_delay >> 0 ),
        ( uint8_t )( params->ramp_down_delay >> 8 ), ( uint8_t )( params->ramp_down_delay >> 0 ),
        ( uint8_t )( params->pld_len_in_bits >> 8 ), ( uint8_t )( params->pld_len_in_bits >> 0 ),
    };

    return sx126x_write_register( context, 0x00F0, buf2, sizeof( buf2 ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
