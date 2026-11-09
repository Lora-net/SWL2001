/*!
 * @file      lr20xx_radio_bpsk.c
 *
 * @brief     Radio BPSK driver implementation for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#include "lr20xx_radio_bpsk.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_BPSK_SET_MODULATION_PARAMS_CMD_LENGTH ( 2 + 5 )
#define LR20XX_RADIO_BPSK_SET_PACKET_PARAMS_CMD_LENGTH ( 2 + 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR20XX_RADIO_BPSK_SET_MODULATION_PARAMS_OC = 0x0250,
    LR20XX_RADIO_BPSK_SET_PACKET_PARAMS_OC     = 0x0251,
};

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

lr20xx_status_t lr20xx_radio_bpsk_set_mod_params( const void*                           context,
                                                  const lr20xx_radio_bpsk_mod_params_t* mod_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_BPSK_SET_MODULATION_PARAMS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BPSK_SET_MODULATION_PARAMS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BPSK_SET_MODULATION_PARAMS_OC >> 0 ),
        ( uint8_t )( ( ( uint8_t ) mod_params->bitrate_unit << 7 ) + ( uint8_t )( mod_params->bitrate >> 24 ) ),
        ( uint8_t )( mod_params->bitrate >> 16 ),
        ( uint8_t )( mod_params->bitrate >> 8 ),
        ( uint8_t )( mod_params->bitrate >> 0 ),
        ( uint8_t )( ( mod_params->pulse_shape << 4 ) | ( mod_params->differential_encoding_enable << 2 ) |
                     ( mod_params->differential_encoding_init_parity ? 0x02 : 0x00 ) |
                     ( mod_params->differential_encoding_init_previous ? 0x01 : 0x00 ) ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_BPSK_SET_MODULATION_PARAMS_CMD_LENGTH,
                                                 0, 0 );
}

lr20xx_status_t lr20xx_radio_bpsk_set_pkt_params( const void*                           context,
                                                  const lr20xx_radio_bpsk_pkt_params_t* pkt_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_BPSK_SET_PACKET_PARAMS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BPSK_SET_PACKET_PARAMS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BPSK_SET_PACKET_PARAMS_OC >> 0 ),
        pkt_params->payload_length,
        ( uint8_t )( ( ( ( uint8_t ) pkt_params->phy_mode ) << 4 ) |
                     ( ( ( uint8_t ) pkt_params->sigfox_message_type ) << 2 ) |
                     ( uint8_t )( pkt_params->sigfox_frame_rank ) ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_BPSK_SET_PACKET_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
