/*!
 * @file      lr20xx_radio_flrc.c
 *
 * @brief     FLRC radio driver implementation for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#include "lr20xx_radio_flrc.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_FLRC_SET_MODULATION_PARAMS_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_FLRC_SET_PKT_PARAMS_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RADIO_FLRC_GET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FLRC_GET_PKT_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FLRC_SET_SYNCWORD_CMD_LENGTH ( 2 + 1 )

#define LR20XX_RADIO_FLRC_GET_RX_STATS_RBUFFER_LENGTH ( 6 )
#define LR20XX_RADIO_FLRC_GET_PKT_STATUS_RBUFFER_LENGTH ( 5 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for FLRC-related operations
 */
enum
{
    LR20XX_RADIO_FLRC_SET_MODULATION_PARAMS_OC = 0x0248,
    LR20XX_RADIO_FLRC_SET_PKT_PARAMS_OC        = 0x0249,
    LR20XX_RADIO_FLRC_GET_RX_STATS_OC          = 0x024A,
    LR20XX_RADIO_FLRC_GET_PKT_STATUS_OC        = 0x024B,
    LR20XX_RADIO_FLRC_SET_SYNCWORD_OC          = 0x024C,
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

lr20xx_status_t lr20xx_radio_flrc_set_modulation_params( const void*                           context,
                                                         const lr20xx_radio_flrc_mod_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_FLRC_SET_MODULATION_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_MODULATION_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_MODULATION_PARAMS_OC >> 0 ),
        ( uint8_t ) params->br_bw,
        ( uint8_t ) ( ( params->cr << 4 ) + params->shape ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FLRC_SET_MODULATION_PARAMS_CMD_LENGTH,
                                                 0, 0 );
}

lr20xx_status_t lr20xx_radio_flrc_set_pkt_params( const void* context, const lr20xx_radio_flrc_pkt_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_FLRC_SET_PKT_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_PKT_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_PKT_PARAMS_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) ( params->sync_word_len ) ) + ( params->preamble_len << 2 ) ),
        ( uint8_t ) ( ( ( uint8_t ) ( params->crc_type ) ) + ( ( uint8_t ) ( params->header_type ) << 2 ) +
                      ( ( uint8_t ) ( params->match_sync_word ) << 3 ) + ( ( ( uint8_t ) params->tx_syncword ) << 6 ) ),
        ( uint8_t ) ( params->pld_len_in_bytes >> 8 ),
        ( uint8_t ) ( params->pld_len_in_bytes >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FLRC_SET_PKT_PARAMS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_flrc_get_rx_stats( const void* context, lr20xx_radio_flrc_rx_stats_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_FLRC_GET_RX_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FLRC_GET_RX_STATS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FLRC_GET_RX_STATS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_FLRC_GET_RX_STATS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_FLRC_GET_RX_STATS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_FLRC_GET_RX_STATS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->received_packets = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + rbuffer[1] );
        statistics->crc_errors       = ( uint16_t ) ( ( ( uint16_t ) rbuffer[2] << 8 ) + rbuffer[3] );
        statistics->length_errors    = ( uint16_t ) ( ( ( uint16_t ) rbuffer[4] << 8 ) + rbuffer[5] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_flrc_get_pkt_status( const void* context, lr20xx_radio_flrc_pkt_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_FLRC_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FLRC_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FLRC_GET_PKT_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_FLRC_GET_PKT_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_FLRC_GET_PKT_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_FLRC_GET_PKT_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->packet_length_bytes =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[0] ) << 8 ) + ( ( uint16_t ) rbuffer[1] ) );
        pkt_status->rssi_avg_in_dbm          = ( int16_t ) ( -( ( int16_t ) rbuffer[2] ) );
        pkt_status->rssi_sync_in_dbm         = ( int16_t ) ( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->rssi_sync_half_dbm_count = ( rbuffer[4] >> 0 ) & 0x01;
        pkt_status->rssi_avg_half_dbm_count  = ( rbuffer[4] >> 2 ) & 0x01;
        pkt_status->syncword_index           = ( rbuffer[4] >> 4 );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_flrc_set_syncword( const void* context, uint8_t syncword_index,
                                                const uint8_t syncword[LR20XX_RADIO_FLRC_SYNCWORD_LENGTH] )
{
    const uint8_t cbuffer[LR20XX_RADIO_FLRC_SET_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_SYNCWORD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_SYNCWORD_OC >> 0 ),
        syncword_index,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FLRC_SET_SYNCWORD_CMD_LENGTH, syncword,
                                                 LR20XX_RADIO_FLRC_SYNCWORD_LENGTH );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
