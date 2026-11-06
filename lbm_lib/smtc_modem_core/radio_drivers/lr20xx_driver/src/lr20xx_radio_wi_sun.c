/*!
 * @file      lr20xx_radio_wi_sun.c
 *
 * @brief     Radio Wi-SUN driver implementation for LR20XX
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

#include "lr20xx_radio_wi_sun.h"
#include "lr20xx_hal.h"
#include "lr20xx_workarounds.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_WI_SUN_SET_OPERATING_MODE_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_WI_SUN_SET_PKT_PARAMS_CMD_LENGTH ( 2 + 5 )
#define LR20XX_RADIO_WI_SUN_SET_PACKET_LENGTH_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_WI_SUN_GET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_CMD_LENGTH ( 2 )

#define LR20XX_RADIO_WI_SUN_GET_RX_STATS_RBUFFER_LENGTH ( 6 )
#define LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_RBUFFER_LENGTH ( 8 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for Wi-SUN related operations
 */
enum
{
    LR20XX_RADIO_WI_SUN_SET_OPERATING_MODE_OC = 0x0270,
    LR20XX_RADIO_WI_SUN_SET_PKT_PARAMS_OC     = 0x0271,
    LR20XX_RADIO_WI_SUN_GET_RX_STATS_OC       = 0x0272,
    LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_OC     = 0x0273,
    LR20XX_RADIO_WI_SUN_SET_PACKET_LENGTH_OC  = 0x0274,
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

lr20xx_status_t lr20xx_radio_wi_sun_set_operating_mode( const void*                                context,
                                                        const lr20xx_radio_wi_sun_operating_mode_t mode,
                                                        lr20xx_radio_fsk_common_bw_t               rx_bw )
{
    const uint8_t cbuffer[LR20XX_RADIO_WI_SUN_SET_OPERATING_MODE_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_SET_OPERATING_MODE_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_SET_OPERATING_MODE_OC >> 0 ),
        ( uint8_t ) mode,
        ( uint8_t ) rx_bw,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_WI_SUN_SET_OPERATING_MODE_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_wi_sun_set_pkt_params( const void*                             context,
                                                    const lr20xx_radio_wi_sun_pkt_params_t* pkt_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_WI_SUN_SET_PKT_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_SET_PKT_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_SET_PKT_PARAMS_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) pkt_params->fcs << 5 ) | ( ( uint8_t ) pkt_params->whitening << 4 ) |
                      ( ( uint8_t ) pkt_params->crc_source << 3 ) | ( ( uint8_t ) pkt_params->payload_mode << 2 ) |
                      ( ( uint8_t ) pkt_params->fec ) ),
        ( uint8_t ) ( pkt_params->tx_payload_len >> 8 ),
        ( uint8_t ) ( pkt_params->tx_payload_len >> 0 ),
        pkt_params->tx_preamble_len_in_bytes,
        pkt_params->rx_preamble_detect_len_in_bits,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_WI_SUN_SET_PKT_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_wi_sun_get_rx_stats( const void* context, lr20xx_radio_wi_sun_rx_stats_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_WI_SUN_GET_RX_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_GET_RX_STATS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_GET_RX_STATS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_WI_SUN_GET_RX_STATS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_WI_SUN_GET_RX_STATS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_WI_SUN_GET_RX_STATS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->received_packets = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
        statistics->crc_errors       = ( uint16_t ) ( ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3] );
        statistics->length_errors    = ( uint16_t ) ( ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_wi_sun_get_pkt_status( const void* context, lr20xx_radio_wi_sun_pkt_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_WI_SUN_GET_PKT_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->rx_header_raw = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
        pkt_status->packet_length_bytes =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[2] ) << 8 ) + ( ( uint16_t ) rbuffer[3] ) );
        pkt_status->rssi_avg_in_dbm          = ( int16_t ) ( -( ( int16_t ) rbuffer[4] ) );
        pkt_status->rssi_sync_in_dbm         = ( int16_t ) ( -( ( int16_t ) rbuffer[5] ) );
        pkt_status->syncword_idx             = ( rbuffer[6] >> 7 ) & 0x01;
        pkt_status->rssi_avg_half_dbm_count  = ( rbuffer[6] >> 2 ) & 0x01;
        pkt_status->rssi_sync_half_dbm_count = ( rbuffer[6] >> 0 ) & 0x01;
        pkt_status->link_quality_indicator   = rbuffer[7];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_wi_sun_set_packet_length( const void* context, uint16_t pkt_len )
{
    const uint8_t cbuffer[LR20XX_RADIO_WI_SUN_SET_PACKET_LENGTH_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_SET_PACKET_LENGTH_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WI_SUN_SET_PACKET_LENGTH_OC >> 0 ),
        ( uint8_t ) ( pkt_len >> 8 ),
        ( uint8_t ) pkt_len,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_WI_SUN_SET_PACKET_LENGTH_CMD_LENGTH, 0,
                                                 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
