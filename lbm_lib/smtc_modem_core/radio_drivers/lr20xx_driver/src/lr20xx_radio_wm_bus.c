/*!
 * @file      lr20xx_radio_wm_bus.c
 *
 * @brief     Radio Wireless M-Bus driver implementation for LR20XX
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

#include "lr20xx_radio_wm_bus.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_WM_BUS_SET_PKT_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR20XX_RADIO_WM_BUS_GET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_WM_BUS_SET_ADDR_CMD_LENGTH ( 2 )

#define LR20XX_RADIO_WM_BUS_GET_RX_STATS_RBUFFER_LENGTH ( 6 )
#define LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_RBUFFER_LENGTH ( 9 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for Wireless M-BUS related operations
 */
enum
{
    LR20XX_RADIO_WM_BUS_SET_PARAMS_OC     = 0x026A,
    LR20XX_RADIO_WM_BUS_GET_RX_STATS_OC   = 0x026C,
    LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_OC = 0x026D,
    LR20XX_RADIO_WM_BUS_SET_ADDR_OC       = 0x026E,
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

lr20xx_status_t lr20xx_radio_wm_bus_set_params( const void* context, const lr20xx_radio_wm_bus_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_WM_BUS_SET_PKT_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_SET_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_SET_PARAMS_OC >> 0 ),
        ( uint8_t ) params->mode,
        ( uint8_t ) params->rx_bw,
        ( uint8_t ) params->pkt_format,
        ( uint8_t ) params->addr_filtering,
        ( uint8_t ) params->payload_length,
        ( uint8_t ) ( params->preamble_len >> 8 ),
        ( uint8_t ) ( params->preamble_len >> 0 ),
        params->preamble_len_detect
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_WM_BUS_SET_PKT_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_wm_bus_get_rx_stats( const void* context, lr20xx_radio_wm_bus_rx_stats_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_WM_BUS_GET_RX_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_GET_RX_STATS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_GET_RX_STATS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_WM_BUS_GET_RX_STATS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_WM_BUS_GET_RX_STATS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_WM_BUS_GET_RX_STATS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->received_packets = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
        statistics->crc_errors       = ( uint16_t ) ( ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3] );
        statistics->length_errors    = ( uint16_t ) ( ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_wm_bus_get_pkt_status( const void* context, lr20xx_radio_wm_bus_pkt_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_WM_BUS_GET_PKT_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->l_field = rbuffer[0];
        pkt_status->packet_length_bytes =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[1] ) << 8 ) + ( ( uint16_t ) rbuffer[2] ) );
        pkt_status->rssi_avg_in_dbm  = ( int16_t ) ( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->rssi_sync_in_dbm = ( int16_t ) ( -( ( int16_t ) rbuffer[4] ) );
        pkt_status->crc_errors = ( uint32_t ) ( ( ( uint32_t ) rbuffer[5] << 9 ) + ( ( uint32_t ) rbuffer[6] << 1 ) +
                                                ( ( ( uint32_t ) rbuffer[7] >> 6 ) & 0x01 ) );
        pkt_status->pkt_format = ( lr20xx_radio_wm_bus_pkt_format_t ) ( ( rbuffer[7] >> 7 ) & 0x01 );
        pkt_status->rssi_avg_half_dbm_count  = ( rbuffer[7] >> 4 ) & 0x01;
        pkt_status->rssi_sync_half_dbm_count = ( rbuffer[7] >> 0 ) & 0x01;
        pkt_status->link_quality_indicator   = rbuffer[8];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_wm_bus_set_addr( const void* context, const uint8_t addr[LR20XX_RADIO_WM_BUS_ADDR_LEN] )
{
    const uint8_t cbuffer[LR20XX_RADIO_WM_BUS_SET_ADDR_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_SET_ADDR_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_WM_BUS_SET_ADDR_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_WM_BUS_SET_ADDR_CMD_LENGTH, addr,
                                                 LR20XX_RADIO_WM_BUS_ADDR_LEN );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
