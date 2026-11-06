/*!
 * @file      lr20xx_radio_bluetooth_le.c
 *
 * @brief     Bluetooth_LE radio driver implementation for LR20XX
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

#include "lr20xx_radio_bluetooth_le.h"
#include "lr20xx_workarounds.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_BLUETOOTH_LE_SET_MODULATION_PARAMS_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_BLUETOOTH_LE_SET_PKT_PARAMS_CMD_LENGTH ( 2 + 9 )
#define LR20XX_RADIO_BLUETOOTH_LE_SET_TX_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_BLUETOOTH_LE_SET_PDU_LENGTH_CMD_LENGTH ( 2 + 1 )

#define LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_RBUFFER_LENGTH ( 6 )
#define LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_RBUFFER_LENGTH ( 6 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for Bluetooth_LE related operations
 */
enum
{
    LR20XX_RADIO_BLUETOOTH_LE_SET_MODULATION_PARAMS_OC = 0x0260,
    LR20XX_RADIO_BLUETOOTH_LE_SET_PKT_PARAMS_OC        = 0x0261,
    LR20XX_RADIO_BLUETOOTH_LE_SET_TX_OC                = 0x0262,
    LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_OC          = 0x0264,
    LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_OC        = 0x0265,
    LR20XX_RADIO_BLUETOOTH_LE_SET_PDU_LENGTH_OC        = 0x0266,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static lr20xx_status_t lr20xx_radio_bluetooth_le_conditionally_apply_phy_coded_syncword_workaround(
    const void* context, lr20xx_radio_bluetooth_le_phy_t phy );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_bluetooth_le_set_modulation_params( const void*                           context,
                                                                 const lr20xx_radio_bluetooth_le_phy_t phy )
{
    const uint8_t cbuffer[LR20XX_RADIO_BLUETOOTH_LE_SET_MODULATION_PARAMS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_MODULATION_PARAMS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_MODULATION_PARAMS_OC >> 0 ),
        ( uint8_t ) phy,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_BLUETOOTH_LE_SET_MODULATION_PARAMS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_bluetooth_le_set_pkt_params( const void*                                   context,
                                                          const lr20xx_radio_bluetooth_le_pkt_params_t* pkt_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_BLUETOOTH_LE_SET_PKT_PARAMS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_PKT_PARAMS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_PKT_PARAMS_OC >> 0 ),
        ( uint8_t )( ( ( uint8_t )( pkt_params->is_crc_in_fifo ? 0x10 : 0x00 ) ) |
                     ( ( uint8_t ) pkt_params->phy_channel_pdu ) ),
        pkt_params->whitening_init,
        ( uint8_t )( pkt_params->crc_init >> 16 ),
        ( uint8_t )( pkt_params->crc_init >> 8 ),
        ( uint8_t )( pkt_params->crc_init >> 0 ),
        ( uint8_t )( pkt_params->sync_word >> 24 ),
        ( uint8_t )( pkt_params->sync_word >> 16 ),
        ( uint8_t )( pkt_params->sync_word >> 8 ),
        ( uint8_t )( pkt_params->sync_word >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_BLUETOOTH_LE_SET_PKT_PARAMS_CMD_LENGTH,
                                                 0, 0 );
}

#define RETURN_STATUS_ON_NOT_OK( call )      \
    do                                       \
    {                                        \
        const lr20xx_status_t status = call; \
        if( status != LR20XX_STATUS_OK )     \
        {                                    \
            return status;                   \
        }                                    \
    } while( 0 )

lr20xx_status_t lr20xx_radio_bluetooth_le_set_modulation_pkt_params(
    const void* context, const lr20xx_radio_bluetooth_le_phy_t phy,
    const lr20xx_radio_bluetooth_le_pkt_params_t* pkt_params )
{
    RETURN_STATUS_ON_NOT_OK( lr20xx_radio_bluetooth_le_set_modulation_params( context, phy ) );
    RETURN_STATUS_ON_NOT_OK( lr20xx_radio_bluetooth_le_set_pkt_params( context, pkt_params ) );
    RETURN_STATUS_ON_NOT_OK(
        lr20xx_radio_bluetooth_le_conditionally_apply_phy_coded_syncword_workaround( context, phy ) );

    // If this point is reached, it means that none of the above returned with a not OK status. Therefore an OK status
    // must be returned here.
    return LR20XX_STATUS_OK;
}
#undef RETURN_STATUS_ON_NOT_OK

lr20xx_status_t lr20xx_radio_bluetooth_le_set_tx( const void* context, uint8_t len )
{
    const uint8_t cbuffer[LR20XX_RADIO_BLUETOOTH_LE_SET_TX_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_TX_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_TX_OC >> 0 ),
        len,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_BLUETOOTH_LE_SET_TX_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_bluetooth_le_get_rx_stats( const void*                           context,
                                                        lr20xx_radio_bluetooth_le_rx_stats_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_CMD_LENGTH,
                                             rbuffer, LR20XX_RADIO_BLUETOOTH_LE_GET_RX_STATS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->received_packets = ( uint16_t )( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
        statistics->crc_errors       = ( uint16_t )( ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3] );
        statistics->length_errors    = ( uint16_t )( ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_bluetooth_le_get_pkt_status( const void*                             context,
                                                          lr20xx_radio_bluetooth_le_pkt_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_CMD_LENGTH,
                                             rbuffer, LR20XX_RADIO_BLUETOOTH_LE_GET_PKT_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->packet_length_bytes =
            ( uint16_t )( ( ( ( uint16_t ) rbuffer[0] ) << 8 ) + ( ( uint16_t ) rbuffer[1] ) );
        pkt_status->rssi_avg_in_dbm          = ( int16_t )( -( ( int16_t ) rbuffer[2] ) );
        pkt_status->rssi_sync_in_dbm         = ( int16_t )( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->rssi_avg_half_dbm_count  = ( rbuffer[4] >> 2 ) & 0x01;
        pkt_status->rssi_sync_half_dbm_count = ( rbuffer[4] >> 0 ) & 0x01;
        pkt_status->link_quality_indicator   = rbuffer[5];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_bluetooth_le_set_pdu_length( const void* context, uint8_t len )
{
    const uint8_t cbuffer[LR20XX_RADIO_BLUETOOTH_LE_SET_PDU_LENGTH_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_PDU_LENGTH_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_BLUETOOTH_LE_SET_PDU_LENGTH_OC >> 0 ),
        len,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_BLUETOOTH_LE_SET_PDU_LENGTH_CMD_LENGTH,
                                                 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

lr20xx_status_t lr20xx_radio_bluetooth_le_conditionally_apply_phy_coded_syncword_workaround(
    const void* context, lr20xx_radio_bluetooth_le_phy_t phy )
{
    switch( phy )
    {
    case LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB:
    case LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB:
    {
        return lr20xx_workarounds_bluetooth_le_phy_coded_syncwords( context );
    }
    default:
    {
        return LR20XX_STATUS_OK;
    }
    }
}

/* --- EOF ------------------------------------------------------------------ */
