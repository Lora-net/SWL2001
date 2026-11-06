/*!
 * @file      lr20xx_radio_z_wave.c
 *
 * @brief     Radio Z-Wave driver implementation for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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

#include "lr20xx_radio_z_wave.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_Z_WAVE_SET_PKT_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR20XX_RADIO_Z_WAVE_SET_HOMEID_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RADIO_Z_WAVE_GET_RX_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_Z_WAVE_SET_BEAM_FILTERING_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RADIO_Z_WAVE_SET_SCAN_PARAMS_CMD_LENGTH \
    ( 2u + 4u )  // This command has variable length depending on call
#define LR20XX_RADIO_Z_WAVE_SET_SCAN_CMD_LENGTH ( 2 )

#define LR20XX_RADIO_Z_WAVE_GET_RX_STATS_RBUFFER_LENGTH ( 6 )
#define LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_RBUFFER_LENGTH ( 7 )
#define LR20XX_RADIO_Z_WAVE_SCAN_PER_CHANNEL_PARAMS_LENGTH ( 5u )
#define LR20XX_RADIO_Z_WAVE_SCAN_MAX_CHANNEL ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for Z-Wave related operations
 */
enum
{
    LR20XX_RADIO_Z_WAVE_SET_PARAMS_OC                = 0x0297,
    LR20XX_RADIO_Z_WAVE_SET_HOMEID_OC                = 0x0298,
    LR20XX_RADIO_Z_WAVE_GET_RX_STATS_OC              = 0x0299,
    LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_OC            = 0x029A,
    LR20XX_RADIO_Z_WAVE_SET_BEAM_FILTERING_PARAMS_OC = 0x029B,
    LR20XX_RADIO_Z_WAVE_SET_SCAN_PARAMS_OC           = 0x029C,
    LR20XX_RADIO_Z_WAVE_SET_SCAN_OC                  = 0x029D,
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

lr20xx_status_t lr20xx_radio_z_wave_set_params( const void* context, const lr20xx_radio_z_wave_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_SET_PKT_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_PARAMS_OC >> 0 ),
        ( uint8_t ) params->datarate,
        ( uint8_t ) params->rx_bw,
        ( uint8_t ) params->addr_filtering,
        ( uint8_t ) params->payload_length,
        ( uint8_t ) ( params->pbl_len_tx_in_bit >> 8 ),
        ( uint8_t ) ( params->pbl_len_tx_in_bit >> 0 ),
        params->pbl_detect_len_in_bit,
        ( uint8_t ) params->fcs_mode,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_Z_WAVE_SET_PKT_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_z_wave_set_homeid( const void* context, const uint32_t homeid )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_SET_HOMEID_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_HOMEID_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_HOMEID_OC >> 0 ),
        ( uint8_t ) ( homeid >> 24 ),
        ( uint8_t ) ( homeid >> 16 ),
        ( uint8_t ) ( homeid >> 8 ),
        ( uint8_t ) ( homeid >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_Z_WAVE_SET_HOMEID_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_z_wave_get_rx_stats( const void* context, lr20xx_radio_z_wave_rx_stats_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_GET_RX_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_GET_RX_STATS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_GET_RX_STATS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_Z_WAVE_GET_RX_STATS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_Z_WAVE_GET_RX_STATS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_Z_WAVE_GET_RX_STATS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->received_packets = ( uint16_t ) ( ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1] );
        statistics->crc_errors       = ( uint16_t ) ( ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3] );
        statistics->length_errors    = ( uint16_t ) ( ( ( uint16_t ) rbuffer[4] << 8 ) + ( uint16_t ) rbuffer[5] );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_z_wave_get_pkt_status( const void* context, lr20xx_radio_z_wave_pkt_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_Z_WAVE_GET_PKT_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->packet_length_bytes =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[0] ) << 8 ) + ( ( uint16_t ) rbuffer[1] ) );
        pkt_status->rssi_avg_in_dbm          = ( int16_t ) ( -( ( int16_t ) rbuffer[2] ) );
        pkt_status->rssi_sync_in_dbm         = ( int16_t ) ( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->last_pkt_datarate_type   = ( lr20xx_radio_z_wave_datarate_type_t ) ( rbuffer[4] );
        pkt_status->rssi_avg_half_dbm_count  = ( rbuffer[5] >> 2 ) & 0x01;
        pkt_status->rssi_sync_half_dbm_count = ( rbuffer[5] >> 0 ) & 0x01;
        pkt_status->link_quality_indicator   = rbuffer[6];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_z_wave_set_beam_filtering_params(
    const void* context, const lr20xx_radio_z_wave_beam_filtering_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_SET_BEAM_FILTERING_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_BEAM_FILTERING_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_BEAM_FILTERING_PARAMS_OC >> 0 ),
        params->tag,
        ( uint8_t ) ( ( uint8_t ) ( params->address_length << 7 ) + ( ( uint8_t ) ( params->node_id >> 8 ) & 0x0F ) ),
        ( uint8_t ) ( params->node_id >> 0 ),
        params->homeid_hash,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_Z_WAVE_SET_BEAM_FILTERING_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_z_wave_set_scan_params( const void*                              context,
                                                     const lr20xx_radio_z_wave_scan_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_SET_SCAN_PARAMS_CMD_LENGTH +
                          ( LR20XX_RADIO_Z_WAVE_SCAN_PER_CHANNEL_PARAMS_LENGTH *
                            LR20XX_RADIO_Z_WAVE_SCAN_MAX_CHANNEL )] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_SCAN_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_SCAN_PARAMS_OC >> 0 ),
        ( uint8_t ) ( ( params->channel_cfg[0].enable_cca ? 0x01 : 0 ) |
                      ( params->channel_cfg[1].enable_cca ? 0x02 : 0 ) |
                      ( params->channel_cfg[2].enable_cca ? 0x04 : 0 ) |
                      ( params->channel_cfg[3].enable_cca ? 0x08 : 0 ) | ( params->number_of_channels_to_scan << 4 ) ),
        ( uint8_t ) ( ( uint8_t ) ( params->channel_cfg[3].datarate << 6 ) | ( params->channel_cfg[2].datarate << 4 ) |
                      ( params->channel_cfg[1].datarate << 2 ) | ( params->channel_cfg[0].datarate << 0 ) ),
        ( uint8_t ) params->addr_filtering,
        ( uint8_t ) params->fcs_mode,
        ( uint8_t ) ( params->channel_cfg[0].rf_freq_in_hz >> 24 ),
        ( uint8_t ) ( params->channel_cfg[0].rf_freq_in_hz >> 16 ),
        ( uint8_t ) ( params->channel_cfg[0].rf_freq_in_hz >> 8 ),
        ( uint8_t ) ( params->channel_cfg[0].rf_freq_in_hz >> 0 ),
        params->channel_cfg[0].timeout_in_30us_step,
        ( uint8_t ) ( params->channel_cfg[1].rf_freq_in_hz >> 24 ),
        ( uint8_t ) ( params->channel_cfg[1].rf_freq_in_hz >> 16 ),
        ( uint8_t ) ( params->channel_cfg[1].rf_freq_in_hz >> 8 ),
        ( uint8_t ) ( params->channel_cfg[1].rf_freq_in_hz >> 0 ),
        params->channel_cfg[1].timeout_in_30us_step,
        ( uint8_t ) ( params->channel_cfg[2].rf_freq_in_hz >> 24 ),
        ( uint8_t ) ( params->channel_cfg[2].rf_freq_in_hz >> 16 ),
        ( uint8_t ) ( params->channel_cfg[2].rf_freq_in_hz >> 8 ),
        ( uint8_t ) ( params->channel_cfg[2].rf_freq_in_hz >> 0 ),
        params->channel_cfg[2].timeout_in_30us_step,
        ( uint8_t ) ( params->channel_cfg[3].rf_freq_in_hz >> 24 ),
        ( uint8_t ) ( params->channel_cfg[3].rf_freq_in_hz >> 16 ),
        ( uint8_t ) ( params->channel_cfg[3].rf_freq_in_hz >> 8 ),
        ( uint8_t ) ( params->channel_cfg[3].rf_freq_in_hz >> 0 ),
        params->channel_cfg[3].timeout_in_30us_step,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write(
        context, cbuffer,
        ( uint16_t ) ( LR20XX_RADIO_Z_WAVE_SET_SCAN_PARAMS_CMD_LENGTH +
                       ( params->number_of_channels_to_scan * LR20XX_RADIO_Z_WAVE_SCAN_PER_CHANNEL_PARAMS_LENGTH ) ),
        0, 0 );
}

lr20xx_status_t lr20xx_radio_z_wave_set_scan( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_Z_WAVE_SET_SCAN_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_SCAN_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_Z_WAVE_SET_SCAN_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_Z_WAVE_SET_SCAN_CMD_LENGTH, 0, 0 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
