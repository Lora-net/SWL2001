/*!
 * @file      lr20xx_radio_ook.c
 *
 * @brief     On-Off Keying radio driver implementation for LR20XX
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

#include "lr20xx_radio_ook.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_OOK_SET_MODULATION_PARAMS_CMD_LENGTH ( 2 + 7 )
#define LR20XX_RADIO_OOK_SET_PKT_PARAMS_CMD_LENGTH ( 2 + 6 )
#define LR20XX_RADIO_OOK_SET_CRC_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR20XX_RADIO_OOK_SET_SYNCWORD_CMD_LENGTH ( 2 + 5 )
#define LR20XX_RADIO_OOK_SET_ADDRESSES_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_OOK_GET_RX_STATISTICS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_OOK_GET_PACKET_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_OOK_SET_RX_DETECTOR_CMD_LENGTH ( 2 + 5 )
#define LR20XX_RADIO_OOK_SET_WHITENING_PARAMS_CMD_LENGTH ( 2 + 4 )

#define LR20XX_RADIO_OOK_GET_RX_STATISTICS_RBUFFER_LENGTH ( 6 )
#define LR20XX_RADIO_OOK_GET_PACKET_STATUS_RBUFFER_LENGTH ( 6 )

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
    LR20XX_RADIO_OOK_SET_MODULATION_PARAMS_OC = 0x0281,
    LR20XX_RADIO_OOK_SET_PKT_PARAMS_OC        = 0x0282,
    LR20XX_RADIO_OOK_SET_CRC_PARAMS_OC        = 0x0283,
    LR20XX_RADIO_OOK_SET_SYNCWORD_OC          = 0x0284,
    LR20XX_RADIO_OOK_SET_ADDRESSES_OC         = 0x0285,
    LR20XX_RADIO_OOK_GET_RX_STATISTICS_OC     = 0x0286,
    LR20XX_RADIO_OOK_GET_PACKET_STATUS_OC     = 0x0287,
    LR20XX_RADIO_OOK_SET_RX_DETECTOR_OC       = 0x0288,
    LR20XX_RADIO_OOK_SET_WHITENING_PARAMS_OC  = 0x0289,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint8_t pulse_shape_to_byte( const lr20xx_radio_ook_pulse_shape_t* pulse_shape );

/**
 * @brief Compute the numerator of time on air expression
 *
 * This is basically the number of bits in the complete payload, encoding taken into account.
 *
 * @param pkt_p Packet parameter structure
 * @param syncword_len_in_bit Length of syncword in bit
 *
 * @return Numerator value of the time on air expression or 0 in case of failure
 */
static uint32_t lr20xx_radio_ook_get_time_on_air_numerator( const lr20xx_radio_ook_pkt_params_t* pkt_p,
                                                            uint8_t                              syncword_len_in_bit );

/**
 * @brief Get the length in byte of CRC field
 *
 * @param crc_type The CRC configuration
 *
 * @return The length of CRC field in byte
 */
static uint8_t lr20xx_radio_ook_get_crc_len_in_bytes( lr20xx_radio_ook_crc_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_ook_set_modulation_params( const void*                          context,
                                                        const lr20xx_radio_ook_mod_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_MODULATION_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_MODULATION_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_MODULATION_PARAMS_OC >> 0 ),
        ( uint8_t ) ( params->br >> 24 ),
        ( uint8_t ) ( params->br >> 16 ),
        ( uint8_t ) ( params->br >> 8 ),
        ( uint8_t ) ( params->br >> 0 ),
        pulse_shape_to_byte( &params->pulse_shape ),
        ( uint8_t ) params->bw,
        ( uint8_t ) params->mag_depth,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_MODULATION_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_ook_set_packet_params( const void* context, const lr20xx_radio_ook_pkt_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_PKT_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_PKT_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_PKT_PARAMS_OC >> 0 ),
        ( uint8_t ) ( params->pbl_length_in_bit >> 8 ),
        ( uint8_t ) ( params->pbl_length_in_bit >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) ( params->address_filtering << 2 ) ) + ( ( uint8_t ) params->header_mode ) ),
        ( uint8_t ) ( params->payload_length >> 8 ),
        ( uint8_t ) ( params->payload_length >> 0 ),
        ( uint8_t ) ( ( uint8_t ) ( params->crc << 4 ) + ( uint8_t ) params->encoding ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_PKT_PARAMS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_ook_set_crc_params( const void* context, uint32_t crc_polynomial, uint32_t crc_seed )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_CRC_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_CRC_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_CRC_PARAMS_OC >> 0 ),
        ( uint8_t ) ( crc_polynomial >> 24 ),
        ( uint8_t ) ( crc_polynomial >> 16 ),
        ( uint8_t ) ( crc_polynomial >> 8 ),
        ( uint8_t ) ( crc_polynomial >> 0 ),
        ( uint8_t ) ( crc_seed >> 24 ),
        ( uint8_t ) ( crc_seed >> 16 ),
        ( uint8_t ) ( crc_seed >> 8 ),
        ( uint8_t ) ( crc_seed >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_CRC_PARAMS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_ook_set_syncword( const void*   context,
                                               const uint8_t syncword[LR20XX_RADIO_OOK_SYNCWORD_LENGTH],
                                               uint8_t nb_bits, lr20xx_radio_ook_syncword_bit_order_t bit_order )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_SYNCWORD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_SYNCWORD_OC >> 0 ),
        syncword[0],
        syncword[1],
        syncword[2],
        syncword[3],
        ( uint8_t ) ( ( ( ( uint8_t ) bit_order ) << 7 ) + nb_bits ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_SYNCWORD_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_ook_set_addresses( const void* context, uint8_t node_address, uint8_t broadcast_address )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_ADDRESSES_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_ADDRESSES_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_ADDRESSES_OC >> 0 ),
        node_address,
        broadcast_address,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_ADDRESSES_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_ook_get_rx_statistics( const void* context, lr20xx_radio_ook_rx_statistics_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_GET_RX_STATISTICS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_GET_RX_STATISTICS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_GET_RX_STATISTICS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_OOK_GET_RX_STATISTICS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_OOK_GET_RX_STATISTICS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_OOK_GET_RX_STATISTICS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->n_received_packets =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[0] ) << 8 ) + ( ( ( uint16_t ) rbuffer[1] ) << 0 ) );
        statistics->n_crc_errors =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[2] ) << 8 ) + ( ( ( uint16_t ) rbuffer[3] ) << 0 ) );
        statistics->n_length_errors =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[4] ) << 8 ) + ( ( ( uint16_t ) rbuffer[5] ) << 0 ) );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_ook_get_packet_status( const void* context, lr20xx_radio_ook_packet_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_GET_PACKET_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_GET_PACKET_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_GET_PACKET_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_OOK_GET_PACKET_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_OOK_GET_PACKET_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_OOK_GET_PACKET_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->packet_length_bytes =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[0] ) << 8 ) + ( ( uint16_t ) rbuffer[1] ) );
        pkt_status->rssi_avg_in_dbm         = ( int16_t ) ( -( ( int16_t ) rbuffer[2] ) );
        pkt_status->rssi_on_in_dbm          = ( int16_t ) ( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->is_addr_match_broadcast = ( ( ( rbuffer[4] >> 5 ) & 0x01 ) != 0x00 );
        pkt_status->is_addr_match_node      = ( ( ( rbuffer[4] >> 4 ) & 0x01 ) != 0x00 );
        pkt_status->rssi_avg_half_dbm_count = ( rbuffer[4] >> 2 ) & 0x01;
        pkt_status->rssi_on_half_dbm_count  = ( rbuffer[4] >> 0 ) & 0x01;
        pkt_status->link_quality_indicator  = rbuffer[5];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_ook_set_rx_detector( const void*                           context,
                                                  const lr20xx_radio_ook_rx_detector_t* rx_detector )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_RX_DETECTOR_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_RX_DETECTOR_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_RX_DETECTOR_OC >> 0 ),
        ( uint8_t ) ( rx_detector->pattern >> 8 ),
        ( uint8_t ) ( rx_detector->pattern >> 0 ),
        rx_detector->pattern_length_in_bit,
        rx_detector->pattern_repeat_nb,
        ( uint8_t ) ( ( rx_detector->is_syncword_encoded ? 0x00 : 0x20 ) | ( rx_detector->sfd_type << 4 ) |
                      rx_detector->sfd_length_in_bit ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_RX_DETECTOR_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_ook_set_whitening_params( const void*                                context,
                                                       const lr20xx_radio_ook_whitening_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RADIO_OOK_SET_WHITENING_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_WHITENING_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_OOK_SET_WHITENING_PARAMS_OC >> 0 ),
        ( uint8_t ) ( ( params->bit_index << 4 ) | ( params->polynomial >> 8 ) ),
        ( uint8_t ) ( params->polynomial >> 0 ),
        ( uint8_t ) ( params->seed >> 8 ),
        ( uint8_t ) ( params->seed >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_OOK_SET_WHITENING_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

uint32_t lr20xx_radio_ook_get_time_on_air_in_ms( const lr20xx_radio_ook_pkt_params_t* pkt_p,
                                                 const lr20xx_radio_ook_mod_params_t* mod_p,
                                                 uint8_t                              syncword_len_in_bit )
{
    uint32_t numerator   = 1000U * lr20xx_radio_ook_get_time_on_air_numerator( pkt_p, syncword_len_in_bit );
    uint32_t denominator = mod_p->br;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint8_t pulse_shape_to_byte( const lr20xx_radio_ook_pulse_shape_t* pulse_shape )
{
    return ( uint8_t ) ( ( pulse_shape->bt ) + ( pulse_shape->filter << 3 ) );
}

uint32_t lr20xx_radio_ook_get_time_on_air_numerator( const lr20xx_radio_ook_pkt_params_t* pkt_p,
                                                     uint8_t                              syncword_len_in_bit )
{
    uint8_t header_len_in_bits;

    switch( pkt_p->header_mode )
    {
    case LR20XX_RADIO_OOK_HEADER_IMPLICIT:
    {
        header_len_in_bits = 0u;
        break;
    }
    case LR20XX_RADIO_OOK_HEADER_EXPLICIT:
    {
        header_len_in_bits = 8u;
        break;
    }
    default:
    {
        return 0;
    }
    }

    uint8_t encoding_factor;
    switch( pkt_p->encoding )
    {
    case LR20XX_RADIO_OOK_ENCODING_OFF:
    {
        encoding_factor = 1;
        break;
    }

    case LR20XX_RADIO_OOK_ENCODING_MANCHESTER:
    case LR20XX_RADIO_OOK_ENCODING_MANCHESTER_INV:
    {
        encoding_factor = 2;
        break;
    }
    default:
    {
        return 0;
    }
    }

    return pkt_p->pbl_length_in_bit +
           ( ( uint16_t ) ( syncword_len_in_bit + header_len_in_bits ) + pkt_p->payload_length * 8u +
             ( pkt_p->address_filtering == LR20XX_RADIO_OOK_ADDRESS_FILTERING_DISABLED ? 0u : 8u ) +
             lr20xx_radio_ook_get_crc_len_in_bytes( pkt_p->crc ) * 8u ) *
               encoding_factor;
}

uint8_t lr20xx_radio_ook_get_crc_len_in_bytes( lr20xx_radio_ook_crc_t crc_type )
{
    switch( crc_type )
    {
    case LR20XX_RADIO_OOK_CRC_OFF:
        return 0;
    case LR20XX_RADIO_OOK_CRC_1_BYTE:
        return 1;
    case LR20XX_RADIO_OOK_CRC_2_BYTES:
        return 2;
    case LR20XX_RADIO_OOK_CRC_3_BYTES:
        return 3;
    case LR20XX_RADIO_OOK_CRC_4_BYTES:
        return 4;
    case LR20XX_RADIO_OOK_CRC_1_BYTE_INVERTED:
        return 1;
    case LR20XX_RADIO_OOK_CRC_2_BYTES_INVERTED:
        return 2;
    case LR20XX_RADIO_OOK_CRC_3_BYTES_INVERTED:
        return 3;
    case LR20XX_RADIO_OOK_CRC_4_BYTES_INVERTED:
        return 4;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
