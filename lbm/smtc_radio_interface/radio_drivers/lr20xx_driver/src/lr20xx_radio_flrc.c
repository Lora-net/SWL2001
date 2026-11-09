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
#include "lr20xx_workarounds.h"
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

#define LR20XX_RADIO_FLRC_GET_RX_STATS_RBUFFER_LENGTH ( 8 )
#define LR20XX_RADIO_FLRC_GET_PKT_STATUS_RBUFFER_LENGTH ( 8 )

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

/**
 * @brief Compute the numerator part of the FLRC Time-On-Air equation
 *
 * @param pkt_params Packet parameter
 * @param mod_params Modulation parameter
 * @return Numerator part of the FLRC ToA equation, which is the total number of bits
 */
static uint32_t lr20xx_get_flrc_time_on_air_numerator( const lr20xx_radio_flrc_pkt_params_t* pkt_params,
                                                       const lr20xx_radio_flrc_mod_params_t* mod_params );

/**
 * @brief Get the numerator of coding ratio, scalled on 12
 *
 * So for instance, for CR 1/2 (=6/12) this returns 6, for CR 2/3 (=8/12) this return 8.
 *
 * @param cr The coding rate parameter
 * @return Numerator of coding rate scalled on 12
 */
static uint32_t lr20xx_get_flrc_cr_scalled_numerator( lr20xx_radio_flrc_cr_t cr );

/**
 * @brief Get bitrate of corresponding configuration in kb/s
 *
 * @param br_bw The bitrate configuration
 * @return Bitrate in kb/s
 */
static uint32_t lr20xx_get_flrc_br_in_kbps( lr20xx_radio_flrc_br_bw_t br_bw );

/**
 * @brief Get length of FLRC packet header in bit
 *
 * @param header_type The corresponding packet header configuration
 * @return Number of bits in the header for given configuration
 */
static uint32_t lr20xx_radio_flrc_get_header_len_in_bits( lr20xx_radio_flrc_pkt_len_modes_t header_type );

/**
 * @brief Get length of the FLRC packet tail part corresponding to configured coding rate
 *
 * @param cr The coding rate
 * @return Length of the tail section of FLRC packet
 */
static uint32_t lr20xx_radio_flrc_get_tail_len_in_bits( lr20xx_radio_flrc_cr_t cr );

/**
 * @brief Get the length of FLRC packet CRC part in bytes
 *
 * @param crc_type The CRC packet configuration
 * @return The number of bytes in the configured CRC
 */
static uint32_t lr20xx_radio_flrc_get_crc_len_in_bytes( lr20xx_radio_flrc_crc_types_t crc_type );

/**
 * @brief Get the length of FLRC packet preamble part in bits
 *
 * This returns only the number of bits in the configurable preamble part (a.k.a. AGC preamble).
 * Therefore 21 bits must be added to the output of this function to have the length of complete preamble.
 *
 * @param preamble_length The preamble length configuration of the packet
 * @return Number of bits in the AGC preamble part of the FLRC packet
 */
static uint32_t lr20xx_radio_flrc_get_preamble_length_in_bits( lr20xx_radio_flrc_preamble_len_t preamble_length );

/**
 * @brief Get the length of FLRC packet syncword part in bits
 *
 * @param syncword_length The syncword configuration
 * @return Number of bits in the syncword FLRC packet part
 */
static uint32_t lr20xx_radio_flrc_get_syncword_length_in_bits( lr20xx_radio_flrc_sync_word_len_t syncword_length );

static lr20xx_status_t lr20xx_radio_flrc_set_syncword_base( const void* context, uint8_t syncword_index,
                                                            const uint8_t* syncword, uint8_t syncword_length );

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
        statistics->crc_ok           = ( uint16_t ) ( ( ( uint16_t ) rbuffer[6] << 8 ) + rbuffer[7] );
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
        /* Extract 24-bit signed value and sign-extend to 32 bits */
        const int32_t freq_offset_24 = ( ( rbuffer[5] << 16 ) | ( rbuffer[6] << 8 ) | rbuffer[7] ) & 0x00FFFFFF;
        /* Sign-extend: shift left 8 bits then arithmetic shift right 8 bits */
        pkt_status->freq_offset_hz = ( freq_offset_24 << 8 ) >> 8;
    }

    return status;
}

lr20xx_status_t lr20xx_radio_flrc_set_short_syncword(
    const void* context, uint8_t syncword_index, const uint8_t short_syncword[LR20XX_RADIO_FLRC_SHORT_SYNCWORD_LENGTH] )
{
    return lr20xx_radio_flrc_set_syncword_base( context, syncword_index, short_syncword,
                                                LR20XX_RADIO_FLRC_SHORT_SYNCWORD_LENGTH );
}

lr20xx_status_t lr20xx_radio_flrc_set_syncword( const void* context, uint8_t syncword_index,
                                                const uint8_t syncword[LR20XX_RADIO_FLRC_SYNCWORD_LENGTH] )
{
    return lr20xx_radio_flrc_set_syncword_base( context, syncword_index, syncword, LR20XX_RADIO_FLRC_SYNCWORD_LENGTH );
}

uint32_t lr20xx_get_flrc_time_on_air_in_us( const lr20xx_radio_flrc_pkt_params_t* pkt_params,
                                            const lr20xx_radio_flrc_mod_params_t* mod_params )
{
    uint32_t numerator   = 1000U * lr20xx_get_flrc_time_on_air_numerator( pkt_params, mod_params );
    uint32_t denominator = lr20xx_get_flrc_br_in_kbps( mod_params->br_bw );

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t lr20xx_get_flrc_time_on_air_numerator( const lr20xx_radio_flrc_pkt_params_t* pkt_params,
                                                const lr20xx_radio_flrc_mod_params_t* mod_params )
{
    const uint32_t ceil_den = lr20xx_get_flrc_cr_scalled_numerator( mod_params->cr );

    if( ceil_den != 0 )
    {
        const uint32_t n_coded = lr20xx_radio_flrc_get_header_len_in_bits( pkt_params->header_type ) +
                                 lr20xx_radio_flrc_get_tail_len_in_bits( mod_params->cr ) +
                                 ( lr20xx_radio_flrc_get_crc_len_in_bytes( pkt_params->crc_type ) * 8u ) +
                                 pkt_params->pld_len_in_bytes * 8u;
        const uint32_t n_uncoded = lr20xx_radio_flrc_get_preamble_length_in_bits( pkt_params->preamble_len ) + 21u +
                                   lr20xx_radio_flrc_get_syncword_length_in_bits( pkt_params->sync_word_len );

        // The '*12' here comes from the coding rate ration that is expressed as /12 (so cr 1/2 is 6/12, cr 2/3 is
        // 8/12,...) See lr20xx_get_flrc_cr_scalled_numerator The n_coded_after_decode is the result of equation Ceil(
        // n_coded / CR )
        const uint32_t n_coded_after_decode = ( ( ( 12u * n_coded ) + ceil_den - 1u ) / ceil_den );
        return n_coded_after_decode + n_uncoded;
    }
    else
    {
        return 0;
    }
}

uint32_t lr20xx_get_flrc_cr_scalled_numerator( lr20xx_radio_flrc_cr_t cr )
{
    uint32_t ceil_den = 0;

    if( cr == LR20XX_RADIO_FLRC_CR_1_2 )
    {
        ceil_den = 6;
    }
    else if( cr == LR20XX_RADIO_FLRC_CR_3_4 )
    {
        ceil_den = 9;
    }
    else if( cr == LR20XX_RADIO_FLRC_CR_NONE )
    {
        ceil_den = 12;
    }
    else if( cr == LR20XX_RADIO_FLRC_CR_2_3 )
    {
        ceil_den = 8;
    }

    return ceil_den;
}

uint32_t lr20xx_get_flrc_br_in_kbps( lr20xx_radio_flrc_br_bw_t br_bw )
{
    switch( br_bw )
    {
    case LR20XX_RADIO_FLRC_BR_2_600_BW_2_666:
        return 2600UL;
    case LR20XX_RADIO_FLRC_BR_2_080_BW_2_222:
        return 2080UL;
    case LR20XX_RADIO_FLRC_BR_1_300_BW_1_333:
        return 1300UL;
    case LR20XX_RADIO_FLRC_BR_1_040_BW_1_333:
        return 1040UL;
    case LR20XX_RADIO_FLRC_BR_0_650_BW_0_740:
        return 650UL;
    case LR20XX_RADIO_FLRC_BR_0_520_BW_0_571:
        return 520UL;
    case LR20XX_RADIO_FLRC_BR_0_325_BW_0_357:
        return 325UL;
    case LR20XX_RADIO_FLRC_BR_0_260_BW_0_307:
        return 260UL;
    }
    return 0;
}

uint32_t lr20xx_radio_flrc_get_header_len_in_bits( lr20xx_radio_flrc_pkt_len_modes_t header_type )
{
    if( header_type == LR20XX_RADIO_FLRC_PKT_VAR_LEN )
    {
        return 16;
    }
    else
    {
        return 0;
    }
}

uint32_t lr20xx_radio_flrc_get_tail_len_in_bits( lr20xx_radio_flrc_cr_t cr )
{
    if( ( cr == LR20XX_RADIO_FLRC_CR_1_2 ) || ( cr == LR20XX_RADIO_FLRC_CR_3_4 ) || ( cr == LR20XX_RADIO_FLRC_CR_2_3 ) )
    {
        return 6;
    }
    else
    {
        return 0;
    }
}

uint32_t lr20xx_radio_flrc_get_crc_len_in_bytes( lr20xx_radio_flrc_crc_types_t crc_type )
{
    switch( crc_type )
    {
    case LR20XX_RADIO_FLRC_CRC_OFF:
    {
        return 0;
    }
    case LR20XX_RADIO_FLRC_CRC_2_BYTES:
    {
        return 2;
    }
    case LR20XX_RADIO_FLRC_CRC_3_BYTES:
    {
        return 3;
    }
    case LR20XX_RADIO_FLRC_CRC_4_BYTES:
    {
        return 4;
    }
    default:
    {
        return 0;
    }
    }
}

uint32_t lr20xx_radio_flrc_get_preamble_length_in_bits( lr20xx_radio_flrc_preamble_len_t preamble_length )
{
    switch( preamble_length )
    {
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_04_BITS:
    {
        return 4;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_08_BITS:
    {
        return 8;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_12_BITS:
    {
        return 12;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_16_BITS:
    {
        return 16;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_20_BITS:
    {
        return 20;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_24_BITS:
    {
        return 24;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_28_BITS:
    {
        return 28;
    }
    case LR20XX_RADIO_FLRC_PREAMBLE_LEN_32_BITS:
    {
        return 32;
    }
    default:
    {
        return 0;
    }
    }
}

uint32_t lr20xx_radio_flrc_get_syncword_length_in_bits( lr20xx_radio_flrc_sync_word_len_t syncword_length )
{
    switch( syncword_length )
    {
    case LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_OFF:
    {
        return 0;
    }
    case LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_2_BYTES:
    {
        return 16;
    }
    case LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_4_BYTES:
    {
        return 32;
    }
    default:
    {
        return 0;
    }
    }
}

lr20xx_status_t lr20xx_radio_flrc_set_syncword_base( const void* context, uint8_t syncword_index,
                                                     const uint8_t* syncword, uint8_t syncword_length )
{
    const uint8_t cbuffer[LR20XX_RADIO_FLRC_SET_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_SYNCWORD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FLRC_SET_SYNCWORD_OC >> 0 ),
        syncword_index,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FLRC_SET_SYNCWORD_CMD_LENGTH, syncword,
                                                 syncword_length );
}

/* --- EOF ------------------------------------------------------------------ */
