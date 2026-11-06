/*!
 * @file      lr20xx_radio_fsk.c
 *
 * @brief     Radio FSK driver implementation for LR20XX
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

#include "lr20xx_radio_fsk.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_FSK_SET_MODULATION_PARAMS_CMD_LENGTH ( 2 + 9 )
#define LR20XX_RADIO_FSK_SET_PACKET_PARAMS_CMD_LENGTH ( 2 + 7 )
#define LR20XX_RADIO_FSK_SET_WHITENING_PARAMS_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_FSK_SET_CRC_PARAMS_CMD_LENGTH ( 2 + 8 )
#define LR20XX_RADIO_FSK_SET_SYNCWORD_CMD_LENGTH ( 2 + 9 )
#define LR20XX_RADIO_FSK_SET_ADDRESSES_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_FSK_GET_RX_STATISTICS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FSK_GET_RX_STATISTICS_RBUFFER_LENGTH ( 14 )
#define LR20XX_RADIO_FSK_GET_PACKET_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_FSK_GET_PACKET_STATUS_RBUFFER_LENGTH ( 6 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Associative structure that link bandwidth in Hz to @ref lr20xx_radio_fsk_common_bw_t
 */
typedef struct
{
    lr20xx_radio_fsk_common_bw_t param;     //!< DSB Rx bandwidth parameter
    uint32_t                     bw_in_hz;  //!< DSB Rx bandwidth in Hz
} lr20xx_radio_gfsk_bw_values_t;

static const lr20xx_radio_gfsk_bw_values_t lr20xx_radio_gfsk_bw_values[] = {
    { LR20XX_RADIO_FSK_COMMON_RX_BW_3_500_HZ, 3500 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_4_200_HZ, 4200 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_4_300_HZ, 4300 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_4_500_HZ, 4500 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_4_800_HZ, 4800 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_5_200_HZ, 5200 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_5_600_HZ, 5600 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_5_800_HZ, 5800 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_6_000_HZ, 6000 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_6_900_HZ, 6900 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_7_400_HZ, 7400 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_8_000_HZ, 8000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_8_300_HZ, 8300 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_8_700_HZ, 8700 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_8_900_HZ, 8900 },        { LR20XX_RADIO_FSK_COMMON_RX_BW_9_600_HZ, 9600 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_10_000_HZ, 10000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_11_000_HZ, 11000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_12_000_HZ, 12000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_13_000_HZ, 13000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_14_000_HZ, 14000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_16_000_HZ, 16000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_17_000_HZ, 17000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_19_000_HZ, 19000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_20_000_HZ, 20000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_22_000_HZ, 22000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_23_000_HZ, 23000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_24_000_HZ, 24000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_27_000_HZ, 27000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_29_000_HZ, 29000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_32_000_HZ, 32000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_33_000_HZ, 33000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_34_000_HZ, 34000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_35_000_HZ, 35000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_38_000_HZ, 38000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_41_000_HZ, 41000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_44_000_HZ, 44000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_46_000_HZ, 46000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_48_000_HZ, 48000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_55_000_HZ, 55000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_59_000_HZ, 59000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_64_000_HZ, 64000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_66_000_HZ, 66000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_69_000_HZ, 69000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_71_000_HZ, 71000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_76_000_HZ, 76000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_83_000_HZ, 83000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_89_000_HZ, 89000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_92_000_HZ, 92000 },      { LR20XX_RADIO_FSK_COMMON_RX_BW_96_000_HZ, 96000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_111_000_HZ, 111000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_128_000_HZ, 128000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_133_000_HZ, 133000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_138_000_HZ, 138000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_142_000_HZ, 142000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_153_000_HZ, 153000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_166_000_HZ, 166000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_178_000_HZ, 178000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_185_000_HZ, 185000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_192_000_HZ, 192000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_222_000_HZ, 222000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_238_000_HZ, 238000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_256_000_HZ, 256000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_266_000_HZ, 266000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_277_000_HZ, 277000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_285_000_HZ, 285000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_307_000_HZ, 307000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_333_000_HZ, 333000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_357_000_HZ, 357000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_370_000_HZ, 370000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_384_000_HZ, 384000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_444_000_HZ, 444000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_476_000_HZ, 476000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_512_000_HZ, 512000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_533_000_HZ, 533000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_555_000_HZ, 555000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_571_000_HZ, 571000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_615_000_HZ, 615000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_666_000_HZ, 666000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_714_000_HZ, 714000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_740_000_HZ, 740000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_769_000_HZ, 769000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_888_000_HZ, 888000 },    { LR20XX_RADIO_FSK_COMMON_RX_BW_1_111_000_HZ, 1111000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_1_333_000_HZ, 1333000 }, { LR20XX_RADIO_FSK_COMMON_RX_BW_2_222_000_HZ, 2222000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_2_666_000_HZ, 2666000 }, { LR20XX_RADIO_FSK_COMMON_RX_BW_2_857_000_HZ, 2857000 },
    { LR20XX_RADIO_FSK_COMMON_RX_BW_3_076_000_HZ, 3076000 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for radio related operations
 */
enum
{
    LR20XX_RADIO_FSK_SET_MODULATION_PARAMS_OC = 0x0240,
    LR20XX_RADIO_FSK_SET_PACKET_PARAMS_OC     = 0x0241,
    LR20XX_RADIO_FSK_SET_WHITENING_PARAMS_OC  = 0x0242,
    LR20XX_RADIO_FSK_SET_CRC_PARAMS_OC        = 0x0243,
    LR20XX_RADIO_FSK_SET_SYNCWORD_OC          = 0x0244,
    LR20XX_RADIO_FSK_SET_ADDRESSES_OC         = 0x0245,
    LR20XX_RADIO_FSK_GET_RX_STATISTICS_OC     = 0x0246,
    LR20XX_RADIO_FSK_GET_PACKET_STATUS_OC     = 0x0247,
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
 * @brief Read two bytes from buffer and return the corresponding uint16_t value MSB first
 *
 * @param buffer The buffer to read the two bytes. It is up to the caller to ensure the buffer is at least two bytes
 * long
 *
 * @return uint16_t
 */
static uint16_t read_2_bytes_msbf( const uint8_t* buffer );

/*!
 * @brief Get the CRC length in byte from the corresponding GFSK radio parameter
 *
 * @param [in] crc_type GFSK CRC parameter
 *
 * @returns CRC length in byte
 */
static inline uint32_t lr20xx_radio_fsk_get_crc_len_in_bytes( lr20xx_radio_fsk_crc_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_fsk_set_modulation_params( const void*                          context,
                                                        const lr20xx_radio_fsk_mod_params_t* mod_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_SET_MODULATION_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_MODULATION_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_MODULATION_PARAMS_OC >> 0 ),
        ( uint8_t ) ( mod_params->br >> 24 ),
        ( uint8_t ) ( mod_params->br >> 16 ),
        ( uint8_t ) ( mod_params->br >> 8 ),
        ( uint8_t ) ( mod_params->br >> 0 ),
        ( uint8_t ) mod_params->pulse_shape,
        ( uint8_t ) mod_params->bw,
        ( uint8_t ) ( mod_params->fdev_in_hz >> 16 ),
        ( uint8_t ) ( mod_params->fdev_in_hz >> 8 ),
        ( uint8_t ) ( mod_params->fdev_in_hz >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FSK_SET_MODULATION_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_fsk_set_packet_params( const void*                          context,
                                                    const lr20xx_radio_fsk_pkt_params_t* pkt_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_SET_PACKET_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_PACKET_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_PACKET_PARAMS_OC >> 0 ),
        ( uint8_t ) ( pkt_params->pbl_length_in_bit >> 8 ),
        ( uint8_t ) ( pkt_params->pbl_length_in_bit >> 0 ),
        ( uint8_t ) pkt_params->preamble_detector,
        ( uint8_t ) ( ( pkt_params->long_preamble_enabled ? ( 0x01 << 5 ) : 0x00 ) +
                      ( ( uint8_t ) ( pkt_params->payload_length_unit << 4 ) ) +
                      ( ( uint8_t ) ( pkt_params->address_filtering << 2 ) ) +
                      ( ( uint8_t ) pkt_params->header_mode ) ),
        ( uint8_t ) ( pkt_params->payload_length >> 8 ),
        ( uint8_t ) ( pkt_params->payload_length >> 0 ),
        ( uint8_t ) ( ( uint8_t ) ( pkt_params->crc << 4 ) + ( uint8_t ) pkt_params->whitening ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FSK_SET_PACKET_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_fsk_set_whitening_params( const void*                                context,
                                                       lr20xx_radio_fsk_whitening_compatibility_t whitening_type,
                                                       uint16_t                                   whitening_seed )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_SET_WHITENING_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_WHITENING_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_WHITENING_PARAMS_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) ( whitening_type << 4 ) ) + ( ( uint8_t ) ( ( whitening_seed & 0x0FFF ) >> 8 ) ) ),
        ( uint8_t ) ( whitening_seed ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FSK_SET_WHITENING_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_fsk_set_crc_params( const void* context, uint32_t crc_polynom, uint32_t crc_seed )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_SET_CRC_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_CRC_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_CRC_PARAMS_OC >> 0 ),
        ( uint8_t ) ( crc_polynom >> 24 ),
        ( uint8_t ) ( crc_polynom >> 16 ),
        ( uint8_t ) ( crc_polynom >> 8 ),
        ( uint8_t ) ( crc_polynom >> 0 ),
        ( uint8_t ) ( crc_seed >> 24 ),
        ( uint8_t ) ( crc_seed >> 16 ),
        ( uint8_t ) ( crc_seed >> 8 ),
        ( uint8_t ) ( crc_seed >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FSK_SET_CRC_PARAMS_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fsk_set_syncword( const void*   context,
                                               const uint8_t syncword[LR20XX_RADIO_FSK_SYNCWORD_LENGTH],
                                               uint8_t nb_bits, lr20xx_radio_fsk_syncword_bit_order_t bit_order )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_SET_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_SYNCWORD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_SYNCWORD_OC >> 0 ),
        syncword[0],
        syncword[1],
        syncword[2],
        syncword[3],
        syncword[4],
        syncword[5],
        syncword[6],
        syncword[7],
        ( uint8_t ) ( ( ( ( uint8_t ) bit_order ) << 7 ) + nb_bits ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FSK_SET_SYNCWORD_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fsk_set_addresses( const void* context, uint8_t node_address, uint8_t broadcast_address )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_SET_ADDRESSES_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_ADDRESSES_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_SET_ADDRESSES_OC >> 0 ),
        node_address,
        broadcast_address,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_FSK_SET_ADDRESSES_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_fsk_get_rx_statistics( const void* context, lr20xx_radio_fsk_rx_statistics_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_GET_RX_STATISTICS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_GET_RX_STATISTICS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_GET_RX_STATISTICS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_FSK_GET_RX_STATISTICS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_FSK_GET_RX_STATISTICS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_FSK_GET_RX_STATISTICS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->n_received_packets    = read_2_bytes_msbf( rbuffer + 0 );
        statistics->n_crc_errors          = read_2_bytes_msbf( rbuffer + 2 );
        statistics->n_length_errors       = read_2_bytes_msbf( rbuffer + 4 );
        statistics->n_preamble_detections = read_2_bytes_msbf( rbuffer + 6 );
        statistics->n_syncword_ok         = read_2_bytes_msbf( rbuffer + 8 );
        statistics->n_syncword_fail       = read_2_bytes_msbf( rbuffer + 10 );
        statistics->n_timeout             = read_2_bytes_msbf( rbuffer + 12 );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_fsk_get_packet_status( const void* context, lr20xx_radio_fsk_packet_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_FSK_GET_PACKET_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_FSK_GET_PACKET_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_FSK_GET_PACKET_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_FSK_GET_PACKET_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_FSK_GET_PACKET_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_FSK_GET_PACKET_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->packet_length_bytes =
            ( uint16_t ) ( ( ( ( uint16_t ) rbuffer[0] ) << 8 ) + ( ( uint16_t ) rbuffer[1] ) );
        pkt_status->rssi_avg_in_dbm          = ( int16_t ) ( -( ( int16_t ) rbuffer[2] ) );
        pkt_status->rssi_sync_in_dbm         = ( int16_t ) ( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->is_addr_match_broadcast  = ( ( ( rbuffer[4] >> 5 ) & 0x01 ) != 0x00 );
        pkt_status->is_addr_match_node       = ( ( ( rbuffer[4] >> 4 ) & 0x01 ) != 0x00 );
        pkt_status->rssi_avg_half_dbm_count  = ( rbuffer[4] >> 2 ) & 0x01;
        pkt_status->rssi_sync_half_dbm_count = ( rbuffer[4] >> 0 ) & 0x01;
        pkt_status->link_quality_indicator   = rbuffer[5];
    }

    return status;
}

lr20xx_status_t lr20xx_radio_fsk_get_rx_bandwidth( uint32_t bw_in_hz, lr20xx_radio_fsk_common_bw_t* bw_parameter )
{
    for( uint8_t i = 0; i < ( sizeof( lr20xx_radio_gfsk_bw_values ) / sizeof( lr20xx_radio_gfsk_bw_values_t ) ); i++ )
    {
        if( bw_in_hz <= lr20xx_radio_gfsk_bw_values[i].bw_in_hz )
        {
            *bw_parameter = lr20xx_radio_gfsk_bw_values[i].param;
            return LR20XX_STATUS_OK;
        }
    }

    return LR20XX_STATUS_ERROR;
}

uint32_t lr20xx_radio_fsk_get_time_on_air_numerator( const lr20xx_radio_fsk_pkt_params_t* pkt_p,
                                                     uint8_t                              syncword_len_in_bit )
{
    uint8_t header_len_in_bits;

    switch( pkt_p->header_mode )
    {
    case LR20XX_RADIO_FSK_HEADER_IMPLICIT:
    {
        header_len_in_bits = 0u;
        break;
    }
    case LR20XX_RADIO_FSK_HEADER_8BITS:
    {
        header_len_in_bits = 8u;
        break;
    }
    case LR20XX_RADIO_FSK_HEADER_SX128X_COMPATIBLE:
    {
        header_len_in_bits = 9u;
        break;
    }
    default:
    {
        return 0;
    }
    }

    return ( uint16_t ) ( ( pkt_p->pbl_length_in_bit ) + ( header_len_in_bits ) + ( syncword_len_in_bit ) ) +
           ( ( pkt_p->payload_length +
               ( pkt_p->address_filtering == LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED ? 0u : 1u ) +
               lr20xx_radio_fsk_get_crc_len_in_bytes( pkt_p->crc ) )
             << 3u );
}

uint32_t lr20xx_radio_fsk_get_time_on_air_in_ms( const lr20xx_radio_fsk_pkt_params_t* pkt_p,
                                                 const lr20xx_radio_fsk_mod_params_t* mod_p,
                                                 uint8_t                              syncword_len_in_bit )
{
    uint32_t numerator   = 1000U * lr20xx_radio_fsk_get_time_on_air_numerator( pkt_p, syncword_len_in_bit );
    uint32_t denominator = mod_p->br;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint16_t read_2_bytes_msbf( const uint8_t* buffer )
{
    return ( uint16_t ) ( ( ( ( uint16_t ) buffer[0] ) << 8 ) + ( ( ( uint16_t ) buffer[1] ) << 0 ) );
}

static inline uint32_t lr20xx_radio_fsk_get_crc_len_in_bytes( lr20xx_radio_fsk_crc_t crc_type )
{
    switch( crc_type )
    {
    case LR20XX_RADIO_FSK_CRC_OFF:
        return 0;
    case LR20XX_RADIO_FSK_CRC_1_BYTE:
        return 1;
    case LR20XX_RADIO_FSK_CRC_2_BYTES:
        return 2;
    case LR20XX_RADIO_FSK_CRC_3_BYTES:
        return 3;
    case LR20XX_RADIO_FSK_CRC_4_BYTES:
        return 4;
    case LR20XX_RADIO_FSK_CRC_1_BYTE_INVERTED:
        return 1;
    case LR20XX_RADIO_FSK_CRC_2_BYTES_INVERTED:
        return 2;
    case LR20XX_RADIO_FSK_CRC_3_BYTES_INVERTED:
        return 3;
    case LR20XX_RADIO_FSK_CRC_4_BYTES_INVERTED:
        return 4;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
