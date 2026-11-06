/*!
 * @file      lr20xx_radio_lora.c
 *
 * @brief     Radio LoRa driver implementation for LR20XX
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

#include "lr20xx_radio_lora.h"
#include "lr20xx_hal.h"
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Length in byte of one side detector CAD configuration
 */
#define LR20XX_RADIO_LORA_CAD_SIDE_DETECTOR_CONFIGURATION_LENGTH ( 2u )

#define LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_CONFIGURE_CAD_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_LORA_SET_MODULATION_PARAMS_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_LORA_SET_PACKET_PARAMS_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RADIO_LORA_SET_LORA_SEARCH_SYMBOLS_CMD_LENGTH ( 2 + 2 )
#define LR20XX_RADIO_LORA_SET_SYNCWORD_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTORS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_SYNCWORD_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_LORA_CONFIGURE_CAD_PARAMS_CMD_LENGTH ( 2 + 7 )
#define LR20XX_RADIO_LORA_SET_CAD_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_LORA_GET_RX_STATISTICS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_LORA_GET_PACKET_STATUS_CMD_LENGTH ( 2 )
#define LR20XX_RADIO_LORA_SET_ADDRESS_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RADIO_LORA_SET_FREQ_HOP_CMD_LENGTH ( 2 + 2 )

#define LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTOR_CAD_TEMP_LENGTH \
    ( 3 * LR20XX_RADIO_LORA_CAD_SIDE_DETECTOR_CONFIGURATION_LENGTH )

#define LR20XX_RADIO_LORA_GET_RX_STATISTICS_RBUFFER_LENGTH ( 8 )
#define LR20XX_RADIO_LORA_GET_PACKET_STATUS_RBUFFER_LENGTH ( 6 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for radio related operations
 */
enum
{
    LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_CONFIGURE_CAD_OC = 0x021E,
    LR20XX_RADIO_LORA_SET_MODULATION_PARAMS_OC           = 0x0220,
    LR20XX_RADIO_LORA_SET_PACKET_PARAMS_OC               = 0x0221,
    LR20XX_RADIO_LORA_SET_LORA_SEARCH_SYMBOLS_OC         = 0x0222,
    LR20XX_RADIO_LORA_SET_SYNCWORD_OC                    = 0x0223,
    LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTORS_OC        = 0x0224,
    LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_SYNCWORD_OC      = 0x0225,
    LR20XX_RADIO_LORA_CONFIGURE_CAD_PARAMS_OC            = 0x0227,
    LR20XX_RADIO_LORA_SET_CAD_OC                         = 0x0228,
    LR20XX_RADIO_LORA_GET_RX_STATISTICS_OC               = 0x0229,
    LR20XX_RADIO_LORA_GET_PACKET_STATUS_OC               = 0x022A,
    LR20XX_RADIO_LORA_SET_ADDRESS_OC                     = 0x022B,
    LR20XX_RADIO_LORA_SET_FREQ_HOP_OC                    = 0x022C,
};

typedef enum
{
    SEARCH_SYMBOL_FORMAT_NUMBER   = 0x00,
    SEARCH_SYMBOL_FORMAT_MANTISSA = 0x01,
} search_symbol_format_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Helper function that abstract the call for lr20xx_radio_lora_set_lora_search_symbols_by_number and
 * lr20xx_radio_lora_set_lora_search_symbols_by_mantissa
 *
 * @param[in] context Chip implementation context
 * @param[in] n_symbols A byte representing the number of symbol. Meaning depends on format
 * @param[in] format  The format that defines the meaning of n_symbols
 * @return lr20xx_status_t
 */
static lr20xx_status_t abstract_search_symbols( const void* context, uint8_t n_symbols, search_symbol_format_t format );

/**
 * @brief Read two bytes from buffer and convert it in 16 bits value MSB first
 *
 * @param buffer Pointer to location where to read 2 bytes. It is up to the caller to ensure there are at least two
 * bytes to read
 *
 * @return The MSB first value corresponding to the consecutive bytes read
 */
static uint16_t read_2_bytes_msbf( const uint8_t* buffer );

/**
 * @brief Compute the byte representation of LoRa side detector configuration
 *
 * @param side_detector_cfg The LoRa side detector configuration
 *
 * @return uint8_t The byte representing the LoRa side detector configuration
 */
static uint8_t radio_lora_side_detector_cfg_to_byte( const lr20xx_radio_lora_side_detector_cfg_t* side_detector_cfg );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_lora_configure_side_detector_cad(
    const void* context, const lr20xx_radio_lora_side_detector_cad_configuration_t* side_detector_cad_configurations,
    uint8_t n_side_detector_cad_configurations )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_CONFIGURE_CAD_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_CONFIGURE_CAD_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_CONFIGURE_CAD_OC >> 0 ),
    };

    uint8_t side_detect_temp[LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTOR_CAD_TEMP_LENGTH] = { 0 };
    for( uint8_t index_side_detector = 0; index_side_detector < n_side_detector_cad_configurations;
         index_side_detector++ )
    {
        const unsigned int local_side_detect_temp_index =
            index_side_detector * LR20XX_RADIO_LORA_CAD_SIDE_DETECTOR_CONFIGURATION_LENGTH;
        const lr20xx_radio_lora_side_detector_cad_configuration_t local_cad_side_detector_configuration =
            side_detector_cad_configurations[index_side_detector];
        side_detect_temp[local_side_detect_temp_index]     = local_cad_side_detector_configuration.pnr_delta;
        side_detect_temp[local_side_detect_temp_index + 1] = local_cad_side_detector_configuration.det_peak;
    }

    const uint16_t n_side_detect_temp =
        ( uint16_t )( n_side_detector_cad_configurations * LR20XX_RADIO_LORA_CAD_SIDE_DETECTOR_CONFIGURATION_LENGTH );
    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_CONFIGURE_CAD_CMD_LENGTH,
                                                 side_detect_temp, n_side_detect_temp );
}

lr20xx_status_t lr20xx_radio_lora_set_modulation_params( const void*                           context,
                                                         const lr20xx_radio_lora_mod_params_t* mod_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_MODULATION_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_MODULATION_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_MODULATION_PARAMS_OC >> 0 ),
        ( uint8_t ) ( ( mod_params->sf << 4 ) + mod_params->bw ),
        ( uint8_t ) ( ( mod_params->cr << 4 ) + mod_params->ppm ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_MODULATION_PARAMS_CMD_LENGTH,
                                                 0, 0 );
}
lr20xx_status_t lr20xx_radio_lora_set_packet_params( const void*                           context,
                                                     const lr20xx_radio_lora_pkt_params_t* pkt_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_PACKET_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_PACKET_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_PACKET_PARAMS_OC >> 0 ),
        ( uint8_t ) ( pkt_params->preamble_len_in_symb >> 8 ),
        ( uint8_t ) ( pkt_params->preamble_len_in_symb >> 0 ),
        pkt_params->pld_len_in_bytes,
        ( uint8_t ) ( ( pkt_params->pkt_mode << 2 ) + ( pkt_params->crc << 1 ) + ( pkt_params->iq << 0 ) ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_PACKET_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_lora_configure_timeout_by_number_of_symbols( const void* context, uint8_t n_symbols )
{
    return abstract_search_symbols( context, n_symbols, SEARCH_SYMBOL_FORMAT_NUMBER );
}

lr20xx_status_t lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols( const void* context, uint8_t mantissa,
                                                                                  uint8_t exponent )
{
    const uint8_t n_symbols_mantissa_format = ( uint8_t ) ( ( mantissa << 3 ) + exponent );
    return abstract_search_symbols( context, n_symbols_mantissa_format, SEARCH_SYMBOL_FORMAT_MANTISSA );
}

lr20xx_status_t lr20xx_radio_lora_set_syncword( const void* context, uint8_t syncword )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_SYNCWORD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_SYNCWORD_OC >> 0 ),
        syncword,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_SYNCWORD_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_lora_configure_side_detectors(
    const void* context, const lr20xx_radio_lora_side_detector_cfg_t* side_detector_cfgs, uint8_t n_side_detector_cfgs )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTORS_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTORS_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTORS_OC >> 0 ),
    };

    uint8_t dbuffer[3];

    for( uint8_t i = 0; i < n_side_detector_cfgs; i++ )
    {
        const lr20xx_radio_lora_side_detector_cfg_t local_side_detector = side_detector_cfgs[i];
        dbuffer[i] = radio_lora_side_detector_cfg_to_byte( &local_side_detector );
    }

    return ( lr20xx_status_t ) lr20xx_hal_write(
        context, cbuffer, LR20XX_RADIO_LORA_CONFIGURE_SIDE_DETECTORS_CMD_LENGTH, dbuffer, n_side_detector_cfgs );
}

lr20xx_status_t lr20xx_radio_lora_set_side_detector_syncwords( const void* context, const uint8_t* syncword,
                                                               uint8_t n_syncword )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t )( LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_SYNCWORD_OC >> 8 ),
        ( uint8_t )( LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_SYNCWORD_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write(
        context, cbuffer, LR20XX_RADIO_LORA_SET_SIDE_DETECTOR_SYNCWORD_CMD_LENGTH, syncword, n_syncword );
}

lr20xx_status_t lr20xx_radio_lora_configure_cad_params( const void*                           context,
                                                        const lr20xx_radio_lora_cad_params_t* cad_params )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_CONFIGURE_CAD_PARAMS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_CONFIGURE_CAD_PARAMS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_CONFIGURE_CAD_PARAMS_OC >> 0 ),
        cad_params->cad_symb_nb,
        ( uint8_t ) cad_params->pnr_delta,
        ( uint8_t ) cad_params->cad_exit_mode,
        ( uint8_t ) ( cad_params->cad_timeout_in_pll_step >> 16 ),
        ( uint8_t ) ( cad_params->cad_timeout_in_pll_step >> 8 ),
        ( uint8_t ) ( cad_params->cad_timeout_in_pll_step >> 0 ),
        cad_params->cad_detect_peak,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_CONFIGURE_CAD_PARAMS_CMD_LENGTH, 0,
                                                 0 );
}

lr20xx_status_t lr20xx_radio_lora_set_cad( const void* context )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_CAD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_CAD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_CAD_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_CAD_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_radio_lora_get_rx_statistics( const void*                        context,
                                                     lr20xx_radio_lora_rx_statistics_t* statistics )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_GET_RX_STATISTICS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_GET_RX_STATISTICS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_GET_RX_STATISTICS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_LORA_GET_RX_STATISTICS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_LORA_GET_RX_STATISTICS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_LORA_GET_RX_STATISTICS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        statistics->n_received_packets      = read_2_bytes_msbf( rbuffer + 0 );
        statistics->n_crc_errors            = read_2_bytes_msbf( rbuffer + 2 );
        statistics->n_header_errors         = read_2_bytes_msbf( rbuffer + 4 );
        statistics->n_false_synchronisation = read_2_bytes_msbf( rbuffer + 6 );
    }

    return status;
}

lr20xx_status_t lr20xx_radio_lora_get_packet_status( const void*                        context,
                                                     lr20xx_radio_lora_packet_status_t* pkt_status )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_GET_PACKET_STATUS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_GET_PACKET_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_GET_PACKET_STATUS_OC >> 0 ),
    };

    uint8_t rbuffer[LR20XX_RADIO_LORA_GET_PACKET_STATUS_RBUFFER_LENGTH] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RADIO_LORA_GET_PACKET_STATUS_CMD_LENGTH, rbuffer,
                                             LR20XX_RADIO_LORA_GET_PACKET_STATUS_RBUFFER_LENGTH );

    if( status == LR20XX_STATUS_OK )
    {
        pkt_status->crc                            = ( lr20xx_radio_lora_crc_t ) ( ( rbuffer[0] >> 4 ) & 0x01 );
        pkt_status->cr                             = ( lr20xx_radio_lora_cr_t ) ( rbuffer[0] & 0x0F );
        pkt_status->packet_length_bytes            = rbuffer[1];
        pkt_status->snr_pkt_raw                    = ( int8_t ) rbuffer[2];
        pkt_status->rssi_pkt_in_dbm                = ( int16_t ) ( -( ( int16_t ) rbuffer[3] ) );
        pkt_status->rssi_signal_pkt_in_dbm         = ( int16_t ) ( -( ( int16_t ) rbuffer[4] ) );
        pkt_status->detector                       = ( rbuffer[5] >> 2 ) & 0x0F;
        pkt_status->rssi_pkt_half_dbm_count        = ( rbuffer[5] >> 1 ) & 0x01;
        pkt_status->rssi_signal_pkt_half_dbm_count = ( rbuffer[5] >> 0 ) & 0x01;
    }

    return status;
}

lr20xx_status_t lr20xx_radio_lora_set_address( const void* context, uint8_t address_offset, uint8_t address_length,
                                               const uint8_t* address )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_ADDRESS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_ADDRESS_OC >> 8 ), ( uint8_t ) ( LR20XX_RADIO_LORA_SET_ADDRESS_OC >> 0 ),
        ( uint8_t ) ( ( address_length << 4 ) + address_offset )
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_ADDRESS_CMD_LENGTH, address,
                                                 address_length );
}

lr20xx_status_t lr20xx_radio_lora_set_freq_hop( const void* context, const lr20xx_radio_lora_hopping_cfg_t* cfg )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_FREQ_HOP_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_FREQ_HOP_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_FREQ_HOP_OC >> 0 ),
        ( uint8_t ) ( ( cfg->hop_ctrl << 6 ) + ( uint8_t ) ( cfg->hop_period >> 8 ) ),
        ( uint8_t ) ( cfg->hop_period >> 0 ),
    };

    uint8_t freq_hop_table[160];

    for( uint8_t i = 0; i < cfg->nb_freq_hop; i++ )
    {
        freq_hop_table[4 * i + 0] = ( uint8_t ) ( cfg->freq_hop[i] >> 24 );
        freq_hop_table[4 * i + 1] = ( uint8_t ) ( cfg->freq_hop[i] >> 16 );
        freq_hop_table[4 * i + 2] = ( uint8_t ) ( cfg->freq_hop[i] >> 8 );
        freq_hop_table[4 * i + 3] = ( uint8_t ) ( cfg->freq_hop[i] >> 0 );
    }

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_FREQ_HOP_CMD_LENGTH,
                                                 freq_hop_table, ( uint16_t )( cfg->nb_freq_hop * 4u ) );
}

uint32_t lr20xx_radio_lora_get_time_on_air_numerator( const lr20xx_radio_lora_pkt_params_t* pkt_p,
                                                      const lr20xx_radio_lora_mod_params_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->pkt_mode == LR20XX_RADIO_LORA_PKT_IMPLICIT;

    int32_t fine_synch        = ( sf <= 6 ) ? 1 : 0;
    bool    long_interleaving = ( mod_p->cr > 4 );

    int32_t total_bytes_nb = pld_len_in_bytes + ( ( pkt_p->crc == LR20XX_RADIO_LORA_CRC_ENABLED ) ? 2 : 0 );
    int32_t tx_bits_symbol = sf - 2 * ( mod_p->ppm != 0 ? 1 : 0 );

    int32_t ceil_numerator;
    int32_t ceil_denominator;

    int32_t symbols_nb_data;
    int32_t tx_infobits_header;
    int32_t tx_infobits_payload;

    if( long_interleaving )
    {
        const int32_t fec_rate_numerator   = 4;
        const int32_t fec_rate_denominator = ( mod_p->cr + ( mod_p->cr == 7 ? 1 : 0 ) );

        if( pld_is_fix )
        {
            int32_t tx_bits_symbol_start = sf - 2 + 2 * fine_synch;
            if( 8 * total_bytes_nb * fec_rate_denominator <= 7 * fec_rate_numerator * tx_bits_symbol_start )
            {
                ceil_numerator   = 8 * total_bytes_nb * fec_rate_denominator;
                ceil_denominator = fec_rate_numerator * tx_bits_symbol_start;
            }
            else
            {
                int32_t tx_codedbits_header = tx_bits_symbol_start * 8;
                ceil_numerator = 8 * fec_rate_numerator * tx_bits_symbol + 8 * total_bytes_nb * fec_rate_denominator -
                                 fec_rate_numerator * tx_codedbits_header;
                ceil_denominator = fec_rate_numerator * tx_bits_symbol;
            }
        }
        else
        {
            tx_infobits_header = ( sf * 4 + fine_synch * 8 - 28 ) & ~0x07;
            if( tx_infobits_header < 8 * total_bytes_nb )
            {
                if( tx_infobits_header > 8 * pld_len_in_bytes )
                {
                    tx_infobits_header = 8 * pld_len_in_bytes;
                }
            }
            tx_infobits_payload = 8 * total_bytes_nb - tx_infobits_header;
            if( tx_infobits_payload < 0 )
            {
                tx_infobits_payload = 0;
            }

            ceil_numerator   = tx_infobits_payload * fec_rate_denominator + 8 * fec_rate_numerator * tx_bits_symbol;
            ceil_denominator = fec_rate_numerator * tx_bits_symbol;
        }
    }
    else
    {
        tx_infobits_header = sf * 4 + fine_synch * 8 - 8;

        if( !pld_is_fix )
        {
            tx_infobits_header -= 20;
        }

        tx_infobits_payload = 8 * total_bytes_nb - tx_infobits_header;

        if( tx_infobits_payload < 0 ) tx_infobits_payload = 0;

        ceil_numerator   = tx_infobits_payload;
        ceil_denominator = 4 * tx_bits_symbol;
    }

    symbols_nb_data = ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator );
    if( !long_interleaving )
    {
        symbols_nb_data = symbols_nb_data * ( mod_p->cr + 4 ) + 8;
    }
    const int32_t intermed = pkt_p->preamble_len_in_symb + 4 + 2 * fine_synch + symbols_nb_data;

    return ( uint32_t ) ( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) ) - 1;
}

uint32_t lr20xx_radio_lora_get_bw_in_hz( lr20xx_radio_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case LR20XX_RADIO_LORA_BW_7:
        bw_in_hz = 7812UL;
        break;
    case LR20XX_RADIO_LORA_BW_10:
        bw_in_hz = 10417UL;
        break;
    case LR20XX_RADIO_LORA_BW_15:
        bw_in_hz = 15625UL;
        break;
    case LR20XX_RADIO_LORA_BW_20:
        bw_in_hz = 20833UL;
        break;
    case LR20XX_RADIO_LORA_BW_31:
        bw_in_hz = 31250UL;
        break;
    case LR20XX_RADIO_LORA_BW_41:
        bw_in_hz = 41667UL;
        break;
    case LR20XX_RADIO_LORA_BW_62:
        bw_in_hz = 62500UL;
        break;
    case LR20XX_RADIO_LORA_BW_83:
        bw_in_hz = 83340UL;
        break;
    case LR20XX_RADIO_LORA_BW_101:
        bw_in_hz = 101563UL;
        break;
    case LR20XX_RADIO_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case LR20XX_RADIO_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case LR20XX_RADIO_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    case LR20XX_RADIO_LORA_BW_203:
        bw_in_hz = 203000UL;
        break;
    case LR20XX_RADIO_LORA_BW_406:
        bw_in_hz = 406000UL;
        break;
    case LR20XX_RADIO_LORA_BW_812:
        bw_in_hz = 812000UL;
        break;
    case LR20XX_RADIO_LORA_BW_1000:
        bw_in_hz = 1000000UL;
        break;
    }

    return bw_in_hz;
}

uint32_t lr20xx_radio_lora_get_time_on_air_in_ms( const lr20xx_radio_lora_pkt_params_t* pkt_p,
                                                  const lr20xx_radio_lora_mod_params_t* mod_p )
{
    uint32_t numerator   = 1000U * lr20xx_radio_lora_get_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = lr20xx_radio_lora_get_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

lr20xx_radio_lora_ppm_t lr20xx_radio_lora_get_recommended_ppm_offset( lr20xx_radio_lora_sf_t sf,
                                                                      lr20xx_radio_lora_bw_t bw )
{
    // PPM offset is LR20XX_RADIO_LORA_PPM_1_4, except for the cases that follow
    lr20xx_radio_lora_ppm_t ppm_offset = LR20XX_RADIO_LORA_PPM_1_4;

    if( ( sf != LR20XX_RADIO_LORA_SF11 ) && ( sf != LR20XX_RADIO_LORA_SF12 ) )
    {
        // 1. If sf is not SF11 nor SF12: no ppm offset
        ppm_offset = LR20XX_RADIO_LORA_NO_PPM;
    }
    else
    {
        // 2. Else it depends on the bandwidth
        switch( bw )
        {
        case LR20XX_RADIO_LORA_BW_1000:
        case LR20XX_RADIO_LORA_BW_500:
        {
            ppm_offset = LR20XX_RADIO_LORA_NO_PPM;
            break;
        }
        case LR20XX_RADIO_LORA_BW_250:
        {
            if( sf == LR20XX_RADIO_LORA_SF11 )
            {
                ppm_offset = LR20XX_RADIO_LORA_NO_PPM;
            }
            else if( sf == LR20XX_RADIO_LORA_SF12 )
            {
                ppm_offset = LR20XX_RADIO_LORA_PPM_1_4;
            }
            break;
        }
        case LR20XX_RADIO_LORA_BW_812:
        case LR20XX_RADIO_LORA_BW_406:
        case LR20XX_RADIO_LORA_BW_203:
        {
            ppm_offset = LR20XX_RADIO_LORA_PPM_1_4;
            break;
        }
        default:
        {
            // Empty on purpose
        }
        }
    }
    return ppm_offset;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

lr20xx_status_t abstract_search_symbols( const void* context, uint8_t n_symbols, search_symbol_format_t format )
{
    const uint8_t cbuffer[LR20XX_RADIO_LORA_SET_LORA_SEARCH_SYMBOLS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_LORA_SEARCH_SYMBOLS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LORA_SET_LORA_SEARCH_SYMBOLS_OC >> 0 ),
        n_symbols,
        ( uint8_t ) format,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LORA_SET_LORA_SEARCH_SYMBOLS_CMD_LENGTH,
                                                 0, 0 );
}

uint16_t read_2_bytes_msbf( const uint8_t* buffer )
{
    return ( uint16_t ) ( ( ( ( uint16_t ) buffer[0] ) << 8 ) + ( ( ( uint16_t ) buffer[1] ) << 0 ) );
}

uint8_t radio_lora_side_detector_cfg_to_byte( const lr20xx_radio_lora_side_detector_cfg_t* side_detector_cfg )
{
    return ( uint8_t ) ( ( side_detector_cfg->sf << 4 ) + ( side_detector_cfg->ppm << 2 ) + side_detector_cfg->iq );
}

/* --- EOF ------------------------------------------------------------------ */
