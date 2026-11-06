/*!
 * @file      lr20xx_radio_lr_fhss.c
 *
 * @brief     LR-FHSS driver implementation for LR20XX
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

#include "lr20xx_radio_lr_fhss.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RADIO_LR_FHSS_BUILD_FRAME_CMD_LENGTH ( 2 + 6 )
#define LR20XX_RADIO_LR_FHSS_SET_SYNCWORD_CMD_LENGTH ( 2 )

#define LR20XX_RADIO_LR_FHSS_HEADER_BITS ( 114 )
#define LR20XX_RADIO_LR_FHSS_FRAG_BITS ( 48 )
#define LR20XX_RADIO_LR_FHSS_BLOCK_PREAMBLE_BITS ( 2 )
#define LR20XX_RADIO_LR_FHSS_BLOCK_BITS ( LR20XX_RADIO_LR_FHSS_FRAG_BITS + LR20XX_RADIO_LR_FHSS_BLOCK_PREAMBLE_BITS )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for radio-related operations
 */
enum
{
    LR20XX_RADIO_LR_FHSS_BUILD_FRAME_OC  = 0x0256,
    LR20XX_RADIO_LR_FHSS_SET_SYNCWORD_OC = 0x0257,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Get the bit count and block count for a LR-FHSS frame
 *
 * @param  [in] params         Parameter structure
 * @param  [in] payload_length Length of physical payload, in bytes
 *
 * @returns                    Length of physical payload, in bits
 */

static uint16_t lr20xx_radio_lr_fhss_get_nb_bits( const lr_fhss_v1_params_t* params, uint16_t payload_length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_radio_lr_fhss_build_frame( const void*                          context,
                                                  const lr20xx_radio_lr_fhss_params_t* lr_fhss_params,
                                                  uint16_t hop_sequence_id, const uint8_t* payload,
                                                  uint8_t payload_length )
{
    const lr20xx_status_t status =
        lr20xx_radio_lr_fhss_set_sync_word( context, lr_fhss_params->lr_fhss_params.sync_word );
    if( status != LR20XX_STATUS_OK )
    {
        return status;
    }

    const uint8_t cbuffer[LR20XX_RADIO_LR_FHSS_BUILD_FRAME_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LR_FHSS_BUILD_FRAME_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LR_FHSS_BUILD_FRAME_OC >> 0 ),
        ( uint8_t ) ( ( lr_fhss_params->lr_fhss_params.header_count << 4 ) +
                      ( uint8_t ) lr_fhss_params->lr_fhss_params.cr ),
        ( uint8_t ) ( ( ( ( uint8_t ) lr_fhss_params->lr_fhss_params.modulation_type ) << 4 ) +
                      ( uint8_t ) lr_fhss_params->lr_fhss_params.grid ),
        ( uint8_t ) ( ( lr_fhss_params->lr_fhss_params.enable_hopping ? ( 0x01 << 4 ) : 0x00 ) +
                      ( uint8_t ) lr_fhss_params->lr_fhss_params.bw ),
        ( uint8_t ) ( hop_sequence_id >> 8 ),
        ( uint8_t ) ( hop_sequence_id >> 0 ),
        ( uint8_t ) lr_fhss_params->device_offset,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LR_FHSS_BUILD_FRAME_CMD_LENGTH, payload,
                                                 payload_length );
}

lr20xx_status_t lr20xx_radio_lr_fhss_set_sync_word( const void*   context,
                                                    const uint8_t sync_word[LR20XX_RADIO_LR_FHSS_SYNC_WORD_LENGTH] )
{
    const uint8_t cbuffer[LR20XX_RADIO_LR_FHSS_SET_SYNCWORD_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RADIO_LR_FHSS_SET_SYNCWORD_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RADIO_LR_FHSS_SET_SYNCWORD_OC >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RADIO_LR_FHSS_SET_SYNCWORD_CMD_LENGTH,
                                                 sync_word, LR20XX_RADIO_LR_FHSS_SYNC_WORD_LENGTH );
}

lr20xx_status_t lr20xx_radio_lr_fhss_init( const void* context )
{
    return lr20xx_radio_common_set_pkt_type( context, LR20XX_RADIO_COMMON_PKT_TYPE_LRFHSS );
}

uint32_t lr20xx_radio_lr_fhss_get_time_on_air_in_ms( const lr20xx_radio_lr_fhss_params_t* params,
                                                     uint16_t                             payload_length )
{
    // Multiply by 1000 / 488.28125, or equivalently 256/125, rounding up
    return ( uint32_t ) ( ( lr20xx_radio_lr_fhss_get_nb_bits( &params->lr_fhss_params, payload_length ) << 8 ) + 124 ) /
           125;
}

unsigned int lr20xx_radio_lr_fhss_get_hop_sequence_count( const lr20xx_radio_lr_fhss_params_t* lr_fhss_params )
{
    if( ( lr_fhss_params->lr_fhss_params.grid == LR_FHSS_V1_GRID_25391_HZ ) ||
        ( ( lr_fhss_params->lr_fhss_params.grid == LR_FHSS_V1_GRID_3906_HZ ) &&
          ( lr_fhss_params->lr_fhss_params.bw < LR_FHSS_V1_BW_335938_HZ ) ) )
    {
        return 384;
    }
    return 512;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION ---------------------------------------------
 */

uint16_t lr20xx_radio_lr_fhss_get_nb_bits( const lr_fhss_v1_params_t* params, uint16_t payload_length )
{
    uint16_t length_bits = ( uint16_t ) ( ( payload_length + 2u ) * 8u + 6u );
    switch( params->cr )
    {
    case LR_FHSS_V1_CR_5_6:
        length_bits = ( uint16_t ) ( ( ( length_bits * 6u ) + 4u ) / 5u );
        break;

    case LR_FHSS_V1_CR_2_3:
        length_bits = ( uint16_t ) ( length_bits * 3u / 2u );
        break;

    case LR_FHSS_V1_CR_1_2:
        length_bits = ( uint16_t ) ( length_bits * 2u );
        break;

    case LR_FHSS_V1_CR_1_3:
        length_bits = ( uint16_t ) ( length_bits * 3u );
        break;
    }

    uint16_t payload_bits =
        ( uint16_t ) ( ( length_bits / LR20XX_RADIO_LR_FHSS_FRAG_BITS ) * LR20XX_RADIO_LR_FHSS_BLOCK_BITS );
    uint16_t last_block_bits = ( uint16_t ) ( length_bits % LR20XX_RADIO_LR_FHSS_FRAG_BITS );
    if( last_block_bits > 0 )
    {
        payload_bits = ( uint16_t ) ( payload_bits + ( uint16_t ) ( last_block_bits + 2u ) );
    }

    return ( uint16_t ) ( LR20XX_RADIO_LR_FHSS_HEADER_BITS * params->header_count + payload_bits );
}
