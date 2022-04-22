/*!
 * @file      lr11xx_lr_fhss.c
 *
 * @brief     LR_FHSS driver implementation for LR11XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#include "lr11xx_lr_fhss.h"
#include "lr11xx_radio.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR11XX_LR_FHSS_PKT_TYPE_LR_FHSS ( 0x04 )
#define LR_FHSS_BITRATE_IN_256_BPS_STEPS ( 125000 )
#define LR11XX_LR_FHSS_SET_MODULATION_PARAMS_LR_FHSS_CMD_LENGTH ( 2 + 5 )
#define LR11XX_LR_FHSS_BUILD_FRAME_LENGTH ( 2 + 9 )
#define LR11XX_LR_FHSS_SET_SYNC_WORD_LENGTH ( 2 + 0 )
#define LR11XX_LR_FHSS_SET_MODULATION_PARAM_DIVIDE_BITRATE_BY_256 ( 0x80000000 )
#define LR11XX_LR_FHSS_HEADER_BITS ( 114 )
#define LR11XX_LR_FHSS_FRAG_BITS ( 48 )
#define LR11XX_LR_FHSS_BLOCK_PREAMBLE_BITS ( 2 )
#define LR11XX_LR_FHSS_BLOCK_BITS ( LR11XX_LR_FHSS_FRAG_BITS + LR11XX_LR_FHSS_BLOCK_PREAMBLE_BITS )
#define LR11XX_LR_FHSS_SYNCWORD_LENGTH ( 4 )

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
    LR11XX_LR_FHSS_SET_MODULATION_PARAM_OC = 0x020F,
    LR11XX_LR_FHSS_BUILD_FRAME_OC          = 0x022C,
    LR11XX_LR_FHSS_SET_SYNC_WORD_OC        = 0x022D,
};

/*!
 * @brief Hopping enable/disabled enumerations for \ref lr11xx_lr_fhss_build_frame
 */
typedef enum
{
    LR11XX_LR_FHSS_HOPPING_DISABLE = 0x00,
    LR11XX_LR_FHSS_HOPPING_ENABLE  = 0x01,
} lr11xx_lr_fhss_hopping_configuration_t;

/*!
 * @brief Pulse shape configurations
 */
typedef enum
{
    LR11XX_LR_FHSS_PULSE_SHAPE_BT_1 = 0x0B  //!< Gaussian BT 1.0
} lr11xx_lr_fhss_pulse_shape_t;

/*!
 * @brief Modulation configuration for LR_FHSS packets
 */
typedef struct lr11xx_lr_fhss_mod_params_lr_fhss_s
{
    uint32_t                     br_in_bps;    //!< LR_FHSS bitrate [bit/s]
    lr11xx_lr_fhss_pulse_shape_t pulse_shape;  //!< LR_FHSS pulse shape
} lr11xx_lr_fhss_mod_params_lr_fhss_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Set the modulation parameters for LR_FHSS
 *
 * The command @ref lr11xx_lr_fhss_set_pkt_type must be called prior this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] mod_params The structure of modulation configuration
 *
 * @returns Operation status
 *
 * @see lr11xx_lr_fhss_set_pkt_type
 */
static lr11xx_status_t lr11xx_lr_fhss_set_lr_fhss_mod_params( const void*                                context,
                                                              const lr11xx_lr_fhss_mod_params_lr_fhss_t* mod_params );

/*!
 * @brief Set the syncword for LR_FHSS
 *
 * Default value: 0x2C0F7995
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word The syncword to set. It is up to the caller to ensure this array is at least four bytes long
 *
 * @returns Operation status
 */
static lr11xx_status_t lr11xx_lr_fhss_set_sync_word( const void*   context,
                                                     const uint8_t sync_word[LR11XX_LR_FHSS_SYNCWORD_LENGTH] );

/*!
 * @brief Get the bit count and block count for a LR-FHSS frame
 *
 * @param  [in] params         Parameter structure
 * @param  [in] payload_length Length of physical payload, in bytes
 *
 * @returns                    Length of physical payload, in bits
 */

static uint16_t lr11xx_lr_fhss_get_nb_bits( const lr_fhss_v1_params_t* params, uint16_t payload_length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_lr_fhss_init( const void* context )
{
    const lr11xx_status_t set_packet_type_status =
        lr11xx_radio_set_pkt_type( context, LR11XX_LR_FHSS_PKT_TYPE_LR_FHSS );
    if( set_packet_type_status != LR11XX_STATUS_OK )
    {
        return set_packet_type_status;
    }

    const lr11xx_lr_fhss_mod_params_lr_fhss_t mod_lr_fhss = {
        .br_in_bps   = LR11XX_LR_FHSS_SET_MODULATION_PARAM_DIVIDE_BITRATE_BY_256 + LR_FHSS_BITRATE_IN_256_BPS_STEPS,
        .pulse_shape = LR11XX_LR_FHSS_PULSE_SHAPE_BT_1
    };

    const lr11xx_status_t set_modulation_param_status = lr11xx_lr_fhss_set_lr_fhss_mod_params( context, &mod_lr_fhss );
    return set_modulation_param_status;
}

lr11xx_status_t lr11xx_lr_fhss_build_frame( const void* context, const lr11xx_lr_fhss_params_t* lr_fhss_params,
                                            uint16_t hop_sequence_id, const uint8_t* payload, uint8_t payload_length )
{
    // Since the build_frame command is last, it is possible to check status through stat1

    lr11xx_status_t status = lr11xx_lr_fhss_set_sync_word( context, lr_fhss_params->lr_fhss_params.sync_word );
    if( status != LR11XX_STATUS_OK )
    {
        return status;
    }

    const uint8_t cbuffer[LR11XX_LR_FHSS_BUILD_FRAME_LENGTH] = {
        ( uint8_t ) ( LR11XX_LR_FHSS_BUILD_FRAME_OC >> 8 ),
        ( uint8_t ) ( LR11XX_LR_FHSS_BUILD_FRAME_OC >> 0 ),
        ( uint8_t ) lr_fhss_params->lr_fhss_params.header_count,
        ( uint8_t ) lr_fhss_params->lr_fhss_params.cr,
        ( uint8_t ) lr_fhss_params->lr_fhss_params.modulation_type,
        ( uint8_t ) lr_fhss_params->lr_fhss_params.grid,
        ( uint8_t ) ( lr_fhss_params->lr_fhss_params.enable_hopping ? LR11XX_LR_FHSS_HOPPING_ENABLE
                                                                    : LR11XX_LR_FHSS_HOPPING_DISABLE ),
        ( uint8_t ) lr_fhss_params->lr_fhss_params.bw,
        ( uint8_t ) ( hop_sequence_id >> 8 ),
        ( uint8_t ) ( hop_sequence_id >> 0 ),
        ( uint8_t ) lr_fhss_params->device_offset,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_LR_FHSS_BUILD_FRAME_LENGTH, payload,
                                                 payload_length );
}

uint32_t lr11xx_lr_fhss_get_time_on_air_in_ms( const lr11xx_lr_fhss_params_t* params, uint16_t payload_length )
{
    // Multiply by 1000 / 488.28125, or equivalently 256/125, rounding up
    return ( ( lr11xx_lr_fhss_get_nb_bits( &params->lr_fhss_params, payload_length ) << 8 ) + 124 ) / 125;
}

unsigned int lr11xx_lr_fhss_get_hop_sequence_count( const lr11xx_lr_fhss_params_t* lr_fhss_params )
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

lr11xx_status_t lr11xx_lr_fhss_set_lr_fhss_mod_params( const void*                                radio,
                                                       const lr11xx_lr_fhss_mod_params_lr_fhss_t* mod_params )
{
    const uint8_t cbuffer[LR11XX_LR_FHSS_SET_MODULATION_PARAMS_LR_FHSS_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_LR_FHSS_SET_MODULATION_PARAM_OC >> 8 ),
        ( uint8_t ) ( LR11XX_LR_FHSS_SET_MODULATION_PARAM_OC >> 0 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 24 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 16 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 8 ),
        ( uint8_t ) ( mod_params->br_in_bps >> 0 ),
        ( uint8_t ) mod_params->pulse_shape,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( radio, cbuffer,
                                                 LR11XX_LR_FHSS_SET_MODULATION_PARAMS_LR_FHSS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_lr_fhss_set_sync_word( const void*   context,
                                              const uint8_t sync_word[LR11XX_LR_FHSS_SYNCWORD_LENGTH] )
{
    const uint8_t cbuffer[LR11XX_LR_FHSS_SET_SYNC_WORD_LENGTH] = {
        ( uint8_t ) ( LR11XX_LR_FHSS_SET_SYNC_WORD_OC >> 8 ),
        ( uint8_t ) ( LR11XX_LR_FHSS_SET_SYNC_WORD_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_LR_FHSS_SET_SYNC_WORD_LENGTH, sync_word,
                                                 LR11XX_LR_FHSS_SYNCWORD_LENGTH );
}

uint16_t lr11xx_lr_fhss_get_nb_bits( const lr_fhss_v1_params_t* params, uint16_t payload_length )
{
    uint16_t length_bits = ( payload_length + 2 ) * 8 + 6;
    switch( params->cr )
    {
    case LR_FHSS_V1_CR_5_6:
        length_bits = ( ( length_bits * 6 ) + 4 ) / 5;
        break;

    case LR_FHSS_V1_CR_2_3:
        length_bits = length_bits * 3 / 2;
        break;

    case LR_FHSS_V1_CR_1_2:
        length_bits = length_bits * 2;
        break;

    case LR_FHSS_V1_CR_1_3:
        length_bits = length_bits * 3;
        break;
    }

    uint16_t payload_bits    = ( length_bits / LR11XX_LR_FHSS_FRAG_BITS ) * LR11XX_LR_FHSS_BLOCK_BITS;
    uint16_t last_block_bits = length_bits % LR11XX_LR_FHSS_FRAG_BITS;
    if( last_block_bits > 0 )
    {
        payload_bits += last_block_bits + 2;
    }

    return LR11XX_LR_FHSS_HEADER_BITS * params->header_count + payload_bits;
}