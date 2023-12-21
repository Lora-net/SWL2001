/**
 * @file      sx126x_lr_fhss.c
 *
 * @brief     SX126x LR-FHSS driver implementation
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

#include "lr_fhss_mac.h"
#include "sx126x_lr_fhss.h"
#include "sx126x_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* \cond */

#define SX126X_SET_MODULATION_PARAMS ( 0x8B )
#define SX126X_SET_PKT_PARAMS ( 0x8C )

#define SX126X_LR_FHSS_DISABLE_HOPPING ( 0 )
#define SX126X_LR_FHSS_ENABLE_HOPPING ( 1 )

#define SX126X_LR_FHSS_HOP_TABLE_SIZE ( 16 )
#define SX126X_LR_FHSS_HOP_ENTRY_SIZE ( 6 )

#define SX126X_LR_FHSS_GRID_3906_HZ_PLL_STEPS ( 4096 )
#define SX126X_LR_FHSS_GRID_25391_HZ_PLL_STEPS ( 26624 )

#define SX126X_LR_FHSS_GRID_INDEX_TO_PLL_STEPS ( 512 )

/* \endcond */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/**
 * @brief Configure the radio to perform frequency hopping
 *
 * @param [in]  context  Chip implementation context
 * @param [in]  nb_bytes Total number of bytes
 * @param [in]  nb_hops  Total number of hops
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_write_hop_config( const void* context, const uint8_t nb_bytes, const uint8_t nb_hops );

/**
 * @brief Write a hop frequency/duration pair to the radio hop table
 *
 * @param [in]  context           Chip implementation context
 * @param [in]  index             Index to chip hop table
 * @param [in]  nb_symbols        Hop duration in symbols
 * @param [in]  freq_in_pll_steps Hop frequency, in PLL steps
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_write_hop( const void* context, const uint8_t index, const uint16_t nb_symbols,
                                          const uint32_t freq_in_pll_steps );

/**
 * @brief Get Frequency, in PLL steps, of the next hop
 *
 * @param [in]  params sx126x LR-FHSS parameter structure
 * @param [out] state  sx126x LR-FHSS state structure that will be initialized by this function
 *
 * @returns Frequency, in PLL steps, of the next hop
 */
uint32_t sx126x_lr_fhss_get_next_freq_in_pll_steps( const sx126x_lr_fhss_params_t* params,
                                                    sx126x_lr_fhss_state_t*        state );

/**
 * @brief Get grid frequency, in PLL steps
 *
 * @param [in]  params sx126x LR-FHSS parameter structure
 *
 * @returns Grid frequency, in PLL steps
 */
static inline unsigned int sx126x_lr_fhss_get_grid_in_pll_steps( const sx126x_lr_fhss_params_t* params );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_status_t sx126x_lr_fhss_init( const void* context, const sx126x_lr_fhss_params_t* params )
{
    const uint8_t pkt_params_buf[] = {
        SX126X_SET_PKT_PARAMS, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    const uint8_t mod_params_buf[] = {
        SX126X_SET_MODULATION_PARAMS, 32, 0, 0, SX126X_GFSK_PULSE_SHAPE_BT_1, 0, 0, 0, 0,
    };

    sx126x_status_t status = sx126x_set_pkt_type( context, SX126X_PKT_TYPE_LR_FHSS );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    status = ( sx126x_status_t ) sx126x_hal_write( context, pkt_params_buf, sizeof( pkt_params_buf ), 0, 0 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    status = ( sx126x_status_t ) sx126x_hal_write( context, mod_params_buf, sizeof( mod_params_buf ), 0, 0 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    status = sx126x_set_buffer_base_address( context, 0x00, 0x00 );
    return status;
}

uint16_t sx126x_lr_fhss_get_bit_delay_in_us( const sx126x_lr_fhss_params_t* params, uint16_t payload_length )
{
    lr_fhss_digest_t digest;

    lr_fhss_process_parameters( &params->lr_fhss_params, payload_length, &digest );

    const uint8_t nb_padding_bits = 1 + ( ( 32768 - digest.nb_bits ) & 0x07 );

    return 1550 + nb_padding_bits * 2048;
}

sx126x_status_t sx126x_lr_fhss_process_parameters( const sx126x_lr_fhss_params_t* params, uint16_t hop_sequence_id,
                                                   uint16_t payload_length, sx126x_lr_fhss_state_t* state )
{
    lr_fhss_process_parameters( &params->lr_fhss_params, payload_length, &state->digest );

    if( state->digest.nb_bytes > LR_FHSS_MAX_PHY_PAYLOAD_BYTES )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }
    if( params->lr_fhss_params.grid == LR_FHSS_V1_GRID_25391_HZ )
    {
        if( params->device_offset > 25 || params->device_offset < -26 )
        {
            return SX126X_STATUS_UNKNOWN_VALUE;
        }
        if( params->lr_fhss_params.bw < LR_FHSS_V1_BW_722656_HZ )
        {
            return SX126X_STATUS_UNKNOWN_VALUE;
        }
    }
    if( params->lr_fhss_params.grid == LR_FHSS_V1_GRID_3906_HZ )
    {
        if( params->device_offset > 3 || params->device_offset < -4 )
        {
            return SX126X_STATUS_UNKNOWN_VALUE;
        }
    }

    // Initialize hop index and params
    state->current_hop = 0;
    lr_fhss_status_t status =
        lr_fhss_get_hop_params( &params->lr_fhss_params, &state->hop_params, &state->lfsr_state, hop_sequence_id );
    if( status != LR_FHSS_STATUS_OK )
    {
        return ( sx126x_status_t ) status;
    }

    // Skip the hop frequencies inside the set [0, 4 - header_count):
    if( params->lr_fhss_params.enable_hopping != 0 )
    {
        for( int i = 0; i < 4 - params->lr_fhss_params.header_count; ++i )
        {
            lr_fhss_get_next_state( &state->lfsr_state, &state->hop_params );
        }
    }

    state->next_freq_in_pll_steps = sx126x_lr_fhss_get_next_freq_in_pll_steps( params, state );
    return SX126X_STATUS_OK;
}

sx126x_status_t sx126x_lr_fhss_write_hop_sequence_head( const void* context, const sx126x_lr_fhss_params_t* params,
                                                        sx126x_lr_fhss_state_t* state )
{
    sx126x_status_t status = sx126x_lr_fhss_write_hop_config( context, state->digest.nb_bytes, state->digest.nb_hops );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    const uint16_t pulse_shape_compensation = 1;

    if( params->lr_fhss_params.enable_hopping == 0 )
    {
        // (LR_FHSS_HEADER_BITS + pulse_shape_compensation) symbols on first sync_word, LR_FHSS_HEADER_BITS on next
        // sync_words, LR_FHSS_BLOCK_BITS on payload
        const uint16_t nb_symbols = state->digest.nb_bits + pulse_shape_compensation;

        status = sx126x_lr_fhss_write_hop( context, state->current_hop, nb_symbols, state->next_freq_in_pll_steps );
        if( status != SX126X_STATUS_OK )
        {
            return status;
        }
        state->current_hop++;
        state->digest.nb_bits = 0;
    }
    else
    {
        // fill at most SX126X_LR_FHSS_HOP_TABLE_SIZE hops of the hardware hop table
        uint8_t truncated_hops = state->digest.nb_hops;
        if( truncated_hops > SX126X_LR_FHSS_HOP_TABLE_SIZE )
        {
            truncated_hops = SX126X_LR_FHSS_HOP_TABLE_SIZE;
        }

        while( state->current_hop < truncated_hops )
        {
            uint16_t nb_symbols;

            // (LR_FHSS_HEADER_BITS + pulse_shape_compensation) symbols on first sync_word, LR_FHSS_HEADER_BITS on
            // next sync_words, LR_FHSS_BLOCK_BITS on payload
            if( state->current_hop >= params->lr_fhss_params.header_count )
            {
                if( state->digest.nb_bits > LR_FHSS_BLOCK_BITS )
                {
                    nb_symbols = LR_FHSS_BLOCK_BITS;
                }
                else
                {
                    nb_symbols = state->digest.nb_bits;
                }
            }
            else if( state->current_hop > 0 )
            {
                nb_symbols = LR_FHSS_HEADER_BITS;
            }
            else
            {
                nb_symbols = LR_FHSS_HEADER_BITS + pulse_shape_compensation;
            }

            status = sx126x_lr_fhss_write_hop( context, state->current_hop, nb_symbols, state->next_freq_in_pll_steps );
            if( status != SX126X_STATUS_OK )
            {
                return status;
            }

            state->current_hop++;
            state->digest.nb_bits -= nb_symbols;

            state->next_freq_in_pll_steps = sx126x_lr_fhss_get_next_freq_in_pll_steps( params, state );
        }
    }

    return status;
}

sx126x_status_t sx126x_lr_fhss_write_payload( const void* context, const sx126x_lr_fhss_state_t* state,
                                              const uint8_t* payload )
{
    return sx126x_write_buffer( context, 0x00, payload, state->digest.nb_bytes );
}

sx126x_status_t sx126x_lr_fhss_build_frame( const void* context, const sx126x_lr_fhss_params_t* params,
                                            sx126x_lr_fhss_state_t* state, uint16_t hop_sequence_id,
                                            const uint8_t* payload, uint16_t payload_length,
                                            uint32_t* first_frequency_in_pll_steps )
{
    sx126x_status_t status = sx126x_lr_fhss_process_parameters( params, hop_sequence_id, payload_length, state );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    if( first_frequency_in_pll_steps )
    {
        *first_frequency_in_pll_steps = state->next_freq_in_pll_steps;
    }

    uint8_t tx_buffer[LR_FHSS_MAX_PHY_PAYLOAD_BYTES];
    lr_fhss_build_frame( &params->lr_fhss_params, state->hop_params.hop_sequence_id, payload, payload_length,
                         tx_buffer );

    status = sx126x_lr_fhss_write_payload( context, state, tx_buffer );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    status = sx126x_lr_fhss_write_hop_sequence_head( context, params, state );

    return status;
}

sx126x_status_t sx126x_lr_fhss_handle_hop( const void* context, const sx126x_lr_fhss_params_t* params,
                                           sx126x_lr_fhss_state_t* state )
{
    if( state->current_hop < state->digest.nb_hops )
    {
        uint16_t nb_bits;
        if( state->digest.nb_bits > LR_FHSS_BLOCK_BITS )
        {
            nb_bits = LR_FHSS_BLOCK_BITS;
        }
        else
        {
            nb_bits = state->digest.nb_bits;
        }
        sx126x_status_t status = sx126x_lr_fhss_write_hop( context, state->current_hop % SX126X_LR_FHSS_HOP_TABLE_SIZE,
                                                           LR_FHSS_BLOCK_BITS, state->next_freq_in_pll_steps );
        if( status != SX126X_STATUS_OK )
        {
            return status;
        }

        state->current_hop++;
        state->digest.nb_bits -= nb_bits;
        state->next_freq_in_pll_steps = sx126x_lr_fhss_get_next_freq_in_pll_steps( params, state );
    }
    return SX126X_STATUS_OK;
}

sx126x_status_t sx126x_lr_fhss_handle_tx_done( const void* context, const sx126x_lr_fhss_params_t* params,
                                               sx126x_lr_fhss_state_t* state )
{
    const uint8_t ctrl = SX126X_LR_FHSS_DISABLE_HOPPING;

    return sx126x_write_register( context, SX126X_LR_FHSS_REG_CTRL, &ctrl, 1 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

sx126x_status_t sx126x_lr_fhss_write_hop_config( const void* context, const uint8_t nb_bytes, const uint8_t nb_hops )
{
    uint8_t data[] = { SX126X_LR_FHSS_ENABLE_HOPPING, nb_bytes, nb_hops };

    return sx126x_write_register( context, SX126X_LR_FHSS_REG_CTRL, data, 3 );
}

sx126x_status_t sx126x_lr_fhss_write_hop( const void* context, const uint8_t index, const uint16_t nb_symbols,
                                          const uint32_t freq_in_pll_steps )
{
    if( index >= SX126X_LR_FHSS_HOP_TABLE_SIZE )
    {
        return SX126X_STATUS_ERROR;
    }

    uint8_t data[SX126X_LR_FHSS_HOP_ENTRY_SIZE] = {
        ( uint8_t ) ( nb_symbols >> 8 ),         ( uint8_t ) nb_symbols,
        ( uint8_t ) ( freq_in_pll_steps >> 24 ), ( uint8_t ) ( freq_in_pll_steps >> 16 ),
        ( uint8_t ) ( freq_in_pll_steps >> 8 ),  ( uint8_t ) freq_in_pll_steps,
    };

    return sx126x_write_register( context, SX126X_LR_FHSS_REG_NUM_SYMBOLS_0 + ( SX126X_LR_FHSS_HOP_ENTRY_SIZE * index ),
                                  data, SX126X_LR_FHSS_HOP_ENTRY_SIZE );
}

uint32_t sx126x_lr_fhss_get_next_freq_in_pll_steps( const sx126x_lr_fhss_params_t* params,
                                                    sx126x_lr_fhss_state_t*        state )
{
#ifdef HOP_AT_CENTER_FREQ
    const int16_t freq_table  = 0;
    uint32_t      grid_offset = 0;
#else
    const int16_t freq_table =
        lr_fhss_get_next_freq_in_grid( &state->lfsr_state, &state->hop_params, &params->lr_fhss_params );
    uint32_t nb_channel_in_grid = params->lr_fhss_params.grid ? 8 : 52;
    uint32_t grid_offset        = ( 1 + ( state->hop_params.n_grid % 2 ) ) * ( nb_channel_in_grid / 2 );
#endif

    unsigned int grid_in_pll_steps = sx126x_lr_fhss_get_grid_in_pll_steps( params );
    uint32_t     freq              = params->center_freq_in_pll_steps - freq_table * grid_in_pll_steps -
                    ( params->device_offset + grid_offset ) * SX126X_LR_FHSS_GRID_INDEX_TO_PLL_STEPS;

#ifndef HOP_AT_CENTER_FREQ
    // Perform frequency correction for every other sync header
    if( params->lr_fhss_params.enable_hopping && ( state->current_hop < params->lr_fhss_params.header_count ) )
    {
        if( ( ( ( params->lr_fhss_params.header_count - state->current_hop ) % 2 ) == 0 ) )
        {
            // OFFSET_SYNCWORD = 488.28125 / 2, and FREQ_STEP_SX1261_2 = 0.95367431640625, so
            // OFFSET_SYNCWORD / FREQ_STEP_SX1261_2 = 256
            freq = freq + 256;
        }
    }
#endif
    return freq;
}

static inline unsigned int sx126x_lr_fhss_get_grid_in_pll_steps( const sx126x_lr_fhss_params_t* params )
{
    return ( params->lr_fhss_params.grid == LR_FHSS_V1_GRID_3906_HZ ) ? SX126X_LR_FHSS_GRID_3906_HZ_PLL_STEPS
                                                                      : SX126X_LR_FHSS_GRID_25391_HZ_PLL_STEPS;
}

/* --- EOF ------------------------------------------------------------------ */
