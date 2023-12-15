/**
 * @file      lr_fhss_mac.h
 *
 * @brief     Radio-independent LR-FHSS driver API
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

#ifndef LR_FHSS_MAC_H__
#define LR_FHSS_MAC_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#include "lr_fhss_v1_base_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define LR_FHSS_HDR_BYTES ( 10 )
#define LR_FHSS_HDR_BITS ( 8 * LR_FHSS_HDR_BYTES )
#define LR_FHSS_HALF_HDR_BYTES ( 5 )
#define LR_FHSS_HALF_HDR_BITS ( 8 * LR_FHSS_HALF_HDR_BYTES )
#define LR_FHSS_SYNC_WORD_BYTES ( 4 )
#define LR_FHSS_SYNC_WORD_BITS ( 8 * LR_FHSS_SYNC_WORD_BYTES )
#define LR_FHSS_MAX_PHY_PAYLOAD_BYTES ( 255 )
#define LR_FHSS_HEADER_BITS ( 114 )
#define LR_FHSS_FRAG_BITS ( 48 )
#define LR_FHSS_BLOCK_PREAMBLE_BITS ( 2 )
#define LR_FHSS_BLOCK_BITS ( LR_FHSS_FRAG_BITS + LR_FHSS_BLOCK_PREAMBLE_BITS )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LR-FHSS return status
 */
typedef enum lr_fhss_status_e
{
    LR_FHSS_STATUS_OK = 0,
    LR_FHSS_STATUS_UNSUPPORTED_FEATURE,
    LR_FHSS_STATUS_UNKNOWN_VALUE,
    LR_FHSS_STATUS_ERROR,
} lr_fhss_status_t;

/**
 * Digest, holding physical payload length and hop information
 */
typedef struct lr_fhss_digest_s
{
    uint16_t nb_bytes; /**< Length of LR-FHSS frame, in bytes */
    uint16_t nb_bits;  /**< Number of bits */
    uint8_t  nb_hops;  /**< Number of hops */
} lr_fhss_digest_t;

/**
 * Hopping configuration, created by @ref lr_fhss_get_hop_params, and used to generate hop sequence
 */
typedef struct lr_fhss_hop_params_s
{
    uint16_t n_grid;          /**< Ngrid, as described in specification */
    uint16_t polynomial;      /**< polynomial, as described in specification, used for hop sequence generation */
    uint16_t xoring_seed;     /**< xoring seed, as described in specification, used for hop sequence generation */
    uint16_t hop_sequence_id; /**< Hopping sequence seed, as described in specification, determines which hop sequence
                                 will be used */
} lr_fhss_hop_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return the number of hop sequences available using the given parameters
 *
 * @param  [in] params         LR-FHSS parameter structure
 *
 * @returns Number of valid hop sequences (512 or 384)
 */
unsigned int lr_fhss_get_hop_sequence_count( const lr_fhss_v1_params_t* params );

/**
 * @brief Fill the digest structure with various size-related data for a LR-FHSS frame
 *
 * @param  [in] params         LR-FHSS parameter structure
 * @param  [in] payload_length Length of payload, in bytes
 * @param [out] digest         Contains block count byte count, and number of hops
 */
void lr_fhss_process_parameters( const lr_fhss_v1_params_t* params, uint16_t payload_length, lr_fhss_digest_t* digest );

/**
 * @brief Fill the hop structure with various hop-related data for a LR-FHSS frame, and provide initial state
 *
 * @param  [in] params          LR-FHSS parameter structure
 * @param [out] hop_params      Hop parameter structure
 * @param [out] initial_state   Initial LFSR state
 * @param  [in] hop_sequence_id The hop sequence ID that will be used to obtain hop-related data
 *
 * @returns Operation status
 */
lr_fhss_status_t lr_fhss_get_hop_params( const lr_fhss_v1_params_t* params, lr_fhss_hop_params_t* hop_params,
                                         uint16_t* initial_state, uint16_t hop_sequence_id );

/**
 * @brief Update the LFSR state by performing a hop, and return the hop grid position
 *
 * @param  [in,out] lfsr_state LFSR state
 * @param      [in] hop_params Hop parameter structure
 *
 * @returns Hop position in the grid
 */
uint16_t lr_fhss_get_next_state( uint16_t* lfsr_state, const lr_fhss_hop_params_t* hop_params );

/**
 * @brief Return the frequency in grid units for given LR-FHSS parameters and hop index
 *
 * @param  [in,out] lfsr_state LFSR state
 * @param      [in] hop_params Hop parameter structure
 * @param      [in] params     LR-FHSS parameter structure
 *
 * @returns Frequency, in grid units
 */
int16_t lr_fhss_get_next_freq_in_grid( uint16_t* lfsr_state, const lr_fhss_hop_params_t* hop_params,
                                       const lr_fhss_v1_params_t* params );

/**
 * @brief Construct the LR-FHSS frame
 *
 * @param  [in] params            LR-FHSS parameter structure
 * @param  [in] hop_sequence_id   The hop sequence ID that will be used to obtain hop-related data
 * @param  [in] data_in           Pointer to input buffer
 * @param  [in] data_in_bytecount Length of input buffer, in bytes
 * @param [out] data_out          Pointer to a buffer into which the final LR-FHSS frame is stored, large enough to hold
 * 255 bytes
 *
 * @returns Length of frame, in bytes
 */
uint16_t lr_fhss_build_frame( const lr_fhss_v1_params_t* params, uint16_t hop_sequence_id, const uint8_t* data_in,
                              uint16_t data_in_bytecount, uint8_t* data_out );

/**
 * @brief Compute the numerator for LR-FHSS time-on-air computation.
 *
 * @remark To get the actual time-on-air in seconds, this value must be divided by the LR-FHSS bitrate in bits per
 * second, 488.28125.
 *
 * @param  [in] params         LR-FHSS parameter structure
 * @param  [in] payload_length Length of application payload, in bytes
 *
 * @returns LR-FHSS time-on-air numerator
 */
static inline uint32_t lr_fhss_get_time_on_air_numerator( const lr_fhss_v1_params_t* params, uint16_t payload_length )
{
    lr_fhss_digest_t digest;
    lr_fhss_process_parameters( params, payload_length, &digest );

    return digest.nb_bits;
}

/**
 * @brief Get the time on air in ms for LR-FHSS transmission
 *
 * @param  [in] params         LR-FHSS parameter structure
 * @param  [in] payload_length Length of application-layer payload
 *
 * @returns Time-on-air value in ms for LR-FHSS transmission
 */
uint32_t lr_fhss_get_time_on_air_in_ms( const lr_fhss_v1_params_t* params, uint16_t payload_length );

#ifdef __cplusplus
}
#endif

#endif  // LR_FHSS_MAC_H__

/* --- EOF ------------------------------------------------------------------ */
