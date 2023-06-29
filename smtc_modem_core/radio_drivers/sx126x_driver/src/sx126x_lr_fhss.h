/**
 * @file      sx126x_lr_fhss.h
 *
 * @brief     SX126x LR-FHSS driver API
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

#ifndef SX126X_LR_FHSS_H__
#define SX126X_LR_FHSS_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "sx126x.h"
#include "lr_fhss_mac.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define SX126X_LR_FHSS_REG_CTRL ( 0x0385 )
#define SX126X_LR_FHSS_REG_PACKET_LEN ( 0x0386 )
#define SX126X_LR_FHSS_REG_NUM_HOPS ( 0x0387 )
#define SX126X_LR_FHSS_REG_NUM_SYMBOLS_0 ( 0x0388 )
#define SX126X_LR_FHSS_REG_FREQ_0 ( 0x038A )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SX126X LR-FHSS LR-FHSS parameter definition
 */
typedef struct sx126x_lr_fhss_params_s
{
    lr_fhss_v1_params_t lr_fhss_params;
    uint32_t            center_freq_in_pll_steps; /**< Center frequency in transceiver units */
    int8_t              device_offset;  //<! Per device offset to avoid collisions over the air. Possible values:
                                        //<! - if (lr_fhss_params.grid == LR_FHSS_V1_GRID_25391_HZ): [-26, 25]
                                        //<! - if (lr_fhss_params.grid == LR_FHSS_V1_GRID_3906_HZ): [-4, 3]
} sx126x_lr_fhss_params_t;

/**
 * @brief SX126X LR-FHSS LR-FHSS state definition
 */
typedef struct sx126x_lr_fhss_state_s
{
    lr_fhss_hop_params_t hop_params;
    lr_fhss_digest_t     digest;
    uint32_t             next_freq_in_pll_steps; /**< Frequency that will be used on next hop */
    uint16_t             lfsr_state;             /**< LFSR state for hop sequence generation */
    uint8_t              current_hop;            /**< Index of the current hop */
} sx126x_lr_fhss_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LR-FHSS packet type and modulation parameters
 *
 * @param [in] context Chip implementation context
 * @param [in] params sx126x LR-FHSS parameter structure
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_init( const void* context, const sx126x_lr_fhss_params_t* params );

/**
 * @brief Get the delay in microsecond between the last bit sent and the TX done interrupt
 *
 * @param [in]  params          sx126x LR-FHSS parameter structure
 * @param [in]  payload_length  Length of application-layer payload
 *
 * @returns Delay in microseconds
 */
uint16_t sx126x_lr_fhss_get_bit_delay_in_us( const sx126x_lr_fhss_params_t* params, uint16_t payload_length );

/**
 * @brief Check the parameters, and in case of success, generate the digest summary which contains important size info
 *
 * @param [in]  params          sx126x LR-FHSS parameter structure
 * @param [in]  hop_sequence_id Specifies which hop sequence to use
 * @param [in]  payload_length  Length of application-layer payload
 * @param [out] state           sx126x LR-FHSS state structure that will be initialized by this function
 *
 * @remark It is not necessary to explicitly call this function if the helper function @ref sx126x_lr_fhss_build_frame
 * is used. If the preprocessor symbol HOP_AT_CENTER_FREQ is defined, hopping will be performed with PA ramp
 * up/down, but without actually changing frequencies.
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_process_parameters( const sx126x_lr_fhss_params_t* params, uint16_t hop_sequence_id,
                                                   uint16_t payload_length, sx126x_lr_fhss_state_t* state );

/**
 * @brief Sent the initial hopping confifguration to the radio
 *
 * @param [in]  context   Chip implementation context
 * @param [in]  params    sx126x LR-FHSS parameter structure
 * @param [in]  state     sx126x LR-FHSS state structure
 *
 * @remark It is not necessary to explicitly call this function if the helper function @ref sx126x_lr_fhss_build_frame
 * is used. If the preprocessor symbol HOP_AT_CENTER_FREQ is defined, hopping will be performed with PA ramp
 * up/down, but without actually changing frequencies.
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_write_hop_sequence_head( const void* context, const sx126x_lr_fhss_params_t* params,
                                                        sx126x_lr_fhss_state_t* state );

/**
 * @brief Write physical LR-FHSS payload to radio
 *
 * @param [in]  context         Chip implementation context
 * @param [in]  state           sx126x LR-FHSS state structure
 * @param [in]  payload         Array containing application-layer payload
 *
 * @remark It is not necessary to explicitly call this function if the helper function @ref sx126x_lr_fhss_build_frame
 * is used.
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_write_payload( const void* context, const sx126x_lr_fhss_state_t* state,
                                              const uint8_t* payload );

/**
 * @brief Check parameter validity, build a frame, then send it
 *
 * @param [in]  context         Chip implementation context
 * @param [in]  params          sx126x LR-FHSS parameter structure
 * @param [in]  state           sx126x LR-FHSS state structure
 * @param [in]  hop_sequence_id Specifies which hop sequence to use
 * @param [in]  payload         Array containing application-layer payload
 * @param [in]  payload_length  Length of application-layer payload
 * @param [out] first_frequency_in_pll_steps If non-NULL, provides the frequency that will be used on the first hop
 *
 * @remark This helper function calls the @ref sx126x_lr_fhss_process_parameters,
 * lr_fhss_build_frame, @ref sx126x_lr_fhss_write_hop_sequence_head, and @ref sx126x_lr_fhss_write_payload
 * functions. If the preprocessor symbol HOP_AT_CENTER_FREQ is defined, hopping will be performed with PA ramp
 * up/down, but without actually changing frequencies.
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_build_frame( const void* context, const sx126x_lr_fhss_params_t* params,
                                            sx126x_lr_fhss_state_t* state, uint16_t hop_sequence_id,
                                            const uint8_t* payload, uint16_t payload_length,
                                            uint32_t* first_frequency_in_pll_steps );

/**
 * @brief Perform an actual frequency hop
 *
 * @param [in]  context        Chip implementation context
 * @param [in]  params         sx126x LR-FHSS parameter structure
 * @param [in]  state          sx126x LR-FHSS state structure
 *
 * @remark This should be called to respond to the SX126X_IRQ_LR_FHSS_HOP interrupt. If the preprocessor symbol
 * HOP_AT_CENTER_FREQ is defined, hopping will be performed with PA ramp up/down, but without actually
 * changing frequencies.
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_handle_hop( const void* context, const sx126x_lr_fhss_params_t* params,
                                           sx126x_lr_fhss_state_t* state );

/**
 * @brief Indicate to the radio that frequency hopping is no longer needed
 *
 * @param [in]  context        Chip implementation context
 * @param [in]  params         sx126x LR-FHSS parameter structure
 * @param [in]  state          sx126x LR-FHSS state structure
 *
 * @remark The should be called to respond to the SX126X_IRQ_TX_DONE interrupt following LR-FHSS transmission.
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_lr_fhss_handle_tx_done( const void* context, const sx126x_lr_fhss_params_t* params,
                                               sx126x_lr_fhss_state_t* state );

/**
 * @brief Get the time on air in ms for LR-FHSS transmission
 *
 * @param [in]  params         sx126x LR-FHSS parameter structure
 * @param [in]  payload_length Length of application-layer payload
 *
 * @returns Time-on-air value in ms for LR-FHSS transmission
 */
static inline uint32_t sx126x_lr_fhss_get_time_on_air_in_ms( const sx126x_lr_fhss_params_t* params,
                                                             uint16_t                       payload_length )
{
    return lr_fhss_get_time_on_air_in_ms( &params->lr_fhss_params, payload_length );
}

/**
 * @brief Return the number of hop sequences available using the given parameters
 *
 * @param [in]  params         sx126x LR-FHSS parameter structure
 *
 * @return Returns the number of valid hop sequences (512 or 384)
 */
static inline unsigned int sx126x_lr_fhss_get_hop_sequence_count( const sx126x_lr_fhss_params_t* params )
{
    return lr_fhss_get_hop_sequence_count( &params->lr_fhss_params );
}

#ifdef __cplusplus
}
#endif

#endif  // SX126X_LR_FHSS_H__

/* --- EOF ------------------------------------------------------------------ */
