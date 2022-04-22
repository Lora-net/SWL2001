/*!
 * @file      lr11xx_lr_fhss.h
 *
 * @brief     LR_FHSS driver definition for LR11XX
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

#ifndef LR11XX_LR_FHSS_H
#define LR11XX_LR_FHSS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_lr_fhss_types.h"
#include "lr11xx_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Length, in bytes, of a LR-FHSS sync word
 */
#define LR_FHSS_SYNC_WORD_BYTES ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize the LR_FHSS
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_lr_fhss_init( const void* context );

/*!
 * @brief Configure a payload to be sent with LR_FHSS
 *
 * When calling this method, lr11xx_lr_fhss_set_sync_word is implicitely called to configure the sync word.
 * Note that the syncword must be 4 bytes long.
 *
 * @param [in] context Chip implementation context
 * @param [in] lr_fhss_params Parameter configuration structure of the LRFHSS
 * @param [in] hop_sequence_id Seed used to derive the hopping sequence pattern. Only the nine LSBs are taken into
 * account
 * @param [in] payload The payload to send. It is the responsibility of the caller to ensure that this references an
 * array containing at least payload_length elements
 * @param [in] payload_length The length of the payload
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_lr_fhss_build_frame( const void* context, const lr11xx_lr_fhss_params_t* lr_fhss_params,
                                            uint16_t hop_sequence_id, const uint8_t* payload, uint8_t payload_length );

/*!
 * @brief Get the time on air in ms for LR-FHSS transmission
 *
 * @param [in]  params         LR11XX LR-FHSS parameter structure
 * @param [in]  payload_length Length of application-layer payload
 *
 * @returns Time-on-air value in ms for LR-FHSS transmission
 */
uint32_t lr11xx_lr_fhss_get_time_on_air_in_ms( const lr11xx_lr_fhss_params_t* params, uint16_t payload_length );

/**
 * @brief Return the number of hop sequences available using the given parameters
 *
 * @param [in] lr_fhss_params Parameter configuration structure of the LRFHSS
 *
 * @return Returns the number of valid hop sequences (512 or 384)
 */
unsigned int lr11xx_lr_fhss_get_hop_sequence_count( const lr11xx_lr_fhss_params_t* lr_fhss_params );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_LR_FHSS_H

/* --- EOF ------------------------------------------------------------------ */
