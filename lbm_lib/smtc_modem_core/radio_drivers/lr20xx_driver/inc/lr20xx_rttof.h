/*!
 * @file      lr20xx_rttof.h
 *
 * @brief     Round-Trip Time of Flight (RTToF) driver definition for LR20XX
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

#ifndef LR20XX_RTTOF_H
#define LR20XX_RTTOF_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "lr20xx_rttof_types.h"
#include "lr20xx_radio_lora_types.h"
#include "lr20xx_status.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Set the RTToF responder address and the number of meaningful bytes
 *
 * @param [in] context Chip implementation context
 * @param [in] address Responder address to be configured - LSB checked first if \p length < 4 (default: 0x00000019)
 * @param [in] length Number of bytes of \p address taken into account for matching - range [0:4] (default: 4)
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_set_responder_address( const void* context, const uint32_t address, const uint8_t length );

/*!
 * @brief Set the RTToF initiator address
 *
 * @param [in] context Chip implementation context
 * @param [in] address Initiator address to be configured (default: 0x00000019)
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_set_initiator_address( const void* context, const uint32_t address );

/*!
 * @brief Configure the Tx->Rx delay for RTToF calibration
 *
 * @param [in] context Chip implementation context
 * @param [in] delay_in_rtc_step Delay between Tx and Rx in RTC step
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_set_tx_rx_delay( const void* context, uint32_t delay_in_rtc_step );

/*!
 * @brief Get results from a RTToF operation
 *
 * @param [in] context Chip implementation context
 * @param [out] result Structure holding the result
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_get_results( const void* context, lr20xx_rttof_results_t* result );

/*!
 * @brief Get results from a RTToF operation
 *
 * @param [in] context Chip implementation context
 * @param [out] result Structure holding the result
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_get_results_extended( const void* context, lr20xx_rttof_results_extended_t* result );

/*!
 * @brief Get gain steps from a RTToF operation in extended mode
 *
 * @param [in] context Chip implementation context
 * @param [out] gain_step_1 Gain step corresponding to the the response to the first request
 * @param [out] gain_step_2 Gain step corresponding to the the response to the second request
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_get_gain_steps_extended( const void* context, uint8_t* gain_step_1, uint8_t* gain_step_2 );

/*!
 * @brief Configure the RTToF parameters
 *
 * @param [in] context Chip implementation context
 * @param [in] params RTToF parameters
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_set_params( const void* context, const lr20xx_rttof_params_t* params );

/*!
 * @brief Get statistics for RTToF operation
 *
 * @remark For extended RTToF mode, counters are incremented twice - one for each request / response
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Structure holding the statistics
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_rttof_get_stats( const void* context, lr20xx_rttof_stats_t* stats );

/**
 * @brief Configure the Timing Synchronization feature
 *
 * The Timing Synchronization feature allows to propagate a pulse from a Initiator Timing Synchronization device to one
 * or more Responder Timing Synchronization devices
 *
 * @param context Chip implementation context
 * @param role Configure the role of the Timing Synchronization
 * @param dio The DIO used to detect the pulse (if @p role is @ref LR20XX_RTTOF_TIMING_SYNCHRONIZATION_INITIATOR) or to
 * propagate the pulse (if @p role is LR20XX_RTTOF_TIMING_SYNCHRONIZATION_RESPONDER)
 * @return lr20xx_status_t
 */
lr20xx_status_t lr20xx_rttof_configure_timing_synchronization( const void*                                context,
                                                               lr20xx_rttof_timing_synchronization_role_t role,
                                                               lr20xx_rttof_dio_t                         dio );

/**
 * @brief Convert the raw distance result obtained from the device to a distance result [m].
 *
 * The input argument @p raw_distance of this function is the field @ref
 * lr20xx_rttof_results_t::val obtained from the output argument of @ref
 * lr20xx_rttof_get_results or @ref lr20xx_rttof_get_results_extended.
 *
 * @param [in] rttof_bw Bandwidth used during RTToF
 * @param [in] raw_distance The raw distance value obtained from a call to @ref lr20xx_rttof_get_results or @ref
 * lr20xx_rttof_get_results_extended
 *
 * @returns int32_t Distance result [m]
 *
 * @see lr20xx_rttof_get_results and lr20xx_rttof_get_results_extended
 *
 * @note The caller must ensure that the @p rttof_bw parameter is a value of @ref lr20xx_radio_lora_bw_t.
 */
int32_t lr20xx_rttof_distance_raw_to_meter( lr20xx_radio_lora_bw_t rttof_bw, const int32_t raw_distance );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RTTOF_H

/* --- EOF ------------------------------------------------------------------ */
