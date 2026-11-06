/*!
 * @file      lr20xx_rttof_types.h
 *
 * @brief     Round-Trip Time of Flight (RTToF) driver types for LR20XX
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

#ifndef LR20XX_RTTOF_TYPES_H
#define LR20XX_RTTOF_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

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

/**
 * @brief RTToF mode
 */
typedef enum lr20xx_rttof_mode_e
{
    LR20XX_RTTOF_MODE_NORMAL   = 0x00,  //!<
    LR20XX_RTTOF_MODE_EXTENDED = 0x01,  //!<
} lr20xx_rttof_mode_t;

/**
 * @brief RTToF spy mode
 */
typedef enum lr20xx_rttof_spy_mode_e
{
    LR20XX_RTTOF_SPY_MODE_DISABLED = 0x00,  //!< RTToF spy mode disabled
    LR20XX_RTTOF_SPY_MODE_ENABLED  = 0x01,  //!< RTToF spy mode enabled
} lr20xx_rttof_spy_mode_t;

/*!
 * @brief RTToF parameters
 */
typedef struct lr20xx_rttof_params_s
{
    lr20xx_rttof_mode_t     mode;       //!< RTToF mode
    lr20xx_rttof_spy_mode_t spy_mode;   //!< RTToF spy mode
    uint8_t                 nb_symbol;  //!< Number of symbols sent in the RTToF response - range within [0:63]
} lr20xx_rttof_params_t;

/*!
 * @brief RTToF results
 */
typedef struct lr20xx_rttof_results_s
{
    int32_t val;   //!< RTToF value
    int8_t  rssi;  //!< RSSI in dBm
} lr20xx_rttof_results_t;

/*!
 * @brief RTToF results - extended
 */
typedef struct lr20xx_rttof_results_extended_s
{
    lr20xx_rttof_results_t res1;  //!< RTToF result index 1
    lr20xx_rttof_results_t res2;  //!< RTToF result index 2
} lr20xx_rttof_results_extended_t;

/*!
 * @brief RTToF statistics
 */
typedef struct lr20xx_rttof_stats_s
{
    uint16_t exchange_valid;  //!< Number of valid responses received in Manager role
    uint16_t request_valid;   //!< Number of valid requests received in Subordinate role
    uint16_t response_done;   //!< Number of responses to valid request sent in Subordinate role
    uint16_t timeout;  //!< Number of timeout - no response received in Manager role / no extended request received in
                       //!< Subordinate role
    uint16_t request_discarded;  //!< Number of requests discarded in Subordinate role
} lr20xx_rttof_stats_t;

/**
 * @brief Role of Timing Synchronization feature
 */
typedef enum
{
    LR20XX_RTTOF_TIMING_SYNCHRONIZATION_DISABLE   = 0x00,  //!< Timing Synchronization disabled
    LR20XX_RTTOF_TIMING_SYNCHRONIZATION_INITIATOR = 0x01,  //!< Initiator Timing Synchronization
    LR20XX_RTTOF_TIMING_SYNCHRONIZATION_RESPONDER = 0x02,  //!< Responder Timing synchronization
} lr20xx_rttof_timing_synchronization_role_t;

/**
 * @brief DIO pins
 */
typedef enum lr20xx_rttof_dio_e
{
    LR20XX_RTTOF_DIO_5  = 0x05,
    LR20XX_RTTOF_DIO_6  = 0x06,
    LR20XX_RTTOF_DIO_7  = 0x07,
    LR20XX_RTTOF_DIO_8  = 0x08,
    LR20XX_RTTOF_DIO_9  = 0x09,
    LR20XX_RTTOF_DIO_10 = 0x0A,
    LR20XX_RTTOF_DIO_11 = 0x0B,
} lr20xx_rttof_dio_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RTTOF_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
