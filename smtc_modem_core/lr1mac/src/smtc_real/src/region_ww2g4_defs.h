/**
 * \file      region_ww2g4_defs.h
 *
 * \brief     region_ww2g4_defs  abstraction layer definition
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

#ifndef REGION_WW2G4_DEFS_H
#define REGION_WW2G4_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#include "lr1mac_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// clang-format off
#define NUMBER_OF_CHANNEL_WW2G4            (16)
#define NUMBER_OF_BOOT_TX_CHANNEL_WW2G4    (3)             // define the number of channel at boot
#define JOIN_ACCEPT_DELAY1_WW2G4           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_WW2G4           (6)             // define in seconds
#define RECEIVE_DELAY1_WW2G4               (1)             // define in seconds
#define TX_POWER_EIRP_WW2G4                (10)            // define in dbm
#define MAX_TX_POWER_IDX_WW2G4             (7)             // index ex LinkADRReq
#define ADR_ACK_LIMIT_WW2G4                (64)
#define ADR_ACK_DELAY_WW2G4                (32)
#define ACK_TIMEOUT_WW2G4                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_WW2G4                      (2400000000)    // Hz
#define FREQMAX_WW2G4                      (2480000000)    // Hz
#define RX2_FREQ_WW2G4                     (2423000000)    // Hz
#define FREQUENCY_FACTOR_WW2G4             (200)           // MHz/200 when coded over 24 bits
#define RX2DR_INIT_WW2G4                   (0)
#define SYNC_WORD_PRIVATE_WW2G4            (0x12)
#define SYNC_WORD_PUBLIC_WW2G4             (0x21)
#define MIN_DR_WW2G4                       (0)
#define MAX_DR_WW2G4                       (7)
#define MIN_TX_DR_LIMIT_WW2G4              (0)
#define NUMBER_OF_TX_DR_WW2G4              (8)
#define DR_BITFIELD_SUPPORTED_WW2G4        (uint16_t)( ( 1 << DR7 ) | ( 1 << DR6 ) | \
                                                       ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define DEFAULT_TX_DR_BIT_FIELD_WW2G4      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define TX_PARAM_SETUP_REQ_SUPPORTED_WW2G4 (true)          // This mac command is required for ww2g4
#define NEW_CHANNEL_REQ_SUPPORTED_WW2G4    (true)
#define DTC_SUPPORTED_WW2G4                (false)
#define LBT_SUPPORTED_WW2G4                (false)
#define CF_LIST_SUPPORTED_WW2G4            (CF_LIST_FREQ)

// Class B
#define BEACON_DR_WW2G4                    (3)
#define BEACON_FREQ_WW2G4                  (2424000000)     // Hz
#define PING_SLOT_FREQ_WW2G4               (2424000000)     // Hz

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
typedef enum ww2g4_channels_bank_e
{
    BANK_0_WW2G4 = 0,  // 0 to 7 channels
    BANK_1_WW2G4 = 1,  // 8 to 15 channels
    BANK_MAX_WW2G4
} ww2g4_channels_bank_t;

typedef struct region_ww2g4_context_s
{
    uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_WW2G4];
    uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_WW2G4];
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_CHANNEL_WW2G4];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_WW2G4];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_WW2G4];
    uint8_t  channel_index_enabled[BANK_MAX_WW2G4];  // Contain the index of the activated channel only
    uint8_t  unwrapped_channel_mask[BANK_MAX_WW2G4];
} region_ww2g4_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * Default frequencies at boot
 */
#if defined( PERF_TEST_ENABLED )
static const uint32_t default_freq_ww2g4[] = { 2479000000, 2479000000, 2479000000 };
#else
static const uint32_t default_freq_ww2g4[] = { 2403000000, 2425000000, 2479000000 };
#endif

/**
 * Up/Down link data rates offset definition
 */
static const uint8_t datarate_offsets_ww2g4[8][6] = {
    { 0, 0, 0, 0, 0, 0 },  // DR 0
    { 1, 0, 0, 0, 0, 0 },  // DR 1
    { 2, 1, 0, 0, 0, 0 },  // DR 2
    { 3, 2, 1, 0, 0, 0 },  // DR 3
    { 4, 3, 2, 1, 0, 0 },  // DR 4
    { 5, 4, 3, 2, 1, 0 },  // DR 5
    { 6, 5, 4, 3, 2, 1 },  // DR 6
    { 7, 6, 5, 4, 3, 2 },  // DR 7
};

/**
 * @brief uplink darate backoff
 *
 */
static const uint8_t datarate_backoff_ww2g4[] = {
    0,  // DR0 -> DR0
    0,  // DR1 -> DR0
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    4,  // DR5 -> DR4
    5,  // DR6 -> DR5
    6   // DR7 -> DR6
};

static const uint8_t NUMBER_RX1_DR_OFFSET_WW2G4 =
    sizeof( datarate_offsets_ww2g4[0] ) / sizeof( datarate_offsets_ww2g4[0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_ww2g4[] = { 12, 11, 10, 9, 8, 7, 6, 5 };

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_ww2g4[] = { BW800, BW800, BW800, BW800, BW800, BW800, BW800, BW800 };

/**
 * Payload max size table definition in bytes
 */
static const uint8_t M_ww2g4[8] = { 59, 123, 228, 228, 228, 228, 228, 228 };

/**
 * Mobile long range datarate distribution
 * DR0: 20%,
 * DR1: 20%,
 * DR2: 30%,
 * DR3: 30%,
 * DR4:  0%,
 * DR5:  0%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_WW2G4[] = { 2, 2, 3, 3, 0, 0, 0, 0 };

/**
 * Mobile low power datarate distribution
 * DR0:  0%,
 * DR1:  0%,
 * DR2: 10%,
 * DR3: 30%,
 * DR4: 30%,
 * DR5: 30%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_WW2G4[] = { 0, 0, 1, 3, 3, 3, 0, 0 };

/**
 * Join datarate distribution
 * DR0:  5%,
 * DR1: 10%,
 * DR2: 15%,
 * DR3: 20%,
 * DR4: 20%,
 * DR5: 30%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t JOIN_DR_DISTRIBUTION_WW2G4[] = { 1, 2, 3, 4, 4, 6, 0, 0 };

/**
 * Default datarate distribution
 * DR0: 100%,
 * DR1:   0%,
 * DR2:   0%,
 * DR3:   0%,
 * DR4:   0%,
 * DR5:   0%,
 * DR6:   0%,
 * DR7:   0%
 */
static const uint8_t DEFAULT_DR_DISTRIBUTION_WW2G4[] = { 1, 0, 0, 0, 0, 0, 0, 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_WW2G4_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
