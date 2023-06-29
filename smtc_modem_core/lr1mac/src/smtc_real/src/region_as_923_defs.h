/**
 * \file      region_as_923_defs.h
 *
 * \brief     region_as_923_defs  abstraction layer definition
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

#ifndef REGION_AS_923_DEFS_H
#define REGION_AS_923_DEFS_H

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
#define NUMBER_OF_CHANNEL_AS_923            (16)
#define NUMBER_OF_BOOT_TX_CHANNEL_AS_923    (2)             // define the number of channel at boot
#define JOIN_ACCEPT_DELAY1_AS_923           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_AS_923           (6)             // define in seconds
#define RECEIVE_DELAY1_AS_923               (1)             // define in seconds
#define TX_POWER_EIRP_AS_923                (16)            // define in db
#define MAX_TX_POWER_IDX_AS_923             (7)             // index ex LinkADRReq
#define ADR_ACK_LIMIT_AS_923                (64)
#define ADR_ACK_DELAY_AS_923                (32)
#define ACK_TIMEOUT_AS_923                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_GRP1_AS_923                 (915000000)     // Hz
#define FREQMAX_GRP1_AS_923                 (928000000)     // Hz
#define FREQMIN_GRP2_AS_923                 (920000000)     // Hz
#define FREQMAX_GRP2_AS_923                 (923000000)     // Hz
#define FREQMIN_GRP3_AS_923                 (915000000)     // Hz
#define FREQMAX_GRP3_AS_923                 (921000000)     // Hz
#define FREQMIN_GRP4_AS_923                 (917000000)     // Hz
#define FREQMAX_GRP4_AS_923                 (920000000)     // Hz
#define FREQOFFSET_GRP1_AS_923              (int32_t)(0x00000000)    //      0 * 100 Hz, Group AS923-1
#define FREQOFFSET_GRP2_AS_923              (int32_t)(0xFFFFB9B0)    // -18000 * 100 Hz, Group AS923-2
#define FREQOFFSET_GRP3_AS_923              (int32_t)(0xFFFEFE30)    // -66000 * 100 Hz, Group AS923-3
#define FREQOFFSET_GRP4_AS_923              (int32_t)(0xFFFF1988)    // -59000 * 100 Hz, Group AS923-3
#define RX2_FREQ_AS_923                     (923200000)     // Hz
#define FREQUENCY_FACTOR_AS_923             (100)           // MHz/100 when coded over 24 bits
#define RX2DR_INIT_AS_923                   (2)
#define SYNC_WORD_PRIVATE_AS_923            (0x12)
#define SYNC_WORD_PUBLIC_AS_923             (0x34)
#define MIN_DR_AS_923                       (0)
#define MAX_DR_AS_923                       (7)
#define MIN_TX_DR_LIMIT_AS_923              (2)
#define DR_BITFIELD_SUPPORTED_AS_923        (uint16_t)( ( 1 << DR7 ) | ( 1 << DR6 ) | \
                                                        ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define DEFAULT_TX_DR_BIT_FIELD_AS_923      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define NUMBER_OF_TX_DR_AS_923              (8)
#define TX_PARAM_SETUP_REQ_SUPPORTED_AS_923 (true)
#define NEW_CHANNEL_REQ_SUPPORTED_AS_923    (true)
#define DTC_SUPPORTED_AS_923                (false)
#define LBT_SUPPORTED_AS_923                (true)
#define LBT_SNIFF_DURATION_MS_AS_923        (5)
#define LBT_THRESHOLD_DBM_AS_923            (int16_t)(-80)
#define LBT_BW_HZ_AS_923                    (200000)
#define CF_LIST_SUPPORTED_AS_923            (CF_LIST_FREQ)
#define UPLINK_DWELL_TIME_AS_923            (true)

// Class B
#define BEACON_FREQ_AS_923                  (923400000)     // Hz
#define BEACON_DR_AS_923                    (3)
#define PING_SLOT_FREQ_AS_923               (923400000)     // Hz

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
typedef enum as_923_channels_bank_e
{
    BANK_0_AS923 = 0,  // 0 to 7 channels
    BANK_1_AS923 = 1,  // 8 to 15 channels
    BANK_MAX_AS923
} as_923_channels_bank_t;

typedef struct region_as923_context_s
{
    uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_AS_923];
    uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_AS_923];
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_CHANNEL_AS_923];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_AS_923];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_AS_923];
    uint8_t  channel_index_enabled[BANK_MAX_AS923];   // Enable by Network
    uint8_t  unwrapped_channel_mask[BANK_MAX_AS923];  // Temp conf send by Network
} region_as923_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
static const uint8_t SYNC_WORD_GFSK_AS_923[] = { 0xC1, 0x94, 0xC1 };

/**
 * Default frequencies at boot
 */
static const uint32_t default_freq_as_923[] = { 923200000, 923400000 };

/**
 * Up/Down link data rates offset definition when no dwell time limitation
 * [dwell time][dr][dr_offset]
 */
static const uint8_t datarate_offsets_as_923[2][8][8] = { {                                // dwell time Off
                                                            { 0, 0, 0, 0, 0, 0, 1, 2 },    // DR 0
                                                            { 1, 0, 0, 0, 0, 0, 2, 3 },    // DR 1
                                                            { 2, 1, 0, 0, 0, 0, 3, 4 },    // DR 2
                                                            { 3, 2, 1, 0, 0, 0, 4, 5 },    // DR 3
                                                            { 4, 3, 2, 1, 0, 0, 5, 6 },    // DR 4
                                                            { 5, 4, 3, 2, 1, 0, 6, 7 },    // DR 5
                                                            { 6, 5, 4, 3, 2, 1, 7, 7 },    // DR 6
                                                            { 7, 6, 5, 4, 3, 2, 7, 7 } },  // DR 7

                                                          {
                                                              // dwell time On
                                                              { 2, 2, 2, 2, 2, 2, 2, 2 },  // DR 0
                                                              { 2, 2, 2, 2, 2, 2, 2, 3 },  // DR 1
                                                              { 2, 2, 2, 2, 2, 2, 3, 4 },  // DR 2
                                                              { 3, 2, 2, 2, 2, 2, 4, 5 },  // DR 3
                                                              { 4, 3, 2, 2, 2, 2, 5, 6 },  // DR 4
                                                              { 5, 4, 3, 2, 2, 2, 6, 7 },  // DR 5
                                                              { 6, 5, 4, 3, 2, 2, 7, 7 },  // DR 6
                                                              { 7, 6, 5, 4, 3, 2, 7, 7 },  // DR 7}
                                                          } };

/**
 * @brief uplink datarate backoff
 *
 */
static const uint8_t datarate_backoff_as_923[2][8] = { {
                                                           // [0][dr_backoff] dwell time Off
                                                           0,  // DR0 -> DR0
                                                           0,  // DR1 -> DR0
                                                           1,  // DR2 -> DR1
                                                           2,  // DR3 -> DR2
                                                           3,  // DR4 -> DR3
                                                           4,  // DR5 -> DR4
                                                           5,  // DR6 -> DR5
                                                           6   // DR7 -> DR6
                                                       },
                                                       {
                                                           // [1][dr_backoff] dwell time On
                                                           2,  // NA
                                                           2,  // NA
                                                           2,  // DR2 -> DR2
                                                           2,  // DR3 -> DR2
                                                           3,  // DR4 -> DR3
                                                           4,  // DR5 -> DR4
                                                           5,  // DR6 -> DR5
                                                           6   // DR7 -> DR6
                                                       } };

static const uint8_t NUMBER_RX1_DR_OFFSET_AS_923 = sizeof( datarate_offsets_as_923[0] ) /
                                                   sizeof( datarate_offsets_as_923[0][0] ) /
                                                   sizeof( datarate_offsets_as_923[0][0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_as_923[] = { 12, 11, 10, 9, 8, 7, 7 };

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_as_923[] = { BW125, BW125, BW125, BW125, BW125, BW125, BW250 };

/**
 * Payload max size table definition in bytes
 */
static const uint8_t M_as_923[2][8] = { { 59, 59, 123, 123, 250, 250, 250, 250 },  // [0][dr] dwell time Off
                                        { 0, 0, 19, 61, 133, 250, 250, 250 } };    // [1][dr] dwell time On

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
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_AS_923[] = { 2, 3, 3, 3, 0, 0, 0, 0 };

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
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_AS_923[] = { 0, 0, 1, 3, 3, 3, 0, 0 };

/**
 * Join datarate distribution
 * DR0:  0%,
 * DR1:  0%,
 * DR2: 20%,
 * DR3: 25%,
 * DR4: 25%,
 * DR5: 30%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t JOIN_DR_DISTRIBUTION_AS_923[] = { 0, 0, 4, 5, 5, 6, 0, 0 };

/**
 * Default datarate distribution
 * DR0:   0%,
 * DR1:   0%,
 * DR2: 100%,
 * DR3:   0%,
 * DR4:   0%,
 * DR5:   0%,
 * DR6:   0%,
 * DR7:   0%
 */
static const uint8_t DEFAULT_DR_DISTRIBUTION_AS_923[] = { 0, 0, 1, 0, 0, 0, 0, 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_AS_923_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
