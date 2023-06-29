/**
 * \file      region_cn_470_rp_1_0_defs.h
 *
 * \brief     region_cn_470_rp_1_0_defs abstraction layer definition
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

#ifndef REGION_CN470_RP_1_0_DEFS_H
#define REGION_CN470_RP_1_0_DEFS_H

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

#if defined( HYBRID_CN470_MONO_CHANNEL )
extern uint32_t freq_tx_cn470_mono_channel_mhz;
#endif

/* clang-format off */
#define NUMBER_OF_TX_CHANNEL_CN_470_RP_1_0         (96)            // Max Tx channels required for a group
#define NUMBER_OF_RX_CHANNEL_CN_470_RP_1_0         (48)            // Max Rx channels required for a group
#define JOIN_ACCEPT_DELAY1_CN_470_RP_1_0           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_CN_470_RP_1_0           (6)             // define in seconds
#define RECEIVE_DELAY1_CN_470_RP_1_0               (1)             // define in seconds
#if defined( LR11XX )
#define TX_POWER_EIRP_CN_470_RP_1_0                (19)            // define in dbm
#else
// This value must be the MIN of MAX supported by the region and the radio, region is 19dBm EIRP but radio is 14dBm ERP (+2 to EIRP)
#define TX_POWER_EIRP_CN_470_RP_1_0                (16)            // define in dbm
#endif
#define MAX_TX_POWER_IDX_CN_470_RP_1_0             (7)            // index ex LinkADRReq
#define ADR_ACK_LIMIT_CN_470_RP_1_0                (64)
#define ADR_ACK_DELAY_CN_470_RP_1_0                (32)
#define ACK_TIMEOUT_CN_470_RP_1_0                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_CN_470_RP_1_0                      (470000000)     // Hz
#define FREQMAX_CN_470_RP_1_0                      (510000000)     // Hz
#define RX2_FREQ_CN_470_RP_1_0                     (505300000)     // Hz
#define FREQUENCY_FACTOR_CN_470_RP_1_0             (100)           // MHz/100 when coded over 24 bits
#define SYNC_WORD_PRIVATE_CN_470_RP_1_0            (0x12)
#define SYNC_WORD_PUBLIC_CN_470_RP_1_0             (0x34)
#define MIN_TX_DR_CN_470_RP_1_0                    (0)
#define MAX_TX_DR_CN_470_RP_1_0                    (5)
#define NUMBER_OF_TX_DR_CN_470_RP_1_0              (6)
#define DR_BITFIELD_SUPPORTED_CN_470_RP_1_0        (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define DEFAULT_TX_DR_BIT_FIELD_CN_470_RP_1_0      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )

#define MIN_TX_DR_LIMIT_CN_470_RP_1_0              (0)
#define MIN_RX_DR_CN_470_RP_1_0                    (0)
#define RX2DR_INIT_CN_470_RP_1_0                   (0)

#define MAX_RX_DR_CN_470_RP_1_0                    (5)
#define TX_PARAM_SETUP_REQ_SUPPORTED_CN_470_RP_1_0 (false)         // This mac command is NOT required for CN470
#define NEW_CHANNEL_REQ_SUPPORTED_CN_470_RP_1_0    (false)         // This mac command is NOT required for CN470
#define DTC_SUPPORTED_CN_470_RP_1_0                (false)

#define LBT_SUPPORTED_CN_470_RP_1_0                (true)
#define LBT_SNIFF_DURATION_MS_CN_470_RP_1_0        (5)             // TODO value must be checked
#define LBT_THRESHOLD_DBM_CN_470_RP_1_0            (int16_t)(-80)  // TODO value must be checked
#define LBT_BW_HZ_CN_470_RP_1_0                    (200000)        // TODO value must be checked

#define CF_LIST_SUPPORTED_CN_470_RP_1_0            (CF_LIST_CH_MASK)

#define DEFAULT_TX_FREQ_CN_470_RP_1_0              (470300000) // Hz
#define DEFAULT_RX_FREQ_CN_470_RP_1_0              (500300000) // Hz

#define DEFAULT_TX_STEP_CN_470_RP_1_0              (200000)    // Hz
#define DEFAULT_RX_STEP_CN_470_RP_1_0              (200000)    // Hz

// Class B
#define BEACON_DR_CN_470_RP_1_0                    (2)
#define BEACON_FREQ_START_CN_470_RP_1_0            (508300000) // Hz
#define BEACON_STEP_CN_470_RP_1_0                  (200000)    // Hz
#define PING_SLOT_FREQ_START_CN_470_RP_1_0         (923300000) // Hz
#define PING_SLOT_STEP_CN_470_RP_1_0               (200000)    // Hz
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t SYNC_WORD_GFSK_CN_470_RP_1_0[] = { 0xC1, 0x94, 0xC1 };

/**
 * Up/Down link data rates offset definition
 */
static const uint8_t datarate_offsets_cn_470_rp_1_0[8][6] = {
    { 0, 0, 0, 0, 0, 0 },  // DR 0
    { 1, 0, 0, 0, 0, 0 },  // DR 1
    { 2, 1, 0, 0, 0, 0 },  // DR 2
    { 3, 2, 1, 0, 0, 0 },  // DR 3
    { 4, 3, 2, 1, 0, 0 },  // DR 4
    { 5, 4, 3, 2, 1, 0 },  // DR 5
};

/**
 * @brief uplink datarate backoff
 *
 */
static const uint8_t datarate_backoff_cn_470_rp_1_0[] = {
    0,  // DR0 -> DR0
    0,  // DR1 -> DR0
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    4   // DR5 -> DR4
};

static const uint8_t NUMBER_RX1_DR_OFFSET_CN_470_RP_1_0 =
    sizeof( datarate_offsets_cn_470_rp_1_0[0] ) / sizeof( datarate_offsets_cn_470_rp_1_0[0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_cn_470_rp_1_0[] = { 12, 11, 10, 9, 8, 7 };

/**
 *
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_cn_470_rp_1_0[] = { BW125, BW125, BW125, BW125, BW125, BW125 };

/**
 * Payload max size table definition in bytes
 */
static const uint8_t M_cn_470_rp_1_0[] = { 59, 59, 59, 123, 250, 250 };

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
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_CN_470_RP_1_0[] = { 2, 2, 3, 3, 0, 0 };

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
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_CN_470_RP_1_0[] = { 0, 0, 1, 3, 3, 3 };

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
static const uint8_t JOIN_DR_DISTRIBUTION_CN_470_RP_1_0[] = { 1, 2, 3, 4, 4, 6 };

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
static const uint8_t DEFAULT_DR_DISTRIBUTION_CN_470_RP_1_0[] = { 1, 0, 0, 0, 0, 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
typedef enum cn_470_rp_1_0_channels_bank_e
{
    BANK_0_125_CN470_RP_1_0  = 0,   //  0 -  7
    BANK_1_125_CN470_RP_1_0  = 1,   //  8 - 15
    BANK_2_125_CN470_RP_1_0  = 2,   // 16 - 23
    BANK_3_125_CN470_RP_1_0  = 3,   // 24 - 31
    BANK_4_125_CN470_RP_1_0  = 4,   // 32 - 39
    BANK_5_125_CN470_RP_1_0  = 5,   // 40 - 47
    BANK_6_125_CN470_RP_1_0  = 6,   // 48 - 55
    BANK_7_125_CN470_RP_1_0  = 7,   // 56 - 63
    BANK_8_125_CN470_RP_1_0  = 8,   // 64 - 71
    BANK_9_125_CN470_RP_1_0  = 9,   // 72 - 79
    BANK_10_125_CN470_RP_1_0 = 10,  // 80 - 87
    BANK_11_125_CN470_RP_1_0 = 11,  // 88 - 95
    BANK_MAX_CN470_RP_1_0
} cn_470_rp_1_0_channels_bank_t;

typedef struct region_cn470_rp_1_0_context_s
{
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_TX_CHANNEL_CN_470_RP_1_0];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_CN_470_RP_1_0];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_CN_470_RP_1_0];
    uint8_t  channel_index_enabled[BANK_MAX_CN470_RP_1_0];  // Contain the index of the activated channel only
    uint8_t  unwrapped_channel_mask[BANK_MAX_CN470_RP_1_0];

    cn_470_rp_1_0_channels_bank_t snapshot_bank_tx_mask;
} region_cn470_rp_1_0_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_CN470_RP_1_0_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
