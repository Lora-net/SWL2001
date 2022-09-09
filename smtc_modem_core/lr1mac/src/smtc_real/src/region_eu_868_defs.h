/**
 * \file      region_eu_868_defs.h
 *
 * \brief     region_eu_868_defs  abstraction layer definition
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

#ifndef REGION_EU_868_DEFS_H
#define REGION_EU_868_DEFS_H

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
#define NUMBER_OF_CHANNEL_EU_868            (16)
#define NUMBER_OF_BOOT_TX_CHANNEL_EU_868    (3)             // define the number of channel at boot
#define JOIN_ACCEPT_DELAY1_EU_868           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_EU_868           (6)             // define in seconds
#define RECEIVE_DELAY1_EU_868               (1)             // define in seconds
#define TX_POWER_EIRP_EU_868                (16)            // define in db
#define MAX_TX_POWER_IDX_EU_868             (7)             // index ex LinkADRReq
#define ADR_ACK_LIMIT_EU_868                (64)
#define ADR_ACK_DELAY_EU_868                (32)
#define ACK_TIMEOUT_EU_868                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_EU_868                      (863000000)     // Hz
#define FREQMAX_EU_868                      (870000000)     // Hz
#define RX2_FREQ_EU_868                     (869525000)     // Hz
#define FREQUENCY_FACTOR_EU_868             (100)           // MHz/100 when coded over 24 bits
#define RX2DR_INIT_EU_868                   (0)
#define SYNC_WORD_PRIVATE_EU_868            (0x12)
#define SYNC_WORD_PUBLIC_EU_868             (0x34)
#define MIN_TX_DR_EU_868                    (0)
#define MAX_TX_DR_EU_868                    (11)
#define MIN_TX_DR_LIMIT_EU_868              (0)
#define NUMBER_OF_TX_DR_EU_868              (12)
#define MIN_RX_DR_EU_868                    (0)
#define MAX_RX_DR_EU_868                    (7)

#if defined( RP2_101 )
#define DR_BITFIELD_SUPPORTED_EU_868        (uint16_t)( ( 1 << DR7 ) | ( 1 << DR6 ) | \
                                                        ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#elif defined( RP2_103 )
#define DR_BITFIELD_SUPPORTED_EU_868        (uint16_t)( ( 1 << DR11 ) | ( 1 << DR10 ) | ( 1 << DR9 ) | ( 1 << DR8 ) | ( 1 << DR7 ) | ( 1 << DR6 ) | \
                                                        ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#endif

#define DEFAULT_TX_DR_BIT_FIELD_EU_868      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define TX_PARAM_SETUP_REQ_SUPPORTED_EU_868 (false)         // This mac command is NOT required for EU868
#define NEW_CHANNEL_REQ_SUPPORTED_EU_868    (true)
#define DTC_SUPPORTED_EU_868                (true)
#define LBT_SUPPORTED_EU_868                (false)
#define LBT_SNIFF_DURATION_MS_EU_868        (5)
#define LBT_THRESHOLD_DBM_EU_868            (int16_t)(-80)
#define LBT_BW_HZ_EU_868                    (200000)
#define CF_LIST_SUPPORTED_EU_868            (CF_LIST_FREQ)

// Class B
#define BEACON_DR_EU_868                    (3)
#define BEACON_FREQ_EU_868                  (869525000)     // Hz
#define PING_SLOT_FREQ_EU_868               (869525000)     // Hz

// LR-FHSS
#define LR_FHSS_NA                          (0xFFFFFFFF) // LR-FHSS Not Applicable
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
typedef enum eu_868_channels_bank_e
{
    BANK_0_EU868 = 0,  // 0 to 7 channels
    BANK_1_EU868 = 1,  // 8 to 15 channels
    BANK_MAX_EU868
} eu_868_channels_bank_t;

/**
 * Bands enumeration
 */
typedef enum region_eu_868_band_e
{
    BAND_EU868_0 = 0,
    BAND_EU868_1,
    BAND_EU868_2,
    BAND_EU868_3,
    BAND_EU868_4,
    BAND_EU868_5,
    BAND_EU868_MAX
} region_eu_868_band_t;

typedef struct region_eu868_context_s
{
    uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_EU_868];
    uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_EU_868];
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_CHANNEL_EU_868];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_EU_868];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_EU_868];
    uint8_t  channel_index_enabled[BANK_MAX_EU868];   // Enable by Network
    uint8_t  unwrapped_channel_mask[BANK_MAX_EU868];  // Temp conf send by Network
} region_eu868_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t SYNC_WORD_GFSK_EU_868[]    = { 0xC1, 0x94, 0xC1 };
static const uint8_t SYNC_WORD_LR_FHSS_EU_868[] = { 0x2C, 0x0F, 0x79, 0x95 };

/**
 * Default frequencies at boot
 */
static const uint32_t default_freq_eu_868[] = { 868100000, 868300000, 868500000 };

/**
 * Up/Down link data rates offset definition
 */
static const uint8_t datarate_offsets_eu_868[12][6] = {
    { 0, 0, 0, 0, 0, 0 },  // DR 0
    { 1, 0, 0, 0, 0, 0 },  // DR 1
    { 2, 1, 0, 0, 0, 0 },  // DR 2
    { 3, 2, 1, 0, 0, 0 },  // DR 3
    { 4, 3, 2, 1, 0, 0 },  // DR 4
    { 5, 4, 3, 2, 1, 0 },  // DR 5
    { 6, 5, 4, 3, 2, 1 },  // DR 6
    { 7, 6, 5, 4, 3, 2 },  // DR 7
    { 1, 0, 0, 0, 0, 0 },  // DR 8
    { 2, 1, 0, 0, 0, 0 },  // DR 9
    { 1, 0, 0, 0, 0, 0 },  // DR 10
    { 2, 1, 0, 0, 0, 0 },  // DR 11
};

/**
 * @brief uplink darate backoff
 *
 */
static const uint8_t datarate_backoff_eu_868[] = {
    0,  // DR0 -> DR0
    0,  // DR1 -> DR0
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    4,  // DR5 -> DR4
    5,  // DR6 -> DR5
    6,  // DR7 -> DR6
    0,  // DR8 -> DR0
    8,  // DR9 -> DR8
    0,  // DR10 -> DR0
    10  // DR11 -> DR10
};

static const uint8_t NUMBER_RX1_DR_OFFSET_EU_868 =
    sizeof( datarate_offsets_eu_868[0] ) / sizeof( datarate_offsets_eu_868[0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_eu_868[] = { 12, 11, 10, 9, 8, 7, 7 };

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_eu_868[] = { BW125, BW125, BW125, BW125, BW125, BW125, BW250 };

/**
 * LR-FHSS Bandwidths table definition in Hz depending on DR
 */
static const uint32_t datarates_to_lr_fhss_bw_eu_868[NUMBER_OF_TX_DR_EU_868] = { LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_NA,
                                                                                 LR_FHSS_V1_BW_136719_HZ,
                                                                                 LR_FHSS_V1_BW_136719_HZ,
                                                                                 LR_FHSS_V1_BW_335938_HZ,
                                                                                 LR_FHSS_V1_BW_335938_HZ };

/**
 * LR-FHSS Coding Rate table definition depending on DR
 */
static const uint32_t datarates_to_lr_fhss_cr_eu_868[NUMBER_OF_TX_DR_EU_868] = {
    LR_FHSS_NA, LR_FHSS_NA, LR_FHSS_NA,        LR_FHSS_NA,        LR_FHSS_NA,        LR_FHSS_NA,
    LR_FHSS_NA, LR_FHSS_NA, LR_FHSS_V1_CR_1_3, LR_FHSS_V1_CR_2_3, LR_FHSS_V1_CR_1_3, LR_FHSS_V1_CR_2_3
};

/**
 * Payload max size table definition in bytes with FHDROFFSET
 */
static const uint8_t M_eu_868[12] = { 59, 59, 59, 123, 250, 250, 250, 250, 58, 123, 58, 123 };

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
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_EU_868[] = { 2, 2, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0 };

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
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_EU_868[] = { 0, 0, 1, 3, 3, 3, 0, 0, 0, 0, 0, 0 };

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
static const uint8_t JOIN_DR_DISTRIBUTION_EU_868[] = { 1, 2, 3, 4, 4, 6, 0, 0, 0, 0, 0, 0 };

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
static const uint8_t DEFAULT_DR_DISTRIBUTION_EU_868[] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/**
 * Duty Cycle table definition by bands
 */
static const uint16_t duty_cycle_by_band_eu_868[BAND_EU868_MAX] = {
    [BAND_EU868_0] = 1000,  // 0.1%
    [BAND_EU868_1] = 100,   //   1%
    [BAND_EU868_2] = 100,   //   1%
    [BAND_EU868_3] = 1000,  // 0.1%
    [BAND_EU868_4] = 10,    //  10%
    [BAND_EU868_5] = 100,   //   1%
};

static const uint32_t frequency_range_by_band_eu_868[BAND_EU868_MAX][2] = {
    // [ band x] = {freq min, freq max}
    [BAND_EU868_0] = { 863000000, 865000000 }, [BAND_EU868_1] = { 865000000, 868000001 },
    [BAND_EU868_2] = { 868000001, 868600001 }, [BAND_EU868_3] = { 868700000, 869200001 },
    [BAND_EU868_4] = { 869400000, 869650001 }, [BAND_EU868_5] = { 869700000, 870000001 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_EU_868_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
