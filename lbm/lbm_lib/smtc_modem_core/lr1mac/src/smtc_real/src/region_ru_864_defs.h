/**
 * \file      region_ru_864_defs.h
 *
 * \brief     region_ru_864_defs  abstraction layer definition
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

#ifndef REGION_RU_864_DEFS_H
#define REGION_RU_864_DEFS_H

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

/* clang-format off */
#if (defined ( RP2_104 ) || defined( RP2_105 ) )
#define DEFAULT_NUMBER_OF_CHANNEL_RU_864    (24)
#else
#define DEFAULT_NUMBER_OF_CHANNEL_RU_864    (16)
#endif

#if (defined ( RP2_104 ) || defined( RP2_105 ) )
    #ifndef USER_ADDED_CHANNELS_RU_864          //If not defined from environment
        #define USER_ADDED_CHANNELS_RU_864 0    //The number of ADDED channels
    #endif
#else // Before RP2_1.0.4, no user RX/TX channels def were possible
    #define USER_ADDED_CHANNELS_RU_864 0
#endif

#define NUMBER_OF_CHANNEL_RU_864            (DEFAULT_NUMBER_OF_CHANNEL_RU_864 + \
                                            USER_ADDED_CHANNELS_RU_864) // Min is 24, end-user can provide a number to add up to 56 to have up to 80 channels
#if (NUMBER_OF_CHANNEL_RU_864 > 80)
#error "Channel number for RU864 can't be > 80. Please reduce USER_ADDED_CHANNELS_RU_864"
#endif
#define NUMBER_OF_BOOT_TX_CHANNEL_RU_864    (2)             // define the number of channel at boot
#define JOIN_ACCEPT_DELAY1_RU_864           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_RU_864           (6)             // define in seconds
#define RECEIVE_DELAY1_RU_864               (1)             // define in seconds
#define TX_POWER_EIRP_RU_864                (16)            // define in db
#define MAX_TX_POWER_IDX_RU_864             (7)             // index ex LinkADRReq
#define ADR_ACK_LIMIT_RU_864                (64)
#define ADR_ACK_DELAY_RU_864                (32)
#define ACK_TIMEOUT_RU_864                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_RU_864                      (864000000)     // Hz
#define FREQMAX_RU_864                      (870000000)     // Hz
#define RX2_FREQ_RU_864                     (869100000)     // Hz
#define FREQUENCY_FACTOR_RU_864             (100)           // MHz/100 when coded over 24 bits
#define RX2DR_INIT_RU_864                   (0)
#define SYNC_WORD_PRIVATE_RU_864            (0x12)
#define SYNC_WORD_PUBLIC_RU_864             (0x34)
#define MIN_DR_RU_864                       (0)

#if defined( RP2_105 )
#define MAX_DR_RU_864                       (13)
#define CF_LIST_SUPPORTED_RU_864            (uint8_t)( ( 1 << CF_LIST_FREQ ) | ( 1 << CF_LIST_CH_MASK ))
#define DR_BITFIELD_SUPPORTED_RU_864        (uint16_t)( ( 1 << DR13 ) | ( 1 << DR12 ) | ( 1 << DR7 ) | ( 1 << DR6 ) | ( 1 << DR5 ) | \
                                                        ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#else
#define MAX_DR_RU_864                       (7)
#define CF_LIST_SUPPORTED_RU_864            (uint8_t)( 1 << CF_LIST_FREQ )
#define DR_BITFIELD_SUPPORTED_RU_864        (uint16_t)( ( 1 << DR7 ) | ( 1 << DR6 ) | ( 1 << DR5 ) | ( 1 << DR4 ) | \
                                                        ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#endif

#define MIN_TX_DR_LIMIT_RU_864              (0)
#define NUMBER_OF_TX_DR_RU_864              (MAX_DR_RU_864 - MIN_DR_RU_864 + 1)

#define DEFAULT_TX_DR_BIT_FIELD_RU_864      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define TX_PARAM_SETUP_REQ_SUPPORTED_RU_864 (false)         // This mac command is NOT required for RU864
#define NEW_CHANNEL_REQ_SUPPORTED_RU_864    (true)
#define DTC_SUPPORTED_RU_864                (true)
#define LBT_SUPPORTED_RU_864                (false)
#define LBT_SNIFF_DURATION_MS_RU_864        (5)
#define LBT_THRESHOLD_DBM_RU_864            (int16_t)(-80)
#define LBT_BW_HZ_RU_864                    (200000)

// Class B
#define BEACON_DR_RU_864                    (3)
#define BEACON_FREQ_RU_864                  (869100000)     // Hz
#define PING_SLOT_FREQ_RU_864               (868900000)     // Hz
/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
#if ( ( NUMBER_OF_CHANNEL_RU_864 % 8 ) != 0 )
#define BANK_MAX_RU864 ( ( NUMBER_OF_CHANNEL_RU_864 / 8 ) + 1 )
#else
#define BANK_MAX_RU864 ( NUMBER_OF_CHANNEL_RU_864 / 8 )
#endif

/**
 * Bands enumeration
 */
typedef enum region_ru_864_band_e
{
    BAND_RU864_0 = 0,
    BAND_RU864_1,
    BAND_RU864_2,
    BAND_RU864_GLOBAL,  // Must be the last band, used as a garbage if there are gaps between sub-bands
    BAND_RU864_MAX
} region_ru_864_band_t;

typedef struct region_ru864_context_s
{
    uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_RU_864];
    uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_RU_864];
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_CHANNEL_RU_864];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_RU_864];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_RU_864];
    uint8_t  join_dr_distribution[NUMBER_OF_TX_DR_RU_864];
    uint8_t  custom_dr_distribution_init[NUMBER_OF_TX_DR_RU_864];
    uint8_t  tx_channel_index_enabled[BANK_MAX_RU864];  // Enable by Network
    uint8_t  unwrapped_channel_mask[BANK_MAX_RU864];    // Temp conf send by Network
} region_ru864_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t SYNC_WORD_GFSK_RU_864[] = { 0xC1, 0x94, 0xC1 };

/**
 * Default frequencies at boot
 */
static const uint32_t default_freq_ru_864[] = { 868900000, 869100000 };

/**
 * Up/Down link data rates offset definition
 */
static const uint8_t datarate_offsets_ru_864[NUMBER_OF_TX_DR_RU_864][6] = {
    { 0, 0, 0, 0, 0, 0 },  // DR 0
    { 1, 0, 0, 0, 0, 0 },  // DR 1
    { 2, 1, 0, 0, 0, 0 },  // DR 2
    { 3, 2, 1, 0, 0, 0 },  // DR 3
    { 4, 3, 2, 1, 0, 0 },  // DR 4
    { 5, 4, 3, 2, 1, 0 },  // DR 5
    { 6, 5, 4, 3, 2, 1 },  // DR 6
    { 7, 6, 5, 4, 3, 2 },  // DR 7
#if defined( RP2_105 )
    { 0, 0, 0, 0, 0, 0 },    // DR 8 (RFU)
    { 0, 0, 0, 0, 0, 0 },    // DR 9 (RFU)
    { 0, 0, 0, 0, 0, 0 },    // DR 10 (RFU)
    { 0, 0, 0, 0, 0, 0 },    // DR 11 (RFU)
    { 12, 5, 4, 3, 2, 1 },   // DR 12
    { 13, 12, 5, 4, 3, 2 },  // DR 13
#endif
};

static const uint8_t NUMBER_RX1_DR_OFFSET_RU_864 = 6;

/**
 * @brief uplink datarate backoff
 *
 */
static const uint8_t datarate_backoff_ru_864[NUMBER_OF_TX_DR_RU_864] = {
    0,  // DR0 -> DR0
    0,  // DR1 -> DR0
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    4,  // DR5 -> DR4
    5,  // DR6 -> DR5
    6,  // DR7 -> DR6
#if defined( RP2_105 )
    0,   // DR 8 (RFU)
    0,   // DR 9 (RFU)
    0,   // DR 10 (RFU)
    0,   // DR 11 (RFU)
    5,   // DR 12 -> DR5
    12,  // DR 13 -> DR12
#endif
};

/**
 * LoRa data rates table definition
 */
#if defined( RP2_105 )
static const uint8_t datarates_to_sf_ru_864[NUMBER_OF_TX_DR_RU_864] = { 12, 11, 10, 9, 8, 7, 7, 0, 0, 0, 0, 0, 6, 5 };
#else
static const uint8_t datarates_to_sf_ru_864[NUMBER_OF_TX_DR_RU_864] = { 12, 11, 10, 9, 8, 7, 7, 0 };
#endif

/**
 * LoRa bandwidths table definition in KHz
 */
#if defined( RP2_105 )
static const uint32_t datarates_to_bandwidths_ru_864[NUMBER_OF_TX_DR_RU_864] = { BW125,  BW125,  BW125,  BW125,  BW125,
                                                                                 BW125,  BW250,  BW_RFU, BW_RFU, BW_RFU,
                                                                                 BW_RFU, BW_RFU, BW125,  BW125 };
#else
static const uint32_t datarates_to_bandwidths_ru_864[NUMBER_OF_TX_DR_RU_864] = { BW125, BW125, BW125, BW125,
                                                                                 BW125, BW125, BW250, BW_RFU };
#endif

/**
 * Payload max size table definition in bytes
 */
#if defined( RP2_105 )
static const uint8_t M_ru_864[NUMBER_OF_TX_DR_RU_864] = { 59, 59, 59, 123, 250, 250, 250, 250, 0, 0, 0, 0, 250, 250 };
#else
static const uint8_t M_ru_864[NUMBER_OF_TX_DR_RU_864] = { 59, 59, 59, 123, 250, 250, 250, 250 };
#endif

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
#if defined( RP2_105 )
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 2, 2, 3, 3, 0, 0, 0,
                                                                                         0, 0, 0, 0, 0, 0, 0 };
#else
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 2, 2, 3, 3, 0, 0, 0, 0 };
#endif

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
#if defined( RP2_105 )
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 0, 0, 1, 3, 3, 3, 0,
                                                                                      0, 0, 0, 0, 0, 0, 0 };
#else
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 0, 0, 1, 3, 3, 3, 0, 0 };
#endif

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
#if defined( RP2_105 )
static const uint8_t JOIN_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 1, 2, 3, 4, 4, 6, 0, 0, 0, 0, 0, 0, 0, 0 };
#else
static const uint8_t JOIN_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 1, 2, 3, 4, 4, 6, 0, 0 };
#endif

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
#if defined( RP2_105 )
static const uint8_t DEFAULT_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = {
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#else
static const uint8_t DEFAULT_DR_DISTRIBUTION_RU_864[NUMBER_OF_TX_DR_RU_864] = { 1, 0, 0, 0, 0, 0, 0, 0 };
#endif

/**
 * Duty Cycle table definition by bands
 */
static const uint16_t duty_cycle_by_band_ru_864[BAND_RU864_MAX] = {
    [BAND_RU864_0] = 1000,  // 0.1%
    [BAND_RU864_1] = 100,   //   1%
    [BAND_RU864_2] = 10,    //  10%
};

static const uint32_t frequency_range_by_band_ru_864[BAND_RU864_MAX][2] = {
    // [ band x] = {freq min, freq max}
    [BAND_RU864_0] = { 864000000, 865000001 },
    [BAND_RU864_1] = { 866000000, 868000001 },
    [BAND_RU864_2] = { 868700000, 869200001 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_RU_864_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
