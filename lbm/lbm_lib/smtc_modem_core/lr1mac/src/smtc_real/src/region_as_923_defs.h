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

/* clang-format off */
#if (defined ( RP2_104 ) || defined( RP2_105 ) )
#define DEFAULT_NUMBER_OF_CHANNEL_AS_923    (24)
#else
#define DEFAULT_NUMBER_OF_CHANNEL_AS_923    (16)
#endif

#if (defined ( RP2_104 ) || defined( RP2_105 ) )
    #ifndef USER_ADDED_CHANNELS_AS_923          //If not defined from environment
        #define USER_ADDED_CHANNELS_AS_923 0    //The number of ADDED channels
    #endif
#else // Before RP2_1.0.4, no user RX/TX channels def were possible
    #define USER_ADDED_CHANNELS_AS_923 0
#endif

#define NUMBER_OF_CHANNEL_AS_923            (DEFAULT_NUMBER_OF_CHANNEL_AS_923 + \
                                            USER_ADDED_CHANNELS_AS_923) // Min is 24, end-user can provide a number to add up to 56 to have up to 80 channels
#if (NUMBER_OF_CHANNEL_AS_923 > 80)
#error "Channel number for AS923 can't be > 80. Please reduce USER_ADDED_CHANNELS_AS_923"
#endif
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
#define FREQOFFSET_GRP1_AS_923              (0)             // 0x00000000 signed 32 bits : 0 * 100 Hz, Group AS923-1
#define FREQOFFSET_GRP2_AS_923              (-18000)        // 0xFFFFB9B0 signed 32 bits : -18000 * 100 Hz, Group AS923-2
#define FREQOFFSET_GRP3_AS_923              (-66000)        // 0xFFFEFE30 signed 32 bits : -66000 * 100 Hz, Group AS923-3
#define FREQOFFSET_GRP4_AS_923              (-59000)        // 0xFFFF1988 signed 32 bits : -59000 * 100 Hz, Group AS923-4
#define RX2_FREQ_AS_923                     (923200000)     // Hz
#define FREQUENCY_FACTOR_AS_923             (100)           // MHz/100 when coded over 24 bits
#define RX2DR_INIT_AS_923                   (2)
#define SYNC_WORD_PRIVATE_AS_923            (0x12)
#define SYNC_WORD_PUBLIC_AS_923             (0x34)
#define MIN_DR_AS_923                       (0)             // When UplinkDwellTime == 0
#define MIN_TX_DR_LIMIT_AS_923              (2)             // When UplinkDwellTime == 1

#if (defined( RP2_105 ) && !defined( SX127X ))
#define MAX_DR_AS_923                       (13)
#define NUMBER_OF_TX_DR_AS_923              (14)            // Includes the invalid ones [8:11] (not used)
#define TX_DR_BITFIELD_SUPPORTED_AS_923     (uint16_t)( ( 1 << DR13 ) | ( 1 << DR12 ) | ( 1 << DR7 ) | ( 1 << DR6 ) | ( 1 << DR5 ) | \
                                                        ( 1 << DR4 )  | ( 1 << DR3 )  | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#else
#define MAX_DR_AS_923                       (7)
#define NUMBER_OF_TX_DR_AS_923              (8)
#define TX_DR_BITFIELD_SUPPORTED_AS_923     (uint16_t)( ( 1 << DR7 ) | ( 1 << DR6 ) | \
                                                        ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#endif

#if defined( RP2_105 )
#define CF_LIST_SUPPORTED_AS_923            (uint8_t)( ( 1 << CF_LIST_FREQ ) | ( 1 << CF_LIST_CH_MASK ))
#else
#define CF_LIST_SUPPORTED_AS_923            (uint8_t)( 1 << CF_LIST_FREQ )
#endif

#define RX_DR_BITFIELD_SUPPORTED_AS_923     TX_DR_BITFIELD_SUPPORTED_AS_923 // No LR-FHSS so RX is same as TX
#define DEFAULT_TX_DR_BIT_FIELD_AS_923      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define TX_PARAM_SETUP_REQ_SUPPORTED_AS_923 (true)
#define NEW_CHANNEL_REQ_SUPPORTED_AS_923    (true)
#define DTC_SUPPORTED_AS_923                (false)
#define LBT_SUPPORTED_AS_923                (true)
#define LBT_SNIFF_DURATION_MS_AS_923        (5)
#define LBT_THRESHOLD_DBM_AS_923            (int16_t)(-80)
#define LBT_BW_HZ_AS_923                    (200000)
#define UPLINK_DWELL_TIME_AS_923            (true)

// Class B
#define BEACON_FREQ_AS_923                  (923400000)     // Hz
#define BEACON_DR_AS_923                    (3)
#define PING_SLOT_FREQ_AS_923               (923400000)     // Hz

/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
#if ( ( NUMBER_OF_CHANNEL_AS_923 % 8 ) != 0 )
#define BANK_MAX_AS923 ( ( NUMBER_OF_CHANNEL_AS_923 / 8 ) + 1 )
#else
#define BANK_MAX_AS923 ( NUMBER_OF_CHANNEL_AS_923 / 8 )
#endif

typedef struct region_as923_context_s
{
    uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_AS_923];
    uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_AS_923];
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_CHANNEL_AS_923];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_AS_923];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_AS_923];
    uint8_t  join_dr_distribution[NUMBER_OF_TX_DR_AS_923];
    uint8_t  custom_dr_distribution_init[NUMBER_OF_TX_DR_AS_923];
    uint8_t  tx_channel_index_enabled[BANK_MAX_AS923];  // Enable by Network
    uint8_t  unwrapped_channel_mask[BANK_MAX_AS923];    // Temp conf send by Network
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
static const uint8_t datarate_offsets_as_923[2][NUMBER_OF_TX_DR_AS_923][8] = {
    {
        // downlink dwell time Off
        { 0, 0, 0, 0, 0, 0, 1, 2 },  // DR 0
        { 1, 0, 0, 0, 0, 0, 2, 3 },  // DR 1
        { 2, 1, 0, 0, 0, 0, 3, 4 },  // DR 2
        { 3, 2, 1, 0, 0, 0, 4, 5 },  // DR 3
        { 4, 3, 2, 1, 0, 0, 5, 6 },  // DR 4
        { 5, 4, 3, 2, 1, 0, 6, 7 },  // DR 5
        { 6, 5, 4, 3, 2, 1, 7, 7 },  // DR 6
        { 7, 6, 5, 4, 3, 2, 7, 7 },  // DR 7
#if ( defined( RP2_105 ) && !defined( SX127X ) )
        { 0, 0, 0, 0, 0, 0, 1, 2 },      // DR 8 (INVALID)
        { 0, 0, 0, 0, 0, 0, 1, 2 },      // DR 9 (INVALID)
        { 0, 0, 0, 0, 0, 0, 1, 2 },      // DR 10 (INVALID)
        { 0, 0, 0, 0, 0, 0, 1, 2 },      // DR 11 (INVALID)
        { 12, 5, 4, 3, 2, 1, 13, 13 },   // DR 12
        { 13, 12, 5, 4, 3, 2, 13, 13 },  // DR 13
#endif
    },

    {
        // downlink dwell time On
        { 2, 2, 2, 2, 2, 2, 2, 2 },  // DR 0
        { 2, 2, 2, 2, 2, 2, 2, 3 },  // DR 1
        { 2, 2, 2, 2, 2, 2, 3, 4 },  // DR 2
        { 3, 2, 2, 2, 2, 2, 4, 5 },  // DR 3
        { 4, 3, 2, 2, 2, 2, 5, 6 },  // DR 4
        { 5, 4, 3, 2, 2, 2, 6, 7 },  // DR 5
        { 6, 5, 4, 3, 2, 2, 7, 7 },  // DR 6
        { 7, 6, 5, 4, 3, 2, 7, 7 },  // DR 7
#if ( defined( RP2_105 ) && !defined( SX127X ) )
        { 2, 2, 2, 2, 2, 2, 2, 2 },      // DR 8 (INVALID)
        { 2, 2, 2, 2, 2, 2, 2, 2 },      // DR 9 (INVALID)
        { 2, 2, 2, 2, 2, 2, 2, 2 },      // DR 10 (INVALID)
        { 2, 2, 2, 2, 2, 2, 2, 2 },      // DR 11 (INVALID)
        { 12, 5, 4, 3, 2, 2, 13, 13 },   // DR 12
        { 13, 12, 5, 4, 3, 2, 13, 13 },  // DR 13
#endif
    }
};

static const uint8_t NUMBER_RX1_DR_OFFSET_AS_923 = 8;

/**
 * @brief uplink datarate backoff
 *
 */
static const uint8_t datarate_backoff_as_923[2][NUMBER_OF_TX_DR_AS_923] = { {
                                                                                // [0][dr_backoff] dwell time Off
                                                                                0,  // DR0 -> DR0
                                                                                0,  // DR1 -> DR0
                                                                                1,  // DR2 -> DR1
                                                                                2,  // DR3 -> DR2
                                                                                3,  // DR4 -> DR3
                                                                                4,  // DR5 -> DR4
                                                                                5,  // DR6 -> DR5
                                                                                6,  // DR7 -> DR6
#if ( defined( RP2_105 ) && !defined( SX127X ) )
                                                                                0,   // DR8 -> DR0 (INVALID)
                                                                                0,   // DR9 -> DR0 (INVALID)
                                                                                0,   // DR10 -> DR0 (INVALID)
                                                                                0,   // DR11 -> DR0 (INVALID)
                                                                                5,   // DR12 -> DR0
                                                                                12,  // DR13 -> DR0
#endif
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
                                                                                6,  // DR7 -> DR6
#if ( defined( RP2_105 ) && !defined( SX127X ) )
                                                                                2,   // DR8 -> DR2 (INVALID)
                                                                                2,   // DR9 -> DR2 (INVALID)
                                                                                2,   // DR10 -> DR2 (INVALID)
                                                                                2,   // DR11 -> DR2 (INVALID)
                                                                                5,   // DR12 -> DR0
                                                                                12,  // DR13 -> DR0
#endif
                                                                            } };

/**
 * LoRa Data rates table definition
 */
#if ( defined( RP2_105 ) && !defined( SX127X ) )
static const uint8_t datarates_to_sf_as_923[NUMBER_OF_TX_DR_AS_923] = { 12, 11, 10, 9, 8, 7, 7, 0, 0, 0, 0, 0, 6, 5 };

#else
static const uint8_t datarates_to_sf_as_923[NUMBER_OF_TX_DR_AS_923] = { 12, 11, 10, 9, 8, 7, 7, 0 };
#endif

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_as_923[] = {
#if ( defined( RP2_105 ) && !defined( SX127X ) )
    BW125,  BW125,  BW125,  BW125,  BW125, BW125, BW250,  BW_RFU,
    BW_RFU, BW_RFU, BW_RFU, BW_RFU, BW125, BW125, BW_RFU, BW_RFU
#else
    BW125,  BW125,  BW125,  BW125,  BW125,  BW125,  BW250,  BW_RFU,
    BW_RFU, BW_RFU, BW_RFU, BW_RFU, BW_RFU, BW_RFU, BW_RFU, BW_RFU
#endif
};

/**
 * Payload max size table definition in bytes
 * TX and RX are using same frequencies
 */
#if ( defined( RP2_105 ) && !defined( SX127X ) )
static const uint8_t M_as_923_dwell_time_off[NUMBER_OF_TX_DR_AS_923] = { 59,  59, 123, 123, 250, 250, 250,
                                                                         250, 0,  0,   0,   0,   250, 250 };
static const uint8_t M_as_923_dwell_time_on[NUMBER_OF_TX_DR_AS_923]  = { 0,   0, 19, 61, 133, 250, 250,
                                                                         250, 0, 0,  0,  0,   250, 250 };
#else
static const uint8_t M_as_923_dwell_time_off[NUMBER_OF_TX_DR_AS_923] = { 59, 59, 123, 123, 250, 250, 250, 250 };
static const uint8_t M_as_923_dwell_time_on[NUMBER_OF_TX_DR_AS_923]  = { 0, 0, 19, 61, 133, 250, 250, 250 };
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
#if ( defined( RP2_105 ) && !defined( SX127X ) )
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 2, 3, 3, 3, 0, 0, 0,
                                                                                         0, 0, 0, 0, 0, 0, 0 };
#else
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 2, 3, 3, 3, 0, 0, 0, 0 };
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
#if ( defined( RP2_105 ) && !defined( SX127X ) )
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 0, 0, 1, 3, 3, 3, 0,
                                                                                      0, 0, 0, 0, 0, 0, 0 };
#else
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 0, 0, 1, 3, 3, 3, 0, 0 };
#endif
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
#if ( defined( RP2_105 ) && !defined( SX127X ) )
static const uint8_t JOIN_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 0, 0, 4, 5, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0 };
#else
static const uint8_t JOIN_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 0, 0, 4, 5, 5, 6, 0, 0 };
#endif

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
#if ( defined( RP2_105 ) && !defined( SX127X ) )
static const uint8_t DEFAULT_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = {
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
#else
static const uint8_t DEFAULT_DR_DISTRIBUTION_AS_923[NUMBER_OF_TX_DR_AS_923] = { 0, 0, 1, 0, 0, 0, 0, 0 };
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_AS_923_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
