/**
 * \file      region_cn_470_defs.h
 *
 * \brief     region_cn_470_defs abstraction layer definition
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

#ifndef REGION_CN470_DEFS_H
#define REGION_CN470_DEFS_H

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
#define NUMBER_OF_TX_CHANNEL_CN_470         (64)            // Max Tx channels required for a group
#define NUMBER_OF_RX_CHANNEL_CN_470         (64)            // Max Rx channels required for a group
#define JOIN_ACCEPT_DELAY1_CN_470           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_CN_470           (6)             // define in seconds
#define RECEIVE_DELAY1_CN_470               (1)             // define in seconds
#if defined( LR11XX )
#define TX_POWER_EIRP_CN_470                (19)            // define in dbm
#else
// This value must be the MIN of MAX supported by the region and the radio, region is 19dBm EIRP but radio is 14dBm ERP (+2 to EIRP)
#define TX_POWER_EIRP_CN_470                (16)            // define in dbm
#endif
#define MAX_TX_POWER_IDX_CN_470             (7)            // index ex LinkADRReq
#define ADR_ACK_LIMIT_CN_470                (64)
#define ADR_ACK_DELAY_CN_470                (32)
#define ACK_TIMEOUT_CN_470                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_CN_470                      (470000000)     // Hz
#define FREQMAX_CN_470                      (510000000)     // Hz
#define FREQUENCY_FACTOR_CN_470             (100)           // MHz/100 when coded over 24 bits
#define SYNC_WORD_PRIVATE_CN_470            (0x12)
#define SYNC_WORD_PUBLIC_CN_470             (0x34)
#define MIN_TX_DR_CN_470                    (0)
#define MAX_TX_DR_CN_470                    (7)
#define NUMBER_OF_TX_DR_CN_470              (8)
#define DR_BITFIELD_SUPPORTED_CN_470        (uint16_t)( ( 1 << DR7 ) | ( 1 << DR6 ) | \
                                                        ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define DEFAULT_TX_DR_BIT_FIELD_CN_470      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) )
#if defined( HYBRID_CN470_MONO_CHANNEL )
    #define MIN_TX_DR_LIMIT_CN_470              (0)
    #define MIN_RX_DR_CN_470                    (0)
    #define RX2DR_INIT_CN_470                   (0)
#else
    #define MIN_TX_DR_LIMIT_CN_470              (1)
    #define MIN_RX_DR_CN_470                    (1)
    #define RX2DR_INIT_CN_470                   (1)
#endif
#define MAX_RX_DR_CN_470                    (7)
#define TX_PARAM_SETUP_REQ_SUPPORTED_CN_470 (false)         // This mac command is NOT required for CN470
#define NEW_CHANNEL_REQ_SUPPORTED_CN_470    (false)         // This mac command is NOT required for CN470
#define DTC_SUPPORTED_CN_470                (false)
#if defined( HYBRID_CN470_MONO_CHANNEL )
    #define LBT_SUPPORTED_CN_470                (false)
#else
    #define LBT_SUPPORTED_CN_470                (true)
#endif
#define LBT_SNIFF_DURATION_MS_CN_470        (5)             // TODO value must be checked
#define LBT_THRESHOLD_DBM_CN_470            (int16_t)(-80)  // TODO value must be checked
#define LBT_BW_HZ_CN_470                    (200000)        // TODO value must be checked

#define CF_LIST_SUPPORTED_CN_470            (CF_LIST_FREQ)

// Class B
#define BEACON_DR_CN_470                    (2)

#if defined( HYBRID_CN470_MONO_CHANNEL )
#define NUMBER_OF_TX_CHANNEL_20MHZ_CN_470   (64)
#define NUMBER_OF_RX_CHANNEL_20MHZ_CN_470   (32)
#define NUMBER_OF_TX_CHANNEL_26MHZ_CN_470   (48)
#define NUMBER_OF_RX_CHANNEL_26MHZ_CN_470   (24)

#define DEFAULT_TX_FREQ_20MHZ_A1_CN_470     (470300000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_A1_CN_470     (500300000) // Hz

#define DEFAULT_TX_FREQ_20MHZ_A2_CN_470     (503500000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_A2_CN_470     (490300000) // Hz

#define DEFAULT_TX_FREQ_20MHZ_B1_CN_470     (476900000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_B1_CN_470     (476900000) // Hz
#define DEFAULT_TX_FREQ_20MHZ_B2_CN_470     (496900000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_B2_CN_470     (496900000) // Hz

#define DEFAULT_TX_FREQ_26MHZ_A_CN_470      (470300000) // Hz
#define DEFAULT_RX_FREQ_26MHZ_A_CN_470      (490100000) // Hz

#define DEFAULT_TX_FREQ_26MHZ_B_CN_470      (480300000) // Hz
#define DEFAULT_RX_FREQ_26MHZ_B_CN_470      (500100000) // Hz

#define DEFAULT_TX_STEP_CN_470              (200000)    // Hz
#define DEFAULT_RX_STEP_CN_470              (200000)    // Hz
#else
#define NUMBER_OF_TX_CHANNEL_20MHZ_CN_470   (64)
#define NUMBER_OF_RX_CHANNEL_20MHZ_CN_470   (64)
#define NUMBER_OF_TX_CHANNEL_26MHZ_CN_470   (48)
#define NUMBER_OF_RX_CHANNEL_26MHZ_CN_470   (24)

#define DEFAULT_TX_FREQ_20MHZ_A1_CN_470     (470300000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_A1_CN_470     (483900000) // Hz
#define DEFAULT_TX_FREQ_20MHZ_A2_CN_470     (503500000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_A2_CN_470     (490300000) // Hz

#define DEFAULT_TX_FREQ_20MHZ_B1_CN_470     (476900000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_B1_CN_470     (476900000) // Hz
#define DEFAULT_TX_FREQ_20MHZ_B2_CN_470     (496900000) // Hz
#define DEFAULT_RX_FREQ_20MHZ_B2_CN_470     (496900000) // Hz

#define DEFAULT_TX_FREQ_26MHZ_A_CN_470      (470300000) // Hz
#define DEFAULT_RX_FREQ_26MHZ_A_CN_470      (490100000) // Hz

#define DEFAULT_TX_FREQ_26MHZ_B_CN_470      (480300000) // Hz
#define DEFAULT_RX_FREQ_26MHZ_B_CN_470      (500100000) // Hz

#define DEFAULT_TX_STEP_CN_470              (200000)    // Hz
#define DEFAULT_RX_STEP_CN_470              (200000)    // Hz
#endif
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
typedef enum cn_470_channels_bank_e
{
    BANK_0_125_CN470 = 0,  //  0 -  7
    BANK_1_125_CN470 = 1,  //  8 - 15
    BANK_2_125_CN470 = 2,  // 16 - 23
    BANK_3_125_CN470 = 3,  // 24 - 31
    BANK_4_125_CN470 = 4,  // 32 - 39
    BANK_5_125_CN470 = 5,  // 40 - 47
    BANK_6_125_CN470 = 6,  // 48 - 55
    BANK_7_125_CN470 = 7,  // 56 - 63
    BANK_MAX_CN470
} cn_470_channels_bank_t;

/**
 * Channel plan enum type
 */
typedef enum channel_plan_type_e
{
    CN_470_20MHZ_A = 0,
    CN_470_20MHZ_B,
    CN_470_26MHZ_A,
    CN_470_26MHZ_B
} channel_plan_type_cn470_t;

typedef struct region_cn470_context_s
{
    uint16_t                  dr_bitfield_tx_channel[NUMBER_OF_TX_CHANNEL_CN_470];
    uint8_t                   dr_distribution_init[NUMBER_OF_TX_DR_CN_470];
    uint8_t                   dr_distribution[NUMBER_OF_TX_DR_CN_470];
    uint8_t                   channel_index_enabled[BANK_MAX_CN470];  // Contain the index of the activated channel only
    uint8_t                   unwrapped_channel_mask[BANK_MAX_CN470];
    uint8_t                   activated_by_join_channel;  // Channel used to join
    channel_plan_type_cn470_t activated_channel_plan;

} region_cn470_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t SYNC_WORD_GFSK_CN_470[] = { 0xC1, 0x94, 0xC1 };

/**
 * Up/Down link data rates offset definition
 */
static const uint8_t datarate_offsets_cn_470[8][6] = {
    { 0, 0, 0, 0, 0, 0 },  // DR 0
    { 1, 1, 1, 1, 1, 1 },  // DR 1
    { 2, 1, 1, 1, 1, 1 },  // DR 2
    { 3, 2, 1, 1, 1, 1 },  // DR 3
    { 4, 3, 2, 1, 1, 1 },  // DR 4
    { 5, 4, 3, 2, 1, 1 },  // DR 5
    { 6, 5, 4, 3, 2, 1 },  // DR 6
    { 7, 6, 5, 4, 3, 2 },  // DR 7
};

/**
 * @brief uplink datarate backoff
 *
 */
static const uint8_t datarate_backoff_cn_470[] = {
    1,  // DR0 -> DR1 // DR 0  !! WARNING RFU FOR CN470 !!
    1,  // DR1 -> DR1
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    4,  // DR5 -> DR4
    5,  // DR6 -> DR5
    6   // DR7 -> DR6
};

static const uint8_t NUMBER_RX1_DR_OFFSET_CN_470 =
    sizeof( datarate_offsets_cn_470[0] ) / sizeof( datarate_offsets_cn_470[0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_cn_470[] = { 12, 11, 10, 9, 8, 7, 7 };

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_cn_470[] = { BW125, BW125, BW125, BW125, BW125, BW125, BW500 };

/**
 * Payload max size table definition in bytes
 */
static const uint8_t M_cn_470[8] = { 0, 31, 94, 192, 250, 250, 250, 250 };

/**
 * Mobile long range datarate distribution
 * DR0:  0%, // DR0 Not allowed in CN470
 * DR1: 20%,
 * DR2: 20%,
 * DR3: 30%,
 * DR4: 30%,
 * DR5:  0%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_CN_470[] = { 0, 2, 2, 3, 3, 0, 0, 0 };

/**
 * Mobile low power datarate distribution
 * DR0:  0%, // DR0 Not allowed in CN470
 * DR1:  0%,
 * DR2: 10%,
 * DR3: 30%,
 * DR4: 30%,
 * DR5: 30%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_CN_470[] = { 0, 0, 1, 3, 3, 3, 0, 0 };

/**
 * Join datarate distribution
 * DR0:  0%, // DR0 Not allowed in CN470
 * DR1: 15%,
 * DR2: 15%,
 * DR3: 20%,
 * DR4: 20%,
 * DR5: 30%,
 * DR6:  0%,
 * DR7:  0%
 */
static const uint8_t JOIN_DR_DISTRIBUTION_CN_470[] = { 0, 3, 3, 4, 4, 6, 0, 0 };

/**
 * Default datarate distribution
 * DR0:   0%, // DR0 Not allowed in CN470
 * DR1: 100%,
 * DR2:   0%,
 * DR3:   0%,
 * DR4:   0%,
 * DR5:   0%,
 * DR6:   0%,
 * DR7:   0%
 */
static const uint8_t DEFAULT_DR_DISTRIBUTION_CN_470[] = { 0, 1, 0, 0, 0, 0, 0, 0 };

/**
 * Common Join channels, frequencies in Hz
 */

#if defined( HYBRID_CN470_MONO_CHANNEL )
static const uint32_t common_join_channel_cn_470[][3] = {
    // { Tx, Rx1, Rx2 }
    { 470900000, 500900000, 505300000 },  // channel  0 - 20MHz, plan A
    { 472500000, 486100000, 505300000 },  // channel  1 - 20MHz, plan A
    { 474100000, 487700000, 505300000 },  // channel  2 - 20MHz, plan A
    { 475700000, 489300000, 505300000 },  // channel  3 - 20MHz, plan A
    { 504100000, 490900000, 505300000 },  // channel  4 - 20MHz, plan A
    { 505700000, 492500000, 505300000 },  // channel  5 - 20MHz, plan A
    { 507300000, 494100000, 505300000 },  // channel  6 - 20MHz, plan A
    { 508900000, 495700000, 505300000 },  // channel  7 - 20MHz, plan A
    { 479900000, 479900000, 505300000 },  // channel  8 - 20MHz, plan B
    { 499900000, 499900000, 505300000 },  // channel  9 - 20MHz, plan B
    { 470300000, 492500000, 505300000 },  // channel 10 - 26MHz, plan A
    { 472300000, 492500000, 505300000 },  // channel 11 - 26MHz, plan A
    { 474300000, 492500000, 505300000 },  // channel 12 - 26MHz, plan A
    { 476300000, 492500000, 505300000 },  // channel 13 - 26MHz, plan A
    { 478300000, 492500000, 505300000 },  // channel 14 - 26MHz, plan A
    { 480300000, 502500000, 505300000 },  // channel 15 - 26MHz, plan B
    { 482300000, 502500000, 505300000 },  // channel 16 - 26MHz, plan B
    { 484300000, 502500000, 505300000 },  // channel 17 - 26MHz, plan B
    { 486300000, 502500000, 505300000 },  // channel 18 - 26MHz, plan B
    { 488300000, 502500000, 505300000 }   // channel 19 - 26MHz, plan B
};
#else
static const uint32_t common_join_channel_cn_470[][3] = {
    // { Tx, Rx1, Rx2 }
    { 470900000, 484500000, 485300000 },  // channel  0 - 20MHz, plan A
    { 472500000, 486100000, 486900000 },  // channel  1 - 20MHz, plan A
    { 474100000, 487700000, 488500000 },  // channel  2 - 20MHz, plan A
    { 475700000, 489300000, 490100000 },  // channel  3 - 20MHz, plan A
    { 504100000, 490900000, 491700000 },  // channel  4 - 20MHz, plan A
    { 505700000, 492500000, 493300000 },  // channel  5 - 20MHz, plan A
    { 507300000, 494100000, 494900000 },  // channel  6 - 20MHz, plan A
    { 508900000, 495700000, 496500000 },  // channel  7 - 20MHz, plan A
    { 479900000, 479900000, 478300000 },  // channel  8 - 20MHz, plan B
    { 499900000, 499900000, 498300000 },  // channel  9 - 20MHz, plan B
    { 470300000, 492500000, 492500000 },  // channel 10 - 26MHz, plan A
    { 472300000, 492500000, 492500000 },  // channel 11 - 26MHz, plan A
    { 474300000, 492500000, 492500000 },  // channel 12 - 26MHz, plan A
    { 476300000, 492500000, 492500000 },  // channel 13 - 26MHz, plan A
    { 478300000, 492500000, 492500000 },  // channel 14 - 26MHz, plan A
    { 480300000, 502500000, 502500000 },  // channel 15 - 26MHz, plan B
    { 482300000, 502500000, 502500000 },  // channel 16 - 26MHz, plan B
    { 484300000, 502500000, 502500000 },  // channel 17 - 26MHz, plan B
    { 486300000, 502500000, 502500000 },  // channel 18 - 26MHz, plan B
    { 488300000, 502500000, 502500000 }   // channel 19 - 26MHz, plan B
};
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_CN470_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
