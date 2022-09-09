/**
 * \file      region_us_915_defs.h
 *
 * \brief     region_us_915_defs abstraction layer definition
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

#ifndef REGION_US915_DEFS_H
#define REGION_US915_DEFS_H

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
#define NUMBER_OF_TX_CHANNEL_US_915         (72)            // TX 64 125KHz + 8 500KHz channels
#define NUMBER_OF_RX_CHANNEL_US_915         (8)             // RX 8 500KHz channels
#define JOIN_ACCEPT_DELAY1_US_915           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_US_915           (6)             // define in seconds
#define RECEIVE_DELAY1_US_915               (1)             // define in seconds
#if defined( LR11XX ) || defined( SX1262 ) || defined( SX1268 )
// This value must be the MIN of MAX supported by the region and the radio, region is 30dBm but radio is 22dBm ERP (+2 to EIRP) 
#define TX_POWER_EIRP_US_915                (24)            // define in dbm
#else
// This value must be the MIN of MAX supported by the region and the radio, region is 30dBm but radio is 14dBm ERP (+2 to EIRP)
#define TX_POWER_EIRP_US_915                (16)            // define in dbm  // TODO must be checked, SX126x dependent for the max power
#endif
#define MAX_TX_POWER_IDX_US_915             (14)            // index ex LinkADRReq
#define ADR_ACK_LIMIT_US_915                (64)
#define ADR_ACK_DELAY_US_915                (32)
#define ACK_TIMEOUT_US_915                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_US_915                      (902000000)     // Hz
#define FREQMAX_US_915                      (928000000)     // Hz
#define RX2_FREQ_US_915                     (923300000)     // Hz
#define FREQUENCY_FACTOR_US_915             (100)           // MHz/100 when coded over 24 bits
#define RX2DR_INIT_US_915                   (8)
#define SYNC_WORD_PRIVATE_US_915            (0x12)
#define SYNC_WORD_PUBLIC_US_915             (0x34)
#define MIN_TX_DR_US_915                    (0)
#define MAX_TX_DR_LORA_US_915               (4)
#define MAX_TX_DR_US_915                    (6)
#define MIN_TX_DR_LIMIT_US_915              (0)
#define NUMBER_OF_TX_DR_US_915              (7)
#define MIN_RX_DR_US_915                    (8)
#define MAX_RX_DR_US_915                    (13)

#if defined( RP2_101 )
#define DR_BITFIELD_SUPPORTED_US_915        (uint16_t)( ( 1 << DR13 ) | ( 1 << DR12 ) | ( 1 << DR11 ) | ( 1 << DR10 ) | ( 1 << DR9 ) | ( 1 << DR8 ) | \
                                                        ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#elif defined( RP2_103 )
#define DR_BITFIELD_SUPPORTED_US_915        (uint16_t)( ( 1 << DR13 ) | ( 1 << DR12 ) | ( 1 << DR11 ) | ( 1 << DR10 ) | ( 1 << DR9 ) | ( 1 << DR8 ) | \
                                                        ( 1 << DR6 ) | ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#endif

#define DEFAULT_TX_DR_125_BIT_FIELD_US_915  (uint16_t)( ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )

#if defined( RP2_101 )
#define DEFAULT_TX_DR_500_BIT_FIELD_US_915  (uint16_t)( ( 1 << DR4 ) )
#elif defined( RP2_103 )
#define DEFAULT_TX_DR_500_BIT_FIELD_US_915  (uint16_t)( ( 1 << DR6 ) | ( 1 << DR5 ) | ( 1 << DR4 ) )
#endif


#define TX_PARAM_SETUP_REQ_SUPPORTED_US_915 (false)         // This mac command is NOT required for US915
#define NEW_CHANNEL_REQ_SUPPORTED_US_915    (false)         // This mac command is NOT required for US915
#define DTC_SUPPORTED_US_915                (false)
#define LBT_SUPPORTED_US_915                (false)

#define CF_LIST_SUPPORTED_US_915            (CF_LIST_CH_MASK)

#define DEFAULT_TX_FREQ_125_START_US_915    (902300000) // Hz
#define DEFAULT_TX_STEP_125_US_915          (200000)    // Hz
#define DEFAULT_TX_FREQ_500_START_US_915    (903000000) // Hz
#define DEFAULT_TX_STEP_500_US_915          (1600000)   // Hz
#define DEFAULT_RX_FREQ_500_START_US_915    (923300000) // Hz
#define DEFAULT_RX_STEP_500_US_915          (600000)    // Hz

// Class B
#define BEACON_DR_US_915                    (8)
#define BEACON_FREQ_START_US_915            (923300000) // Hz
#define BEACON_STEP_US_915                  (600000)    // Hz
#define PING_SLOT_FREQ_START_US_915         (923300000) // Hz
#define PING_SLOT_STEP_US_915               (600000)    // Hz

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
typedef enum us_915_channels_bank_e
{
    BANK_0_125_US915 = 0,  // 125KHZ  0 -  7
    BANK_1_125_US915 = 1,  // 125KHZ  8 - 15
    BANK_2_125_US915 = 2,  // 125KHZ 16 - 23
    BANK_3_125_US915 = 3,  // 125KHZ 24 - 31
    BANK_4_125_US915 = 4,  // 125KHZ 32 - 39
    BANK_5_125_US915 = 5,  // 125KHZ 40 - 47
    BANK_6_125_US915 = 6,  // 125KHZ 48 - 55
    BANK_7_125_US915 = 7,  // 125KHZ 56 - 63
    BANK_8_500_US915 = 8,  // 500KHz 64 - 72
    BANK_MAX_US915
} us_915_channels_bank_t;

typedef struct region_us915_context_s
{
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_TX_CHANNEL_US_915];
    uint8_t  channel_index_enabled[BANK_MAX_US915];     // 8ch-125KHz + 1ch-500KHZ // Enable by Network
    uint8_t  unwrapped_channel_mask[BANK_MAX_US915];    // 8ch-125KHz + 1ch-500KHZ // Temp conf send by Network
    uint8_t  snapshot_channel_tx_mask[BANK_MAX_US915];  // 8ch-125KHz + 1ch-500KHZ // snapshot of used channels
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_US_915];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_US_915];
    uint8_t  first_ch_mask_received;

    us_915_channels_bank_t snapshot_bank_tx_mask;

} region_us915_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t SYNC_WORD_LR_FHSS_US_915[] = { 0x2C, 0x0F, 0x79, 0x95 };

/**
 * Up/Down link data rates offset definition
 */
static const uint8_t datarate_offsets_us_915[7][4] = {
    { 10, 9, 8, 8 },     // DR 0
    { 11, 10, 9, 8 },    // DR 1
    { 12, 11, 10, 9 },   // DR 2
    { 13, 12, 11, 10 },  // DR 3
    { 13, 13, 12, 11 },  // DR 4
    { 10, 9, 8, 8 },     // DR 5
    { 11, 10, 9, 8 },    // DR 6
};

/**
 * @brief uplink darate backoff
 *
 */
static const uint8_t datarate_backoff_us_915[] = {
    0,  // DR0 -> DR0
    0,  // DR1 -> DR0
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    0,  // DR5 -> DR0
    5   // DR6 -> DR5
};

static const uint8_t NUMBER_RX1_DR_OFFSET_US_915 =
    sizeof( datarate_offsets_us_915[0] ) / sizeof( datarate_offsets_us_915[0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_us_915[] = { 10, 9, 8, 7, 8, 0, 0, 0, 12, 11, 10, 9, 8, 7, 0, 0 };

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_us_915[] = { BW125, BW125, BW125, BW125, BW500, BW_RFU, BW_RFU, BW_RFU,
                                                           BW500, BW500, BW500, BW500, BW500, BW500,  BW_RFU, BW_RFU };

/**
 * LR-FHSS Coding Rate table definition depending on DR
 */
static const uint32_t datarates_to_lr_fhss_cr_us_915[NUMBER_OF_TX_DR_US_915] = {
    LR_FHSS_NA, LR_FHSS_NA, LR_FHSS_NA, LR_FHSS_NA, LR_FHSS_NA, LR_FHSS_V1_CR_1_3, LR_FHSS_V1_CR_2_3
};

/**
 * Payload max size table definition in bytes
 */
static const uint8_t M_us_915[] = { 19, 61, 133, 250, 250, 58, 133, 0, 61, 137, 250, 250, 250, 250, 0, 0 };

/**
 * Mobile long range datarate distribution
 * DR0: 20%,
 * DR1: 20%,
 * DR2: 30%,
 * DR3: 30%,
 * DR4:  0%
 */
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_US_915[] = { 2, 2, 3, 3, 0, 0, 0 };

/**
 * Mobile low power datarate distribution
 * DR0:  0%,
 * DR1: 10%,
 * DR2: 40%,
 * DR3: 50%,
 * DR4:  0%
 */
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_US_915[] = { 0, 1, 4, 5, 0, 0, 0 };

/**
 * !! NOT USED IN US915 !!
 *
 * Join datarate distribution
 * DR0: 50%,
 * DR1:  0%,
 * DR2:  0%,
 * DR3:  0%,
 * DR4: 50%
 */
static const uint8_t JOIN_DR_DISTRIBUTION_US_915[] = { 5, 0, 0, 0, 5, 0, 0 };

/**
 * Default datarate distribution
 * DR0: 100%,
 * DR1:   0%,
 * DR2:   0%,
 * DR3:   0%,
 * DR4:   0%
 */
static const uint8_t DEFAULT_DR_DISTRIBUTION_US_915[] = { 1, 0, 0, 0, 0, 0, 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_US915_DEFS_H

/* --- EOF ------------------------------------------------------------------ */