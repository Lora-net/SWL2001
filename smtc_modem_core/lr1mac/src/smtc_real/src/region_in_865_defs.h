/**
 * \file      region_in_865_defs.h
 *
 * \brief     region_in_865_defs  abstraction layer definition
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

#ifndef REGION_IN_865_DEFS_H
#define REGION_IN_865_DEFS_H

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
#define NUMBER_OF_CHANNEL_IN_865            (16)
#define NUMBER_OF_BOOT_TX_CHANNEL_IN_865    (3)             // define the number of channel at boot
#define JOIN_ACCEPT_DELAY1_IN_865           (5)             // define in seconds
#define JOIN_ACCEPT_DELAY2_IN_865           (6)             // define in seconds
#define RECEIVE_DELAY1_IN_865               (1)             // define in seconds
#if defined( LR11XX )
// This value must be the MIN of MAX supported by the region and the radio, region is 30dBm but radio is 22dBm ERP (+2 to EIRP) 
#define TX_POWER_EIRP_IN_865                (24)            // define in dbm
#else
// This value must be the MIN of MAX supported by the region and the radio, region is 30dBm but radio is 14dBm ERP (+2 to EIRP)
#define TX_POWER_EIRP_IN_865                (16)            // define in dbm  // TODO must be checked, SX126x dependent for the max power
#endif
#define MAX_TX_POWER_IDX_IN_865             (10)             // index ex LinkADRReq
#define ADR_ACK_LIMIT_IN_865                (64)
#define ADR_ACK_DELAY_IN_865                (32)
#define ACK_TIMEOUT_IN_865                  (2)             // +/- 1 s (random delay between 1 and 3 seconds)
#define FREQMIN_IN_865                      (865000000)     // Hz
#define FREQMAX_IN_865                      (867000000)     // Hz
#define RX2_FREQ_IN_865                     (866550000)     // Hz
#define FREQUENCY_FACTOR_IN_865             (100)           // MHz/100 when coded over 24 bits
#define RX2DR_INIT_IN_865                   (2)
#define SYNC_WORD_PRIVATE_IN_865            (0x12)
#define SYNC_WORD_PUBLIC_IN_865             (0x34)
#define MIN_DR_IN_865                       (0)
#define MAX_DR_IN_865                       (7)
#define MIN_TX_DR_LIMIT_IN_865              (0)
#define NUMBER_OF_TX_DR_IN_865              (8)
#define DR_BITFIELD_SUPPORTED_IN_865        (uint16_t)( ( 1 << DR7 ) | ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define DEFAULT_TX_DR_BIT_FIELD_IN_865      (uint16_t)( ( 1 << DR5 ) | ( 1 << DR4 ) | ( 1 << DR3 ) | ( 1 << DR2 ) | ( 1 << DR1 ) | ( 1 << DR0 ) )
#define TX_PARAM_SETUP_REQ_SUPPORTED_IN_865 (false)         // This mac command is NOT required for IN865
#define NEW_CHANNEL_REQ_SUPPORTED_IN_865    (true)
#define DTC_SUPPORTED_IN_865                (false)
#define LBT_SUPPORTED_IN_865                (false)
#define CF_LIST_SUPPORTED_IN_865            (CF_LIST_FREQ)

// Class B
#define BEACON_DR_IN_865                    (4)
#define BEACON_FREQ_IN_865                  (866550000)     // Hz
#define PING_SLOT_FREQ_IN_865               (866550000)     // Hz

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Bank contains 8 channels
 */
typedef enum in_865_channels_bank_e
{
    BANK_0_IN865 = 0,  // 0 to 7 channels
    BANK_1_IN865 = 1,  // 8 to 15 channels
    BANK_MAX_IN865
} in_865_channels_bank_t;

typedef struct region_in865_context_s
{
    uint32_t tx_frequency_channel[NUMBER_OF_CHANNEL_IN_865];
    uint32_t rx1_frequency_channel[NUMBER_OF_CHANNEL_IN_865];
    uint16_t dr_bitfield_tx_channel[NUMBER_OF_CHANNEL_IN_865];
    uint8_t  dr_distribution_init[NUMBER_OF_TX_DR_IN_865];
    uint8_t  dr_distribution[NUMBER_OF_TX_DR_IN_865];
    uint8_t  channel_index_enabled[BANK_MAX_IN865];   // Enable by Network
    uint8_t  unwrapped_channel_mask[BANK_MAX_IN865];  // Temp conf send by Network
} region_in865_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t SYNC_WORD_GFSK_IN_865[] = { 0xC1, 0x94, 0xC1 };

/**
 * Default frequencies at boot
 */
static const uint32_t default_freq_in_865[] = { 865062500, 865402500, 865985000 };

/**
 * Up/Down link data rates offset definition when no dwell time limitation
 */
static const uint8_t datarate_offsets_in_865[8][8] = {
    { 0, 0, 0, 0, 0, 0, 1, 2 },  // DR 0
    { 1, 0, 0, 0, 0, 0, 2, 3 },  // DR 1
    { 2, 1, 0, 0, 0, 0, 3, 4 },  // DR 2
    { 3, 2, 1, 0, 0, 0, 4, 5 },  // DR 3
    { 4, 3, 2, 1, 0, 0, 5, 5 },  // DR 4
    { 5, 4, 3, 2, 1, 0, 5, 7 },  // DR 5
    { 0, 0, 0, 0, 0, 0, 0, 0 },  // DR 6  !! WARNING RFU FOR IN865 !!
    { 7, 5, 5, 4, 3, 2, 7, 7 },  // DR 7
};

/**
 * @brief uplink datarate backoff
 *
 */
static const uint8_t datarate_backoff_in_865[] = {
    0,  // DR0 -> DR0
    0,  // DR1 -> DR0
    1,  // DR2 -> DR1
    2,  // DR3 -> DR2
    3,  // DR4 -> DR3
    4,  // DR5 -> DR4
    5,  // DR 6  !! WARNING RFU FOR IN865 !!
    5   // DR7 -> DR5
};

static const uint8_t NUMBER_RX1_DR_OFFSET_IN_865 =
    sizeof( datarate_offsets_in_865[0] ) / sizeof( datarate_offsets_in_865[0][0] );

/**
 * Data rates table definition
 */
static const uint8_t datarates_to_sf_in_865[] = { 12, 11, 10, 9, 8, 7, 0 };

/**
 * Bandwidths table definition in KHz
 */
static const uint32_t datarates_to_bandwidths_in_865[] = { BW125, BW125, BW125, BW125, BW125, BW125, BW250 };

/**
 * Payload max size table definition in bytes
 */
static const uint8_t M_in_865[8] = { 59, 59, 59, 123, 250, 250, 250, 250 };

/**
 * Mobile long range datarate distribution
 * DR0: 20%,
 * DR1: 20%,
 * DR2: 30%,
 * DR3: 30%,
 * DR4:  0%,
 * DR5:  0%,
 * DR6:  0%, // DR6 RFU in IN865
 * DR7:  0%
 */
static const uint8_t MOBILE_LONGRANGE_DR_DISTRIBUTION_IN_865[] = { 2, 2, 3, 3, 0, 0, 0, 0 };

/**
 * Mobile low power datarate distribution
 * DR0:  0%,
 * DR1:  0%,
 * DR2: 10%,
 * DR3: 30%,
 * DR4: 30%,
 * DR5: 30%,
 * DR6:  0%, // DR6 RFU in IN865
 * DR7:  0%
 */
static const uint8_t MOBILE_LOWPER_DR_DISTRIBUTION_IN_865[] = { 0, 0, 1, 3, 3, 3, 0, 0 };

/**
 * Join datarate distribution
 * DR0:  5%,
 * DR1: 10%,
 * DR2: 15%,
 * DR3: 20%,
 * DR4: 20%,
 * DR5: 30%,
 * DR6:  0%, // DR6 RFU in IN865
 * DR7:  0%
 */
static const uint8_t JOIN_DR_DISTRIBUTION_IN_865[] = { 1, 2, 3, 4, 4, 6, 0, 0 };

/**
 * Default datarate distribution
 * DR0: 100%,
 * DR1:   0%,
 * DR2:   0%,
 * DR3:   0%,
 * DR4:   0%,
 * DR5:   0%,
 * DR6:   0%, // DR6 RFU in IN865
 * DR7:   0%
 */
static const uint8_t DEFAULT_DR_DISTRIBUTION_IN_865[] = { 1, 0, 0, 0, 0, 0, 0, 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // REGION_IN_865_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
