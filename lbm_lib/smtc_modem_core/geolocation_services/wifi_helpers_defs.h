/**
 * @file      wifi_helpers_defs.h
 *
 * @brief     Types and constants definitions of Wi-Fi helpers.
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

#ifndef __WIFI_HELPERS_DEFS_H__
#define __WIFI_HELPERS_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_wifi.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief The maximal time to spend in preamble detection for each single scan, in ms
 */
#define WIFI_TIMEOUT_PER_SCAN_DEFAULT ( 90 )

/*!
 * @brief The time to spend scanning one channel, in ms
 */
#define WIFI_TIMEOUT_PER_CHANNEL_DEFAULT ( 300 )

/*!
 * @brief The maximal number of results to gather during the scan. Maximum value is 32
 */
#define WIFI_MAX_RESULTS ( 8 )

/*!
 * @brief The maximal number of results to send. Maximum value is 32
 */
#define WIFI_MAX_RESULTS_TO_SEND ( 5 )

/**
 * @brief Size in bytes of a WiFi Access-Point address
 */
#define WIFI_AP_ADDRESS_SIZE ( 6 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Structure representing the configuration of Wi-Fi scan
 */
typedef struct
{
    lr11xx_wifi_channel_mask_t     channels;             //!< A mask of the channels to be scanned
    lr11xx_wifi_signal_type_scan_t types;                //!< Wi-Fi types to be scanned
    uint8_t                        max_results;          //!< Maximum number of results expected for a scan
    uint32_t                       timeout_per_channel;  //!< Time to spend scanning one channel, in ms
    uint32_t timeout_per_scan;  //!< Maximal time to spend in preamble detection for each single scan, in ms
} wifi_settings_t;

/*!
 * @brief Structure representing a single scan result
 */
typedef struct
{
    lr11xx_wifi_mac_address_t        mac_address;  //!< MAC address of the Wi-Fi access point which has been detected
    lr11xx_wifi_channel_t            channel;      //!< Channel on which the access point has been detected
    lr11xx_wifi_signal_type_result_t type;         //!< Type of Wi-Fi which has been detected
    int8_t                           rssi;         //!< Strength of the detected signal
    lr11xx_wifi_mac_origin_t origin;  //!< Estimation of the origin of a MAC address (fix or mobile Access Point)
} wifi_scan_single_result_t;

/*!
 * @brief Structure representing a collection of scan results
 */
typedef struct
{
    uint8_t                   nbr_results;                //!< Number of results
    uint32_t                  power_consumption_nah;      //!< Power consumption to acquire this set of results
    wifi_scan_single_result_t results[WIFI_MAX_RESULTS];  //!< Buffer containing the results
    uint32_t                  scan_duration_ms;           //!< Duration of the scan in millisecond
} wifi_scan_all_result_t;

#ifdef __cplusplus
}
#endif

#endif  // __WIFI_HELPERS_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */