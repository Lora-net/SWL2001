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
 * @brief The maximal number of results to gather during the scan for MAC addresses. Maximum value is 32
 */
#define WIFI_MAX_BASIC_RESULTS ( 8 )

/*!
 * @brief The maximal number of results to gather during the scan for SSID/Country code. Maximum value is 32
 */
#define WIFI_MAX_EXTENDED_RESULTS ( 32 )

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
 * @brief Enumeration representing the type of results
 */
typedef enum
{
    WIFI_RESULT_TYPE_BASIC,     //!< basic: mac_address, type, rssi, origin
    WIFI_RESULT_TYPE_EXTENDED,  //!< extended: basic + country_code + ssid_bytes
} wifi_result_type_t;

/*!
 * @brief Structure representing the configuration of Wi-Fi scan
 */
typedef struct
{
    lr11xx_wifi_mode_t             scan_mode;
    lr11xx_wifi_channel_mask_t     channels;             //!< A mask of the channels to be scanned
    lr11xx_wifi_signal_type_scan_t types;                //!< Wi-Fi types to be scanned
    uint8_t                        max_results;          //!< Maximum number of results expected for a scan
    uint32_t                       timeout_per_channel;  //!< Time to spend scanning one channel, in ms
    uint32_t timeout_per_scan;  //!< Maximal time to spend in preamble detection for each single scan, in ms
} wifi_settings_t;

/*!
 * @brief Structure representing the information of a single Wi-Fi access point detected during a scan
 */
typedef struct
{
    wifi_result_type_t               result_type; /* basic or extended results */
    lr11xx_wifi_mac_address_t        mac_address;
    lr11xx_wifi_channel_t            channel;
    lr11xx_wifi_signal_type_result_t type;
    int8_t                           rssi;
    lr11xx_wifi_mac_origin_t         origin;
    uint8_t                          country_code[LR11XX_WIFI_STR_COUNTRY_CODE_SIZE]; /* only for extented results */
    uint8_t                          ssid_bytes[LR11XX_WIFI_RESULT_SSID_LENGTH];      /* only for extented results */
} wifi_scan_single_result_t;

/*!
 * @brief Structure representing the results of a Wi-Fi scan
 */
typedef struct
{
    uint8_t                   nbr_results;
    uint32_t                  power_consumption_nah;
    uint32_t                  scan_duration_ms;
    wifi_scan_single_result_t results[WIFI_MAX_EXTENDED_RESULTS];
} wifi_scan_result_t;

#ifdef __cplusplus
}
#endif

#endif  // __WIFI_HELPERS_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */