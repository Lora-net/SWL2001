/*!
 * @file      lr11xx_wifi_types.h
 *
 * @brief     Wi-Fi passive scan driver types for LR11XX
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

#ifndef LR11XX_WIFI_TYPES_H
#define LR11XX_WIFI_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define LR11XX_WIFI_MAC_ADDRESS_LENGTH ( 6 )
#define LR11XX_WIFI_MAX_RESULTS ( 32 )
#define LR11XX_WIFI_RESULT_SSID_LENGTH ( 32 )
#define LR11XX_WIFI_MAX_COUNTRY_CODE ( 32 )
#define LR11XX_WIFI_STR_COUNTRY_CODE_SIZE ( 2 )

#define LR11XX_WIFI_CHANNEL_1_POS ( 0U )  //!< Channel at frequency 2.412 GHz
#define LR11XX_WIFI_CHANNEL_1_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_1_POS )
#define LR11XX_WIFI_CHANNEL_2_POS ( 1U )  //!< Channel at frequency 2.417 GHz
#define LR11XX_WIFI_CHANNEL_2_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_2_POS )
#define LR11XX_WIFI_CHANNEL_3_POS ( 2U )  //!< Channel at frequency 2.422 GHz
#define LR11XX_WIFI_CHANNEL_3_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_3_POS )
#define LR11XX_WIFI_CHANNEL_4_POS ( 3U )  //!< Channel at frequency 2.427 GHz
#define LR11XX_WIFI_CHANNEL_4_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_4_POS )
#define LR11XX_WIFI_CHANNEL_5_POS ( 4U )  //!< Channel at frequency 2.432 GHz
#define LR11XX_WIFI_CHANNEL_5_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_5_POS )
#define LR11XX_WIFI_CHANNEL_6_POS ( 5U )  //!< Channel at frequency 2.437 GHz
#define LR11XX_WIFI_CHANNEL_6_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_6_POS )
#define LR11XX_WIFI_CHANNEL_7_POS ( 6U )  //!< Channel at frequency 2.442 GHz
#define LR11XX_WIFI_CHANNEL_7_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_7_POS )
#define LR11XX_WIFI_CHANNEL_8_POS ( 7U )  //!< Channel at frequency 2.447 GHz
#define LR11XX_WIFI_CHANNEL_8_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_8_POS )
#define LR11XX_WIFI_CHANNEL_9_POS ( 8U )  //!< Channel at frequency 2.452 GHz
#define LR11XX_WIFI_CHANNEL_9_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_9_POS )
#define LR11XX_WIFI_CHANNEL_10_POS ( 9U )  //!< Channel at frequency 2.457 GHz
#define LR11XX_WIFI_CHANNEL_10_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_10_POS )
#define LR11XX_WIFI_CHANNEL_11_POS ( 10U )  //!< Channel at frequency 2.462 GHz
#define LR11XX_WIFI_CHANNEL_11_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_11_POS )
#define LR11XX_WIFI_CHANNEL_12_POS ( 11U )  //!< Channel at frequency 2.467 GHz
#define LR11XX_WIFI_CHANNEL_12_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_12_POS )
#define LR11XX_WIFI_CHANNEL_13_POS ( 12U )  //!< Channel at frequency 2.472 GHz
#define LR11XX_WIFI_CHANNEL_13_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_13_POS )
#define LR11XX_WIFI_CHANNEL_14_POS ( 13U )  //!< Channel at frequency 2.484 GHz
#define LR11XX_WIFI_CHANNEL_14_MASK ( 0x01UL << LR11XX_WIFI_CHANNEL_14_POS )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Type to store a Wi-Fi channel mask
 */
typedef uint16_t lr11xx_wifi_channel_mask_t;

/*!
 * @brief Type to store a Wi-Fi channel info byte
 */
typedef uint8_t lr11xx_wifi_channel_info_byte_t;

/*!
 * @brief Type to store a Wi-Fi datarate info byte
 */
typedef uint8_t lr11xx_wifi_datarate_info_byte_t;

/*!
 * @brief Type to store a Wi-Fi frame type info byte
 */
typedef uint8_t lr11xx_wifi_frame_type_info_byte_t;

/*!
 * @brief Type to store a Wi-Fi frame sub_type
 */
typedef uint8_t lr11xx_wifi_frame_sub_type_t;

/*!
 * @brief Wi-Fi FCS info byte
 */
typedef struct lr11xx_wifi_fcs_info_byte_s
{
    bool is_fcs_ok;       //!< True if the LR11XX has checked the FCS and the check succeeded
    bool is_fcs_checked;  //!< True if the LR11XX has checked the FCS
} lr11xx_wifi_fcs_info_byte_t;

/*!
 * @brief Type to store a MAC address
 */
typedef uint8_t lr11xx_wifi_mac_address_t[LR11XX_WIFI_MAC_ADDRESS_LENGTH];

/*!
 * @brief Type to store the Country Code
 */
typedef uint8_t lr11xx_wifi_country_code_str_t[LR11XX_WIFI_STR_COUNTRY_CODE_SIZE];

/*!
 * @brief Wi-Fi Channels index
 */
typedef enum
{
    LR11XX_WIFI_NO_CHANNEL   = 0x00,
    LR11XX_WIFI_CHANNEL_1    = 0x01,  //!< Channel at frequency 2.412 GHz
    LR11XX_WIFI_CHANNEL_2    = 0x02,  //!< Channel at frequency 2.417 GHz
    LR11XX_WIFI_CHANNEL_3    = 0x03,  //!< Channel at frequency 2.422 GHz
    LR11XX_WIFI_CHANNEL_4    = 0x04,  //!< Channel at frequency 2.427 GHz
    LR11XX_WIFI_CHANNEL_5    = 0x05,  //!< Channel at frequency 2.432 GHz
    LR11XX_WIFI_CHANNEL_6    = 0x06,  //!< Channel at frequency 2.437 GHz
    LR11XX_WIFI_CHANNEL_7    = 0x07,  //!< Channel at frequency 2.442 GHz
    LR11XX_WIFI_CHANNEL_8    = 0x08,  //!< Channel at frequency 2.447 GHz
    LR11XX_WIFI_CHANNEL_9    = 0x09,  //!< Channel at frequency 2.452 GHz
    LR11XX_WIFI_CHANNEL_10   = 0x0A,  //!< Channel at frequency 2.457 GHz
    LR11XX_WIFI_CHANNEL_11   = 0x0B,  //!< Channel at frequency 2.462 GHz
    LR11XX_WIFI_CHANNEL_12   = 0x0C,  //!< Channel at frequency 2.467 GHz
    LR11XX_WIFI_CHANNEL_13   = 0x0D,  //!< Channel at frequency 2.472 GHz
    LR11XX_WIFI_CHANNEL_14   = 0x0E,  //!< Channel at frequency 2.484 GHz
    LR11XX_WIFI_ALL_CHANNELS = 0x0F,
} lr11xx_wifi_channel_t;

/*!
 * @brief WiFi theoretical Datarates
 */
typedef enum
{
    LR11XX_WIFI_DATARATE_1_MBPS    = 1,
    LR11XX_WIFI_DATARATE_2_MBPS    = 2,
    LR11XX_WIFI_DATARATE_6_MBPS    = 3,
    LR11XX_WIFI_DATARATE_9_MBPS    = 4,
    LR11XX_WIFI_DATARATE_12_MBPS   = 5,
    LR11XX_WIFI_DATARATE_18_MBPS   = 6,
    LR11XX_WIFI_DATARATE_24_MBPS   = 7,
    LR11XX_WIFI_DATARATE_36_MBPS   = 8,
    LR11XX_WIFI_DATARATE_48_MBPS   = 9,
    LR11XX_WIFI_DATARATE_54_MBPS   = 10,
    LR11XX_WIFI_DATARATE_6_5_MBPS  = 11,
    LR11XX_WIFI_DATARATE_13_MBPS   = 12,
    LR11XX_WIFI_DATARATE_19_5_MBPS = 13,
    LR11XX_WIFI_DATARATE_26_MBPS   = 14,
    LR11XX_WIFI_DATARATE_39_MBPS   = 15,
    LR11XX_WIFI_DATARATE_52_MBPS   = 16,
    LR11XX_WIFI_DATARATE_58_MBPS   = 17,
    LR11XX_WIFI_DATARATE_65_MBPS   = 18,
    LR11XX_WIFI_DATARATE_7_2_MBPS  = 19,
    LR11XX_WIFI_DATARATE_14_4_MBPS = 20,
    LR11XX_WIFI_DATARATE_21_7_MBPS = 21,
    LR11XX_WIFI_DATARATE_28_9_MBPS = 22,
    LR11XX_WIFI_DATARATE_43_3_MBPS = 23,
    LR11XX_WIFI_DATARATE_57_8_MBPS = 24,
    LR11XX_WIFI_DATARATE_65_2_MBPS = 25,
    LR11XX_WIFI_DATARATE_72_2_MBPS = 26,
} lr11xx_wifi_datarate_t;

/*!
 * @brief WiFi Frame Types
 */
typedef enum
{
    LR11XX_WIFI_FRAME_TYPE_MANAGEMENT = 0x00,
    LR11XX_WIFI_FRAME_TYPE_CONTROL    = 0x01,
    LR11XX_WIFI_FRAME_TYPE_DATA       = 0x02,
} lr11xx_wifi_frame_type_t;

/*!
 * @brief The WiFi MAC address origin
 *
 * @see lr11xx_wifi_parse_channel_info for details about the MAC address origin estimation of the LR11XX
 */
typedef enum
{
    LR11XX_WIFI_ORIGIN_BEACON_FIX_AP    = 1,  //!< MAC address extracted from a packet coming from a fix Access Point
    LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP = 2,  //!< MAC address extracted from a packet coming from a mobile Access Point
    LR11XX_WIFI_ORIGIN_UNKNOWN = 3,  //!< Impossible to determine the origin of the packet the MAC is extracted from
} lr11xx_wifi_mac_origin_t;

/*!
 * @brief Wi-Fi signal type for passive scanning configuration
 *
 * Note it is not possible to configure the WiFi passive scanning to search Wi-Fi type N GreenField. Only Wi-Fi type N
 * Mixed Mode can be scanned by LR11XX.
 *
 * @warning ::LR11XX_WIFI_TYPE_SCAN_G and ::LR11XX_WIFI_TYPE_SCAN_N configurations are implemented the same way, and
 * both will scan Wi-Fi type G **AND** Wi-Fi type N.
 */
typedef enum
{
    LR11XX_WIFI_TYPE_SCAN_B     = 0x01,  //!< Wi-Fi B
    LR11XX_WIFI_TYPE_SCAN_G     = 0x02,  //!< Wi-Fi G
    LR11XX_WIFI_TYPE_SCAN_N     = 0x03,  //!< Wi-Fi N
    LR11XX_WIFI_TYPE_SCAN_B_G_N = 0x04,  //!< Wi-Fi B and Wi-Fi G/N
} lr11xx_wifi_signal_type_scan_t;

/*!
 * @brief Wi-Fi signal type for passive scan results
 *
 * Note that the Wi-Fi N detected is Wi-Fi N Mixed mode, and not GreenField.
 */
typedef enum
{
    LR11XX_WIFI_TYPE_RESULT_B = 0x01,  //!< WiFi B
    LR11XX_WIFI_TYPE_RESULT_G = 0x02,  //!< WiFi G
    LR11XX_WIFI_TYPE_RESULT_N = 0x03,  //!< WiFi N
} lr11xx_wifi_signal_type_result_t;

/*!
 * @brief Wi-Fi scan mode
 *
 * When the LR11XX receives a Wi-Fi frame, it starts demodulating it. Depending on the scan mode selected, only some
 * Wi-Fi frame type/sub-types are to be kept. The demodulation step is stopped as soon as the LR11XX detects the current
 * Wi-Fi frame is not of the required type/sub-types. This saves scan time and consumption.
 *
 * A Wi-Fi frame is never completely demodulated. The ::LR11XX_WIFI_SCAN_MODE_FULL_BEACON uses a special configuration
 * allowing to demodulate more fields (until Frame Check Sequence field), at a price of higher scan duration and higher
 * consumption.
 *
 * @note Not all results formats are available depending on the scan mode selected. Refer to
 * @ref lr11xx_wifi_are_scan_mode_result_format_compatible to know which result formats are available depending on scan
 * mode selected.
 *
 * @see lr11xx_wifi_are_scan_mode_result_format_compatible
 */
typedef enum
{
    LR11XX_WIFI_SCAN_MODE_BEACON =
        1,  //!< Exposes Beacons and Probe Responses Access Points frames until Period Beacon field (Basic result)
    LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT =
        2,  //!< Exposes some Management Access Points frames until Period Beacon field, and some other packets frame
            //!< until third Mac Address field (Basic result)
    LR11XX_WIFI_SCAN_MODE_FULL_BEACON =
        4,  //!< Exposes Beacons and Probes Responses Access Points frames until Frame Check Sequence (FCS) field
            //!< (Extended result). In this mode, only signal type LR11XX_WIFI_TYPE_SCAN_B is executed and other signal
            //!< types are silently discarded.
    LR11XX_WIFI_SCAN_MODE_UNTIL_SSID = 5,  //!< Exposes Beacons and Probes Responses Access Points frames until the end
                                           //!< of SSID field (Extended result) - available since firmware 0x0306
} lr11xx_wifi_mode_t;

/*!
 * @brief Cumulative timings
 *
 * This structure is representing the cumulative time spent in the different modes of Wi-Fi passive scanning procedure.
 * All timings are provided in [us].
 * */
typedef struct lr11xx_wifi_cumulative_timings_s
{
    uint32_t rx_detection_us;    //!< Cumulative time spent during NFE or TOA
    uint32_t rx_correlation_us;  //!< Cumulative time spent during preamble detection
    uint32_t rx_capture_us;      //!< Cumulative time spent during signal acquisition
    uint32_t demodulation_us;    //!< Cumulative time spent during software demodulation
} lr11xx_wifi_cumulative_timings_t;

/*!
 * @brief Basic complete result structure
 *
 * The beacon period is expressed in TU (Time Unit). 1 TU is 1024 microseconds.
 */
typedef struct lr11xx_wifi_basic_complete_result_s
{
    lr11xx_wifi_datarate_info_byte_t   data_rate_info_byte;
    lr11xx_wifi_channel_info_byte_t    channel_info_byte;
    int8_t                             rssi;
    lr11xx_wifi_frame_type_info_byte_t frame_type_info_byte;
    lr11xx_wifi_mac_address_t          mac_address;
    int16_t                            phi_offset;
    uint64_t timestamp_us;  //!< Indicate the up-time of the Access Point transmitting the Beacon [us]
    uint16_t beacon_period_tu;
} lr11xx_wifi_basic_complete_result_t;

/*!
 * @brief Basic MAC, type, channel result structure
 */
typedef struct lr11xx_wifi_basic_mac_type_channel_result_s
{
    lr11xx_wifi_datarate_info_byte_t data_rate_info_byte;
    lr11xx_wifi_channel_info_byte_t  channel_info_byte;
    int8_t                           rssi;
    lr11xx_wifi_mac_address_t        mac_address;
} lr11xx_wifi_basic_mac_type_channel_result_t;

/*!
 * @brief Extended full result structure
 *
 * @note The beacon period is expressed in TU (Time Unit). 1 TU is 1024 microseconds.
 *
 * @remark When used with @ref ::LR11XX_WIFI_SCAN_MODE_UNTIL_SSID, the following field are always set to 0:
 *      - field is_fcs_ok and is_fcs_checked in fcs_check_byte structure
 *      - current_channel
 *      - country_code
 *      - io_regulation
 */
typedef struct
{
    lr11xx_wifi_datarate_info_byte_t data_rate_info_byte;
    lr11xx_wifi_channel_info_byte_t  channel_info_byte;
    int8_t                           rssi;
    uint8_t                          rate;           //!< Rate index
    uint16_t                         service;        //!< Service value
    uint16_t                         length;         //!< Length of MPDU (in microseconds for WiFi B, bytes for WiFi G)
    uint16_t                         frame_control;  //!< Frame Control structure
    lr11xx_wifi_mac_address_t        mac_address_1;
    lr11xx_wifi_mac_address_t        mac_address_2;
    lr11xx_wifi_mac_address_t        mac_address_3;
    uint64_t                         timestamp_us;  //!< Indicate the up-time of the Access Point
                                                    //!< transmitting the Beacon [us]
    uint16_t beacon_period_tu;
    uint16_t seq_control;                                 //!< Sequence Control value
    uint8_t  ssid_bytes[LR11XX_WIFI_RESULT_SSID_LENGTH];  //!< Service Set
                                                          //!< IDentifier
    lr11xx_wifi_channel_t          current_channel;       //!< Current channel indicated in the Wi-Fi frame
    lr11xx_wifi_country_code_str_t country_code;          //!< Country Code
    uint8_t                        io_regulation;         //!< Input Output Regulation
    lr11xx_wifi_fcs_info_byte_t    fcs_check_byte;        //!< Frame Check Sequence info
    int16_t                        phi_offset;
} lr11xx_wifi_extended_full_result_t;

/*!
 * @brief Wi-Fi scan result formats
 *
 * @note Result format to use depends on the scan mode selected when calling @ref lr11xx_wifi_scan or @ref
 * lr11xx_wifi_scan_time_limit API. Refer to @ref lr11xx_wifi_are_scan_mode_result_format_compatible to know which
 * result formats are available depending on scan mode selected.
 *
 * @see lr11xx_wifi_are_scan_mode_result_format_compatible
 */
typedef enum
{
    LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE,          //!< Basic complete result format: @ref
                                                       //!< lr11xx_wifi_basic_complete_result_t
    LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL,  //!<  Basic MAC/type/channel result format: @ref
                                                       //!<  lr11xx_wifi_basic_mac_type_channel_result_t
    LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL,  //!< Extended full result format: @ref lr11xx_wifi_extended_full_result_t
} lr11xx_wifi_result_format_t;

/*!
 * @brief Wi-Fi country code structure
 */
typedef struct lr11xx_wifi_country_code_s
{
    lr11xx_wifi_country_code_str_t  country_code;
    uint8_t                         io_regulation;  //!< Input Output Regulation
    lr11xx_wifi_channel_info_byte_t channel_info_byte;
    lr11xx_wifi_mac_address_t       mac_address;
} lr11xx_wifi_country_code_t;

/*!
 * @brief Wi-Fi firmware version
 */
typedef struct lr11xx_wifi_version_s
{
    uint8_t major;
    uint8_t minor;
} lr11xx_wifi_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_WIFI_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
