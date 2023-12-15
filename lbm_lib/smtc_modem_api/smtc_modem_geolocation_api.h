/**
 * @file      smtc_modem_api.h
 *
 * @brief     Generic Modem API description
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

#ifndef SMTC_MODEM_GEOLOCATION_API_H__
#define SMTC_MODEM_GEOLOCATION_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"

#include "wifi_helpers_defs.h"
#include "gnss_helpers_defs.h"
#include "lr11xx_gnss_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief GNSS scanning modes. Configures the scanning sequence according to the use case.
 */
typedef enum
{
    SMTC_MODEM_GNSS_MODE_STATIC,  //!< Scanning mode for non moving objects
    SMTC_MODEM_GNSS_MODE_MOBILE,  //!< Scanning mode for moving objects
    __SMTC_MODEM_GNSS_MODE__SIZE  //!< Number of modes available
} smtc_modem_gnss_mode_t;

/**
 * @brief GNSS constellations to be scanned (GPS, BEIDOU or both)
 */
typedef enum
{
    SMTC_MODEM_GNSS_CONSTELLATION_GPS,         //!< Use GPS only constellation
    SMTC_MODEM_GNSS_CONSTELLATION_BEIDOU,      //!< Use BEIDOU only constellation
    SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU,  //!< Use both GPS and BEIDOU constellations
    __SMTC_MODEM_GNSS_CONSTELLATION__SIZE      //!< Number of constellation settings available
} smtc_modem_gnss_constellation_t;

/**
 * @brief GNSS Almanac update status from the almanac demodulation service (SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE
 * event)
 */
typedef enum
{
    SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN,    //!< The service could not get a status. Typically because no time
                                                      //!< is available.
    SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_COMPLETED,  //!< The almanac update is complete
    SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_NOT_COMPLETED  //!< The almanac update is not complete
} smtc_modem_gnss_almanac_update_status_t;

/**
 * @brief Send mode used by geolocation services
 */
typedef enum
{
    SMTC_MODEM_SEND_MODE_UPLINK,             //!< Scan results are directly sent over LoRaWAN
    SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD,  //!< Scan results are pushed to the Store and Forward service
    SMTC_MODEM_SEND_MODE_BYPASS,             //!< Scan results are not sent
    __SMTC_MODEM_SEND_MODE__SIZE             //!< Number of send modes available
} smtc_modem_geolocation_send_mode_t;

/**
 * @brief Wi-Fi payload format (as defined by LR1110 WiFi positioning protocol of LoRaCloud).
 */
typedef enum
{
    SMTC_MODEM_WIFI_PAYLOAD_MAC      = 0x00,  //!< Only the MAC addresses of the detected Access Points are sent
    SMTC_MODEM_WIFI_PAYLOAD_MAC_RSSI = 0x01,  //!< Both MAC address and RSSI of detected Access Points are sent
} smtc_modem_wifi_payload_format_t;

/**
 * @brief The configuration context in which a scan has been performed.
 */
typedef struct
{
    smtc_modem_gnss_mode_t mode;         //!< Scan mode that has been used (STATIC, MOBILE...)
    uint32_t               almanac_crc;  //!< Almanac CRC when the scan was performed
} smtc_modem_gnss_scan_context_t;

/**
 * @brief Description of a scan result
 */
typedef struct
{
    uint32_t timestamp;  //!< Scan timestamp (GPS time in seconds modulo 1024 weeks)
    uint8_t* nav;        //!< Pointer to NAV message result for this scan
    uint8_t  nav_size;   //!< NAV message size
    uint8_t  nb_svs;     //!< Number of Space Vehicles detected by this scan
    lr11xx_gnss_detected_satellite_t*
                                             info_svs;  //!< Pointer to information about the SVs detected. See scan_result_t for array size and format.
    lr11xx_gnss_solver_assistance_position_t aiding_position;  //!< Aiding position computed after the scan
    lr11xx_gnss_scan_mode_launched_t
             scan_mode_launched;  //!< Internal scan mode launched by the LR11xx chip (autonomous, assisted,...)
    uint32_t scan_duration_ms;    //!< Duration of the single scan in milliseconds
} smtc_modem_gnss_event_data_scan_desc_t;

/**
 * @brief The data that can be retrieved when a SMTC_MODEM_EVENT_GNSS_SCAN_DONE event occurs.
 */
typedef struct
{
    bool     is_valid;   //!< Is the scan group valid ? (enough SV detected...)
    uint8_t  token;      //!< Scan group identifier
    uint32_t timestamp;  //!< Timestamp of the last scan of the group, even if not valid. (GPS time in seconds modulo
                         //!< 1024 weeks)
    uint8_t                                nb_scans_valid;                 //!< Number of valid scans in that scan group
    smtc_modem_gnss_event_data_scan_desc_t scans[GNSS_NAVGROUP_SIZE_MAX];  //!< Descriptions of all single scan results
    uint32_t                               power_consumption_nah;  //!< Power consumption induced by this scan group
    smtc_modem_gnss_scan_context_t         context;                //!< Configuration context used for this scan
    bool                                   indoor_detected;        //!< Indicates if an indoor detection occurred
    uint32_t navgroup_duration_ms;  //!< Total duration of the navgroup (from the start of the 1st scan, to the
                                    //!< SCAN_DONE event)
} smtc_modem_gnss_event_data_scan_done_t;

/**
 * @brief The data that can be retrieved when a SMTC_MODEM_EVENT_GNSS_TERMINATED event occurs.
 */
typedef struct
{
    uint8_t
        nb_scans_sent;  //!< Number of scans which have been sent over the air or pushed to the store & forward service
} smtc_modem_gnss_event_data_terminated_t;

/**
 * @brief The data that can be retrieved when a SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE event occurs.
 */
typedef struct
{
    smtc_modem_gnss_almanac_update_status_t status_gps;     //!< Is the almanac update completed for GPS constellation
    smtc_modem_gnss_almanac_update_status_t status_beidou;  //!< Is the almanac update completed for GPS constellation
    uint8_t  update_progress_gps;              //!< Update completion percentage status for GPS constellation
    uint8_t  update_progress_beidou;           //!< Update completion percentage status for BEIDOU constellation
    uint32_t power_consumption_nah;            //!< Power consumption induced by the almanac scan and demodulation.
    uint32_t stat_nb_update_from_sat_done;     //!< Debug statistics: number of scan for almanac performed
    uint32_t stat_nb_update_from_sat_success;  //!< Debug statistics: number of scan for almanac performed with
                                               //!< successful demodulation
    uint32_t stat_nb_aborted_by_rp;  //!< Debug statistics: number of times a scan for almanac has been aborted by the
                                     //!< Radio Planner
    uint32_t stat_cumulative_timings_s;  //!< Debug statistics: Cumulative timings of the almanac demodulation service
} smtc_modem_almanac_demodulation_event_data_almanac_update_t;

/**
 * @brief The data that can be retrieved when a SMTC_MODEM_EVENT_WIFI_SCAN_DONE event occurs
 */
typedef wifi_scan_all_result_t smtc_modem_wifi_event_data_scan_done_t;

/**
 * @brief The data that can be retrieved when a SMTC_MODEM_EVENT_WIFI_TERMINATED event occurs.
 */
typedef struct
{
    uint8_t
        nb_scans_sent;  //!< Number of scans which have been sent over the air or pushed to the store & forward service
} smtc_modem_wifi_event_data_terminated_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * ------------------------ GEOLOCATION FUNCTIONS  -----------------------------
 */

/**
 * @brief Program a GNSS "scan & send" sequence to start in a given delay
 *
 * @param [in] stack_id         Stack identifier
 * @param [in] mode             Scanning mode to be used (STATIC, MOBILE...)
 * @param [in] start_delay_s    Delay before starting the scan sequence, in seconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK     Command executed without errors
 * @retval SMTC_MODEM_RC_FAIL   Failed to insert the task into the supervisor
 */
smtc_modem_return_code_t smtc_modem_gnss_scan( uint8_t stack_id, smtc_modem_gnss_mode_t mode, uint32_t start_delay_s );

/**
 * @brief Cancel the current programmed GNSS "scan & send" sequence. This command will be accepted only if the sequence
 * has not started yet.
 *
 * @param [in] stack_id         Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK     Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY   The "scan & send" sequence has already started, cannot be cancelled
 * @retval SMTC_MODEM_RC_FAIL   Failed to remove the task from the supervisor
 */
smtc_modem_return_code_t smtc_modem_gnss_scan_cancel( uint8_t stack_id );

/**
 * @brief Retrieve the data associated with the SMTC_MODEM_EVENT_GNSS_SCAN_DONE event
 *
 * @param [in]  stack_id        Stack identifier
 * @param [out] data            Description of the scan group results
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold results data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_GNSS_SCAN_DONE event pending
 */
smtc_modem_return_code_t smtc_modem_gnss_get_event_data_scan_done( uint8_t                                 stack_id,
                                                                   smtc_modem_gnss_event_data_scan_done_t* data );

/**
 * @brief Retrieve the data associated with the SMTC_MODEM_EVENT_GNSS_TERMINATED event
 *
 * @param [in]  stack_id        Stack identifier
 * @param [out] data            Status of the end of the "scan & send" sequence
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_GNSS_TERMINATED event pending
 */
smtc_modem_return_code_t smtc_modem_gnss_get_event_data_terminated( uint8_t                                  stack_id,
                                                                    smtc_modem_gnss_event_data_terminated_t* data );

/**
 * @brief Set the GNSS constellations to be used for scanning for all subsequent scans (optional)
 *
 * @param [in] stack_id         Stack identifier
 * @param [in] constellations   Constellation(s) to be used for the scans
 *
 * By default it is configured for using both GPS and BEIDOU constellations
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Unsupported constellation
 */
smtc_modem_return_code_t smtc_modem_gnss_set_constellations( uint8_t                         stack_id,
                                                             smtc_modem_gnss_constellation_t constellations );

/**
 * @brief Set the LoRaWAN port on which to send the GNSS scan results uplinks
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] port         LoRaWAN port
 *
 * By default it is set to 192 (GNSS_DEFAULT_UPLINK_PORT)
 */
smtc_modem_return_code_t smtc_modem_gnss_set_port( uint8_t stack_id, uint8_t port );

/**
 * @brief Indicates if the current scan group identifier (token) has to be kept unchanged for all subsequent "scan &
 * send" sequences. It can be used for non-mobile objects to aggregate multiple scan results and use more frames in
 * a multiframe solve to get more accurate position overtime. Calling this function will increment the NAV group
 * token, so it should be called only once for all scans to be aggregated together.
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] aggregate    Boolean to aggregate or not
 *
 * By default it is set to false, meaning that the token will change for each call to smtc_modem_gnss_scan() if the
 * result was valid
 */
void smtc_modem_gnss_scan_aggregate( uint8_t stack_id, bool aggregate );

/**
 * @brief Select the send mode of the "scan & send" sequence, by default the scan groups are sent by direct LoRaWAN
 * uplinks but it can be replace by the store and forward service, or be bypassed (no send).
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] send_mode    \ref smtc_modem_geolocation_send_mode_t
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Send mode has been correctly selected
 * @retval SMTC_MODEM_RC_INVALID    Store & Forward service is not available (not compiled)
 * compiled
 */
smtc_modem_return_code_t smtc_modem_gnss_send_mode( uint8_t stack_id, smtc_modem_geolocation_send_mode_t send_mode );

/**
 * @brief Start the GNSS almanac demodulation service. This allows the internal almanac to be updated without any
 * connection to any cloud service.
 *
 * @param [in]  stack_id  Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 */
smtc_modem_return_code_t smtc_modem_almanac_demodulation_start( uint8_t stack_id );

/**
 * @brief Set the GNSS constellations to be used for almanac demodulation
 *
 * @param [in] stack_id         Stack identifier
 * @param [in] constellations   Constellation(s) to be used for almanac demodulation
 *
 * By default it is configured for using both GPS and BEIDOU constellations
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Unsupported constellation
 */
smtc_modem_return_code_t smtc_modem_almanac_demodulation_set_constellations(
    uint8_t stack_id, smtc_modem_gnss_constellation_t constellations );

/**
 * @brief Retrieve the data associated with the SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE event
 *
 * @param [in]  stack_id        Stack identifier
 * @param [out] data            Pointer to status data of the almanac update
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE event pending
 */
smtc_modem_return_code_t smtc_modem_almanac_demodulation_get_event_data_almanac_update(
    uint8_t stack_id, smtc_modem_almanac_demodulation_event_data_almanac_update_t* data );

/**
 * @brief Program a Wi-Fi "scan & send" sequence to start in a given delay
 *
 * @param [in] stack_id         Stack identifier
 * @param [in] start_delay_s    Delay before starting the scan sequence, in seconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK     Command executed without errors
 * @retval SMTC_MODEM_RC_FAIL   Failed to insert the task into the supervisor
 */
smtc_modem_return_code_t smtc_modem_wifi_scan( uint8_t stack_id, uint32_t start_delay_s );

/**
 * @brief Cancel the current programmed Wi-Fi "scan & send" sequence. This command will be accepted only if the sequence
 * has not started yet.
 *
 * @param [in] stack_id         Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK     Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY   The "scan & send" sequence has already started, cannot be cancelled
 * @retval SMTC_MODEM_RC_FAIL   Failed to remove the task from the supervisor
 */
smtc_modem_return_code_t smtc_modem_wifi_scan_cancel( uint8_t stack_id );

/**
 * @brief Retrieve the data associated with the SMTC_MODEM_EVENT_WIFI_SCAN_DONE event
 *
 * @param [in]  stack_id        Stack identifier
 * @param [out] data            Description of the Wi-Fi scan results
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold results data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_WIFI_SCAN_DONE event pending
 */
smtc_modem_return_code_t smtc_modem_wifi_get_event_data_scan_done( uint8_t                                 stack_id,
                                                                   smtc_modem_wifi_event_data_scan_done_t* data );

/**
 * @brief Retrieve the data associated with the SMTC_MODEM_EVENT_WIFI_TERMINATED event
 *
 * @param [in]  stack_id        Stack identifier
 * @param [out] data            Status of the end of the "scan & send" sequence
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_WIFI_TERMINATED event pending
 */
smtc_modem_return_code_t smtc_modem_wifi_get_event_data_terminated( uint8_t                                  stack_id,
                                                                    smtc_modem_wifi_event_data_terminated_t* data );

/**
 * @brief Set the LoRaWAN port on which to send the Wi-Fi scan results uplinks
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] port         LoRaWAN port
 *
 * By default it is set to 197 (WIFI_DEFAULT_UPLINK_PORT)
 */
smtc_modem_return_code_t smtc_modem_wifi_set_port( uint8_t stack_id, uint8_t port );

/**
 * @brief Select the send mode of the "scan & send" sequence, by default the scan groups are sent by direct LoRaWAN
 * uplinks but it can be replace by the store and forward service, or be bypassed (no send).
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] send_mode    \ref smtc_modem_geolocation_send_mode_t
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Send mode has been correctly selected
 * @retval SMTC_MODEM_RC_INVALID    Store & Forward service is not available (not compiled)
 * compiled
 */
smtc_modem_return_code_t smtc_modem_wifi_send_mode( uint8_t stack_id, smtc_modem_geolocation_send_mode_t send_mode );

/**
 * @brief Set the format of the payload to be sent: MAC address only or MAC address with RSSI
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] format       Payload format to be used
 *
 * By default it is configured for using SMTC_MODEM_WIFI_PAYLOAD_MAC format
 */
void smtc_modem_wifi_set_payload_format( uint8_t stack_id, smtc_modem_wifi_payload_format_t format );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_GEOLOCATION_API_H__

/* --- EOF
 * ------------------------------------------------------------------ */
