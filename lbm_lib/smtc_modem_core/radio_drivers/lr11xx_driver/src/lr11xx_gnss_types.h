/*!
 * @file      lr11xx_gnss_types.h
 *
 * @brief     GNSS scan driver types for LR11XX
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

#ifndef LR11XX_GNSS_TYPES_H
#define LR11XX_GNSS_TYPES_H

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

/*!
 * @brief Maximal buffer size
 */
#define LR11XX_GNSS_MAX_SIZE_ARRAY 2820  //!< (128sv * 22bytes + 4bytes for CRC)

/*!
 * @brief Number of almanacs in full update payload
 */
#define LR11XX_GNSS_FULL_UPDATE_N_ALMANACS ( 128 )

/*!
 * @brief Size of the almanac of a single satellite when reading
 */
#define LR11XX_GNSS_SINGLE_ALMANAC_READ_SIZE ( 22 )

/*!
 * @brief Size of the almanac of a single satellite when writing
 */
#define LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE ( 20 )

/*!
 * @brief Size of the almanac of the GNSS context status buffer
 */
#define LR11XX_GNSS_CONTEXT_STATUS_LENGTH ( 9 )

/*!
 * @brief Size of the whole almanac when reading
 */
#define LR11XX_GNSS_FULL_ALMANAC_READ_BUFFER_SIZE \
    ( ( LR11XX_GNSS_FULL_UPDATE_N_ALMANACS * LR11XX_GNSS_SINGLE_ALMANAC_READ_SIZE ) + 4 )

#define LR11XX_GNSS_DMC_ALMANAC_UPDATE_POS ( 1U )
#define LR11XX_GNSS_DMC_ALMANAC_UPDATE_GPS_MASK ( 0x01UL << LR11XX_GNSS_DMC_ALMANAC_UPDATE_POS )
#define LR11XX_GNSS_DMC_ALMANAC_UPDATE_BEIDOU_MASK ( 0x02UL << LR11XX_GNSS_DMC_ALMANAC_UPDATE_POS )

#define LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_POS ( 0U )
#define LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_MASK ( 0x01UL << LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_MSB_POS )

#define LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_POS ( 7U )
#define LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_MASK ( 0x01UL << LR11XX_GNSS_DMC_FREQUENCY_SEARCH_SPACE_LSB_POS )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Satellite ID type
 */
typedef uint8_t lr11xx_gnss_satellite_id_t;

/*!
 * @brief bit mask indicating which information is added in the output payload
 */
enum lr11xx_gnss_result_fields_e
{
    LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK = ( 1 << 0 ),  //!< Add Doppler information if set
    LR11XX_GNSS_RESULTS_DOPPLER_MASK = ( 1 << 1 ),  //!< Add up to 14 Doppler if set - up to 7 if not. Valid if @ref
                                                    //!< LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK is set
    LR11XX_GNSS_RESULTS_BIT_CHANGE_MASK =
        ( 1 << 2 ),  //!< Add bit change if set, LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS mode only
    LR11XX_GNSS_RESULTS_DEMODULATE_TIME_MASK =
        ( 1 << 3 ),  //!< Add time demodulation if set, LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS mode only
    LR11XX_GNSS_RESULTS_REMOVE_TIME_FROM_NAV_MASK = ( 1 << 4 ),  //!< Remove time from NAV if set
    LR11XX_GNSS_RESULTS_REMOVE_AP_FROM_NAV_MASK   = ( 1 << 5 ),  //!< Remove aiding position from NAV if set
};

/*!
 * @brief Constellation identifiers
 */
typedef enum
{
    LR11XX_GNSS_GPS_MASK    = 0x01,
    LR11XX_GNSS_BEIDOU_MASK = 0x02,
} lr11xx_gnss_constellation_t;

/*!
 * @brief Bit mask of constellation configurations
 *
 * @see lr11xx_gnss_constellation_t
 */
typedef uint8_t lr11xx_gnss_constellation_mask_t;

/*!
 * @brief Search mode for GNSS scan
 */
typedef enum
{
    LR11XX_GNSS_OPTION_LOW_EFFORT = 0x00,  //!< Search all requested satellites or fail, scan duration is low
    LR11XX_GNSS_OPTION_MID_EFFORT =
        0x01,  //!< Add additional search if not all satellites are found, scan duration is standard
    LR11XX_GNSS_OPTION_HIGH_EFFORT =
        0x02,  //!< Add additional search if not all satellites are found, scan duration is very high
} lr11xx_gnss_search_mode_t;

/*!
 * @brief GNSS response type indicates the destination: Host MCU, GNSS solver or GNSS DMC
 */
typedef enum
{
    LR11XX_GNSS_DESTINATION_HOST   = 0x00,  //!< Host MCU
    LR11XX_GNSS_DESTINATION_SOLVER = 0x01,  //!< GNSS Solver
    LR11XX_GNSS_DESTINATION_DMC    = 0x02,  //!< GNSS DMC
} lr11xx_gnss_destination_t;

/*!
 * @brief Message to host indicating the status of the message
 */
typedef enum
{
    LR11XX_GNSS_HOST_OK                                                            = 0x00,
    LR11XX_GNSS_HOST_UNEXPECTED_CMD                                                = 0x01,
    LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD                                             = 0x02,
    LR11XX_GNSS_HOST_INVALID_PARAMETERS                                            = 0x03,
    LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR                                    = 0x04,
    LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS                                              = 0x05,
    LR11XX_GNSS_HOST_NO_TIME                                                       = 0x06,
    LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED                                         = 0x07,
    LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD                                      = 0x08,
    LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR                                = 0x09,
    LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR                    = 0x0A,
    LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED                                    = 0x0C,
    LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR                                             = 0x0D,
    LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED                                 = 0x0E,
    LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE                 = 0x10,
    LR11XX_GNSS_HOST_TIME_DEMODULATION_FAIL                                        = 0x11,
    LR11XX_GNSS_HOST_ALMANAC_DEMODULATION_FAIL                                     = 0x12,
    LR11XX_GNSS_HOST_AT_LEAST_THE_DETECTED_SV_OF_ONE_CONSTELLATION_ARE_DEACTIVATED = 0x13,
    LR11XX_GNSS_HOST_ASSISTANCE_POSITION_POSSIBLY_WRONG_BUT_FAILS_TO_UPDATE        = 0x14,
    LR11XX_GNSS_HOST_SCAN_ABORTED                                                  = 0x15,
    LR11XX_GNSS_HOST_NAV_MESSAGE_CANNOT_BE_GENERATED_INTERVAL_GREATER_THAN_63_SEC  = 0x16,
} lr11xx_gnss_message_host_status_t;

/*!
 * @brief Message to DMC operation code
 */
typedef enum
{
    LR11XX_GNSS_DMC_STATUS = 0x18,  //!< Status message in payload
} lr11xx_gnss_message_dmc_opcode_t;

/*!
 * @brief GNSS single or double scan mode
 */
typedef enum
{
    LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY           = 0x00,  //!< Generated NAV message format = NAV3
    LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS = 0x03,  //!< Generated NAV message format = NAV3
} lr11xx_gnss_scan_mode_t;

/*!
 * @brief GNSS error codes
 */
typedef enum lr11xx_gnss_error_code_e
{
    LR11XX_GNSS_NO_ERROR                            = 0,
    LR11XX_GNSS_ERROR_ALMANAC_TOO_OLD               = 1,
    LR11XX_GNSS_ERROR_UPDATE_CRC_MISMATCH           = 2,
    LR11XX_GNSS_ERROR_UPDATE_FLASH_MEMORY_INTEGRITY = 3,
    LR11XX_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED = 4,  //!< Impossible to update more than one constellation at a time
} lr11xx_gnss_error_code_t;

/*!
 * @brief GNSS frequency search space
 */
typedef enum lr11xx_gnss_freq_search_space_e
{
    LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ = 0,
    LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ = 1,
    LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ  = 2,
    LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ  = 3,
} lr11xx_gnss_freq_search_space_t;

/*!
 * @brief GNSS fetch time option
 */
typedef enum lr11xx_gnss_fetch_time_option_e
{
    LR11XX_GNSS_SEARCH_TOW = 0,     //!< Fetch the time of week. This option can only be used either after a successful
                                    //!< GNSS scan, or after a successful call to lr11xx_gnss_fetch_time with option
                                    //!< LR11XX_GNSS_SEARCH_TOW_WN or LR11XX_GNSS_SEARCH_TOW_WN_ROLLOVER
    LR11XX_GNSS_SEARCH_TOW_WN = 1,  //!< Fetch the time of week and the week number
    LR11XX_GNSS_SEARCH_TOW_WN_ROLLOVER =
        2,  //!< Fetch the time of week, week number and week number rollover since 1980
} lr11xx_gnss_fetch_time_option_t;

/*!
 * @brief GNSS time status
 */
typedef enum lr11xx_gnss_read_time_status_e
{
    LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR       = 0,
    LR11XX_GNSS_READ_TIME_STATUS_32K_STOPPED    = 1,
    LR11XX_GNSS_READ_TIME_STATUS_WN_TOW_NOT_SET = 2
} lr11xx_gnss_read_time_status_t;

/*!
 * @brief GNSS week number number rollover status
 */
typedef enum lr11xx_gnss_week_number_rollover_status_e
{
    LR11XX_GNSS_WN_ROLLOVER_ROLLOVER_NEVER_SET   = 0,
    LR11XX_GNSS_WN_ROLLOVER_ROLLOVER_SET_BY_SCAN = 1,
} lr11xx_gnss_week_number_rollover_status_t;

/*!
 * @brief GNSS demod status
 */
typedef enum lr11xx_gnss_demod_status_e
{
    LR11XX_GNSS_NO_DEMOD_BDS_ALMANAC_SV31_43           = -21,
    LR11XX_GNSS_SV_SELECTED_FOR_DEMOD_LOST             = -20,
    LR11XX_GNSS_ALMANAC_DEMOD_ERROR                    = -19,
    LR11XX_GNSS_WAKE_UP_AFTER_PREAMBLE                 = -18,
    LR11XX_GNSS_20MS_REAL_TIME_FAILURE                 = -17,
    LR11XX_GNSS_WAKE_UP_SYNC_FAILURE                   = -16,
    LR11XX_GNSS_WEEK_NUMBER_NOT_VALIDATED              = -15,
    LR11XX_GNSS_NO_ACTIVATED_SAT_IN_SV_LIST            = -14,
    LR11XX_GNSS_SLEEP_TIME_TOO_LONG                    = -13,
    LR11XX_GNSS_WRONG_TIME_OF_WEEK_DEMOD               = -12,
    LR11XX_GNSS_PREAMBLE_NOT_VALIDATED                 = -11,
    LR11XX_GNSS_DEMOD_DISABLE                          = -10,
    LR11XX_GNSS_DEMOD_EXTRACTION_FAILURE               = -9,
    LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_START_DEMOD = -8,
    LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_MULTISCAN   = -7,
    LR11XX_GNSS_NO_SAT_FOUND                           = -6,
    LR11XX_GNSS_WORD_SYNC_LOST                         = -5,
    LR11XX_GNSS_NOT_ENOUGH_PARITY_CHECK_FOUND          = -3,
    LR11XX_GNSS_TOO_MANY_PARITY_CHECK_FOUND            = -2,
    LR11XX_GNSS_NO_PARITY_CHECK_FOUND                  = -1,
    LR11XX_GNSS_WORD_SYNC_SEARCH_NOT_STARTED           = 0,
    LR11XX_GNSS_WORD_SYNC_POTENTIALLY_FOUND            = 1,
    LR11XX_GNSS_WORD_SYNC_FOUND                        = 2,
    LR11XX_GNSS_TIME_OF_WEEK_FOUND                     = 3,
    LR11XX_GNSS_WEEK_NUMBER_FOUND                      = 4,
    LR11XX_GNSS_ALMANAC_FOUND_BUT_NO_SAVED             = 5,
    LR11XX_GNSS_HALF_ALMANAC_FOUND_AND_SAVED           = 6,
    LR11XX_GNSS_ALMANAC_FOUND_AND_SAVED                = 7,
} lr11xx_gnss_demod_status_t;

/*!
 * @brief GNSS doppler solver error code
 */
typedef enum lr11xx_gnss_doppler_solver_error_code_e
{
    LR11XX_GNSS_DOPPLER_SOLVER_NO_ERROR                          = 0,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_RESIDUE_HIGH                = 1,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONVERGED               = 2,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_ENOUGH_SV               = 3,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ILL_MATRIX                  = 4,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_TIME_ERROR                  = 5,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_PARTIAL_ALMANAC_TOO_OLD     = 6,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONSISTENT_WITH_HISTORY = 7,
    LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ALL_ALMANAC_TOO_OLD         = 8,
} lr11xx_gnss_doppler_solver_error_code_t;

/*!
 * @brief GNSS almanac status
 */
typedef enum lr11xx_gnss_almanac_status_e
{
    LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW        = -4,
    LR11XX_GNSS_NO_TIME_SET                      = -3,
    LR11XX_GNSS_IMPOSSIBLE_TO_FIND_NEXT_TIME     = -2,
    LR11XX_GNSS_NO_PAGE_ID_KNOWN                 = -1,
    LR11XX_GNSS_NO_SAT_TO_UPDATE                 = 0,
    LR11XX_GNSS_AT_LEAST_ONE_SAT_MUST_BE_UPDATED = 1,
} lr11xx_gnss_almanac_status_t;

/*!
 * @brief GNSS SV type
 */
typedef enum lr11xx_gnss_sv_type_e
{
    LR11XX_GNSS_MEO_SAT  = 0,
    LR11XX_GNSS_IGSO_SAT = 1,
} lr11xx_gnss_sv_type_t;

typedef enum
{
    LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED                   = 3,
    LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_TIME_NO_AP   = 4,
    LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_AP           = 5,
    LR11XX_GNSS_LAST_SCAN_FETCH_TIME_OR_DOPPLER_SOLVER    = 6,
    LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE                  = 7,
    LR11XX_GNSS_LAST_SCAN_KEEP_SYNC                       = 8,
    LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_1_CONSTELLATION  = 9,
    LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_2_CONSTELLATIONS = 10,
} lr11xx_gnss_scan_mode_launched_t;

/*!
 * @brief GNSS time structure
 */
typedef struct lr11xx_gnss_time_s
{
    lr11xx_gnss_read_time_status_t error_code;
    uint32_t                       gps_time_s;
    uint32_t                       nb_us_in_s;
    uint32_t                       time_accuracy;
} lr11xx_gnss_time_t;

/*!
 * @brief GNSS demod info structure
 */
typedef struct lr11xx_gnss_demod_info_s
{
    bool word_sync_found;  //!< 0: no word synchronization found / 1: a word synchronization has been found
    bool first_tow_found;  //!< 0: no Time Of Week found / 1: a Time Of Week has been found
    bool wn_demodulated;   //!< 0: no Week number demodulated / 1: a Week Number has been demodulated
    bool wn_found;         //!< 0: no Week number found  / 1: a Week Number has been found
    bool sub1_found;       //!< 0: subframe ID not found  / 1: subframe ID found
    bool sub4_found;       //!< 0: subframe ID not found  / 1: subframe ID found
    bool sub5_found;       //!< 0: subframe ID not found  / 1: subframe ID found
} lr11xx_gnss_demod_info_t;

/*!
 * @brief GNSS cumulative_timing
 */
typedef struct lr11xx_gnss_cumulative_timing_s
{
    uint32_t init;
    uint32_t phase1_gps_capture;
    uint32_t phase1_gps_process;
    uint32_t multiscan_gps_capture;
    uint32_t multiscan_gps_process;
    uint32_t multiscan_gps_sleep_32k;
    uint32_t phase1_beidou_capture;
    uint32_t phase1_beidou_process;
    uint32_t multiscan_beidou_capture;
    uint32_t multiscan_beidou_process;
    uint32_t multiscan_beidou_sleep_32k;
    uint32_t demod_capture;
    uint32_t demod_process;
    uint32_t demod_sleep_32k;
    uint32_t demod_sleep_32m;
    uint32_t total_gps_capture;
    uint32_t total_gps_process;
    uint32_t total_gps_sleep_32k;
    uint32_t total_gps_sleep_32m;
    uint32_t total_gps;
    uint32_t total_beidou_capture;
    uint32_t total_beidou_process;
    uint32_t total_beidou_sleep_32k;
    uint32_t total_beidou_sleep_32m;
    uint32_t total_beidou;
    uint32_t total_capture;
    uint32_t total_process;
    uint32_t total_sleep_32k;
    uint32_t total_sleep_32m;
    uint32_t total;
    uint32_t last_capture_size_32k_cnt;
    uint8_t  constellation_demod;
} lr11xx_gnss_cumulative_timing_t;

/*!
 * @brief GNSS instantaneous power consumption in ua
 */
typedef struct lr11xx_gnss_instantaneous_power_consumption_ua_s
{
    uint16_t board_voltage_mv;
    uint16_t init_ua;
    uint16_t phase1_gps_capture_ua;
    uint16_t phase1_gps_process_ua;
    uint16_t multiscan_gps_capture_ua;
    uint16_t multiscan_gps_process_ua;
    uint16_t phase1_beidou_capture_ua;
    uint16_t phase1_beidou_process_ua;
    uint16_t multiscan_beidou_capture_ua;
    uint16_t multiscan_beidou_process_ua;
    uint16_t sleep_32k_ua;
    uint16_t demod_sleep_32m_ua;
} lr11xx_gnss_instantaneous_power_consumption_ua_t;

/*!
 * @brief
 */
typedef struct lr11xx_gnss_doppler_solver_result_s
{
    lr11xx_gnss_doppler_solver_error_code_t error_code;
    uint8_t                                 nb_sv_used;
    uint16_t                                one_shot_latitude;
    uint16_t                                one_shot_longitude;
    uint16_t                                one_shot_accuracy;
    uint16_t                                one_shot_xtal_ppb;
    uint16_t                                filtered_latitude;
    uint16_t                                filtered_longitude;
    uint16_t                                filtered_accuracy;
    uint16_t                                filtered_xtal_ppb;
} lr11xx_gnss_doppler_solver_result_t;

/*!
 * @brief
 */
typedef struct lr11xx_gnss_read_almanac_status_s
{
    lr11xx_gnss_almanac_status_t status_gps;
    uint32_t next_gps_time_sat_to_update;  //!< Next gps time sat to update: give the duration in milliseconds before
                                           //!< the next start subframe where to catch the new almanac
    uint8_t next_gps_nb_subframe_to_demodulate;
    uint8_t next_gps_sat_id_to_update_in_sub_4;  //!< Next gps sat id to update in subframe 4: satellite number that can
                                                 //!< be demodulated in next subframe 4
    uint8_t next_gps_sat_id_to_update_in_sub_5;  //!< Next gps sat id to update in subframe 5: satellite number that can
    //!< be demodulated in next subframe 5
    uint8_t  nb_sat_gps_to_update;        //!< the number total gps and bds that needs almanac update
    uint8_t  next_gps_subframe_id_start;  //!<  Next gps subframe ID start: can be equal to 4, 5 or 0
    uint32_t sat_id_gps_to_update;  //!< Sat id gps to update: bit mask indicating which sat id almanac must be updated.
                                    //!< bit at 0 : almanac is already updated, bit at 1: almanac sat must be updated
    uint32_t sat_id_gps_activated;  //!< Sat id gps activated : bit mask indicating which sat id is activated . bit at 0
                                    //!< : sat id is not activated, bit at 1: sat id is activated
    lr11xx_gnss_almanac_status_t status_beidou;
    uint32_t next_beidou_time_sat_to_update;  //!< Next beidou time sat to update: give the duration in milliseconds
                                              //!< before the next start subframe where to catch the new almanac
    uint8_t next_beidou_nb_subframe_to_demodulate;
    uint8_t next_beidou_sat_id_to_update_in_sub_4;  //!< Next beidou sat id to update in subframe 4: satellite number
                                                    //!< that can be demodulated in next subframe 4
    uint8_t next_beidou_sat_id_to_update_in_sub_5;  //!< Next beidou sat id to update in subframe 5: satellite number
    //!< that can be demodulated in next subframe 5
    uint8_t  nb_sat_beidou_to_update;        //!< the number total gps and bds that needs almanac update
    uint8_t  next_beidou_subframe_id_start;  //!<  Next beidou subframe ID start: can be equal to 4, 5 or 0
    uint32_t sat_id_beidou_to_update[2];  //!< Sat id beidou to update: bit mask indicating which sat id almanac must be
                                          //!< updated. bit at 0 : almanac is already updated, bit at 1: almanac sat
                                          //!< must be updated
    uint32_t sat_id_beidou_activated[2];  //!< Sat id gps activated : bit mask indicating which sat id is activated .
    //!< bit at 0 : sat id is not activated, bit at 1: sat id is activated
    uint32_t sat_id_beidou_black_list[2];  //!< Sat id bds black list: bit mask indicating which bds sv does not
                                           //!< broadcast the almanac
                                           //!< bit at 0 : sat id is not black listed, bit at 1: sat id is black listed
    uint8_t next_am_id;  //!< Next AmID: For beidou only. Page 11-24 of subframe 5 are used to broadcast almanac of sat
                         //!< 31 to 63
} lr11xx_gnss_read_almanac_status_t;

/*!
 * @brief Representation of absolute time for GNSS operations
 *
 * The GNSS absolute time is represented as a 32 bits word that is the number of seconds elapsed since January 6th
 * 1980, 00:00:00
 *
 * The GNSS absolute time must take into account the Leap Seconds between UTC time and GPS time.
 */
typedef uint32_t lr11xx_gnss_date_t;

/*!
 * @brief Buffer that holds data for all almanacs full update - when reading
 */
typedef uint8_t lr11xx_gnss_almanac_full_read_bytestream_t[LR11XX_GNSS_FULL_ALMANAC_READ_BUFFER_SIZE];

/*!
 * @brief Buffer that holds data for context status
 */
typedef uint8_t lr11xx_gnss_context_status_bytestream_t[LR11XX_GNSS_CONTEXT_STATUS_LENGTH];

/*!
 * @brief Assistance position.
 */
typedef struct lr11xx_gnss_solver_assistance_position_s
{
    float latitude;   //!< Latitude 12 bits (latitude in degree * 2048/90) with resolution 0.044°
    float longitude;  //!< Longitude 12 bits (longitude in degree * 2048/180) with resolution 0.088°
} lr11xx_gnss_solver_assistance_position_t;

/*!
 * @brief Detected SV structure
 */
typedef struct lr11xx_gnss_detected_satellite_s
{
    lr11xx_gnss_satellite_id_t satellite_id;
    int8_t                     cnr;      //!< Carrier-to-noise ration (C/N) in dB
    int16_t                    doppler;  //!< SV doppler in Hz
} lr11xx_gnss_detected_satellite_t;

/*!
 * @brief Version structure of the LR11XX GNSS firmware
 */
typedef struct lr11xx_gnss_version_s
{
    uint8_t gnss_firmware;  //!< Version of the firmware
    uint8_t gnss_almanac;   //!< Version of the almanac format
} lr11xx_gnss_version_t;

/*!
 * @brief Structure for GNSS context status
 */
typedef struct lr11xx_gnss_context_status_s
{
    uint8_t                         firmware_version;
    uint32_t                        global_almanac_crc;
    lr11xx_gnss_error_code_t        error_code;
    bool                            almanac_update_gps;
    bool                            almanac_update_beidou;
    lr11xx_gnss_freq_search_space_t freq_search_space;
} lr11xx_gnss_context_status_t;

/*!
 * @brief Structure for information about visible SV
 */
typedef struct lr11xx_gnss_visible_satellite_s
{
    lr11xx_gnss_satellite_id_t satellite_id;   //!< SV ID
    int16_t                    doppler;        //!< SV doppler in Hz
    int16_t                    doppler_error;  //!< SV doppler error - step of 125Hz
} lr11xx_gnss_visible_satellite_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_GNSS_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
