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
 * @brief Bit mask indicating which information is added in the output payload - to be used with @ref
 * LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY
 */
enum lr11xx_gnss_result_fields_legacy_e
{
    LR11XX_GNSS_RESULTS_LEGACY_PSEUDO_RANGE_MASK = ( 1 << 0 ),  //!< Add pseudo-range information if set
    LR11XX_GNSS_RESULTS_LEGACY_DOPPLER_MASK      = ( 1 << 1 ),  //!< Add all Doppler information if set - up to 5 if not
    LR11XX_GNSS_RESULTS_LEGACY_BIT_CHANGE_MASK   = ( 1 << 2 ),  //!< Add bit change if set
};

/*!
 * @brief bit mask indicating which information is added in the output payload - to be used with @ref
 * LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS
 */
enum lr11xx_gnss_result_fields_e
{
    LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK = ( 1 << 0 ),  //!< Add Doppler information if set
    LR11XX_GNSS_RESULTS_DOPPLER_MASK = ( 1 << 1 ),     //!< Add up to 14 Doppler if set - up to 7 if not. Valid if @ref
                                                       //!< LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK is set
    LR11XX_GNSS_RESULTS_BIT_CHANGE_MASK = ( 1 << 2 ),  //!< Add bit change if set
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
    LR11XX_GNSS_OPTION_DEFAULT     = 0x00,  //!< Search all requested satellites or fail
    LR11XX_GNSS_OPTION_BEST_EFFORT = 0x01,  //!< Add additional search if not all satellites are found
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
    LR11XX_GNSS_HOST_OK                                            = 0x00,
    LR11XX_GNSS_HOST_UNEXPECTED_CMD                                = 0x01,
    LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD                             = 0x02,
    LR11XX_GNSS_HOST_INVALID_PARAMETERS                            = 0x03,
    LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR                    = 0x04,
    LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS                              = 0x05,
    LR11XX_GNSS_HOST_NO_TIME                                       = 0x06,
    LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED                         = 0x07,
    LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD                      = 0x08,
    LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR                = 0x09,
    LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR    = 0x0A,
    LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED                    = 0x0B,
    LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR                             = 0x0C,
    LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED                 = 0x0D,
    LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE = 0x10,
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
    LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY           = 0x00,  //!< Generated NAV message format = NAV1
    LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS = 0x03,  //!< Generated NAV message format = NAV2
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
 * @brief Representation of absolute time for GNSS operations
 *
 * The GNSS absolute time is represented as a 32 bits word that is the number of seconds elapsed since January 6th 1980,
 * 00:00:00
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
 * @brief GNSS timings of the LR11XX
 */
typedef struct lr11xx_gnss_timings_s
{
    uint32_t radio_ms;
    uint32_t computation_ms;
} lr11xx_gnss_timings_t;

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_GNSS_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
