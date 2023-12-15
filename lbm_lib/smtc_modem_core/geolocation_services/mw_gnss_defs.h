/**
 * @file      mw_gnss_scan.h
 *
 * @brief     GNSS middleware
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

#ifndef MW_GNSS_DEFS_H
#define MW_GNSS_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_gnss.h"
#include "lr11xx_gnss_types.h"

#include "gnss_helpers_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Maximal number of Space Vehicles to be included in a NAV message
 */
#define GNSS_RESULT_NB_SVS_MAX ( 10 )

/**
 * @brief Maximum size for a NAV message
 */
#define GNSS_RESULT_SIZE_MAX ( 49 + 1 ) /* 1 byte for destination byte */

/**
 * @brief Maximum number of Space Vehicles per constellation
 */
#define GNSS_NB_SVS_PER_CONSTELLATION_MAX ( 12 )

/**
 * @brief Number of bytes of the metadata field concerning a GNSS scan
 */
#define GNSS_SCAN_METADATA_SIZE ( 1 )

/**
 * @brief Enable power consumption debug info (verbose)
 */
#define GNSS_ENABLE_POWER_DEBUG_INFO ( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Description of a single scan in the scan group
 */
typedef struct scan_result_s
{
    uint32_t start_time_ms;  //!< local time at which a scan has started, in milliseconds
    uint32_t end_time_ms;    //!< local time at which a scan has ended, in milliseconds
    uint32_t gps_timestamp;  //!< GPS time (number of seconds modulo 1024 weeks) at which the scan has completed
    uint8_t  results_size;   //!< Size of the result (NAV)
    uint8_t  results_buffer[GNSS_SCAN_METADATA_SIZE +
                           GNSS_RESULT_SIZE_MAX];                //!< Buffer containing scan result (NAV message)
    bool     indoor_detected;                                     //!< Boolean for indoor detection
    lr11xx_gnss_solver_assistance_position_t aiding_position;     //!< The aiding position computed with this scan
    lr11xx_gnss_scan_mode_launched_t         scan_mode_launched;  //!< The internal scan mode used by the LR11xx
    uint8_t                                  nb_detected_svs;     //!< Number of Space Vehicles detected during the scan
    lr11xx_gnss_detected_satellite_t
             info_svs[2 * GNSS_NB_SVS_PER_CONSTELLATION_MAX];  //!< Information about each SV detected (ID, CNR...)
    uint32_t power_consumption_nah;                            //!< Power consumption induced by this single scan
} scan_result_t;

/**
 * @brief Description of a scan group (or NAV group)
 */
typedef struct navgroup_s
{
    uint8_t       token;
    scan_result_t scans[GNSS_NAVGROUP_SIZE_MAX];  //!< Scan results. The actual size populated is nb_scans_valid
    uint32_t
             timestamp;  //!< GPS time of the last scan tentative in the scan group (number of seconds modulo 1024 weeks)
    uint8_t  nb_scans_valid;  //!< Number of successful scans
    uint8_t  nb_scans_total;  //!< Total number of scans performed to build this scan group
    uint32_t start_time_ms;   //!< local start time of the first scan of the navgroup
    uint32_t end_time_ms;     //!< local end time of the last scan of the navgroup
} navgroup_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // MW_GNSS_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
