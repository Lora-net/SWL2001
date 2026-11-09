/**
 * @file      gnss_helpers.h
 *
 * @brief     Helper functions for GNSS driver.
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

#ifndef __GNSS_HELPERS_H__
#define __GNSS_HELPERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_read_time_status_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_read_time_error_code_enum2str( lr11xx_gnss_read_time_status_t x );

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_doppler_solver_error_code_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_doppler_solver_error_code_enum2str( lr11xx_gnss_doppler_solver_error_code_t x );

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_almanac_status_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_almanac_status_enum2str( lr11xx_gnss_almanac_status_t x );

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_demod_status_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_almanac_demod_status_enum2str( lr11xx_gnss_demod_status_t x );

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_constellation_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_constellation_enum2str( lr11xx_gnss_constellation_t x );

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_scan_mode_launched_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_scan_mode_launched_enum2str( lr11xx_gnss_scan_mode_launched_t x );

/**
 * @brief Helper function to return a string from a @ref lr11xx_gnss_message_host_status_t
 *
 * @param [in] x the enum to convert to a string
 *
 * @return a string
 */
const char* smtc_gnss_message_host_status_enum2str( lr11xx_gnss_message_host_status_t x );

/**
 * @brief Helper function to print a position without support of float formatter
 *
 * @param [in] str      Prefix string to be printed with the position
 * @param [in] position The position coordinates to be printed
 *
 * @return a string
 */
void smtc_gnss_trace_print_position( const char* str, const lr11xx_gnss_solver_assistance_position_t* position );

#ifdef __cplusplus
}
#endif

#endif  // __GNSS_HELPERS_H__

/* --- EOF ------------------------------------------------------------------ */
