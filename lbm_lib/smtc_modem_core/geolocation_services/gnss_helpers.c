/**
 * @file      gnss_helpers.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_gnss.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const char* smtc_gnss_doppler_solver_error_code_enum2str( lr11xx_gnss_doppler_solver_error_code_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_DOPPLER_SOLVER_NO_ERROR:
        return "NO ERROR";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_RESIDUE_HIGH:
        return "RESIDUE HIGH";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONVERGED:
        return "NOT CONVERGED";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_ENOUGH_SV:
        return "NOT ENOUGH SV";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ILL_MATRIX:
        return "ILL MATRIX";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_TIME_ERROR:
        return "TIME ERROR";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_PARTIAL_ALMANAC_TOO_OLD:
        return "PARTIAL ALMANAC TOO OLD";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONSISTENT_WITH_HISTORY:
        return "NOT CONSISTENT WITH HISTORY";
    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ALL_ALMANAC_TOO_OLD:
        return "ALL ALMANAC TOO OLD";
    default:
        return "???";
    }
}

const char* smtc_gnss_read_time_error_code_enum2str( lr11xx_gnss_read_time_status_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR:
        return "NO ERROR";
    case LR11XX_GNSS_READ_TIME_STATUS_32K_STOPPED:
        return "32K STOPPED";
    case LR11XX_GNSS_READ_TIME_STATUS_WN_TOW_NOT_SET:
        return "WN TOW NOT SET";
    default:
        return "???";
    }
}

const char* smtc_gnss_almanac_status_enum2str( lr11xx_gnss_almanac_status_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW:
        return "TIME ACCURACY TOO LOW";
    case LR11XX_GNSS_NO_TIME_SET:
        return "NO TIME SET";
    case LR11XX_GNSS_IMPOSSIBLE_TO_FIND_NEXT_TIME:
        return "IMPOSSIBLE TO FIND NEXT TIME";
    case LR11XX_GNSS_NO_PAGE_ID_KNOWN:
        return "NO PAGE ID KNOWN";
    case LR11XX_GNSS_NO_SAT_TO_UPDATE:
        return "NO SAT TO UPDATE";
    case LR11XX_GNSS_AT_LEAST_ONE_SAT_MUST_BE_UPDATED:
        return "AT LEAST ONE SAT MUST BE UPDATED";
    default:
        return "???";
    }
}

const char* smtc_gnss_almanac_demod_status_enum2str( lr11xx_gnss_demod_status_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_NO_DEMOD_BDS_ALMANAC_SV31_43:
        return "NO_DEMOD_BDS_ALMANAC_SV31_43";
    case LR11XX_GNSS_SV_SELECTED_FOR_DEMOD_LOST:
        return "SV_SELECTED_FOR_DEMOD_LOST";
    case LR11XX_GNSS_ALMANAC_DEMOD_ERROR:
        return "ALMANAC_DEMOD_ERROR";
    case LR11XX_GNSS_WAKE_UP_AFTER_PREAMBLE:
        return "WAKE_UP_AFTER_PREAMBLE";
    case LR11XX_GNSS_20MS_REAL_TIME_FAILURE:
        return "20MS_REAL_TIME_FAILURE";
    case LR11XX_GNSS_WAKE_UP_SYNC_FAILURE:
        return "WAKE_UP_SYNC_FAILURE";
    case LR11XX_GNSS_WEEK_NUMBER_NOT_VALIDATED:
        return "WEEK_NUMBER_NOT_VALIDATED";
    case LR11XX_GNSS_NO_ACTIVATED_SAT_IN_SV_LIST:
        return "NO_ACTIVATED_SAT_IN_SV_LIST";
    case LR11XX_GNSS_SLEEP_TIME_TOO_LONG:
        return "SLEEP_TIME_TOO_LONG";
    case LR11XX_GNSS_WRONG_TIME_OF_WEEK_DEMOD:
        return "WRONG_TIME_OF_WEEK_DEMOD";
    case LR11XX_GNSS_PREAMBLE_NOT_VALIDATED:
        return "PREAMBLE_NOT_VALIDATED";
    case LR11XX_GNSS_DEMOD_DISABLE:
        return "DEMOD_DISABLE";
    case LR11XX_GNSS_DEMOD_EXTRACTION_FAILURE:
        return "DEMOD_EXTRACTION_FAILURE";
    case LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_START_DEMOD:
        return "NO_BIT_CHANGE_FOUND_DURING_START_DEMOD";
    case LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_MULTISCAN:
        return "NO_BIT_CHANGE_FOUND_DURING_MULTISCAN";
    case LR11XX_GNSS_NO_SAT_FOUND:
        return "NO_SAT_FOUND";
    case LR11XX_GNSS_WORD_SYNC_LOST:
        return "WORD_SYNC_LOST";
    case LR11XX_GNSS_NOT_ENOUGH_PARITY_CHECK_FOUND:
        return "NOT_ENOUGH_PARITY_CHECK_FOUND";
    case LR11XX_GNSS_TOO_MANY_PARITY_CHECK_FOUND:
        return "TOO_MANY_PARITY_CHECK_FOUND";
    case LR11XX_GNSS_NO_PARITY_CHECK_FOUND:
        return "NO_PARITY_CHECK_FOUND";
    case LR11XX_GNSS_WORD_SYNC_SEARCH_NOT_STARTED:
        return "WORD_SYNC_SEARCH_NOT_STARTED";
    case LR11XX_GNSS_WORD_SYNC_POTENTIALLY_FOUND:
        return "WORD_SYNC_POTENTIALLY_FOUND";
    case LR11XX_GNSS_WORD_SYNC_FOUND:
        return "WORD_SYNC_FOUND";
    case LR11XX_GNSS_TIME_OF_WEEK_FOUND:
        return "TIME_OF_WEEK_FOUND";
    case LR11XX_GNSS_WEEK_NUMBER_FOUND:
        return "WEEK_NUMBER_FOUND";
    case LR11XX_GNSS_ALMANAC_FOUND_BUT_NO_SAVED:
        return "ALMANAC_FOUND_BUT_NO_SAVED";
    case LR11XX_GNSS_HALF_ALMANAC_FOUND_AND_SAVED:
        return "HALF_ALMANAC_FOUND_AND_SAVE";
    case LR11XX_GNSS_ALMANAC_FOUND_AND_SAVED:
        return "ALMANAC_FOUND_AND_SAVED";
    default:
        return "???";
    }
}

const char* smtc_gnss_constellation_enum2str( lr11xx_gnss_constellation_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_GPS_MASK:
        return "GPS";
    case LR11XX_GNSS_BEIDOU_MASK:
        return "BEIDOU";
    default:
        return "???";
    }
}

const char* smtc_gnss_scan_mode_launched_enum2str( lr11xx_gnss_scan_mode_launched_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED:
        return "ASSISTED";
    case LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_TIME_NO_AP:
        return "AUTONOMOUS_NO_TIME_NO_AP";
    case LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_AP:
        return "AUTONOMOUS_NO_AP";
    case LR11XX_GNSS_LAST_SCAN_FETCH_TIME_OR_DOPPLER_SOLVER:
        return "FETCH_TIME_OR_DOPPLER_SOLVER";
    case LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE:
        return "ALMANAC_UPDATE_NO_FLASH";
    case LR11XX_GNSS_LAST_SCAN_KEEP_SYNC:
        return "KEEP_SYNC";
    case LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_1_CONSTELLATION:
        return "ALMANAC_UPDATE_1_CONSTELLATION";
    case LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_2_CONSTELLATIONS:
        return "ALMANAC_UPDATE_2_CONSTELLATIONS";
    default:
        return "???";
    }
}

const char* smtc_gnss_message_host_status_enum2str( lr11xx_gnss_message_host_status_t x )
{
    switch( x )
    {
    case LR11XX_GNSS_HOST_OK:
        return "OK";
    case LR11XX_GNSS_HOST_UNEXPECTED_CMD:
        return "UNEXPECTED_CMD";
    case LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD:
        return "UNIMPLEMENTED_CMD";
    case LR11XX_GNSS_HOST_INVALID_PARAMETERS:
        return "INVALID_PARAMETERS";
    case LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR:
        return "MESSAGE_SANITY_CHECK_ERROR";
    case LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS:
        return "IQ_CAPTURE_FAILS";
    case LR11XX_GNSS_HOST_NO_TIME:
        return "NO_TIME";
    case LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED:
        return "NO_SATELLITE_DETECTED";
    case LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD:
        return "ALMANAC_IN_FLASH_TOO_OLD";
    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR:
        return "ALMANAC_UPDATE_FAILS_CRC_ERROR";
    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR:
        return "ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR";
    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED:
        return "ALMANAC_UPDATE_NOT_ALLOWED";
    case LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR:
        return "ALMANAC_CRC_ERROR";
    case LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED:
        return "ALMANAC_VERSION_NOT_SUPPORTED";
    case LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE:
        return "NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE";
    case LR11XX_GNSS_HOST_TIME_DEMODULATION_FAIL:
        return "TIME_DEMODULATION_FAIL";
    case LR11XX_GNSS_HOST_ALMANAC_DEMODULATION_FAIL:
        return "ALMANAC_DEMODULATION_FAIL";
    case LR11XX_GNSS_HOST_AT_LEAST_THE_DETECTED_SV_OF_ONE_CONSTELLATION_ARE_DEACTIVATED:
        return "AT_LEAST_THE_DETECTED_SV_OF_ONE_CONSTELLATION_ARE_DEACTIVATED";
    case LR11XX_GNSS_HOST_ASSISTANCE_POSITION_POSSIBLY_WRONG_BUT_FAILS_TO_UPDATE:
        return "ASSISTANCE_POSITION_POSSIBLY_WRONG_BUT_FAILS_TO_UPDATE";
    case LR11XX_GNSS_HOST_SCAN_ABORTED:
        return "SCAN_ABORTED";
    case LR11XX_GNSS_HOST_NAV_MESSAGE_CANNOT_BE_GENERATED_INTERVAL_GREATER_THAN_63_SEC:
        return "NAV_MESSAGE_CANNOT_BE_GENERATED_INTERVAL_GREATER_THAN_63_SE";
    default:
        return "???";
    }
}

void smtc_gnss_trace_print_position( const char* str, const lr11xx_gnss_solver_assistance_position_t* position )
{
#define N_DECIMAL_POINTS_PRECISION ( 10000 )  // n = 4. Four decimal points.

    int lat_int = ( int ) position->latitude;
    int lat_dec = ( int ) ( ( position->latitude - lat_int ) * N_DECIMAL_POINTS_PRECISION );
    if( lat_dec < 0 )
    {
        lat_dec = -lat_dec;
    }

    int lon_int = ( int ) position->longitude;
    int lon_dec = ( int ) ( ( position->longitude - lon_int ) * N_DECIMAL_POINTS_PRECISION );
    if( lon_dec < 0 )
    {
        lon_dec = -lon_dec;
    }

    SMTC_MODEM_HAL_TRACE_PRINTF( "%s(%d.%04d, %d.%04d)\n", str, lat_int, lat_dec, lon_int, lon_dec );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
