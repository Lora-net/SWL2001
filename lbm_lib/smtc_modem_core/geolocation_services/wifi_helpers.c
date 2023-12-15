/**
 * @file      wifi_helpers.c
 *
 * @brief     Interface between the WI-Fi middleware and the LR11xx radio driver.
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

#include <string.h>

#include "smtc_modem_hal_dbg_trace.h"

#include "lr11xx_wifi.h"
#include "lr11xx_system.h"

#include "wifi_helpers.h"

#include "geolocation_bsp.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define TIMESTAMP_AP_PHONE_FILTERING 86400  // 24 hours in second

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

static wifi_settings_t settings = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_RESULTS];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_wifi_settings_init( const wifi_settings_t* wifi_settings )
{
    /* Set current context Wi-Fi settings */
    memcpy( &settings, wifi_settings, sizeof settings );
}

mw_return_code_t smtc_wifi_start_scan( const void* radio_context )
{
    lr11xx_status_t status;

    MW_RETURN_ON_FAILURE( lr11xx_system_set_dio_irq_params( radio_context, LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE,
                                                            LR11XX_SYSTEM_IRQ_NONE ) == LR11XX_STATUS_OK );

    MW_RETURN_ON_FAILURE( lr11xx_wifi_cfg_timestamp_ap_phone( radio_context, TIMESTAMP_AP_PHONE_FILTERING ) ==
                          LR11XX_STATUS_OK );

    /* Enable Wi-Fi path */
    geolocation_bsp_wifi_prescan_actions( );

    status = lr11xx_wifi_scan_time_limit( radio_context, settings.types, settings.channels,
                                          LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT, settings.max_results,
                                          settings.timeout_per_channel, settings.timeout_per_scan );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to start Wi-Fi scan\n" );
        geolocation_bsp_wifi_postscan_actions( );
        return MW_RC_FAILED;
    }

    return MW_RC_OK;
}

void smtc_wifi_scan_ended( void )
{
    /* Disable the Wi-Fi path */
    geolocation_bsp_wifi_postscan_actions( );
}

mw_return_code_t smtc_wifi_get_results( const void* radio_context, wifi_scan_all_result_t* wifi_results )
{
    uint8_t nb_results;
    uint8_t max_nb_results;

    memset( wifi_results_mac_addr, 0, sizeof wifi_results_mac_addr );

    MW_RETURN_ON_FAILURE( lr11xx_wifi_get_nb_results( radio_context, &nb_results ) == LR11XX_STATUS_OK );

    /* check if the array is big enough to hold all results */
    max_nb_results = sizeof( wifi_results_mac_addr ) / sizeof( wifi_results_mac_addr[0] );
    if( nb_results > max_nb_results )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Wi-Fi scan result size exceeds %u (%u)\n", max_nb_results, nb_results );
        return MW_RC_FAILED;
    }

    MW_RETURN_ON_FAILURE( lr11xx_wifi_read_basic_complete_results( radio_context, 0, nb_results,
                                                                   wifi_results_mac_addr ) == LR11XX_STATUS_OK );

    /* add scan to results */
    for( uint8_t index = 0; index < nb_results; index++ )
    {
        const lr11xx_wifi_basic_complete_result_t* local_basic_result = &wifi_results_mac_addr[index];
        lr11xx_wifi_channel_t                      channel;
        bool                                       rssi_validity;
        lr11xx_wifi_mac_origin_t                   mac_origin_estimation;

        lr11xx_wifi_parse_channel_info( local_basic_result->channel_info_byte, &channel, &rssi_validity,
                                        &mac_origin_estimation );

        wifi_results->results[index].channel = channel;
        wifi_results->results[index].origin  = mac_origin_estimation;
        wifi_results->results[index].type =
            lr11xx_wifi_extract_signal_type_from_data_rate_info( local_basic_result->data_rate_info_byte );
        memcpy( wifi_results->results[index].mac_address, local_basic_result->mac_address,
                LR11XX_WIFI_MAC_ADDRESS_LENGTH );
        wifi_results->results[index].rssi = local_basic_result->rssi;
        wifi_results->nbr_results++;
    }

    return MW_RC_OK;
}

mw_return_code_t smtc_wifi_get_power_consumption( const void* radio_context, uint32_t* power_consumption_nah )
{
    lr11xx_wifi_cumulative_timings_t timing;
    lr11xx_system_reg_mode_t         reg_mode;

    MW_RETURN_ON_FAILURE( lr11xx_wifi_read_cumulative_timing( radio_context, &timing ) == LR11XX_STATUS_OK );

    geolocation_bsp_get_lr11xx_reg_mode( radio_context, &reg_mode );
    *power_consumption_nah = ( uint32_t )( lr11xx_wifi_get_consumption_nah( reg_mode, timing ) );

    /* Accumulate timings until there is a significant amount of energy consumed */
    MW_RETURN_ON_FAILURE( lr11xx_wifi_reset_cumulative_timing( radio_context ) == LR11XX_STATUS_OK );

    return MW_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
