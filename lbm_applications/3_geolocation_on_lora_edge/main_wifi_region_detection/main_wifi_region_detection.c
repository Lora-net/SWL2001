/*!
 * \file      main_wifi_region_detection.c
 *
 * \brief     main program for region detection example based on WiFi scanning.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>   // strstr
#include <ctype.h>    // tolower

#include "main.h"

#include "smtc_modem_api.h"
#include "smtc_modem_geolocation_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "lr11xx_system.h"

#include "lr1110_board.h"

#include "wifi_region_finder.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Returns the minimum value between a and b
 *
 * @param [in] a 1st value
 * @param [in] b 2nd value
 * @retval Minimum value
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Stack id value (multistacks modem is not yet available)
 */
#define STACK_ID 0

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS ( 20000 )

/*!
 * @brief Time during which a LED is turned on when pulse, in [ms]
 */
#define LED_PERIOD_MS ( 250 )

/**
 * @brief Supported LR11XX radio firmware
 */
#define LR1110_FW_VERSION 0x0401
#define LR1120_FW_VERSION 0x0201

/**
 * @brief Minimum score to consider a region as detected
 */
#define REGION_DETECT_SCORE_THRESHOLD ( 3 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static wrf_region_score_t score = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void );

/**
 * @brief Read the LR11xx firmware version to ensure it is compatible with the almanac update
 */
static bool check_lr11xx_fw_version( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

static const char* region_to_string( smtc_modem_region_t region );

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_wifi_region_detection( void )
{
    uint32_t sleep_time_ms = 0;

    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    SMTC_HAL_TRACE_INFO( "Region detection from WiFi example is starting\n" );

    // Init the modem and use modem_event_callback as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &modem_event_callback );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    while( 1 )
    {
        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine( );

        // Atomically check sleep conditions
        hal_mcu_disable_irq( );
        if( smtc_modem_is_irq_flag_pending( ) == false )
        {
            hal_watchdog_reload( );
            hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
        }
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void modem_event_callback( void )
{
    smtc_modem_event_t                      current_event;
    uint8_t                                 event_pending_count;
    uint8_t                                 stack_id = STACK_ID;
    smtc_modem_wifi_event_data_scan_done_t  wifi_scan_done_data;
    smtc_modem_wifi_event_data_terminated_t wifi_terminated_data;
    smtc_modem_region_t                     region_detected             = 0;
    uint8_t                                 region_detection_confidence = 0;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        smtc_modem_get_event( &current_event, &event_pending_count );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );
            if( check_lr11xx_fw_version( ) != true )
            {
                SMTC_HAL_TRACE_ERROR( "LR11xx firmware version is not compatible with this example\n" );
                break;
            }

            /* Initialize WiFi Region Finder */
            wrf_init( );
            SMTC_HAL_TRACE_PRINTF( "\n" );
            SMTC_HAL_TRACE_INFO( "-----------------------\n" );
            SMTC_HAL_TRACE_WARNING( "Estimated region: UNKNOWN\n" );
            SMTC_HAL_TRACE_INFO( "-----------------------\n" );
            SMTC_HAL_TRACE_PRINTF( "\n" );
            /* Set GNSS and Wi-Fi send mode */
            smtc_modem_wifi_send_mode( stack_id, SMTC_MODEM_SEND_MODE_BYPASS );
            /* Program Wi-Fi scan */
            smtc_modem_wifi_set_scan_mode( stack_id, SMTC_MODEM_WIFI_SCAN_MODE_MAC_COUNTRY_CODE_SSID );
            smtc_modem_wifi_scan( stack_id, 0 );
            /* Notify user with leds */
            smtc_board_led_set( smtc_board_get_led_tx_mask( ), true );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE (%d)\n", current_event.event_data.txdone.status );
            SMTC_HAL_TRACE_INFO( "Transmission done\n" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            SMTC_HAL_TRACE_WARNING( "Join request failed \n" );
            break;

        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
            SMTC_HAL_TRACE_INFO( "Event received: TIME\n" );
            break;

        case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: GNSS_SCAN_DONE\n" );
            break;

        case SMTC_MODEM_EVENT_GNSS_TERMINATED:
            SMTC_HAL_TRACE_INFO( "Event received: GNSS_TERMINATED\n" );
            break;

        case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
            SMTC_HAL_TRACE_INFO( "Event received: GNSS_ALMANAC_DEMOD_UPDATE\n" );
            break;

        case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
            SMTC_HAL_TRACE_INFO( "Event received: WIFI_SCAN_DONE\n" );
            /* Get event data */
            smtc_modem_wifi_get_event_data_scan_done( stack_id, &wifi_scan_done_data );
            /* Analyze results with WiFi Region Finder */
            for( uint8_t j = 0; j < wifi_scan_done_data.nbr_results; ++j )
            {
                wifi_scan_single_result_t* result = &wifi_scan_done_data.results[j];

                const uint8_t* mac  = result->mac_address;
                const char*    ssid = ( const char* ) result->ssid_bytes;
                const char*    cc   = ( const char* ) result->country_code;

                wrf_process_ap( mac, ssid, cc, &score );
            }
            wrf_print_region_scores( &score );
            int ret = wrf_get_highest_score_region( &score, REGION_DETECT_SCORE_THRESHOLD, &region_detected,
                                                    &region_detection_confidence );
            SMTC_HAL_TRACE_PRINTF( "\n" );
            SMTC_HAL_TRACE_INFO( "-----------------------\n" );
            if( ret == 0 )
            {
                SMTC_HAL_TRACE_INFO( "Estimated region: %s with %d%% confidence\n", region_to_string( region_detected ),
                                     region_detection_confidence );
            }
            else
            {
                SMTC_HAL_TRACE_WARNING( "Estimated region: UNKNOWN\n" );
            }
            SMTC_HAL_TRACE_INFO( "-----------------------\n" );
            SMTC_HAL_TRACE_PRINTF( "\n" );
            break;

        case SMTC_MODEM_EVENT_WIFI_TERMINATED:
            SMTC_HAL_TRACE_INFO( "Event received: WIFI_TERMINATED\n" );
            /* Notify user with leds */
            smtc_board_led_pulse( smtc_board_get_led_tx_mask( ), true, LED_PERIOD_MS );
            /* Get event data */
            smtc_modem_wifi_get_event_data_terminated( stack_id, &wifi_terminated_data );
            /* launch next scan */
            smtc_modem_wifi_scan( stack_id, 0 );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

static const char* region_to_string( smtc_modem_region_t region )
{
    switch( region )
    {
    case SMTC_MODEM_REGION_EU_868:
        return "EU868";
    case SMTC_MODEM_REGION_US_915:
        return "US915";
    case SMTC_MODEM_REGION_CN_470:
        return "CN470";
    case SMTC_MODEM_REGION_AU_915:
        return "AU915";
    case SMTC_MODEM_REGION_KR_920:
        return "KR920";
    case SMTC_MODEM_REGION_IN_865:
        return "IN865";
    case SMTC_MODEM_REGION_RU_864:
        return "RU864";
    case SMTC_MODEM_REGION_AS_923_GRP1:
        return "AS923";
    default:
        return "UNKNOWN";
    }
}

static bool check_lr11xx_fw_version( void )
{
    lr11xx_status_t         status;
    lr11xx_system_version_t lr11xx_fw_version;

    /* suspend modem to get access to the radio */
    smtc_modem_suspend_radio_communications( true );

    status = lr11xx_system_get_version( NULL, &lr11xx_fw_version );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get LR11XX firmware version\n" );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }

    if( ( lr11xx_fw_version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1110 ) &&
        ( lr11xx_fw_version.fw < LR1110_FW_VERSION ) )
    {
        SMTC_HAL_TRACE_ERROR( "Wrong LR1110 firmware version, expected 0x%04X, got 0x%04X\n", LR1110_FW_VERSION,
                              lr11xx_fw_version.fw );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }
    if( ( lr11xx_fw_version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1120 ) &&
        ( lr11xx_fw_version.fw < LR1120_FW_VERSION ) )
    {
        SMTC_HAL_TRACE_ERROR( "Wrong LR1120 firmware version, expected 0x%04X, got 0x%04X\n", LR1120_FW_VERSION,
                              lr11xx_fw_version.fw );
        smtc_modem_suspend_radio_communications( false );
        return false;
    }

    /* release radio to the modem */
    smtc_modem_suspend_radio_communications( false );
    SMTC_HAL_TRACE_INFO( "LR11XX FW: 0x%04X, type: 0x%02X\n", lr11xx_fw_version.fw, lr11xx_fw_version.type );
    return true;
}

/* --- EOF ------------------------------------------------------------------ */
