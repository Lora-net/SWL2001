/*!
 * \file      main_full_almanac_update.c
 *
 * \brief     main program for geolocation full almanac update example
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
#include <string.h>
#include <time.h>

#include "main.h"

#include "smtc_modem_api.h"
#include "smtc_modem_geolocation_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"

#include "lr11xx_system.h"

#include "almanac.h"

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
#define STACK_ID ( 0 )

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS ( 20000 )

#define OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH ( 315964800 )

#define TIME_BUFFER_SIZE ( 80 )

/**
 * @brief Supported LR11XX radio firmware
 */
#define LR1110_FW_VERSION 0x0401
#define LR1120_FW_VERSION 0x0201

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS -------------------------------------------------------
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

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void get_event( void );

/**
 * @brief Read the LR11xx firmware version to ensure it is compatible with the almanac update
 */
static bool check_lr11xx_fw_version( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_full_almanac_update( void )
{
    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    SMTC_HAL_TRACE_INFO( "FULL ALMANAC UPDATE example is starting\n" );

    // Init the modem and use get_event as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &get_event );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    uint32_t sleep_time_ms = 0;
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
static void get_event( void )
{
    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;

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

            smtc_modem_return_code_t rc =
                smtc_modem_almanac_full_update( STACK_ID, full_almanac, sizeof( full_almanac ) );
            if( rc == SMTC_MODEM_RC_OK )
            {
                /* Convert raw almanac date to epoch time */
                uint16_t almanac_date_raw = ( uint16_t ) ( ( full_almanac[2] << 8 ) | full_almanac[1] );
                time_t   almanac_date =
                    ( OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH + 24 * 3600 * ( 2048 * 7 + almanac_date_raw ) );
                /* Convert epoch time to human readbale format */
                char             buf[TIME_BUFFER_SIZE];
                const struct tm* time = localtime( &almanac_date );
                strftime( buf, TIME_BUFFER_SIZE, "%a %Y-%m-%d %H:%M:%S %Z", time );

                SMTC_HAL_TRACE_INFO( "SUCESS: almanac source date %s\n", buf );
            }
            else
            {
                SMTC_HAL_TRACE_ERROR( "FAILED\n" );
            }
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
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
