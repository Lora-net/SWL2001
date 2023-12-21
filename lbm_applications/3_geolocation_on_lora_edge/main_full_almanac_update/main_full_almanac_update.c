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
#include "smtc_modem_utilities.h"

#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"

#include "lr11xx_system.h"
#include "lr11xx_gnss_types.h"

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

#define OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH 315964800

#define TIME_BUFFER_SIZE 80

/**
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS -------------------------------------------------------
 */

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

#define HAL_DBG_TRACE_INFO SMTC_HAL_TRACE_INFO
#define HAL_DBG_TRACE_ERROR SMTC_HAL_TRACE_ERROR
#define HAL_DBG_TRACE_WARNING SMTC_HAL_TRACE_WARNING
#define HAL_DBG_TRACE_PRINTF SMTC_HAL_TRACE_PRINTF
#define HAL_DBG_TRACE_ARRAY SMTC_HAL_TRACE_ARRAY

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

static void get_event( void );

/*!
 * @brief write full almanac image to LR11xx
 */
static bool almanac_update( const void* ral_context );

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
    lr11xx_system_version_t lr11xx_fw_version;
    lr11xx_status_t         status;

    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Init the modem and use get_event as event callback, please note that the callback will be
    // called immediately after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &get_event );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "FULL ALMANAC UPDATE example is starting \n" );

    /* Check LR11XX Firmware version */
    status = lr11xx_system_get_version( NULL, &lr11xx_fw_version );
    if( status != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get LR11XX firmware version\n" );
        mcu_panic( );
    }
    HAL_DBG_TRACE_INFO( "LR11XX FW: 0x%04X\n", lr11xx_fw_version.fw );

    /* Convert raw almanac date to epoch time */
    uint16_t almanac_date_raw = ( uint16_t ) ( ( full_almanac[2] << 8 ) | full_almanac[1] );
    time_t   almanac_date = ( OFFSET_BETWEEN_GPS_EPOCH_AND_UNIX_EPOCH + 24 * 3600 * ( 2048 * 7 + almanac_date_raw ) );

    /* Convert epoch time to human readbale format */
    char             buf[TIME_BUFFER_SIZE];
    const struct tm* time = localtime( &almanac_date );
    strftime( buf, TIME_BUFFER_SIZE, "%a %Y-%m-%d %H:%M:%S %Z", time );
    HAL_DBG_TRACE_PRINTF( "Source almanac date: %s\n\n", buf );

    /* Update full almanac */
    almanac_update( NULL );

    uint32_t sleep_time_ms = 0;
    while( 1 )
    {
        sleep_time_ms = smtc_modem_run_engine( );

        /* Go to sleep */
        hal_mcu_disable_irq( );
        hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
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
    SMTC_HAL_TRACE_MSG_COLOR( "get_event () callback\n", HAL_DBG_TRACE_COLOR_BLUE );

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
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

static bool get_almanac_crc( const void* ral_context, uint32_t* almanac_crc )
{
    lr11xx_status_t                         err;
    lr11xx_gnss_context_status_bytestream_t context_status_bytestream;
    lr11xx_gnss_context_status_t            context_status;

    err = lr11xx_gnss_get_context_status( ral_context, context_status_bytestream );
    if( err != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get gnss context status\n" );
        return false;
    }

    err = lr11xx_gnss_parse_context_status_buffer( context_status_bytestream, &context_status );
    if( err != LR11XX_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed to parse gnss context status to get almanac status\n" );
        return false;
    }

    *almanac_crc = context_status.global_almanac_crc;

    return true;
}

static bool almanac_update( const void* ral_context )
{
    uint32_t global_almanac_crc, local_almanac_crc;
    local_almanac_crc =
        ( full_almanac[6] << 24 ) + ( full_almanac[5] << 16 ) + ( full_almanac[4] << 8 ) + ( full_almanac[3] );

    if( get_almanac_crc( ral_context, &global_almanac_crc ) == false )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get almanac CRC before update\n" );
        return false;
    }
    if( global_almanac_crc != local_almanac_crc )
    {
        HAL_DBG_TRACE_INFO( "Local almanac doesn't match LR11XX almanac -> start update\n" );

        /* Load almanac in flash */
        uint16_t almanac_idx = 0;
        while( almanac_idx < sizeof( full_almanac ) )
        {
            if( lr11xx_gnss_almanac_update( ral_context, full_almanac + almanac_idx, 1 ) != LR11XX_STATUS_OK )
            {
                HAL_DBG_TRACE_ERROR( "Failed to update almanac\n" );
                return false;
            }
            almanac_idx += LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE;
        }

        /* Check CRC again to confirm proper update */
        if( get_almanac_crc( ral_context, &global_almanac_crc ) == false )
        {
            HAL_DBG_TRACE_ERROR( "Failed to get almanac CRC after update\n" );
            return false;
        }
        if( global_almanac_crc != local_almanac_crc )
        {
            HAL_DBG_TRACE_ERROR( "Local almanac doesn't match LR11XX almanac -> update failed\n" );
            return false;
        }
        else
        {
            HAL_DBG_TRACE_INFO( "Almanac update succeeded\n" );
        }
    }
    else
    {
        HAL_DBG_TRACE_INFO( "Local almanac matches LR11XX almanac -> no update\n" );
    }

    return true;
}

/* --- EOF ------------------------------------------------------------------ */
