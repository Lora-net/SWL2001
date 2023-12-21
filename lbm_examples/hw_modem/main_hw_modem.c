/**
 * @file      main_hw_modem.c
 *
 * @brief     Main HW Modem application
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

#include "main.h"

#include "git_version.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "smtc_modem_api.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_watchdog.h"

#include "hw_modem.h"
#include "smtc_modem_utilities.h"
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
 * @brief Watchdog counter reload value during sleep (The period must be lower than MCU watchdog period (here 32s))
 */
#define WATCHDOG_RELOAD_PERIOD_MS 20000
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

/**
 * @brief Main function for hw modem use
 */
void main_hw_modem( void )
{
    hal_mcu_disable_irq( );

    hal_mcu_init( );

    hw_modem_init( );

    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "Modem is starting\n" );
    SMTC_HAL_TRACE_PRINTF( "Commit SHA1: %s\n", get_software_git_commit( ) );
    SMTC_HAL_TRACE_PRINTF( "Commit date: %s\n", get_software_git_date( ) );
    SMTC_HAL_TRACE_PRINTF( "Build date: %s\n", get_software_build_date( ) );

    uint32_t sleep_time_ms = 0;
    while( 1 )
    {
        // Check if a command is available
        if( hw_modem_is_a_cmd_available( ) == true )
        {
            // Command may generate work for the stack, so drop down to smtc_modem_run_engine().
            hw_modem_process_cmd( );
        }

        // Modem process launch
        sleep_time_ms = smtc_modem_run_engine( );

        // Atomically check sleep conditions (no command available and low power is possible)
        hal_mcu_disable_irq( );
        if( ( hw_modem_is_a_cmd_available( ) == false ) && ( hw_modem_is_low_power_ok( ) == true ) &&
            ( smtc_modem_is_irq_flag_pending( ) == false ) )
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

/* --- EOF ------------------------------------------------------------------ */
