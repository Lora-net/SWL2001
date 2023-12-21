/*!
 * \file      main_thread_x.c
 *
 * \brief     main program for exti example
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
#include "modem_pinout.h"
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"
#include "example_options.h"
#include "hw_modem.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"
#include "app_threadx.h"
#include <string.h>

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

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return SMTC_MODEM_RC_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref smtc_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

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



/**
 * @brief callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_thread_x( void )
{
    // Disable IRQ to avoid unwanted behavior during init
    hal_mcu_disable_irq( );
    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );
#if defined( HW_MODEM_ENABLED )
    // Init hw modem (emulate an at cmd modem)
    hw_modem_init( );
#endif
    // Init lbm
    //smtc_modem_init( &modem_empty_callback );
    // start threadx
   
    mx_threadx_init( );
}



/* --- EOF ------------------------------------------------------------------ */
