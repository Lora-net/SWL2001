/*!
 * \file      mw_common.c
 *
 * \brief     Middleware common functions implementation.
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <stddef.h>

#include "mw_common.h"
#include "geolocation_bsp.h"

#include "lr11xx_system.h"

#include "smtc_modem_api.h"
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

static bool configure_for_scan_done = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool mw_radio_configure_for_scan( const void* radio_context )
{
    lr11xx_status_t           status;
    lr11xx_system_errors_t    errors;
    lr11xx_system_lfclk_cfg_t lf_clock_cfg = geolocation_bsp_get_lr11xx_lf_clock_cfg( );

    // LF clock must not be reconfigured (GNSS)
    if( configure_for_scan_done == true )
    {
        return true;
    }

    // Clear potential old errors
    status = lr11xx_system_clear_errors( radio_context );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to clear error\n" );
        return false;
    }

    // Configure lf clock
    status = lr11xx_system_cfg_lfclk( radio_context, lf_clock_cfg, true );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to config lfclk\n" );
        return false;
    }

    // Get errors
    status = lr11xx_system_get_errors( radio_context, &errors );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fail to get lr11xx error\n" );
        return false;
    }

    // In case clock config is XTAL check if there is no LF_XOSC_START error
    if( ( lf_clock_cfg == LR11XX_SYSTEM_LFCLK_XTAL ) &&
        ( ( errors & LR11XX_SYSTEM_ERRORS_LF_XOSC_START_MASK ) == LR11XX_SYSTEM_ERRORS_LF_XOSC_START_MASK ) )
    {
        // lr11xx specification is telling to reset the radio to fix this error
        return false;
    }

    configure_for_scan_done = true;
    SMTC_MODEM_HAL_TRACE_PRINTF( "radio configure for scan done\n" );

    return true;
}

void mw_radio_set_sleep( const void* radio_context )
{
    lr11xx_system_sleep_cfg_t radio_sleep_cfg = { 0 };

    radio_sleep_cfg.is_warm_start  = true;
    radio_sleep_cfg.is_rtc_timeout = true;
    if( lr11xx_system_set_sleep( radio_context, radio_sleep_cfg, 0 ) != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to set the radio to sleep\n" );
    }
}

/* --- EOF ------------------------------------------------------------------ */
