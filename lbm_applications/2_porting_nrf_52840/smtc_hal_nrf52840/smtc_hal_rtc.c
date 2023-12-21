/*!
 * \file      smtc_hal_rtc.c
 *
 * \brief     RTC Hardware Abstraction Layer implementation
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"

#include "smtc_hal_rtc.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define SMTC_RTC_MS_TO_TICKS( ms, freq ) ( ( ( ms ) * ( freq ) ) / 1000UL )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static nrf_drv_rtc_t      rtc1             = NRFX_RTC_INSTANCE( 1 );
static uint32_t           rtc_wrap_counter = 0;
static hal_lp_timer_irq_t lptim_tmr_irq[2] = { [NRFX_RTC_INT_COMPARE0].context  = NULL,
                                               [NRFX_RTC_INT_COMPARE0].callback = NULL,
                                               [NRFX_RTC_INT_COMPARE1].context  = NULL,
                                               [NRFX_RTC_INT_COMPARE1].callback = NULL };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void rtc_handler1( nrf_drv_rtc_int_type_t int_type );
static void rtc_wakeup_handler( void* obj );

static void lfclk_config( void );
static void rtc_config( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init( void )
{
    rtc_wrap_counter = 0;
    lfclk_config( );
    rtc_config( );
}

// The RTC module features a 24-bit COUNTER,
uint32_t hal_rtc_get_time_s( void )
{
    uint32_t tmp_rtc = nrf_drv_rtc_counter_get( &rtc1 );
    uint64_t tmp     = ( ( uint64_t )( tmp_rtc ) + ( uint64_t )( ( 1 << 24 ) * rtc_wrap_counter ) ) /
                   NRFX_RTC_DEFAULT_CONFIG_FREQUENCY;
    return ( uint32_t ) tmp;
}

uint32_t hal_rtc_get_time_ms( void )
{
    uint32_t tmp_rtc = nrf_drv_rtc_counter_get( &rtc1 );
    uint64_t tmp     = ( ( ( uint64_t )( tmp_rtc ) + ( uint64_t )( ( 1 << 24 ) * rtc_wrap_counter ) ) * 1000 ) /
                   NRFX_RTC_DEFAULT_CONFIG_FREQUENCY;
    return ( uint32_t ) tmp;
}

uint32_t hal_rtc_get_time_100us( void )
{
    uint32_t tmp_rtc = nrf_drv_rtc_counter_get( &rtc1 );
    uint64_t tmp     = ( ( ( uint64_t )( tmp_rtc ) + ( uint64_t )( ( 1 << 24 ) * rtc_wrap_counter ) ) * 10000 ) /
                   NRFX_RTC_DEFAULT_CONFIG_FREQUENCY;
    return ( uint32_t ) tmp;
}

void hal_lp_timer_start( const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    uint32_t tmp_rtc = nrf_drv_rtc_counter_get( &rtc1 );
    // Setup RTC to fire during the next period
    nrfx_err_t err_code =
        nrfx_rtc_cc_set( &rtc1, NRFX_RTC_INT_COMPARE0,
                         SMTC_RTC_MS_TO_TICKS( milliseconds, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY ) + tmp_rtc, true );
    APP_ERROR_CHECK( err_code );

    lptim_tmr_irq[NRFX_RTC_INT_COMPARE0] = *tmr_irq;
}

void hal_lp_timer_stop( void )
{
    nrfx_rtc_cc_disable( &rtc1, NRFX_RTC_INT_COMPARE0 );
}

void hal_lp_timer_irq_enable( void )
{
    NVIC_EnableIRQ( RTC1_IRQn );
    // nrfx_rtc_counter_clear( &smtc_rtc[1].nrf_rtc );
    // nrfx_rtc_enable( &smtc_rtc[1].nrf_rtc );
}

void hal_lp_timer_irq_disable( void )
{
    NVIC_DisableIRQ( RTC1_IRQn );
    // nrfx_rtc_cc_disable( &rtc1, NRFX_RTC_INT_COMPARE0 );
}

void hal_rtc_wakeup_timer_set_ms( const int32_t milliseconds )
{
    uint32_t tmp_rtc = nrf_drv_rtc_counter_get( &rtc1 );
    // Setup RTC to fire during the next period
    nrfx_err_t err_code =
        nrfx_rtc_cc_set( &rtc1, NRFX_RTC_INT_COMPARE1,
                         SMTC_RTC_MS_TO_TICKS( milliseconds, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY ) + tmp_rtc, true );
    APP_ERROR_CHECK( err_code );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config( void )
{
    if( nrf_drv_clock_init_check( ) )
    {
        nrf_drv_clock_uninit( );
    }
    ret_code_t err_code = nrf_drv_clock_init( );
    APP_ERROR_CHECK( err_code );

    nrf_drv_clock_lfclk_request( NULL );

    while( !nrf_drv_clock_lfclk_is_running( ) )
    {
        // spin lock
    }
    nrfx_clock_lfclk_start( );
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config( void )
{
    uint32_t err_code;

    // Initialize RTC instance
    nrf_drv_rtc_config_t config = { .prescaler          = 0,
                                    .interrupt_priority = 6,
                                    .tick_latency = ( ( ( 2000 ) * ( NRFX_RTC_DEFAULT_CONFIG_FREQUENCY ) ) / 1000000U ),
                                    .reliable     = true };

    err_code = nrfx_rtc_init( &rtc1, &config, rtc_handler1 );
    APP_ERROR_CHECK( err_code );

    // Power on RTC instance
    nrfx_rtc_overflow_enable( &rtc1, true );
    nrfx_rtc_enable( &rtc1 );

    lptim_tmr_irq[NRFX_RTC_INT_COMPARE1] = ( hal_lp_timer_irq_t ){ .context = NULL, .callback = rtc_wakeup_handler };
}

/** @brief: Function for handling the RTC1 interrupts.
 *
 */
static void rtc_handler1( nrf_drv_rtc_int_type_t int_type )
{
    if( int_type == NRFX_RTC_INT_OVERFLOW )
    {
        rtc_wrap_counter++;
    }
    else if( int_type == NRFX_RTC_INT_COMPARE0 )
    {
        hal_lp_timer_stop( );
        if( lptim_tmr_irq[int_type].callback != NULL )
        {
            lptim_tmr_irq[int_type].callback( lptim_tmr_irq[int_type].context );
        }
    }
    else if( int_type == NRFX_RTC_INT_COMPARE1 )
    {
        if( lptim_tmr_irq[int_type].callback != NULL )
        {
            lptim_tmr_irq[int_type].callback( lptim_tmr_irq[int_type].context );
        }
    }
}

static void rtc_wakeup_handler( void* obj )
{
}