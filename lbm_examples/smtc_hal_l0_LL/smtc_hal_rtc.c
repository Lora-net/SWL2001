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

#include <time.h>
#include "smtc_hal_rtc.h"

#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * Calculates ceiling( X / N )
 */
#define DIVC( X, N ) ( ( ( X ) + ( N ) - 1 ) / ( N ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* clang-format off */

// MCU Wake Up Time
#define MIN_ALARM_DELAY_IN_TICKS       3U              // in ticks

// sub-second number of bits
#define N_PREDIV_S                     10U

// Synchronous prediv
#define PREDIV_S                       ( ( 1U << N_PREDIV_S ) - 1U )

// Asynchronous prediv
#define PREDIV_A                       ( ( 1U << ( 15U - N_PREDIV_S ) ) - 1U )

// Sub-second mask definition
#define ALARM_SUBSECOND_MASK           ( N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos )

// RTC Time base in us
#define USEC_NUMBER                    1000000U
#define MSEC_NUMBER                    ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                  3U
#define CONV_NUMER                     ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                     ( 1U << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR              ( ( uint32_t ) 366U )
#define DAYS_IN_YEAR                   ( ( uint32_t ) 365U )
#define SECONDS_IN_1DAY                ( ( uint32_t ) 86400U )
#define SECONDS_IN_1HOUR               ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE             ( ( uint32_t ) 60U )
#define MINUTES_IN_1HOUR               ( ( uint32_t ) 60U )
#define HOURS_IN_1DAY                  ( ( uint32_t ) 24U )

/*!
 * Correction factors
 */
#define DAYS_IN_MONTH_CORRECTION_NORM  ( ( uint32_t ) 0x99AAA0 )
#define DAYS_IN_MONTH_CORRECTION_LEAP  ( ( uint32_t ) 0x445550 )


/**
 * @brief Date Register mask
 */
#define RTC_DR_RESERVED_MASK  ((uint32_t) (RTC_DR_YT | RTC_DR_YU | RTC_DR_WDU | \
                                           RTC_DR_MT | RTC_DR_MU | RTC_DR_DT  | \
                                           RTC_DR_DU))
/**
 * @brief Time Register mask
 */
#define RTC_TR_RESERVED_MASK  ((uint32_t) (RTC_TR_PM | RTC_TR_HT | RTC_TR_HU | \
                                           RTC_TR_MNT | RTC_TR_MNU| RTC_TR_ST | \
                                           RTC_TR_SU))

/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
static uint32_t offset_to_test_wrapping = 0;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static uint32_t rtc_get_offset_to_test_wrapping( void );
/*!
 * Converts time in ms to time in wake up timer ticks
 * Assuming WUCKSEL[2:0] = 000: RTCCLK/16 clock is selected
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval ticks Time in wake up timer ticks
 */
static uint32_t rtc_ms_2_wakeup_timer_tick( const uint32_t milliseconds );

/*!
 * Converts time in ticks to time in 100 us
 *
 * \param[IN] ticks Time in timer ticks
 * \retval milliseconds Time in 100 us
 */
static uint32_t rtc_tick_2_100us( const uint32_t tick );

/*!
 * Get the elapsed time in seconds and milliseconds since RTC initialization
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since RTC
 *                           initialization
 * \retval seconds           Number of seconds elapsed since RTC initialization
 */
static uint32_t rtc_get_calendar_time( uint16_t* milliseconds );

/*!
 * Get current full resolution RTC timestamp in ticks
 *
 * \retval timestamp_in_ticks Current timestamp in ticks
 */
static uint64_t rtc_get_timestamp_in_ticks( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init( void )
{
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_PWR );

    // Enable RTC Clock
    LL_RCC_EnableRTC( );

    // Disable the write protection for RTC registers
    LL_RTC_DisableWriteProtection( RTC );

    // Enable Direct Read of the calendar registers (not through Shadow registers)
    LL_RTC_EnableShadowRegBypass( RTC );

    // Set Initialization mode
    LL_RTC_EnableInitMode( RTC );

    // Check if the Initialization mode is set
    while( LL_RTC_IsActiveFlag_INIT( RTC ) != 1 )
    {
    }

    // Set Hour Format
    LL_RTC_SetHourFormat( RTC, LL_RTC_HOURFORMAT_24HOUR );
    // Set Asynch Prediv (value according to source clock)
    LL_RTC_SetAsynchPrescaler( RTC, PREDIV_A );
    // Set Synch Prediv (value according to source clock)
    LL_RTC_SetSynchPrescaler( RTC, PREDIV_S );

    // Set Date: Monday January 1st 2015 */
    LL_RTC_DATE_Config( RTC, LL_RTC_WEEKDAY_MONDAY, 0x01, LL_RTC_MONTH_JANUARY, 0x00 );

    // Set Time: 00:00:00 AM*/
    LL_RTC_TIME_Config( RTC, LL_RTC_TIME_FORMAT_AM_OR_24, 0x00, 0x00, 0x00 );

    // Exit Initialization mode
    LL_RTC_DisableInitMode( RTC );

    // Enable the write protection for RTC registers.
    LL_RTC_EnableWriteProtection( RTC );

    // Init exti line irq related to Wake Up Timer
    LL_EXTI_EnableIT_0_31( LL_EXTI_LINE_20 );
    LL_EXTI_EnableRisingTrig_0_31( LL_EXTI_LINE_20 );
    NVIC_SetPriority( RTC_IRQn, 0 );
    NVIC_EnableIRQ( RTC_IRQn );
}

uint32_t hal_rtc_get_time_s( void )
{
    uint16_t milliseconds = 0;
    return rtc_get_calendar_time( &milliseconds );
}

uint32_t hal_rtc_get_time_ms( void )
{
    uint32_t seconds             = 0;
    uint16_t milliseconds_div_10 = 0;

    seconds = rtc_get_calendar_time( &milliseconds_div_10 );

    return seconds * 1000 + ( milliseconds_div_10 / 10 );
}
#include "smtc_hal_dbg_trace.h"

void hal_rtc_wakeup_timer_set_ms( const int32_t milliseconds )
{
    uint32_t delay_ms_2_tick = rtc_ms_2_wakeup_timer_tick( milliseconds );

    LL_RTC_DisableWriteProtection( RTC );

    // Check if WUT is enabled
    if( LL_RTC_WAKEUP_IsEnabled( RTC ) )
    {
        // Disable wake up timer to modify it
        LL_RTC_WAKEUP_Disable( RTC );
        LL_RTC_DisableIT_WUT( RTC );

        // Wait until WUTW flag is set to 1
        while( LL_RTC_IsActiveFlag_WUTW( RTC ) != 1 )
        {
        }
    }

    // Set clock and reload value
    LL_RTC_WAKEUP_SetClock( RTC, LL_RTC_WAKEUPCLOCK_DIV_16 );
    LL_RTC_WAKEUP_SetAutoReload( RTC, delay_ms_2_tick );

    // Enable wake up counter and wake up interrupt
    LL_RTC_WAKEUP_Enable( RTC );
    LL_RTC_EnableIT_WUT( RTC );

    // Enable RTC registers write protection
    LL_RTC_EnableWriteProtection( RTC );
}

void hal_rtc_wakeup_timer_stop( void )
{
    LL_RTC_DisableWriteProtection( RTC );
    // Disable Wakeup timer interrupt
    LL_RTC_DisableIT_WUT( RTC );
    LL_RTC_WAKEUP_Disable( RTC );
}
void hal_rtc_set_offset_to_test_wrapping( uint32_t offset )
{
    offset_to_test_wrapping = offset;
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t rtc_tick_2_100us( const uint32_t tick )
{
    uint32_t seconds    = tick >> N_PREDIV_S;
    uint32_t local_tick = tick & PREDIV_S;

    return ( uint32_t ) ( ( seconds * 10000 ) + ( ( local_tick * 10000 ) >> N_PREDIV_S ) );
}

static uint32_t rtc_ms_2_wakeup_timer_tick( const uint32_t milliseconds )
{
    uint32_t nb_tick = 0;
    // Compute is done for LSE @ 32.768kHz
    // Assuming that RTC_WAKEUPCLOCK_RTCCLK_DIV16 is used => tick is 488.281µs
    nb_tick = milliseconds * 2 + ( ( 6 * milliseconds ) >> 7 );
    return nb_tick;
}

static uint32_t rtc_get_calendar_time( uint16_t* milliseconds_div_10 )
{
    uint32_t ticks;

    uint64_t timestamp_in_ticks = rtc_get_timestamp_in_ticks( ) +
                                  ( uint64_t ) ( ( ( uint64_t ) rtc_get_offset_to_test_wrapping( ) ) << N_PREDIV_S );

    uint32_t seconds = ( uint32_t ) ( timestamp_in_ticks >> N_PREDIV_S );

    ticks = ( uint32_t ) timestamp_in_ticks & PREDIV_S;

    *milliseconds_div_10 = rtc_tick_2_100us( ticks );

    return seconds;
}

static uint64_t rtc_get_timestamp_in_ticks( void )
{
    uint64_t timestamp_in_ticks = 0;
    uint32_t correction;
    uint32_t total_seconds;

    uint8_t  year = 0, month = 0, date = 0, hours = 0, minutes = 0, seconds = 0;
    uint32_t sub_seconds = 0;

    // Make sure it is correct due to asynchronous nature of RTC
    volatile uint32_t ssr;

    do
    {
        ssr         = RTC->SSR;
        sub_seconds = LL_RTC_TIME_GetSubSecond( RTC );

        // Get the DR register (Date)
        uint32_t dr_val = ( uint32_t ) ( RTC->DR & RTC_DR_RESERVED_MASK );

        // Fill the data with the read parameters
        year  = __LL_RTC_CONVERT_BCD2BIN( ( uint8_t ) ( ( dr_val & ( RTC_DR_YT | RTC_DR_YU ) ) >> 16U ) );
        month = __LL_RTC_CONVERT_BCD2BIN( ( uint8_t ) ( ( dr_val & ( RTC_DR_MT | RTC_DR_MU ) ) >> 8U ) );
        date  = __LL_RTC_CONVERT_BCD2BIN( ( uint8_t ) ( dr_val & ( RTC_DR_DT | RTC_DR_DU ) ) );

        // Get the TR register (Time)
        uint32_t tr_val = ( uint32_t ) ( RTC->TR & RTC_TR_RESERVED_MASK );

        // Fill the data with the read parameters
        hours   = __LL_RTC_CONVERT_BCD2BIN( ( uint8_t ) ( ( tr_val & ( RTC_TR_HT | RTC_TR_HU ) ) >> 16U ) );
        minutes = __LL_RTC_CONVERT_BCD2BIN( ( uint8_t ) ( ( tr_val & ( RTC_TR_MNT | RTC_TR_MNU ) ) >> 8U ) );
        seconds = __LL_RTC_CONVERT_BCD2BIN( ( uint8_t ) ( tr_val & ( RTC_TR_ST | RTC_TR_SU ) ) );

    } while( ssr != RTC->SSR );

    // Calculate amount of elapsed days since 01/01/2000
    total_seconds = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * year, 4 );

    correction = ( ( year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;

    total_seconds +=
        ( DIVC( ( month - 1 ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( month - 1 ) * 2 ) ) & 0x03 ) ) );

    total_seconds += ( date - 1 );

    // Convert from days to seconds
    total_seconds *= SECONDS_IN_1DAY;

    total_seconds += ( ( uint32_t ) seconds + ( ( uint32_t ) minutes * SECONDS_IN_1MINUTE ) +
                       ( ( uint32_t ) hours * SECONDS_IN_1HOUR ) );

    timestamp_in_ticks = ( ( ( uint64_t ) total_seconds ) << N_PREDIV_S ) + ( PREDIV_S - sub_seconds );

    return timestamp_in_ticks;
}

void RTC_IRQHandler( void )
{
    // Check WUT flag
    if( LL_RTC_IsActiveFlag_WUT( RTC ) == 1 )
    {
        // Reset Wake up flag
        LL_RTC_ClearFlag_WUT( RTC );
        // clear exti line 20 flag
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_20 );
    }
}
static uint32_t rtc_get_offset_to_test_wrapping( void )
{
    return offset_to_test_wrapping;
}
/* --- EOF ------------------------------------------------------------------ */
