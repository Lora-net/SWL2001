/*!
 * \file      smtc_modem_hal.c
 *
 * \brief     Modem Hardware Abstraction Layer API implementation.
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

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_gpio.h"
#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_rng.h"
#include "smtc_hal_rtc.h"
#include "smtc_hal_trace.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_watchdog.h"

#if defined( STM32L073xx )
#include "smtc_hal_eeprom.h"
#endif
#if defined( STM32L476xx )
#include "smtc_hal_flash.h"
#include "smtc_hal_adc.h"
#endif

#include "modem_pinout.h"

// for variadic args
#include <stdio.h>
#include <stdarg.h>

// for memcpy
#include <string.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#if defined( STM32L476xx )
#define ADDR_FLASH_LORAWAN_CONTEXT ADDR_FLASH_PAGE_254
#define ADDR_FLASH_MODEM_CONTEXT ADDR_FLASH_PAGE_255
#define ADDR_FLASH_DEVNONCE_CONTEXT ADDR_FLASH_PAGE_253
#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT ADDR_FLASH_PAGE_252
#endif

#if defined( STM32L073xx )
// Data eeprom base address is 0x08080000
#define ADDR_EEPROM_LORAWAN_CONTEXT_OFFSET 0
#define ADDR_EEPROM_MODEM_CONTEXT_OFFSET 100
#define ADDR_EEPROM_DEVNONCE_CONTEXT_OFFSET 200
#define ADDR_EEPROM_CRASHLOG_OFFSET 280
#define ADDR_EEPROM_SECURE_ELEMENT_CONTEXT_OFFSET 300

#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static hal_gpio_irq_t radio_dio_irq;

#if defined( STM32L476xx )
uint8_t __attribute__( ( section( ".noinit" ) ) ) saved_crashlog[CRASH_LOG_SIZE];
volatile bool __attribute__( ( section( ".noinit" ) ) ) crashlog_available;
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Reset management ------------*/
void smtc_modem_hal_reset_mcu( void )
{
    hal_mcu_reset( );
}

/* ------------ Watchdog management ------------*/

void smtc_modem_hal_reload_wdog( void )
{
    hal_watchdog_reload( );
}

/* ------------ Time management ------------*/

uint32_t smtc_modem_hal_get_time_in_s( void )
{
    return hal_rtc_get_time_s( );
}

uint32_t smtc_modem_hal_get_compensated_time_in_s( void )
{
    return hal_rtc_get_time_s( );
}

int32_t smtc_modem_hal_get_time_compensation_in_s( void )
{
    return 0;
}

uint32_t smtc_modem_hal_get_time_in_ms( void )
{
    return hal_rtc_get_time_ms( );
}

uint32_t smtc_modem_hal_get_time_in_100us( void )
{
    return hal_rtc_get_time_100us( );
}

uint32_t smtc_modem_hal_get_radio_irq_timestamp_in_100us( void )
{
    // in lbm current implementation the call of this function is done in radio_planner radio irq handler
    // so the current time is the irq time
    return hal_rtc_get_time_100us( );
}

/* ------------ Timer management ------------*/

void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context )
{
    hal_lp_timer_start( milliseconds, &( hal_lp_timer_irq_t ){ .context = context, .callback = callback } );
}

void smtc_modem_hal_stop_timer( void )
{
    hal_lp_timer_stop( );
}

/* ------------ IRQ management ------------*/

void smtc_modem_hal_disable_modem_irq( void )
{
    hal_gpio_irq_disable( );
    hal_lp_timer_irq_disable( );
}

void smtc_modem_hal_enable_modem_irq( void )
{
    hal_gpio_irq_enable( );
    hal_lp_timer_irq_enable( );
}

/* ------------ Context saving management ------------*/

void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint8_t* buffer, const uint32_t size )
{
    switch( ctx_type )
    {
#if defined( STM32L073xx )
    case CONTEXT_MODEM:
        hal_eeprom_read_buffer( ADDR_EEPROM_MODEM_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_LR1MAC:
        hal_eeprom_read_buffer( ADDR_EEPROM_LORAWAN_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_DEVNONCE:
        hal_eeprom_read_buffer( ADDR_EEPROM_DEVNONCE_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_eeprom_read_buffer( ADDR_EEPROM_SECURE_ELEMENT_CONTEXT_OFFSET, buffer, size );
        break;
#elif defined( STM32L476xx )
    case CONTEXT_MODEM:
        hal_flash_read_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
        break;
    case CONTEXT_LR1MAC:
        hal_flash_read_buffer( ADDR_FLASH_LORAWAN_CONTEXT, buffer, size );
        break;
    case CONTEXT_DEVNONCE:
        hal_flash_read_buffer( ADDR_FLASH_DEVNONCE_CONTEXT, buffer, size );
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_flash_read_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
        break;
#endif
    default:
        mcu_panic( );
        break;
    }
}

void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, const uint8_t* buffer, const uint32_t size )
{
    switch( ctx_type )
    {
#if defined( STM32L073xx )
    case CONTEXT_MODEM:
        hal_eeprom_write_buffer( ADDR_EEPROM_MODEM_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_LR1MAC:
        hal_eeprom_write_buffer( ADDR_EEPROM_LORAWAN_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_DEVNONCE:
        hal_eeprom_write_buffer( ADDR_EEPROM_DEVNONCE_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_eeprom_write_buffer( ADDR_EEPROM_SECURE_ELEMENT_CONTEXT_OFFSET, buffer, size );
        break;
#elif defined( STM32L476xx )
    case CONTEXT_MODEM:
        hal_flash_erase_page( ADDR_FLASH_MODEM_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
        break;
    case CONTEXT_LR1MAC:
        hal_flash_erase_page( ADDR_FLASH_LORAWAN_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_LORAWAN_CONTEXT, buffer, size );
        break;
    case CONTEXT_DEVNONCE:
        hal_flash_erase_page( ADDR_FLASH_DEVNONCE_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_DEVNONCE_CONTEXT, buffer, size );
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_flash_erase_page( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
        break;
#endif
    default:
        mcu_panic( );
        break;
    }
}

/* ------------ Crashlog management ------------*/

void smtc_modem_hal_store_crashlog( uint8_t crashlog[CRASH_LOG_SIZE] )
{
#if defined( STM32L073xx )
    hal_eeprom_write_buffer( ADDR_EEPROM_CRASHLOG_OFFSET + 4, crashlog, CRASH_LOG_SIZE );
#elif defined( STM32L476xx )
    strncpy( ( char* ) saved_crashlog, ( char* ) crashlog, CRASH_LOG_SIZE );
#endif
}

void smtc_modem_hal_restore_crashlog( uint8_t crashlog[CRASH_LOG_SIZE] )
{
#if defined( STM32L073xx )
    hal_eeprom_read_buffer( ADDR_EEPROM_CRASHLOG_OFFSET + 4, crashlog, CRASH_LOG_SIZE );
#elif defined( STM32L476xx )
    memcpy( crashlog, &saved_crashlog, CRASH_LOG_SIZE );
#endif
}

void smtc_modem_hal_set_crashlog_status( bool available )
{
#if defined( STM32L073xx )
    uint8_t status = ( uint8_t ) available;
    hal_eeprom_write_buffer( ADDR_EEPROM_CRASHLOG_OFFSET, &status, 1 );
#elif defined( STM32L476xx )
    crashlog_available = available;
#endif
}

bool smtc_modem_hal_get_crashlog_status( void )
{
#if defined( STM32L073xx )
    uint8_t crashlog_available;
    hal_eeprom_read_buffer( ADDR_EEPROM_CRASHLOG_OFFSET, &crashlog_available, 1 );
    return ( bool ) crashlog_available;
#elif defined( STM32L476xx )
    return crashlog_available;
#endif
}

/* ------------ assert management ------------*/

void smtc_modem_hal_assert_fail( uint8_t* func, uint32_t line )
{
    smtc_modem_hal_store_crashlog( ( uint8_t* ) func );
    smtc_modem_hal_set_crashlog_status( true );
    smtc_modem_hal_print_trace(
        "\x1B[0;31m"  // red color
        "crash log :%s:%u\n"
        "\x1B[0m",  // revert default color
        func, line );
    smtc_modem_hal_reset_mcu( );
}

/* ------------ Random management ------------*/

uint32_t smtc_modem_hal_get_random_nb( void )
{
    return hal_rng_get_random( );
}

uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    return hal_rng_get_random_in_range( val_1, val_2 );
}

int32_t smtc_modem_hal_get_signed_random_nb_in_range( const int32_t val_1, const int32_t val_2 )
{
    return hal_rng_get_signed_random_in_range( val_1, val_2 );
}

/* ------------ Radio env management ------------*/

void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context )
{
    radio_dio_irq.pin      = RADIO_DIOX;
    radio_dio_irq.callback = callback;
    radio_dio_irq.context  = context;

    hal_gpio_irq_attach( &radio_dio_irq );
}

void smtc_modem_hal_radio_irq_clear_pending( void )
{
    hal_gpio_clear_pending_irq( RADIO_DIOX );
}

void smtc_modem_hal_start_radio_tcxo( void )
{
    // put here the code that will start the tcxo if needed
}

void smtc_modem_hal_stop_radio_tcxo( void )
{
    // put here the code that will stop the tcxo if needed
}

uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void )
{
#if defined( LR11XX ) && !defined( LR1121 )
    // lr1121 ref board does not have tcxo but only 32MHz xtal
    return 5;
#else
    return 0;
#endif
}

/* ------------ Environment management ------------*/

uint8_t smtc_modem_hal_get_battery_level( void )
{
    // Please implement according to used board
    // According to LoRaWan 1.0.4 spec:
    // 0: The end-device is connected to an external power source.
    // 1..254: Battery level, where 1 is the minimum and 254 is the maximum.
    // 255: The end-device was not able to measure the battery level.
    return 255;
}

int8_t smtc_modem_hal_get_temperature( void )
{
#if defined( STM32L073xx )
    // Please implement according to used board
    return 25;
#elif defined( STM32L476xx )
    int8_t temperature;
    hal_adc_init( );
    temperature = hal_adc_get_temp( );
    hal_adc_deinit( );
    return temperature;
#endif
}

uint8_t smtc_modem_hal_get_voltage( void )
{
#if defined( STM32L073xx )
    // Please implement according to used board
    return 165;  // 3300 mv/20
#elif defined( STM32L476xx )
    uint16_t measure_vref_mv = 0;
    hal_adc_init( );
    measure_vref_mv = hal_adc_get_vref_int( );
    hal_adc_deinit( );
    // convert voltage from mv to cloud readable (1/50V = 20mv)
    return ( uint8_t )( measure_vref_mv / 20 );
#endif
}

int8_t smtc_modem_hal_get_board_delay_ms( void )
{
#if defined( LR1121 )
    return 2;
#else
    return 1;
#endif  // LR1121
}

/* ------------ Trace management ------------*/

void smtc_modem_hal_print_trace( const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    hal_trace_print( fmt, args );
    va_end( args );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
