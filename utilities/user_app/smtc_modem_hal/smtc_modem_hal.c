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

#include "smtc_hal_flash.h"

#include "modem_pinout.h"

// for variadic args
#include <stdio.h>
#include <stdarg.h>

// for memcpy
#include <string.h>

#if defined( SX1272 ) || defined( SX1276 )
#include "smtc_modem_utilities.h"
#include "sx127x.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define ADDR_FLASH_LORAWAN_CONTEXT ADDR_FLASH_PAGE_254
#define ADDR_FLASH_MODEM_CONTEXT ADDR_FLASH_PAGE_255
#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT ADDR_FLASH_PAGE_252
#define ADDR_FLASH_FUOTA ADDR_FLASH_PAGE_150

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if !defined( SX127X )
static hal_gpio_irq_t radio_dio_irq;
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

uint32_t smtc_modem_hal_get_time_in_ms( void )
{
    return hal_rtc_get_time_ms( );
}

uint32_t smtc_modem_hal_get_time_in_100us( void )
{
    return hal_rtc_get_time_100us( );
}

/* ------------ Timer management ------------*/

void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context )
{
    hal_lp_timer_start( HAL_LP_TIMER_ID_1, milliseconds,
                        &( hal_lp_timer_irq_t ){ .context = context, .callback = callback } );
}

void smtc_modem_hal_stop_timer( void )
{
    hal_lp_timer_stop( HAL_LP_TIMER_ID_1 );
}

/* ------------ IRQ management ------------*/

void smtc_modem_hal_disable_modem_irq( void )
{
    hal_gpio_irq_disable( );
    hal_lp_timer_irq_disable( HAL_LP_TIMER_ID_1 );
#if ( SX127X )
    hal_lp_timer_irq_disable( HAL_LP_TIMER_ID_2 );
#endif
}

void smtc_modem_hal_enable_modem_irq( void )
{
    hal_gpio_irq_enable( );
    hal_lp_timer_irq_enable( HAL_LP_TIMER_ID_1 );
#if ( SX127X )
    hal_lp_timer_irq_enable( HAL_LP_TIMER_ID_2 );
#endif
}

/* ------------ Context saving management ------------*/

void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint32_t offset, uint8_t* buffer,
                                     const uint32_t size )
{
    switch( ctx_type )
    {
    case CONTEXT_MODEM:
        hal_flash_read_buffer( ADDR_FLASH_MODEM_CONTEXT + offset, buffer, size );
        break;
    case CONTEXT_LORAWAN_STACK:
        hal_flash_read_buffer( ADDR_FLASH_LORAWAN_CONTEXT + offset, buffer, size );
        break;
    case CONTEXT_FUOTA:
        hal_flash_read_buffer( ADDR_FLASH_FUOTA + offset, buffer, size );
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_flash_read_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT + offset, buffer, size );
        break;
    default:
        mcu_panic( );
        break;
    }
}

void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer,
                                   const uint32_t size )
{
    // Offset is only used for fuota purpose and for multistack features. To avoid ram consumption the use of
    // hal_flash_read_modify_write is only done in these cases
    switch( ctx_type )
    {
    case CONTEXT_MODEM:
        hal_flash_erase_page( ADDR_FLASH_MODEM_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
        break;
    case CONTEXT_LORAWAN_STACK:
#if defined( MULTISTACK )
        hal_flash_read_modify_write( ADDR_FLASH_LORAWAN_CONTEXT + offset, buffer, size );
#else
        hal_flash_erase_page( ADDR_FLASH_LORAWAN_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_LORAWAN_CONTEXT, buffer, size );
#endif
        break;
    case CONTEXT_FUOTA:
#if defined( ALLOW_FUOTA )
        hal_flash_read_modify_write( ADDR_FLASH_FUOTA + offset, buffer, size );
#endif
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_flash_erase_page( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
        break;
    default:
        mcu_panic( );
        break;
    }
}

/* ------------ assert management ------------*/

void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line )
{
    SMTC_HAL_TRACE_ERROR( "Modem panic :%s:%u\n", func, line );
    smtc_modem_hal_reset_mcu( );
}

/* ------------ Random management ------------*/

uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    return hal_rng_get_random_in_range( val_1, val_2 );
}

/* ------------ Radio env management ------------*/

void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context )
{
#if defined( SX1272 ) || defined( SX1276 )
    sx127x_t* radio = ( sx127x_t* ) smtc_modem_get_radio_context( );

    sx127x_irq_attach( radio, callback, context );
#else
    radio_dio_irq.pin      = RADIO_DIOX;
    radio_dio_irq.callback = callback;
    radio_dio_irq.context  = context;

    hal_gpio_irq_attach( &radio_dio_irq );
#endif
}

void smtc_modem_hal_radio_irq_clear_pending( void )
{
#if defined( SX127X )
    hal_gpio_clear_pending_irq( RADIO_DIO_0 );
    hal_gpio_clear_pending_irq( RADIO_DIO_1 );
    hal_gpio_clear_pending_irq( RADIO_DIO_2 );
#else
    hal_gpio_clear_pending_irq( RADIO_DIOX );
#endif
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
    // Tcxo is present on LR1110 and LR1120 evk boards
#if defined( LR11XX )
    return 5;
#else
    return 0;
#endif
}

void smtc_modem_hal_set_ant_switch( bool is_tx_on )
{
#if defined( SX127X )
    hal_gpio_set_value( RADIO_ANTENNA_SWITCH, ( is_tx_on == true ) ? 1 : 0 );
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

int8_t smtc_modem_hal_get_board_delay_ms( void )
{
    return 1;
}

/* ------------ Trace management ------------*/

void smtc_modem_hal_print_trace( const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    hal_trace_print( fmt, args );
    va_end( args );
}

/* ------------ Fuota management ------------*/

#if defined( ALLOW_FUOTA )
uint32_t smtc_modem_hal_get_hw_version_for_fuota( void )
{
    // Example value, please fill with application value
    return 0x12345678;
}

/**
 * @brief Only use if fmp package is activated
 *
 * @return uint32_t fw version as defined in fmp Alliance package TS006-1.0.0
 */
uint32_t smtc_modem_hal_get_fw_version_for_fuota( void )
{
    // Example value, please fill with application value
    return 0x11223344;
}

/**
 * @brief Only use if fmp package is activated
 *
 * @return uint8_t fw status field as defined in fmp Alliance package TS006-1.0.0
 */
uint8_t smtc_modem_hal_get_fw_status_available_for_fuota( void )
{
    // Example value, please fill with application value
    return 3;
}

uint32_t smtc_modem_hal_get_next_fw_version_for_fuota( void )
{
    // Example value, please fill with application value
    return 0x17011973;
}
/**
 * @brief Only use if fmp package is activated
 * @param [in] fw_to_delete_version    fw_to_delete_version as described in TS006-1.0.0
 * @return uint8_t fw status field as defined in fmp Alliance package TS006-1.0.0
 */
uint8_t smtc_modem_hal_get_fw_delete_status_for_fuota( uint32_t fw_to_delete_version )
{
    if( fw_to_delete_version != smtc_modem_hal_get_next_fw_version_for_fuota( ) )
    {
        return 2;
    }
    else
    {
        return 0;
    }
}
#endif  // ALLOW_FUOTA

/* ------------ Needed for Cloud  ------------*/

int8_t smtc_modem_hal_get_temperature( void )
{
    // Please implement according to used board
    return 25;
}

uint16_t smtc_modem_hal_get_voltage_mv( void )
{
    return 3300;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
