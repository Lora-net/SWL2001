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
#endif

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

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#if defined( STM32L476xx )
#define ADDR_FLASH_FUOTA ADDR_FLASH_PAGE_150
#define ADDR_FLASH_STORE_AND_FORWARD ADDR_FLASH_PAGE_200
#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT ADDR_FLASH_PAGE_252
#define ADDR_FLASH_MODEM_CONTEXT ADDR_FLASH_PAGE_253
#define ADDR_FLASH_LORAWAN_CONTEXT ADDR_FLASH_PAGE_254
#define ADDR_FLASH_MODEM_KEY_CONTEXT ADDR_FLASH_PAGE_255
#endif

#if defined( STM32L073xx )
// Data eeprom base address is 0x08080000

#define ADDR_EEPROM_LORAWAN_CONTEXT_OFFSET 0  // in case of multistack the size of the lorawan context shall be extended
#define ADDR_EEPROM_MODEM_KEY_CONTEXT_OFFSET 50
#define ADDR_EEPROM_MODEM_CONTEXT_OFFSET 75
#define ADDR_EEPROM_SECURE_ELEMENT_CONTEXT_OFFSET 100
#endif

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

__attribute__( ( section( ".noinit" ) ) ) static uint8_t          crashlog_buff_noinit[CRASH_LOG_SIZE];
__attribute__( ( section( ".noinit" ) ) ) static volatile uint8_t crashlog_length_noinit;
__attribute__( ( section( ".noinit" ) ) ) static volatile bool    crashlog_available_noinit;

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

void smtc_modem_hal_set_offset_to_test_wrapping( const uint32_t offset_to_test_wrapping )
{
    hal_rtc_set_offset_to_test_wrapping( offset_to_test_wrapping );
}
/* ------------ Timer management ------------*/

void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context )
{
    hal_lp_timer_start( HAL_LP_TIMER_ID_1, milliseconds,
                        &( hal_lp_timer_irq_t ) { .context = context, .callback = callback } );
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
    // Offset is only used for fuota and store and forward purpose and for multistack features. To avoid ram consumption
    // the use of hal_flash_read_modify_write is only done in these cases
    switch( ctx_type )
    {
#if defined( STM32L073xx )
    case CONTEXT_MODEM:
        hal_eeprom_read_buffer( ADDR_EEPROM_MODEM_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_KEY_MODEM:
        hal_eeprom_read_buffer( ADDR_EEPROM_MODEM_KEY_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_LORAWAN_STACK:
        hal_eeprom_read_buffer( ADDR_EEPROM_LORAWAN_CONTEXT_OFFSET + offset, buffer, size );
        break;
    case CONTEXT_FUOTA:
        // no fuota example on stm32l0
        break;
    case CONTEXT_STORE_AND_FORWARD:
        // no store and fw example on stm32l0
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_eeprom_read_buffer( ADDR_EEPROM_SECURE_ELEMENT_CONTEXT_OFFSET, buffer, size );
        break;
#elif defined( STM32L476xx )
    case CONTEXT_MODEM:
        hal_flash_read_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
        break;
    case CONTEXT_KEY_MODEM:
        hal_flash_read_buffer( ADDR_FLASH_MODEM_KEY_CONTEXT, buffer, size );
        break;
    case CONTEXT_LORAWAN_STACK:
        hal_flash_read_buffer( ADDR_FLASH_LORAWAN_CONTEXT + offset, buffer, size );
        break;
    case CONTEXT_FUOTA:
        hal_flash_read_buffer( ADDR_FLASH_FUOTA + offset, buffer, size );
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_flash_read_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
        break;
    case CONTEXT_STORE_AND_FORWARD:
        hal_flash_read_buffer( ADDR_FLASH_STORE_AND_FORWARD + offset, buffer, size );
        break;
#endif
    default:
        mcu_panic( );
        break;
    }
}

void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer,
                                   const uint32_t size )
{
    // Offset is only used for fuota and store and forward purpose and for multistack features. To avoid ram consumption
    // the use of hal_flash_read_modify_write is only done in these cases
    switch( ctx_type )
    {
#if defined( STM32L073xx )
    case CONTEXT_MODEM:
        hal_eeprom_write_buffer( ADDR_EEPROM_MODEM_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_KEY_MODEM:
        hal_eeprom_write_buffer( ADDR_EEPROM_MODEM_KEY_CONTEXT_OFFSET, buffer, size );
        break;
    case CONTEXT_LORAWAN_STACK:
        hal_eeprom_write_buffer( ADDR_EEPROM_LORAWAN_CONTEXT_OFFSET + offset, buffer, size );
        break;
    case CONTEXT_FUOTA:
        // no fuota example on stm32l0
        break;
    case CONTEXT_STORE_AND_FORWARD:
        // no store and fw example on stm32l0
        break;
    case CONTEXT_SECURE_ELEMENT:
        hal_eeprom_write_buffer( ADDR_EEPROM_SECURE_ELEMENT_CONTEXT_OFFSET, buffer, size );
        break;
#elif defined( STM32L476xx )
    case CONTEXT_MODEM:
        hal_flash_erase_page( ADDR_FLASH_MODEM_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_MODEM_CONTEXT, buffer, size );
        break;
    case CONTEXT_KEY_MODEM:
        hal_flash_erase_page( ADDR_FLASH_MODEM_KEY_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_MODEM_KEY_CONTEXT, buffer, size );
        break;
    case CONTEXT_LORAWAN_STACK:
#if defined( MULTISTACK )
        // In case code is built for multiple stacks, read_modify_write feature is mandatory
        hal_flash_read_modify_write( ADDR_FLASH_LORAWAN_CONTEXT + offset, buffer, size );
#else
        hal_flash_erase_page( ADDR_FLASH_LORAWAN_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_LORAWAN_CONTEXT, buffer, size );
#endif
        break;
    case CONTEXT_FUOTA:
#if defined( USE_FLASH_READ_MODIFY_WRITE )
        hal_flash_read_modify_write( ADDR_FLASH_FUOTA + offset, buffer, size );
#endif
        break;
    case CONTEXT_SECURE_ELEMENT:
#if defined( USE_FLASH_READ_MODIFY_WRITE )
        // In case code is built for multiple stacks, read_modify_write feature is mandatory
        hal_flash_read_modify_write( ADDR_FLASH_SECURE_ELEMENT_CONTEXT + offset, buffer, size );
#else
        hal_flash_erase_page( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, 1 );
        hal_flash_write_buffer( ADDR_FLASH_SECURE_ELEMENT_CONTEXT, buffer, size );
#endif
        break;
    case CONTEXT_STORE_AND_FORWARD:
        hal_flash_write_buffer( ADDR_FLASH_STORE_AND_FORWARD + offset, buffer, size );
        break;
#endif
    default:
        mcu_panic( );
        break;
    }
}

void smtc_modem_hal_context_flash_pages_erase( const modem_context_type_t ctx_type, uint32_t offset, uint8_t nb_page )
{
    switch( ctx_type )
    {
#if defined( STM32L476xx )
    case CONTEXT_STORE_AND_FORWARD:
        hal_flash_erase_page( ADDR_FLASH_STORE_AND_FORWARD + offset, nb_page );
        break;
#endif
    default:
        mcu_panic( );
        break;
    };
}

/* ------------ crashlog management ------------*/

void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length )
{
    crashlog_length_noinit = MIN( crash_string_length, CRASH_LOG_SIZE );
    memcpy( crashlog_buff_noinit, crash_string, crashlog_length_noinit );
    crashlog_available_noinit = true;
}

void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length )
{
    *crash_string_length = ( crashlog_length_noinit > CRASH_LOG_SIZE ) ? CRASH_LOG_SIZE : crashlog_length_noinit;
    memcpy( crash_string, crashlog_buff_noinit, *crash_string_length );
}

void smtc_modem_hal_crashlog_set_status( bool available )
{
    crashlog_available_noinit = available;
}

static volatile bool temp = 0x1;  // solve new behaviour introduce with gcc11 compilo
bool                 smtc_modem_hal_crashlog_get_status( void )
{
    bool temp2 = crashlog_available_noinit & temp;
    return temp2;
}

/* ------------ assert management ------------*/

void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... )
{
    uint8_t out_buff[255] = { 0 };
    uint8_t out_len       = snprintf( ( char* ) out_buff, sizeof( out_buff ), "%s:%lu ", func, line );

    va_list args;
    va_start( args, fmt );
    out_len += vsprintf( ( char* ) &out_buff[out_len], fmt, args );
    va_end( args );

    smtc_modem_hal_crashlog_store( out_buff, out_len );

    SMTC_HAL_TRACE_ERROR( "Modem panic: %s\n", out_buff );
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
    // Tcxo is present on LR1110 and LR1120 evk boards, LR1121 ref board does not have tcxo but only 32MHz xtal
#if defined( LR11XX ) && !defined( LR1121 )
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

/* ------------ Fuota management ------------*/

#if defined( USE_FUOTA )
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
#endif  // USE_FUOTA

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

/* ------------ Needed for Store and Forward service  ------------*/
#if defined( USE_STORE_AND_FORWARD )
uint16_t smtc_modem_hal_store_and_forward_get_number_of_pages( void )
{
    return 10;
}

uint16_t smtc_modem_hal_flash_get_page_size( void )
{
    return hal_flash_get_page_size( );
}
#endif

/* ------------ For Real Time OS compatibility  ------------*/

void smtc_modem_hal_user_lbm_irq( void )
{
    // Do nothing in case implementation is bare metal
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
