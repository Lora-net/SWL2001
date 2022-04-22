/**
 * @file      smtc_modem_hal.h
 *
 * @brief     Modem Hardware Abstraction Layer API description
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
#ifndef __SMTC_MODEM_HAL_H__
#define __SMTC_MODEM_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Panic function for mcu issues
 */
#define smtc_modem_hal_mcu_panic( ... )                            \
    do                                                             \
    {                                                              \
        smtc_modem_hal_store_crashlog( ( uint8_t* ) __func__ );    \
        smtc_modem_hal_set_crashlog_status( true );                \
        SMTC_MODEM_HAL_TRACE_ERROR( "crash log :%s\n", __func__ ); \
        SMTC_MODEM_HAL_TRACE_ERROR( "-> "__VA_ARGS__ );            \
        smtc_modem_hal_reset_mcu( );                               \
    } while( 0 );

/**
 * @brief Panic function for lr1mac issues
 */
#define smtc_modem_hal_lr1mac_panic( ... )                         \
    do                                                             \
    {                                                              \
        smtc_modem_hal_store_crashlog( ( uint8_t* ) __func__ );    \
        smtc_modem_hal_set_crashlog_status( true );                \
        SMTC_MODEM_HAL_TRACE_ERROR( "crash log :%s\n", __func__ ); \
        SMTC_MODEM_HAL_TRACE_ERROR( "-> "__VA_ARGS__ );            \
        smtc_modem_hal_reset_mcu( );                               \
    } while( 0 );

/**
 * @brief  The smtc_modem_hal_assert macro is used for function's parameters check.
 * @param  expr If expr is false, it calls smtc_modem_hal_assert_fail function
 *         which reports the name of the source function and the source
 *         line number of the call that failed.
 *         If expr is true, it returns no value.
 * @retval None
 */
#define smtc_modem_hal_assert( expr ) \
    ( ( expr ) ? ( void ) 0U : smtc_modem_hal_assert_fail( ( uint8_t* ) __func__, __LINE__ ) )

/**
 * @brief Document that a parameter is unused
 */
#ifndef UNUSED
#define UNUSED( x ) ( void ) ( x )
#endif  // UNUSED

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Crash log size in byte
 */
#define CRASH_LOG_SIZE 32

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Modem context type enum
 */
typedef enum
{
    CONTEXT_MODEM,
    CONTEXT_LR1MAC,
    CONTEXT_DEVNONCE,
    CONTEXT_SECURE_ELEMENT,
    MODEM_CONTEXT_TYPE_SIZE
} modem_context_type_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/* ------------ Reset management ------------*/

/**
 * @brief Resets the MCU
 */
void smtc_modem_hal_reset_mcu( void );

/* ------------ Watchdog management ------------*/

/**
 * @brief Reloads watchdog counter
 *
 * @remark Application has to call this function periodically.
 *         The call period must be less than WATCHDOG_RELOAD_PERIOD
 *
 */
void smtc_modem_hal_reload_wdog( void );

/* ------------ Time management ------------*/

/**
 * @brief Returns the current time in seconds
 *
 * @remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *         transmitting MAC answers, basically any delay without accurate time
 *         constraints. It is also used to measure the time spent inside the
 *         LoRaWAN process for the integrated failsafe.
 *
 * @return uint32_t Current time in seconds
 */
uint32_t smtc_modem_hal_get_time_in_s( void );

/**
 * @brief Returns the compensated current time in seconds
 *
 * @remark Used for Clock synchronization process ALCSync which need an accurate clock with compensated drift
 *
 * @return uint32_t Current time in seconds
 */
uint32_t smtc_modem_hal_get_compensated_time_in_s( void );

/**
 * @brief Returns the time compensation in seconds
 *
 * @remark Used for Clock synchronization process ALCSync which need an accurate clock with compensated drift
 *
 * @return int32_t the positive or negative compensation offset in seconds
 */
int32_t smtc_modem_hal_get_time_compensation_in_s( void );

/**
 * @brief Returns the current time in milliseconds
 *
 *
 * @return uint32_t Current time in milliseconds (wraps every 49 days)
 */
uint32_t smtc_modem_hal_get_time_in_ms( void );

/**
 * @brief Returns the current time in 0.1 milliseconds
 *
 * @remark Used for class B ping slot openings.
 * Must be the same timer as the one used for \ref smtc_modem_hal_get_radio_irq_timestamp_in_100us.
 *
 * @return uint32_t Current time in 100µs (wraps every 4,9 days)
 */
uint32_t smtc_modem_hal_get_time_in_100us( void );

/**
 * @brief Returns the time, in 0.1 milliseconds, of the last radio interrupt request
 *
 *  @remark Used to obtain the timestamp of radio events (i.e.: end of TX).
 *  Must be the same timer as the one used for \ref smtc_modem_hal_get_time_in_100us.
 *
 * @return uint32_t
 */
uint32_t smtc_modem_hal_get_radio_irq_timestamp_in_100us( void );

/* ------------ Timer management ------------*/

/**
 * @brief Starts the provided timer objet for the given time
 *
 * @param [in] milliseconds Number of milliseconds (timer value)
 * @param [in] callback     Callback that will be called in case of timer irq
 * @param [in] context      Context that will be passed on callback argument
 */
void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context );

/**
 * @brief Stop the provided timer
 */
void smtc_modem_hal_stop_timer( void );

/* ------------ IRQ management ------------*/

/**
 * @brief Disables interruptions used in Modem (radio_dio and timer)
 */
void smtc_modem_hal_disable_modem_irq( void );

/**
 * @brief Enables interruptions used in Modem (radio_dio and timer)
 */
void smtc_modem_hal_enable_modem_irq( void );

/* ------------ Context saving management ------------*/

/**
 * @brief Restores the data context
 *
 * @remark This function is used to restore Modem data from a non volatile memory
 *
 * @param [in] ctx_type   Type of modem context that need to be restored
 * @param [out] buffer    Buffer pointer to write to
 * @param [in] size       Buffer size to read in bytes
 */
void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint8_t* buffer, const uint32_t size );

/**
 * @brief Stores the data context
 *
 * @remark This function is used to store Modem data in a non volatile memory
 *
 * @param [in] ctx_type   Type of modem context that need to be saved
 * @param [in] buffer     Buffer pointer to write from
 * @param [in] size       Buffer size to write in bytes
 */
void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, const uint8_t* buffer, const uint32_t size );

/* ------------ Crashlog management ------------*/

/**
 * @brief Stores the crashlog
 *
 * @remark This function is used to store the Modem crashlog in a non volatile memory
 *
 * @param [in] crashlog   Buffer of 32 bytes containing crashlog data to store
 */
void smtc_modem_hal_store_crashlog( uint8_t crashlog[CRASH_LOG_SIZE] );

/**
 * @brief Restores the crashlog
 *
 * @remark This function is used to restore the Modem crashlog from a non volatile memory
 *
 * @param [out] crashlog   Buffer of 32 bytes containing crashlog data restored
 */
void smtc_modem_hal_restore_crashlog( uint8_t crashlog[CRASH_LOG_SIZE] );

/**
 * @brief Stores the crashlog status
 *
 * @remark This function is used to store the Modem crashlog status in a non volatile memory. This status will
 * allow the Modem to handle crashlog send task if needed after a crash
 *
 * @param [in] available  True if a crashlog is available, false otherwise
 */
void smtc_modem_hal_set_crashlog_status( bool available );

/**
 * @brief Get the previously stored crashlog status
 *
 * @remark This function is used to get the Modem crashlog status from a non volatile memory. This status will
 * allow the Modem to handle crashlog send task if needed after a crash
 *
 * @return bool True if a crashlog is available, false otherwise
 */
bool smtc_modem_hal_get_crashlog_status( void );

/* ------------ assert management ------------*/

/**
 * @brief smtc_modem_hal_assert_fail return the source function and the source line number where the assert error has
 * occurred
 *
 * @param func
 * @param line
 */
void smtc_modem_hal_assert_fail( uint8_t* func, uint32_t line );

/* ------------ Random management ------------*/
/**
 * @brief Returns a 32bits random number
 *
 * @return uint32_t Generated random number
 */
uint32_t smtc_modem_hal_get_random_nb( void );

/**
 * @brief Returns an unsigned random number between min and max
 *
 * @param [in] val_1 first range unsigned value
 * @param [in] val_2 second range unsigned value
 *
 * @return uint32_t Generated random unsigned number between smallest value and biggest value (between val_1 and val_2)
 */
uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 );

/**
 * @brief Returns a signed random number between min and max
 *
 * @param [in] val_1 first range signed value
 * @param [in] val_2 second range signed value
 *
 * @return int32_t Generated random signed number between smallest value and biggest value (between val_1 and val_2)
 */
int32_t smtc_modem_hal_get_signed_random_nb_in_range( const int32_t val_1, const int32_t val_2 );

/* ------------ Radio env management ------------*/

/**
 * @brief Config the radio interruption callback
 *
 * @param [in] callback     Callback that will be called in case of timer irq
 * @param [in] context      Context that will be passed on callback argument
 *
 */
void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context );

/*
 * @brief Clear any MCU-layer pending radio IRQ flags
 */
void smtc_modem_hal_radio_irq_clear_pending( void );

/**
 * @brief Start radio tcxo
 *
 * @remark In case used radio has no tcxo please implement an empty function
 */
void smtc_modem_hal_start_radio_tcxo( void );

/**
 * @brief Stop radio tcxo
 *
 * @remark In case used radio has no tcxo please implement an empty function
 */
void smtc_modem_hal_stop_radio_tcxo( void );

/**
 * @brief Get TCXO startup delay, in ms
 *
 * @remark In case used radio has no tcxo please implement a function which returns 0
 *
 * @return uint32_t TCXO startup delay in ms
 */
uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void );

/* ------------ Environment management ------------*/

/**
 * @brief Return the battery level
 *
 * @return uint8_t Battery level for lorawan stack
 */
uint8_t smtc_modem_hal_get_battery_level( void );

/**
 * @brief Return MCU temperature in celsius
 *
 * @return int8_t MCU temperature in celsius
 */
int8_t smtc_modem_hal_get_temperature( void );

/**
 * @brief Return mcu voltage (can be needed for dm uplink payload)
 *
 * @return uint8_t MCU voltage
 */
uint8_t smtc_modem_hal_get_voltage( void );

/**
 * @brief Return board wake up delay in ms
 *
 * @return uint8_t Board wake up delay in ms
 */
int8_t smtc_modem_hal_get_board_delay_ms( void );

/* ------------ Trace management ------------*/

/**
 * @brief Prints debug trace
 *
 * @param variadics arguments
 */
void smtc_modem_hal_print_trace( const char* fmt, ... );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_MODEM_HAL_H__

/* --- EOF ------------------------------------------------------------------ */
