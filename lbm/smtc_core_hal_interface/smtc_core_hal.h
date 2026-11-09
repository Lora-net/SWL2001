/**
 * @file      smtc_core_hal.h
 *
 * @brief     Semtech Core Hardware Abstraction Layer API description
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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
#ifndef SMTC_CORE_HAL_H
#define SMTC_CORE_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_lbm_code_ptr.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Crash log size (in bytes)
 */
#define CRASH_LOG_SIZE 242

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
    CONTEXT_KEY_MODEM,
    CONTEXT_LORAWAN_STACK,
    CONTEXT_FUOTA,
    CONTEXT_SECURE_ELEMENT,
    CONTEXT_STORE_AND_FORWARD,
} modem_context_type_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Document that a parameter is unused
 */
#ifndef UNUSED
#define UNUSED( x ) ( void ) ( x )
#endif  // UNUSED

/**
 * @brief Panic macro for modem issues
 */

#define SMTC_MODEM_HAL_PANIC( ... )                                                \
    do                                                                             \
    {                                                                              \
        smtc_modem_hal_on_panic( ( uint8_t* ) __func__, __LINE__, ""__VA_ARGS__ ); \
    } while( 0 );

/**
 * @brief  The SMTC_MODEM_HAL_PANIC_ON_FAILURE macro is used for function's parameters check.
 * @param  expr If expr is false, it calls smtc_modem_hal_on_panic function
 *         which reports the name of the source function and the source
 *         line number of the call that failed.
 *         If expr is true, it returns no value.
 * @retval None
 */
#define SMTC_MODEM_HAL_PANIC_ON_FAILURE( expr ) \
    ( ( expr ) ? ( void ) 0U : smtc_modem_hal_on_panic( ( uint8_t* ) __func__, __LINE__, "" ) )

/**
 * @brief Document that a parameter is unused
 */
#ifndef UNUSED
#define UNUSED( x ) ( void ) ( x )
#endif  // UNUSED

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
 * @brief Returns the current time in milliseconds
 *
 *
 * @return uint32_t Current time in milliseconds (wraps every 49 days)
 */
uint32_t smtc_modem_hal_get_time_in_ms( void );

/**
 * @brief set an offset into the rtc counter
 *
 * @remark Used for debug purpose such as wrapping issue.
 *
 * @return
 */
void smtc_modem_hal_set_offset_to_test_wrapping( const uint32_t offset_to_test_wrapping );

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

/* ------------ Panic management ------------*/

/**
 * @brief Action to be taken in case on modem panic
 *
 * @remark In case Device Management is used, it is recommended to perform the crashlog storage and status update in
 * this function
 *
 * @param [in] func The name of the function where the panic occurs
 * @param [in] line The line number where the panic occurs
 * @param [in] fmt  String Format
 * @param ...  String Arguments
 */
void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... );

/* ------------ Random management ------------*/

/**
 * @brief Returns an unsigned random number between min and max
 *
 * @param [in] val_1 first range unsigned value
 * @param [in] val_2 second range unsigned value
 *
 * @return uint32_t Generated random unsigned number between smallest value and biggest value (between val_1 and val_2)
 */
uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 );

/* ------------ Radio env management ------------*/

/**
 * @brief Config the radio interruption callback
 *
 * @param [in] callback     Callback that will be called in case of timer irq
 * @param [in] context      Context that will be passed on callback argument
 *
 */
void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context );

/* ------------ Radio management ------------
 *@brief Check if the radio is free
 * @remark except a very specific application, this function should always return false
 * @remark This function is used to check if the radio is use by an external stack
 *
 * @return bool False if the radio is free, false otherwise
 */
bool smtc_modem_external_stack_currently_use_radio( void );
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

/**
 * @brief Set antenna switch for Tx operation or not.
 *
 * @param [in] is_tx_on Indicates if the antenna switch must be set for Tx operation or not
 */
void smtc_modem_hal_set_ant_switch( bool is_tx_on );

/* ------------ Environment management ------------*/

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

/* ------------ Context saving management ------------*/

/**
 * @brief Restores the data context
 *
 * @remark This function is used to restore Modem data from a non volatile memory
 *
 * @param [in]  ctx_type   Type of modem context that need to be restored
 * @param [in]  offset     Memory offset after ctx_type address
 * @param [out] buffer     Buffer pointer to write to
 * @param [in]  size       Buffer size to read in bytes
 */
void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint32_t offset, uint8_t* buffer,
                                     const uint32_t size );

/**
 * @brief Stores the data context
 *
 * @remark This function is used to store Modem data in a non volatile memory
 *
 * @param [in] ctx_type   Type of modem context that need to be saved
 * @param [in] offset     Memory offset after ctx_type address
 * @param [in] buffer     Buffer pointer to write from
 * @param [in] size       Buffer size to write in bytes
 */
void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer,
                                   const uint32_t size );

/**
 * @brief Erase a chosen number of flash pages of a context
 * @remark This function is only used with CONTEXT_STORE_AND_FORWARD
 *
 * @param [in] ctx_type   Type of modem context that need to be erased
 * @param [in] offset     Memory offset after ctx_type address
 * @param [in] nb_page    Number of pages that
 */
void smtc_modem_hal_context_flash_pages_erase( const modem_context_type_t ctx_type, uint32_t offset, uint8_t nb_page );

/* ------------ For Real Time OS compatibility  ------------*/

/**
 * @brief This function is used whenever the app needs to sleep for an determined period of time.
 * This might be used in a RTOS context to suspend the calling thread (await for notify/semaphore, ...)
 * @param [in] timeout Timeout in ms
 */
void smtc_modem_hal_interruptible_msleep( uint32_t timeout );

/**
 * @brief This function is used whenever the app needs to wakeup (for example, on smtc_modem_hal_user_lbm_irq().)
 * It might be used in a RTOS context to notify/release a semaphore on the desired thread.
 */
void smtc_modem_hal_wake_up( void );

/**
 * @brief This function is called by the LBM stack on each LBM interruption (radio interrupt or low-power timer
 * interrupt). It could be convenient in the case of an RTOS implementation to notify the thread that manages the LBM
 * stack
 */
void smtc_modem_hal_user_lbm_irq( void );

/**
 * @brief Protects the API calls to the rac library
 * @remark this function is used to protect the rac_lib call incase of RTOS implementation
 */
void smtc_modem_hal_protect_api_call( void );
/**
 * @brief Unprotects the API calls to the rac library
 * @remark this function is used to unprotect the rac_lib call incase of RTOS implementation
 */

void smtc_modem_hal_unprotect_api_call( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_CORE_HAL_H