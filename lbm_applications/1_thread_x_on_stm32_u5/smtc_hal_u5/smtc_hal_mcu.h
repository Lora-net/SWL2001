/*!
 * \file      smtc_hal_mcu.h
 *
 * \brief     MCU Hardware Abstraction Layer definition
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
#ifndef __SMTC_HAL_MCU_H__
#define __SMTC_HAL_MCU_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * Panic function for mcu issues
 */
#define mcu_panic( ... )                                    \
    do                                                      \
    {                                                       \
        SMTC_HAL_TRACE_ERROR( "mcu_panic:%s\n", __func__ ); \
        SMTC_HAL_TRACE_ERROR( "-> "__VA_ARGS__ );           \
        hal_mcu_reset( );                                   \
    } while( 0 );

/*!
 * Begins critical section
 */
#define CRITICAL_SECTION_BEGIN( ) \
    uint32_t mask;                \
    hal_mcu_critical_section_begin( &mask )

/*!
 * Ends critical section
 */
#define CRITICAL_SECTION_END( ) hal_mcu_critical_section_end( &mask )
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * Disable interrupts, begins critical section
 *
 * \param [IN] mask Pointer to a variable where to store the CPU IRQ mask
 */
void hal_mcu_critical_section_begin( uint32_t* mask );

/*!
 * Ends critical section
 *
 * \param [IN] mask Pointer to a variable where the CPU IRQ mask was stored
 */
void hal_mcu_critical_section_end( uint32_t* mask );

/*!
 * Disable all irq at mcu side
 */
void hal_mcu_disable_irq( void );

/*!
 * Enable all irq at mcu side
 */
void hal_mcu_enable_irq( void );

/*!
 * Initializes BSP used MCU
 */
void hal_mcu_init( void );

/*!
 * Reset mcu
 */
void hal_mcu_reset( void );

/*!
 * Blocking wait
 */
void hal_mcu_wait_us( const int32_t microseconds );

/*!
 * Sets the MCU in sleep mode for the given number of milliseconds.
 *
 * \param[IN] milliseconds Number of milliseconds to stay in sleep mode
 */
void hal_mcu_set_sleep_for_ms( const int32_t milliseconds );

/*!
 * Suspend low power process and avoid looping on it
 */
void hal_mcu_disable_low_power_wait( void );

/*!
 * Enable low power process
 */
void hal_mcu_enable_low_power_wait( void );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_MCU_H__

/* --- EOF ------------------------------------------------------------------ */
