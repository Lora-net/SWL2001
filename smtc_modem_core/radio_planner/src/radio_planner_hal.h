/*!
 * \file      radio_planner_hal.h
 *
 * \brief     Radio planner Hardware Abstraction Layer functions definition
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
#ifndef __RADIO_PLANNER_HAL_H__
#define __RADIO_PLANNER_HAL_H__

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

/*
 * ============================================================================
 * API definitions to be implemented by the user for the specific platform.
 * ============================================================================
 */

/*!
 *
 */
void rp_hal_critical_section_begin( void );

/*!
 *
 */
void rp_hal_critical_section_end( void );

/*!
 *
 */
void rp_hal_timer_stop( void );

/*!
 *
 */
void rp_hal_timer_start( void* rp, uint32_t alarm_in_ms, void ( *callback )( void* context ) );

/**
 * @brief Gets current time in ms
 *
 * @return uint32_t
 */
uint32_t rp_hal_get_time_in_ms( void );

/**
 * @brief Gets the time in 100µs at which the last radio IRQ occurred
 *
 * @return uint32_t
 */
uint32_t rp_hal_get_radio_irq_timestamp_in_100us( void );

/*!
 *
 */
void rp_hal_irq_clear_pending( void );

#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
/*!
 *
 */
void rp_hal_get_gnss_conso_us( uint32_t* p_radio_t, uint32_t* p_arc_process_t );
#endif  // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_WIFI_ENABLE ) /*! \
                                                                  *  \
                                                                  */
void rp_hal_get_wifi_conso_us( uint32_t* p_radio_t, uint32_t* p_arc_process_t );
#endif  // LR1110_MODEM_E && _MODEM_E_WIFI_ENABLE

#ifdef __cplusplus
}
#endif

#endif  // __RADIO_PLANNER_HAL_H__

/* --- EOF ------------------------------------------------------------------ */
