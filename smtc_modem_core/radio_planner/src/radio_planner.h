/*!
 * \file      radio-planner.h
 *
 * \brief     Radio planner definition
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
#ifndef __RADIO_PLANNER_H__
#define __RADIO_PLANNER_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "radio_planner_types.h"
#include "radio_planner_stats.h"
#include "radio_planner_hal.h"
#include "radio_planner_hook_id_defs.h"

#include "ralf.h"

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

/*!
 *
 */
typedef struct radio_planner_s
{
    rp_task_t         priority_task;
    rp_radio_params_t radio_params[RP_NB_HOOKS];
    rp_task_t         tasks[RP_NB_HOOKS];
    uint8_t*          payload[RP_NB_HOOKS];
    uint16_t          payload_size[RP_NB_HOOKS];
    uint8_t           rankings[RP_NB_HOOKS];
    void*             hooks[RP_NB_HOOKS];
    rp_status_t       status[RP_NB_HOOKS];
    ral_irq_t         raw_radio_irq[RP_NB_HOOKS];
    uint32_t          irq_timestamp_ms[RP_NB_HOOKS];
    uint32_t          irq_timestamp_100us[RP_NB_HOOKS];
    rp_stats_t        stats;
    uint8_t           hook_to_execute;
    uint32_t          hook_to_execute_time_ms;
    uint8_t           radio_task_id;
    uint8_t           timer_task_id;
    uint8_t           semaphore_radio;
    uint32_t          timer_value;
    uint8_t           timer_hook_id;
    void ( *hook_callbacks[RP_NB_HOOKS] )( void* );
    rp_next_state_status_t next_state_status;
    const ralf_t*          radio;
    uint32_t               margin_delay;
} radio_planner_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 *
 */
void rp_radio_irq_callback( void* obj );

/*!
 *
 */
void rp_init( radio_planner_t* rp, const ralf_t* radio );

/*!
 *
 */
rp_hook_status_t rp_hook_init( radio_planner_t* rp, const uint8_t id, void ( *callback )( void* context ), void* hook );

/*!
 *
 */
rp_hook_status_t rp_hook_get_id( const radio_planner_t* rp, const void* hook, uint8_t* id );

/*!
 *
 */
rp_hook_status_t rp_release_hook( radio_planner_t* rp, uint8_t id );
/*!
 * Enqueue a task to be handled by the radio planner
 *
 * \param [in/out] rp           Radio planner data structure
 * \param [in]     task         Radio planner task to be handled
 * \param [in]     payload      Pointer to the buffer holding the data to be
 *                                  Tx/Rx
 * \param [in]     payload_size Buffer size. Tx: Size to be transmitted,
 *                                  Rx: Maximum payload to be received
 * \param [in]     radio_params Holds the radio parameters to be used while
 *                                  handling the task.
 * \retval status               Function execution status
 */
rp_hook_status_t rp_task_enqueue( radio_planner_t* rp, const rp_task_t* task, uint8_t* payload, uint16_t payload_size,
                                  const rp_radio_params_t* radio_params );

/*!
 *
 */
rp_hook_status_t rp_task_abort( radio_planner_t* rp, const uint8_t hook_id );

/*!
 *
 */
rp_stats_t rp_get_stats( const radio_planner_t* rp );

/*!
 *
 */
void rp_get_status( const radio_planner_t* rp, const uint8_t id, uint32_t* irq_timestamp_ms, rp_status_t* status );
/*!
 *
 */
void rp_get_and_clear_raw_radio_irq( radio_planner_t* rp, const uint8_t id, ral_irq_t* raw_radio_irq );
#ifdef __cplusplus
}
#endif

#endif  // __RADIO_PLANNER_H__

/* --- EOF ------------------------------------------------------------------ */
