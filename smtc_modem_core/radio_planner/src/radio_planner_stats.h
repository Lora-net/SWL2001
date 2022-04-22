/*!
 * \file      radio_planner_stats.h
 *
 * \brief     Radio planner statistics
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
#ifndef __RADIO_PLANNER_STATS_H__
#define __RADIO_PLANNER_STATS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#if defined( RP_STAT_PRINT_ENBALE )
#include "smtc_modem_hal_dbg_trace.h"
#endif  // RP_STAT_PRINT_ENBALE

#include <string.h>  // for memset

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
typedef struct rp_stats_s
{
    uint32_t tx_last_toa_ms[RP_NB_HOOKS];
    uint32_t tx_consumption_ms[RP_NB_HOOKS];
    uint32_t rx_consumption_ms[RP_NB_HOOKS];
    uint32_t none_consumption_ms[RP_NB_HOOKS];
    uint32_t tx_consumption_ma[RP_NB_HOOKS];
    uint32_t rx_consumption_ma[RP_NB_HOOKS];
    uint32_t none_consumption_ma[RP_NB_HOOKS];
    uint32_t tx_total_consumption_ms;
    uint32_t rx_total_consumption_ms;
    uint32_t none_total_consumption_ms;
    uint32_t tx_total_consumption_ma;
    uint32_t rx_total_consumption_ma;
    uint32_t none_total_consumption_ma;
    uint32_t tx_timestamp;
    uint32_t rx_timestamp;
    uint32_t none_timestamp;
    uint32_t task_hook_aborted_nb[RP_NB_HOOKS];
    uint32_t rp_error;
} rp_stats_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 *
 */
static inline void rp_stats_init( rp_stats_t* rp_stats )
{
    memset( rp_stats, 0, sizeof( rp_stats_t ) );
}

/*!
 *
 */
static inline void rp_stats_set_tx_timestamp( rp_stats_t* rp_stats, uint32_t timestamp )
{
    rp_stats->tx_timestamp = timestamp;
}

/*!
 *
 */
static inline void rp_stats_set_rx_timestamp( rp_stats_t* rp_stats, uint32_t timestamp )
{
    rp_stats->rx_timestamp = timestamp;
}

/*!
 *
 */
static inline void rp_stats_set_none_timestamp( rp_stats_t* rp_stats, uint32_t timestamp )
{
    rp_stats->none_timestamp = timestamp;
}

/*!
 *
 */
static inline void rp_stats_update( rp_stats_t* rp_stats, uint32_t timestamp, uint8_t hook_id, uint32_t micro_ampere )
{
    uint32_t computed_time        = 0;
    uint32_t computed_consumption = 0;
    if( rp_stats->tx_timestamp != 0 )
    {
        // wrapping is impossible with this time base
        computed_time                     = timestamp - rp_stats->tx_timestamp;
        rp_stats->tx_last_toa_ms[hook_id] = computed_time;
        rp_stats->tx_consumption_ms[hook_id] += computed_time;
        rp_stats->tx_total_consumption_ms += computed_time;

        computed_consumption = ( computed_time * micro_ampere );
        rp_stats->tx_consumption_ma[hook_id] += ( computed_consumption / 1000 );
        rp_stats->tx_total_consumption_ma += ( computed_consumption / 1000 );
    }
    if( rp_stats->rx_timestamp != 0 )
    {
        computed_time = timestamp - rp_stats->rx_timestamp;
        rp_stats->rx_consumption_ms[hook_id] += computed_time;
        rp_stats->rx_total_consumption_ms += computed_time;

        computed_consumption = ( computed_time * micro_ampere );
        rp_stats->rx_consumption_ma[hook_id] += ( computed_consumption / 1000 );
        rp_stats->rx_total_consumption_ma += ( computed_consumption / 1000 );
    }
    if( rp_stats->none_timestamp != 0 )
    {
        computed_time = timestamp - rp_stats->none_timestamp;
        rp_stats->none_consumption_ms[hook_id] += computed_time;
        rp_stats->none_total_consumption_ms += computed_time;

        computed_consumption = ( computed_time * micro_ampere );
        rp_stats->none_consumption_ma[hook_id] += ( computed_consumption / 1000 );
        rp_stats->none_total_consumption_ma += ( computed_consumption / 1000 );
    }
    rp_stats->tx_timestamp   = 0;
    rp_stats->rx_timestamp   = 0;
    rp_stats->none_timestamp = 0;
}

/*!
 *
 */
static inline void rp_stats_sniff_update( rp_stats_t* rp_stats, uint32_t timestamp, uint32_t time_radio,
                                          uint32_t time_proc, uint8_t hook_id, uint32_t ma_radio, uint32_t ma_proc )
{
    uint32_t computed_time        = 0;
    uint32_t computed_consumption = 0;

    computed_time = timestamp - rp_stats->none_timestamp;
    // SMTC_MODEM_HAL_TRACE_WARNING( "stat %d %d\n", time_radio/1000, time_proc/1000 );
    // SMTC_MODEM_HAL_TRACE_WARNING( "stat %d \n", computed_time);
    rp_stats->none_consumption_ms[hook_id] += computed_time;
    rp_stats->none_total_consumption_ms += computed_time;

    computed_consumption = ( time_radio / 1000 * ma_radio ) + ( time_proc / 1000 * ma_proc );
    rp_stats->none_consumption_ma[hook_id] += ( computed_consumption / 1000 );
    rp_stats->none_total_consumption_ma += ( computed_consumption / 1000 );

    rp_stats->tx_timestamp   = 0;
    rp_stats->rx_timestamp   = 0;
    rp_stats->none_timestamp = 0;
}

#if defined( RP_STAT_PRINT_ENBALE )
/*!
 *
 */
static inline void rp_stats_print( rp_stats_t* rp_stats )
{
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "###### ===================================== ######\n" );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "###### ===== Radio Planner Statistics ====== ######\n" );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "###### ===================================== ######\n" );
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Tx consumption hook #%ld = %lu ms\n", i, rp_stats->tx_consumption_ms[i] );
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Tx consumption hook #%ld = %lu ua\n", i, rp_stats->tx_consumption_ma[i] );
    }
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Rx consumption hook #%ld = %lu ms\n", i, rp_stats->rx_consumption_ms[i] );
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Rx consumption hook #%ld = %lu ua\n", i, rp_stats->rx_consumption_ma[i] );
    }
    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "None consumption hook #%ld = %lu ms\n", i, rp_stats->none_consumption_ms[i] );
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "None consumption hook #%ld = %lu ua\n", i, rp_stats->none_consumption_ma[i] );
    }
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Tx total consumption     = %lu ms\n ", rp_stats->tx_total_consumption_ms );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Tx total consumption     = %lu uA\n ", rp_stats->tx_total_consumption_ma );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Rx total consumption     = %lu ms\n ", rp_stats->rx_total_consumption_ms );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Rx total consumption     = %lu uA\n ", rp_stats->rx_total_consumption_ma );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "None total consumption   = %lu ms\n ", rp_stats->none_total_consumption_ms );
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "None total consumption   = %lu uA\n ", rp_stats->none_total_consumption_ma );

    for( int32_t i = 0; i < RP_NB_HOOKS; i++ )
    {
        SMTC_MODEM_HAL_RP_TRACE_PRINTF( "Number of aborted tasks for hook #%ld = %lu \n", i,
                                        rp_stats->task_hook_aborted_nb[i] );
    }
    SMTC_MODEM_HAL_RP_TRACE_PRINTF( "RP: number of errors is %lu\n\n\n", rp_stats->rp_error );
}
#endif  // RP_STAT_PRINT_ENBALE

#ifdef __cplusplus
}
#endif

#endif  // __RADIO_PLANNER_STATS_H__

/* --- EOF ------------------------------------------------------------------ */
