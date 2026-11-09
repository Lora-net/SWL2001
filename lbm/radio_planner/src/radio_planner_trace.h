/*!
 * \file      radio_planner_trace.h
 *
 * \brief     Radio planner trace helper file
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
#ifndef __RADIO_PLANNER_TRACE_H__
#define __RADIO_PLANNER_TRACE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "smtc_core_hal_trace.h"  // For top-level macro call (will later be renamed)
#include "smtc_core_hal.h"

/* clang-format off */

// Standard RP logs
#ifndef SMTC_RADIO_PLANNER_TRACE
#define SMTC_RADIO_PLANNER_TRACE                            MODEM_HAL_FEATURE_ON
#endif

// Debug RP logs
#ifndef SMTC_RADIO_PLANNER_TRACE_DBG
#define SMTC_RADIO_PLANNER_TRACE_DBG                        MODEM_HAL_FEATURE_OFF
#endif

// TODO : When modem HAL is properly split, change SMTC_MODEM_HAL_xxx calls to more generic macro name - 
// Also, renaming smtc_modem_hal_print_trace() to a more generic name could be good practice
#if ( SMTC_RADIO_PLANNER_TRACE == MODEM_HAL_FEATURE_ON )
// Standard logging for RP
    #define RP_PREFIX "RP - "
    #define SMTC_RP_TRACE_PRINTF( ... )                                                     \
    do                                                                                      \
    {                                                                                       \
        SMTC_MODEM_HAL_TRACE_PRINTF( RP_PREFIX __VA_ARGS__ );                               \
    } while ( 0 );

    #define SMTC_RP_TRACE_MSG( msg )                                                         \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s%s", MODEM_HAL_DBG_TRACE_COLOR_DEFAULT, msg);               \
    } while ( 0 );

    #define SMTC_RP_TRACE_MSG_COLOR( msg, color )                                            \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s%s%s", color, msg, MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );     \
    } while ( 0 );

    #define SMTC_RP_TRACE_INFO( ... )                                                        \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_GREEN "RP - INFO: " __VA_ARGS__);         \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                           \
    } while ( 0 );

    #define SMTC_RP_TRACE_WARNING( ... )                                                     \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_YELLOW "RP - WARN: " __VA_ARGS__ );       \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                           \
    } while ( 0 );

    #define SMTC_RP_TRACE_ERROR( ... )                                                       \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_RED "RP - ERROR: " __VA_ARGS__);          \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                           \
    } while ( 0 );

    // Deep (debug) logging for RP
    #if ( SMTC_RADIO_PLANNER_TRACE_DBG == MODEM_HAL_FEATURE_ON )
        #define SMTC_RP_TRACE_MSG_DEBUG( msg )                                          \
        do                                                                                  \
        {                                                                                   \
            SMTC_MODEM_HAL_TRACE_PRINTF( "    " );                                          \
            SMTC_MODEM_HAL_TRACE_PRINTF( msg );                                             \
        } while( 0 )

        #define SMTC_RP_TRACE_PRINTF_DEBUG( ... )                                       \
        do                                                                                  \
        {                                                                                   \
            SMTC_MODEM_HAL_TRACE_PRINTF( "    " );                                          \
            SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                     \
        } while( 0 )
    #else
        #define SMTC_RP_TRACE_MSG_DEBUG( msg )
        #define SMTC_RP_TRACE_PRINTF_DEBUG( ... )
    #endif //SMTC_RADIO_PLANNER_TRACE_DBG
#else
    #define SMTC_RP_TRACE_PRINTF( ... )
    #define SMTC_RP_TRACE_MSG( msg )
    #define SMTC_RP_TRACE_MSG_COLOR( msg, color )
    #define SMTC_RP_TRACE_INFO( ... )
    #define SMTC_RP_TRACE_WARNING( ... )
    #define SMTC_RP_TRACE_ERROR( ... )
    #define SMTC_RP_TRACE_MSG_DEBUG( msg )
    #define SMTC_RP_TRACE_PRINTF_DEBUG( ... )
#endif //SMTC_RADIO_PLANNER_TRACE

/* clang-format on */

#ifdef __cplusplus
}
#endif

#endif  // __RADIO_PLANNER_TRACE_H__
        /* --- EOF ------------------------------------------------------------------ */