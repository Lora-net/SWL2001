/*!
 * \file      smtc_modem_hal_dbg_trace.h
 *
 * \brief     Modem Hardware Abstraction Layer trace features template.
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
#ifndef __SMTC_MODEM_HAL_DBG_TRACE_H__
#define __SMTC_MODEM_HAL_DBG_TRACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

// clang-format off
#define MODEM_HAL_FEATURE_OFF                             0
#define MODEM_HAL_FEATURE_ON                              !MODEM_HAL_FEATURE_OFF

// Sensible default values. Change in Makefile if needed
#ifndef MODEM_HAL_DBG_TRACE
#define MODEM_HAL_DBG_TRACE                               MODEM_HAL_FEATURE_ON
#endif

#ifndef MODEM_HAL_DBG_TRACE_COLOR
#define MODEM_HAL_DBG_TRACE_COLOR                         MODEM_HAL_FEATURE_ON
#endif

#ifndef MODEM_HAL_DBG_TRACE_RP
#define MODEM_HAL_DBG_TRACE_RP                            MODEM_HAL_FEATURE_OFF
#endif

#ifndef MODEM_HAL_DEEP_DBG_TRACE
#define MODEM_HAL_DEEP_DBG_TRACE                          MODEM_HAL_FEATURE_OFF
#endif
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// clang-format off
#if ( MODEM_HAL_DBG_TRACE_COLOR == MODEM_HAL_FEATURE_ON )
    #define MODEM_HAL_DBG_TRACE_COLOR_BLACK   "\x1B[0;30m"
    #define MODEM_HAL_DBG_TRACE_COLOR_RED     "\x1B[0;31m"
    #define MODEM_HAL_DBG_TRACE_COLOR_GREEN   "\x1B[0;32m"
    #define MODEM_HAL_DBG_TRACE_COLOR_YELLOW  "\x1B[0;33m"
    #define MODEM_HAL_DBG_TRACE_COLOR_BLUE    "\x1B[0;34m"
    #define MODEM_HAL_DBG_TRACE_COLOR_MAGENTA "\x1B[0;35m"
    #define MODEM_HAL_DBG_TRACE_COLOR_CYAN    "\x1B[0;36m"
    #define MODEM_HAL_DBG_TRACE_COLOR_WHITE   "\x1B[0;37m"
    #define MODEM_HAL_DBG_TRACE_COLOR_DEFAULT "\x1B[0m"
#else
    #define MODEM_HAL_DBG_TRACE_COLOR_BLACK   ""
    #define MODEM_HAL_DBG_TRACE_COLOR_RED     ""
    #define MODEM_HAL_DBG_TRACE_COLOR_GREEN   ""
    #define MODEM_HAL_DBG_TRACE_COLOR_YELLOW  ""
    #define MODEM_HAL_DBG_TRACE_COLOR_BLUE    ""
    #define MODEM_HAL_DBG_TRACE_COLOR_MAGENTA ""
    #define MODEM_HAL_DBG_TRACE_COLOR_CYAN    ""
    #define MODEM_HAL_DBG_TRACE_COLOR_WHITE   ""
    #define MODEM_HAL_DBG_TRACE_COLOR_DEFAULT ""
#endif

#if ( MODEM_HAL_DBG_TRACE )

    #define SMTC_MODEM_HAL_TRACE_PRINTF( ... )  smtc_modem_hal_print_trace (  __VA_ARGS__ )

    #define SMTC_MODEM_HAL_TRACE_MSG( msg )                                                         \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s%s", MODEM_HAL_DBG_TRACE_COLOR_DEFAULT, msg);               \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR( msg, color )                                            \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s%s%s", color, msg, MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );     \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_INFO( ... )                                                        \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_GREEN "INFO: " __VA_ARGS__);         \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                           \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_WARNING( ... )                                                     \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_YELLOW "WARN: " __VA_ARGS__ );       \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                           \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_ERROR( ... )                                                       \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_RED "ERROR: " __VA_ARGS__);          \
        SMTC_MODEM_HAL_TRACE_PRINTF( MODEM_HAL_DBG_TRACE_COLOR_DEFAULT );                           \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_ARRAY( msg, array, len )                                           \
    do                                                                                              \
    {                                                                                               \
        SMTC_MODEM_HAL_TRACE_PRINTF("%s - (%lu bytes):\n", msg, ( uint32_t )len );                  \
        for( uint32_t i = 0; i < ( uint32_t )len; i++ )                                             \
        {                                                                                           \
            if( ( ( i % 16 ) == 0 ) && ( i > 0 ) )                                                  \
            {                                                                                       \
                SMTC_MODEM_HAL_TRACE_PRINTF("\n");                                                  \
            }                                                                                       \
            SMTC_MODEM_HAL_TRACE_PRINTF( " %02X", array[i] );                                       \
        }                                                                                           \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n" );                                                        \
    } while ( 0 );

    #define SMTC_MODEM_HAL_TRACE_PACKARRAY( msg, array, len )                                       \
    do                                                                                              \
    {                                                                                               \
        for( uint32_t i = 0; i < ( uint32_t ) len; i++ )                                            \
        {                                                                                           \
            SMTC_MODEM_HAL_TRACE_PRINTF( "%02X", array[i] );                                        \
        }                                                                                           \
    } while( 0 );

    #if (MODEM_HAL_DEEP_DBG_TRACE)
// Deep debug trace default definitions
    #define SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( ... )            SMTC_MODEM_HAL_TRACE_PRINTF(__VA_ARGS__)
    #define SMTC_MODEM_HAL_TRACE_MSG_DEBUG( msg )               SMTC_MODEM_HAL_TRACE_MSG(msg)
    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR_DEBUG( msg, color )  SMTC_MODEM_HAL_TRACE_MSG_COLOR( msg, color )
    #define SMTC_MODEM_HAL_TRACE_INFO_DEBUG( ... )              SMTC_MODEM_HAL_TRACE_INFO(__VA_ARGS__)
    #define SMTC_MODEM_HAL_TRACE_WARNING_DEBUG( ... )           SMTC_MODEM_HAL_TRACE_WARNING(__VA_ARGS__)
    #define SMTC_MODEM_HAL_TRACE_ERROR_DEBUG( ... )             SMTC_MODEM_HAL_TRACE_ERROR(__VA_ARGS__)
    #define SMTC_MODEM_HAL_TRACE_ARRAY_DEBUG( msg, array, len ) SMTC_MODEM_HAL_TRACE_ARRAY( msg, array, len )
    #define SMTC_MODEM_HAL_TRACE_PACKARRAY_DEBUG( ... )         SMTC_MODEM_HAL_TRACE_PACKARRAY(__VA_ARGS__)

    #else
// Deep debug trace default definitions
    #define SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_MSG_DEBUG( msg )
    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR_DEBUG( msg, color )
    #define SMTC_MODEM_HAL_TRACE_INFO_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_WARNING_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_ERROR_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_ARRAY_DEBUG( msg, array, len )
    #define SMTC_MODEM_HAL_TRACE_PACKARRAY_DEBUG( ... )
    #endif

#else
//Trace default definitions
    #define SMTC_MODEM_HAL_TRACE_PRINTF( ... )
    #define SMTC_MODEM_HAL_TRACE_MSG( msg )
    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR( msg, color )
    #define SMTC_MODEM_HAL_TRACE_INFO( ... )
    #define SMTC_MODEM_HAL_TRACE_WARNING( ... )
    #define SMTC_MODEM_HAL_TRACE_ERROR( ... )
    #define SMTC_MODEM_HAL_TRACE_ARRAY( msg, array, len )
    #define SMTC_MODEM_HAL_TRACE_PACKARRAY( ... )

// Deep debug trace default definitions
    #define SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_MSG_DEBUG( msg )
    #define SMTC_MODEM_HAL_TRACE_MSG_COLOR_DEBUG( msg, color )
    #define SMTC_MODEM_HAL_TRACE_INFO_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_WARNING_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_ERROR_DEBUG( ... )
    #define SMTC_MODEM_HAL_TRACE_ARRAY_DEBUG( msg, array, len )
    #define SMTC_MODEM_HAL_TRACE_PACKARRAY_DEBUG( ... )
#endif

#if ( MODEM_HAL_DBG_TRACE_RP == MODEM_HAL_FEATURE_ON )
    #define SMTC_MODEM_HAL_RP_TRACE_MSG( msg )                                        \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( "    " );                                        \
        SMTC_MODEM_HAL_TRACE_PRINTF( msg );                                           \
    } while( 0 )

    #define SMTC_MODEM_HAL_RP_TRACE_PRINTF( ... )                                     \
    do                                                                         \
    {                                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( "    " );                                        \
        SMTC_MODEM_HAL_TRACE_PRINTF( __VA_ARGS__ );                                   \
    } while( 0 )

#else
    #define SMTC_MODEM_HAL_RP_TRACE_MSG( msg )
    #define SMTC_MODEM_HAL_RP_TRACE_PRINTF( ... )
#endif
// clang-format on

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

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_MODEM_HAL_DBG_TRACE_H__

/* --- EOF ------------------------------------------------------------------ */
