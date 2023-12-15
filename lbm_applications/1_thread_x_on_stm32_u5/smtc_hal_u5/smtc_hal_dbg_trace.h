/*!
 * \file      smtc_hal_dbg_trace.h
 *
 * \brief     Hardware Abstraction Layer trace features
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
#ifndef __SMTC_HAL_DBG_TRACE_H__
#define __SMTC_HAL_DBG_TRACE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_hal_trace.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

// clang-format off
#define HAL_FEATURE_OFF                             0
#define HAL_FEATURE_ON                              !HAL_FEATURE_OFF

// Sensible default values. Change in Makefile if needed
#ifndef HAL_DBG_TRACE
#define HAL_DBG_TRACE                               HAL_FEATURE_ON
#endif

#ifndef HAL_DBG_TRACE_COLOR
#define HAL_DBG_TRACE_COLOR                         HAL_FEATURE_ON
#endif

#ifndef HAL_DBG_TRACE_RP
#define HAL_DBG_TRACE_RP                            HAL_FEATURE_OFF
#endif
// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// clang-format off
#if ( HAL_DBG_TRACE_COLOR == HAL_FEATURE_ON )
    #define HAL_DBG_TRACE_COLOR_BLACK               "\x1B[0;30m"
    #define HAL_DBG_TRACE_COLOR_RED                 "\x1B[0;31m"
    #define HAL_DBG_TRACE_COLOR_GREEN               "\x1B[0;32m"
    #define HAL_DBG_TRACE_COLOR_YELLOW              "\x1B[0;33m"
    #define HAL_DBG_TRACE_COLOR_BLUE                "\x1B[0;34m"
    #define HAL_DBG_TRACE_COLOR_MAGENTA             "\x1B[0;35m"
    #define HAL_DBG_TRACE_COLOR_CYAN                "\x1B[0;36m"
    #define HAL_DBG_TRACE_COLOR_WHITE               "\x1B[0;37m"
    #define HAL_DBG_TRACE_COLOR_DEFAULT             "\x1B[0m"
#else
    #define HAL_DBG_TRACE_COLOR_BLACK   ""
    #define HAL_DBG_TRACE_COLOR_RED     ""
    #define HAL_DBG_TRACE_COLOR_GREEN   ""
    #define HAL_DBG_TRACE_COLOR_YELLOW  ""
    #define HAL_DBG_TRACE_COLOR_BLUE    ""
    #define HAL_DBG_TRACE_COLOR_MAGENTA ""
    #define HAL_DBG_TRACE_COLOR_CYAN    ""
    #define HAL_DBG_TRACE_COLOR_WHITE   ""
    #define HAL_DBG_TRACE_COLOR_DEFAULT ""
#endif

#if ( HAL_DBG_TRACE )

    #define SMTC_HAL_TRACE_PRINTF( ... )  hal_trace_print_var (  __VA_ARGS__ )

    #define SMTC_HAL_TRACE_MSG( msg )                                               \
    do                                                                              \
    {                                                                               \
        SMTC_HAL_TRACE_PRINTF( "%s%s", HAL_DBG_TRACE_COLOR_DEFAULT, msg );          \
    } while ( 0 );

    #define SMTC_HAL_TRACE_MSG_COLOR( msg, color )                                  \
    do                                                                              \
    {                                                                               \
        SMTC_HAL_TRACE_PRINTF( "%s%s%s", color, msg, HAL_DBG_TRACE_COLOR_DEFAULT );                                         \
    } while ( 0 );

    #define SMTC_HAL_TRACE_INFO( ... )                                              \
    do                                                                              \
    {                                                                               \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_GREEN "INFO: " __VA_ARGS__);     \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT );                       \
    } while ( 0 );

    #define SMTC_HAL_TRACE_WARNING( ... )                                           \
    do                                                                              \
    {                                                                               \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_YELLOW "WARN: " __VA_ARGS__ );   \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT );                       \
    } while ( 0 );

    #define SMTC_HAL_TRACE_ERROR( ... )                                             \
    do                                                                              \
    {                                                                               \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_RED "ERROR: " __VA_ARGS__ );     \
        SMTC_HAL_TRACE_PRINTF( HAL_DBG_TRACE_COLOR_DEFAULT );                       \
    } while ( 0 );

    #define SMTC_HAL_TRACE_ARRAY( msg, array, len )                                 \
    do                                                                              \
    {                                                                               \
        SMTC_HAL_TRACE_PRINTF("%s - (%lu bytes):\n", msg, ( uint32_t )len );        \
        for( uint32_t i = 0; i < ( uint32_t )len; i++ )                             \
        {                                                                           \
            if( ( ( i % 16 ) == 0 ) && ( i > 0 ) )                                  \
            {                                                                       \
                SMTC_HAL_TRACE_PRINTF("\n");                                        \
            }                                                                       \
            SMTC_HAL_TRACE_PRINTF( " %02X", array[i] );                             \
        }                                                                           \
        SMTC_HAL_TRACE_PRINTF( "\n" );                                              \
    } while ( 0 );

    #define SMTC_HAL_TRACE_PACKARRAY( msg, array, len )                             \
    do                                                                              \
    {                                                                               \
        for( uint32_t i = 0; i < ( uint32_t ) len; i++ )                            \
        {                                                                           \
            SMTC_HAL_TRACE_PRINTF( "%02X", array[i] );                              \
        }                                                                           \
    } while( 0 );

#else
    #define SMTC_HAL_TRACE_PRINTF( ... )
    #define SMTC_HAL_TRACE_MSG( msg )
    #define SMTC_HAL_TRACE_MSG_COLOR( msg, color )
    #define SMTC_HAL_TRACE_INFO( ... )
    #define SMTC_HAL_TRACE_WARNING( ... )
    #define SMTC_HAL_TRACE_ERROR( ... )
    #define SMTC_HAL_TRACE_ARRAY( msg, array, len )
    #define SMTC_HAL_TRACE_PACKARRAY( ... )

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

#endif  // __SMTC_HAL_DBG_TRACE_H__

/* --- EOF ------------------------------------------------------------------ */
