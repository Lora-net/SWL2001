/**
 * @file      modem_core_common.h
 *
 * @brief     Common defines for modem_core source
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

#ifndef _MODEM_CORE_COMMON_H_
#define _MODEM_CORE_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_modem_services_hal.h"

#ifdef TEST
#include "unity.h"
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

// Basics

#ifndef MAX
/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( a ) > ( b ) ) ? ( a ) : ( b )
#endif

#ifndef MIN
/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( a ) < ( b ) ) ? ( a ) : ( b )
#endif

#ifndef UNUSED
#define UNUSED( x ) ( void ) ( x )
#endif

// Logging

#ifndef LOG_ERROR
#define LOG_ERROR( ... ) LOG_PRINT( "ERROR", __VA_ARGS__ )
#endif
#ifndef LOG_WARN
#define LOG_WARN( ... ) LOG_PRINT( "WARN", __VA_ARGS__ )
#endif
#ifndef LOG_INFO
#define LOG_INFO( ... ) LOG_PRINT( "INFO", __VA_ARGS__ )
#endif
#ifndef LOG_DEBUG
#define LOG_DEBUG( ... ) LOG_PRINT( "DEBUG", __VA_ARGS__ )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE !FALSE
#endif

// Export private functions for unit tests
#ifdef TEST
#define STATIC
#define STATIC_INLINE
#else
#define STATIC static
#define STATIC_INLINE static inline
#endif

// Disable these assertions outside of unit and integration tests
#ifndef TEST
#define TEST_ASSERT_NOT_NULL( ... )
#endif

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

#endif  // _MODEM_CORE_COMMON_H_

/* --- EOF ------------------------------------------------------------------ */
