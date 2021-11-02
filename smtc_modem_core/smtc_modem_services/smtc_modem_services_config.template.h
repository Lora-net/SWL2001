/**
 * @file      smtc_modem_services_config.h
 *
 * @brief     Configuration file for the API provided to modem_services
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

#ifndef SMTC_MODEM_SERVICES_CONFIG_H
#define SMTC_MODEM_SERVICES_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Variadic macro used in LOG_* macros
 *
 * @param level     String containing the name of the error level
 * @param ...       Variable argument list
 *
 * @exemple     This exemple uses the libc printf() function as a final backend
 */
#define LOG_PRINT( level, ... )                             \
    {                                                       \
        printf( "%s (%s:%d) ", level, __FILE__, __LINE__ ); \
        printf( __VA_ARGS__ );                              \
    }

/**
 * @brief   LOG_* macro with various levels
 *          Defining these macros will override the default implementation in module_services.
 *          This could be used to disable certain levels
 */
//#define LOG_ERROR( ... ) LOG_PRINT( "ERROR", __VA_ARGS__ )
//#define LOG_WARN( ... ) LOG_PRINT( "WARNING", __VA_ARGS__ )
//#define LOG_INFO( ... ) LOG_PRINT( "INFO", __VA_ARGS__ )
//#define LOG_DEBUG( ... ) LOG_PRINT( "DEBUG", __VA_ARGS__ )

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_SERVICES_HAL_H

/* --- EOF ------------------------------------------------------------------ */
