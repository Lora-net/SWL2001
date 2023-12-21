/**
 * @file      mw_common.h
 *
 * @brief     Middleware common functions definition.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#ifndef MW_COMMON_H__
#define MW_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define MW_RETURN_ON_FAILURE( expr )                                               \
    if( ( expr ) == false )                                                        \
    {                                                                              \
        SMTC_MODEM_HAL_TRACE_ERROR( "%s failed - line %d\n", __func__, __LINE__ ); \
        return MW_RC_FAILED;                                                       \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Middleware return code definition
 */
typedef enum mw_return_code_e
{
    MW_RC_OK,     //!< No error
    MW_RC_BUSY,   //!< A middleware task is already running
    MW_RC_FAILED  //!< Failed to execute the requested task
} mw_return_code_t;

/**
 * @brief Middleware version descriptor definition
 */
typedef struct mw_version_s
{
    uint8_t major;  //!< Major value
    uint8_t minor;  //!< Minor value
    uint8_t patch;  //!< Patch value
} mw_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Configure the lr11xx radio for scan
 *
 * @param [in] radio_context Chip implementation context
 *
 * @return a boolean set to true for success, false otherwise
 */
bool mw_radio_configure_for_scan( const void* radio_context );

/*!
 * @brief Set the lr11xx radio to sleep. To be called by middlewares at the end of the RP task_done handler.
 *
 * @param [in] radio_context Chip implementation context
 */
void mw_radio_set_sleep( const void* radio_context );

#ifdef __cplusplus
}
#endif

#endif  // MW_COMMON_H__

/* --- EOF ------------------------------------------------------------------ */