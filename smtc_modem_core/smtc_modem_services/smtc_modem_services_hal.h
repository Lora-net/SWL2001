/**
 * @file      smtc_modem_services_hal.h
 *
 * @brief     API that needs to be implemented by the end user to allow modem_services to link.
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
#ifndef SMTC_MODEM_SERVICES_HAL_H
#define SMTC_MODEM_SERVICES_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types

#include "smtc_modem_services_config.h"  // Defined by the project

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#ifndef LOG_PRINT
#warning "Compiling without logging support."
#warning "Define a variadic macro LOG_PRINT( level, ...) in 'smtc_modem_services_config.h' to enable logging."
#define LOG_PRINT( ... )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
typedef enum radio_return_code_e
{
    MODEM_SERVICES_RADIO_OK = 0,
    MODEM_SERVICES_RADIO_ERROR,
} radio_return_code_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Computes the LoRaMAC payload encryption
 *
 * @param [in]  raw_buffer    Data buffer
 * @param [in]  size          Data buffer size
 * @param [in]  aes_ctr_nonce The AES CTR nonce to be used for encryption
 * @param [out] enc_buffer    Encrypted buffer
 */
void smtc_modem_services_aes_encrypt( const uint8_t* raw_buffer, uint16_t size, uint8_t aes_ctr_nonce[14],
                                      uint8_t* enc_buffer );

/**
 * @brief  Return elapsed time in seconds since a global common epoch.
 *
 * @remark It is supposed that this returns the time in seconds via a RTC for
 * instance.
 */
uint32_t smtc_modem_services_get_time_s( void );

/**
 * @brief Get dm_upload counter
 *
 * @remark This is defined in modem_api.h
 *         Maybe this could be stored locally to the service, and provided to
 * the device management functionality instead
 */
uint32_t smtc_modem_services_get_dm_upload_sctr( void );

/**
 * @brief Set dm_upload counter
 *
 * @remark This is defined in modem_api.h
 *         Maybe this could be stored locally to the service, and provided to
 * the device management functionality instead
 */
void smtc_modem_services_set_dm_upload_sctr( uint32_t ctr );

/*!
 * @brief Abstraction function for lr11xx function that get the GNSS context status
 *
 * This function returns the GNSS context status as a raw buffer. It is possible to use
 * lr11xx_gnss_parse_context_status_buffer to obtain the details of the context status.
 *
 * @param [in] radio_ctx Chip implementation context
 * @param [out] buff Pointer to a buffer to be filled with context status information. Must be at least
 * 9 bytes long. It is up to the caller to ensure there is enough place in this buffer. The call is garenteed
 *
 * @returns Operation status
 *
 */
radio_return_code_t smtc_modem_services_lr11xx_gnss_get_context_status( const void* radio_ctx, uint8_t buff[9] );

/*!
 * @brief Abstraction function for lr11xx function lr11xx_gnss_push_dmc_msg
 * Host receives an update from the network or assembles itself the update message and send it to the LR11XX.
 *
 * @param [in] radio_ctx Chip implementation context
 * @param [in] buff buffer containing the update the network
 * @param [in] buff_len len of this buffer
 *
 * @returns Operation status
 */
radio_return_code_t smtc_modem_services_lr11xx_gnss_push_dmc_msg( const void* radio_ctx, uint8_t* buff,
                                                                  uint16_t buff_len );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_SERVICES_HAL_H

/* --- EOF ------------------------------------------------------------------
 */
