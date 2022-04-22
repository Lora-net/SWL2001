/**
 * @file      smtc_basic_modem_lr11xx_api_extension.h
 *
 * @brief     Modem API extension for Basic Modem with lr11xx radio
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

#ifndef SMTC_BASIC_MODEM_LR11XX_API_EXTENSION_H
#define SMTC_BASIC_MODEM_LR11XX_API_EXTENSION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Modem PIN code length in byte
 */
#define SMTC_MODEM_PIN_LENGTH 4

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Get the modem PIN code
 *
 * @remark This command can only be used with LR11XX radio
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] chip_pin  4-byte PIN code
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_pin( uint8_t stack_id, uint8_t chip_pin[SMTC_MODEM_PIN_LENGTH] );

/*!
 * @brief Get the modem chip EUI
 *
 * @remark This command can only be used with LR11XX radio
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] chip_eui  8-byte chip EUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_chip_eui( uint8_t stack_id, uint8_t chip_eui[SMTC_MODEM_EUI_LENGTH] );

/*!
 * @brief Derive keys
 *
 * @remark This command can only be used with LR11XX radio
 *           Derives application key taking saved dev_eui (default set to chip_eui) and saved join_eui
 *
 * @param [in]  stack_id  Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_derive_keys( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_BASIC_MODEM_LR11XX_API_EXTENSION_H

/* --- EOF ------------------------------------------------------------------ */
