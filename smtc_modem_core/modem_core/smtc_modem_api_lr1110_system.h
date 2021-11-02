/*!
 * @file      smtc_modem_api_lr1110_system.h
 *
 * @brief     System api definition for modem on LR1110
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

#ifndef SMTC_MODEM_API_LR1110_SYSTEM_H
#define SMTC_MODEM_API_LR1110_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_system.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Read and return the Unique Identifier of the LR1110
 *
 * @param [in] context Chip implementation context
 * @param [out] unique_identifier The buffer to be filled with the Unique Identifier of the LR1110. It is up to the
 * application to ensure unique_identifier is long enough to hold the unique identifier
 *
 * @returns Operation status
 *
 * @see LR1110_SYSTEM_UID_LENGTH
 */
lr1110_status_t smtc_modem_lr1110_system_read_uid( const void* context, lr1110_system_uid_t unique_identifier );

/*!
 * @brief Read and return the Join EUI of the LR1110
 *
 * @param [in] context Chip implementation context
 * @param [out] join_eui The buffer to be filled with Join EUI of the LR1110. It is up to the application to ensure
 * join_eui is long enough to hold the join EUI
 *
 * @returns Operation status
 *
 * @see LR1110_SYSTEM_JOIN_EUI_LENGTH
 */
lr1110_status_t smtc_modem_lr1110_system_read_join_eui( const void* context, lr1110_system_join_eui_t join_eui );

/*!
 * @brief Read and return the PIN of the LR1110
 *
 * @param [in] context Chip implementation context
 * @param [out] pin The buffer to be filled with PIN of the LR1110. It is up to the application to ensure pin is long
 * enough to hold the PIN
 *
 * @returns Operation status
 *
 * @see LR1110_SYSTEM_PIN_LENGTH
 */
lr1110_status_t smtc_modem_lr1110_system_read_pin( const void* context, lr1110_system_pin_t pin );

/*!
 * @brief Read and return the PIN of the LR1110 based on EUIs provided as parameters
 *
 * @param [in] context Chip implementation context
 * @param [in] device_eui Custom Device EUI
 * @param [in] join_eui Custom Join EUI
 * @param [in] rfu Parameter RFU - shall be set to 0x00
 * @param [out] pin The buffer to be filled with PIN of the LR1110. It is up to the application to ensure pin is long
 * enough to hold the PIN
 *
 * @returns Operation status
 *
 * @see LR1110_SYSTEM_PIN_LENGTH
 */
lr1110_status_t smtc_modem_lr1110_system_read_pin_custom_eui( const void* context, lr1110_system_uid_t device_eui,
                                                              lr1110_system_join_eui_t join_eui, uint8_t rfu,
                                                              lr1110_system_pin_t pin );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_API_LR1110_SYSTEM_H

/* --- EOF ------------------------------------------------------------------ */
