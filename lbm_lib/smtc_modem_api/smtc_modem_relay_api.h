/**
 * @file      smtc_modem_relay_api.h
 *
 * @brief     Generic Modem Relay API description
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#ifndef SMTC_MODEM_RELAY_API_H__
#define SMTC_MODEM_RELAY_API_H__

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

typedef enum smtc_modem_relay_tx_activation_mode_e
{
    SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_DISABLED,
    SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_ENABLE,
    SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_DYNAMIC,
    SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_ED_CONTROLLED,
} smtc_modem_relay_tx_activation_mode_t;

typedef enum smtc_modem_relay_tx_sync_status_e
{
    SMTC_MODEM_RELAY_TX_SYNC_STATUS_INIT,
    SMTC_MODEM_RELAY_TX_SYNC_STATUS_UNSYNC,
    SMTC_MODEM_RELAY_TX_SYNC_STATUS_SYNC
} smtc_modem_relay_tx_sync_status_t;

typedef struct smtc_modem_relay_tx_channel_config_s
{
    uint32_t freq_hz;
    uint32_t ack_freq_hz;
    uint8_t  dr;
} smtc_modem_relay_tx_channel_config_t;

typedef struct smtc_modem_relay_tx_config_s
{
    smtc_modem_relay_tx_channel_config_t  second_ch;
    bool                                  second_ch_enable;
    uint8_t                               backoff;
    smtc_modem_relay_tx_activation_mode_t activation;
    uint8_t                               smart_level;
    uint8_t                               number_of_miss_wor_ack_to_switch_in_nosync_mode;
} smtc_modem_relay_tx_config_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Return the activation mode of the relay tx
 *
 * @param[in]   stack_id    Stack identifier
 * @param[out]  mode        Activation mode of the relay tx
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This stack doesn't have the relay tx feature
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_relay_tx_get_activation_mode( uint8_t                                stack_id,
                                                                  smtc_modem_relay_tx_activation_mode_t* mode );

/**
 * @brief Return the relay configuration tx
 *
 * @param[in]   stack_id    Stack identifier
 * @param[out]  smtc_modem_relay_tx_config_t       the relay tx configuration
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_relay_tx_get_config( uint8_t stack_id, smtc_modem_relay_tx_config_t* config );
/**
 * @brief Return the synchronisation status of the relay tx stack
 *
 * @param[in]   stack_id    Stack identifier
 * @param[out]  status      Synchronisation status

 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This stack doesn't have the relay tx feature
 */
smtc_modem_return_code_t smtc_modem_relay_tx_get_sync_status( uint8_t                            stack_id,
                                                              smtc_modem_relay_tx_sync_status_t* status );
/**
 * @brief Return if the relay is currently enable or not
 *
 * Works regardless of the activation mode.
 *
 * @param[in]   stack_id    Stack identifier
 * @param[out]  is_enable   is relay tx mode enable(true) or not(false)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 *
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This stack doesn't have the relay tx feature
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_relay_tx_is_enable( uint8_t stack_id, bool* is_enable );

/**
 * @brief Enable the relay tx
 *
 * Only works if the ED is in SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_ED_CONTROLLED
 * @param[in]   stack_id        Stack identifier
 * @param[in]   smtc_modem_relay_tx_config_t   full relay config
 *
 * backoff_level set to 0 means that the LoRaWAN uplink will always be send regardless if the end-device has received a
 * WOR ACK. For other value, the end-device will send a LoRaWAN uplink each X WOR frames without a WOR ACK.
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This stack doesn't have the relay tx feature
 *                                          or is not in ED controlled mode
 *                                          or Relay Rx enabled by the network
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_relay_tx_enable( uint8_t                             stack_id,
                                                     const smtc_modem_relay_tx_config_t* relay_config );

/**
 * @brief Disable the relay tx
 *
 * Only works if the ED is in SMTC_MODEM_RELAY_TX_ACTIVATION_MODE_ED_CONTROLLED
 * @param[in]   stack_id    Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This stack doesn't have the relay tx feature or is not in ED controlled mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_relay_tx_disable( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_RELAY_API_H__

/* --- EOF
 * ------------------------------------------------------------------ */
