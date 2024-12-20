/*!
 * \file      modem_tx_protocol_manager.h
 *
 * \brief
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

#ifndef __MODEM_TX_PROTOCOL_MANAGER__H
#define __MODEM_TX_PROTOCOL_MANAGER__H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "radio_planner.h"
#include "lr1_stack_mac_layer.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define FAILSAFE_TPM_S 300
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
typedef enum tx_protocol_manager_tx_type
{
    TX_PROTOCOL_NONE                        = 0x00,
    TX_PROTOCOL_JOIN_LORA                   = 0x01,  //!<
    TX_PROTOCOL_TRANSMIT_LORA               = 0x02,  //!<
    TX_PROTOCOL_TRANSMIT_CID                = 0x03,
    TX_PROTOCOL_TRANSMIT_LORA_AT_TIME       = 0x04,
    TX_PROTOCOL_TRANSMIT_TEST_MODE          = 0x05,
    TX_PROTOCOL_TRANSMIT_LORA_CERTIFICATION = 0x06,
} tx_protocol_manager_tx_type_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   TPM Initialization
 * \remark  This function is called  by the modem's upper layer itself, it shouldn't be useful at the application layer
 * \retval  None
 */
void modem_tx_protocol_manager_init( radio_planner_t* rp );

/*!
 * @brief tx_protocol_manager_request function is called each time a LoRaWAN transmission will be processed
 * \remark  This function is called  by the modem's upper layer itself, it shouldn't be useful at the application layer
 * @param request_type could be Join, Normal LoRaWAN, Or CID cmd
 * @param fport LoRaWAN fport
 * @param fport_enabled LoRaWAN fport enable
 * @param data LoRaWAN user payload
 * @param data_len LoRaWAN user payload length
 * @param packet_type LoRaWAN packet type (confirmed/unconfirmed)
 * @param target_time_ms Target starting time of the LoRaWAN packet
 * @param stack_id Stack id
 * @return LoRaWAN status
 */
status_lorawan_t tx_protocol_manager_request( tx_protocol_manager_tx_type_t request_type, uint8_t fport,
                                              bool fport_enabled, const uint8_t* data, uint8_t data_len,
                                              lr1mac_layer_param_t packet_type, uint32_t target_time_ms,
                                              uint8_t stack_id );

/*! @brief tx_protocol_manager_is_busy indicate if tpm is in used in order to pre process LoRaWan transmission (could be
 * LBT, CSMA , or Relay pre process)
 * \remark  This function is called  by the modem's upper layer itself, it shouldn't be useful at the application layer
 * @param NONE
 * @return uint32_t indicated time to sleep , if the return value is equal to 0 it is meaning that tpm is ready to
 * execute the LoRaWAN tx or that tpm has no more task to execute
 */
uint32_t tx_protocol_manager_is_busy( void );

/*! @brief this function is used to manage LoRaWAN retransmission or any frame transmit as an answer by the stack itself
 * (network frame) \remark  This function is called  by the modem's upper layer itself, it shouldn't be useful at the
 * application layer
 * @param  NONE
 * @return NONE
 */
void tx_protocol_manager_lr1mac_stand_alone_tx( void );

/*! @brief this function is called by the send manager service to update the status of the TX_DONE_EVENT
 * @param  NONE
 * @return true if current transmission has been aborted
 */
bool tx_protocol_manager_tx_is_aborted( void );

/*! @brief this function is called by leave cmd to abort every thing
 * @param  NONE
 * @return void
 */
void tx_protocol_manager_abort( void );
#ifdef __cplusplus
}
#endif

#endif  //__MODEM_TX_PROTOCOL_MANAGER__H
