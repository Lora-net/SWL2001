/*!
 * \file    relay_tx_api.h
 *
 * \brief   Main function for the relayed ED
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

#ifndef __RELAY_TX_API_H__
#define __RELAY_TX_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "lr1_stack_mac_layer.h"
#include "wake_on_radio_def.h"

typedef enum relay_tx_activation_mode_e
{
    RELAY_TX_ACTIVATION_MODE_DISABLED,
    RELAY_TX_ACTIVATION_MODE_ENABLE,
    RELAY_TX_ACTIVATION_MODE_DYNAMIC,
    RELAY_TX_ACTIVATION_MODE_ED_CONTROLED
} relay_tx_activation_mode_t;

typedef struct relay_tx_channel_config_s
{
    uint32_t freq_hz;
    uint32_t ack_freq_hz;
    uint8_t  dr;
} relay_tx_channel_config_t;

typedef struct relay_tx_config_s
{
    relay_tx_channel_config_t  second_ch;
    bool                       second_ch_enable;
    uint8_t                    backoff;
    relay_tx_activation_mode_t activation;
    uint8_t                    smart_level;
} relay_tx_config_t;

/**
 * @brief Init relay TX data struture
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @return true     Success
 * @return false    Fail
 */
bool smtc_relay_tx_init( lr1_stack_mac_t* lr1mac );

/**
 * @brief Disable the WOR before a LoRaWAN Uplink
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 */
void smtc_relay_tx_disable( lr1_stack_mac_t* lr1mac );

/**
 * @brief Enable the WOR before a LoRaWAN uplink
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 */
void smtc_relay_tx_enable( lr1_stack_mac_t* lr1mac );

/**
 * @brief Check if WOR is enable before LoRaWAN Uplink
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @return true     WOR is enable before LoRaWAN Uplink
 * @return false    WOR is not enable
 */
bool smtc_relay_tx_is_enable( lr1_stack_mac_t* lr1mac );

/**
 * @brief Update the relay TX configuration
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @param[in]   config      New config
 * @return true     Success
 * @return false    Failed
 */
bool smtc_relay_tx_update_config( lr1_stack_mac_t* lr1mac, const relay_tx_config_t* config );

/**
 * @brief   Return current relay TX configuration
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @param[out]  config      Relay TX configuration
 */
void smtc_relay_tx_get_config( lr1_stack_mac_t* lr1mac, relay_tx_config_t* config );

/**
 * @brief Program to send WOR before the LoRaWAN uplink
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 */
void smtc_relay_tx_send_wor( lr1_stack_mac_t* lr1mac );

/**
 * @brief Return RXR windows parameters
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @param[out]  dr          Datarate of RXR
 * @param[out]  freq        Frequency in Hz of RXR
 */
void smtc_relay_tx_get_rxr_param( lr1_stack_mac_t* lr1mac, uint8_t* dr, uint32_t* freq );

/**
 * @brief Return the maximum payload that can send through a relay
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @return uint8_t  Max size in byte
 */
uint8_t smtc_relay_get_tx_max_payload( lr1_stack_mac_t* lr1mac );

/**
 * @brief Return ED crytstal error in PPM
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @return uint32_t Value in PPM
 */
uint32_t smtc_relay_tx_get_crystal_error( lr1_stack_mac_t* lr1mac );

/**
 * @brief Data has been received on RXR
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 */
void smtc_relay_tx_data_receive_on_rxr( lr1_stack_mac_t* lr1mac );

#ifdef __cplusplus
}
#endif
#endif
