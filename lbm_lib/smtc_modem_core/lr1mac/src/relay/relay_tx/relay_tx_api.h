/*!
 * \file    relay_tx_api.h
 *
 * \brief   Main function for the relayed ED
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

#ifndef RELAY_TX_API_H
#define RELAY_TX_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lr1_stack_mac_layer.h"
#include "wake_on_radio_def.h"

typedef enum relay_tx_activation_mode_e
{
    RELAY_TX_ACTIVATION_MODE_DISABLED     = 0,
    RELAY_TX_ACTIVATION_MODE_ENABLE       = 1,
    RELAY_TX_ACTIVATION_MODE_DYNAMIC      = 2,
    RELAY_TX_ACTIVATION_MODE_ED_CONTROLED = 3
} relay_tx_activation_mode_t;
typedef enum relay_tx_sync_status_e
{
    RELAY_TX_SYNC_STATUS_INIT,
    RELAY_TX_SYNC_STATUS_UNSYNC,
    RELAY_TX_SYNC_STATUS_SYNC
} relay_tx_sync_status_t;

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
    uint8_t                    number_of_miss_wor_ack_to_switch_in_nosync_mode;
} relay_tx_config_t;

typedef struct wor_lr1_infos_s
{
    uint32_t dev_addr;
    uint32_t freq_hz;
    uint32_t crystal_error_ppm;
    uint8_t  dr;
    bool     is_join;
    uint8_t  payload_size;

} wor_lr1_infos_t;
typedef struct wor_tx_prepare_s
{
    uint32_t      target_time_ms;
    uint32_t      freq_hz;
    bool          at_time;  // send the WOR at time, if false target_time_ms can be ignored
    uint8_t       dr;
    uint8_t       sf;
    ral_lora_bw_t bw;
    uint8_t       payload_size;
} wor_tx_prepare_t;

typedef struct cb_return_param_s
{
    bool     abort;
    uint8_t  stack_id;
    uint32_t lr1_timing;
} cb_return_param_t;
/**
 * @brief Init relay TX data struture
 *
 * @param[in]   relay_stack_id  relay stack id
 * @return true     Success
 * @return false    Fail
 */
bool smtc_relay_tx_init( uint8_t relay_stack_id, radio_planner_t* rp, smtc_real_t* real,
                         void ( *free_callback )( void* free_context ), void*   free_context,
                         void ( *busy_callback )( void* busy_context ), void*   busy_context,
                         void ( *abort_callback )( void* abort_context ), void* abort_context );

/**
 * @brief Disable the WOR before a LoRaWAN Uplink
 *
 * @param[in]   relay_stack_id  relay stack id
 */
void smtc_relay_tx_disable( uint8_t relay_stack_id );

/**
 * @brief Enable the WOR before a LoRaWAN uplink
 *
 * @param[in]   relay_stack_id  relay stack id
 */
void smtc_relay_tx_enable( uint8_t relay_stack_id );

/**
 * @brief Check if WOR is enable before LoRaWAN Uplink
 *
 * @param[in]   relay_stack_id  relay stack id
 * @return true     WOR is enable before LoRaWAN Uplink
 * @return false    WOR is not enable
 */
bool smtc_relay_tx_is_enable( uint8_t relay_stack_id );

/**
 * @brief Return relay tx sync status
 *
 * @param[in]   relay_stack_id  relay stack id
 * @return  synchronisation status
 */
relay_tx_sync_status_t smtc_relay_tx_get_sync_status( uint8_t relay_stack_id );

/**
 * @brief Update the relay TX configuration
 *
 * @param[in]   relay_stack_id  relay stack id
 * @param[in]   config          New config
 * @return true     Success
 * @return false    Failed
 */
bool smtc_relay_tx_update_config( uint8_t relay_stack_id, const relay_tx_config_t* config );

/**
 * @brief   Return current relay TX configuration
 *
 * @param[in]   relay_stack_id  relay stack id
 * @param[out]  config          Relay TX configuration
 */
void smtc_relay_tx_get_config( uint8_t relay_stack_id, relay_tx_config_t* config );

/**
 * @brief Precompute the WOR frame (preamble length, synchronisation and payloa)
 *
 * @param[in]   relay_stack_id  relay stack id
 * @param[in]   target_time     Coarse time to send WOR, will be adjusted by function see @param wor_tx
 * @param[in]   dev_addr        LoRaWAN dev addr
 * @param[in]   lr1_freq_hz     LoRaWAN uplink frequency in Hz
 * @param[in]   lr1_dr          LoRaWAN uplink datarate
 * @param[out]  wor_tx          WOR TX physical parameter
 * @return true     Success
 * @return false    Failed to compute WOR
 */
bool smtc_relay_tx_prepare_wor( uint8_t relay_stack_id, uint32_t target_time, const wor_lr1_infos_t* lr1_infos,
                                wor_tx_prepare_t* wor_tx );

/**
 * @brief Load the last precomputed WOR frame in the radio planner
 *
 * @param[in]   relay_stack_id  relay stack id
 * @param[in]   wor_tx          WOR TX physical parameter
 * @return true     Success
 * @return false    Failed to load WOR
 */
bool smtc_relay_tx_send_wor( uint8_t relay_stack_id, const wor_tx_prepare_t* wor_tx );

/**
 * @brief Return RXR windows parameters
 *
 * @param[in]   relay_stack_id  relay stack id
 * @param[out]  dr              Datarate of RXR
 * @param[out]  freq            Frequency in Hz of RXR
 */
void smtc_relay_tx_get_rxr_param( uint8_t relay_stack_id, uint8_t tx_dr, uint8_t* dr, uint32_t* freq );

/**
 * @brief Return the maximum payload that can send through a relay
 *
 * @param[in]   relay_stack_id  relay stack id
 * @return uint8_t  Max size in byte
 */
uint8_t smtc_relay_get_tx_max_payload( uint8_t relay_stack_id );

/**
 * @brief Return ED crystal error in PPM
 *
 * @param[in]   relay_stack_id  relay stack id
 * @return uint32_t Value in PPM
 */
uint32_t smtc_relay_tx_get_crystal_error( uint8_t relay_stack_id );

/**
 * @brief Data has been received on RXR
 *
 * @param[in]   relay_stack_id  relay stack id
 */
void smtc_relay_tx_data_receive_on_rxr( uint8_t relay_stack_id );

#ifdef __cplusplus
}
#endif
#endif
