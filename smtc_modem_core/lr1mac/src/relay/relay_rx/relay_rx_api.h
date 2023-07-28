/*!
 * \file    relay_rx_api.h
 *
 * \brief   Main function to interract with the relay (start,stop, configure,...)
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
#ifndef __RELAY_RX_API_H__
#define __RELAY_RX_API_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "lr1_stack_mac_layer.h"
#include "wake_on_radio.h"

typedef struct relay_channel_config_s
{
    uint32_t freq_hz;
    uint32_t ack_freq_hz;
    uint8_t  dr;
} relay_channel_config_t;

typedef struct relay_fwd_config_s
{
    uint32_t token_available;
    uint16_t bucket_size;
    uint16_t reload_rate;
    bool     unlimited_fwd;
} relay_fwd_config_t;

typedef enum relay_forward_limit_action_e
{
    LIMIT_CNT_SET_TO_0,
    LIMIT_CNT_SET_TO_RELOAD_RATE,
    LIMIT_CNT_SET_TO_MAX_VAL,
    LIMIT_CNT_DONT_CHANGE
} relay_forward_limit_action_t;

typedef enum relay_forward_limit_list_e
{
    LIMIT_OVERALL,
    LIMIT_GLOBAL_UPLINK,
    LIMIT_NOTIFY,
    LIMIT_JOINREQ,
    LIMIT__LAST_ELT,
} relay_forward_limit_list_t;

typedef struct relay_config_s
{
    wor_cad_periodicity_t  cad_period;
    relay_channel_config_t channel_cfg[MAX_WOR_CH];
    uint8_t                nb_wor_channel;
} relay_config_t;

typedef enum relay_filter_fwd_type_s
{
    RELAY_FILTER_FWD_TYPE_CLEAR,
    RELAY_FILTER_FWD_TYPE_FILTER,
    RELAY_FILTER_FWD_TYPE_FORWARD,
    RELAY_FILTER_FWD_TYPE_RFU,
} relay_filter_fwd_type_t;

typedef struct relay_stats_s
{
    uint32_t nb_cad1;  // Total CAD1 since start
    uint32_t nb_cad2;  // Total CAD2 since start
    uint32_t nb_cad2_ok;
    uint16_t nb_wor_fail;  // No WUS after double cad
    uint16_t nb_wor_ok;    // WUS ok (no mic check)
    uint16_t nb_rx_ok;     // RX receive
    uint16_t nb_rx_fail;   // No RX after a WUS
    uint16_t nb_ack_tx;    //
} relay_stats_t;

/**
 * @brief Init the relay data structure and FSM.
 *
 * Do only once
 *
 * @param[in]   lr1mac      LoRaWAN stack pointer
 * @param[in]   error_ppm   Relay crystal error
 * @param[in]   cad_to_rx   Relay CAD to RX in symbols
 * @return true     Init success
 * @return false    Init failled
 */
bool relay_init( lr1_stack_mac_t* lr1mac, wor_ack_ppm_error_t error_ppm, wor_ack_cad_to_rx_t cad_to_rx );

/**
 * @brief Stop the periodic CAD
 *
 * @param[in]   stop_to_fwd     true if relay is temporaly stop (to forward uplink)
 * @return true     Succes
 * @return false    Failed
 */
bool relay_stop( bool stop_to_fwd );

/**
 * @brief Start the periodic CAD
 *
 * @return true     Success
 * @return false    Failed
 */
bool relay_start( void );

/**
 * @brief Send a downlink to an ED on RXR
 *
 * @param[in]   stack_id    LoRaWAN stack ID
 * @param[in]   buffer      Buffer to send
 * @param[in]   len         Length of buffer
 */
void relay_fwd_dl( uint8_t stack_id, const uint8_t* buffer, uint8_t len );

/**
 * @brief Update relay config
 *
 * If relay is running, the relay will stop, apply the new config and restart.
 *
 * @param[in]   config      New config
 * @return true     Succesfuly applied
 * @return false    Failed to applied the new config
 */
bool relay_update_config( const relay_config_t* config );

/**
 * @brief Update join request filter/fwd list
 *
 * @param[in]   idx             Index of the rule
 * @param[in]   join_eui        Join EUI (could be partial)
 * @param[in]   len_join_eui    Lenght of Join EUI
 * @param[in]   dev_eui         Dev EUI (could be partial)
 * @param[in]   len_dev_eui     length of Dev EUI
 * @param[in]   action          Clear,Filter Or Forward
 */
void relay_fwd_join_request_update_rule( const uint8_t idx, const uint8_t* join_eui, const uint8_t len_join_eui,
                                         const uint8_t* dev_eui, const uint8_t len_dev_eui,
                                         const relay_filter_fwd_type_t action );

/**
 * @brief Add a trusted ED to the list
 *
 * @param[in]   idx             Index of the ED (between 0 and 15)
 * @param[in]   dev_addr        ED Dev ADDR
 * @param[in]   root_wor_skey   Root WOR session key
 * @param[in]   unlimited_fwd   True if ED has no fwd restriction
 * @param[in]   bucket_factor   Bucket factor
 * @param[in]   reload_rate     Reload rate
 * @param[in]   wfcnt32         Last knwonw wfcnt32
 * @return true     ED has been added to the list
 * @return false    Failed to add the ED
 */
bool relay_fwd_uplink_add_device( const uint8_t idx, const uint32_t dev_addr, const uint8_t* root_wor_skey,
                                  const bool unlimited_fwd, const uint8_t bucket_factor, const uint8_t reload_rate,
                                  const uint32_t wfcnt32 );

/**
 * @brief Remove an ED from the trusted list
 *
 * @param[in]   idx     Index of the ED (between 0 and 15)
 * @return true     ED has been removed from the list
 * @return false    Failed to remove the ED
 */
bool relay_fwd_uplink_remove_device( const uint8_t idx );

/**
 * @brief Return last knwown WFCNT32
 *
 * @param[in]   idx         Index of the ED (between 0 and 15)
 * @param[out]  wfcnt32     Last knwonw wfcnt32
 * @return true     Success
 * @return false    Failed to read WFCNT32
 */
bool relay_fwd_uplink_read_wfcnt32( const uint8_t idx, uint32_t* wfcnt32 );

/**
 * @brief Update forward limit of the relay
 *
 * @param[in]   limit       Limit type (global, notify, ...)
 * @param[in]   action      Action on current limit (set to 0/max, ...)
 * @param[in]   unlimitted  True for no fwd restriction
 * @param[in]   reload_rate Reload rate
 * @param[in]   factor      Multiplier factor
 */
void relay_fwd_update_fwd_limit( const relay_forward_limit_list_t limit, const relay_forward_limit_action_t action,
                                 const bool unlimitted, const uint16_t reload_rate, const uint8_t factor );

/**
 * @brief Return statistics
 *
 * @param[in]   stat        relay statistics
 */
void relay_get_stats( relay_stats_t* stat );

/**
 * @brief Print relay statistics
 *
 */
void relay_print_stats( void );

#ifdef __cplusplus
}
#endif
#endif
