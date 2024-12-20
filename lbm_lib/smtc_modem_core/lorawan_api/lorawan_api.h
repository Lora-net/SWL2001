/*!
 * \file      lorawan_api.h
 *
 * \brief     Lorawan abstraction layer definitions.
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

#ifndef __LORAWAN_API_H__
#define __LORAWAN_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "lr1mac_defs.h"
#include "lr1_stack_mac_layer.h"
#include "smtc_real_defs.h"
#if defined( ADD_CLASS_B )
#include "smtc_beacon_sniff.h"
#endif
#include "radio_planner.h"
#include "fifo_ctrl.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define PANIC_IF_STACK_ID_TOO_HIGH( x )                          \
    do                                                           \
    {                                                            \
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( x < NUMBER_OF_STACKS ); \
    } while( 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * Lorawan keys size in bytes
 */
#define LORAWAN_KEY_SIZE 16
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum lorawan_multicast_rc_e
{
    LORAWAN_MC_RC_OK,
    LORAWAN_MC_RC_ERROR_BAD_ID,
    LORAWAN_MC_RC_ERROR_BUSY,
    LORAWAN_MC_RC_ERROR_CRYPTO,
    LORAWAN_MC_RC_ERROR_PARAM,
    LORAWAN_MC_RC_ERROR_INCOMPATIBLE_SESSION,
    LORAWAN_MC_RC_ERROR_CLASS_NOT_ENABLED,
    LORAWAN_MC_RC_ERROR_NOT_IMPLEMENTED,
} lorawan_multicast_rc_t;

typedef struct lorawan_down_metadata_s
{
    lr1mac_down_metadata_t* lr1mac_down_metadata;

} lorawan_down_metadata_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Init the LoRaWAN stack
 *
 * @param [in] rp Pointer on radio planner object
 */
void lorawan_api_init( radio_planner_t* rp, uint8_t stack_id,
                       void ( *lr1mac_downlink_callback )( lr1_stack_mac_down_data_t* push_context ) );

/**
 * @brief Get the current LoRaWAN region
 *
 * @return smtc_real_region_types_t Current region
 */
smtc_real_region_types_t lorawan_api_get_region( uint8_t stack_id );

/**
 * @brief Set the current LoRaWAN region
 *
 * @param [in] region_type LoRaWAN region
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type, uint8_t stack_id );

/**
 * @brief  Sends an uplink as soon as possible at a chosen time
 *
 * @param [in] fport          Uplink port
 * @param [in] fport_enabled  Fport present or not
 * @param [in] data           User payload
 * @param [in] data_len       User payload length
 * @param [in] packet_type    User packet type : UNCONF_DATA_UP, CONF_DATA_UP,
 * @param [in] target_time_ms RTC time when the packet should be sent
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_payload_send( uint8_t fport, bool fport_enabled, const uint8_t* data, uint8_t data_len,
                                           lr1mac_layer_param_t packet_type, uint32_t target_time_ms,
                                           uint8_t stack_id );

/**
 * @brief
 *
 * @param [in] fport          Uplink port
 * @param [in] fport_enabled  Fport present or not
 * @param [in] data           User payload
 * @param [in] data_len       User payload length
 * @param [in] packet_type    User packet type : UNCONF_DATA_UP, CONF_DATA_UP,
 * @param [in] target_time_ms RTC time when the packet shall be sent
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_payload_send_at_time( uint8_t fport, bool fport_enabled, const uint8_t* data,
                                                   uint8_t data_len, lr1mac_layer_param_t packet_type,
                                                   uint32_t target_time_ms, uint8_t stack_id );

/**
 * @brief Send a LoRaWAN cid request
 *
 * @param [in] cid_req_list  Commands ID list requested by the User LINK_CHECK_REQ, DEVICE_TIME_REQ or
 *                           PING_SLOT_INFO_REQ
 * @param [in] cid_req_list_size  Number of command in list
 * @param [in] target_time_ms RTC time when the packet shall be sent
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_send_stack_cid_req( uint8_t* cid_req_list, uint8_t cid_req_list_size,
                                                 uint32_t target_time_ms, uint8_t stack_id );

/**
 * @brief Send a join request
 *
 * @param [in] target_time_ms
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_join( uint32_t target_time_ms, uint8_t stack_id );

/**
 * @brief Returns the join status
 *
 * @return join_status_t the join status. NOT_JOINED: the device is not connected
 *                                        JOINING: the device is trying to join
 *                                        JOINED: the device is joined to a network
 *
 */
join_status_t lorawan_api_isjoined( uint8_t stack_id );

/**
 * @brief Clear the join status (reset to NOT_JOINED)
 *
 */
void lorawan_api_join_status_clear( uint8_t stack_id );

/**
 * @brief Set datarate strategy
 * @remark The current implementation support 4 different dataRate Strategy :
 *    STATIC_ADR_MODE                   for static Devices with ADR managed by the Network
 *    MOBILE_LONGRANGE_DR_DISTRIBUTION  for Mobile Devices with strong Long range requirement
 *    MOBILE_LOWPER_DR_DISTRIBUTION     for Mobile Devices with strong Low power requirement
 *    USER_DR_DISTRIBUTION              User datarate distribution (can be defined with @ref lorawan_api_dr_custom_set)
 *    JOIN_DR_DISTRIBUTION              Dedicated for Join requests
 * @param [in] dr_strategy Datarate strategy (describe above)
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t dr_strategy, uint8_t stack_id );

/**
 * @brief Get the current datarate strategy
 *
 * @return dr_strategy_t Current datatate strategy
 */
dr_strategy_t lorawan_api_dr_strategy_get( uint8_t stack_id );

/**
 * @brief Set user custom datarate
 *
 * @param [in] custom_dr Custom datarate
 */
void lorawan_api_dr_custom_set( uint8_t* custom_dr, uint8_t stack_id );

/**
 * @brief Get previously set user custom datarate
 *
 * @param [in] custom_dr
 * @param [in] stack_id
 */
void lorawan_api_dr_custom_get( uint8_t* custom_dr, uint8_t stack_id );

/**
 * @brief Set user custom datarate for JoinRequest
 *
 * @param [in] custom_dr Custom datarate
 */
void lorawan_api_dr_join_distribution_set( uint8_t* custom_dr, uint8_t stack_id );

/**
 * @brief Runs the MAC layer state machine. Must be called periodically by the application. Not timing critical. Can be
 * interrupted.
 *
 * @return lr1mac_states_t return the lorawan state machine state
 */
lr1mac_states_t lorawan_api_process( uint8_t stack_id );

/**
 * @brief Reload the LoraWAN context saved in the flash
 */
void lorawan_api_context_load( uint8_t stack_id );

/**
 * @brief Reload the factory Config in the LoraWAN Stack
 */
void lorawan_api_factory_reset( uint8_t stack_id );

/**
 * @brief Get Device activation mode
 *
 * @return lr1mac_activation_mode_t Activation mode: OTAA or ABP
 */
lr1mac_activation_mode_t lorawan_api_get_activation_mode( uint8_t stack_id );

/**
 * @brief Set Device activation mode
 *
 * @param [in] activation_mode Activation mode: OTAA or ABP
 */
void lorawan_api_set_activation_mode( lr1mac_activation_mode_t activation_mode, uint8_t stack_id );

/**
 * @brief Return the Max payload length allowed for the next transmit
 * @remark  DataRate + FOPTS + region  dependant ( )
 * @remark  In any case if user set a too long payload, the send method will answer by an error status
 *
 * @return uint32_t The max payload allowed
 */
uint32_t lorawan_api_next_max_payload_length_get( uint8_t stack_id );

/**
 * @brief Return the DevAddr of the device
 *
 * @return uint32_t The devaddr
 */
uint32_t lorawan_api_devaddr_get( uint8_t stack_id );

/**
 * @brief Set the DevAddr of the device
 * @remark Before use this command the stack must be set to ABP with
 * lorawan_api_set_activation_mode(ACTIVATION_MODE_ABP, stack_id)
 *
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_devaddr_set( uint32_t dev_addr, uint8_t stack_id );

/**
 * @brief Get the DevEUI of the device
 *
 * @param [out] dev_eui The Device EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_deveui( uint8_t* dev_eui, uint8_t stack_id );

/**
 * @brief Set the DevEUI of the device
 *
 * @param [in] dev_eui The Device EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_deveui( const uint8_t* dev_eui, uint8_t stack_id );

/**
 * @brief Get the join_eui of the device
 *
 * @param [out] join_eui The current Join EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_joineui( uint8_t* join_eui, uint8_t stack_id );

/**
 * @brief Set the join_eui of the device
 *
 * @param [in] join_eui The Join EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_joineui( const uint8_t* join_eui, uint8_t stack_id );

/**
 * @brief Set the AppKey of the device
 *
 * @param [in] app_key The LoRaWan 1.0.x gen application Key
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_appkey( const uint8_t* app_key, uint8_t stack_id );

/**
 * @brief Set the NwkKey of the device
 *
 * @param [in] nwk_key The LoRaWan 1.0.x application Key
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_nwkkey( const uint8_t* nwk_key, uint8_t stack_id );

/**
 * @brief Return the next transmission power
 *
 * @return uint8_t the next transmission power
 */
uint8_t lorawan_api_next_power_get( uint8_t stack_id );

/**
 * @brief Return the returns the next datarate
 *
 * @return uint8_t the next datarate
 */
uint8_t lorawan_api_next_dr_get( uint8_t stack_id );

/**
 * @brief Return the modulation type from a datarate
 *
 * @param datarate
 * @return modulation_type_t LORA, FSK, LR_FHSS
 */
modulation_type_t lorawan_api_get_modulation_type_from_datarate( uint8_t stack_id, uint8_t datarate );

/**
 * @brief Return the returns the next Tx Frequency
 *
 * @return uint32_t the next transmission frequency
 */
uint32_t lorawan_api_next_frequency_get( uint8_t stack_id );

/**
 * @brief Return the returns the max datarate of all enabled channels
 *
 * @return uint8_t  the max data rate
 */
uint8_t lorawan_api_max_tx_dr_get( uint8_t stack_id );

/**
 * @brief Return the returns the min datarate of all enabled channels
 *
 * @return uint8_t the min data rate
 */
uint8_t lorawan_api_min_tx_dr_get( uint8_t stack_id );

/**
 * @brief Return the returns the current data rate mask of all enabled channels
 *
 * @return uint16_t the mask data rate
 */
uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( uint8_t stack_id );

/**
 * @brief returns the current state of the MAC layer.
 * @remark  If the MAC is not in the idle state, the user cannot call any methods except the lorawan_api_process()
 *          and the lorawan_api_state_get() functions
 *
 * @return lr1mac_states_t THe current state of the stack
 */
lr1mac_states_t lorawan_api_state_get( uint8_t stack_id );

/**
 * @brief returns the last devnonce
 *
 * @return uint16_t the last devnonce
 */
uint16_t lorawan_api_devnonce_get( uint8_t stack_id );

/**
 * @brief returns the min time to perform a new join request
 *
 * @return uint32_t The time before a new join request can be issued
 */
uint32_t lorawan_api_next_join_time_second_get( uint8_t stack_id );

/**
 * @brief Is duty-cycle supported by the region
 *
 * @param stack_id
 * @return true     duty-cycle supported
 * @return false    duty-cycle not supported
 */
bool lorawan_api_is_dtc_supported( uint8_t stack_id );

/**
 * @brief when > 0, returns the min time to perform a new uplink request
 *
 * @return uint32_t TimeOff before the stack is able to perform a new uplink
 */
uint32_t lorawan_api_next_network_free_duty_cycle_ms_get( uint8_t stack_id );

/**
 * @brief return the last uplink frame counter
 *
 * @return uint32_t Last frame counter
 */
uint32_t lorawan_api_fcnt_up_get( uint8_t stack_id );

/**
 * @brief Enable/disable class C
 *
 * @param [in] enable true to enable, false to disable
 */
void lorawan_api_class_c_enabled( bool enable, uint8_t stack_id );

/**
 * @brief Start class C
 */
void lorawan_api_class_c_start( uint8_t stack_id );

/**
 * @brief Is class C started
 *
 * @param stack_id
 * @return true
 * @return false
 */
bool lorawan_api_class_c_is_running( uint8_t stack_id );

/**
 * @brief Stop class C
 */
void lorawan_api_class_c_stop( uint8_t stack_id );

/**
 * @brief Configure a multicast group session keys
 *
 * @param [in] mc_group_id The multicast group id
 * @param [in] mc_ntw_skey The multicast network session key for the group
 * @param [in] mc_app_skey The multicast application session key for the group
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_set_group_session_keys( uint8_t       mc_group_id,
                                                                     const uint8_t mc_ntw_skey[LORAWAN_KEY_SIZE],
                                                                     const uint8_t mc_app_skey[LORAWAN_KEY_SIZE],
                                                                     uint8_t       stack_id );

/**
 * @brief Configure a multicast group address
 *
 * @param [in] mc_group_id      The multicast group id that will be configured (0 to 3)
 * @param [in] mc_group_address The multicast group addr
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_set_group_address( uint8_t mc_group_id, uint32_t mc_group_address,
                                                                uint8_t stack_id );

/**
 * @brief Get a multicast group configuration
 *
 * @param [in]  mc_group_id      The multicast group id
 * @param [out] mc_group_address The current multicast group addr for chosen group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_group_address( uint8_t mc_group_id, uint32_t* mc_group_address,
                                                                uint8_t stack_id );

/**
 * @brief Set the frame counter min and max allowed for a multicast group
 *
 * @param [in] mc_group_id          The multicast group id
 * @param [in] mc_min_fcnt_down     Frame counter min accepted
 * @param [in] mc_max_fcnt_down     Frame counter max accepted
 * @param [in] stack_id             LoRaWAN stack ID
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_set_fcnt_down_range( uint8_t mc_group_id, uint32_t mc_min_fcnt_down,
                                                                  uint32_t mc_max_fcnt_down, uint8_t stack_id );

/**
 * @brief Get the frame counter min and max allowed for a multicast group
 *
 * @param [in]  mc_group_id          The multicast group id
 * @param [out] mc_min_fcnt_down     Frame counter min accepted
 * @param [out] mc_max_fcnt_down     Frame counter max accepted
 * @param [in]  stack_id             LoRaWAN stack ID
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_fcnt_down_range( uint8_t mc_group_id, uint32_t* mc_min_fcnt_down,
                                                                  uint32_t* mc_max_fcnt_down, uint8_t stack_id );

/**
 * @brief Derive multicast group session keys from the multicast encrypted group key
 *
 * @param [in] mc_group_id       Multicast group id
 * @param [in] mc_key_encrypted  Encrypted multicast key key received
 * @param [in] stack_id          Stack identifier
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_derive_group_keys( uint8_t       mc_group_id,
                                                                const uint8_t mc_key_encrypted[LORAWAN_KEY_SIZE],
                                                                uint8_t       stack_id );

/**
 * @brief Get the current running status of a multicast session
 *
 * @remark In class B a session will be marcked as running only after the beacon reception
 *
 * @param mc_group_id
 * @param session_running
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_running_status( uint8_t mc_group_id, bool* session_running,
                                                                 uint8_t stack_id );

/**
 * @brief Get the status of a class C multicast session
 *
 * @param [in]  mc_group_id         The multicast group id
 * @param [out] is_session_started  Boolean to indicate if session is active
 * @param [out] freq                Rx frequency
 * @param [out] dr                  Rx Datarate
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                   uint32_t* freq, uint8_t* dr, uint8_t stack_id );

/**
 * @brief Start a class C multicast on a previously configured group id
 *
 * @param [in] mc_group_id  The multicast group id
 * @param [in] freq         Rx frequency
 * @param [in] dr           Rx Datarate
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                              uint8_t stack_id );

/**
 * @brief Stop the chosen class C multicast session
 *
 * @param [in] mc_group_id The multicast group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_stop_session( uint8_t mc_group_id, uint8_t stack_id );

/**
 * @brief Stop all class C multicast sessions
 *
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_stop_all_sessions( uint8_t stack_id );

/**
 * @brief Get the status of a class B multicast session
 *
 * @param [in]  mc_group_id             The multicast group id
 * @param [out] is_session_started      Boolean to indicate if session is active
 * @param [out] waiting_beacon_to_start Boolean to indicate if session is waiting for beacon
 * @param [out] freq                    The session Rx frequency
 * @param [out] dr                      The session Rx Datarate
 * @param [out] ping_slot_periodicity   The session ping slot periodicity
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_b_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                   bool* waiting_beacon_to_start, uint32_t* freq,
                                                                   uint8_t* dr, uint8_t* ping_slot_periodicity,
                                                                   uint8_t stack_id );

/**
 * @brief Start a class B multicast on a previously configured group id
 *
 * @param [in] mc_group_id           The multicast group id
 * @param [in] freq                  The session Rx frequency
 * @param [in] dr                    The session Rx Datarate
 * @param [in] ping_slot_periodicity The session ping slot periodicity
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_b_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                              uint8_t ping_slot_periodicity, uint8_t stack_id );

/**
 * @brief Stop the chosen class B multicast session
 *
 * @param [in] mc_group_id The multicast group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_b_stop_session( uint8_t mc_group_id, uint8_t stack_id );

/**
 * @brief Stop all class B multicast sessions
 *
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_b_stop_all_sessions( uint8_t stack_id );

/**
 * @brief set the ack bit for uplink
 *
 * @return void
 */
void lorawan_api_tx_ack_bit_set( uint8_t stack_id, bool enable );

/**
 * @brief get the ack bit for uplink
 *
 * @return uint8_t
 */
bool lorawan_api_tx_ack_bit_get( uint8_t stack_id );

/**
 * @brief Disable Rx windows after a Tx
 *
 * @param stack_id
 * @param disable_rx_windows
 */
void lorawan_api_set_no_rx_windows( uint8_t stack_id, uint8_t disable_rx_windows );

/**
 * @brief Get the status disable/enable Rx windows after a Tx
 *
 * @param stack_id
 * @return uint8_t
 */
uint8_t lorawan_api_get_no_rx_windows( uint8_t stack_id );

/**
 * @brief Set the threshold number of uplinks without downlink before reset stack
 *
 * @param [in] no_rx_packet_reset_threshold
 */
void lorawan_api_set_no_rx_packet_threshold( uint16_t no_rx_packet_reset_threshold, uint8_t stack_id );

/**
 * @brief Get the configured threshold number of uplink without downlink before reset stack
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_no_rx_packet_threshold( uint8_t stack_id );

/**
 * @brief Get the current value of internal adr ack cnt that is used for reset threshold trigger
 *
 * @remark The adr_ack_cnt values is not incremented during nb trans and will also fallow the backoff strategy in case
 * device is in network controlled mode. The value is reset when a downlink happened
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_current_adr_ack_cnt( uint8_t stack_id );

/**
 * @brief Get the threshold number of uplinks without downlink in mobile mode before going network controlled
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_current_no_rx_packet_in_mobile_mode_cnt( uint8_t stack_id );

/**
 * @brief Reset the counter of uplinks without downlink in mobile mode before going network controlled
 */
void lorawan_api_reset_no_rx_packet_in_mobile_mode_cnt( uint8_t stack_id );

/**
 * @brief Get the current value of internal "tx without rx" counter
 *
 * @remark This counter is incremented at each tx done (even during nb trans) and reset when a downlink happened
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_current_no_rx_packet_cnt( uint8_t stack_id );

/**
 * @brief Get the current value of internal duration in second since "tx without rx"
 *
 * @remark This value is reset when a downlink happened
 *
 * @return uint32_t
 */
uint32_t lorawan_api_get_current_no_rx_packet_cnt_since_s( uint8_t stack_id );

/**
 * @brief Get the Bypass join backoff duty-cycle status
 * @remark  The LoRaWAN certification_set enable/disable the backoff bypass
 *
 * @param stack_id
 * @return true     bypassed is enabled
 * @return false    bypassed is not enabled
 */
bool lorawan_api_join_duty_cycle_backoff_bypass_get( uint8_t stack_id );

/**
 * @brief Bypass join backoff duty-cycle
 * @remark  The LoRaWAN certification_set enable/disable the backoff bypass
 *
 * @param stack_id
 * @param [in] enable    true to bypass
 */
void lorawan_api_join_duty_cycle_backoff_bypass_set( uint8_t stack_id, bool enable );

/**
 * @brief Certification: Set the status of the Modem LoRaWAN certification
 * @remark  To authorized LoRaWAN certification in modem
 *
 * @param [in] enable true to enable, false to disable
 */
void lorawan_api_modem_certification_set( uint8_t enable, uint8_t stack_id );

/**
 * @brief Certification: Get the status of the Modem LoRaWAN certification
 * @remark  Is certification is authorized in modem
 *
 * @return uint8_t Modem LoRaWAN certification status
 */
uint8_t lorawan_api_modem_certification_is_enabled( uint8_t stack_id );

/**
 * @brief return true if stack receive a link adr request
 * @remark reset the flag automatically each time the upper layer call this function
 *
 * @return true
 * @return false
 */
bool lorawan_api_available_link_adr_get( uint8_t stack_id );

/**
 * @brief return the current stack obj
 *
 * @return lr1_stack_mac_t* the pointer on stack
 */
lr1_stack_mac_t* lorawan_api_stack_mac_get( uint8_t stack_id );

/**
 * @brief Get the stack fifo pointer
 *
 * @return fifo_ctrl_t* The pointer of the fifo
 */
fifo_ctrl_t* lorawan_api_get_fifo_obj( uint8_t stack_id );

/**
 * @brief Set network type
 *
 * @param [in] network_type true : public, false : private
 */
void lorawan_api_set_network_type( bool network_type, uint8_t stack_id );

/**
 * @brief  get network type
 *
 * @return true public network
 * @return false private network
 */
bool lorawan_api_get_network_type( uint8_t stack_id );

/**
 * @brief Get current nb trans value
 *
 * @return uint8_t nb trans value
 */
uint8_t lorawan_api_nb_trans_get( uint8_t stack_id );

/**
 * @brief Set nb trans value
 * @remark Nb trans have to be smaller than 16
 *
 * @param [in] nb_trans nb trans value
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans, uint8_t stack_id );

/**
 * @brief Get the current crystal error
 *
 * @return uint32_t Crystal error
 */
uint32_t lorawan_api_get_crystal_error( uint8_t stack_id );

/**
 * @brief Set the crystal error
 *
 * @param [in] crystal_error Crystal error
 */
void lorawan_api_set_crystal_error( uint32_t crystal_error, uint8_t stack_id );

/**
 * @brief Get the LoRaWAN spec version
 *
 * @return lr1mac_version_t
 */
lr1mac_version_t lorawan_api_get_spec_version( uint8_t stack_id );

/**
 * @brief Get the Regional Parameters spec version
 *
 * @return lr1mac_version_t
 */
lr1mac_version_t lorawan_api_get_regional_parameters_version( uint8_t stack_id );

/**
 * @brief Convert RTC to GPS epoch time
 *
 * @param [in]  rtc_ms              rtc time
 * @param [out] seconds_since_epoch Number of seconds since epoch
 * @param [out] fractional_second   Fractional second
 * @return true                 Time is valid
 * @return false                Time is not valid
 */
bool lorawan_api_convert_rtc_to_gps_epoch_time( uint32_t rtc_ms, uint32_t* seconds_since_epoch,
                                                uint32_t* fractional_second, uint8_t stack_id );

/**
 * @brief Get if Network time is still valid or not
 *
 * @return true
 * @return false
 */
bool lorawan_api_is_time_valid( uint8_t stack_id );

/**
 * @brief
 *
 * @return uint32_t last timestamp when clock is received
 */
uint32_t lorawan_api_get_timestamp_last_device_time_ans_s( uint8_t stack_id );

/**
 * @brief Get the left delais before to concider device time no more valid
 *
 * @return uint32_t
 */
uint32_t lorawan_api_get_time_left_connection_lost( uint8_t stack_id );

/**
 * @brief Set delay in seconds to concider time no more valid if no time sync received
 *
 * @param [in] delay_s
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_device_time_invalid_delay_s( uint32_t delay_s, uint8_t stack_id );

/**
 * @brief Get delay in seconds to concider time no more valid if no time sync received
 *
 * @return uint32_t
 */
uint32_t lorawan_api_get_device_time_invalid_delay_s( uint8_t stack_id );

/**
 * @brief Get the Margin and the Gateway count returned by the LinkCheckAns mac command
 *
 * @param [out] margin The demodulation margin
 * @param [out] gw_cnt The gateway count
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_link_check_ans( uint8_t* margin, uint8_t* gw_cnt, uint8_t stack_id );

/**
 * @brief Get Device Time Request status
 *
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_device_time_req_status( uint8_t stack_id );

/**
 * @brief Enable the class B
 *
 * @param [in] enable true to enable class B, false to disable
 */
void lorawan_api_class_b_enabled( bool enable, uint8_t stack_id );
/**
 * @brief get if  class B iss enabled
 *
 * @param [out] enable true  class B enabled
 */
bool lorawan_api_class_b_enabled_get( uint8_t stack_id );

/**
 * @brief start beacon sniffing
 */
void lorawan_api_beacon_sniff_start( uint8_t stack_id );

/**
 * @brief stop beacon sniffing
 */
void lorawan_api_beacon_sniff_stop( uint8_t stack_id );

#if defined( ADD_CLASS_B )
/**
 * @brief Get the beacon metadata
 *
 * @param [out] beacon_statistics The beacon statistics
 */
void lorawan_api_beacon_get_statistics( smtc_beacon_statistics_t* beacon_statistics, uint8_t stack_id );
#endif  // ADD_CLASS_B

/**
 * @brief Get Ping Slot Info Request status
 *
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_ping_slot_info_req_status( uint8_t stack_id );

/**
 * @brief Set the ping-slot periodicity as described in Link layer specification [TS001]
 *
 * @param [in] ping_slot_periodicity Ping slot periodicity
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_ping_slot_periodicity( uint8_t ping_slot_periodicity, uint8_t stack_id );

/**
 * @brief Get the ping-slot periodicity as described in Link layer specification [TS001]
 *
 * @return uint8_t
 */
uint8_t lorawan_api_get_ping_slot_periodicity( uint8_t stack_id );

/**
 * @brief Get the status of class B bit
 *
 * @return true
 * @return false
 */
bool lorawan_api_get_class_b_status( uint8_t stack_id );

/**
 * @brief  Convert LoRaWAN Datarate to SF and bandwidth
 *
 * @param [in]  in_dr  Datarate
 * @param [out] out_sf Corresponding SF
 * @param [out] out_bw Corresponding bandwith
 */
void lorawan_api_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw, uint8_t stack_id );

/**
 * @brief Get the LoRaWAN Frequency factor to convert freq to 24bits
 *
 * @return uint8_t
 */
uint8_t lorawan_api_get_frequency_factor( uint8_t stack_id );

/**
 * @brief Set the ADR ACK limit and ADR ACK delay regarding the ADR fallback in case no downlink are received
 *
 * @param [in] adr_ack_limit Accepted value: ( adr_ack_limit > 1 ) && ( adr_ack_limit < 128 )
 * @param [in] adr_ack_delay Accepted value: ( adr_ack_delay > 1 ) && ( adr_ack_delay < 128 )
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_adr_ack_limit_delay( uint8_t adr_ack_limit, uint8_t adr_ack_delay, uint8_t stack_id );

/**
 * @brief Get the ADR ACK limit and ADR ACK delay configured regarding the ADR fallback in case no downlink are received
 *
 * @param [out] adr_ack_limit the configured adr ack limit
 * @param [out] adr_ack_delay the configured adr ack delay
 */
void lorawan_api_get_adr_ack_limit_delay( uint8_t* adr_ack_limit, uint8_t* adr_ack_delay, uint8_t stack_id );

/**
 * @brief Get a list of frequencies enabled in the stack
 *
 * @param number_of_freq    The Number of freq return in list
 * @param freq_list         The list of frequencies
 * @param max_size          The max size of array to contains freq
 * @param stack_id          The stack ID requested
 * @return true
 * @return false            max size is < that the number of channel in region
 */
bool lorawan_api_get_current_enabled_frequencies_list( uint8_t* number_of_freq, uint32_t* freq_list, uint8_t max_size,
                                                       uint8_t stack_id );

/**
 * @brief Check if a frequency is valid according to current stack parameters
 *
 * @param [in] stack_id Stack identifier
 * @param [in] freq     Frequency to be checked
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_is_frequency_valid( uint8_t stack_id, uint32_t freq );

/**
 * @brief Check if a datarate is valid according to current stack parameters
 *
 * @param [in] stack_id Stack identifier
 * @param [in] dr       Frequency to be checked
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_is_datarate_valid( uint8_t stack_id, uint8_t dr );
/**
 * @brief abort stack
 *
 * @param [in] stack_id Stack identifier
 * @return status_lorawan_t
 */
void lorawan_api_core_abort( uint8_t stack_id );

/**
 * @brief Prepare next join , update next channel
 *
 * @param [in] stack_id Stack identifier
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_update_join_channel( uint8_t stack_id );

/**
 * @brief Prepare next transmission , update next channel
 *
 * @param [in] stack_id Stack identifier
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_update_next_tx_channel( uint8_t stack_id );

/**
 * @brief In case of transmission initiated by the stack itself (nwk ans or retransmission) , this function return the
 * schedule time of this next transmission
 *
 * @param [in] stack_id Stack identifier
 * @return uint32_t return target time of next transmission in ms
 */
uint32_t lorawan_api_get_time_of_nwk_ans( uint8_t stack_id );

/**
 * @brief In case of transmission initiated by the stack itself (nwk ans or retransmission) , update the
 * schedule time of this next transmission
 *
 * @param [in] stack_id Stack identifier
 * @param [in] uint32_t  target time of next transmission in ms
 */
void lorawan_api_set_time_of_nwk_ans( uint8_t stack_id, uint32_t target_time );

/**
 * @brief update the next transmission to start at time or asap;
 *
 * @param [in] stack_id Stack identifier
 * @param [in] bool  is_send_at_time :  true to transmit at time
 */
void lorawan_api_set_next_tx_at_time( uint8_t stack_id, bool is_send_at_time );

/**
 * @brief update the internal join status;
 *
 * @param [in] stack_id Stack identifier
 * @param [in] join_status_t  join_status
 */
void lorawan_api_set_join_status( uint8_t stack_id, join_status_t join_status );
#ifdef __cplusplus
}
#endif

#endif  // __LORAWAN_API_H__

/* --- EOF ------------------------------------------------------------------ */
