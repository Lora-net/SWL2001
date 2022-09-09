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
#include "lorawan_certification.h"
#include "smtc_real_defs.h"
#include "smtc_beacon_sniff.h"
#include "smtc_d2d.h"
#include "radio_planner.h"
#include "fifo_ctrl.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Init the LoRaWAN stack
 *
 * @param [in] rp Pointer on radio planner object
 */
void lorawan_api_init( radio_planner_t* rp );

/**
 * @brief Get the current LoRaWAN region
 *
 * @return smtc_real_region_types_t Current region
 */
smtc_real_region_types_t lorawan_api_get_region( void );

/**
 * @brief Set the current LoRaWAN region
 *
 * @param [in] region_type LoRaWAN region
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type );

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
                                           uint8_t packet_type, uint32_t target_time_ms );

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
                                                   uint8_t data_len, uint8_t packet_type, uint32_t target_time_ms );

/**
 * @brief Send a LoRaWAN cid request
 *
 * @param [in] cid_req Command ID request by the User LINK_CHECK_REQ or DEVICE_TIME_REQ
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_send_stack_cid_req( cid_from_device_t cid_req );

/**
 * @brief Send a join request
 *
 * @param [in] target_time_ms
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_join( uint32_t target_time_ms );

/**
 * @brief Returns the join status
 *
 * @return join_status_t the join status. NOT_JOINED: the device is not connected
 *                                        JOINING: the device is trying to join
 *                                        JOINED: the device is joined to a network
 *
 */
join_status_t lorawan_api_isjoined( void );

/**
 * @brief Clear the join status (reset to NOT_JOINED)
 *
 */
void lorawan_api_join_status_clear( void );

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
status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t dr_strategy );

/**
 * @brief Get the current datarate strategy
 *
 * @return dr_strategy_t Current datatate strategy
 */
dr_strategy_t lorawan_api_dr_strategy_get( void );

/**
 * @brief Set user custom datarate
 *
 * @param [in] custom_dr Custom datarate
 */
void lorawan_api_dr_custom_set( uint32_t* custom_dr );

/**
 * @brief Runs the MAC layer state machine. Must be called periodically by the application. Not timing critical. Can be
 * interrupted.
 *
 * @return lr1mac_states_t return the lorawan state machine state
 */
lr1mac_states_t lorawan_api_process( void );

/**
 * @brief Reload the LoraWAN context saved in the flash
 */
void lorawan_api_context_load( void );

/**
 * @brief Save The LoraWAN context in the flash
 */
void lorawan_api_context_save( void );

/**
 * @brief Get the snr of the last user receive packet
 *
 * @return int16_t last snr
 */
int16_t lorawan_api_last_snr_get( void );

/**
 * @brief Get the snr of the last user receive packet
 *
 * @return int16_t last rssi
 */
int16_t lorawan_api_last_rssi_get( void );

/**
 * @brief Reload the factory Config in the LoraWAN Stack
 */
void lorawan_api_factory_reset( void );

/**
 * @brief Get Device activation mode
 *
 * @return lr1mac_activation_mode_t Activation mode: OTAA or ABP
 */
lr1mac_activation_mode_t lorawan_api_get_activation_mode( void );

/**
 * @brief Set Device activation mode
 *
 * @param [in] activation_mode Activation mode: OTAA or ABP
 */
void lorawan_api_set_activation_mode( lr1mac_activation_mode_t activation_mode );

/**
 * @brief Return the Max payload length allowed for the next transmit
 * @remark  DataRate + FOPTS + region  dependant ( )
 * @remark  In any case if user set a too long payload, the send method will answer by an error status
 *
 * @return uint32_t The max payload allowed
 */
uint32_t lorawan_api_next_max_payload_length_get( void );

/**
 * @brief Return the DevAddr of the device
 *
 * @return uint32_t The devaddr
 */
uint32_t lorawan_api_devaddr_get( void );

/**
 * @brief Get the DevEUI of the device
 *
 * @param [out] dev_eui The Device EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_deveui( uint8_t* dev_eui );

/**
 * @brief Set the DevEUI of the device
 *
 * @param [in] dev_eui The Device EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_deveui( const uint8_t* dev_eui );

/**
 * @brief Set the AppKey of the device
 *
 * @param [in] app_key The LoRaWan 1.0.x application Key
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_appkey( const uint8_t* app_key );

/**
 * @brief Get the join_eui of the device
 *
 * @param [out] join_eui The current Join EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_joineui( uint8_t* join_eui );

/**
 * @brief Set the join_eui of the device
 *
 * @param [in] join_eui The Join EUI
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_joineui( const uint8_t* join_eui );

/**
 * @brief Return the next transmission power
 *
 * @return uint8_t the next transmission power
 */
uint8_t lorawan_api_next_power_get( void );

/**
 * @brief Return the returns the next datarate
 *
 * @return uint8_t the next datarate
 */
uint8_t lorawan_api_next_dr_get( void );

/**
 * @brief Return the returns the next Tx Frequency
 *
 * @return uint32_t the next transmission frequency
 */
uint32_t lorawan_api_next_frequency_get( void );

/**
 * @brief Return the returns the max datarate of all enabled channels
 *
 * @return uint8_t  the max data rate
 */
uint8_t lorawan_api_max_tx_dr_get( void );

/**
 * @brief Return the returns the min datarate of all enabled channels
 *
 * @return uint8_t the min data rate
 */
uint8_t lorawan_api_min_tx_dr_get( void );

/**
 * @brief Return the returns the current data rate mask of all enabled channels
 *
 * @return uint16_t the mask data rate
 */
uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( void );

/**
 * @brief returns the current state of the MAC layer.
 * @remark  If the MAC is not in the idle state, the user cannot call any methods except the lorawan_api_process()
 *          and the lorawan_api_state_get() functions
 *
 * @return lr1mac_states_t THe current state of the stack
 */
lr1mac_states_t lorawan_api_state_get( void );

/**
 * @brief returns the number of reset
 *
 * @return uint16_t the number of resets
 */
uint16_t lorawan_api_nb_reset_get( void );

/**
 * @brief returns the last devnonce
 *
 * @return uint16_t the last devnonce
 */
uint16_t lorawan_api_devnonce_get( void );

/**
 * @brief returns the reception window used by the downlink
 *
 * @return receive_win_t Reception window
 */
receive_win_t lorawan_api_rx_window_get( void );

/**
 * @brief returns the min time to perform a new join request
 *
 * @return uint32_t The time before a new join request can be issued
 */
uint32_t lorawan_api_next_join_time_second_get( void );

/**
 * @brief when > 0, returns the min time to perform a new uplink request
 *
 * @return int32_t Miniam time before the stack is able to perform a new uplink
 */
int32_t lorawan_api_next_free_duty_cycle_ms_get( void );

/**
 * @brief Enable / disable the dutycycle
 *
 * @param [in] dtc_type Duty cycle type as described in @ref smtc_dtc_enablement_type_t
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_duty_cycle_enable_set( smtc_dtc_enablement_type_t dtc_type );

/**
 * @brief Get status Enable / disable dutycycle
 *
 * @return smtc_dtc_enablement_type_t
 */
smtc_dtc_enablement_type_t lorawan_api_duty_cycle_enable_get( void );

/**
 * @brief return the last uplink frame counter
 *
 * @return uint32_t Last frame counter
 */
uint32_t lorawan_api_fcnt_up_get( void );

/**
 * @brief Get the LoRaWAN radio planner hook ID
 *
 * @return uint8_t The used hook it
 */
uint8_t lorawan_api_rp_hook_id_get( void );

/**
 * @brief Enable/disable class C
 *
 * @param [in] enable true to enable, false to disable
 */
void lorawan_api_class_c_enabled( bool enable );

/**
 * @brief Start class C
 */
void lorawan_api_class_c_start( void );

/**
 * @brief Stop class C
 */
void lorawan_api_class_c_stop( void );

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
                                                                     const uint8_t mc_app_skey[LORAWAN_KEY_SIZE] );

/**
 * @brief Configure a multicast group address
 *
 * @param [in] mc_group_id      The multicast group id that will be configured (0 to 3)
 * @param [in] mc_group_address The multicast group addr
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_set_group_address( uint8_t mc_group_id, uint32_t mc_group_address );

/**
 * @brief Get a multicast group configuration
 *
 * @param [in]  mc_group_id      The multicast group id
 * @param [out] mc_group_address The current multicast group addr for chosen group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_group_address( uint8_t mc_group_id, uint32_t* mc_group_address );

/**
 * @brief Get the current running status of a multicast session
 *
 * @remark In class B a session will be marcked as running only after the beacon reception
 *
 * @param mc_group_id
 * @param session_running
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_running_status( uint8_t mc_group_id, bool* session_running );

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
                                                                   uint32_t* freq, uint8_t* dr );

/**
 * @brief Start a class C multicast on a previously configured group id
 *
 * @param [in] mc_group_id  The multicast group id
 * @param [in] freq         Rx frequency
 * @param [in] dr           Rx Datarate
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr );

/**
 * @brief Stop the chosen class C multicast session
 *
 * @param [in] mc_group_id The multicast group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_stop_session( uint8_t mc_group_id );

/**
 * @brief Stop all class C multicast sessions
 *
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_c_stop_all_sessions( void );

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
                                                                   uint8_t* dr, uint8_t* ping_slot_periodicity );

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
                                                              uint8_t ping_slot_periodicity );

/**
 * @brief Stop the chosen class B multicast session
 *
 * @param [in] mc_group_id The multicast group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_b_stop_session( uint8_t mc_group_id );

/**
 * @brief Stop all class B multicast sessions
 *
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_b_stop_all_sessions( void );

/**
 * @brief Get the ack bit status corresponding to the last uplink
 *
 * @return uint8_t
 */
uint8_t lorawan_api_rx_ack_bit_get( void );

/**
 * @brief Get the frame pending bit status to know if downlink opportunity is required
 *
 * @return uint8_t
 */
uint8_t lorawan_api_rx_fpending_bit_get( void );

/**
 * @brief Set the threshold number of uplinks without downlink before reset stack
 *
 * @param [in] no_rx_packet_reset_threshold
 */
void lorawan_api_set_no_rx_packet_threshold( uint16_t no_rx_packet_reset_threshold );

/**
 * @brief Get the configured threshold number of uplink without downlink before reset stack
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_no_rx_packet_threshold( void );

/**
 * @brief Get the current value of internal adr ack cnt that is used for reset threshold trigger
 *
 * @remark The adr_ack_cnt values is not incremented during nb trans and will also fallow the backoff strategy in case
 * device is in network controlled mode. The value is reset when a downlink happened
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_current_adr_ack_cnt( void );

/**
 * @brief Get the threshold number of uplinks without downlink in mobile mode before going network controlled
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_current_no_rx_packet_in_mobile_mode_cnt( void );

/**
 * @brief Reset the counter of uplinks without downlink in mobile mode before going network controlled
 */
void lorawan_api_reset_no_rx_packet_in_mobile_mode_cnt( void );

/**
 * @brief Get the current value of internal "tx without rx" counter
 *
 * @remark This counter is incremented at each tx done (even during nb trans) and reset when a downlink happened
 *
 * @return uint16_t
 */
uint16_t lorawan_api_get_current_no_rx_packet_cnt( void );

/**
 * @brief Certification: Set the status of the Modem LoRaWAN certification
 * @remark  To authorized LoRaWAN certification in modem
 *
 * @param [in] enable true to enable, false to disable
 */
void lorawan_api_modem_certification_set( uint8_t enable );

/**
 * @brief Certification: Get the status of the LoRaWAN certification
 * @remark  Is enabled by the Test Tool
 *
 * @return true if enabled
 * @return false if disabled
 */
bool lorawan_api_certification_is_enabled( void );

/**
 * @brief Certification: Build Class B Beacon Status Indication frame
 *
 * @param [in]  beacon_buffer        Beacon buffer
 * @param [in]  beacon_buffer_length Length of the buffer
 * @param [out] tx_buffer            Uplink buffer
 * @param [out] tx_buffer_length     Uplink buffer length
 * @param [in]  rssi                 Beacon rssi
 * @param [in]  snr                  Beacon snr
 * @param [in]  beacon_dr            Beacon datarate
 * @param [in]  beacon_freq          Beacon frequency
 */
void lorawan_api_certification_build_beacon_rx_status_ind( uint8_t* beacon_buffer, uint8_t beacon_buffer_length,
                                                           uint8_t* tx_buffer, uint8_t* tx_buffer_length, int8_t rssi,
                                                           int8_t snr, uint8_t beacon_dr, uint32_t beacon_freq );

/**
 * @brief Certification: Get the status of the Modem LoRaWAN certification
 * @remark  Is certification is authorized in modem
 *
 * @return uint8_t Modem LoRaWAN certification status
 */
uint8_t lorawan_api_modem_certification_is_enabled( void );

/**
 * @brief Certification: Get the requested class of the certification mode
 *
 * @return lorawan_certification_class_t the requested class
 */
lorawan_certification_class_t lorawan_api_certification_get_requested_class( void );

/**
 * @brief Certification: call LoRaWAN Certification state machine
 *
 * @param [in]  rx_buffer        Reception buffer
 * @param [in]  rx_buffer_length Reception buffer length
 * @param [out] tx_buffer        Uplink buffer
 * @param [out] tx_buffer_length Uplink buffer length
 * @param [out] tx_fport         Uplink fport
 * @return lorawan_certification_parser_ret_t
 */
lorawan_certification_parser_ret_t lorawan_api_certification( uint8_t* rx_buffer, uint8_t rx_buffer_length,
                                                              uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                              uint8_t* tx_fport );

/**
 * @brief Certification: get uplink periodicity
 *
 * @return uint16_t uplink periodicity
 */
uint16_t lorawan_api_certification_get_ul_periodicity( void );

/**
 * @brief Certification: Get frame type
 *
 * @return true
 * @return false
 */
bool lorawan_api_certification_get_frame_type( void );

/**
 * @brief Certification: Get the CW configuration requested by the TCL
 *
 * @param [out] timeout_s Timeout for CW
 * @param [out] frequency CW frequency
 * @param [out] tx_power  CW power
 */
void lorawan_api_certification_get_cw_config( uint16_t* timeout_s, uint32_t* frequency, int8_t* tx_power );

/**
 * @brief Certification: Get if the CW was requested by the TCL
 *
 * @return true
 * @return false
 */
bool lorawan_api_certification_is_cw_running( void );

/**
 * @brief Certification: Set CW as stopped
 *
 */
void lorawan_api_certification_cw_set_as_stopped( void );

/**
 * @brief Certification: Get the status of beacon rx status indication control
 *
 * @return true
 * @return false
 */
bool lorawan_api_certification_get_beacon_rx_status_ind_ctrl( void );

/**
 * @brief return true if stack receive a link adr request
 * @remark reset the flag automatically each time the upper layer call this function
 *
 * @return true
 * @return false
 */
bool lorawan_api_available_link_adr_get( void );

/**
 * @brief return the current stack obj
 *
 * @return lr1_stack_mac_t* the pointer on stack
 */
lr1_stack_mac_t* lorawan_api_stack_mac_get( void );

/**
 * @brief Get the stack fifo pointer
 *
 * @return fifo_ctrl_t* The pointer of the fifo
 */
fifo_ctrl_t* lorawan_api_get_fifo_obj( void );

/**
 * @brief Set network type
 *
 * @param [in] network_type true : public, false : private
 */
void lorawan_api_set_network_type( bool network_type );

/**
 * @brief  get network type
 *
 * @return true public network
 * @return false private network
 */
bool lorawan_api_get_network_type( void );

/**
 * @brief Get current nb trans value
 *
 * @return uint8_t nb trans value
 */
uint8_t lorawan_api_nb_trans_get( void );

/**
 * @brief Set nb trans value
 * @remark Nb trans have to be smaller than 16
 *
 * @param [in] nb_trans nb trans value
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans );

/**
 * @brief Get the current crystal error
 *
 * @return uint32_t Crystal error
 */
uint32_t lorawan_api_get_crystal_error( void );

/**
 * @brief Set the crystal error
 *
 * @param [in] crystal_error Crystal error
 */
void lorawan_api_set_crystal_error( uint32_t crystal_error );

/**
 * @brief Get the LoRaWAN spec version
 *
 * @return lr1mac_version_t
 */
lr1mac_version_t lorawan_api_get_spec_version( void );

/**
 * @brief Get the Regional Parameters spec version
 *
 * @return lr1mac_version_t
 */
lr1mac_version_t lorawan_api_get_regional_parameters_version( void );

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
                                                uint32_t* fractional_second );

/**
 * @brief Get if Network time is still valid or not
 *
 * @return true
 * @return false
 */
bool lorawan_api_is_time_valid( void );

/**
 * @brief
 *
 * @return uint32_t last timestamp when clock is received
 */
uint32_t lorawan_api_get_timestamp_last_device_time_ans_s( void );

/**
 * @brief Get the left delais before to concider device time no more valid
 *
 * @return uint32_t
 */
uint32_t lorawan_api_get_time_left_connection_lost( void );

/**
 * @brief Configure the callback for the stack when will received the network time sync
 *
 * @param [in] device_time_callback callback that will be called in case a device time answer happened
 * @param [in] context              context of the callback
 * @param [in] rx_timestamp_s
 */
void lorawan_api_set_device_time_callback( void ( *device_time_callback )( void* context, uint32_t rx_timestamp_s ),
                                           void* context, uint32_t rx_timestamp_s );

/**
 * @brief Set delay in seconds to concider time no more valid if no time sync received
 *
 * @param [in] delay_s
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_device_time_invalid_delay_s( uint32_t delay_s );

/**
 * @brief Get delay in seconds to concider time no more valid if no time sync received
 *
 * @return uint32_t
 */
uint32_t lorawan_api_get_device_time_invalid_delay_s( void );

/**
 * @brief Get the Margin and the Gateway count returned by the LinkCheckAns mac command
 *
 * @param [out] margin The demodulation margin
 * @param [out] gw_cnt The gateway count
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_link_check_ans( uint8_t* margin, uint8_t* gw_cnt );

/**
 * @brief Get Device Time Request status
 *
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_device_time_req_status( void );

/**
 * @brief Set the LBT parameters
 *
 * @param [in] listen_duration_ms   duration of the listen task
 * @param [in] threshold_dbm        threshold in dbm to decide if the channel is free or busy
 * @param [in] bw_hz                bandwith in hertz to listen a channel
 */
void lorawan_api_lbt_set_parameters( uint32_t listen_duration_ms, int16_t threshold_dbm, uint32_t bw_hz );

/**
 * @brief Get the configured lbt parameters
 *
 * @param [out] listen_duration_ms  duration of the listen task
 * @param [out] threshold_dbm       threshold in dbm
 * @param [out] bw_hz               bandwith in hertz
 */
void lorawan_api_lbt_get_parameters( uint32_t* listen_duration_ms, int16_t* threshold_dbm, uint32_t* bw_hz );

/**
 * @brief  Enable/Disable LBT service
 *
 * @param [in] enable true to enable lbt service, false to disable it
 */
void lorawan_api_lbt_set_state( bool enable );

/**
 * @brief Return the current enabled state of the lbt service
 *
 * @return true if service is currently enabled
 * @return false  if service is currently disabled
 */
bool lorawan_api_lbt_get_state( void );

/**
 * @brief Enable the class B
 *
 * @param [in] enable true to enable class B, false to disable
 */
void lorawan_api_class_b_enabled( bool enable );

/**
 * @brief start beacon sniffing
 */
void lorawan_api_beacon_sniff_start( void );

/**
 * @brief stop beacon sniffing
 */
void lorawan_api_beacon_sniff_stop( void );

/**
 * @brief Get the beacon metadata
 *
 * @param [out] beacon_metadata The beacon metadata
 */
void lorawan_api_beacon_get_metadata( smtc_beacon_metadata_t* beacon_metadata );

/**
 * @brief Get Ping Slot Info Request status
 *
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_get_ping_slot_info_req_status( void );

/**
 * @brief Set the ping-slot periodicity as described in Link layer specification [TS001]
 *
 * @param [in] ping_slot_periodicity Ping slot periodicity
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_ping_slot_periodicity( uint8_t ping_slot_periodicity );

/**
 * @brief Get the ping-slot periodicity as described in Link layer specification [TS001]
 *
 * @return uint8_t
 */
uint8_t lorawan_api_get_ping_slot_periodicity( void );

/**
 * @brief Get the status of class B bit
 *
 * @return true
 * @return false
 */
bool lorawan_api_get_class_b_status( void );

/**
 * @brief  Convert LoRaWAN Datarate to SF and bandwidth
 *
 * @param [in]  in_dr  Datarate
 * @param [out] out_sf Corresponding SF
 * @param [out] out_bw Corresponding bandwith
 */
void lorawan_api_lora_dr_to_sf_bw( uint8_t in_dr, uint8_t* out_sf, lr1mac_bandwidth_t* out_bw );

/**
 * @brief Get the LoRaWAN Frequency factor to convert freq to 24bits
 *
 * @return uint8_t
 */
uint8_t lorawan_api_get_frequency_factor( void );

/**
 * @brief Get status of push network downlink (mac commands, beacon, ..) to the user
 *
 * @return true
 * @return false
 */
bool lorawan_api_get_status_push_network_downlink_to_user( void );

/**
 * @brief Set status of push network downlink (mac commands, beacon, ..) to the user
 *
 * @param [in] enable
 */
void lorawan_api_set_status_push_network_downlink_to_user( bool enable );

/**
 * @brief Set the ADR ACK limit and ADR ACK delay regarding the ADR fallback in case no downlink are received
 *
 * @param [in] adr_ack_limit Accepted value: ( adr_ack_limit > 1 ) && ( adr_ack_limit < 128 )
 * @param [in] adr_ack_delay Accepted value: ( adr_ack_delay > 1 ) && ( adr_ack_delay < 128 )
 * @return status_lorawan_t The status of the operation
 */
status_lorawan_t lorawan_api_set_adr_ack_limit_delay( uint8_t adr_ack_limit, uint8_t adr_ack_delay );

/**
 * @brief Get the ADR ACK limit and ADR ACK delay configured regarding the ADR fallback in case no downlink are received
 *
 * @param [out] adr_ack_limit the configured adr ack limit
 * @param [out] adr_ack_delay the configured adr ack delay
 */
void lorawan_api_get_adr_ack_limit_delay( uint8_t* adr_ack_limit, uint8_t* adr_ack_delay );

/**
 * @brief Device To Device Reques Tx
 *
 * @param [in] multi_cast_group_id  The multicast group identifier
 * @param [in] fport                The LoRaWAN FPort on which the uplink is done
 * @param [in] priority             The priority of the D2D uplink
 * @param [in] payload              The data to be sent
 * @param [in] payload_size         The number of bytes from payload to be sent
 * @param [in] nb_rep               The number of repetitions for the D2D uplink
 * @param [in] nb_ping_slot_tries   The number of ping slot tries before stop
 * @param [in] ping_slots_mask      The mask defining autorised ping slots for this uplink (shall be in line with
 * current ping slot periodicity)
 * @param [in] ping_slots_mask_size The mask size
 * @return smtc_class_b_d2d_status_t
 */
smtc_class_b_d2d_status_t lorawan_api_class_b_d2d_request_tx( rx_session_type_t multi_cast_group_id, uint8_t fport,
                                                              uint8_t priority, const uint8_t* payload,
                                                              uint8_t payload_size, uint8_t nb_rep,
                                                              uint16_t nb_ping_slot_tries, uint8_t* ping_slots_mask,
                                                              uint8_t ping_slots_mask_size );

/**
 * @brief Get the next max payload length for multicast class B session
 *
 * @param [in] multi_cast_group_id The group ID
 * @return uint8_t The next max payload length
 */
uint8_t lorawan_api_class_b_d2d_next_max_payload_length_get( rx_session_type_t multi_cast_group_id );

/**
 * @brief Direct call to radio planner irq callback
 *
 * @param [in] rp Radio Planner pointer
 */
void lorawan_rp_callback_api( radio_planner_t* rp );

#ifdef __cplusplus
}
#endif

#endif  // __LORAWAN_API_H__

/* --- EOF ------------------------------------------------------------------ */
