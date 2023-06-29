/*!
 * \file      lr1_mac_core.h
 *
 * \brief     LoRaWan core definition
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

#ifndef __LR1MAC_CORE_H__
#define __LR1MAC_CORE_H__
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include "stdint.h"
#include "lr1mac_defs.h"
#include "lr1_stack_mac_layer.h"
#include "smtc_duty_cycle.h"

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------------
 */

/**
 * @brief LoRaWAN stack initialisation
 *
 * @param lr1_mac_obj             // lr1mac object
 * @param real                    // Regional Abstraction Layer object
 * @param lbt_obj                 // Listen Before Talk object
 * @param dtc_obj                 // Duty cycle object
 * @param rp                      // Radio Planner object
 * @param otaa_abp_conf           // Activation mode, only OTAA is supported
 * @param push_callback           // Callback to push received downlink
 * @param push_context            // Context concerning the downlink
 */
void lr1mac_core_init( lr1_stack_mac_t* lr1_mac_obj, smtc_real_t* real, smtc_lbt_t* lbt_obj, smtc_dtc_t* dtc_obj,
                       radio_planner_t* rp, lr1mac_activation_mode_t        otaa_abp_conf,
                       void ( *push_callback )( void* push_context ), void* push_context );

/**
 * \brief Sends an uplink
 * \param [IN] uint8_t           fPort          Uplink Fport
 * \param [IN] const uint8_t*    dataInFport    User Payload
 * \param [IN] const uint8_t     sizeIn         User Payload Size
 * \param [IN] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,
 * \param [OUT] lr1mac_states_t         Current state of the LoraWan stack :
 * \param                                            => return LWPSATE_SEND if all is ok
 * \param                                            => return Error in case of payload too long
 * \param                                            => return Error In case of the Lorawan stack previous state is
   not equal to idle
*/

/**
 * @brief to Send a Join request
 *
 * @param lr1_mac_obj
 * @param target_time_ms      RTC time to transmit the join
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_join( lr1_stack_mac_t* lr1_mac_obj, uint32_t target_time_ms );

/**
 * @brief Reset the join status to NotJoined
 *
 * @param lr1_mac_obj
 */
void lr1mac_core_join_status_clear( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief abort LoRaWAN task
 * @remark Tx, Rx will be aborted
 *
 * @param lr1_mac_obj
 */
void lr1mac_core_abort( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set the datarate strategy of the device
 *
 * @param lr1_mac_obj
 * @param adrModeSelect
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_dr_strategy_set( lr1_stack_mac_t* lr1_mac_obj, dr_strategy_t adrModeSelect );

/**
 * @brief Get the datarate strategy of the device
 *
 * @param lr1_mac_obj
 * @return dr_strategy_t
 */
dr_strategy_t lr1mac_core_dr_strategy_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set the custom datarate distribution
 * @remark each nibble represent a datarate:
 *            => 0x0000008F DR0 will be used twice more than DR1, other DRs will not be used
 *
 * @param lr1_mac_obj
 * @param datarate_custom
 */
void lr1mac_core_dr_custom_set( lr1_stack_mac_t* lr1_mac_obj, uint32_t* datarate_custom );

/**
 * @brief Runs the MAC layer state machine.
 * @remark Must be called periodically by the application. Not timing critical. Can be interrupted.
 *
 * @param lr1_mac_obj
 * @return lr1mac_states_t   return the lorawan state machine state
 */
lr1mac_states_t lr1mac_core_process( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Reload the LoraWAN context saved in the flash
 *
 * @param lr1_mac_obj
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_context_load( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Save The LoraWAN context  in the flash
 *
 * @param lr1_mac_obj
 */
void lr1mac_core_context_save( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the snr of the last user receive packet
 *
 * @param lr1_mac_obj
 * @return int16_t
 */
int16_t lr1mac_core_last_snr_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the RSSI of the last user receive packet
 *
 * @param lr1_mac_obj
 * @return int16_t
 */
int16_t lr1mac_core_last_rssi_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Reload the factory Config in the LoraWAN Stack
 *
 * @param lr1_mac_obj
 */
void lr1mac_core_factory_reset( lr1_stack_mac_t* lr1_mac_obj );  // load factory MAC context from constructor

/**
 * @brief Set device activation mode
 *
 * @param [in] lr1_mac_obj     The lr1mac object
 * @param [in] activation_mode LoRaWAN activation mode: OTAA or ABP
 */
void lr1mac_core_set_activation_mode( lr1_stack_mac_t* lr1_mac_obj, lr1mac_activation_mode_t activation_mode );

/**
 * @brief  Get current device activation mode
 *
 * @param [in] lr1_mac_obj          The lr1mac object
 * @return lr1mac_activation_mode_t LoRaWAN activation mode: OTAA or ABP
 */
lr1mac_activation_mode_t lr1mac_core_get_activation_mode( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Return the Max payload length allowed for the next transmit
 *
 * @remark  DataRate + FOPTS + region  dependant
 * @remark  In any case if user sent a too long payload, the send method will answer by an error status
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_next_max_payload_length_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Return the DevAddr of the device
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_devaddr_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Return the next transmission power
 *
 * @param lr1_mac_obj
 * @return uint8_t
 */
uint8_t lr1mac_core_next_power_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Return the returns the next datarate
 *
 * @param lr1_mac_obj
 * @return uint8_t
 */
uint8_t lr1mac_core_next_dr_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Return the returns the next Tx Frequency
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_next_frequency_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Returns the current state of the MAC layer
 * @remark If the MAC is not in the idle state, the user cannot call any methods except the lr1mac_core_process() method
 * and the lr1mac_core_state_get() method
 *
 * @param lr1_mac_obj
 * @return lr1mac_states_t
 */
lr1mac_states_t lr1mac_core_state_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get number of stack reset
 *
 * @param lr1_mac_obj
 * @return uint16_t
 */
uint16_t lr1mac_core_nb_reset_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the current devnonce used to join
 *
 * @param lr1_mac_obj
 * @return uint16_t
 */
uint16_t lr1mac_core_devnonce_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Returns the join state
 *
 * @param lr1_mac_obj
 * @return join_status_t
 */
join_status_t lr1_mac_joined_status_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */

/**
 * @brief Send an uplink payload as soon as possible after the target time
 *
 * @param lr1_mac_obj
 * @param fPort           LoRaWAN Frame Port
 * @param fport_enabled   Frame send with a Fport or not (empty payload can be sent without Fport)
 * @param dataIn          Payload that will be send
 * @param sizeIn          Payload length in byte
 * @param PacketType      CONF_DATA_UP or CONF_DATA_DOWN
 * @param target_time_ms  RTC time to send the packet
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_payload_send( lr1_stack_mac_t* lr1_mac_obj, uint8_t fPort, bool fport_enabled,
                                           const uint8_t* dataIn, uint8_t sizeIn, uint8_t PacketType,
                                           uint32_t target_time_ms );

/**
 * @brief Send an uplink payload at the target time
 *
 * @param lr1_mac_obj
 * @param fPort           LoRaWAN Frame Port
 * @param fport_enabled   Frame send with a Fport or not (empty payload can be sent without Fport)
 * @param dataIn          Payload that will be send
 * @param sizeIn          Payload length in byte
 * @param PacketType      CONF_DATA_UP or CONF_DATA_DOWN
 * @param target_time_ms  RTC time to send the packet
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_payload_send_at_time( lr1_stack_mac_t* lr1_mac_obj, uint8_t fport, bool fport_enabled,
                                                   const uint8_t* data_in, uint8_t size_in, uint8_t packet_type,
                                                   uint32_t target_time_ms );

/**
 * @brief Send a device mac command request to the network
 *
 * @remark Only LINK_CHECK_REQ, DEVICE_TIME_REQ and PING_SLOT_INFO_REQ can be requested
 *
 * @param lr1_mac_obj
 * @param cid_req
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_send_stack_cid_req( lr1_stack_mac_t* lr1_mac_obj, cid_from_device_t cid_req );

/**
 * @brief Return the Radio planner object
 *
 * @param lr1_mac_obj
 * @return radio_planner_t*
 */
radio_planner_t* lr1mac_core_rp_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the Tx power set in stack
 *
 * @param lr1_mac_obj
 * @return int8_t
 */
int8_t lr1mac_core_tx_power_offset_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set the Tx power set in stack
 *
 * @param lr1_mac_obj
 * @param power_off
 */
void lr1mac_core_tx_power_offset_set( lr1_stack_mac_t* lr1_mac_obj, int8_t power_off );

/**
 * @brief Get the Rx window type that received the last Class A downlink
 *
 * @param lr1_mac_obj
 * @return receive_win_t
 */
receive_win_t lr1mac_core_rx_window_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the current uplink frame counter
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_fcnt_up_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the next time in second when a join could be requested
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_next_join_time_second_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the next free time for the duty cycle
 *
 * @remark  if > 0: the next slot availble, else the available time
 *
 * @param lr1_mac_obj
 * @return int32_t
 */
int32_t lr1mac_core_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the Rx network ACK bit status
 *
 * @param lr1_mac_obj
 * @return uint8_t
 */
uint8_t lr1mac_core_rx_ack_bit_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the last frame pending bit status
 *
 * @param lr1_mac_obj
 * @return uint8_t
 */
uint8_t lr1mac_core_rx_fpending_bit_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the current region (EU868, US915, ...)
 *
 * @param lr1_mac_obj
 * @return smtc_real_region_types_t
 */
smtc_real_region_types_t lr1mac_core_get_region( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set the region (EU868, US915, ...)
 *
 * @param lr1_mac_obj
 * @param region_type
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_set_region( lr1_stack_mac_t* lr1_mac_obj, smtc_real_region_types_t region_type );

/**
 * @brief Set the number of consecutive uplink packet without downlink to concidere the stack out of range
 *
 * @param lr1_mac_obj
 * @param no_rx_packet_reset_threshold
 */

void lr1mac_core_set_no_rx_packet_threshold( lr1_stack_mac_t* lr1_mac_obj, uint16_t no_rx_packet_reset_threshold );

/**
 * @brief Get the configured number of consecutive uplink packet count value without downlink
 *
 * @param lr1_mac_obj
 * @return uint16_t
 */
uint16_t lr1mac_core_get_no_rx_packet_threshold( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the number of consecutive uplink packet without downlink
 *
 * @param lr1_mac_obj
 * @return uint16_t
 */

/**
 * @brief Get the current value of internal adr ack cnt that is used for reset threshold trigger
 *
 * @remark The adr_ack_cnt values is not incremented during nb trans and will also fallow the backoff strategy in case
 * device is in network controlled mode. The value is reset when a downlink happened
 *
 * @param [in] lr1_mac_obj
 * @return uint16_t
 */
uint16_t lr1mac_core_get_current_adr_ack_cnt( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the number of consecutive uplink packet without downlink in mobile adr mode
 *
 * @param lr1_mac_obj
 * @return uint16_t
 */
uint16_t lr1mac_core_get_current_no_rx_packet_in_mobile_mode( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Reset the number of consecutive uplink packet without downlink to concider the stack out of range in mobile
 * mode
 *
 * @param lr1_mac_obj
 */
void lr1mac_core_reset_no_rx_packet_in_mobile_mode_cnt( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the current value of internal "tx without rx" counter
 *
 * @remark This counter is incremented at each tx done (even during nb trans) and reset when a downlink happened
 *
 * @return uint16_t
 */
uint16_t lr1mac_core_get_current_no_rx_packet_cnt( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief stack receive a link adr request
 *
 * @remark reset the flag automatically each time the upper layer call this function
 *
 * @param lr1_mac_obj
 * @return true
 * @return false
 */
bool lr1mac_core_available_link_adr_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set the certification mode
 *
 * @remark Status is saved in flash
 *
 * @param [IN]  none
 * @param [OUT] bool
 */
void lr1mac_core_certification_set( lr1_stack_mac_t* lr1_mac_obj, uint8_t enable );

/**
 * @brief Set the certification mode status
 *
 * @param lr1_mac_obj
 * @return uint8_t
 */
uint8_t lr1mac_core_certification_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the LoRaWAN version
 *
 * @param lr1_mac_obj
 * @return lr1mac_version_t
 */
lr1mac_version_t lr1mac_core_get_lorawan_version( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Convert current rtc time to gps time
 *
 * @param lr1_mac_obj
 * @param rtc_ms
 * @param [out] seconds_since_epoch
 * @param [out] fractional_second
 * @return true               Network Time is valid
 * @return false              Network Time is no more valid
 */
bool lr1mac_core_convert_rtc_to_gps_epoch_time( lr1_stack_mac_t* lr1_mac_obj, uint32_t rtc_ms,
                                                uint32_t* seconds_since_epoch, uint32_t* fractional_second );

/**
 * @brief Get if the network time is still valid or not
 *
 * @param lr1_mac_obj
 * @return true
 * @return false
 */
bool lr1mac_core_is_time_valid( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_get_timestamp_last_device_time_ans_s( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the left delais before to concider device time no more valid
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1mac_core_get_time_left_connection_lost( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set delay in seconds to concider time no more valid if no time sync received
 *
 * @param lr1_mac_obj
 * @param delay_s
 * @return status_lorawan_t
 */
status_lorawan_t lr1_mac_core_set_device_time_invalid_delay_s( lr1_stack_mac_t* lr1_mac_obj, uint32_t delay_s );

/**
 * @brief Get delay in seconds to concider time no more valid if no time sync received
 *
 * @param lr1_mac_obj
 * @return uint32_t
 */
uint32_t lr1_mac_core_get_device_time_invalid_delay_s( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set the Ping Slot Periodicity
 *
 * @param lr1_mac_obj
 * @param ping_slot_periodicity
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_set_ping_slot_periodicity( lr1_stack_mac_t* lr1_mac_obj, uint8_t ping_slot_periodicity );

/**
 * @brief Get the Ping Slot Periodicity
 *
 * @param lr1_mac_obj
 * @return uint8_t
 */
uint8_t lr1mac_core_get_ping_slot_periodicity( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the status of the class B bit
 *
 * @param lr1_mac_obj
 * @return true
 * @return false
 */
bool lr1mac_core_get_class_b_status( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get the Margin and Gateway count returned by LinkCheckAns mac command when answered
 *
 * @param lr1_mac_obj
 * @param [out] margin
 * @param [out] gw_cnt
 */
status_lorawan_t lr1_mac_core_get_link_check_ans( lr1_stack_mac_t* lr1_mac_obj, uint8_t* margin, uint8_t* gw_cnt );

/**
 * @brief Get Device Time Request
 *
 * @param lr1_mac_obj
 * @return status_lorawan_t
 */
status_lorawan_t lr1_mac_core_get_device_time_req_status( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get Ping Slot Info Request status
 *
 * @param lr1_mac_obj
 * @return status_lorawan_t
 */
status_lorawan_t lr1_mac_core_get_ping_slot_info_req_status( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Get status of push network downlink (mac commands, beacon, ..) to the user
 *
 * @param lr1_mac_obj
 * @return true
 * @return false
 */
bool lr1mac_core_get_status_push_network_downlink_to_user( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief Set status of push network downlink (mac commands, beacon, ..) to the user
 *
 * @param lr1_mac_obj
 * @param enabled
 */
void lr1mac_core_set_status_push_network_downlink_to_user( lr1_stack_mac_t* lr1_mac_obj, bool enabled );

/**
 * @brief Set the ADR ACK limit and ADR ACK delay regarding the ADR fallback in case no downlink are received
 *
 * @param lr1_mac_obj
 * @param adr_ack_limit   Accepted value: ( adr_ack_limit > 1 ) && ( adr_ack_limit < 128 )
 * @param adr_ack_delay   Accepted value: ( adr_ack_delay > 1 ) && ( adr_ack_delay < 128 )
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_set_adr_ack_limit_delay( lr1_stack_mac_t* lr1_mac_obj, uint8_t adr_ack_limit,
                                                      uint8_t adr_ack_delay );

/**
 * @brief  Get the ADR ACK limit and ADR ACK delay configured regarding the ADR fallback in case no downlink are
 * received
 *
 * @param lr1_mac_obj
 * @param adr_ack_limit
 * @param adr_ack_delay
 */
void lr1mac_core_get_adr_ack_limit_delay( lr1_stack_mac_t* lr1_mac_obj, uint8_t* adr_ack_limit,
                                          uint8_t* adr_ack_delay );

#endif
