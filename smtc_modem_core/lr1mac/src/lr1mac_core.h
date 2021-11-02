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

/*!
 * \class LoraWanObject
 * \brief An absolutely minimalistic LoRaWAN Class A stack implementation .
 * \remark  In future implementation the constructor will contain :
 *            APPSKey, NWSKey, DevAdrr mandatory for ABP devices
 *            DevEui, JoinEUI, APPKey mandatory for OTAA devices
 *            OTAA or ABP Flag.
 *         In future implementation A Radio objet will be also a parameter of this class.
 */
void lr1mac_core_init( lr1_stack_mac_t* lr1_mac_obj, smtc_lbt_t* lbt_obj, smtc_dtc_t* dtc_obj, radio_planner_t* rp,
                       lr1mac_activation_mode_t otaa_abp_conf, smtc_real_region_types_t smtc_real_region_types,
                       void ( *push_callback )( void* push_context ), void*             push_context );

/*!
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

/*!
     * \brief  Receive Applicative Downlink
     * \param [IN] uint8_t*          UserRxFport          Downlinklink Fport
     * \param [IN] uint8_t*          UserRxPayload        Applicative Downlink Payload
     * \param [IN] uint8_t*          UserRxPayloadSize    Applicative Downlink Payload Size
     * \param [IN] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

     * \param [OUT] status_lorawan_t   Return an error if No Packet available.
     */
status_lorawan_t lr1mac_core_payload_receive( lr1_stack_mac_t* lr1_mac_obj, uint8_t* UserRxFport,
                                              uint8_t* UserRxPayload, uint8_t* UserRxPayloadSize );

/*!
 * \brief to Send a Join request
 * \param [] None
 * \param [OUT] lr1mac_states_t         Current state of the LoraWan stack :
 *                                                 => return LWPSATE_SEND if all is ok
 *                                                 => return Error In case of the Lorawan stack previous state is not
 * equal to idle
 */
lr1mac_states_t lr1mac_core_join( lr1_stack_mac_t* lr1_mac_obj, uint32_t target_time_ms );

/*!
 * \brief Returns the join state
 * \param [] None
 * \param [OUT] Returns the join state         NOT_JOINED: the device is joined to a network
 *                                             JOINED: the device is not connected
 *                                             Always returns JOINED for ABP devices
 */
join_status_t lr1mac_core_is_joined( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief Reset the join status to NotJoined
 * \param [] None
 * \param [OUT] None
 */
void lr1mac_core_join_status_clear( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief abort loRawan task
 * \param [] None
 * \param [OUT] None
 */
void lr1mac_core_abort( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief SetDataRateStrategy of the devices
 * \remark Refereed to the dedicated chapter in Wiki page for detailed explanation about
 *         implemented data rate choice (distribution data rate).
 * \remark The current implementation support 4 different dataRate Strategy :
 *            STATIC_ADR_MODE                  for static Devices with ADR managed by the Network
 *            MOBILE_LONGRANGE_DR_DISTRIBUTION for Mobile Devices with strong Long range requirement
 *            MOBILE_LOWPER_DR_DISTRIBUTION    for Mobile Devices with strong Low power requirement
 *            JOIN_DR_DISTRIBUTION             Dedicated for Join requests
 *
 * \param [IN]  dr_strategy_t              DataRate Mode (describe above)
 * \param [OUT] None
 */
status_lorawan_t lr1mac_core_dr_strategy_set( lr1_stack_mac_t* lr1_mac_obj, dr_strategy_t adrModeSelect );
dr_strategy_t    lr1mac_core_dr_strategy_get( lr1_stack_mac_t* lr1_mac_obj );
void             lr1mac_core_dr_custom_set( lr1_stack_mac_t* lr1_mac_obj, uint32_t DataRateCustom );

/*!
 * \brief   Runs the MAC layer state machine.
 *          Must be called periodically by the application. Not timing critical. Can be interrupted.
 * \remark  Not timing critical. Can be interrupted.
 *
 * \param [IN]  AvailableRxPacket *            Return if an applicative packet is available
 * \param [OUT] lr1mac_states_t        return the lorawan state machine state
 */
lr1mac_states_t lr1mac_core_process( lr1_stack_mac_t* lr1_mac_obj, user_rx_packet_type_t* AvailableRxPacket );

/*!
 * \brief   Return the state of the Radio
 * \param [IN]  none
 * \param [OUT] return the state of the radio (Not yet finalized will be replace by an enum)
 */
uint8_t lr1mac_core_radio_state_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Reload the LoraWAN context saved in the flash
 * \param [IN]  none
 * \param [OUT] none
 */
status_lorawan_t lr1mac_core_context_load( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief   Save The LoraWAN context  in the flash
 * \param [IN]  none
 * \param [OUT] none
 */
void lr1mac_core_context_save( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [IN]  none
 * \param [OUT] Int 16 last snr
 */
int16_t lr1mac_core_last_snr_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [IN]  none
 * \param [OUT] Int 16 last snr
 */
int16_t lr1mac_core_last_rssi_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Reload the factory Config in the LoraWAN Stack
 * \param [IN]  none
 * \param [OUT] none
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
 * @brief  Get current deveice activation mode
 *
 * @param [in] lr1_mac_obj          The lr1mac object
 * @return lr1mac_activation_mode_t LoRaWAN activation mode: OTAA or ABP
 */
lr1mac_activation_mode_t lr1mac_core_get_activation_mode( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the Max payload length allowed for the next transmit
 * \remark  DataRate + FOPTS + region  dependant  (lr1_stack_mac_t  * lr1_mac_obj ,)
 * \remark  In any case if user set a too long payload, the send method will answer by an error status
 * \param [IN]  none
 * \param [OUT] Return max payload length for next Transmission
 */
uint32_t lr1mac_core_next_max_payload_length_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Call this function to set the loraWan join variable in NOT_JOINED state
 * \param [IN]  none
 * \param [OUT] none
 */
void lr1mac_core_new_join( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the DevAddr of the device
 * \param [IN]  none
 * \param [OUT] return DevAddr
 */
uint32_t lr1mac_core_devaddr_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the next transmission power
 * \remark
 * \param [IN]  none
 * \param [OUT] return the next transmission power
 */
uint8_t lr1mac_core_next_power_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the returns the next data rate
 * \remark
 * \param [IN]  none
 * \param [OUT] return the next transmission power
 */
uint8_t lr1mac_core_next_dr_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the returns the next Tx Frequency
 * \remark
 * \param [IN]  none
 * \param [OUT] return the next transmission power
 */

uint32_t lr1mac_core_next_frequency_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the returns the max data rate of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the max data rate
 */
uint8_t lr1mac_core_max_tx_dr_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the returns the current data rate mask of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the mask data rate
 */
uint16_t lr1mac_core_mask_tx_dr_channel_up_dwell_time_check( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   Return the returns the min data rate of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the min data rate
 */

uint8_t lr1mac_core_min_tx_dr_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief   returns the current state of the MAC layer.
 * \remark  If the MAC is not in the idle state, the user cannot call any methods except the LoraWanProcess() method and
 * the GetLorawanProcessState() method \param [IN]  none \param [OUT] return the next transmission power
 */

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1mac_states_t lr1mac_core_state_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint16_t lr1mac_core_nb_reset_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint16_t lr1mac_core_devnonce_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
join_status_t lr1_mac_joined_status_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1mac_states_t lr1mac_core_payload_send( lr1_stack_mac_t* lr1_mac_obj, uint8_t fPort, bool fport_enabled,
                                          const uint8_t* dataIn, const uint8_t sizeIn, uint8_t PacketType,
                                          uint32_t target_time_ms );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
lr1mac_states_t lr1mac_core_payload_send_at_time( lr1_stack_mac_t* lr1_mac_obj, uint8_t fport, bool fport_enabled,
                                                  const uint8_t* data_in, const uint8_t size_in, uint8_t packet_type,
                                                  uint32_t target_time_ms );

/**
 * @brief
 *
 * @param [IN] lr1_mac_obj
 * @param [IN] cid_req
 * @return lr1mac_states_t
 */
lr1mac_states_t lr1mac_core_send_stack_cid_req( lr1_stack_mac_t* lr1_mac_obj, cid_from_device_t cid_req );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
radio_planner_t* lr1mac_core_rp_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int8_t lr1mac_core_tx_power_offset_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
void lr1mac_core_tx_power_offset_set( lr1_stack_mac_t* lr1_mac_obj, int8_t power_off );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
receive_win_t lr1mac_core_rx_window_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t lr1mac_core_fcnt_up_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint32_t lr1mac_core_next_join_time_second_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
int32_t lr1mac_core_next_free_duty_cycle_ms_get( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] uint8_t
 */
uint8_t lr1mac_core_duty_cycle_enable_set( lr1_stack_mac_t* lr1_mac_obj, smtc_dtc_enablement_type_t enable );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] uint8_t
 */
smtc_dtc_enablement_type_t lr1mac_core_duty_cycle_enable_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
uint8_t lr1mac_core_rx_ack_bit_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1mac_core_is_supported_region( lr1_stack_mac_t* lr1_mac_obj, smtc_real_region_types_t region_type );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
smtc_real_region_types_t lr1mac_core_get_region( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] return
 */
status_lorawan_t lr1mac_core_set_region( lr1_stack_mac_t* lr1_mac_obj, smtc_real_region_types_t region_type );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] status_lorawan_t
 */
status_lorawan_t lr1mac_core_set_no_rx_packet_count_config( lr1_stack_mac_t* lr1_mac_obj, uint16_t no_rx_packet_count );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] uint32_t
 */
uint16_t lr1mac_core_get_no_rx_packet_count_config( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief
 *
 * @param lr1_mac_obj
 * @return uint16_t
 */
uint16_t lr1mac_core_get_no_rx_packet_count_current( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] uint32_t
 */
uint16_t lr1mac_core_get_no_rx_packet_count_in_mobile_mode( lr1_stack_mac_t* lr1_mac_obj );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] void
 */
void lr1mac_core_set_no_rx_packet_count_in_mobile_mode( lr1_stack_mac_t* lr1_mac_obj, uint16_t no_rx_packet_count );
/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] bool
 */
bool lr1mac_core_available_link_adr_get( lr1_stack_mac_t* lr1_mac_obj );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] bool
 */
void lr1mac_core_certification_set( lr1_stack_mac_t* lr1_mac_obj, uint8_t enable );

/*!
 * \brief
 * \remark
 * \param [IN]  none
 * \param [OUT] bool
 */
uint8_t lr1mac_core_certification_get( lr1_stack_mac_t* lr1_mac_obj );

/**
 * @brief
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
 * @param seconds_since_epoch
 * @param fractional_second
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
 * @param device_time_callback
 */
void lr1mac_core_set_device_time_callback( lr1_stack_mac_t* lr1_mac_obj,
                                           void ( *device_time_callback )( void* context, uint32_t rx_timestamp_s ),
                                           void* context, uint32_t rx_timestamp_s );

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
 * @brief
 *
 * @param lr1_mac_obj
 * @param ping_slot_periodicity
 * @return status_lorawan_t
 */
status_lorawan_t lr1mac_core_set_ping_slot_periodicity( lr1_stack_mac_t* lr1_mac_obj, uint8_t ping_slot_periodicity );

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

#endif
