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
    LORAWAN_MC_RC_ERROR_NOT_INIT,
} lorawan_multicast_rc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Init the LoRaWAN stack
 */
void lorawan_api_init( radio_planner_t* rp );

/*!
 * \brief
 */
void lorawan_rp_callback_api( radio_planner_t* rp );

/*!
 * \brief Set the LoRaWAN regional parameters
 * \param [out] smtc_real_region_types_t* Region
 */
smtc_real_region_types_t lorawan_api_get_region( void );

/*!
 * \brief Set the LoRaWAN regional parameters
 * \param [in] smtc_real_region_types_t Region
 */
status_lorawan_t lorawan_api_set_region( smtc_real_region_types_t region_type );

/*!
 * \brief Sends an uplink when it's possible
 * \param [in] uint8_t           fPort          Uplink Fport
 * \param [in] bool              fport_enabled  Fport present or not
 * \param [in] const uint8_t*    dataInFport    User Payload
 * \param [in] const uint8_t     sizeIn         User Payload Size
 * \param [in] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,
 * \param [in] uint32_t          TargetTimeMs   RTC time when the packet must be sent
 * \param [out] lr1mac_states_t         Current state of the LoraWan stack :
 * \param                                            => return LWPSATE_SEND if all is ok
 * \param                                            => return Error in case of payload too long
 * \param                                            => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_payload_send( uint8_t fPort, bool fport_enabled, const uint8_t* dataIn,
                                          const uint8_t sizeIn, uint8_t PacketType, uint32_t TargetTimeMs );

/*!
 * \brief Sends an uplink at time
 * \param [in] uint8_t           fPort          Uplink Fport
 * \param [in] bool              fport_enabled  Fport present or not
 * \param [in] const uint8_t*    dataInFport    User Payload
 * \param [in] const uint8_t     sizeIn         User Payload Size
 * \param [in] const uint8_t     PacketType     User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,
 * \param [in] uint32_t          TargetTimeMs   RTC time when the packet must be sent
 * \param [out] lr1mac_states_t         Current state of the LoraWan stack :
 * \param                                            => return LWPSATE_SEND if all is ok
 * \param                                            => return Error in case of payload too long
 * \param                                            => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_payload_send_at_time( uint8_t fPort, bool fport_enabled, const uint8_t* dataIn,
                                                  const uint8_t sizeIn, uint8_t PacketType, uint32_t TargetTimeMs );

/**
 * @brief
 *
 * @param  [in] cid_req          Command ID request by the User LINK_CHECK_REQ or DEVICE_TIME_REQ
 * @return lr1mac_states_t Current state of the LoraWan stack :
 * \param                                            => return LWPSATE_SEND if all is ok
 * \param                                            => return Error in case of payload too long
 * \param                                            => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_send_stack_cid_req( cid_from_device_t cid_req );

/*!
 * \brief  Receive Applicative Downlink
 * \param [in] uint8_t*          UserRxFport            Downlinklink Fport
 * \param [in] uint8_t*          UserRxPayload          Applicative Downlink Payload
 * \param [in] uint8_t*          UserRxPayloadSize      Applicative Downlink Payload Size
 * \param [in] const uint8_t     PacketType             User Packet Type : UNCONF_DATA_UP, CONF_DATA_UP,

 * \param [out] eStatusLoRaWan   Return an error if No Packet available.
 */
status_lorawan_t lorawan_api_payload_receive( uint8_t* UserRxFport, uint8_t* UserRxPayload,
                                              uint8_t* UserRxPayloadSize );

/*!
 * \brief to Send a Join request
 * \param [] None
 * \param [out] lr1mac_states_t         Current state of the LoraWan stack :
 *                                                 => return LWPSATE_SEND if all is ok
 *                                                 => return Error In case of the Lorawan stack previous state is not
 *                                                      equal to idle
 */
lr1mac_states_t lorawan_api_join( uint32_t target_time_ms );

/*!
 * \brief Returns the join state
 * \param [] None
 * \param [out] Returns the join state         NOT_JOINED: the device is joined to a network
 *                                             JOINED: the device is not connected
 *                                             Always returns JOINED for ABP devices
 */
join_status_t lorawan_api_isjoined( void );

/*!
 * \brief Rreset the join status to NotJoined
 * \param [] None
 * \param [out] None
 */
void lorawan_api_join_status_clear( void );

/*!
 * \brief SetDataRateStrategy of the devices
 * \remark Refered to the dedicated chapter in Wiki page for detailed explanation about
 *         implemented data rate choice (distribution data rate).
 * \remark The current implementation support 4 different dataRate Strategy :
 *            STATIC_ADR_MODE                   for static Devices with ADR managed by the Network
 *            MOBILE_LONGRANGE_DR_DISTRIBUTION  for Mobile Devices with strong Long range requirement
 *            MOBILE_LOWPER_DR_DISTRIBUTION     for Mobile Devices with strong Low power requirement
 *            JOIN_DR_DISTRIBUTION              Dedicated for Join requests
 *
 * \param [in]  dr_strategy_t                   DataRate Mode (describe above)
 * \param [out] None
 */
status_lorawan_t lorawan_api_dr_strategy_set( dr_strategy_t adrModeSelect );
dr_strategy_t    lorawan_api_dr_strategy_get( void );
void             lorawan_api_dr_custom_set( uint32_t DataRateCustom );

/*!
 * \brief   Runs the MAC layer state machine.
 *          Must be called periodically by the application. Not timing critical. Can be interrupted.
 * \remark  Not timing critical. Can be interrupted.
 *
 * \param [in]  AvailableRxPacket *             Return if an applicative packet is available
 * \param [out] lr1mac_states_t                 return the lorawan state machine state
 */
lr1mac_states_t lorawan_api_process( user_rx_packet_type_t* AvailableRxPacket );

/*!
 * \brief   Return the state of the Radio
 * \param [in]  none
 * \param [out] return the state of the radio (Not yet finalized will be replace by an enum)
 */
uint8_t lorawan_api_GetRadioState( void );

/*!
 * \brief   Reload the LoraWAN context saved in the flash
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_context_load( void );
/*!
 * \brief   Save The LoraWAN context in the flash
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_context_save( void );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [in]  none
 * \param [out] Int 16 last snr
 */
int16_t lorawan_api_last_snr_get( void );
/*!
 * \brief   Get the snr of the last user receive packet
 * \param [in]  none
 * \param [out] Int 16 last snr
 */
int16_t lorawan_api_last_rssi_get( void );

/*!
 * \brief   Reload the factory Config in the LoraWAN Stack
 * \param [in]  none
 * \param [out] none
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

/*!
 * \brief   Return the Max payload length allowed for the next transmit
 * \remark  DataRate + FOPTS + region  dependant ( )
 * \remark  In any case if user set a too long payload, the send method will answer by an error status
 * \param [in]  none
 * \param [out] Return max payload length for next Transmission
 */
uint32_t lorawan_api_next_max_payload_length_get( void );

/*!
 * \brief   Call this function to set the loraWan join variable in NOT_JOINED state
 * \param [in]  none
 * \param [out] none
 */
void lorawan_api_new_join( void );

/*!
 * \brief   Return the DevAddr of the device
 * \param [in]  none
 * \param [out] return DevAddr
 */
uint32_t lorawan_api_devaddr_get( void );

/**
 * @brief Get the DevEUI of the device
 *
 * @param [out] DevEui The Device EUI
 */
void lorawan_api_get_deveui( uint8_t* DevEui );

/**
 * @brief Set the DevEUI of the device
 *
 * @param [in] DevEui The Device EUI
 */
void lorawan_api_set_deveui( const uint8_t* DevEui );

/**
 * @brief Set the AppKey of the device
 *
 * @param [in] AppKey The LoRaWan 1.0.x application Key
 */
void lorawan_api_set_appkey( const uint8_t* AppKey );

/**
 * @brief Get the join_eui of the device
 *
 * @param [out] join_eui The current Join EUI
 */
void lorawan_api_get_joineui( uint8_t* join_eui );

/**
 * @brief Set the join_eui of the device
 *
 * @param [in] join_eui The Join EUI
 */
void lorawan_api_set_joineui( const uint8_t* join_eui );

/*!
 * \brief   Return the next transmission power
 * \remark
 * \param [in]  none
 * \param [out] return the next transmission power
 */
uint8_t lorawan_api_next_power_get( void );

/*!
 * \brief   Return the returns the next data rate
 * \remark
 * \param [in]  none
 * \param [out] return the next transmission power
 */
uint8_t lorawan_api_next_dr_get( void );

/*!
 * \brief   Return the returns the next Tx Frequency
 * \remark
 * \param [in]  none
 * \param [out] return the next transmission power
 */

uint32_t lorawan_api_next_frequency_get( void );

/*!
 * \brief   Return the returns the max data rate of all enabled channels
 * \remark
 * \param [in]  none
 * \param [out] return the max data rate
 */
uint8_t lorawan_api_max_tx_dr_get( void );
/*!
 * \brief   Return the returns the min data rate of all enabled channels
 * \remark
 * \param [in]  none
 * \param [out] return the min data rate
 */
uint8_t lorawan_api_min_tx_dr_get( void );

/*!
 * \brief   Return the returns the current data rate mask of all enabled channels
 * \remark
 * \param [IN]  none
 * \param [OUT] return the mask data rate
 */
uint16_t lorawan_api_mask_tx_dr_channel_up_dwell_time_check( void );

/*!
 * \brief   returns the current state of the MAC layer.
 * \remark  If the MAC is not in the idle state, the user cannot call any methods except the LoraWanProcess()
 *          method and the GetLorawanProcessState() method
 * \param [in]  none
 * \param [out] return the next transmission power
 */
lr1mac_states_t lorawan_api_state_get( void );
/*!
 * \brief   returns the number of reset
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint16_t lorawan_api_nb_reset_get( void );
/*!
 * \brief   returns the last devnonce
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint16_t lorawan_api_devnonce_get( void );
/*!
 * \brief   returns the Rx window used by the downlink
 * \remark
 * \param [in]  none
 * \param [out] return
 */
receive_win_t lorawan_api_rx_window_get( void );
/*!
 * \brief   returns the min time to perform a new join request
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint32_t lorawan_api_next_join_time_second_get( void );
/*!
 * \brief   when > 0, returns the min time to perform a new uplink request
 * \remark
 * \param [in]  none
 * \param [out] return
 */
int32_t lorawan_api_next_free_duty_cycle_ms_get( void );
/*!
 * \brief   Enable / disable the dutycycle
 * \remark
 * \param [in]  smtc_dtc_enablement_type_t enable
 * \param [out] status_lorawan_t
 */
status_lorawan_t lorawan_api_duty_cycle_enable_set( smtc_dtc_enablement_type_t enable );
/**
 * @brief Get status Enable / disable dutycycle
 *
 * @return smtc_dtc_enablement_type_t
 */
smtc_dtc_enablement_type_t lorawan_api_duty_cycle_enable_get( void );
/*!
 * \brief   return the last uplink frame counter
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint32_t lorawan_api_fcnt_up_get( void );
/*!
 * \brief   Get the LoRaWAN hook ID in radio planner
 * \remark
 * \param [in]  none
 * \param [out] return
 */
uint8_t lorawan_api_rp_hook_id_get( void );

/*!
 * \brief   Enable/disable class C
 * \remark
 * \param [in]  uint8_t
 * \param [out] none
 */
void lorawan_api_class_c_enabled( bool enable );
/*!
 * \brief   Start class C
 * \remark
 * \param [in]  uint8_t
 * \param [out] none
 */
void lorawan_api_class_c_start( void );
/*!
 * \brief   Stop class C
 * \remark
 * \param [in]  uint8_t
 * \param [out] none
 */
void lorawan_api_class_c_stop( void );
/*!
 * \brief   Get the downlink frame ACK bit state
 * \remark
 * \param [in]  none
 * \param [out] return
 */

/**
 * @brief Configure a multicast group
 *
 * @param [in] mc_group_id      The multicast group id that will be configured (0 to 3)
 * @param [in] mc_group_address The multicast group addr
 * @param [in] mc_group_key     The Multicast key associated
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_set_group_config( uint8_t mc_group_id, uint32_t mc_group_address,
                                                               const uint8_t mc_ntw_skey[LORAWAN_KEY_SIZE],
                                                               const uint8_t mc_app_skey[LORAWAN_KEY_SIZE] );

/**
 * @brief Get a multicast group configuration
 *
 * @param [in]  mc_group_id      The multicast group id
 * @param [out] mc_group_address The current multicast group addr for chosen group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_group_config( uint8_t mc_group_id, uint32_t* mc_group_address );

/**
 * @brief Start a multicast on a previously configured group id
 *
 * @param [in] mc_group_id  The multicast group id
 * @param [in] freq         Rx frequency
 * @param [in] dr           Rx Datarate
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_start_session( uint8_t mc_group_id, uint32_t freq, uint8_t dr );

/**
 * @brief Get the status of a multicast session
 *
 * @param [in]  mc_group_id         The multicast group id
 * @param [out] is_session_started  Boolean to indicate if session is active
 * @param [out] freq                Rx frequency
 * @param [out] dr                  Rx Datarate
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_get_session_status( uint8_t mc_group_id, bool* is_session_started,
                                                                 uint32_t* freq, uint8_t* dr );

/**
 * @brief Stop the chosen multicast session
 *
 * @param [in] mc_group_id The multicast group id
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_stop_session( uint8_t mc_group_id );

/**
 * @brief Stop all multicast sessions
 *
 * @return lorawan_multicast_rc_t
 */
lorawan_multicast_rc_t lorawan_api_multicast_stop_all_sessions( void );

/**
 * @brief
 *
 * @return uint8_t
 */
uint8_t lorawan_api_rx_ack_bit_get( void );

/*!
 * \brief   Set the number uplink without downlink before reset stack
 * \remark
 * \param  [in]  uint16_t no_rx_packet_count
 * \retval [out] status_lorawan_t
 */
status_lorawan_t lorawan_api_no_rx_packet_count_config_set( uint16_t no_rx_packet_count );

/*!
 * \brief   Get the configured number of uplink without downlink before reset stack
 * \remark
 * \retval [out] uint16_t
 */
uint16_t lorawan_api_no_rx_packet_count_config_get( void );

/**
 * @brief  Get the current number of uplink without downlink before reset stack
 *
 * @return uint16_t
 */
uint16_t lorawan_api_no_rx_packet_count_current_get( void );

/*!
 * \brief   Get the number uplink without downlink in mobile mode
 * \remark
 * \retval [out] uint16_t
 */
uint16_t lorawan_api_no_rx_packet_count_in_mobile_mode_get( void );

/*!
 * \brief   Set the current counter of number uplink without downlink in mobile mode
 * \remark
 * \retval [in] uint32_t
 */
void lorawan_api_no_rx_packet_count_in_mobile_mode_set( uint16_t no_rx_packet_count );

/*!
 * \brief   Set the status of the Modem LoRaWAN certification
 * \remark  To authorized LoRaWAN certification in modem
 * \param [in]  uint8_t true/false
 * \param [out] return
 */
void lorawan_api_modem_certification_set( uint8_t enable );

/*!
 * \brief   Get the status of the LoRaWAN certification
 * \remark  Is enabled by the Test Tool
 * \param [in]  none
 * \param [out] return uint8_t
 */
bool lorawan_api_certification_is_enabled( void );

/*!
 * \brief   Get the status of the Modem LoRaWAN certification
 * \remark  Is certification is authorized in modem
 * \param [in]  none
 * \param [out] return uint8_t
 */
uint8_t lorawan_api_modem_certification_is_enabled( void );

/**
 * @brief call LoRaWAN Certification state machine
 *
 * @param rx_buffer
 * @param rx_buffer_length
 * @param tx_buffer
 * @param tx_buffer_length
 * @param tx_fport
 * @return lorawan_certification_parser_ret_t
 */
lorawan_certification_parser_ret_t lorawan_api_certification( uint8_t* rx_buffer, uint8_t rx_buffer_length,
                                                              uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                              uint8_t* tx_fport );

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t lorawan_api_certification_get_ul_periodicity( void );

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool lorawan_api_certification_get_frame_type( void );

/**
 * @brief Get the CW configuration requested by the TCL
 *
 * @param timeout_s
 * @param frequency
 * @param tx_power
 */
void lorawan_api_certification_get_cw_config( uint16_t* timeout_s, uint32_t* frequency, int8_t* tx_power );

/**
 * @brief Get if the CW was requested by the TCL
 *
 * @return true
 * @return false
 */
bool lorawan_api_certification_is_cw_running( void );

/**
 * @brief Set CW as stopped
 *
 */
void lorawan_api_certification_cw_set_as_stopped( void );

/*!
 * \brief   Api to choose the lorawan key in case of a crc error
 * \remark  a crc error is present at the first start
 * \param [in]  device key
 * \param [out] none
 */
void lorawan_api_set_default_key( uint8_t default_app_key[16], uint8_t default_dev_eui[8],
                                  uint8_t default_join_eui[8] );

/*!
 * \brief  return true if stack receive a link adr request
 * \remark reset the flag automatically each time the upper layer call this function
 * \param [in]  void
 * \param [out] bool
 */
bool lorawan_api_available_link_adr_get( void );

/*!
 * \brief  return the current stack obj
 * \remark
 * \param [in]  void
 * \param [out] lr1_stack_mac_t*
 */
lr1_stack_mac_t* lorawan_api_stack_mac_get( void );

/**
 * @brief
 *
 * @return fifo_ctrl_t*
 */
fifo_ctrl_t* lorawan_api_get_fifo_obj( void );

/**
 * @brief set network type
 *
 * @param network_type true : public, false : private
 */
void lorawan_api_set_network_type( bool network_type );
/**
 * @brief  get network type
 *
 * @return true public network
 * @return false private network
 */
bool lorawan_api_get_network_type( void );

/*!
 * @brief lr1_stack_nb_trans_get
 * @remark
 * @return nb_trans
 */
uint8_t lorawan_api_nb_trans_get( void );

/*!
 * @brief lorawan_api_nb_trans_set
 * @remark
 * @param   nb_trans have to be smaller than 16
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_nb_trans_set( uint8_t nb_trans );

/**
 * @brief get the current crystal error
 *
 */
uint32_t lorawan_api_get_crystal_error( void );

/**
 * @brief set the crystal error
 *
 * @param crystal_error
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
 * @brief Get Network Time
 *
 * @param [in] rtc_ms
 * @param [out] seconds_since_epoch
 * @param [out] fractional_second
 */
void lorawan_api_convert_rtc_to_gps_epoch_time( uint32_t rtc_ms, uint32_t* seconds_since_epoch,
                                                uint32_t* fractional_second );

/**
 * @brief Get if Network time is still valid or not
 *
 * @return true
 * @return false
 */
bool lorawan_api_is_time_valid( void );

/**
 * @brief Configure the callback for the stack when will received the network time sync
 *
 * @param device_time_callback
 */
void lorawan_api_set_device_time_callback( void ( *device_time_callback )( void* context, uint32_t rx_timestamp_s ),
                                           void* context, uint32_t rx_timestamp_s );

/**
 * @brief Set delay in seconds to concider time no more valid if no time sync received
 *
 * @param delay_s
 * @return status_lorawan_t
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
 * @param [out] margin
 * @param [out] gw_cnt
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_get_link_check_ans( uint8_t* margin, uint8_t* gw_cnt );

/**
 * @brief Get Device Time Request status
 *
 * @return status_lorawan_t
 */
status_lorawan_t lorawan_api_get_device_time_req_status( void );

#ifdef __cplusplus
}
#endif

#endif  // __LORAWAN_API_H__

/* --- EOF ------------------------------------------------------------------ */
