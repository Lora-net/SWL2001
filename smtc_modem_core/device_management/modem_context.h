/*!
 * \file      modem_context.h
 *
 * \brief     share functions + context of the soft modem .
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

#ifndef __SOFT_MODEM_CONTEXT_H__
#define __SOFT_MODEM_CONTEXT_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "smtc_modem_api.h"
#include "smtc_modem_hal.h"
#include "device_management_defs.h"
#include "lr1mac_defs.h"
#include "alc_sync.h"
#include "radio_planner.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define DEFAULT_DM_PORT 199
#define DEFAULT_FRAG_PORT 201
#define DEFAULT_DM_REPORTING_INTERVAL 0x81  // 1h
#define DEFAULT_DM_REPORTING_FIELDS 0x7B    // status, charge, temp, signal, uptime, rxtime
#define DEFAULT_DM_MUTE_DAY 0
#define DEFAULT_ADR_MOBILE_MODE_TIMEOUT 0  // desactivated by default

#define UPLOAD_SID 0

#define DM_STATUS_NOW_MIN_TIME 2
#define DM_STATUS_NOW_MAX_TIME 5

#define POWER_CONFIG_LUT_SIZE 6

#define MODEM_NUMBER_OF_EVENTS 0x19  // number of possible events in modem

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct modem_context_class_b_d2d_s
{
    bool    tx_done;
    uint8_t nb_trans_not_send;
    uint8_t mc_grp_id;
} modem_context_class_b_d2d_t;

typedef struct power_config_e
{
    int8_t  expected_power;
    int8_t  configured_power;
    uint8_t pa_param1;
    uint8_t pa_param2;
    uint8_t pa_ramp_time;
} modem_power_config_t;

typedef void ( *func_callback )( void );

typedef enum charge_counter_value_e
{
    CHARGE_COUNTER_MODEM       = 0,
    CHARGE_COUNTER_USER_DEFINE = 1,
} charge_counter_value_t;

/**
 * @brief Modem context return code
 */
typedef enum modem_ctx_rc_s
{
    MODEM_CTX_RC_SUCCESS,
    MODEM_CTX_RC_ERROR,
} modem_ctx_rc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief   init modem context
 * \retval  void
 */
void modem_context_init( );

/*!
 * \brief  Init events data
 * \retval void
 */
void modem_event_init( void );

/*!
 * \brief For each type of asynchronous message have to manage a counter in case of overrun
 *
 * \param [in] event_type Type of asynchronous message
 *
 * \return The number of asynchronous events for this type of event since the last get event cmd
 */
uint8_t get_modem_event_count( uint8_t event_type );

/*!
 * \brief For each type of asynchronous message have to manage a status
 *
 * \param [in] event_type  type of asynchronous message
 *
 * \return The last status of asynchronous event for this type of event since the last get event cmd
 */
uint8_t get_modem_event_status( uint8_t event_type );

/*!
 * \brief set a value in the counter of a type of asynchronous event
 *
 * \param [in] event_type type of asynchronous message
 * \param [in] value      value's type of asynchronous message
 * \param [in] status     status of asynchronous message
 */
void set_modem_event_count_and_status( uint8_t event_type, uint8_t value, uint8_t status );

/*!
 * \brief increment the counter of a type of asynchronous event
 *
 * \param [in] event_type type of asynchronous message
 * \param [in] status     status of asynchronous message
 */
void increment_modem_event_count_and_status( uint8_t event_type, uint8_t status );

/*!
 * \brief decrement the asynchronous message number
 * \retval void
 */
void decrement_asynchronous_msgnumber( void );

/*!
 * \brief increment the asynchronous message number
 *
 * \param [in] event_type type of asynchronous message
 * \param [in] status     status of asynchronous message
 */
void increment_asynchronous_msgnumber( uint8_t event_type, uint8_t status );

/*!
 * \brief get the last message event
 *
 * \return The last modem event
 */
uint8_t get_last_msg_event( void );

/*!
 * \brief get asynchronous message number
 *
 * \return The number of asynchronous message
 */
uint8_t get_asynchronous_msgnumber( void );

/*!
 * \brief set modem dm interval
 * \remark  This command sets the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * \param   [in]  interval                - Set DM interval
 * \retval dm_rc_t                        - Return dm_rc_t
 */
dm_rc_t set_modem_dm_interval( uint8_t interval );

/*!
 * \brief get modem dm interval
 * \remark  This command returns the device management reporting interval.
 *          The periodic status reporting interval field is encoded in one
 *          byte where the two top-most bits specify the unit :
 *          (00 - sec, 01 - day, 10 - hour, 11 - min),
 *          and the lower six bits the value 0-63.
 *          A value of zero disables the periodic status reporting
 *
 * \retval uint8_t                       - Return the DM interval
 */
uint8_t get_modem_dm_interval( void );

/*!
 * \brief get modem dm interval in second
 * \remark
 * \retval   uint32_t                    - Return the DM interval in seconds
 */
uint32_t get_modem_dm_interval_second( void );

/*!
 * \brief   Set the modem LoRaWAN Class
 * \remark  This command set the LoRaWAN device class.
 *
 * \param   [in]    lorawan_class       - LoRaWAN class
 * \retval   void
 */
void set_modem_class( smtc_modem_class_t lorawan_class );

/*!
 * \brief   Get the modem LoRaWAN Class
 * \remark  This command gets the LoRaWAN device class.
 *
 * \retval   smtc_modem_class_t
 */
smtc_modem_class_t get_modem_class( void );

/*!
 * \brief   Set modem DM port
 * \remark  This command sets the device management port.
 *
 * \param   [in]    port                        - DM port
 * \retval   dm_rc_t
 */
dm_rc_t set_modem_dm_port( uint8_t port );

/*!
 * \brief   Get DM port
 * \remark  This command gets the device management port.
 *
 * \retval [out]    return                      - DM port
 */
uint8_t get_modem_dm_port( void );

/*!
 * \brief   Get Fragmentation port
 * \remark  This command gets the fragmentation port.
 *
 * \retval [out]    return                      - Fragmentation port
 */
uint8_t get_modem_frag_port( void );

/*!
 * \brief   Set ADR profile
 * \remark  This command sets the ADR profile and parameters.
 *
 * \param  [in]     user_dr                     - dr_strategy_t
 * \param  [in]     adr_custom_data             - ADR custom profile data
 * \param  [in]     adr_custom_length           - ADR custom profile data length
 * \retval [out]    dm_rc_t
 */
dm_rc_t set_modem_adr_profile( smtc_modem_adr_profile_t adr_profile, const uint8_t* adr_custom_data,
                               uint8_t adr_custom_length );

/*!
 * \brief   Get ADR profile
 * \remark  This command returns the ADR profile mode.
 *
 * \retval [out]    smtc_modem_adr_profile_t               - Return adr profile
 */
smtc_modem_adr_profile_t get_modem_adr_profile( void );

/*!
 * \brief   merge the join status of the stack and the "join on going" state of the modem
 * \remark
 * \retval   modem_join_state_t           - return join state
 */
modem_join_state_t get_join_state( void );

/*!
 * \brief   Set application-specific status in DM
 * \remark  This commands sets application-specific status information to be reported to the DM service.
 *
 * \param   [out]   app_status*                 - App status payload
 \retval void
 */
void set_modem_appstatus( const uint8_t* app_status );

/*!
 * \brief   Get application-specific status in DM
 * \remark  This commands gets application-specific status information
 *
 * \param   [out]   app_status*                 - App status payload
 * \retval   void
 */
void get_modem_appstatus( uint8_t* app_status );

/*!
 * \brief   Reset the modem charge
 * \remark  This command resets the accumulated charge counter to zero.
 *
 * \retval void
 */
void reset_modem_charge( void );

/*!
 * \brief   Get the modem charge mAs
 * \remark  This command returns the total charge counter of the modem in mAs.
 *
 * \retval   uint32_t                    - Return accumulated charge
 */
uint32_t get_modem_charge_ma_s( void );

/*!
 * \brief   Get the modem charge mAh
 * \remark  This command returns the total charge counter of the modem in mAh.
 *
 * \retval   uint32_t                    - Return accumulated charge
 */
uint32_t get_modem_charge_ma_h( void );

/*!
 * \brief   Get the modem user define charge mAh
 * \remark  This command returns the total charge counter of the modem user define in mAh.
 *          This counter is a read write register for the end user.
 *          Either this value or get_modem_charge_ma_h will be send with dm message status
 * \retval   uint32_t                    - Return accumulated charge
 */
uint16_t get_modem_user_define_charge_ma_h( void );

/*!
 * \brief   Set the modem user define charge mAh
 * \remark  This command set the total charge counter of the modem user define in mAh.
 * \param   [out]   value   - New user define charge counter value
 * \retval  void
 */
void set_modem_user_define_charge_ma_h( const uint16_t value );

/*!
 * \brief   Choose modem charge counter to send within DMmessage status
 * \retval  void
 */
void choose_modem_charge_counter( void );

/*!
 * \brief   Choose user define charge counter to send within DMmessage status
 * \retval  void
 */
void choose_user_define_charge_counter( void );

/*!
 * \brief   Return which charge counter must be send
 * \retval  void
 */
charge_counter_value_t get_charge_counter_to_send( void );

/*!
 * \brief   Get the modem voltage
 * \remark  This command returns the modem voltage
 *
 * \retval   uint8_t                    - Return modem voltage
 */
uint8_t get_modem_voltage( void );

/*!
 * \brief   Get the modem temperature
 * \remark  This command returns the modem temperature
 *
 * \retval  uint8_t                     - Return modem temperature
 */
int8_t get_modem_temp( void );

/*!
 * \brief   return the modem status
 * \remark
 * \retval  uint8_t      bit 0 : reset after brownout
 *                       bit 1 : reset after panic
 *                       bit 2 : modem is muted
 *                       bit 3 : modem is joined
 *                       bit 4 : modem radio communication is suspended
 *                       bit 5 : file upload in progress
 *                       bit 6 : modem is trying to join the network
 *                       bit 7 : streaming in progress
 */
uint8_t get_modem_status( void );

/*!
 * \brief   Set the region
 * \remark  This command sets the regulatory region
 *
 * \param   [in]    region                      - region
 * \retval   return                      - dm_rc_t
 */
dm_rc_t set_modem_region( uint8_t region );

/*!
 * \brief   Get the region
 * \remark  This command returns the current regulatory region.
 *
 * \retval   return                     - Return the region
 */
uint8_t get_modem_region( void );

/*!
 * \brief   Get the modem status after a crash
 * \remark  From bitfield status
 *
 * \retval   return                      - Return crash in bit field status
 */
bool get_modem_status_reset_after_crash( void );

/*!
 * \brief   Get the modem status file upload
 * \remark  From bitfield status
 *
 * \retval   return                      - Return file upload in bit field status
 */
bool get_modem_status_file_upload( void );

/*!
 * \brief   Get the modem status joining
 * \remark  From bitfield status
 *
 * \retval   return                      - Return joining in bit field status
 */
bool get_modem_status_joining( void );

/*!
 * \brief   Get the modem status streaming
 * \remark  From bitfield status
 *
 * \retval  [out]   return                      - Return streaming in bit field status
 */
bool get_modem_status_streaming( void );

/*!
 * \brief   Set the modem status file upload
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set file upload in bit field status
 */
void set_modem_status_file_upload( bool value );

/*!
 * \brief   Set the modem status joining
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set joining in bit field status
 */
void set_modem_status_joining( bool value );

/*!
 * \brief   Set the modem status streaming
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set streaming in bit field status
 */
void set_modem_status_streaming( bool value );

/*!
 * \brief   Set the modem status radio suspend
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set radio suspend in bit field status
 */
void set_modem_status_radio_suspend( bool value );

/*!
 * \brief   Set the modem status join
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set join in bit field status
 */
void set_modem_status_modem_joined( bool value );

/*!
 * \brief   Set the modem status mute
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set mute in bit field status
 */
void set_modem_status_modem_mute( bool value );

/*!
 * \brief   Set the modem status reset after panic
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set reset after panic in bit field status
 */
void set_modem_status_reset_after_crash( bool value );

/*!
 * \brief   Set the modem status reset after brownout
 * \remark  To bitfield status
 *
 * \param   [in]    value                       - Set reset after brownout in bit field status
 */
void set_modem_status_reset_after_brownout( bool value );

/**
 * @brief Set the modem downlink frame object
 *
 * @param data the downlink data received by the lora stack class A or B or C
 * @param data_length the downlink data length
 * @param metadata the downlink metadata (timestamp,rssi,snr and port)
 */
void set_modem_downlink_frame( uint8_t* data, uint8_t data_length, lr1mac_down_metadata_t* metadata );

/*!
 * \brief   Get the Downlink frame in modem context
 * \remark  This function must be called after set_modem_downlink_frame()
 *
 * \param   [in]    modem_dwn*                  - modem_downlink_msg_t
 */
void get_modem_downlink_frame( modem_downlink_msg_t* modem_dwn );

/*!
 * \brief   Set DM retrieve pending downlink frame
 * \remark  This function set the modem supervisor to create downlink opportunities
 *
 * \param   [in]    up_count                    - number of requested uplink
 * \param   [in]    up_delay                    - Delay in second between each uplink
 */
void set_dm_retrieve_pending_dl( uint8_t up_count, uint8_t up_delay );

/*!
 * \brief   Get DM retrieve pending downlink frame
 * \remark  This function get requested downlink opportunities configuration
 *
 * \param   [in]    pending_dl                  - dm_dl_opportunities_config_t
 */
void get_dm_retrieve_pending_dl( dm_dl_opportunities_config_t* pending_dl );

/*!
 * \brief   Decrement DM retrieve pending downlink frame
 * \remark  This function decrement the number of requested downlink opportunities
 *
 */
void decrement_dm_retrieve_pending_dl( void );

/*!
 * \brief   check DM Info cmd Size
 *
 * \param   [in]  cmd                           - Current dm info code that must be checked
 * \param   [in]  length                        - Length of the tested requested dm info code
 * \retval dm_cmd_length_valid_t         - Return valid length or not
 */
dm_cmd_length_valid_t dm_check_dminfo_size( dm_info_field_t cmd, uint8_t length );

/*!
 * \brief   DM SetConf
 * \param   [in]  tag                           - dm_info_field_t that will be handle
 * \param   [in]  data *                        - Current dm info code that must be checked
 * \param   [in]  length                        - Length of the tested requested dm info code+data
 * \retval dm_rc_t                         - Return valid or not
 */
dm_rc_t dm_set_conf( dm_info_field_t tag, uint8_t* data, uint8_t length );

/*!
 * \brief   Check the modem mute state
 *
 * \retval modem_mute_status_t                - Return Modem Muted state
 */
modem_mute_status_t get_modem_muted( void );

/*!
 * \brief   Get the number of muted days when the modem will send a status message anyway
 * \remark  0x00: applicative message allowed
 *          0x01: to 0xFE: number of muted day(s) with status message (applicative message not allowed)
 *          0xFF: never send status message or applicative message allowed
 *
 * \retval uint8_t                       - Return a number of day(s)
 */
uint8_t dm_get_number_of_days_mute( void );

/*!
 * \brief   set the number of muted days when the modem will send a status message anyway
 * \remark  0x00: applicative message allowed
 *          0x01: to 0xFE: number of muted day(s) with status message (applicative message not allowed)
 *          0xFF: never send status message or applicative message allowed
 *
 * \param   [in]  days                          - Set a number of day(s)
 * \retval void
 */
void dm_set_number_of_days_mute( uint8_t days );

/*!
 * \brief   get the DM fields included in the periodic DM status messages
 *
 * \param   [in]  dm *                          - Returned array that contains fields included in the periodic
 *                                                DM status messages.
 * \param   [in]  flag
 *                                              - dm_info_rate_t : If DM_INFO_NOW: set bitfield given by the user
 *                                                                  with GetInfo command,
 *                                                               Else: set bitfield for saved context with SetDmInfo
 * \retval len                           - Return the len of the dm payload
 */
uint8_t get_dm_info_tag_list( uint8_t* dm, dm_info_rate_t flag );

/*!
 * \brief   This command sets the default info fields to be included in the periodic DM status messages.
 *          The set is specified as list of field codes as defined in Uplink Message Format.
 *          An empty set is valid and will effectively disable the DM status message.
 *
 * \param   [in]  requested_info_list         - Array of bytes with requested DM code in each bytes
 * \param   [in]  len                         - Number of byte that composed requested_info_list
 * \param   [in]  flag                        - dm_info_rate_t: If DM_INFO_NOW: set bitfield given by the user with
 *                                                                  GetInfo command,
 *                                                              Else: set bitfield for saved context with SetDmInfo
 * \retval dm_rc_t               - Return DM_ERROR in case of failure, else false DM_OK
 */
dm_rc_t set_dm_info( const uint8_t* requested_info_list, uint8_t len, dm_info_rate_t flag );

/*!
 * \brief   DM status messages
 *
 * \param   [out] dm_uplink_message *         - Returned array that contains one or more concatenated device
 *                                              information fields.
 * \param   [out] dm_uplink_message_len *     - Returned array length
 * \param   [in]  max_size                    - max payload size that must be returned
 * \param   [in]  flag                        - dm_info_rate_t: If DM_INFO_NOW: set bitfield given by the user with
 *                                                                  GetInfo command,
 *                                                              Else: set bitfield for saved context with SetDmInfo
 * \retval bool                               - Return true if there are pending message(s) else false
 */
bool dm_status_payload( uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len, uint8_t max_size,
                        dm_info_rate_t flag );

#if defined( LR1110_MODEM_E )
/*!
 * \brief   DM Fragmented Data Block uplink payload
 *
 * \param   [in]  max_payload_length        - final max length of the constructed payload
 * \param   [out] dm_uplink_message *       - Returned array that contains one or more concatenated ALC Sync data
 * \param   [out] dm_uplink_message_len *   - Returned array length
 * checked \retval void
 */
void dm_frag_uplink_payload( uint8_t max_payload_length, uint8_t* dm_uplink_message, uint8_t* dm_uplink_message_len );
#endif  // LR1110_MODEM_E

#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
/*!
 * \brief   DM Almanac debug answer uplink payload
 *
 * \param   [in]  max_payload_length        - final max length of the constructed payload
 * \param   [out] dm_uplink_message *       - Returned array that contains one or more concatenated ALC Sync data
 * \param   [out] dm_uplink_message_len *   - Returned array length
 * checked \retval void
 */
void dm_alm_dbg_uplink_payload( uint8_t max_payload_length, uint8_t* dm_uplink_message,
                                uint8_t* dm_uplink_message_len );
#endif  // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

/*!
 * \brief    add a join task in scheduler
 * \remark
 */
void modem_supervisor_add_task_join( void );

/*!
 * \brief    add a DM Status task in scheduler
 * \remark
 */
void modem_supervisor_add_task_dm_status( uint32_t next_execute );

/*!
 * \brief    add a DM Status Now task in scheduler
 * \remark
 */
void modem_supervisor_add_task_dm_status_now( void );

/*!
 * \brief    add a crash log task in scheduler
 * \remark
 */
void modem_supervisor_add_task_crash_log( uint32_t next_execute );
/*!
 * \brief    add a ALC Sync time request task in scheduler
 * \remark
 */
void modem_supervisor_add_task_clock_sync_time_req( uint32_t next_execute );

/*!
 * \brief    remove the last ALC Sync time request task in scheduler
 * \remark
 */
void modem_supervisor_remove_task_clock_sync( void );

/*!
 * \brief    Is the Clock Sync time task running
 * \remark
 */
bool modem_supervisor_is_clock_sync_running( void );

/*!
 * \brief    add a ALC Sync answer task in scheduler
 * \remark
 */
void modem_supervisor_add_task_alc_sync_ans( uint32_t next_execute );

/*!
 * \brief    add a Alm Dbg answer task in scheduler
 * \remark
 */
void modem_supervisor_add_task_alm_dbg_ans( uint32_t next_execute );

/*!
 * \brief    add a DM Mute Task to decrement the number of muted day(s)
 * \remark
 */
void modem_supervisor_add_task_modem_mute( void );

/*!
 * \brief    add a task to retrieve all pending downlink
 * \remark
 */
void modem_supervisor_add_task_retrieve_dl( uint32_t next_execute );

#if defined( ADD_SMTC_STREAM )
/*!
 * \brief    add a stream task in scheduler
 * \remark
 */
void modem_supervisor_add_task_stream( void );
#endif  // ADD_SMTC_STREAM

#if defined( ADD_SMTC_FILE_UPLOAD )
/**
 * @brief add a file upload task in scheduler
 *
 * @param [in] delay_in_s The delay in s before task is launched
 */
void modem_supervisor_add_task_file_upload( uint32_t delay_in_s );
#endif  // ADD_SMTC_FILE_UPLOAD

/*!
 * \brief    add a fragmented data block task in scheduler
 * \remark
 */
void modem_supervisor_add_task_frag( uint32_t next_execute );

/*!
 * \brief    add a Link Check Req task in scheduler
 * \remark
 */
void modem_supervisor_add_task_link_check_req( uint32_t delay_in_s );

/*!
 * \brief    add a Device Time Req task in scheduler
 * \remark
 */
void modem_supervisor_add_task_device_time_req( uint32_t delay_in_s );

/*!
 * \brief    add a ping slot info req task in scheduler
 * \remark
 */
void modem_supervisor_add_task_ping_slot_info_req( uint32_t delay_in_s );

/*!
 * \brief    Set modem Suspend
 * \remark
 * \param   [in]  suspend               - True: Suspend modem, False: un-suspend modem
 * \retval dm_rc_t                   - Return DM_ERROR in case of failure, else false DM_OK
 */
dm_rc_t set_modem_suspend( bool suspend );

/*!
 * \brief    Get modem Suspend status
 * \remark
 * \retval modem_suspend_status_t               - Return suspend type
 */
modem_suspend_status_t get_modem_suspend( void );

/*!
 * \brief    Get uptime since last reset in seconds
 * \remark
 * \retval uint32_t                          - Return modem uptime since last reset
 */
uint32_t get_modem_uptime_s( void );

/*!
 * \brief    Set modem start time to compute uptime
 * \remark
 * \param   [in]  time                          - Modem start time
 * \retval  void
 */
void set_modem_start_time_s( uint32_t time );

/*!
 * \brief       Save modem context in non volatile memory
 * \remark
 * \retval   void
 */
void modem_store_context( void );

/*!
 * \brief    load modem context in non volatile memory
 * \remark
 * \retval   void
 */
void modem_load_context( void );

/*!
 * \brief    store the context factory of the modem in the non volatile memory
 * \retval   void
 */
void modem_context_factory_reset( void );

#if defined( ADD_SMTC_STREAM )
/*!
 * \brief    get the stream state
 * \param   [in]  void
 * \retval  [out] modem_stream_status_t
 */
modem_stream_status_t modem_get_stream_state( void );

/*!
 * \brief    get the stream port
 * \param   [in]  void
 * \retval  [out] port
 */
uint8_t modem_get_stream_port( void );

/*!
 * \brief    get the stream encryption
 * \param   [in]  void
 * \retval  [out] encryption
 */
bool modem_get_stream_encryption( void );

/*!
 * \brief    set the stream state
 * \param   [in]  modem_stream_status_t
 * \param   [out] void
 */
void modem_set_stream_state( modem_stream_status_t stream_state );

/*!
 * \brief    set the stream port
 * \param   [in]  port
 * \param   [out] void
 */
void modem_set_stream_port( uint8_t port );

/*!
 * \brief    set the stream encryption
 * \param   [in]  encryption
 * \param   [out] void
 */
void modem_set_stream_encryption( bool enc );
#endif  // ADD_SMTC_STREAM

/**
 * @brief compute the next session counter value and return it
 *
 * @return uint8_t session_counter value on 4 bits
 */
uint8_t modem_context_compute_and_get_next_dm_upload_sctr( void );

/**
 * @brief Get modem internal upload state
 *
 * @return modem_upload_state_t
 */
modem_upload_state_t modem_get_upload_state( void );

/**
 * @brief Set modem internal upload state
 *
 * @param [in] upload_state The upload state
 */
void modem_set_upload_state( modem_upload_state_t upload_state );

/*!
 * \brief    set info_bitfield_periodic
 * \param   [in]  value
 * \retval void
 */
void modem_set_dm_info_bitfield_periodic( uint32_t value );

/*!
 * \brief    get info_bitfield_periodic
 * \remark return bit field for periodic DM status
 * \retval uint32_t
 */
uint32_t modem_get_dm_info_bitfield_periodic( void );

/**
 * @brief Reset dm current tag number
 */
void modem_context_reset_dm_tag_number( void );

/*!
 * \brief    get info_bitfield_periodic
 * \remark   return alarm value in seconds
 * \retval   uint32_t
 */
uint32_t modem_get_user_alarm( void );
/*!
 * \brief   set_user_alarm
 * \param   [in]  alarm value in seconds
 * \retval void
 */
void modem_set_user_alarm( uint32_t alarm );

/*!
 * \brief   Get modem is requested a reset
 * \retval bool          - true if reset is requested
 */
bool get_modem_reset_requested( void );

/*!
 * \brief   Set modem is requested a reset
 * \param   [in]  reset_req     - true if reset is requested
 * \retval  void
 */
void set_modem_reset_requested( bool reset_req );

/*!
 * \brief   Get modem RF Output
 * \retval  rf_output_t
 */
rf_output_t modem_get_rfo_pa( void );

/*!
 * \brief   Set modem RF Output
 * \param   [in]  rf_output     - rf_output_t
 * \retval  uint8_t
 */
uint8_t modem_set_rfo_pa( rf_output_t rf_output );

/*!
 * \brief   Set modem duty cycle when disabled by host
 * \param   [in]  uint8_t     - disabled_by_host
 */
void modem_set_duty_cycle_disabled_by_host( uint8_t disabled_by_host );

/*!
 * \brief   Get modem duty cycle when disabled by host
 * \param   [in]  uint8_t     - disabled_by_host
 * \retval  uint8_t
 */
uint8_t modem_get_duty_cycle_disabled_by_host( void );

/*!
 * \brief   Set modem adr mobile timeout config
 * \param   [in]  uint16_t     - nb_tx
 */
void modem_set_adr_mobile_timeout_config( uint16_t nb_tx );

/*!
 * \brief   Get modem adr mobile timeout config
 * \retval  uint16_t
 */
uint16_t modem_get_adr_mobile_timeout_config( void );

/*!
 * \brief   return true when you receive a link adr request from the network
 * \retval [out]    bool
 */
bool modem_available_new_link_adr_request( void );

/*!
 * \brief   Set modem test mode
 * \param   [in]  bool     - true/false
 */
void modem_set_test_mode_status( bool enable );

/*!
 * \brief   return true if modem test mode is active
 * \retval [out]    bool
 */
bool modem_get_test_mode_status( void );

/*!
 * \brief   Set the Rx Pathloss
 * \remark  This command sets the board-specific correction pathloss for reception
 *
 * \param   [in]     rx_pathloss        - Rx pathloss in dB
 * \retval  None
 */
void modem_context_set_rx_pathloss_db( int8_t rx_pathloss );

/*!
 * \brief   Get the Rx Pathloss
 * \remark  This command gets the board-specific correction pathloss for reception
 *
 * \retval  int8_t
 */
int8_t modem_context_get_rx_pathloss_db( void );

/*!
 * \brief   Set the Tx power offset
 * \remark  This command sets the board-specific correction offset for transmission power to be used
 *
 * \param  [in]     int8_t        - Tx power offset in dB
 * \retval  None
 */
void modem_context_set_tx_power_offset_db( int8_t tx_power_offset );

/*!
 * \brief   Get the Tx power offset
 * \remark  This command gets the board-specific correction offset for transmission power to be used
 *          (signed integer in dB)
 *
 * \retval  int8_t
 */
int8_t modem_context_get_tx_power_offset_db( void );

/*!
 * \brief   Get the pointer on the radio planner used by modem
 *
 * \retval  radio_planner_t* pointer on used radio planner
 */
radio_planner_t* modem_context_get_modem_rp( void );

/*!
 * \brief   Save in context the pointer on the radio planner used by modem
 *
 * \param  [in]     radio_planner_t*        - the pointer on radio planner
 * \retval  none
 */
void modem_context_set_modem_rp( radio_planner_t* rp );

/*!
 * \brief   suspend radio access for user
 * \remark
 * \param  [in]     rp_task_types_t        - the type on radio planner task
 * \retval  true if operation was ok, false otherwise
 */

bool modem_context_suspend_user_radio_access( rp_task_types_t type );

/*!
 * \brief   resume radio access for user
 * \remark  This command must be moved to a better place of code (modem_engine for instance)
 *
 * \retval  true if operation was ok, false otherwise
 */
bool modem_context_resume_user_radio_access( void );

/*!
 * \brief   suspend radio access
 * \remark  This command must be moved to a better place of code (modem_engine for instance)
 *
 * \retval  true if operation was ok, false otherwise
 */
bool modem_context_suspend_radio_access( rp_task_types_t type );

/*!
 * \brief   resume radio access
 * \remark  This command must be moved to a better place of code (modem_engine for instance)
 *
 * \retval  true if operation was ok, false otherwise
 */
bool modem_context_resume_radio_access( void );

/*!
 * \brief   Set the power config lut
 * \remark  This command sets the internal power config look up table
 *
 * \param  [in]     config        - the 30 bytes buffer received from user
 * \retval  None
 */
void modem_context_set_power_config_lut( uint8_t config[30] );

/*!
 * \brief   Get the power config lut
 * \remark  This command returns the pointer on the internal look up table
 *
 * \retval  modem_power_config_t *
 */
modem_power_config_t* modem_context_get_power_config_lut( void );

/**
 * @brief Check appkey crc and status. And set them if required
 *
 * @param [in] app_key Key buffer
 * @return modem_ctx_rc_t
 */
modem_ctx_rc_t modem_context_set_appkey( const uint8_t app_key[16] );

/**
 * @brief Update appkey_crc status to invalid. (because appkey is no longer know)
 *
 */
void modem_context_appkey_is_derived( void );

/**
 * @brief get network type
 *
 * @return true public network
 * @return false private network
 */
bool modem_context_get_network_type( void );

/**
 * @brief set network_type
 *
 * @param network_type
 */
void modem_context_set_network_type( bool network_type );

/**
 * @brief get modem radio context
 *
 * @return the radio context
 */
const void* modem_context_get_modem_radio_ctx( void );

/**
 * @brief save the modem radio context
 *
 * @param radio_ctx the radio context
 */
void modem_context_set_modem_radio_ctx( const void* radio_ctx );

/**
 * @brief Set D2D metadata in modem context
 *
 * @param mc_grp_id
 * @param tx_done
 * @param nb_trans
 */
void modem_context_set_class_b_d2d_last_metadata( uint8_t mc_grp_id, bool tx_done, uint8_t nb_trans );

/**
 * @brief Get D2D metadata in modem context
 *
 * @param [out] class_b_d2d
 */
void modem_context_get_class_b_d2d_last_metadata( modem_context_class_b_d2d_t* class_b_d2d );

/**
 * @brief Set  callback provided by the middleware layer
 *
 * @param [int ] callback provided by the middleware layer , lbm have to call it once the extended tx is finished
 * @param [int ] extended_uplink_id, the id of the extended tx queue
 */
void modem_set_extended_callback( func_callback, uint8_t extended_uplink_id );
/**
 * @brief Get  callback provided by the middleware layer
 **@param [int ] extended_uplink_id, the id of the extended tx queue
 * @param [out ] callback provided by the middleware layer , lbm have to call it once the extended tx is finished
 */
func_callback modem_get_extended_callback( uint8_t extended_uplink_id );

/**
 * @brief Take action for leaving the network
 *
 */
void modem_leave( void );


#ifdef __cplusplus
}
#endif

#endif  // __SOFT_MODEM_CONTEXT_H__

/* --- EOF ------------------------------------------------------------------ */
