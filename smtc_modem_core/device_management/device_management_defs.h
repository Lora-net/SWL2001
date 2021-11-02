/*!
 * \file      device_management_defs.h
 *
 * \brief     definition file for device management
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

#ifndef __DEVICE_MANAGEMENT_DEFS_H__
#define __DEVICE_MANAGEMENT_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
#define GNSS_DM_MSG 2  // Message for the device management
#endif                 // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \typedef e_dm_cmd_t
 * \brief   The following downlink requests from the cloud are defined
 */
typedef enum dm_cmd
{
    DM_RESET       = 0x00,  //!< reset modem or application MCU
    DM_FUOTA       = 0x01,  //!< FUOTA firmware update chunk
    DM_FILE_DONE   = 0x02,  //!< file upload complete
    DM_GET_INFO    = 0x03,  //!< get info fields
    DM_SET_CONF    = 0x04,  //!< set config field
    DM_REJOIN      = 0x05,  //!< rejoin network
    DM_MUTE        = 0x06,  //!< permanently disable/enable modem
    DM_SET_DM_INFO = 0x07,  //!< set list of default info fields
    DM_STREAM      = 0x08,  //!< set data stream parameters
    DM_ALC_SYNC    = 0x09,  //!< application layer clock sync data
    DM_ALM_UPDATE  = 0x0A,  //!< almanac update, short, long, full updates
    DM_ALM_DBG     = 0x0B,  //!< almanac debug
    DM_SOLV_UPDATE = 0x0C,  //!< assistance position, xtal update
    DM_ALM_FUPDATE = 0x0D,  //!< Force almanac update, short, long, full updates
    DM_CMD_MAX              //!< number of elements
} e_dm_cmd_t;

/*!
 * \typedef s_dm_cmd_input_t
 * \brief   Downlink Message Format.
 *          The cloud service might return one or more request messages which have to be delivered over
 *          the network back to the modem on the device management port.
 *          All downlink messages have the following format.
 *
 * \remark  Next to the request code and data each message contains an upcount field which indicates the
 *          number of uplinks to generate.
 *          These uplinks can be used to create additional downlink opportunities and should be generated
 *          at the rate specified by the updelay field.
 *          The reception of a new request message resets the uplink generation.
 */
typedef struct s_dm_cmd_input
{
    uint8_t    up_count;      //!< uplink count
    uint8_t    up_delay;      //!< uplink delay [s]
    e_dm_cmd_t request_code;  //!< request code
    uint8_t*   buffer;        //!< request data
    uint8_t    buffer_len;    //!< request data length in byte(s)
} s_dm_cmd_input_t;

/*!
 * \typedef e_dm_reset_code_t
 * \brief   Defined the type of reset that can be requested by downlink
 */
typedef enum DM_RESET_CODE
{
    DM_RESET_MODEM   = 0x01,  //!< Reset the modem
    DM_RESET_APP_MCU = 0x02,  //!< Reset the MCU
    DM_RESET_BOTH    = 0x03,  //!< Reset modem and app MCU
    DM_RESET_MAX              //!< number of elements
} e_dm_reset_code_t;

/*!
 * \typedef e_set_error_t
 * \brief   create a typedef error for set function with illegal input value
 */
typedef enum set_error_e
{
    SET_ERROR,  //!< Invalid parameter(s)
    SET_OK,     //!< Valid parameter(s)
} e_set_error_t;

/*!
 * \typedef eModemJoinState_t
 * \brief   Current modem join state
 */
typedef enum ModemJoinState
{
    MODEM_NOT_JOINED,    //!< The modem joined a network
    MODEM_JOIN_ONGOING,  //!< The modem is ongoing to join a network
    MODEM_JOINED         //!< The modem is not joined to a network
} eModemJoinState_t;

/**
 * @brief The parameter of the TxDone event indicates the status of the requested Tx
 *
 * @enum event_tx_done_state_t
 */
typedef enum event_tx_done_state_e
{
    MODEM_TX_FAILED           = 0,  //!< The frame was not sent
    MODEM_TX_SUCCESS          = 1,  //!< The frame was but not acknowledge
    MODEM_TX_SUCCESS_WITH_ACK = 2   //!< The frame was and acknowledge
} event_tx_done_state_t;

/**
 * @brief Almanac update event status
 *
 * @enum event_almanac_update_status_t
 */
typedef enum event_almanac_update_status_e
{
    ALMANAC_EVENT_ALL_UPDATES_RECEIVED = 0,  //!< Almanac update was fully received
    ALMANAC_EVENT_DAS_NEED_NEW_CRC     = 1,  //!< Almanac update was not fully received, DAS need to dl more packets
} event_almanac_update_status_t;

/*!
 * \typedef e_dm_error_t
 * \brief   create a typedef error for set function with illegal input value
 */
typedef enum dm_error
{
    DM_ERROR = 0,  //!< DM downlink with invalid parameter(s)
    DM_OK          //!< DM downlink with valid parameter(s)
} e_dm_error_t;

/*!
 * \typedef e_dm_info_rate_t
 * \brief   Type of DM, immediate or periodic reporting
 */
typedef enum e_dm_info_rate
{
    DM_INFO_PERIODIC = 0,  //!< Related to the Device Management periodic reporting
    DM_INFO_NOW            //!< Related to the Device Management immediately reporting
} e_dm_info_rate_t;

/*!
 * \typedef e_dm_cmd_length_valid
 * \brief   Status of DM downlink
 */
typedef enum DM_CMD_LENGTH
{
    DM_CMD_LENGTH_VALID,      //!< The length of the command is valid
    DM_CMD_LENGTH_NOT_VALID,  //!< The length of the command is not valid
    DM_CMD_NOT_VALID,         //!< The command is not valid
} e_dm_cmd_length_valid;

/*!
 * \typedef e_modem_mute_t
 * \brief   Modem mute state
 */
typedef enum e_modem_mute
{
    MODEM_NOT_MUTE = 0x00,      //!< The modem is not muted
    MODEM_TEMPORARY_MUTE,       //!< The modem is muted for a defined time
    MODEM_INFINITE_MUTE = 0xFF  //!< The modem is infinitely muted
} e_modem_mute_t;

/*!
 * \typedef e_modem_suspend_t
 * \brief   Modem suspend state
 */
typedef enum e_modem_suspend
{
    MODEM_NOT_SUSPEND = 0x00,  //!< The modem is not suspend
    MODEM_SUSPEND              //!< The modem is suspend
} e_modem_suspend_t;

/*!
 * \typedef modem_upload_state_t
 * \brief   Modem file upload state
 */
typedef enum modem_upload_state_e
{
    MODEM_UPLOAD_NOT_INIT = 0,     //!< The file upload is not initialized
    MODEM_UPLOAD_INIT_AND_FILLED,  //!< The file upload is initialized and filled with datas
    MODEM_UPLOAD_ON_GOING,         //!< The upload is in progress
    MODEM_UPLOAD_FINISHED          //!< The upload process is finished
} modem_upload_state_t;

/*!
 * \typedef e_modem_stream_state_t
 * \brief   Modem stream state
 */
typedef enum e_modem_stream_state
{
    MODEM_STREAM_NOT_INIT = 0,
    MODEM_STREAM_INIT,
    MODEM_STREAM_DATA_PENDING,
} e_modem_stream_state_t;

/*!
 * \typedef e_modem_dwn_data_t
 * \brief   not used
 */
typedef enum e_modem_dwn_data
{
    MODEM_DWN_DATA_ACK       = 0x80,  //!< Data acked
    MODEM_DWN_DATA_NACK      = 0x40,  //!< Data not acked
    MODEM_DWN_DATA_RX_1      = 0x01,  //!< Data received through Rx1
    MODEM_DWN_DATA_RX_2      = 0x02,  //!< Data received through Rx2
    MODEM_DWN_DATA_PING_SLOT = 0x04,  //!< Data received through ping slot (class B)
} e_modem_dwn_data_t;

/*!
 * \typedef rf_output_e
 * \brief  RF Output
 */
typedef enum rf_output_e
{
    MODEM_RFO_LP_LF        = 0x00,
    MODEM_RFO_HP_LF        = 0x01,
    MODEM_RFO_LP_AND_HP_LF = 0x02,
    MODEM_RFO_MAX,
} rf_output_t;

/*!
 * \typedef e_modem_status_t
 * \brief   Modem Status
 */
typedef enum e_modem_status
{
    modem_status_brownout  = 0,  //!< reset after brownout
    modem_status_crash     = 1,  //!< reset after panic
    modem_status_mute      = 2,  //!< device is muted
    modem_status_joined    = 3,  //!< device has joined network
    modem_status_suspend   = 4,  //!< radio operations suspended (low power)
    modem_status_upload    = 5,  //!< file upload in progress
    modem_status_joining   = 6,  //!< device is trying to join the network
    modem_status_streaming = 7   //!< streaming in progress
} e_modem_status_t;

/*!
 * \typedef e_dm_info_t
 * \brief   Periodic Status Reporting field
 */
typedef enum e_dm_info
{
    e_inf_status    = 0x00,  //!< modem status
    e_inf_charge    = 0x01,  //!< charge counter [mAh]
    e_inf_voltage   = 0x02,  //!< supply voltage [1/50 V]
    e_inf_temp      = 0x03,  //!< junction temperature [deg Celsius]
    e_inf_signal    = 0x04,  //!< strength of last downlink (RSSI [dBm]+64, SNR [0.25 dB])
    e_inf_uptime    = 0x05,  //!< duration since last reset [h]
    e_inf_rxtime    = 0x06,  //!< duration since last downlink [h]
    e_inf_firmware  = 0x07,  //!< firmware CRC and fuota progress (completed/total chunks)
    e_inf_adrmode   = 0x08,  //!< ADR profile (0-3)
    e_inf_joineui   = 0x09,  //!< JoinEUI
    e_inf_interval  = 0x0A,  //!< reporting interval [values 0-63, units s/m/h/d]
    e_inf_region    = 0x0B,  //!< regulatory region
    e_inf_rfu_0     = 0x0C,  //!< not defined
    e_inf_crashlog  = 0x0D,  //!< crash log data
    e_inf_upload    = 0x0E,  //!< application file fragments
    e_inf_rstcount  = 0x0F,  //!< modem reset count
    e_inf_deveui    = 0x10,  //!< DevEUI
    e_inf_rfu_1     = 0x11,  //!< not defined, old owner number
    e_inf_session   = 0x12,  //!< session id / join nonce
    e_inf_chipeui   = 0x13,  //!< ChipEUI
    e_inf_stream    = 0x14,  //!< data stream fragments
    e_inf_streampar = 0x15,  //!< data stream parameters
    e_inf_appstatus = 0x16,  //!< application-specific status
    e_inf_alcsync   = 0x17,  //!< application layer clock sync data
    e_inf_almstatus = 0x18,  //!< almanac status
    e_inf_dbgrsp    = 0x19,  //!< almanac dbg response
    e_inf_gnssloc   = 0x1A,  //!< GNSS scan NAV message
    e_inf_wifiloc   = 0x1B,  //!< Wifi scan results message
    e_inf_max                //!< number of elements
} e_dm_info_t;

// field sizes
static const uint8_t dm_info_field_sz[e_inf_max] = {
    [e_inf_status] = 1,    [e_inf_charge] = 2,    [e_inf_voltage] = 1,  [e_inf_temp] = 1,    [e_inf_signal] = 2,
    [e_inf_uptime] = 2,    [e_inf_rxtime] = 2,    [e_inf_firmware] = 8, [e_inf_adrmode] = 1, [e_inf_joineui] = 8,
    [e_inf_interval] = 1,  [e_inf_region] = 1,    [e_inf_rfu_0] = 4,
    [e_inf_crashlog] = 0,  // (variable-length, send as last field or in separate frame)
    [e_inf_upload]   = 0,  // (variable-length, not sent periodically)
    [e_inf_rstcount] = 2,  [e_inf_deveui] = 8,    [e_inf_rfu_1] = 2,    [e_inf_session] = 2, [e_inf_chipeui] = 8,
    [e_inf_stream]    = 0,  // (variable-length, not sent periodically)
    [e_inf_streampar] = 2, [e_inf_appstatus] = 8,
    [e_inf_alcsync]   = 0,  // (variable-length, not sent periodically)
    [e_inf_almstatus] = 7
};

/*!
 * \typedef e_dm_interval_unit_t
 * \brief   DM interval unit
 */
typedef enum e_dm_interval_unit
{
    DM_INTERVAL_UNIT_SEC  = 0,  //!< Interval in second(s)
    DM_INTERVAL_UNIT_DAY  = 1,  //!< Interval in day(s)
    DM_INTERVAL_UNIT_HOUR = 2,  //!< Interval in hour(s)
    DM_INTERVAL_UNIT_MIN  = 3   //!< Interval in minute(s)
} e_dm_interval_unit_t;

/*!
 * \typedef s_modem_stream_t
 * \brief   Uplink Stream structure
 */
typedef struct s_modem_stream
{
    uint8_t                port;
    e_modem_stream_state_t state;
    bool                   encryption;
} s_modem_stream_t;

/*!
 * \typedef s_modem_dwn_t
 * \brief   Downlink packet structure
 */
typedef struct s_modem_dwn
{
    uint8_t  port;       //!< LoRaWAN FPort
    uint8_t  data[242];  //!< data received
    uint8_t  length;     //!< data length in byte(s)
    int16_t  rssi;       //!< RSSI is a signed value in dBm + 64
    int16_t  snr;        //!< SNR is a signed value in 0.25 dB steps
    uint32_t timestamp;  //!< timestamp of the received message
} s_modem_dwn_t;

/*!
 * \typedef s_dm_retrieve_pending_dl_t
 * \brief
 */
typedef struct s_dm_retrieve_pending_dl
{
    uint8_t up_count;  //!< uplink count
    uint8_t up_delay;  //!< uplink delay [s]
} s_dm_retrieve_pending_dl_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

static const uint8_t dm_cmd_len[DM_CMD_MAX][2] = {  // CMD              = {min,       max}
    [DM_RESET] = { 3, 3 },         [DM_FUOTA] = { 1, 255 },
    [DM_FILE_DONE] = { 1, 1 },     [DM_GET_INFO] = { 1, 255 },
    [DM_SET_CONF] = { 2, 255 },    [DM_REJOIN] = { 2, 2 },
    [DM_MUTE] = { 1, 1 },          [DM_SET_DM_INFO] = { 1, e_inf_max },
    [DM_STREAM] = { 1, 255 },      [DM_ALC_SYNC] = { 1, 255 },
    [DM_ALM_UPDATE] = { 1, 255 },  [DM_ALM_DBG] = { 1, 255 },
    [DM_SOLV_UPDATE] = { 1, 255 }, [DM_ALM_FUPDATE] = { 1, 255 }
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __DEVICE_MANAGEMENT_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */
