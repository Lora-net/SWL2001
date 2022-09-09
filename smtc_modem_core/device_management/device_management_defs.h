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

#ifndef DEVICE_MANAGEMENT_DEFS_H__
#define DEVICE_MANAGEMENT_DEFS_H__

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

/**
 * @brief Definition of the DAS downlink request opcodes
 *
 * @enum dm_opcode_t
 */
typedef enum dm_opcode_e
{
    DM_RESET = 0x00,  //!< reset modem or application MCU
    DM_FUOTA = 0x01,  //!< FUOTA firmware update chunk
#if defined( ADD_SMTC_FILE_UPLOAD )
    DM_FILE_DONE = 0x02,    //!< file upload complete
#endif                      // ADD_SMTC_FILE_UPLOAD
    DM_GET_INFO    = 0x03,  //!< get info fields
    DM_SET_CONF    = 0x04,  //!< set config field
    DM_REJOIN      = 0x05,  //!< rejoin network
    DM_MUTE        = 0x06,  //!< permanently disable/enable modem
    DM_SET_DM_INFO = 0x07,  //!< set list of default info fields
#if defined( ADD_SMTC_STREAM )
    DM_STREAM = 0x08,      //!< set data stream parameters
#endif                     // ADD_SMTC_STREAM
    DM_ALC_SYNC   = 0x09,  //!< application layer clock sync data
    DM_ALM_UPDATE = 0x0A,  //!< almanac update, short, long, full updates
#if !defined( DISABLE_ALMANAC_DBG_OPCODE )
    DM_ALM_DBG = 0x0B,      //!< almanac debug
#endif                      // !DISABLE_ALMANAC_DBG_OPCODE
    DM_SOLV_UPDATE = 0x0C,  //!< assistance position, xtal update
    DM_ALM_FUPDATE = 0x0D,  //!< Force almanac update, short, long, full updates
    DM_CMD_MAX              //!< number of elements
} dm_opcode_t;

/**
 * @brief Downlink Message Format.
 *        The cloud service might return one or more request messages which have to be delivered over
 *        the network back to the modem on the device management port.
 *        All downlink messages have the following format.
 *
 * @remark  Next to the request code and data each message contains an upcount field which indicates the
 *          number of uplinks to generate.
 *          These uplinks can be used to create additional downlink opportunities and should be generated
 *          at the rate specified by the updelay field.
 *          The reception of a new request message resets the uplink generation.
 *
 * @struct dm_cmd_msg_t
 */
typedef struct dm_cmd_msg_s
{
    uint8_t     up_count;      //!< uplink count
    uint8_t     up_delay;      //!< uplink delay [s]
    dm_opcode_t request_code;  //!< request code
    uint8_t*    buffer;        //!< request data
    uint8_t     buffer_len;    //!< request data length in byte(s)
} dm_cmd_msg_t;

/**
 * @brief Defined the type of reset that can be requested by downlink
 *
 * @enum dm_reset_code_t
 */
typedef enum dm_reset_code_e
{
    DM_RESET_MODEM   = 0x01,  //!< Reset the modem
    DM_RESET_APP_MCU = 0x02,  //!< Reset the MCU
    DM_RESET_BOTH    = 0x03,  //!< Reset modem and app MCU
    DM_RESET_MAX              //!< number of elements
} dm_reset_code_t;

/**
 * @brief Definition of return codes for dm functions
 *
 * @enum dm_rc_t
 */
typedef enum dm_rc_e
{
    DM_ERROR,  //!< Error during dm function
    DM_OK,     //!< dm function executed without error
} dm_rc_t;

/**
 * @brief Current modem join state
 *
 * @enum modem_join_state_t
 */
typedef enum modem_join_state_e
{
    MODEM_NOT_JOINED,    //!< The modem joined a network
    MODEM_JOIN_ONGOING,  //!< The modem is ongoing to join a network
    MODEM_JOINED         //!< The modem is not joined to a network
} modem_join_state_t;

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

/**
 * @brief Type of DM, immediate or periodic reporting
 *
 * @enum dm_info_rate_t
 */
typedef enum dm_info_rate_e
{
    DM_INFO_PERIODIC = 0,  //!< Related to the Device Management periodic reporting
    DM_INFO_NOW            //!< Related to the Device Management immediately reporting
} dm_info_rate_t;

/**
 * @brief Status of DM downlink length
 *
 * @enum dm_cmd_length_valid_t
 */
typedef enum dm_cmd_length_valid_e
{
    DM_CMD_LENGTH_VALID,      //!< The length of the command is valid
    DM_CMD_LENGTH_NOT_VALID,  //!< The length of the command is not valid
    DM_CMD_NOT_VALID,         //!< The command is not valid
} dm_cmd_length_valid_t;

/**
 * @brief Modem mute status
 *
 * @enum modem_mute_status_t
 */
typedef enum modem_mute_status_e
{
    MODEM_NOT_MUTE = 0x00,      //!< The modem is not muted
    MODEM_TEMPORARY_MUTE,       //!< The modem is muted for a defined time
    MODEM_INFINITE_MUTE = 0xFF  //!< The modem is infinitely muted
} modem_mute_status_t;

/**
 * @brief Modem suspend status
 *
 * @enum modem_suspend_status_t
 */
typedef enum modem_suspend_status_e
{
    MODEM_NOT_SUSPEND = 0x00,  //!< The modem is not suspend
    MODEM_SUSPEND              //!< The modem is suspend
} modem_suspend_status_t;

/**
 * @brief Modem file upload state
 *
 * @enum modem_upload_state_t
 */
typedef enum modem_upload_state_e
{
    MODEM_UPLOAD_NOT_INIT = 0,     //!< The file upload is not initialized
    MODEM_UPLOAD_INIT_AND_FILLED,  //!< The file upload is initialized and filled with datas
    MODEM_UPLOAD_ON_GOING,         //!< The upload is in progress
    MODEM_UPLOAD_FINISHED          //!< The upload process is finished
} modem_upload_state_t;

#if defined( ADD_SMTC_STREAM )
/**
 * @brief Modem stream state
 *
 * @enum modem_stream_status_t
 */
typedef enum modem_stream_status_e
{
    MODEM_STREAM_NOT_INIT = 0,
    MODEM_STREAM_INIT,
    MODEM_STREAM_DATA_PENDING,
} modem_stream_status_t;
#endif  // ADD_SMTC_STREAM

/**
 * @brief RF Output definition
 *
 * @enum rf_output_t
 */
typedef enum rf_output_e
{
    MODEM_RFO_LP_LF        = 0x00,
    MODEM_RFO_HP_LF        = 0x01,
    MODEM_RFO_LP_AND_HP_LF = 0x02,
    MODEM_RFO_MAX,
} rf_output_t;

/**
 * @brief Modem Status offset
 *
 * @enum modem_status_offset_t
 */
typedef enum modem_status_offset_e
{
    MODEM_STATUS_OFFSET_BROWNOUT  = 0,  //!< reset after brownout
    MODEM_STATUS_OFFSET_CRASH     = 1,  //!< reset after panic
    MODEM_STATUS_OFFSET_MUTE      = 2,  //!< modem is muted
    MODEM_STATUS_OFFSET_JOINED    = 3,  //!< modem has joined network
    MODEM_STATUS_OFFSET_SUSPEND   = 4,  //!< radio operations suspended (low power)
    MODEM_STATUS_OFFSET_UPLOAD    = 5,  //!< file upload in progress
    MODEM_STATUS_OFFSET_JOINING   = 6,  //!< modem is trying to join the network
    MODEM_STATUS_OFFSET_STREAMING = 7   //!< streaming in progress
} modem_status_offset_t;

/**
 * @brief Periodic Status Reporting field opcode
 *
 * @enum dm_info_field_t
 */
typedef enum dm_info_field_e
{
    DM_INFO_STATUS    = 0x00,  //!< modem status
    DM_INFO_CHARGE    = 0x01,  //!< charge counter [mAh]
    DM_INFO_VOLTAGE   = 0x02,  //!< supply voltage [1/50 V]
    DM_INFO_TEMP      = 0x03,  //!< junction temperature [deg Celsius]
    DM_INFO_SIGNAL    = 0x04,  //!< strength of last downlink (RSSI [dBm]+64, SNR [0.25 dB])
    DM_INFO_UPTIME    = 0x05,  //!< duration since last reset [h]
    DM_INFO_RXTIME    = 0x06,  //!< duration since last downlink [h]
    DM_INFO_FIRMWARE  = 0x07,  //!< firmware CRC and fuota progress (completed/total chunks)
    DM_INFO_ADRMODE   = 0x08,  //!< ADR profile (0-3)
    DM_INFO_JOINEUI   = 0x09,  //!< JoinEUI
    DM_INFO_INTERVAL  = 0x0A,  //!< reporting interval [values 0-63, units s/m/h/d]
    DM_INFO_REGION    = 0x0B,  //!< regulatory region
    DM_INFO_RFU_0     = 0x0C,  //!< not defined
    DM_INFO_CRASHLOG  = 0x0D,  //!< crash log data
    DM_INFO_UPLOAD    = 0x0E,  //!< application file fragments
    DM_INFO_RSTCOUNT  = 0x0F,  //!< modem reset count
    DM_INFO_DEVEUI    = 0x10,  //!< DevEUI
    DM_INFO_RFU_1     = 0x11,  //!< not defined, old owner number
    DM_INFO_SESSION   = 0x12,  //!< session id / join nonce
    DM_INFO_CHIPEUI   = 0x13,  //!< ChipEUI
    DM_INFO_STREAM    = 0x14,  //!< data stream fragments
    DM_INFO_STREAMPAR = 0x15,  //!< data stream parameters
    DM_INFO_APPSTATUS = 0x16,  //!< application-specific status
    DM_INFO_ALCSYNC   = 0x17,  //!< application layer clock sync data
    DM_INFO_ALMSTATUS = 0x18,  //!< almanac status
    DM_INFO_DBGRSP    = 0x19,  //!< almanac dbg response
    DM_INFO_GNSSLOC   = 0x1A,  //!< GNSS scan NAV message
    DM_INFO_WIFILOC   = 0x1B,  //!< Wifi scan results message
    DM_INFO_MAX                //!< number of elements
} dm_info_field_t;

/**
 * @brief DM interval unit definitions
 *
 * @enum dm_interval_unit_t
 */
typedef enum dm_interval_unit_e
{
    DM_INTERVAL_UNIT_SEC  = 0,  //!< Interval in second(s)
    DM_INTERVAL_UNIT_DAY  = 1,  //!< Interval in day(s)
    DM_INTERVAL_UNIT_HOUR = 2,  //!< Interval in hour(s)
    DM_INTERVAL_UNIT_MIN  = 3   //!< Interval in minute(s)
} dm_interval_unit_t;

#if defined( ADD_SMTC_STREAM )
/**
 * @brief Modem stream structure
 *
 * @struct modem_stream_t
 */
typedef struct modem_stream_s
{
    uint8_t               port;
    modem_stream_status_t state;
    bool                  encryption;
} modem_stream_t;
#endif  // ADD_SMTC_STREAM

/**
 * @brief Downlink message structure
 *
 * @struct modem_downlink_msg_t
 */
typedef struct modem_downlink_msg_s
{
    uint8_t  port;          //!< LoRaWAN FPort
    uint8_t  data[242];     //!< data received
    uint8_t  length;        //!< data length in byte(s)
    int16_t  rssi;          //!< RSSI is a signed value in dBm + 64
    int16_t  snr;           //!< SNR is a signed value in 0.25 dB steps
    uint32_t timestamp;     //!< timestamp of the received message
    bool     fpending_bit;  //!< status of the frame pending bit
    uint32_t frequency_hz;  //!< Frequency of the received message
    uint8_t  datarate;      //!< Datarate of the received message
} modem_downlink_msg_t;

/**
 * @brief Downlink opportunities configuration
 *
 * @struct dm_dl_opportunities_config_t
 */
typedef struct dm_dl_opportunities_config_s
{
    uint8_t up_count;  //!< uplink count
    uint8_t up_delay;  //!< uplink delay [s]
} dm_dl_opportunities_config_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // DEVICE_MANAGEMENT_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */
