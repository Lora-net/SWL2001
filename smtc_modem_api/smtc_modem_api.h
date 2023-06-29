/**
 * @file      smtc_modem_api.h
 *
 * @brief     Generic Modem API description
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

#ifndef SMTC_MODEM_API_H__
#define SMTC_MODEM_API_H__

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief EUI (joinEUI, devEUI, etc.) length in byte
 */
#define SMTC_MODEM_EUI_LENGTH 8

/**
 * @brief Cryptographic key length in byte
 */
#define SMTC_MODEM_KEY_LENGTH 16

/**
 * @brief Application-defined user data length in byte
 */
#define SMTC_MODEM_DM_USER_DATA_LENGTH 8

/**
 * @brief ADR custom configuration length in byte
 */
#define SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH 16

/**
 * @brief Maximum payload size in byte of a downlink
 */
#define SMTC_MODEM_MAX_DOWNLINK_LENGTH 242

/**
 * @brief Size of the device to device ping slots mask
 */
#define SMTC_MODEM_D2D_PING_SLOTS_MASK_SIZE 16

/**
 * @defgroup SMTC_MODEM_EVENT_DEF Event codes definitions
 * @{
 */
#define SMTC_MODEM_EVENT_RESET 0x00                   //!< Modem has been reset
#define SMTC_MODEM_EVENT_ALARM 0x01                   //!< Alarm timer expired
#define SMTC_MODEM_EVENT_JOINED 0x02                  //!< Network successfully joined
#define SMTC_MODEM_EVENT_TXDONE 0x03                  //!< Frame transmitted
#define SMTC_MODEM_EVENT_DOWNDATA 0x04                //!< Downlink data received
#define SMTC_MODEM_EVENT_UPLOADDONE 0x05              //!< File upload completed
#define SMTC_MODEM_EVENT_SETCONF 0x06                 //!< Configuration has been changed by the Device Management
#define SMTC_MODEM_EVENT_MUTE 0x07                    //!< Modem has been muted or un-muted by the Device Management
#define SMTC_MODEM_EVENT_STREAMDONE 0x08              //!< Stream upload completed (stream data buffer depleted)
#define SMTC_MODEM_EVENT_JOINFAIL 0x0A                //!< Attempt to join network failed
#define SMTC_MODEM_EVENT_TIME 0x0D                    //!< Update on time happened (synced or invalid)
#define SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED 0x0E     //!< ADR profile was switched to network controlled
#define SMTC_MODEM_EVENT_NEW_LINK_ADR 0x0F            //!< New link ADR requested by network
#define SMTC_MODEM_EVENT_LINK_CHECK 0x10              //!< Link Check answered by network
#define SMTC_MODEM_EVENT_ALMANAC_UPDATE 0x11          //!< An almanac update has been received
#define SMTC_MODEM_EVENT_USER_RADIO_ACCESS 0x12       //!< radio callback when user use the radio by itself
#define SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO 0x13  //!< Ping Slot Info answered by network
#define SMTC_MODEM_EVENT_CLASS_B_STATUS 0x14          //!< Downlink class B is ready or not
#define SMTC_MODEM_EVENT_NONE 0xFF                    //!< No event available
/**
 * @}
 */

/**
 * @defgroup SMTC_MODEM_DM_INFO_DEF DM info fields codes
 * @{
 */
#define SMTC_MODEM_DM_FIELD_STATUS 0x00          //!< modem status
#define SMTC_MODEM_DM_FIELD_CHARGE 0x01          //!< charge counter [mAh]
#define SMTC_MODEM_DM_FIELD_VOLTAGE 0x02         //!< supply voltage [1/50 V]
#define SMTC_MODEM_DM_FIELD_TEMPERATURE 0x03     //!< junction temperature [deg Celsius]
#define SMTC_MODEM_DM_FIELD_SIGNAL 0x04          //!< strength of last downlink (RSSI [dBm]+64, SNR [0.25 dB])
#define SMTC_MODEM_DM_FIELD_UP_TIME 0x05         //!< duration since last reset [h]
#define SMTC_MODEM_DM_FIELD_RX_TIME 0x06         //!< duration since last downlink [h]
#define SMTC_MODEM_DM_FIELD_ADR_MODE 0x08        //!< ADR profile (0-3)
#define SMTC_MODEM_DM_FIELD_JOIN_EUI 0x09        //!< JoinEUI
#define SMTC_MODEM_DM_FIELD_INTERVAL 0x0A        //!< reporting interval [values 0-63, units s/m/h/d]
#define SMTC_MODEM_DM_FIELD_REGION 0x0B          //!< regulatory region
#define SMTC_MODEM_DM_FIELD_RST_COUNT 0x0F       //!< modem reset count
#define SMTC_MODEM_DM_FIELD_DEV_EUI 0x10         //!< DevEUI
#define SMTC_MODEM_DM_FIELD_SESSION 0x12         //!< session id / join nonce
#define SMTC_MODEM_DM_FIELD_CHIP_EUI 0x13        //!< ChipEUI
#define SMTC_MODEM_DM_FIELD_APP_STATUS 0x16      //!< application-specific status
#define SMTC_MODEM_DM_FIELD_ALMANAC_STATUS 0x18  //!< almanac status
/**
 * @}
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Modem Return Codes
 * @remark Command output values must not be read if the return code differs from SMTC_MODEM_RC_OK
 */
typedef enum smtc_modem_return_code_e
{
    SMTC_MODEM_RC_OK                  = 0x00,  //!< command executed without errors
    SMTC_MODEM_RC_NOT_INIT            = 0x03,  //!< command not initialized
    SMTC_MODEM_RC_INVALID             = 0x04,  //!< command parameters invalid
    SMTC_MODEM_RC_BUSY                = 0x05,  //!< command cannot be executed now
    SMTC_MODEM_RC_FAIL                = 0x06,  //!< command execution failed
    SMTC_MODEM_RC_BAD_SIZE            = 0x0A,  //!< size check failed
    SMTC_MODEM_RC_MODEM_E_FRAME_ERROR = 0x0F,  //!< serial port framing error (Modem-E only)
    SMTC_MODEM_RC_NO_TIME             = 0x10,  //!< no time available
    SMTC_MODEM_RC_INVALID_STACK_ID    = 0x11,  //!< invalid stack_id parameter
} smtc_modem_return_code_t;

/**
 * @brief Modem Datarate Profiles
 */
typedef enum smtc_modem_adr_profile_e
{
    SMTC_MODEM_ADR_PROFILE_NETWORK_CONTROLLED = 0x00,  //!< Network Server controlled for static devices
    SMTC_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE  = 0x01,  //!< Long range distribution for mobile devices
    SMTC_MODEM_ADR_PROFILE_MOBILE_LOW_POWER   = 0x02,  //!< Low power distribution for mobile devices
    SMTC_MODEM_ADR_PROFILE_CUSTOM             = 0x03,  //!< User defined distribution
} smtc_modem_adr_profile_t;

/**
 * @brief Modem status
 */
enum smtc_modem_status_mask_e
{
    SMTC_MODEM_STATUS_BROWNOUT = ( 1 << 0 ),
    SMTC_MODEM_STATUS_CRASH    = ( 1 << 1 ),
    SMTC_MODEM_STATUS_MUTE     = ( 1 << 2 ),
    SMTC_MODEM_STATUS_JOINED   = ( 1 << 3 ),
    SMTC_MODEM_STATUS_SUSPEND  = ( 1 << 4 ),
    SMTC_MODEM_STATUS_UPLOAD   = ( 1 << 5 ),
    SMTC_MODEM_STATUS_JOINING  = ( 1 << 6 ),
    SMTC_MODEM_STATUS_STREAM   = ( 1 << 7 ),
};

/**
 * @brief   Modem Status masks
 */
typedef uint32_t smtc_modem_status_mask_t;

/**
 * @brief Modem class enumeration
 */
typedef enum smtc_modem_class_e
{
    SMTC_MODEM_CLASS_A = 0x00,  //!< Modem class A
    SMTC_MODEM_CLASS_B = 0x01,  //!< Modem class B
    SMTC_MODEM_CLASS_C = 0x02,  //!< Modem class C
} smtc_modem_class_t;

/**
 * @brief Cipher mode for file upload service
 */
typedef enum smtc_modem_file_upload_cipher_mode_e
{
    SMTC_MODEM_FILE_UPLOAD_NO_CIPHER,         //!< Do not encrypt file
    SMTC_MODEM_FILE_UPLOAD_AES_WITH_APPSKEY,  //!< Encrypt file using AES with appskey
} smtc_modem_file_upload_cipher_mode_t;

/**
 * @brief Cipher mode for stream service
 */
typedef enum smtc_modem_stream_cipher_mode_e
{
    SMTC_MODEM_STREAM_NO_CIPHER,         //!< Do not encrypt stream
    SMTC_MODEM_STREAM_AES_WITH_APPSKEY,  //!< Encrypt stream using AES with appskey
} smtc_modem_stream_cipher_mode_t;

/**
 * @brief Modem firmware version structure definition
 */
typedef struct smtc_modem_version_s
{
    uint8_t major;  //!< Major value
    uint8_t minor;  //!< Minor value
    uint8_t patch;  //!< Patch value
} smtc_modem_version_t;

/**
 * @brief Modem LoRaWan version structure definition
 */
typedef struct smtc_modem_lorawan_version_s
{
    uint8_t major;     //!< Major value
    uint8_t minor;     //!< Minor value
    uint8_t patch;     //!< Patch value
    uint8_t revision;  //!< Revision value
} smtc_modem_lorawan_version_t;

/**
 * @brief DM uplink reporting internal format
 */
typedef enum smtc_modem_dm_info_interval_format_e
{
    SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND = 0x00,
    SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY    = 0x01,
    SMTC_MODEM_DM_INFO_INTERVAL_IN_HOUR   = 0x02,
    SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE = 0x03,
} smtc_modem_dm_info_interval_format_t;

/**
 * @brief Modem region ID
 */
typedef enum smtc_modem_region_e
{
    SMTC_MODEM_REGION_EU_868        = 1,
    SMTC_MODEM_REGION_AS_923_GRP1   = 2,
    SMTC_MODEM_REGION_US_915        = 3,
    SMTC_MODEM_REGION_AU_915        = 4,
    SMTC_MODEM_REGION_CN_470        = 5,
    SMTC_MODEM_REGION_WW2G4         = 6,
    SMTC_MODEM_REGION_AS_923_GRP2   = 7,
    SMTC_MODEM_REGION_AS_923_GRP3   = 8,
    SMTC_MODEM_REGION_IN_865        = 9,
    SMTC_MODEM_REGION_KR_920        = 10,
    SMTC_MODEM_REGION_RU_864        = 11,
    SMTC_MODEM_REGION_CN_470_RP_1_0 = 12,
    SMTC_MODEM_REGION_AS_923_GRP4   = 13,
} smtc_modem_region_t;

/**
 * @brief Multicast group identifier
 */
typedef enum smtc_modem_mc_grp_id_e
{
    SMTC_MODEM_MC_GRP_0,
    SMTC_MODEM_MC_GRP_1,
    SMTC_MODEM_MC_GRP_2,
    SMTC_MODEM_MC_GRP_3,
} smtc_modem_mc_grp_id_t;

/**
 * @brief Stack states
 */
typedef enum smtc_modem_stack_state_e
{
    SMTC_MODEM_STACK_STATE_IDLE,     //!< The stack is idle
    SMTC_MODEM_STACK_STATE_BUSY,     //!< A process is currently running in the stack
    SMTC_MODEM_STACK_STATE_TX_WAIT,  //!< The stack is currently waiting before sending a new uplink (for nb trans or
                                     //!< network command answer)
} smtc_modem_stack_state_t;

/**
 * @brief Rx window returned by the DOWNDATA event
 */
typedef enum smtc_modem_event_downdata_window_e
{
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1         = 0x01,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2         = 0x02,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC         = 0x03,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC_MC_GRP0 = 0x04,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC_MC_GRP1 = 0x05,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC_MC_GRP2 = 0x06,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC_MC_GRP3 = 0x07,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXB         = 0x08,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXB_MC_GRP0 = 0x09,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXB_MC_GRP1 = 0x0A,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXB_MC_GRP2 = 0x0B,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXB_MC_GRP3 = 0x0C,
    SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXBEACON    = 0x0D,
} smtc_modem_event_downdata_window_t;

/**
 * @brief Clock Synchronization services
 */
typedef enum smtc_modem_time_sync_service_e
{
    SMTC_MODEM_TIME_MAC_SYNC = 0,
    SMTC_MODEM_TIME_ALC_SYNC = 1,
} smtc_modem_time_sync_service_t;

/**
 * @brief Status returned by the TIME event
 */
typedef enum smtc_modem_event_time_status_e
{
    SMTC_MODEM_EVENT_TIME_NOT_VALID          = 0,
    SMTC_MODEM_EVENT_TIME_VALID              = 1,
    SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC = 2,
} smtc_modem_event_time_status_t;

typedef enum smtc_modem_event_link_check_status_e
{
    SMTC_MODEM_EVENT_LINK_CHECK_NOT_RECEIVED = 0,
    SMTC_MODEM_EVENT_LINK_CHECK_RECEIVED     = 1,
} smtc_modem_event_link_check_status_t;

/**
 * @brief Status returned by the Tx done event
 */
typedef enum smtc_modem_event_txdone_status_e
{
    SMTC_MODEM_EVENT_TXDONE_NOT_SENT  = 0,
    SMTC_MODEM_EVENT_TXDONE_SENT      = 1,
    SMTC_MODEM_EVENT_TXDONE_CONFIRMED = 2,
} smtc_modem_event_txdone_status_t;

/**
 * @brief Status returned by the MUTE event
 */
typedef enum smtc_modem_event_mute_status_e
{
    SMTC_MODEM_EVENT_MUTE_OFF = 0,
    SMTC_MODEM_EVENT_MUTE_ON  = 1,
} smtc_modem_event_mute_status_t;

/**
 * @brief Status returned by the UPLOADDONE event
 */
typedef enum smtc_modem_event_uploaddone_status_e
{
    SMTC_MODEM_EVENT_UPLOADDONE_ABORTED    = 0,
    SMTC_MODEM_EVENT_UPLOADDONE_SUCCESSFUL = 1,
} smtc_modem_event_uploaddone_status_t;

/**
 * @brief Tag returned by the SETCONF event
 */
typedef enum smtc_modem_event_setconf_e
{
    SMTC_MODEM_EVENT_SETCONF_ADR_MODE_UPDATED    = SMTC_MODEM_DM_FIELD_ADR_MODE,
    SMTC_MODEM_EVENT_SETCONF_JOIN_EUI_UPDATED    = SMTC_MODEM_DM_FIELD_JOIN_EUI,
    SMTC_MODEM_EVENT_SETCONF_DM_INTERVAL_UPDATED = SMTC_MODEM_DM_FIELD_INTERVAL,
} smtc_modem_event_setconf_tag_t;

/**
 * @brief Status returned by the ALMANAC_UPDATE event
 */
typedef enum smtc_modem_event_almanac_update_status_e
{
    SMTC_MODEM_EVENT_ALMANAC_UPDATE_COMPLETED        = 0,
    SMTC_MODEM_EVENT_ALMANAC_UPDATE_STATUS_REQUESTED = 1,
} smtc_modem_event_almanac_update_status_t;

/**
 * @brief Status returned by the SMTC_MODEM_EVENT_CLASS_B_STATUS
 *
 */
typedef enum smtc_modem_event_class_b_status_e
{
    SMTC_MODEM_EVENT_CLASS_B_NOT_READY = 0,
    SMTC_MODEM_EVENT_CLASS_B_READY     = 1,
} smtc_modem_event_class_b_status_t;

/**
 * @brief Status returned by the SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO
 *
 */
typedef enum smtc_modem_event_class_b_ping_slot_status_e
{
    SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_NOT_ANSWERED = 0,
    SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_ANSWERED     = 1,
} smtc_modem_event_class_b_ping_slot_status_t;

/**
 * @brief Status returned by USER_RADIO_ACCESS event
 */
typedef enum
{
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_RX_ERROR       = 0,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_CAD_OK         = 1,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_CAD_DONE       = 2,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_TX_DONE        = 3,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_RX_DONE        = 4,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_RX_TIMEOUT     = 5,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_WIFI_SCAN_DONE = 6,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_GNSS_SCAN_DONE = 7,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_ABORTED        = 8,
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_UNKNOWN        = 9,
} smtc_modem_event_user_radio_access_status_t;

/**
 * @brief Periodicity available for Class B ping slot
 */
typedef enum smtc_modem_class_b_ping_slot_periodicity_e
{
    SMTC_MODEM_CLASS_B_PINGSLOT_1_S = 0,
    SMTC_MODEM_CLASS_B_PINGSLOT_2_S,
    SMTC_MODEM_CLASS_B_PINGSLOT_4_S,
    SMTC_MODEM_CLASS_B_PINGSLOT_8_S,
    SMTC_MODEM_CLASS_B_PINGSLOT_16_S,
    SMTC_MODEM_CLASS_B_PINGSLOT_32_S,
    SMTC_MODEM_CLASS_B_PINGSLOT_64_S,
    SMTC_MODEM_CLASS_B_PINGSLOT_128_S,
} smtc_modem_class_b_ping_slot_periodicity_t;

/**
 * @brief Frame pending status
 */
typedef enum smtc_modem_frame_pending_bit_status_e
{
    SMTC_MODEM_NO_DATA_ARE_PENDING = 0,
    SMTC_MODEM_DATA_ARE_PENDING,
} smtc_modem_frame_pending_bit_status_t;

/**
 * @brief Status returned by the SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE event
 */
typedef enum smtc_modem_d2d_class_b_tx_done_status_e
{
    SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE_NOT_SENT = 0,
    SMTC_MODEM_EVENT_D2D_CLASS_B_TX_DONE_SENT     = 1,
} smtc_modem_d2d_class_b_tx_done_status_t;

/**
 * @brief Structure holding event-related data
 */
typedef struct smtc_modem_event_s
{
    uint8_t stack_id;
    uint8_t event_type;
    uint8_t missed_events;  //!< Number of event_type events missed before the current one
    union
    {
        struct
        {
            uint16_t count;
        } reset;
        struct
        {
            smtc_modem_event_txdone_status_t status;
        } txdone;
        struct
        {
            int8_t                             rssi;  //!< Signed value in dBm + 64
            int8_t                             snr;   //!< Signed value in dB given in 0.25dB step
            smtc_modem_event_downdata_window_t window;
            uint8_t                            fport;
            uint8_t                            data[SMTC_MODEM_MAX_DOWNLINK_LENGTH];
            uint16_t                           length;
            uint8_t                            fpending_bit;
            uint32_t                           frequency_hz;
            uint8_t                            datarate;
        } downdata;
        struct
        {
            smtc_modem_event_uploaddone_status_t status;
        } uploaddone;
        struct
        {
            smtc_modem_event_setconf_tag_t tag;
        } setconf;
        struct
        {
            smtc_modem_event_mute_status_t status;
        } mute;
        struct
        {
            smtc_modem_event_time_status_t status;
        } time;
        struct
        {
            smtc_modem_event_link_check_status_t status;
            uint8_t
                margin;  //!< The demodulation margin indicates the link margin in dB of the most recently transmitted
                         //!< LinkCheckReq command. A value of 0 means that the frame was received at the demodulation
                         //!< floor (0 dB or no margin) whereas a value of 20, for example, means that the frame reached
                         //!< the best gateway 20 dB above the demodulation floor. The value 255 is reserved.
            uint8_t gw_cnt;  //!< The gateway count ( GwCnt ) is the number of gateways that received the most recent
                             //!< LinkCheckReq command
        } link_check;
        struct
        {
            smtc_modem_event_almanac_update_status_t status;
        } almanac_update;
        struct
        {
            smtc_modem_event_user_radio_access_status_t status;
            uint32_t                                    timestamp_ms;
        } user_radio_access;
        struct
        {
            smtc_modem_event_class_b_ping_slot_status_t status;
        } class_b_ping_slot_info;
        struct
        {
            smtc_modem_event_class_b_status_t status;
        } class_b_status;
        struct
        {
            smtc_modem_d2d_class_b_tx_done_status_t status;
            smtc_modem_mc_grp_id_t mc_grp_id;  //!< The multicast group id on which the uplink was requested
            uint8_t                nb_trans_not_send;
        } d2d_class_b_tx_done;
        struct
        {
            uint8_t status;
        } middleware_event_status;
    } event_data;
} smtc_modem_event_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Get the modem event
 *
 * @remark This command can be used to retrieve pending events from the modem.
 *
 * @param [out] event                   Structure holding event-related information
 * @param [out] event_pending_count     Number of pending event(s)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p event or \p event_pending_count are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */

smtc_modem_return_code_t smtc_modem_get_event( smtc_modem_event_t* event, uint8_t* event_pending_count );

/**
 * @brief Get the modem firmware version
 *
 * @param [out] firmware_version Firmware version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p firmware_version is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_modem_version( smtc_modem_version_t* firmware_version );

/**
 * @brief Get the LoRaWAN stack version
 *
 * @param [out] lorawan_version  LoRaWAN version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p lorawan_version is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_lorawan_version( smtc_modem_lorawan_version_t* lorawan_version );

/**
 * @brief Get the stack Regional Parameters version
 *
 * @param [out] regional_params_version Stack regional parameters version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p regional_params_version is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_regional_params_version(
    smtc_modem_lorawan_version_t* regional_params_version );

/**
 * @brief Reset the modem
 *
 * @remark Resets modem transient state (including session information) by reseting the MCU.
 *         Device Management Port, Modem Region and LoRaWan DevNonce are kept
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_reset( void );

/**
 * @brief Reset the modem to its original state
 *
 * @remark Resets all modem-related non-volatile settings to their default values, then reset the MCU.
 *         Only LoRaWAN DevNonce is kept
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_factory_reset( void );

/**
 * @brief Get the total charge counter of the modem in mAh
 *
 * @param [out] charge_mah Accumulated charge in mAh
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p charge_mah is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_charge( uint32_t* charge_mah );

/**
 * @brief Reset the total charge counter of the modem
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_reset_charge( void );

/**
 * @brief Get the Tx power offset in dB
 *
 * @param [in]  stack_id         Stack identifier
 * @param [out] tx_pwr_offset_db Tx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p tx_pwr_offset_db is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_tx_power_offset_db( uint8_t stack_id, int8_t* tx_pwr_offset_db );

/**
 * @brief Set the Tx power offset in dB
 *
 * @param [in] stack_id         Stack identifier
 * @param [in] tx_pwr_offset_db Tx power offset in dB to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p tx_pwr_offset_db is out of [-30:30] range
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_tx_power_offset_db( uint8_t stack_id, int8_t tx_pwr_offset_db );

/**
 * @brief Start a chosen time synchronization service
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] sync_service Time synchronization service to use
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              A time synchronization service is already running
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_time_start_sync_service( uint8_t                        stack_id,
                                                             smtc_modem_time_sync_service_t sync_service );

/**
 * @brief Stop current time synchronization service
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              No time synchronization service is running
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_time_stop_sync_service( uint8_t stack_id );

/**
 * @brief Get GPS epoch time - number of seconds elapsed since GPS epoch (00:00:00, Sunday 6th of January 1980).
 *
 * @param [out] gps_time_s       GPS time in seconds
 * @param [out] gps_fractional_s GPS fractional second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p gps_time_s or \p gps_fractional_s are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_NO_TIME       No time available
 */
smtc_modem_return_code_t smtc_modem_get_time( uint32_t* gps_time_s, uint32_t* gps_fractional_s );

/**
 * @brief Trigger a single uplink requesting time using current enabled time synchronization service
 *
 * @param [in] stack_id     Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined) or no time
 *                                         synchronization service is running
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_time_trigger_sync_request( uint8_t stack_id );

/**
 * @brief Set ALCSync service LoRaWAN FPort
 *
 * @remark When using Device Management (DM) port for \p alcsync_fport, ALCsync messages are encapsulated into DM frames
 *
 * @param [in] alcsync_fport LoRaWAN FPort for ALCSync messages
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p alcsync_fport is invalid: out of [0:223] range
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_set_alcsync_fport( uint8_t alcsync_fport );

/**
 * @brief Get ALCSync service LoRaWAN FPort
 *
 * @param [out] alcsync_fport FPort for ALCsync messages
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p alcsync_fport is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_get_alcsync_fport( uint8_t* alcsync_fport );

/**
 * @brief Set the interval between two time synchronization messages
 *
 * @remark \p sync_interval_s has to be lower than the value set with @ref smtc_modem_time_set_sync_invalid_delay_s
 * @remark The default value is set to 36 hours (129600 seconds)
 *
 * @param [in] sync_interval_s Interval in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p sync_interval_s is invalid
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_set_sync_interval_s( uint32_t sync_interval_s );

/**
 * @brief Get the interval between time synchronization messages
 *
 * @param [out] sync_interval_s Interval in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p sync_interval_s is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_get_sync_interval_s( uint32_t* sync_interval_s );

/**
 * @brief Set the delay beyond which the time synchronization is no longer considered valid by the modem
 *
 * @remark \p sync_invalid_delay_s has to be higher than the value set with @ref smtc_modem_time_set_sync_interval_s
 * @remark The default value is set to 49 days (4233600 seconds)
 * @remark Modem will generate a SMTC_MODEM_EVENT_TIME event if the there are no time synchronizations for more
 *         time than this "invalid delay".
 *
 * @param [in] sync_invalid_delay_s Invalid delay in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p sync_invalid_delay_s" is higher than 49 days
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_set_sync_invalid_delay_s( uint32_t sync_invalid_delay_s );

/**
 * @brief Get the configured delay beyond which the time synchronization is no longer valid
 *
 * @param [out] sync_invalid_delay_s Invalid delay in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p sync_invalid_delay_s is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_get_sync_invalid_delay_s( uint32_t* sync_invalid_delay_s );

/**
 * @brief Get the modem status
 *
 * @param [in]  stack_id    Stack identifier
 * @param [out] status_mask Modem status (see @ref smtc_modem_status_mask_e)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p status_mask is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_status( uint8_t stack_id, smtc_modem_status_mask_t* status_mask );

/**
 * @brief Set and start the alarm timer (up to 864000s ie 10 days)
 *
 * @remark When the timer expires, an alarm event is generated
 *
 * @param [in] alarm_timer_in_s Alarm timer in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p alarm_timer_in_s exceed max value of 864000s (10 days)
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_alarm_start_timer( uint32_t alarm_timer_in_s );

/**
 * @brief Stop and clear the alarm timer
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT      No alarm timer currently running
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_alarm_clear_timer( void );

/**
 * @brief Get the number of seconds remaining before the alarm triggers an event
 *
 * @param [out] remaining_time_in_s Number of seconds remaining before the alarm triggers an event
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT      No alarm timer currently running
 * @retval SMTC_MODEM_RC_INVALID       \p remaining_time_in_s is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_alarm_get_remaining_time( uint32_t* remaining_time_in_s );

/**
 * @brief Get the JoinEUI
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] joineui  Current JoinEUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_joineui( uint8_t stack_id, uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the JoinEUI
 *
 * @param [in]  stack_id Stack identifier
 * @param [in]  joineui  JoinEUI to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Get the DevEUI
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] deveui   Current DevEUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the DevEUI
 *
 * @param [in]  stack_id Stack identifier
 * @param [in]  deveui   DevEUI to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the LoRaWAN v1.1.x Network Key (aka Application Key in LoRaWAN v1.0.x)
 *
 * @param [in] stack_id Stack identifier
 * @param [in] nwkkey   Key to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] );

/**
 * @brief Get the current LoRaWAN class
 *
 * @param [in]  stack_id      Stack identifier
 * @param [out] lorawan_class Current LoRaWAN class
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_class( uint8_t stack_id, smtc_modem_class_t* lorawan_class );

/**
 * @brief Set the LoRaWAN class
 *
 * @param [in] stack_id      Stack identifier
 * @param [in] lorawan_class LoRaWAN class to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           LoRaWAN class is not in an acceptable range
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              For Class B only: no time is available or modem is not joined
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class );

/**
 * @brief Configure a multicast group
 *
 * @param [in] stack_id     Stack identifier
 * @param [in] mc_grp_id    Multicast group identifier
 * @param [in] mc_grp_addr  Multicast group address
 * @param [in] mc_nwk_skey  Multicast network session key for the group
 * @param [in] mc_app_skey  Multicast application session key for the group
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3]
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Error during crypto process or a running session already exists on this id
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_set_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t      mc_grp_addr,
                                                              const uint8_t mc_nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                              const uint8_t mc_app_skey[SMTC_MODEM_KEY_LENGTH] );

/**
 * @brief Get the configuration of the chosen multicast group
 *
 * @param [in]  stack_id     Stack identifier
 * @param [in]  mc_grp_id    Multicast group identifier
 * @param [out] mc_grp_addr  Multicast group address
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3] or \p mc_grp_addr is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_get_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t* mc_grp_addr );

/**
 * @brief Start class C multicast session for a specific group
 *
 * @param [in] stack_id   Stack identifier
 * @param [in] mc_grp_id  Multicast group identifier
 * @param [in] freq       Downlink frequency in Hz for this session
 * @param [in] dr         Downlink datarate for this session
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3]
 *                              Frequency or Datarate are not in acceptable range (according to current regional params)
 *                              Frequency or Datarate are not compatible with an already running multicast session
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This session is already started or modem is not in class C
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_c_start_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                     uint32_t freq, uint8_t dr );

/**
 * @brief Get class C multicast session status for a chosen group
 *
 * @param [in]  stack_id            Stack identifier
 * @param [in]  mc_grp_id           Multicast group identifier
 * @param [out] is_session_started  Session status
 * @param [out] freq                Downlink frequency in Hz for this session
 * @param [out] dr                  Downlink datarate for this session
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3] or a parameter is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_c_get_session_status( uint8_t                stack_id,
                                                                          smtc_modem_mc_grp_id_t mc_grp_id,
                                                                          bool* is_session_started, uint32_t* freq,
                                                                          uint8_t* dr );

/**
 * @brief Stop class C multicast session for a chosen group
 *
 * @param [in] stack_id   Stack identifier
 * @param [in] mc_grp_id  Multicast group identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3]
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_c_stop_session( uint8_t                stack_id,
                                                                    smtc_modem_mc_grp_id_t mc_grp_id );

/**
 * @brief Stop all started class C multicast sessions
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_c_stop_all_sessions( uint8_t stack_id );

/**
 * @brief Start class B multicast session for a specific group
 *
 * @param [in] stack_id              Stack identifier
 * @param [in] mc_grp_id             Multicast group identifier
 * @param [in] freq                  Downlink frequency for this session
 * @param [in] dr                    Downlink datarate for this session
 * @param [in] ping_slot_periodicity Ping slot periodicity for this session
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3]
 *                              Frequency or Datarate are not in acceptable range (according to current regional params)
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              This session is already started or modem is not in class B
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_b_start_session(
    uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id, uint32_t freq, uint8_t dr,
    smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity );

/**
 * @brief Get class B multicast session status for a chosen group
 *
 * @param [in]  stack_id                      Stack identifier
 * @param [in]  mc_grp_id                     Multicast group identifier
 * @param [out] is_session_started            Session status
 * @param [out] is_session_waiting_for_beacon Session beacon waiting status
 * @param [out] dr                            Session downlink datarate
 * @param [out] freq                          Session downlink frequency
 * @param [out] ping_slot_periodicity         Session ping slot periodicity
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3] or a parameter is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_b_get_session_status(
    uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id, bool* is_session_started, bool* is_session_waiting_for_beacon,
    uint32_t* freq, uint8_t* dr, smtc_modem_class_b_ping_slot_periodicity_t* ping_slot_periodicity );

/**
 * @brief Stop class B multicast session for a chosen group
 *
 * @param [in] stack_id   Stack identifier
 * @param [in] mc_grp_id  Multicast group identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mc_grp_id is not in the range [0:3]
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_b_stop_session( uint8_t                stack_id,
                                                                    smtc_modem_mc_grp_id_t mc_grp_id );

/**
 * @brief Stop all started class B multicast sessions
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_multicast_class_b_stop_all_sessions( uint8_t stack_id );

/**
 * @brief Get the current LoRaWAN region
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] region   Current LoRaWAN region
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p region is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_region( uint8_t stack_id, smtc_modem_region_t* region );

/**
 * @brief Set the LoRaWAN region
 *
 * @param [in]  stack_id Stack identifier
 * @param [in]  region   LoRaWAN region to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p region is not supported
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_region( uint8_t stack_id, smtc_modem_region_t region );

/**
 * @brief Get the current adaptative data rate (ADR) profile
 *
 * @param [in]  stack_id    Stack identifier
 * @param [out] adr_profile Current ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter \p adr_profile is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_adr_get_profile( uint8_t stack_id, smtc_modem_adr_profile_t* adr_profile );

/**
 * @brief Set the adaptative data rate (ADR) profile
 *
 * @remark If @ref SMTC_MODEM_ADR_PROFILE_CUSTOM is selected, custom data are taken into account
 *
 * @param [in] stack_id        Stack identifier
 * @param [in] adr_profile     ADR profile to be configured
 * @param [in] adr_custom_data Definition of the custom ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           One or more invalid parameters
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_adr_set_profile( uint8_t stack_id, smtc_modem_adr_profile_t adr_profile,
                                                     const uint8_t adr_custom_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] );

/**
 * @brief Get the current available Datarate in regards of Uplink ChMash and DwellTime
 *
 * @param [in]  stack_id                 The stack identifier
 * @param [out] available_datarates_mask The available datarates, described in a bitfield
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p available_datarates_mask is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_available_datarates( uint8_t stack_id, uint16_t* available_datarates_mask );

/**
 * @brief Get the Device Management (DM) LoRaWAN FPort
 *
 * @param [out] dm_fport LoRaWAN FPort on which the DM info is sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p dm_fport is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_fport( uint8_t* dm_fport );

/**
 * @brief Set the Device Management (DM) LoRaWAN FPort
 *
 * @param [in] dm_fport LoRaWAN FPort on which the DM info is sent. This value must be in the range [1:223]
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p dm_fport is out of the [1:223] range
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_fport( uint8_t dm_fport );

/**
 * @brief Get the interval between two Device Management (DM) info field messages
 *
 * @param [out] format   Reporting interval format
 * @param [out] interval Interval in unit defined in format
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameters \p format and/or \p interval are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_info_interval( smtc_modem_dm_info_interval_format_t* format,
                                                          uint8_t*                              interval );

/**
 * @brief Set the interval between two Device Management (DM) info field messages
 *
 * @remark An interval value set to 0 disables the feature - no matter the format.
 *
 * @param [in] format   Reporting interval format
 * @param [in] interval Interval in unit defined in format, from 0 to 63
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p interval is not in the [0:63] range.
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_info_interval( smtc_modem_dm_info_interval_format_t format,
                                                          uint8_t                              interval );

/**
 * @brief Get the Device Management (DM) info fields
 *
 * @param [out] dm_fields_payload DM info fields (see @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [out] dm_field_length   DM info field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameters \p dm_fields_payload and/or \p dm_field_length are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length );

/**
 * @brief Set the Device Management (DM) info fields to be sent on a regular basis
 *
 * @remark The interval between two DM info field messages is defined with @ref smtc_modem_dm_set_info_interval
 *
 * @param [in] dm_fields_payload DM info fields (see @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length   DM info field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Invalid or duplicated DM info fields
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_info_fields( const uint8_t* dm_fields_payload, uint8_t dm_field_length );

/**
 * @brief Request an immediate Device Management (DM) status
 *
 * @remark The content is independent from the configuration set with @ref smtc_modem_dm_set_info_fields
 *
 * @param [in] dm_fields_payload DM info fields (see @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length   DM info field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Invalid or duplicated field code or \p dm_fields_payload is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          Modem is not available (suspended, muted, or not joined)
 */
smtc_modem_return_code_t smtc_modem_dm_request_single_uplink( const uint8_t* dm_fields_payload,
                                                              uint8_t        dm_field_length );

/**
 * @brief Set user-specific data to be reported by Device Management (DM) frames
 *
 * @remark This field will be sent only if it is selected in @ref smtc_modem_dm_set_info_fields or @ref
 * smtc_modem_dm_request_single_uplink
 *
 * @param [in] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_user_data( const uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Get user-specific data to be reported by Device Management (DM) frames
 *
 * @param [out] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p user_data is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_user_data( uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Join the network
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors or modem has already joined the network
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended or muted)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id );

/**
 * @brief Leave an already joined network or cancels on ongoing join process
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_leave_network( uint8_t stack_id );

/**
 * @brief Suspend the radio communications initiated by the modem
 *
 * @param [in] suspend The configuration to be applied (true: suspend communications / false: resume communications)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_suspend_radio_communications( bool suspend );

/**
 * @brief Get the maximum payload size that can be used for the next uplink
 *
 * @remark This value depends on the LoRaWAN regional parameters for the next transmission using the current data rate
 *
 * @param [in]  stack_id            Stack identifier
 * @param [out] tx_max_payload_size Maximum payload size in byte
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters \p tx_max_payload_size is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem has not joined a network
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size );

/**
 * @brief Request a LoRaWAN uplink
 *
 * @remark LoRaWAN NbTrans parameter can be set in mobile and custom ADR modes with @ref smtc_modem_set_nb_trans
 *
 * @param [in] stack_id       Stack identifier
 * @param [in] fport          LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed      Message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        Data to be sent
 * @param [in] payload_length Number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p fport is out of the [1:223] range or equal to the DM LoRaWAN FPort
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_request_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Request an immediate LoRaWAN uplink
 *
 * @remark It has higher priority than all other services and is not subject to duty cycle restrictions, if any
 * @remark LoRaWAN NbTrans parameter can be set in mobiles and custom ADR modes with @ref smtc_modem_set_nb_trans
 *
 * @param [in] stack_id       Stack identifier
 * @param [in] fport          LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed      Message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        Data to be sent
 * @param [in] payload_length Number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p fport is out of the [1:223] range or equal to the DM LoRaWAN FPort
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Request a LoRaWAN uplink without payload, and an optional FPort
 *
 * @remark It can be used to create downlink opportunities / heartbeat without routing messages to an application server
 *
 * @param [in] stack_id       Stack identifier
 * @param [in] send_fport     Add the FPort to the payload (true: add the FPort, false: send without FPort)
 * @param [in] fport          LoRaWAN FPort on which the uplink is done, if used
 * @param [in] confirmed      Message type (true: confirmed, false: unconfirmed)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p fport is out of the [1:223] range or equal to the DM LoRaWAN FPort
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_request_empty_uplink( uint8_t stack_id, bool send_fport, uint8_t fport,
                                                          bool confirmed );

/**
 * @brief Create and initialize a file upload session
 *
 * @param [in] stack_id        Stack identifier
 * @param [in] index           Index on which the upload is done
 * @param [in] cipher_mode     Cipher mode
 * @param [in] file            File buffer
 * @param [in] file_length     File size in bytes
 * @param [in] average_delay_s Minimum delay between two file upload fragments in seconds (from the end of an uplink to
 *                             the start of the next one)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p file_length is equal to 0 or greater than 8180 bytes, or \p file is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode, or a file upload is already ongoing
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_file_upload_init( uint8_t stack_id, uint8_t index,
                                                      smtc_modem_file_upload_cipher_mode_t cipher_mode,
                                                      const uint8_t* file, uint16_t file_length,
                                                      uint32_t average_delay_s );

/**
 * @brief Start the file upload session
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode, or a file upload is already ongoing
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_BAD_SIZE          Total data sent does not match the declared Size value in @ref
 *                                         smtc_modem_file_upload_init()
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_file_upload_start( uint8_t stack_id );

/**
 * @brief Reset the file upload session
 *
 * @remark This function will stop any ongoing file upload session
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT          No file upload session currently running
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_file_upload_reset( uint8_t stack_id );

/**
 * @brief Create and initialize a data stream
 *
 * @param [in] stack_id                  Stack identifier
 * @param [in] fport                     LoRaWAN FPort on which the stream is sent (0 forces the DM LoRaWAN FPort)
 * @param [in] redundancy_ratio_percent  Stream redundancy ratio
 * @param [in] cipher_mode               Cipher mode
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           FPort is out of the [0:223] range
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or the streaming buffer is not empty
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t fport,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode,
                                                 uint8_t                         redundancy_ratio_percent );

/**
 * @brief Add data to the stream
 *
 * @remark If @ref smtc_modem_stream_init is not called beforehand, the stream uses the DM FPort with a redundancy ratio
 * set to 110%
 *
 * @param [in] stack_id Stack identifier
 * @param [in] data     Data to be added to the stream
 * @param [in] len      Number of bytes from data to be added to the stream
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p len is not in range [1-254] or \p data is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or the streaming buffer is full
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len );

/**
 * @brief Return the current stream status
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] pending  Length of pending data for transmission
 * @param [out] free     Length of free space in the buffer
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT          No stream session is running
 * @retval SMTC_MODEM_RC_INVALID           \p pending or \p free are NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free );

/**
 * @brief Enable / disable the certification mode
 *
 * @param [in] stack_id Stack identifier
 * @param [in] enable   Certification mode state (default: disabled)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable );

/**
 * @brief Get the current state of the certification mode
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] enable   Certification mode state
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter \p enable is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_certification_mode( uint8_t stack_id, bool* enable );

/**
 * @brief Set the connection timeout thresholds
 *
 * @remark The value 0 deactivates the command
 * @remark It is recommended to have \p nb_of_uplinks_before_network_controlled smaller than \p
 * nb_of_uplink_before_reset
 *
 * @param [in] stack_id                                Stack identifier
 * @param [in] nb_of_uplinks_before_network_controlled Number of uplinks without downlink before the ADR profile
 *                                                     switches to network-controlled
 * @param [in] nb_of_uplinks_before_reset              Number of uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_connection_timeout_set_thresholds( uint8_t  stack_id,
                                                                       uint16_t nb_of_uplinks_before_network_controlled,
                                                                       uint16_t nb_of_uplinks_before_reset );

/**
 * @brief Get the configured connection timeout thresholds
 *
 * @param [in]  stack_id                                Stack identifier
 * @param [out] nb_of_uplinks_before_network_controlled Number of uplinks without downlink before the ADR profile
 *                                                      switches to network-controlled
 * @param [out] nb_of_uplinks_before_reset              Number of uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters \p nb_of_uplinks_before_network_controlled and/or
 *                                         \p nb_of_uplinks_before_reset are NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_connection_timeout_get_thresholds(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset );

/**
 * @brief Get the current status of the connection timeouts
 *
 * @param [in]  stack_id                                 Stack identifier
 * @param [out] nb_of_uplinks_before_network_controlled  Number of remaining uplinks without downlink before the ADR
 *                                                       profile switches to network-controlled
 * @param [out] nb_of_uplinks_before_reset               Number of remaining uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters \p nb_of_uplinks_before_network_controlled and/or
 *                                         \p nb_of_uplinks_before_reset are NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_connection_timeout_get_current_values(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset );

/**
 * @brief Get the current value of the lost connection counter
 *
 * @remark The counter is incremented after any uplink and is only reset when a valid downlink is received from Network
 * Server
 *
 * @param [in]  stack_id            Stack identifier
 * @param [out] lost_connection_cnt Lost connection counter current value
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p lost_connection_cnt is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lorawan_get_lost_connection_counter( uint8_t   stack_id,
                                                                         uint16_t* lost_connection_cnt );

/**
 * @brief Get the current status of the duty cycle
 *
 * @remark If the returned value is positive, it is the time still available. A negative value indicates the time to
 * wait until band availability
 *
 * @param [out] duty_cycle_status_ms Status of the duty cycle in milliseconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p duty_cycle_status_ms is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( int32_t* duty_cycle_status_ms );

/**
 * @brief Get the current state of the stack
 *
 * @param [in]  stack_id     Stack identifier
 * @param [out] stack_state  Stack current state
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p stack_state is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_stack_state( uint8_t stack_id, smtc_modem_stack_state_t* stack_state );

/**
 * @brief Configure LoRaWAN network type to private or public (default: public)
 *
 * @param [in]  stack_id      Stack identifier
 * @param [in]  network_type  Configuration to be applied (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_network_type( uint8_t stack_id, bool network_type );

/**
 * @brief Get the configured LoRaWAN network type
 *
 * @param [in]  stack_id      Stack identifier
 * @param [out] network_type  Current configuration (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p network_type is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_network_type( uint8_t stack_id, bool* network_type );

/**
 * @brief Set the parameters of the Listen Before Talk (LBT) feature
 *
 * @param [in]  stack_id               Stack identifier
 * @param [in]  listening_duration_ms  Listening duration in ms to be configured
 * @param [in]  threshold_dbm          LBT threshold in dbm to be configured
 * @param [in]  bw_hz                  LBT bandwith in Hertz to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_set_parameters( uint8_t stack_id, uint32_t listening_duration_ms,
                                                        int16_t threshold_dbm, uint32_t bw_hz );

/**
 * @brief Get the parameters of the Listen Before Talk (LBT) feature
 *
 * @param [in]  stack_id               Stack identifier
 * @param [out] listening_duration_ms  Current listening duration in ms
 * @param [out] threshold_dbm          Current LBT threshold in dbm
 * @param [out] bw_hz                  Current LBT bandwith in Hertz
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           At least one parameter is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_get_parameters( uint8_t stack_id, uint32_t* listening_duration_ms,
                                                        int16_t* threshold_dbm, uint32_t* bw_hz );

/**
 * @brief Enable or disable the Listen Before Talk (LBT) feature
 *
 * @remark The configuration function @ref smtc_modem_lbt_set_parameters must be called before enabling the LBT feature.
 * @remark LBT is silently enabled if the feature is mandatory in a region selected with @ref smtc_modem_set_region
 *
 * @param [in] stack_id  Stack identifier
 * @param [in] enable    Status of the LBT feature to set (true: enable, false: disable)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_set_state( uint8_t stack_id, bool enable );

/**
 * @brief Get the state of the Listen Before Talk (LBT) feature
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] enabled   Current status of the LBT feature (true: enabled, false: disabled)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p enabled is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_get_state( uint8_t stack_id, bool* enabled );

/**
 * @brief Set the number of transmissions in case of unconfirmed uplink
 *
 * @param [in]  stack_id  Stack identifier
 * @param [in]  nb_trans  Number of transmissions ( 0 < value < 16 )
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p nb_trans is not in the [1:15] range or ADR profile is network-controlled
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_nb_trans( uint8_t stack_id, uint8_t nb_trans );

/**
 * @brief Get the configured number of transmissions in case of unconfirmed uplink
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] nb_trans  Number of transmissions
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p nb_trans is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_nb_trans( uint8_t stack_id, uint8_t* nb_trans );

/**
 * @brief Set modem crystal error
 *
 * @param [in] crystal_error_ppm Crystal error in ppm
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_set_crystal_error_ppm( uint32_t crystal_error_ppm );

/**
 * @brief Get the modem crystal error
 *
 * @param [out] crystal_error_ppm Crystal error in ppm
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p crystal_error_ppm is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_crystal_error_ppm( uint32_t* crystal_error_ppm );

/**
 * @brief Request a Link Check Req MAC command to the network
 *
 * @remark The request will be sent in a new uplink frame
 *
 * @param [in]  stack_id  Stack identifier
 *
 * @return smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 *
 */
smtc_modem_return_code_t smtc_modem_lorawan_request_link_check( uint8_t stack_id );

/**
 * @brief Grant user radio access by suspending the modem and kill all current modem radio tasks
 *
 * @remark The user must call this command before performing operations requiring a direct access to the radio (e.g.
 * test modes). Otherwise, undefined behavior may occur.
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_suspend_before_user_radio_access( void );

/**
 * @brief Release user radio access and resume modem features (scheduler and radio access)
 *
 * @remark The user must call this function after performing operations requiring a direct access to the radio (e.g.
 * test modes). Otherwise, all modem-related tasks remain pending.
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_resume_after_user_radio_access( void );

/**
 * @brief Request a Ping Slot Info MAC command to the network
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lorawan_class_b_request_ping_slot_info( uint8_t stack_id );

/**
 * @brief Set Class B Ping Slot Periodicity
 *
 * @param [in] stack_id Stack identifier
 * @param [in] ping_slot_periodicity Ping slot periodicity
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_class_b_set_ping_slot_periodicity(
    uint8_t stack_id, smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity );

/**
 * @brief Get Class B Ping Slot Periodicity
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] ping_slot_periodicity Ping slot periodicity
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID            \p ping_slot_periodicity is NULL
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_class_b_get_ping_slot_periodicity(
    uint8_t stack_id, smtc_modem_class_b_ping_slot_periodicity_t* ping_slot_periodicity );

/**
 * @brief Get network frame pending status
 *
 * @remark This bit is set by the network when data are available and a downlink opportunity is required
 *
 * @param [in] stack_id Stack identifier
 * @param [in] frame_pending_bit_status Frame pending bit status
 *
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID            \p frame_pending_bit_status is NULL
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_network_frame_pending_status(
    uint8_t stack_id, smtc_modem_frame_pending_bit_status_t* frame_pending_bit_status );

/**
 * @brief Set the LoRaWan stack ADR ACK limit and ADR ACK delay regarding the ADR fallback if no downlink are received
 *
 * @param [in] stack_id Stack identifier
 * @param [in] adr_ack_limit ADR ACK limit. Accepted value: ( adr_ack_limit > 1 ) && ( adr_ack_limit < 128 )
 * @param [in] adr_ack_delay ADR ACK delay. Accepted value: ( adr_ack_delay > 1 ) && ( adr_ack_delay < 128 )
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID            \p adr_ack_limit and \p adr_ack_delay are not in the range [2:127]
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_adr_ack_limit_delay( uint8_t stack_id, uint8_t adr_ack_limit,
                                                             uint8_t adr_ack_delay );

/**
 * @brief Get the LoRaWan stack ADR ACK limit and ADR ACK delay configured regarding the ADR fallback if no downlink are
 * received
 *
 * @param [in] stack_id Stack identifier
 * @param [out] adr_ack_limit ADR ACK limit
 * @param [out] adr_ack_delay ADR ACK delay
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID            \p adr_ack_limit or \p adr_ack_delay are NULL
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_adr_ack_limit_delay( uint8_t stack_id, uint8_t* adr_ack_limit,
                                                             uint8_t* adr_ack_delay );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_API_H__

/* --- EOF ------------------------------------------------------------------ */
