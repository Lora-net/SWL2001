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
 * @defgroup SMTC_MODEM_EVENT_DEF Event codes definitions
 * @{
 */
#define SMTC_MODEM_EVENT_RESET 0x00                //!< Modem has been reset
#define SMTC_MODEM_EVENT_ALARM 0x01                //!< Alarm timer expired
#define SMTC_MODEM_EVENT_JOINED 0x02               //!< Network successfully joined
#define SMTC_MODEM_EVENT_TXDONE 0x03               //!< Frame transmitted
#define SMTC_MODEM_EVENT_DOWNDATA 0x04             //!< Downlink data received
#define SMTC_MODEM_EVENT_UPLOADDONE 0x05           //!< File upload completed
#define SMTC_MODEM_EVENT_SETCONF 0x06              //!< Configuration has been changed by the Device Management
#define SMTC_MODEM_EVENT_MUTE 0x07                 //!< Modem has been muted or un-muted by the Device Management
#define SMTC_MODEM_EVENT_STREAMDONE 0x08           //!< Stream upload completed (stream data buffer depleted)
#define SMTC_MODEM_EVENT_JOINFAIL 0x0A             //!< Attempt to join network failed
#define SMTC_MODEM_EVENT_TIME 0x0D                 //!< Update on time happened (synced or invalid)
#define SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED 0x0E  //!< ADR profile was switched to network controlled
#define SMTC_MODEM_EVENT_NEW_LINK_ADR 0x0F         //!< New link ADR requested by network
#define SMTC_MODEM_EVENT_LINK_CHECK 0x10           //!< Link Check answered by network
#define SMTC_MODEM_EVENT_ALMANAC_UPDATE 0x11       //!< An almanac update has been received
#define SMTC_MODEM_EVENT_USER_RADIO_ACCESS 0x12    //!< radio callback when user use the radio by itself
#define SMTC_MODEM_EVENT_NONE 0xFF                 //!< No event available
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
    SMTC_MODEM_STACK_STATE_IDLE,
    SMTC_MODEM_STACK_STATE_BUSY,
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
    SMTC_MODEM_EVENT_USER_RADIO_ACCESS_UNKNOWN        = 8,
} smtc_modem_event_user_radio_access_status_t;

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
 * @retval SMTC_MODEM_RC_INVALID       At least one of the following paramters is NULL: "event" or "event_pending_count"
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */

smtc_modem_return_code_t smtc_modem_get_event( smtc_modem_event_t* event, uint8_t* event_pending_count );

/**
 * @brief Get the modem firmware version
 *
 * @param [out] firmware_version The firmware version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "firmware_version" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_modem_version( smtc_modem_version_t* firmware_version );

/**
 * @brief Get the LoRaWAN stack version
 *
 * @param [out] lorawan_version  The LoRaWAN version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "lorawan_version" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_lorawan_version( smtc_modem_lorawan_version_t* lorawan_version );

/**
 * @brief Get the stack Regional Parameters version
 *
 * @param [out] regional_params_version The stack regional paramaters version
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "regional_params_version" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_regional_params_version(
    smtc_modem_lorawan_version_t* regional_params_version );

/**
 * @brief Reset the modem
 *
 * @remark Resets modem transient state (including session information) by reseting the MCU.
 *         Device Management Port, Modem Region and LoRaWan Devnonce are kept
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
 *         Only LoRaWAN Devnonce is kept
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_factory_reset( void );

/**
 * @brief Get the total charge counter of the modem in mAh
 *
 * @param [out] charge_mah The accumulated charge in mAh
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "charge_mah" is NULL
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
 * @param [in]  stack_id         The stack identifier
 * @param [out] tx_pwr_offset_db The Tx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "tx_pwr_offset_db" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_tx_power_offset_db( uint8_t stack_id, int8_t* tx_pwr_offset_db );

/**
 * @brief Set the Tx power offset in dB
 *
 * @param [in] stack_id         The stack identifier
 * @param [in] tx_pwr_offset_db The Tx power offset in dB
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "tx_pwr_offset_db" is out of range
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_tx_power_offset_db( uint8_t stack_id, int8_t tx_pwr_offset_db );

/**
 * @brief Start a chosen time synchronization service
 *
 * @param [in] stack_id     The stack identifier
 * @param [in] sync_service The time synchronization service to use
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              A time synchronization service is already running
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_time_start_sync_service( uint8_t                        stack_id,
                                                             smtc_modem_time_sync_service_t sync_service );

/**
 * @brief Stop current time synchronization service
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              No time synchronization service is running
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_time_stop_sync_service( uint8_t stack_id );

/**
 * @brief Get GPS epoch time
 *
 * @remark The returned time specifies the number of seconds elapsed since GPS epoch (00:00:00, Sunday 6th of January
 * 1980).
 *
 * @param [out] gps_time_s       The GPS time in seconds
 * @param [out] gps_fractional_s The GPS fractional second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "gps_time_s" or "gps_fractional_s" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_NO_TIME       No time available
 */
smtc_modem_return_code_t smtc_modem_get_time( uint32_t* gps_time_s, uint32_t* gps_fractional_s );

/**
 * @brief Trigger a single uplink requesting time
 *
 * @remark Can only be used with SMTC_MODEM_TIME_MAC_SYNC, not compatible with SMTC_MODEM_TIME_ALC_SYNC
 *
 * @param [in] stack_id     The stack identifier
 * @param [in] sync_service The time synchronization service to use
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Invalid sync_service parameter: only SMTC_MODEM_TIME_MAC_SYNC is currently
 * allowed
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_time_trigger_sync_request( uint8_t                        stack_id,
                                                               smtc_modem_time_sync_service_t sync_service );

/**
 * @brief Set ALCSync service port
 *
 * @remark When using DM port as alcsync_fport, the alcsync messages will be encapsulated into DM frames.
 *
 * @param [in] alcsync_fport The FPort for ALCSync messages. A value of 0 or currently configured DM port
 *                           will encapsulate it into DM messages
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "alcsync_fport" is invalid: out of [0:223] range
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_set_alcsync_fport( uint8_t alcsync_fport );

/**
 * @brief Get ALCSync service FPort
 *
 * @param [out] alcsync_fport The Clock Sync FPort
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "alcsync_fport" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_get_alcsync_fport( uint8_t* alcsync_fport );

/**
 * @brief Set the interval between time synchronizations
 *
 * @param [in] sync_interval_s The interval in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "sync_interval_s" is invalid if greater than the delay before
 *                                     sync is no more valid
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_set_sync_interval_s( uint32_t sync_interval_s );

/**
 * @brief Get the interval between time synchronizations
 *
 * @param [out] sync_interval_s The interval in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "sync_interval_s" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_get_sync_interval_s( uint32_t* sync_interval_s );

/**
 * @brief Set the delay beyond which the time synchronization is no longer considered valid by the modem
 *
 * @remark Modem will generate a SMTC_MODEM_EVENT_TIME event if the there are no time synchronizations for more
 *         time than this "invalid delay".
 *
 * @param [in] sync_invalid_delay_s The invalid delay in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "sync_invalid_delay_s" is greater 49 days
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_set_sync_invalid_delay_s( uint32_t sync_invalid_delay_s );

/**
 * @brief Get the configured delay beyond which the time synchronization is no longer valid
 *
 * @param [out] sync_invalid_delay_s The invalid delay in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "sync_invalid_delay_s" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_time_get_sync_invalid_delay_s( uint32_t* sync_invalid_delay_s );

/**
 * @brief Get the modem status
 *
 * @param [in]  stack_id    The stack identifier
 * @param [out] status_mask The modem status defined in @ref smtc_modem_status_mask_e
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "status_mask" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_status( uint8_t stack_id, smtc_modem_status_mask_t* status_mask );

/**
 * @brief Set and start the alarm timer
 *
 * @remark When the timer expires, an alarm event is generated
 *
 * @param [in] alarm_timer_in_s The alarm timer in second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_alarm_start_timer( uint32_t alarm_timer_in_s );

/**
 * @brief Stop and clear alarm timer
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
 * @param [out] remaining_time_in_s The number of seconds remaining before the alarm triggers an event
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT      No alarm timer currently running
 * @retval SMTC_MODEM_RC_INVALID       Parameter "remaining_time_in_s" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_alarm_get_remaining_time( uint32_t* remaining_time_in_s );

/**
 * @brief Get the JoinEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] joineui  The current JoinEUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_joineui( uint8_t stack_id, uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the JoinEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  joineui  The JoinEUI to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Get the DevEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] deveui   The current DevEUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_deveui( uint8_t stack_id, uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the DevEUI
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  deveui   The DevEUI to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );

/**
 * @brief Set the LoRaWAN v1.1.x Network Key (aka Application Key in LoRaWAN v1.0.x)
 *
 * @param [in] stack_id The stack identifier
 * @param [in] nwkkey   The key to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] );

/**
 * @brief Get the current LoRaWAN class
 *
 * @param [in]  stack_id      The stack identifier
 * @param [out] lorawan_class The current LoRaWAN class
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_class( uint8_t stack_id, smtc_modem_class_t* lorawan_class );

/**
 * @brief Set the LoRaWAN class
 *
 * @param [in] stack_id      The stack identifier
 * @param [in] lorawan_class The LoRaWAN class to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class );

/**
 * @brief Configure a multicast group
 *
 * @param [in] stack_id     The stack identifier
 * @param [in] mc_grp_id    The multicast group identifier
 * @param [in] mc_grp_addr  The multicast group address
 * @param [in] mc_nwk_skey  The multicast network session key for the group
 * @param [in] mc_app_skey  The multicast application session key for the group
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Group id is not in acceptable range [0:3]
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          Error during cryptographic process, a running session already exists on this id
 */
smtc_modem_return_code_t smtc_modem_multicast_set_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t      mc_grp_addr,
                                                              const uint8_t mc_nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                              const uint8_t mc_app_skey[SMTC_MODEM_KEY_LENGTH] );

/**
 * @brief Get the configuration of the chosen multicast group
 *
 * @param [in]  stack_id     The stack identifier
 * @param [in]  mc_grp_id    The multicast group identifier
 * @param [out] mc_grp_addr  The multicast group address
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Group id is not in acceptable range [0:3] or parameter "mc_grp_addr" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_multicast_get_grp_config( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                              uint32_t* mc_grp_addr );

/**
 * @brief Start a multicast session for a specific group
 *
 * @param [in] stack_id   The stack identifier
 * @param [in] mc_grp_id  The multicast group identifier
 * @param [in] freq       The downlink frequency for this session
 * @param [in] dr         The downlink datarate for this session
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Group id is not in acceptable range [0:3]
 *                          Frequency or Datarate are not in acceptable range (according to current regional params)
 *                          Frequency or Datarate are not compatible with an already running multicast session
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          This session is already started
 */
smtc_modem_return_code_t smtc_modem_multicast_start_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                             uint32_t freq, uint8_t dr );

/**
 * @brief Get a multicast session status for a chosen group
 *
 * @param [in]  stack_id            The stack identifier
 * @param [in]  mc_grp_id           The multicast group identifier
 * @param [out] is_session_started  Session status
 * @param [out] freq                Session downlink frequency
 * @param [out] dr                  Session downlink datarate
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Group id is not in acceptable range [0:3] or a paramater is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_multicast_get_session_status( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id,
                                                                  bool* is_session_started, uint32_t* freq,
                                                                  uint8_t* dr );

/**
 * @brief Stop a multicast session for a chosen group
 *
 * @param [in] stack_id   The stack identifier
 * @param [in] mc_grp_id  The multicast group identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Group id is not in acceptable range [0:3]
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_multicast_stop_session( uint8_t stack_id, smtc_modem_mc_grp_id_t mc_grp_id );

/**
 * @brief Stop all started multicast sessions
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_multicast_stop_all_sessions( uint8_t stack_id );

/**
 * @brief Get the current LoRaWAN region
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] region   The current LoRaWAN region
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "region" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_region( uint8_t stack_id, smtc_modem_region_t* region );

/**
 * @brief Set the LoRaWAN region
 *
 * @param [in]  stack_id The stack identifier
 * @param [in]  region   The LoRaWAN region to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Chosen region is not supported
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_region( uint8_t stack_id, smtc_modem_region_t region );

/**
 * @brief Get the current ADR profile
 *
 * @param [in]  stack_id    The stack identifier
 * @param [out] adr_profile The current ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "adr_profile" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_adr_get_profile( uint8_t stack_id, smtc_modem_adr_profile_t* adr_profile );

/**
 * @brief Set the ADR profile
 *
 * @remark If @ref SMTC_MODEM_ADR_PROFILE_CUSTOM is selected, custom data are taken into account
 *
 * @param [in] stack_id        The stack identifier
 * @param [in] adr_profile     The ADR profile to be configured
 * @param [in] adr_custom_data The definition of the custom ADR profile
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           One or more invalid parameter
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
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
 * @retval SMTC_MODEM_RC_INVALID           Parameter "available_datarates_mask" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_available_datarates( uint8_t stack_id, uint16_t* available_datarates_mask );

/**
 * @brief Get the Device Management LoRaWAN FPort
 *
 * @param [out] dm_fport The FPort on which the DM info is sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "dm_fport" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_fport( uint8_t* dm_fport );

/**
 * @brief Set the Device Management LoRaWAN FPort
 *
 * @param [in] dm_fport The FPort on which the DM info is sent. This value must be in the range [1:223]
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter dm_fport is out of the [1:223] range
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_fport( uint8_t dm_fport );

/**
 * @brief Get the interval between 2 DM info field messages
 *
 * @param [out] format   The reporting interval format
 * @param [out] interval The interval in unit defined in format
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameters "format" and/or "interval" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_info_interval( smtc_modem_dm_info_interval_format_t* format,
                                                          uint8_t*                              interval );

/**
 * @brief Set the interval between 2 DM info field messages
 *
 * @remark A value set to 0 disables the feature - no matter the format.
 *
 * @param [in] format   The reporting interval format
 * @param [in] interval The interval in unit defined in format, from 0 to 63
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Interval not in [0:63] range.
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_info_interval( smtc_modem_dm_info_interval_format_t format,
                                                          uint8_t                              interval );

/**
 * @brief Get DM info fields
 *
 * @param [out] dm_fields_payload The DM fields  (defined in @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [out] dm_field_length   The DM field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameters "dm_fields_payload" and/or "dm_field_length" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_info_fields( uint8_t* dm_fields_payload, uint8_t* dm_field_length );

/**
 * @brief Set the DM info fields to be sent on a regular basis
 *
 * @remark The interval between two DM info field messages is defined with @ref smtc_modem_dm_set_info_interval
 *
 * @param [in] dm_fields_payload The DM fields  (defined in @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length   The DM field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Invalid or duplicated dm field code
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_info_fields( const uint8_t* dm_fields_payload, uint8_t dm_field_length );

/**
 * @brief Request an immediate DM status
 *
 * @remark The content is independent from the configuration set with @ref smtc_modem_dm_set_info_fields
 *
 * @param [in] dm_fields_payload The DM fields  (defined in @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length   The DM field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Invalid or duplicated field code or parameter "dm_fields_payload" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          Modem is not available (suspended, muted or not joined)
 */
smtc_modem_return_code_t smtc_modem_dm_request_single_uplink( const uint8_t* dm_fields_payload,
                                                              uint8_t        dm_field_length );

/**
 * @brief Set user-specific data to be reported by Device Management frames
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
 * @brief Get user-specific data to be reported by Device Management frames
 *
 * @param [out] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "user_data" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_user_data( uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Join the network
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors or modem has already joined the network
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended or muted)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id );

/**
 * @brief Leave an already joined network or cancels on ongoing join process
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
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
 * @param [in]  stack_id            The stack identifier
 * @param [out] tx_max_payload_size The maximum payload size in byte
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters "tx_max_payload_size" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem has not joined a network
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size );

/**
 * @brief Request a LoRaWAN uplink
 *
 * @remark LoRaWAN NbTrans parameter can be set in mobiles and custom ADR modes with @ref smtc_modem_set_nb_trans
 *
 * @param [in] stack_id       The stack identifier
 * @param [in] fport          The LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed      The message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        The data to be sent
 * @param [in] payload_length The number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter fport is out of the [1:223] range or equal to dm_fport
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_request_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                    const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Request an immediate LoRaWAN uplink
 *
 * @remark It has higher priority than all other services and is not subject to duty cycle restrictions, if any
 * @remark LoRaWAN NbTrans parameter can be set in mobiles and custom ADR modes with @ref smtc_modem_set_nb_trans
 *
 * @param [in] stack_id       The stack identifier
 * @param [in] fport          The LoRaWAN FPort on which the uplink is done
 * @param [in] confirmed      The message type (true: confirmed, false: unconfirmed)
 * @param [in] payload        The data to be sent
 * @param [in] payload_length The number of bytes from payload to be sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter fport is out of the [1:223] range or equal to dm_fport
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_request_emergency_uplink( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                              const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Request a LoRaWAN uplink without payload, and an optional FPort
 *
 * @remark Can be used to create downlink opportunities or heartbeat without involving message routing to the AS
 *
 * @param [in] stack_id       The stack identifier
 * @param [in] send_fport     Add the FPort to the payload (true: add the FPort, false: send without FPort)
 * @param [in] fport          The LoRaWAN FPort on which the uplink is done, if used
 * @param [in] confirmed      The message type (true: confirmed, false: unconfirmed)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "fport" is out of the [1;223] range or equal to dm_fport
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_request_empty_uplink( uint8_t stack_id, bool send_fport, uint8_t fport,
                                                          bool confirmed );

/**
 * @brief Create and initialize a file upload session
 *
 * @param [in] stack_id        The stack identifier
 * @param [in] index           The index on which the upload is done
 * @param [in] cipher_mode     The cipher mode
 * @param [in] file            The file buffer
 * @param [in] file_length     The file size
 * @param [in] average_delay_s The minimal delay between two file upload fragments, in seconds (from the end of an
 *                             uplink to the start of the next one)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           file_length is equal to 0 or greater than 2048 bytes, or "file" pointer is
 *                                         NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or a file upload is already ongoing
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_file_upload_init( uint8_t stack_id, uint8_t index,
                                                      smtc_modem_file_upload_cipher_mode_t cipher_mode,
                                                      const uint8_t* file, uint16_t file_length,
                                                      uint32_t average_delay_s );

/**
 * @brief Start the file upload towards the DAS
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or a file upload is already ongoing
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_BAD_SIZE          Total data sent does not match the declared Size value in
 *                                         smtc_modem_file_upload_init()
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_file_upload_start( uint8_t stack_id );

/**
 * @brief Reset the file upload session
 *
 * @remark This function will stop any ongoing file upload session
 *
 * @param [in] stack_id The stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT          No file upload session currently running
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_file_upload_reset( uint8_t stack_id );

/**
 * @brief Create and initialize a data stream
 *
 * @param [in] stack_id                  The stack identifier
 * @param [in] fport                     The FPort on which the stream is sent. A value of 0 will use configured DM port
 * @param [in] redundancy_ratio_percent  The stream redundancy ratio.
 * @param [in] cipher_mode               The cipher mode
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           FPort is out of the [0:223] range
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or the streaming buffer is not empty
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t fport,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode,
                                                 uint8_t                         redundancy_ratio_percent );

/**
 * @brief Add data to the stream
 *
 * @remark If no stream has been initialized,  open a new unencrypted stream on the DM FPort with a redundancy ratio set
 *         to 110%
 *
 * @param [in] stack_id The stack identifier
 * @param [in] data     The data to be added to the stream
 * @param [in] len      The number of bytes from data to be added to the stream
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Data length is not in range [1-254] or parameter "data" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or a the streaming buffer is full
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len );

/**
 * @brief Return the current stream status
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] pending  The length of pending data for transmission
 * @param [out] free     The length of free space in the buffer
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT          No stream session is running
 * @retval SMTC_MODEM_RC_INVALID           Parameters pending and/or free is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free );

/**
 * @brief Enable / disable the certification mode
 *
 * @param [in] stack_id The stack identifier
 * @param [in] enable   The certification mode state (default: disabled)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable );

/**
 * @brief Get the current state of the certification mode
 *
 * @param [in]  stack_id The stack identifier
 * @param [out] enable   The certification mode state
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "enable " is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_certification_mode( uint8_t stack_id, bool* enable );

/**
 * @brief Set the connection timeout thresholds
 *
 * @remark The value 0 deactivates the function
 * @remark It is recommended to have nb_of_uplinks_before_network_controlled smaller than nb_of_uplink_before_reset
 *
 * @param [in] stack_id                                The stack identifier
 * @param [in] nb_of_uplinks_before_network_controlled The number of uplinks without downlink before ADR profile
 *                                                     switches to network controlled
 * @param [in] nb_of_uplinks_before_reset              The number of uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_connection_timeout_set_thresholds( uint8_t  stack_id,
                                                                       uint16_t nb_of_uplinks_before_network_controlled,
                                                                       uint16_t nb_of_uplinks_before_reset );

/**
 * @brief Get the configured connection timeout thresholds
 *
 * @param [in]  stack_id                                The stack identifier
 * @param [out] nb_of_uplinks_before_network_controlled The number of uplinks without downlink before ADR profile
 *                                                      switches to network controlled
 * @param [out] nb_of_uplinks_before_reset              The number of uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters "nb_of_uplinks_before_network_controlled" and/or
 *                                         "nb_of_uplinks_before_reset" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_connection_timeout_get_thresholds(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset );

/**
 * @brief Get the current status of the connection timeouts
 *
 * @param [in]  stack_id                                 The stack identifier
 * @param [out] nb_of_uplinks_before_network_controlled  The number of remaining uplinks without downlink before ADR
 *                                                       profile switches to network controlled
 * @param [out] nb_of_uplinks_before_reset               The number of remaining uplinks without downlink before reset
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters "nb_of_uplinks_before_network_controlled" and/or
 *                                         "nb_of_uplinks_before_reset" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_connection_timeout_get_current_values(
    uint8_t stack_id, uint16_t* nb_of_uplinks_before_network_controlled, uint16_t* nb_of_uplinks_before_reset );

/**
 * @brief Get the current status of the duty cycle
 *
 * @remark If the returned value is positive, it is the time still available. A negative value indicates the time to
 * wait until band availability
 *
 * @param [out] duty_cycle_status_ms The status of the duty cycle in milliseconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "duty_cycle_status_ms" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( int32_t* duty_cycle_status_ms );

/**
 * @brief Get the current state of the stack
 *
 * @param [in]  stack_id     The stack identifier
 * @param [out] stack_state  The stack current state
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "stack_state" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_stack_state( uint8_t stack_id, smtc_modem_stack_state_t* stack_state );

/**
 * @brief Configure network type to private or public
 *
 * @param [in]  stack_id      The stack identifier
 * @param [in]  network_type  The configuration to be applied (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "network_type" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_network_type( uint8_t stack_id, bool network_type );

/**
 * @brief Get the configured network type
 *
 * @param [in]  stack_id      The stack identifier
 * @param [in]  network_type  The configuration to be applied (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "network_type" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_network_type( uint8_t stack_id, bool* network_type );

/**
 * @brief Set the parameters of the LBT (Listen Before Talk) feature
 *
 * @param [in]  stack_id               The stack identifier
 * @param [in]  listening_duration_ms  The listening duration in ms to be configured
 * @param [in]  threshold_dbm          The LBT threshold in dbm to be configured
 * @param [in]  bw_hz                  The LBT bandwith in Hertz to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_set_parameters( uint8_t stack_id, uint32_t listening_duration_ms,
                                                        int16_t threshold_dbm, uint32_t bw_hz );

/**
 * @brief Get the parameters of the LBT (Listen Before Talk) feature
 *
 * @param [in]  stack_id               The stack identifier
 * @param [out] listening_duration_ms  The current listening duration in ms
 * @param [out] threshold_dbm          The current LBT threshold in dbm
 * @param [out] bw_hz                  The current LBT bandwith in Hertz
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           At least one pointer is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_get_parameters( uint8_t stack_id, uint32_t* listening_duration_ms,
                                                        int16_t* threshold_dbm, uint32_t* bw_hz );

/**
 * @brief Enable or disable the LBT (Listen Before Talk) feature
 *
 * @remark The configuration function @ref smtc_modem_lbt_set_parameters must be called before enabling the LBT feature.
 * @remark LBT is silently enabled if the feature is mandatory in a region selected with @ref smtc_modem_set_region
 *
 * @param [in] stack_id  The stack identifier
 * @param [in] enable    The status of the LBT feature to set (true: enable, false: disable)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_set_state( uint8_t stack_id, bool enable );

/**
 * @brief Get the state of the LBT (Listen Before Talk) feature
 *
 * @param [in]  stack_id  The stack identifier
 * @param [out] enabled   The current status of the LBT feature (true: enabled, false: disabled)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "enabled" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_lbt_get_state( uint8_t stack_id, bool* enabled );

/**
 * @brief Set the number of transmissions (nb_trans) in case of unconfirmed uplink
 *
 * @param [in]  stack_id  The stack identifier
 * @param [in]  nb_trans  The number of transmissions ( 0 < value < 16 )
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "nb_trans" is not in [1:15] range or current ADR
 *                                         profile is "Network controlled"
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_set_nb_trans( uint8_t stack_id, uint8_t nb_trans );

/**
 * @brief Get the configured number of transmissions (nb_trans) in case of unconfirmed uplink
 *
 * @param [in]  stack_id  The stack identifier
 * @param [out] nb_trans  The number of transmissions
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameter "nb_trans" is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_nb_trans( uint8_t stack_id, uint8_t* nb_trans );

/**
 * @brief Set modem crystal error
 *
 * @param [in] crystal_error_ppm The crystal error in per thousand
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "crystal_error_per_thousand" is not in [0:10] range
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_set_crystal_error( uint32_t crystal_error_per_thousand );

/**
 * @brief Get the modem crystal error
 *
 * @param [out] crystal_error_ppm The crystal error in per thousand
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter "crystal_error_per_thousand" is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
*/
smtc_modem_return_code_t smtc_modem_get_crystal_error( uint32_t* crystal_error_per_thousand );

/**
 * @brief Request a Link Check Req MAC command to the network
 *
 * @remark The request will be sent in a new uplink frame
 *
 * @return smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is not available (suspended, muted or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid stack_id
 *
 */
smtc_modem_return_code_t smtc_modem_lorawan_request_link_check( uint8_t stack_id );

/**
 * @brief Grant user radio access by suspending the modem and kill all current modem radio tasks
 *
 * @remark The user must call this function before performing operations requiring a direct access to the radio (e.g.
 * test modes). Otherwise, undefined behavior may occurs.
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

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_API_H__

/* --- EOF ------------------------------------------------------------------ */
