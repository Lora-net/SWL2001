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
 * @brief Modem PIN code length in byte
 */
#define SMTC_MODEM_PIN_LENGTH 4

/**
 * @brief ADR custom configuration length in byte
 */
#define SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH 16

/**
 * @brief Maximum payload size in byte of LoRaWAN payload
 */
#define SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH 242

/**
 * @brief Application-defined user data length in byte
 */
#define SMTC_MODEM_DM_USER_DATA_LENGTH 8

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
    SMTC_MODEM_RC_OK = 0x00,         //!< command executed without errors
    SMTC_MODEM_RC_NOT_INIT,          //!< command not initialized
    SMTC_MODEM_RC_INVALID,           //!< command parameters invalid
    SMTC_MODEM_RC_BUSY,              //!< command cannot be executed now
    SMTC_MODEM_RC_FAIL,              //!< command execution failed
    SMTC_MODEM_RC_NO_TIME,           //!< no time available
    SMTC_MODEM_RC_INVALID_STACK_ID,  //!< invalid stack_id parameter
    SMTC_MODEM_RC_NO_EVENT,          //!< no event available
} smtc_modem_return_code_t;

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
 * @brief Modem class enumeration
 */
typedef enum smtc_modem_class_e
{
    SMTC_MODEM_CLASS_A = 0x00,  //!< Modem class A
    SMTC_MODEM_CLASS_B = 0x01,  //!< Modem class B
    SMTC_MODEM_CLASS_C = 0x02,  //!< Modem class C
    SMTC_MODEM_CLASS_MAX,
} smtc_modem_class_t;

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
 * @brief LoRaWAN Mac request mask definition
 */
enum smtc_modem_lorawan_mac_request_mask_e
{
    SMTC_MODEM_LORAWAN_MAC_REQ_LINK_CHECK     = ( 1 << 0 ),
    SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME    = ( 1 << 1 ),
    SMTC_MODEM_LORAWAN_MAC_REQ_PING_SLOT_INFO = ( 1 << 2 ),
};

typedef uint8_t smtc_modem_lorawan_mac_request_mask_t;

/**
 * @brief Rx window type
 */
typedef enum smtc_modem_dl_window_e
{
    SMTC_MODEM_DL_WINDOW_RX1         = 0x01,
    SMTC_MODEM_DL_WINDOW_RX2         = 0x02,
    SMTC_MODEM_DL_WINDOW_RXC         = 0x03,
    SMTC_MODEM_DL_WINDOW_RXC_MC_GRP0 = 0x04,
    SMTC_MODEM_DL_WINDOW_RXC_MC_GRP1 = 0x05,
    SMTC_MODEM_DL_WINDOW_RXC_MC_GRP2 = 0x06,
    SMTC_MODEM_DL_WINDOW_RXC_MC_GRP3 = 0x07,
    SMTC_MODEM_DL_WINDOW_RXB         = 0x08,
    SMTC_MODEM_DL_WINDOW_RXB_MC_GRP0 = 0x09,
    SMTC_MODEM_DL_WINDOW_RXB_MC_GRP1 = 0x0A,
    SMTC_MODEM_DL_WINDOW_RXB_MC_GRP2 = 0x0B,
    SMTC_MODEM_DL_WINDOW_RXB_MC_GRP3 = 0x0C,
    SMTC_MODEM_DL_WINDOW_RXBEACON    = 0x0D,
    SMTC_MODEM_DL_WINDOW_RXR         = 0x0E,
} smtc_modem_dl_window_t;

/**
 * @brief Definition of Store and forward enablement
 *
 */
typedef enum smtc_modem_store_and_forward_state_e
{
    SMTC_MODEM_STORE_AND_FORWARD_DISABLE = 0,  // Service disabled
    SMTC_MODEM_STORE_AND_FORWARD_ENABLE  = 1,  // Service enabled
    SMTC_MODEM_STORE_AND_FORWARD_SUSPEND = 2,  // Service enabled, but send data is suspend
} smtc_modem_store_and_forward_state_t;

/**
 * @brief  Downlink metadata structure
 */
typedef struct smtc_modem_dl_metadata_s
{
    uint8_t                stack_id;
    int8_t                 rssi;  //!< Signed value in dBm + 64
    int8_t                 snr;   //!< Signed value in dB given in 0.25dB step
    smtc_modem_dl_window_t window;
    uint8_t                fport;
    uint8_t                fpending_bit;
    uint32_t               frequency_hz;
    uint8_t                datarate;
} smtc_modem_dl_metadata_t;

/**
 * @brief Cipher mode for stream service
 */
typedef enum smtc_modem_stream_cipher_mode_e
{
    SMTC_MODEM_STREAM_NO_CIPHER,         //!< Do not encrypt stream
    SMTC_MODEM_STREAM_AES_WITH_APPSKEY,  //!< Encrypt stream using AES with appskey
} smtc_modem_stream_cipher_mode_t;

/**
 * @brief Cipher mode for file upload service
 */
typedef enum smtc_modem_file_upload_cipher_mode_e
{
    SMTC_MODEM_FILE_UPLOAD_NO_CIPHER,         //!< Do not encrypt file
    SMTC_MODEM_FILE_UPLOAD_AES_WITH_APPSKEY,  //!< Encrypt file using AES with appskey
} smtc_modem_file_upload_cipher_mode_t;

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
 * @brief SMTC_MODEM_DM_INFO_DEF DM info fields codes
 */
typedef enum smtc_modem_dm_field_e
{
    SMTC_MODEM_DM_FIELD_STATUS      = 0x00,  //!< modem status
    SMTC_MODEM_DM_FIELD_CHARGE      = 0x01,  //!< charge counter [mAh]
    SMTC_MODEM_DM_FIELD_VOLTAGE     = 0x02,  //!< supply voltage [1/50 V]
    SMTC_MODEM_DM_FIELD_TEMPERATURE = 0x03,  //!< junction temperature [deg Celsius]
    SMTC_MODEM_DM_FIELD_SIGNAL      = 0x04,  //!< strength of last downlink (RSSI [dBm]+64, SNR [0.25 dB])
    SMTC_MODEM_DM_FIELD_UP_TIME     = 0x05,  //!< duration since last reset [h]
    SMTC_MODEM_DM_FIELD_RX_TIME     = 0x06,  //!< duration since last downlink [h]
    SMTC_MODEM_DM_FIELD_ADR_MODE    = 0x08,  //!< ADR profile (0-3)
    SMTC_MODEM_DM_FIELD_JOIN_EUI    = 0x09,  //!< JoinEUI
    SMTC_MODEM_DM_FIELD_INTERVAL    = 0x0A,  //!< reporting interval [values 0-63, units s/m/h/d]
    SMTC_MODEM_DM_FIELD_REGION      = 0x0B,  //!< regulatory region
    SMTC_MODEM_DM_FIELD_CRASHLOG    = 0x0D,  //!< Crashlog data (cannot be part of periodic frame)
    SMTC_MODEM_DM_FIELD_RST_COUNT   = 0x0F,  //!< modem reset count
    SMTC_MODEM_DM_FIELD_DEV_EUI     = 0x10,  //!< DevEUI
    SMTC_MODEM_DM_FIELD_SESSION     = 0x12,  //!< session id / join nonce
    SMTC_MODEM_DM_FIELD_CHIP_EUI    = 0x13,  //!< ChipEUI
    SMTC_MODEM_DM_INFO_STREAMPAR    = 0x15,  //!< data stream parameters
    SMTC_MODEM_DM_FIELD_APP_STATUS  = 0x16,  //!< application-specific status
} smtc_modem_dm_field_t;

/**
 * @brief Modem events
 */
typedef enum smtc_modem_event_type_e
{
    SMTC_MODEM_EVENT_RESET = 0x00,                       //!< Modem has been reset
    SMTC_MODEM_EVENT_ALARM,                              //!< Alarm timer expired
    SMTC_MODEM_EVENT_JOINED,                             //!< Network successfully joined
    SMTC_MODEM_EVENT_TXDONE,                             //!< Frame transmitted
    SMTC_MODEM_EVENT_DOWNDATA,                           //!< Downlink data received
    SMTC_MODEM_EVENT_JOINFAIL,                           //!< Attempt to join network failed
    SMTC_MODEM_EVENT_ALCSYNC_TIME,                       //!< ALCSync time updated
    SMTC_MODEM_EVENT_LINK_CHECK,                         //!< Link Check answered or not by network
    SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO,             //!< Ping Slot Info answered or not by network
    SMTC_MODEM_EVENT_CLASS_B_STATUS,                     //!< Downlink class B is ready or not
    SMTC_MODEM_EVENT_LORAWAN_MAC_TIME,                   //!< LoRaWAN mac device time answered or not by network
    SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE,                 //!< End of FUOTA session
    SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C,  //!< End of multicast session in class C
    SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B,  //!< End of multicast session in class B
    SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C,      //!< New active multicast session in class C
    SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B,      //!< New active multicast session in class B
    SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT,                //!< Firmware Management Package (FMP) event
    SMTC_MODEM_EVENT_STREAM_DONE,                        //!< Stream upload completed (stream data buffer depleted)
    SMTC_MODEM_EVENT_UPLOAD_DONE,                        //!< File upload completed
    SMTC_MODEM_EVENT_DM_SET_CONF,                        //!< Configuration has been changed by the Device Management
    SMTC_MODEM_EVENT_MUTE,                               //!< Modem has been muted or un-muted by the Device Management
    SMTC_MODEM_EVENT_GNSS_SCAN_DONE,
    SMTC_MODEM_EVENT_GNSS_TERMINATED,
    SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE,
    SMTC_MODEM_EVENT_WIFI_SCAN_DONE,
    SMTC_MODEM_EVENT_WIFI_TERMINATED,
    SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC,  //!< Relay TX dynamic mode has enable or disable the WOR protocol
    SMTC_MODEM_EVENT_RELAY_TX_MODE,     //!< Relay TX activation has been updated
    SMTC_MODEM_EVENT_RELAY_TX_SYNC,     //!< Relay TX synchronisation has changed
    SMTC_MODEM_EVENT_RELAY_RX_RUNNING,  //!< Relay RX running has changed
    SMTC_MODEM_EVENT_TEST_MODE,
    SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE,
    SMTC_MODEM_EVENT_MAX,
} smtc_modem_event_type_t;

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
 * @brief Status returned by the SMTC_MODEM_EVENT_CLASS_B_STATUS
 */
typedef enum smtc_modem_event_class_b_status_e
{
    SMTC_MODEM_EVENT_CLASS_B_NOT_READY = 0,
    SMTC_MODEM_EVENT_CLASS_B_READY     = 1,
} smtc_modem_event_class_b_status_t;

/**
 * @brief Status returned by the LoRaWAN mac request events
 * (SMTC_MODEM_EVENT_LINK_CHECK,SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO,SMTC_MODEM_EVENT_LORAWAN_MAC_TIME)
 */
typedef enum smtc_modem_event_mac_request_status_e
{
    SMTC_MODEM_EVENT_MAC_REQUEST_NOT_ANSWERED = 0,
    SMTC_MODEM_EVENT_MAC_REQUEST_ANSWERED     = 1,
} smtc_modem_event_mac_request_status_t;

/**
 * @brief Status returned by the Firmware Management Package
 */
typedef enum smtc_modem_event_fmp_status_e
{
    SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY = 0,
    SMTC_MODEM_EVENT_FMP_CANCEL_REBOOT      = 1,
} smtc_modem_event_fmp_status_t;

typedef enum smtc_modem_event_setconf_e
{
    SMTC_MODEM_EVENT_SETCONF_ADR_MODE_UPDATED    = SMTC_MODEM_DM_FIELD_ADR_MODE,
    SMTC_MODEM_EVENT_SETCONF_JOIN_EUI_UPDATED    = SMTC_MODEM_DM_FIELD_JOIN_EUI,
    SMTC_MODEM_EVENT_SETCONF_DM_INTERVAL_UPDATED = SMTC_MODEM_DM_FIELD_INTERVAL,
    SMTC_MODEM_EVENT_SETCONF_REGION_UPDATED      = SMTC_MODEM_DM_FIELD_REGION,
} smtc_modem_event_setconf_opcode_t;

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
    SMTC_MODEM_EVENT_UPLOAD_DONE_ABORTED    = 0,
    SMTC_MODEM_EVENT_UPLOAD_DONE_SUCCESSFUL = 1,
} smtc_modem_event_uploaddone_status_t;

typedef enum smtc_modem_event_test_mode_status_e
{
    SMTC_MODEM_EVENT_TEST_MODE_ENDED        = 0,
    SMTC_MODEM_EVENT_TEST_MODE_TX_COMPLETED = 1,
    SMTC_MODEM_EVENT_TEST_MODE_TX_DONE      = 2,
    SMTC_MODEM_EVENT_TEST_MODE_RX_DONE      = 3,
    SMTC_MODEM_EVENT_TEST_MODE_RX_ABORTED   = 4,
} smtc_modem_event_test_mode_status_t;
/**
 * @brief Structure holding event-related data
 */
typedef struct smtc_modem_event_s
{
    uint8_t                 stack_id;
    smtc_modem_event_type_t event_type;
    uint8_t                 missed_events;  //!< Number of event_type events missed before the current one
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
            smtc_modem_event_mac_request_status_t status;
        } link_check;
        struct
        {
            smtc_modem_event_mac_request_status_t status;
        } class_b_ping_slot_info;
        struct
        {
            smtc_modem_event_mac_request_status_t status;
        } lorawan_mac_time;
        struct
        {
            smtc_modem_event_class_b_status_t status;
        } class_b_status;
        struct
        {
            bool successful;
        } fuota_status;
        struct
        {
            smtc_modem_mc_grp_id_t group_id;
        } new_multicast_class_c;
        struct
        {
            smtc_modem_mc_grp_id_t group_id;
        } new_multicast_class_b;
        struct
        {
            smtc_modem_event_fmp_status_t status;
        } fmp;
        struct
        {
            smtc_modem_event_uploaddone_status_t status;
        } uploaddone;
        struct
        {
            smtc_modem_event_setconf_opcode_t opcode;
        } setconf;
        struct
        {
            smtc_modem_event_mute_status_t status;
        } mute;
        struct
        {
            uint8_t status;
        } relay_tx;
        struct
        {
            uint8_t status;
        } relay_rx;
        struct
        {
            smtc_modem_event_test_mode_status_t status;
        } test_mode_status;
        struct
        {
            uint8_t status;
        } regional_duty_cycle;
    } event_data;
} smtc_modem_event_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * ----------- BASIC MODEM FUNCTIONS -------------------------------------------
 */

/**
 * @brief Get the current LoRaWAN region
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] region_list        List of region builded in firmware
 * @param [out] number_of_region   Number of region in list
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p region is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_list_region( uint8_t stack_id, uint8_t* region_list,
                                                     uint8_t* number_of_region );

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
 * @brief Join the network
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors or modem has already joined the network
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id );

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
 * @retval SMTC_MODEM_RC_NO_EVENT      No event available
 */

smtc_modem_return_code_t smtc_modem_get_event( smtc_modem_event_t* event, uint8_t* event_pending_count );

/**
 * @brief Get all the data from a downlink
 *
 *  @remark This command shall be called after the reception of a SMTC_MODEM_EVENT_DOWNDATA event
 *
 * @param [out] buff              Buffer containing the downlink payload
 * @param [out] length            Length of the downlink payload
 * @param [out] metadata          Structure holding downlink metadata
 * @param [out] remaining_data_nb Number of downlink data remaining
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       At least one parameter is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          No downlink data is available
 */
smtc_modem_return_code_t smtc_modem_get_downlink_data( uint8_t  buff[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH],
                                                       uint8_t* length, smtc_modem_dl_metadata_t* metadata,
                                                       uint8_t* remaining_data_nb );

/*
 * -----------------------------------------------------------------------------
 * ----------- CREDENTIAL MANAGEMENT FUNCTIONS ---------------------------------
 */

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
 * @brief Set the LoRaWAN v1.1.x Application Key (aka Gen Application Key in LoRaWAN v1.0.x)
 *
 * @param [in] stack_id Stack identifier
 * @param [in] appkey   Key to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or in joining/joined state
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_appkey( uint8_t stack_id, const uint8_t appkey[SMTC_MODEM_KEY_LENGTH] );

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

/*!
 * @brief Get the modem PIN code
 *
 * @remark This command can only be used with LR11XX radio
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] chip_pin  4-byte PIN code
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is already joined or is joining
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid stack_id
 */
smtc_modem_return_code_t smtc_modem_get_pin( uint8_t stack_id, uint8_t chip_pin[SMTC_MODEM_PIN_LENGTH] );

/*!
 * @brief Get the modem chip EUI
 *
 * @remark This command can only be used with LR11XX radio
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] chip_eui  8-byte chip EUI
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_chip_eui( uint8_t stack_id, uint8_t chip_eui[SMTC_MODEM_EUI_LENGTH] );

/*!
 * @brief Derive keys
 *
 * @remark This command can only be used with LR11XX radio
 *           Derives application key taking saved dev_eui (default set to chip_eui) and saved join_eui
 *
 * @param [in]  stack_id  Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is already joined or is joining
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_derive_keys( uint8_t stack_id );

#if defined( USE_LR11XX_CE )
/**
 * @brief Get Fragmented DataBlockIntKey
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] data_block_int_key  the derived DataBlockIntKey from GenAppKey
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is already joined or is joining
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_data_block_int_key( uint8_t stack_id,
                                                            uint8_t data_block_int_key[SMTC_MODEM_KEY_LENGTH] );

/**
 * @brief Derive and set DataBlockIntKey
 * @remark this key is not saved in crypto engine due to an hardware limitation
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] gen_appkey  GenAppKey for fragmented data block
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is already joined or is joining
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID   Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_derive_and_set_data_block_int_key(
    uint8_t stack_id, const uint8_t gen_appkey[SMTC_MODEM_KEY_LENGTH] );
#endif  // USE_LR11XX_CE
/*
 * -----------------------------------------------------------------------------
 * ----------- ADVANCED MODEM FUNCTIONS ----------------------------------------
 */

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
 * @brief Get the current value of the lost connection counter
 *
 * @remark The counter is incremented after any uplink and is only reset when a valid downlink is received from Network
 * Server
 *
 * @param [in]  stack_id            Stack identifier

 * @param [out] lost_connection_s   Lost connection in second since last downlink
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p lost_connection_cnt is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_lorawan_get_lost_connection_counter_since_s( uint8_t   stack_id,
                                                                                 uint32_t* lost_connection_s );

/**
 * @brief Get the status to bypass the join_duty-cycle backoff
 *
 * @remark  The LoRaWAN certification_set enable/disable the backoff bypass
 *
 * @param [in]  stack_id                    Stack identifier
 * @param [out] enable                      True if bypass enabled
 * @return smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_join_duty_cycle_backoff_bypass( uint8_t stack_id, bool* enable );

/**
 * @brief Set the status to bypass the join_duty-cycle backoff
 * @remark  The LoRaWAN certification_set enable/disable the backoff bypass
 *
 * @param [in] stack_id                     Stack identifier
 * @param [in] enable                       True to bypass
 * @return smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_join_duty_cycle_backoff_bypass( uint8_t stack_id, bool enable );

/**
 * @brief Get the status of the LoRaWAN certification service
 *
 * @param [in]  stack_id            Stack identifier
 * @param [out] enable              True if the certification is enabled, else False
 * @return smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_certification_mode( uint8_t stack_id, bool* enable );

/**
 * @brief Enable / disable the LoRaWAN certification mode
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_request_empty_uplink( uint8_t stack_id, bool send_fport, uint8_t fport,
                                                          bool confirmed );

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
 * @brief Get the radio communications suspend status
 *
 * @param [in] stack_id     Stack identifier
 * @param [out] suspend     Get the suspend status (true: suspend communications / false: resume communications)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_suspend_radio_communications( uint8_t stack_id, bool* suspend );

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

/*
 * -----------------------------------------------------------------------------
 * ----------- NETWORK MANAGEMENT MODEM FUNCTIONS ------------------------------
 */

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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_next_tx_max_payload( uint8_t stack_id, uint8_t* tx_max_payload_size );

/**
 * @brief Get the current status of the duty cycle
 *
 * @remark If the returned value is positive, it is the time still available. A negative value indicates the time to
 * wait until band availability
 *
 * @param [in]  stack_id             Stack identifier
 * @param [out] duty_cycle_status_ms Status of the duty cycle in milliseconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p duty_cycle_status_ms is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_duty_cycle_status( uint8_t stack_id, int32_t* duty_cycle_status_ms );

/**
 * @brief Get Configured LoRaWAN network type to private or public
 *
 * @param [in]  stack_id      Stack identifier
 * @param [in]  network_type  Configuration (true: public network / false: private network)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_network_type( uint8_t stack_id, bool* network_type );

/**
 * @brief Configure LoRaWAN network type to private or public
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
 * @brief Set the custom data rate distribution for join procedure
 *
 * @remark  This function has no effect for FCC regions like US915 and AU915 and return SMTC_MODEM_RC_OK
 *
 * @param [in] stack_id        Stack identifier
 * @param [in] dr_custom_distribution_data Definition of the custom datarate distribution for join
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           One or more invalid parameters
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is already joined or is joining
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_adr_set_join_distribution(
    uint8_t stack_id, const uint8_t dr_custom_distribution_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] );

/**
 * @brief Set the adaptive data rate (ADR) profile
 *
 * @remark If @ref SMTC_MODEM_ADR_PROFILE_CUSTOM is selected, custom data are taken into account
 *
 * @param [in] stack_id        Stack identifier
 * @param [in] adr_profile     ADR profile to be configured
 * @param [in] dr_custom_distribution_data Definition of the custom datarate distribution
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           One or more invalid parameters
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_adr_set_profile(
    uint8_t stack_id, smtc_modem_adr_profile_t adr_profile,
    const uint8_t dr_custom_distribution_data[SMTC_MODEM_CUSTOM_ADR_DATA_LENGTH] );

/**
 * @brief Get the adaptive data rate (ADR) profile
 *
 * @remark If @ref SMTC_MODEM_ADR_PROFILE_CUSTOM is selected, custom data are taken into account
 *
 * @param [in] stack_id        Stack identifier
 * @param [out] adr_profile     ADR profile to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p adr_profile is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_adr_get_profile( uint8_t stack_id, smtc_modem_adr_profile_t* adr_profile );

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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
 * @brief Get the current enabled Datarate in regards of Uplink ChMash and DwellTime
 *
 * @param [in]  stack_id                 Stack identifier
 * @param [out] enabled_datarates_mask   Enabled datarates, described in a bitfield
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p enabled_datarates_mask is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_enabled_datarates( uint8_t stack_id, uint16_t* enabled_datarates_mask );

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

/*
 * -----------------------------------------------------------------------------
 * ----------- BOARD MANAGEMENT MODEM FUNCTIONS --------------------------------
 */

/**
 * @brief Get modem crystal error
 *
 * @param [out] crystal_error_ppm Crystal error in ppm
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_get_crystal_error_ppm( uint32_t* crystal_error_ppm );

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
 * @brief Enable or disable the CSMA feature
 *
 * @remark The configuration function @ref smtc_modem_csma_set_state
 * @remark The CSMA is silently enabled if the LBT feature is not mandatory in a region selected with @ref
 * smtc_modem_set_region
 *
 * @param [in] stack_id  Stack identifier
 * @param [in] enable    Status of the CSMA feature to set (true: enable, false: disable)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_csma_set_state( uint8_t stack_id, bool enable );

/**
 * @brief Get the state of the CSMA feature
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] enabled   Current status of the CSMA feature (true: enabled, false: disabled)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p enabled is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_csma_get_state( uint8_t stack_id, bool* enable );

/**
 * @brief Set the parameters of the CSMA feature
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] max_ch_change           Number of channel change when noisy before send the packet in ALOHA mode
 *                                          (default:4)
 * @param [out] bo_enabled              Enable the back-off (multiple short listen on the same channel (CAD))
 *                                          (default:false)
 * @param [out] nb_bo_max               Configure the number of short listen to check if the channel is free
 *                                          (default:6)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           At least one parameter is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_csma_set_parameters( uint8_t stack_id, uint8_t max_ch_change, bool bo_enabled,
                                                         uint8_t nb_bo_max );

/**
 * @brief Get the parameters of the CSMA feature
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] max_ch_change           Number of channel change when noisy before send the packet in ALOHA mode
 * @param [out] bo_enabled              Is the back-off enabled
 * @param [out] nb_bo_max               The number of short listen configured to check if the channel is free
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           At least one parameter is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_csma_get_parameters( uint8_t stack_id, uint8_t* max_ch_change, bool* bo_enabled,
                                                         uint8_t* nb_bo_max );

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
 * @brief Get the Radio Planner statistics in array
 *
 * @param stats_array
 * @param stats_array_length
 * @return smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_get_rp_stats_to_array( uint8_t* stats_array, uint16_t* stats_array_length );

/**
 * @brief Reset the total charge counter of the modem
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_reset_charge( void );

/*
 * -----------------------------------------------------------------------------
 * ----------- CLASS B/C MODEM FUNCTIONS ---------------------------------------
 */

/**
 * @brief Get the LoRaWAN class
 *
 * @param [in] stack_id      Stack identifier
 * @param [out] lorawan_class LoRaWAN class to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_set_class( uint8_t stack_id, smtc_modem_class_t lorawan_class );

/**
 * @brief Set Class B Ping Slot Periodicity
 *
 * @param [in] stack_id Stack identifier
 * @param [in] ping_slot_periodicity Ping slot periodicity
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                 Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY               Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL               Modem is not available (suspended, muted, or not joined)
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

/*
 * -----------------------------------------------------------------------------
 * ----------- LORAWAN PACKAGES FUNCTIONS --------------------------------------
 */

/**
 * @brief Start Application Layer Clock Synchronization (ALCSync) service
 *
 * @param [in] stack_id     Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_start_alcsync_service( uint8_t stack_id );

/**
 * @brief Stop Application Layer Clock Synchronization (ALCSync) service
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_stop_alcsync_service( uint8_t stack_id );

/**
 * @brief Get GPS epoch time - number of seconds elapsed since GPS epoch (00:00:00, Sunday 6th of January 1980).
 *
 * @param [in] stack_id    Stack identifier
 * @param [out] gps_time_s GPS time in seconds
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p gps_time_s is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_NO_TIME       No time available
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_get_alcsync_time( uint8_t stack_id, uint32_t* gps_time_s );

/**
 * @brief Trigger a single uplink requesting time on Application Layer Clock Synchronization (ALCSync) service
 *
 * @param [in] stack_id     Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined) or no ALCSync
 * service is running
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_trigger_alcsync_request( uint8_t stack_id );

/*
 * -----------------------------------------------------------------------------
 * ---------------- LORAWAN MAC REQUEST FUNCTIONS  -----------------------------
 */

/**
 * @brief Trig a LoRaWAN mac request
 *
 * @param [in] stack_id Stack identifier
 * @param [in] mac_request_mask  Mac commands mask requested by the user (SMTC_MODEM_LORAWAN_MAC_REQ_LINK_CHECK,
 *                               SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME or SMTC_MODEM_LORAWAN_MAC_REQ_PING_SLOT_INFO)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p mac_request_mask is not in acceptable range [1:7]
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_trig_lorawan_mac_request( uint8_t                               stack_id,
                                                              smtc_modem_lorawan_mac_request_mask_t mac_request_mask );

/**
 * @brief Get time from loRaWAN network. Available after the launch of LoRaWAN mac request with
 *        SMTC_MODEM_LORAWAN_MAC_REQ_DEVICE_TIME
 * @remark In GPS epoch time - number of seconds elapsed since GPS epoch (00:00:00, Sunday 6th of January 1980).
 *
 * @param [in]  stack_id         Stack identifier
 * @param [out] gps_time_s       GPS time in seconds
 * @param [out] gps_fractional_s GPS fractional second
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p gps_time_s or \p gps_fractional_s are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_NO_TIME       No time available
 */
smtc_modem_return_code_t smtc_modem_get_lorawan_mac_time( uint8_t stack_id, uint32_t* gps_time_s,
                                                          uint32_t* gps_fractional_s );

/**
 * @brief Get the link check data after a successful link_check request
 *
 * @remark The demodulation margin indicates the link margin in dB of the most recently transmitted LinkCheckReq
 * command. A value of 0 means that the frame was received at the demodulation floor (0 dB or no margin) whereas a value
 * of 20, for example, means that the frame reached the best gateway 20 dB above the demodulation floor. The value 255
 * is reserved.
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] margin   Demodulation margin
 * @param [out] gw_cnt   Number of gateways that received the most recent LinkCheckReq command
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       \p margin or \p gw_cnt are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          No data available
 */
smtc_modem_return_code_t smtc_modem_get_lorawan_link_check_data( uint8_t stack_id, uint8_t* margin, uint8_t* gw_cnt );

/*
 * -----------------------------------------------------------------------------
 * ----------- SMTC CLOUD MODEM FUNCTIONS --------------------------------------
 */

/**
 * @brief Start geolocation almanac update service (using LoRaCloud)
 * @remark This function is only allowed on geolocation compatible radios (lr1110 and lr1120)
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_almanac_start( uint8_t stack_id );

/**
 * @brief Stop geolocation almanac update service
 * @remark This function is only allowed on geolocation compatible radios (lr1110 and lr1120)

 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_almanac_stop( uint8_t stack_id );

/**
 * @brief Create and initialize a data stream
 *
 * @param [in] stack_id                  Stack identifier
 * @param [in] f_port                    LoRaWAN FPort on which the stream is sent (0 forces the DM LoRaWAN FPort)
 * @param [in] redundancy_ratio_percent  Stream redundancy ratio
 * @param [in] cipher_mode               Cipher mode
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           FPort is out of the [0:223] range
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or the streaming buffer is not empty
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p service_id
 */
smtc_modem_return_code_t smtc_modem_stream_init( uint8_t stack_id, uint8_t f_port,
                                                 smtc_modem_stream_cipher_mode_t cipher_mode,
                                                 uint8_t                         redundancy_ratio_percent );

/**
 * @brief Add data to the stream
 *
 * @remark If @ref smtc_modem_stream_init is not called beforehand, the stream uses the DM FPort with a redundancy ratio
 * set to 110%
 *
 * @param [in] stack_id                     Stack identifier
 * @param [in] data                         Data to be added to the stream
 * @param [in] len                          Number of bytes from data to be added to the stream
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           \p len is not in range [1-254] or \p data is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode or the streaming buffer is full
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p service_id
 */
smtc_modem_return_code_t smtc_modem_stream_add_data( uint8_t stack_id, const uint8_t* data, uint8_t len );

/**
 * @brief Return the current stream status
 *
 * @param [in]  stack_id                    Stack identifier
 * @param [out] pending                     Length of pending data for transmission
 * @param [out] free                        Length of free space in the buffer
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_NOT_INIT          No stream session is running
 * @retval SMTC_MODEM_RC_INVALID           \p pending or \p free are NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p service_id
 */
smtc_modem_return_code_t smtc_modem_stream_status( uint8_t stack_id, uint16_t* pending, uint16_t* free );

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
 * @brief Enable / disable Device Management service
 *
 * @param stack_id                      Stack identifier
 * @param enabled                       True: enable and start service, false: disable and stop service
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK             Command executed without errors
 */
smtc_modem_return_code_t smtc_modem_dm_enable( uint8_t stack_id, bool enabled );

/**
 * @brief Get the Device Management (DM) LoRaWAN FPort
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] dm_fport                LoRaWAN FPort on which the DM info is sent
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p dm_fport is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_fport( uint8_t stack_id, uint8_t* dm_fport );

/**
 * @brief Set the Device Management (DM) LoRaWAN FPort
 *
 * @param [in] stack_id                 Stack identifier
 * @param [in] dm_fport                 LoRaWAN FPort on which the DM info is sent.
 *                                          This value must be in the range [1:223]
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p dm_fport is out of the [1:223] range
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_fport( uint8_t stack_id, uint8_t dm_fport );

/**
 * @brief Get the interval between two Device Management (DM) info field messages
 *
 * @param [in] stack_id                 Stack identifier
 * @param [out] format                  Reporting interval format
 * @param [out] interval                Interval in unit defined in format
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameters \p format and/or \p interval are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_info_interval( uint8_t                               stack_id,
                                                          smtc_modem_dm_info_interval_format_t* format,
                                                          uint8_t*                              interval );

/**
 * @brief Set the interval between two Device Management (DM) info field messages
 *
 * @remark An interval value set to 0 disables the feature - no matter the format.
 *
 * @param [in] stack_id                 Stack identifier
 * @param [in] format                   Reporting interval format
 * @param [in] interval                 Interval in unit defined in format, from 0 to 63
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p interval is not in the [0:63] range.
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_info_interval( uint8_t stack_id, smtc_modem_dm_info_interval_format_t format,
                                                          uint8_t interval );

/**
 * @brief Get the Device Management (DM) info fields
 *
 * @param [in]  stack_id                Stack identifier
 * @param [out] dm_fields_payload       DM info fields (see @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [out] dm_field_length         DM info field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameters \p dm_fields_payload and/or \p dm_field_length are NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_periodic_info_fields( uint8_t                stack_id,
                                                                 smtc_modem_dm_field_t* dm_fields_payload,
                                                                 uint8_t*               dm_field_length );

/**
 * @brief Set the Device Management (DM) info fields to be sent on a regular basis
 *
 * @remark The interval between two DM info field messages is defined with @ref smtc_modem_dm_set_info_interval
 *
 * @param [in] stack_id                 Stack identifier
 * @param [in] dm_fields_payload        DM info fields (see @ref SMTC_MODEM_DM_INFO_DEF)
 * @param [in] dm_field_length          DM info field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Invalid or duplicated DM info fields
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_periodic_info_fields( uint8_t                      stack_id,
                                                                 const smtc_modem_dm_field_t* dm_fields_payload,
                                                                 uint8_t                      dm_field_length );

/**
 * @brief Request an immediate Device Management (DM) status
 *
 * @remark The content is independent from the configuration set with @ref smtc_modem_dm_set_periodic_info_fields
 *
 * @param [in] stack_id                 Stack identifier
 * @param [in] dm_fields_payload        DM info fields (see @ref smtc_modem_dm_field_t)
 * @param [in] dm_field_length          DM info field length
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Invalid or duplicated field code or \p dm_fields_payload is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          Modem is not available (suspended, muted, or not joined)
 */
smtc_modem_return_code_t smtc_modem_dm_request_immediate_info_field( uint8_t                      stack_id,
                                                                     const smtc_modem_dm_field_t* dm_fields_payload,
                                                                     uint8_t                      dm_field_length );

/**
 * @brief Set user-specific data to be reported by Device Management (DM) frames
 *
 * @remark This field will be sent only if it is selected in @ref smtc_modem_dm_set_periodic_info_fields or @ref
 * smtc_modem_dm_request_immediate_info_field
 *
 * @param [in] stack_id                 Stack identifier
 * @param [in] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_set_user_data( uint8_t       stack_id,
                                                      const uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Get user-specific data to be reported by Device Management (DM) frames
 *
 * @param [in] stack_id                 Stack identifier
 * @param [out] user_data User-specific data
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p user_data is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 */
smtc_modem_return_code_t smtc_modem_dm_get_user_data( uint8_t stack_id,
                                                      uint8_t user_data[SMTC_MODEM_DM_USER_DATA_LENGTH] );

/**
 * @brief Handle ALCSync service using cloud and device management
 * @remark This configuration function shall be called before the ALCSync service is started
 *
 * @param [in] stack_id Stack identifier
 * @param [in] handle_alcsync True to handle ALCSync traffic on dm frame, false otherwise
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL          ALCSync service is already running
 * */
smtc_modem_return_code_t smtc_modem_dm_handle_alcsync( uint8_t stack_id, bool handle_alcsync );

/*
 * -----------------------------------------------------------------------------
 * ----------- MISCELLANEOUS MODEM FUNCTIONS -----------------------------------
 */

/**
 * @brief Set the state of the store and forward service
 *
 * @param [in] stack_id Stack identifier
 * @param [in] state    State of the service as define in @ref smtc_modem_store_and_forward_state_t
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_store_and_forward_set_state( uint8_t                              stack_id,
                                                                 smtc_modem_store_and_forward_state_t state );

/**
 * @brief Get the current state of store and forward service
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] state    Current state of the service
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID       Parameter \p state is NULL
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_store_and_forward_get_state( uint8_t                               stack_id,
                                                                 smtc_modem_store_and_forward_state_t* state );

/**
 * @brief Add data to the store and forward service
 *
 * @param [in] stack_id       Stack identifier
 * @param [in] fport          LoRaWAN FPort on which the uplink will be done
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
smtc_modem_return_code_t smtc_modem_store_and_forward_flash_add_data( uint8_t stack_id, uint8_t fport, bool confirmed,
                                                                      const uint8_t* payload, uint8_t payload_length );

/**
 * @brief Clear all data stored in flash
 *
 * @param [in] stack_id Stack identifier
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK            Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY          Modem is currently in test mode
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_store_and_forward_flash_clear_data( uint8_t stack_id );

/**
 * @brief Get the fifo capacity and the number of free slots before data loss by overwriting the slot already in use
 *
 * @param [in]  stack_id  Stack identifier
 * @param [out] capacity  Capacity of the fifo (number of slot)
 * @param [out] free_slot Number of free slots
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           Parameters \p capacity and/or \p free_slot are NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is not available (suspended, muted, or not joined)
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_store_and_forward_flash_get_number_of_free_slot( uint8_t   stack_id,
                                                                                     uint32_t* capacity,
                                                                                     uint32_t* free_slot );

/*
 * -----------------------------------------------------------------------------
 * ---------------------- DEBUG PURPOSE FUNCTIONS  -----------------------------
 */

/**
 * @brief  Enable or disable duty cycle feature
 *
 * @param [in] enable Status of the duty cycle feature to set (true: enable, false: disable)
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Duty cycle feature is not ready
 */
smtc_modem_return_code_t smtc_modem_debug_set_duty_cycle_state( bool enable );

/**
 * @brief join network in ABP
 *
 * @param [in]  stack_id    Stack identifier
 * @param [in]  dev_addr    The network device address
 * @param [in]  nwk_skey    Network Session Key to be configured
 * @param [in]  app_skey    Application Session Key to be configured
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK                Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID           At least \p nwk_skey or \p nwk_sapp_skeykey is NULL
 * @retval SMTC_MODEM_RC_BUSY              Modem is currently in test mode
 * @retval SMTC_MODEM_RC_FAIL              Modem is already joined
 * @retval SMTC_MODEM_RC_INVALID_STACK_ID  Invalid \p stack_id
 */
smtc_modem_return_code_t smtc_modem_debug_connect_with_abp( uint8_t stack_id, uint32_t dev_addr,
                                                            uint8_t nwk_skey[SMTC_MODEM_KEY_LENGTH],
                                                            uint8_t app_skey[SMTC_MODEM_KEY_LENGTH] );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_API_H__

/* --- EOF ------------------------------------------------------------------ */
