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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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

/*
 * -----------------------------------------------------------------------------
 * ----------- ADVANCED MODEM FUNCTIONS ----------------------------------------
 */

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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
smtc_modem_return_code_t smtc_modem_csma_set_parameters( uint8_t stack_id, uint8_t nb_bo_max, bool bo_enabled,
                                                         uint8_t max_ch_change );

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
 * @retval SMTC_MODEM_RC_FAIL               Modem is not joined
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined or no ALCSync service is running
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
 * @retval SMTC_MODEM_RC_FAIL              Modem is not joined
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
