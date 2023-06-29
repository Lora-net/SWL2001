/*!
 * \file      lr1mac_defs.h
 *
 * \brief     LoRaWan stack mac layer types definition
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

#ifndef __LR1MAC_DEFS_H__
#define __LR1MAC_DEFS_H__
#ifdef __cplusplus
extern "C" {
#endif

/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>
#include "ral_defs.h"
#include "smtc_secure_element.h"

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS -------------------------------------------------------------
 */
// clang-format off
#define LORAWAN_VERSION_MAJOR           (1)
#define LORAWAN_VERSION_MINOR           (0)
#define LORAWAN_VERSION_PATCH           (4)
#define LORAWAN_VERSION_REVISION        (1)

// never use #define LINK_CHECK_REQ_SIZE
#define LINK_CHECK_REQ_SIZE             (1)
#define LINK_CHECK_ANS_SIZE             (3)
#define LINK_ADR_REQ_SIZE               (5)
#define LINK_ADR_ANS_SIZE               (2)
#define DUTY_CYCLE_REQ_SIZE             (2)
#define DUTY_CYCLE_ANS_SIZE             (1)
#define RXPARRAM_SETUP_REQ_SIZE         (5)
#define RXPARRAM_SETUP_ANS_SIZE         (2)
#define DEV_STATUS_REQ_SIZE             (1)
#define DEV_STATUS_ANS_SIZE             (3)
#define NEW_CHANNEL_REQ_SIZE            (6)
#define NEW_CHANNEL_ANS_SIZE            (2)
#define RXTIMING_SETUP_REQ_SIZE         (2)
#define RXTIMING_SETUP_ANS_SIZE         (1)
#define TXPARAM_SETUP_REQ_SIZE          (2)
#define TXPARAM_SETUP_ANS_SIZE          (1)
#define DL_CHANNEL_REQ_SIZE             (5)
#define DL_CHANNEL_ANS_SIZE             (2)
#define DEVICE_TIME_REQ_SIZE            (1)
#define DEVICE_TIME_ANS_SIZE            (6)
#define PING_SLOT_INFO_REQ_SIZE         (2)
#define PING_SLOT_INFO_ANS_SIZE         (1)
#define PING_SLOT_CHANNEL_REQ_SIZE      (5)
#define PING_SLOT_CHANNEL_ANS_SIZE      (2)
#define BEACON_FREQ_REQ_SIZE            (4)
#define BEACON_FREQ_ANS_SIZE            (2)
#define MAX_RETRY_JOIN_DUTY_CYCLE_100   (10)
#define MAX_RETRY_JOIN_DUTY_CYCLE_1000  (10 + MAX_RETRY_JOIN_DUTY_CYCLE_100)
#define MIN_LORAWAN_PAYLOAD_SIZE        (12)
#define PORTNWK                         (0)
#define MAX_TX_PAYLOAD_SIZE             (255)
#define FHDROFFSET                      (8)  // MHDR+FHDR offset if OPT = 0
#define MICSIZE                         (4)
#define LR1MAC_PROTOCOL_VERSION         (0x00010300)
#define GFSK_WHITENING_SEED             (0x01FF)
#define GFSK_CRC_SEED                   (0x1D0F)
#define GFSK_CRC_POLYNOMIAL             (0x1021)

#define SMTC_LR1MAC_DEVNONCE_SAVE_PERIOD ( 1 )

#define LR1MAC_DEVICE_TIME_DELAY_TO_BE_NO_SYNC (4233600UL)  // 49 days -> 49×24×60×60

#define NWK_MAC_PAYLOAD_MAX_SIZE                                                                                   \
    ( LINK_CHECK_ANS_SIZE + ( LINK_ADR_REQ_SIZE * 8 ) + DUTY_CYCLE_REQ_SIZE + RXPARRAM_SETUP_REQ_SIZE +            \
      DEV_STATUS_REQ_SIZE + ( NEW_CHANNEL_REQ_SIZE * 16 ) + RXTIMING_SETUP_REQ_SIZE + TXPARAM_SETUP_REQ_SIZE +     \
      ( DL_CHANNEL_REQ_SIZE * 16 ) + DEVICE_TIME_ANS_SIZE + PING_SLOT_INFO_ANS_SIZE + PING_SLOT_CHANNEL_REQ_SIZE + \
      BEACON_FREQ_REQ_SIZE )
#if NWK_MAC_PAYLOAD_MAX_SIZE > 242
#undef NWK_MAC_PAYLOAD_MAX_SIZE
#define NWK_MAC_PAYLOAD_MAX_SIZE 242
#endif

#define DEVICE_MAC_PAYLOAD_MAX_SIZE                                                                                \
    ( LINK_CHECK_REQ_SIZE + ( LINK_ADR_ANS_SIZE * 8 ) + DUTY_CYCLE_ANS_SIZE + RXPARRAM_SETUP_ANS_SIZE +            \
      DEV_STATUS_ANS_SIZE + ( NEW_CHANNEL_ANS_SIZE * 16 ) + RXTIMING_SETUP_ANS_SIZE + TXPARAM_SETUP_ANS_SIZE +     \
      ( DL_CHANNEL_ANS_SIZE * 16 ) + DEVICE_TIME_REQ_SIZE + PING_SLOT_INFO_REQ_SIZE + PING_SLOT_CHANNEL_ANS_SIZE + \
      BEACON_FREQ_ANS_SIZE )
#if DEVICE_MAC_PAYLOAD_MAX_SIZE > 242
#undef DEVICE_MAC_PAYLOAD_MAX_SIZE
#define DEVICE_MAC_PAYLOAD_MAX_SIZE 242
#endif

// if there were no rx packet before the last LR1MAC_NO_RX_PACKET_RESET_THRESHOLD tx packets the lr1mac goes in panic
#define LR1MAC_NO_RX_PACKET_RESET_THRESHOLD                (2400)



// #define MAX_FCNT_GAP 16384

// clang-format on

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC MACROS ----------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC TYPES ----------------------------------------------------------------
 */

// Frame direction definition for up/down link communications
typedef enum direction_frame_e
{
    UP_LINK   = 0,
    DOWN_LINK = 1
} direction_frame_t;

typedef enum lr1mac_states_e
{
    LWPSTATE_IDLE,
    LWPSTATE_SEND,
    LWPSTATE_RX1,
    LWPSTATE_RX2,
    LWPSTATE_TX_WAIT,
    LWPSTATE_INVALID,
    LWPSTATE_ERROR,
    LWPSTATE_DUTY_CYCLE_FULL,
    LWPSTATE_BUSY,
} lr1mac_states_t;

/*****************************************************************************/
/*                                 Radio Process States                      */
/*****************************************************************************/
typedef enum lr1mac_radio_state_e
{
    RADIOSTATE_IDLE,
    RADIOSTATE_PENDING,
    RADIOSTATE_TX_ON,
    RADIOSTATE_TX_FINISHED,
    RADIOSTATE_RX_ON,
    RADIOSTATE_RX_FINISHED,
    RADIOSTATE_ABORTED_BY_RP,
} lr1mac_radio_state_t;
/********************************************************************************/
/*                   LoraWan Mac Layer Parameters                               */
/********************************************************************************/
typedef enum lr1mac_layer_param_e
{
    JOIN_REQUEST,
    JOIN_ACCEPT,
    UNCONF_DATA_UP,
    UNCONF_DATA_DOWN,
    CONF_DATA_UP,
    CONF_DATA_DOWN,
    REJOIN_REQUEST,
    PROPRIETARY,
} lr1mac_layer_param_t;

typedef enum lr1mac_layer_dl_fctrl_bits_e
{
    DL_FPENDING_BIT = 4,
    DL_ACK_BIT      = 5,
    DL_RFU          = 6,
    DL_ADR_BIT      = 7,
} lr1mac_layer_dl_fctrl_bits_t;

enum
{
    LORAWANR1,
    RFU,
};

typedef enum lr1mac_activation_mode_e
{
    ACTIVATION_MODE_OTAA,
    ACTIVATION_MODE_ABP,
} lr1mac_activation_mode_t;

typedef enum cid_from_network_e
{
    LINK_CHECK_ANS     = 0x02,
    LINK_ADR_REQ       = 0x03,
    DUTY_CYCLE_REQ     = 0x04,
    RXPARRAM_SETUP_REQ = 0x05,
    DEV_STATUS_REQ     = 0x06,
    NEW_CHANNEL_REQ    = 0x07,
    RXTIMING_SETUP_REQ = 0x08,
    TXPARAM_SETUP_REQ  = 0x09,
    DL_CHANNEL_REQ     = 0x0A,
    DEVICE_TIME_ANS    = 0x0D,
    // Class B
    PING_SLOT_INFO_ANS    = 0x10,
    PING_SLOT_CHANNEL_REQ = 0x11,
    BEACON_FREQ_REQ       = 0x13,
    NB_MAC_CMD_REQ
} cid_from_network_t;

typedef enum cid_from_device_e
{
    LINK_CHECK_REQ     = 0x02,
    LINK_ADR_ANS       = 0x03,
    DUTY_CYCLE_ANS     = 0x04,
    RXPARRAM_SETUP_ANS = 0x05,
    DEV_STATUS_ANS     = 0x06,
    NEW_CHANNEL_ANS    = 0x07,
    RXTIMING_SETUP_ANS = 0x08,
    TXPARAM_SETUP_ANS  = 0x09,
    DL_CHANNEL_ANS     = 0x0A,
    DEVICE_TIME_REQ    = 0x0D,
    // Class B
    PING_SLOT_INFO_REQ    = 0x10,
    PING_SLOT_CHANNEL_ANS = 0x11,
    BEACON_FREQ_ANS       = 0x13,
    NB_MAC_CMD_ANS
} cid_from_device_t;

static const uint8_t lr1mac_cmd_mac_ans_size[NB_MAC_CMD_ANS] = {
    [LINK_CHECK_ANS] = LINK_CHECK_ANS_SIZE,         [LINK_ADR_ANS] = LINK_ADR_ANS_SIZE,
    [DUTY_CYCLE_ANS] = DUTY_CYCLE_ANS_SIZE,         [RXPARRAM_SETUP_ANS] = RXPARRAM_SETUP_ANS_SIZE,
    [DEV_STATUS_ANS] = DEV_STATUS_ANS_SIZE,         [NEW_CHANNEL_ANS] = NEW_CHANNEL_ANS_SIZE,
    [RXTIMING_SETUP_ANS] = RXTIMING_SETUP_ANS_SIZE, [TXPARAM_SETUP_ANS] = TXPARAM_SETUP_ANS_SIZE,
    [DL_CHANNEL_ANS] = DL_CHANNEL_ANS_SIZE,
};

typedef enum lr1mac_bandwidth_e
{
    BW125  = RAL_LORA_BW_125_KHZ,
    BW250  = RAL_LORA_BW_250_KHZ,
    BW500  = RAL_LORA_BW_500_KHZ,
    BW800  = RAL_LORA_BW_800_KHZ,
    BW1600 = RAL_LORA_BW_1600_KHZ,
    BW_RFU = 255,
} lr1mac_bandwidth_t;

typedef enum lr1mac_datarate_e
{
    DR0 = 0,
    DR1,
    DR2,
    DR3,
    DR4,
    DR5,
    DR6,
    DR7,
    DR8,
    DR9,
    DR10,
    DR11,
    DR12,
    DR13,
    DR14,
    DR15,
} lr1mac_datarate_t;
enum
{
    CHANNEL_DISABLED,
    CHANNEL_ENABLED,
};

// User Config for Adr Mode select
typedef enum dr_strategy_e
{
    STATIC_ADR_MODE,                   // for static Devices with ADR managed by the Network
    MOBILE_LONGRANGE_DR_DISTRIBUTION,  // for Mobile Devices with strong Long range requirement
    MOBILE_LOWPER_DR_DISTRIBUTION,     // for Mobile Devices with strong Low power requirement
    USER_DR_DISTRIBUTION,              // User distribution
    JOIN_DR_DISTRIBUTION,              // Dedicated for Join requests
    UNKNOWN_DR,
} dr_strategy_t;

typedef enum status_lorawan_e
{
    ERRORLORAWAN = -1,
    OKLORAWAN    = 0,
} status_lorawan_t;

typedef enum status_channel_e
{
    ERROR_CHANNEL      = -3,
    ERROR_CHANNEL_CNTL = -2,
    ERROR_CHANNEL_MASK = -1,
    OKCHANNEL          = 0,
} status_channel_t;
typedef enum rx_packet_type_e
{
    NO_MORE_VALID_RX_PACKET,
    USER_RX_PACKET,
    USERRX_FOPTSPACKET,
    NWKRXPACKET,
    JOIN_ACCEPT_PACKET,
} rx_packet_type_t;
typedef enum rx_win_type_e
{
    RX1 = 0,
    RX2,
} rx_win_type_t;

/**
 * @brief LR-FHSS header count
 */
typedef enum lr_fhss_hc_e
{
    LR_FHSS_HC_2 = 0x02,
    LR_FHSS_HC_3 = 0x03,
} lr_fhss_hc_t;

/*************************/
/*    SHARE WITH USER    */
/*************************/
typedef enum user_rx_packet_type_e
{
    NO_LORA_RXPACKET_AVAILABLE,
    LORA_RX_PACKET_AVAILABLE,
} user_rx_packet_type_t;

typedef enum join_status_e
{
    NOT_JOINED,
    JOINING,
    JOINED,
} join_status_t;

typedef enum lora_frame_type_e
{
    NOFRAME_TOSEND,
    NWKFRAME_TOSEND,
    USERACK_TOSEND,
    USRFRAME_TORETRANSMIT,
} lora_frame_type_t;

typedef enum modulation_type_e
{
    LORA,
    FSK,
    LR_FHSS,
} modulation_type_t;

/**
 * @brief Rx Session type enum
 *
 */
typedef enum rx_session_type_e
{
    RX_SESSION_UNICAST,
#if defined( SMTC_MULTICAST ) || defined( SMTC_D2D )
    RX_SESSION_MULTICAST_G0,
    RX_SESSION_MULTICAST_G1,
    RX_SESSION_MULTICAST_G2,
    RX_SESSION_MULTICAST_G3,
#endif
    RX_SESSION_COUNT,
} rx_session_type_t;

typedef enum user_mac_req_status_e
{
    USER_MAC_REQ_NOT_REQUESTED = 0,
    USER_MAC_REQ_REQUESTED     = 1,
    USER_MAC_REQ_SENT          = 2,
    USER_MAC_REQ_ACKED         = 3,
} user_mac_req_status_t;

typedef enum smtc_multicast_fpending_bit_prioritization_e
{
    UNICAST_WO_FPENDING   = 0,
    MULTICAST_WO_FPENDING = 1,
    UNICAST_FPENDING      = 2,
    MULTICAST_FPENDING    = 3,
} smtc_multicast_fpending_bit_prioritization_t;

/********************************************************************************/
/*                         LORA Metadata                                        */
/********************************************************************************/
typedef enum receive_win_s
{
    RECEIVE_NONE   = 0,
    RECEIVE_ON_RX1 = 1,
    RECEIVE_ON_RX2 = 2,
    RECEIVE_ON_RXC = 3,
#if defined( SMTC_MULTICAST )
    RECEIVE_ON_RXC_MC_GRP0 = 4,
    RECEIVE_ON_RXC_MC_GRP1 = 5,
    RECEIVE_ON_RXC_MC_GRP2 = 6,
    RECEIVE_ON_RXC_MC_GRP3 = 7,
#endif
    RECEIVE_ON_RXB = 8,
#if defined( SMTC_MULTICAST )
    RECEIVE_ON_RXB_MC_GRP0 = 9,
    RECEIVE_ON_RXB_MC_GRP1 = 10,
    RECEIVE_ON_RXB_MC_GRP2 = 11,
    RECEIVE_ON_RXB_MC_GRP3 = 12,
#endif
    RECEIVE_ON_RXBEACON = 13,
} receive_win_t;

typedef struct lr1mac_down_metadata_s
{
    uint32_t      timestamp;
    int16_t       rx_snr;
    int16_t       rx_rssi;
    uint8_t       rx_fport;
    receive_win_t rx_window;
    bool          rx_fpending_bit;
    uint32_t      rx_frequency_hz;
    uint8_t       rx_datarate;
} lr1mac_down_metadata_t;

typedef struct smtc_ping_slot_parameters_e
{
    uint16_t ping_period;  // Period of the end-device receiver wake-up expressed in number of slots:
    uint8_t  ping_number;  // Number of ping slot in a beacon window
    uint32_t ping_offset_time;
    uint32_t ping_offset_time_100us;
} smtc_ping_slot_parameters_t;

typedef struct lr1mac_rx_session_param_s
{
    bool                     enabled;
    uint32_t                 dev_addr;
    uint32_t                 fcnt_dwn;
    smtc_se_key_identifier_t nwk_skey;
    smtc_se_key_identifier_t app_skey;
    uint32_t                 rx_frequency;
    uint8_t                  rx_data_rate;
    uint16_t                 rx_window_symb;

    // For class B
    uint8_t                                      ping_slot_periodicity;  // Value set by the user [0 to 7]
    smtc_ping_slot_parameters_t                  ping_slot_parameters;
    smtc_multicast_fpending_bit_prioritization_t fpending_bit;
    bool                                         waiting_beacon_to_start;
} lr1mac_rx_session_param_t;

/********************************************************************************/
/*                    Mac Context and counter Context                           */
/********************************************************************************/

typedef struct mac_context_s
{
    uint8_t  not_used_1[8];
    uint8_t  not_used_2[8];
    uint8_t  not_used_3[16];
    uint32_t adr_custom;
    uint8_t  region_type;
    uint8_t  certification_enabled;
    uint8_t  rfu[11];  // bytes reserved for future used
    uint32_t crc;      // !! crc MUST be the last field of the structure !!
} mac_context_t;

typedef struct lr1_counter_context_s
{
    uint16_t devnonce;
    uint32_t nb_reset;
    uint8_t  join_nonce[6];
    uint32_t rfu[3];  // bytes reserved for future used
    uint32_t crc;     // !! crc MUST be the last field of the structure !!
} lr1_counter_context_t;

typedef enum cf_list_type
{
    CF_LIST_FREQ = 0,
    CF_LIST_CH_MASK,
} cf_list_type_t;

/********************************************************************************/
/*                         LoRaWAN Version                                      */
/********************************************************************************/
typedef struct lr1mac_version_s
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t revision;
} lr1mac_version_t;

#ifdef __cplusplus
}
#endif

#endif  // __LR1MAC_DEFS_H__
