/*!
 * \file      cmd_parser.h
 *
 * \brief     Command parser for hw modem over lora basics modem
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
#ifndef CMD_PARSER_H__
#define CMD_PARSER_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Host command opcode definition
 */
typedef enum host_cmd_id_e
{
    CMD_RESET                                = 0x00,
    CMD_SET_REGION                           = 0x01,
    CMD_GET_REGION                           = 0x02,
    CMD_JOIN_NETWORK                         = 0x03,
    CMD_REQUEST_UPLINK                       = 0x04,
    CMD_GET_EVENT                            = 0x05,
    CMD_GET_DOWNLINK_DATA                    = 0x06,
    CMD_GET_DOWNLINK_METADATA                = 0x07,
    CMD_GET_JOIN_EUI                         = 0x08,
    CMD_SET_JOIN_EUI                         = 0x09,
    CMD_GET_DEV_EUI                          = 0x0A,
    CMD_SET_DEV_EUI                          = 0x0B,
    CMD_SET_NWKKEY                           = 0x0C,
    CMD_GET_PIN                              = 0x0D,
    CMD_GET_CHIP_EUI                         = 0x0E,
    CMD_DERIVE_KEYS                          = 0x0F,
    CMD_GET_MODEM_VERSION                    = 0x10,
    CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER  = 0x11,
    CMD_SET_CERTIFICATION_MODE               = 0x12,
    CMD_EMERGENCY_UPLINK                     = 0x13,
    CMD_REQUEST_EMPTY_UPLINK                 = 0x14,
    CMD_LEAVE_NETWORK                        = 0x15,
    CMD_ALARM_START_TIMER                    = 0x16,
    CMD_ALARM_CLEAR_TIMER                    = 0x17,
    CMD_ALARM_GET_REMAINING_TIME             = 0x18,
    CMD_GET_NEXT_TX_MAX_PAYLOAD              = 0x19,
    CMD_GET_DUTY_CYCLE_STATUS                = 0x1A,
    CMD_SET_NETWORK_TYPE                     = 0x1B,
    CMD_SET_JOIN_DR_DISTRIBUTION             = 0x1C,
    CMD_SET_ADR_PROFILE                      = 0x1D,
    CMD_SET_NB_TRANS                         = 0x1E,
    CMD_GET_NB_TRANS                         = 0x1F,
    CMD_GET_ENABLED_DATARATE                 = 0x20,
    CMD_SET_ADR_ACK_LIMIT_DELAY              = 0x21,
    CMD_GET_ADR_ACK_LIMIT_DELAY              = 0x22,
    CMD_SET_CRYSTAL_ERR                      = 0x23,
    CMD_LBT_SET_PARAMS                       = 0x24,
    CMD_LBT_GET_PARAMS                       = 0x25,
    CMD_LBT_SET_STATE                        = 0x26,
    CMD_LBT_GET_STATE                        = 0x27,
    CMD_GET_CHARGE                           = 0x28,
    CMD_RESET_CHARGE                         = 0x29,
    CMD_SET_CLASS                            = 0x2A,
    CMD_CLASS_B_SET_PING_SLOT_PERIODICITY    = 0x2B,
    CMD_CLASS_B_GET_PING_SLOT_PERIODICITY    = 0x2C,
    CMD_MULTICAST_SET_GROUP_CONFIG           = 0x2D,
    CMD_MULTICAST_GET_GROUP_CONFIG           = 0x2E,
    CMD_MULTICAST_CLASS_C_START_SESSION      = 0x2F,
    CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS = 0x30,
    CMD_MULTICAST_CLASS_C_STOP_SESSION       = 0x31,
    CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS  = 0x32,
    CMD_MULTICAST_CLASS_B_START_SESSION      = 0x33,
    CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS = 0x34,
    CMD_MULTICAST_CLASS_B_STOP_SESSION       = 0x35,
    CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS  = 0x36,
    CMD_START_ALCSYNC_SERVICE                = 0x37,
    CMD_STOP_ALCSYNC_SERVICE                 = 0x38,
    CMD_GET_ALCSYNC_TIME                     = 0x39,
    CMD_TRIG_ALCSYNC_REQUEST                 = 0x3A,
    CMD_LORAWAN_MAC_REQUEST                  = 0x3B,
    CMD_GET_LORAWAN_TIME                     = 0x3C,
    CMD_GET_LINK_CHECK_DATA                  = 0x3D,
    CMD_SET_DUTY_CYCLE_STATE                 = 0x3E,
    CMD_DEBUG_CONNECT_WITH_ABP               = 0x3F,
    CMD_TEST                                 = 0x40,
    CMD_SET_TX_POWER_OFFSET                  = 0x41,
    CMD_GET_TX_POWER_OFFSET                  = 0x42,
    CMD_CSMA_SET_STATE                       = 0x43,
    CMD_CSMA_GET_STATE                       = 0x44,
    CMD_CSMA_SET_PARAMETERS                  = 0x45,
    CMD_CSMA_GET_PARAMETERS                  = 0x46,
    CMD_STREAM_INIT                          = 0x47,
    CMD_STREAM_ADD_DATA                      = 0x48,
    CMD_STREAM_STATUS                        = 0x49,
    CMD_LFU_INIT                             = 0x4A,
    CMD_LFU_DATA                             = 0x4B,
    CMD_LFU_START                            = 0x4C,
    CMD_LFU_RESET                            = 0x4D,
    CMD_DM_ENABLE                            = 0x4E,
    CMD_DM_GET_PORT                          = 0x4F,
    CMD_DM_SET_PORT                          = 0x50,
    CMD_DM_GET_INFO_INTERVAL                 = 0x51,
    CMD_DM_SET_INFO_INTERVAL                 = 0x52,
    CMD_DM_GET_PERIODIC_INFO_FIELDS          = 0x53,
    CMD_DM_SET_PERIODIC_INFO_FIELDS          = 0x54,
    CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS     = 0x55,
    CMD_DM_SET_USER_DATA                     = 0x56,
    CMD_DM_GET_USER_DATA                     = 0x57,
    CMD_MAX
} host_cmd_id_t;

/**
 * @brief Host test command opcode definition
 */
typedef enum host_cmd_test_id_e
{
    CMD_TST_START                = 0x00,
    CMD_TST_NOP                  = 0x01,
    CMD_TST_TX_SINGLE            = 0x02,
    CMD_TST_TX_CONT              = 0x03,
    CMD_TST_TX_HOP               = 0x04,
    CMD_TST_NA_1                 = 0x05,
    CMD_TST_TX_CW                = 0x06,
    CMD_TST_RX_CONT              = 0x07,
    CMD_TST_RSSI                 = 0x08,
    CMD_TST_RADIO_RST            = 0x09,
    CMD_TST_EXIT                 = 0x0B,
    CMD_TST_BUSYLOOP             = 0x0C,
    CMD_TST_PANIC                = 0x0D,
    CMD_TST_WATCHDOG             = 0x0E,
    CMD_TST_RADIO_READ           = 0x0F,
    CMD_TST_RADIO_WRITE          = 0x10,
    CMD_TST_TX_SINGLE_PREAM      = 0x14,
    CMD_TST_RSSI_GET             = 0x15,
    CMD_TST_READ_NB_PKTS_RX_CONT = 0x18,
    CMD_TST_MAX
} host_cmd_test_id_t;

/**
 * @brief Command parser serial return code
 */
typedef enum cmd_serial_rc_code_e
{
    CMD_RC_OK               = 0x00,
    CMD_RC_UNKNOWN          = 0x01,
    CMD_RC_NOT_IMPLEMENTED  = 0x02,
    CMD_RC_NOT_INIT         = 0x03,
    CMD_RC_INVALID          = 0x04,
    CMD_RC_BUSY             = 0x05,
    CMD_RC_FAIL             = 0x06,
    CMD_RC_BAD_CRC          = 0x08,
    CMD_RC_BAD_SIZE         = 0x0A,
    CMD_RC_FRAME_ERROR      = 0x0F,
    CMD_RC_NO_TIME          = 0x10,
    CMD_RC_INVALID_STACK_ID = 0x11,
    CMD_RC_NO_EVENT         = 0x12,
} cmd_serial_rc_code_t;

/**
 * @brief Input command structure
 */
typedef struct cmd_input_e
{
    host_cmd_id_t cmd_code;
    uint8_t       length;
    uint8_t*      buffer;
} cmd_input_t;

/**
 * @brief Command response struture
 */
typedef struct cmd_response_e
{
    cmd_serial_rc_code_t return_code;
    uint8_t              length;
    uint8_t*             buffer;
} cmd_response_t;

/**
 * @brief Input test command structure
 */
typedef struct cmd_tst_response_e
{
    cmd_serial_rc_code_t return_code;
    uint8_t              length;
    uint8_t*             buffer;
} cmd_tst_response_t;

/**
 * @brief Test command response struture
 */
typedef struct cmd_tst_input_e
{
    host_cmd_test_id_t cmd_code;
    uint8_t            length;
    uint8_t*           buffer;
} cmd_tst_input_t;

/**
 * @brief Command parser status
 */
typedef enum cmd_parse_status_e
{
    PARSE_ERROR,
    PARSE_OK,
} cmd_parse_status_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Parse command received on serial link
 *
 * @param [in]  cmd_input  Contains the command received
 * @param [out] cmd_output Contains the response to the received command
 * @return cmd_parse_status_t
 */
cmd_parse_status_t parse_cmd( cmd_input_t* cmd_input, cmd_response_t* cmd_output );

/**
 * @brief Parse test command received on serial link
 *
 * @param [in] cmd_tst_input   Contains the command received
 * @param [out] cmd_tst_output Contains the response to the received command
 * @return cmd_parse_status_t
 */
cmd_parse_status_t cmd_test_parser( cmd_tst_input_t* cmd_tst_input, cmd_tst_response_t* cmd_tst_output );

#ifdef __cplusplus
}
#endif

#endif  // CMD_PARSER_H__

/* --- EOF ------------------------------------------------------------------ */
