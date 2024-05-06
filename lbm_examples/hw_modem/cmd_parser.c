/*!
 * \file      cmd_parser.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "cmd_parser.h"

#include "smtc_modem_test_api.h"
#include "smtc_modem_api.h"
#if defined( ADD_APP_GEOLOCATION )
#include "smtc_modem_geolocation_api.h"
#include "lr11xx_hal.h"
#endif

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "modem_pinout.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_mcu.h"

#include "radio_utilities.h"

#include <string.h>  //for memset

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define STACK_ID 0
#if defined( STM32L073xx )
#define FILE_UPLOAD_MAX_SIZE 4096
#else
#define FILE_UPLOAD_MAX_SIZE 8192
#endif

#define MODEM_MAX_INFO_FIELD_SIZE 19
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Host command tab index
 */
typedef enum host_cmd_tab_idx_e
{
    HOST_CMD_TAB_IDX_AVAILABILITY = 0,
    HOST_CMD_TAB_IDX_MIN_LENGTH   = 1,
    HOST_CMD_TAB_IDX_MAX_LENGTH   = 2,
    HOST_CMD_TAB_IDX_COUNT,
} host_cmd_tab_idx_t;

/**
 * @brief Command length status
 */
typedef enum cmd_length_valid_e
{
    CMD_LENGTH_VALID,
    CMD_LENGTH_NOT_VALID,
} cmd_length_valid_t;

typedef enum upload_status_e
{
    UPLOAD_NOT_INIT,
    UPLOAD_INIT,
    UPLOAD_DATA_ON_GOING,
    UPLOAD_STARTED,
} upload_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static bool                     modem_in_test_mode = false;
static smtc_modem_dl_metadata_t last_dl_metadata   = { 0 };

// LFU handling
static uint8_t         file_store[FILE_UPLOAD_MAX_SIZE] = { 0 };
static uint16_t        file_size                        = 0;
static uint16_t        upload_current_size              = 0;
static upload_status_t upload_status                    = UPLOAD_NOT_INIT;

#if defined( ADD_APP_GEOLOCATION ) && defined( STM32L476xx )
// Geolocation handling
static smtc_modem_gnss_event_data_scan_done_t gnss_scan_data    = { 0 };
static cmd_serial_rc_code_t                   gnss_scan_done_rc = CMD_RC_FAIL;
#endif  // ADD_APP_GEOLOCATION && STM32L476xx

/**
 * @brief Modem commands tab for availability, min length and max length
 *
 */
static const uint8_t host_cmd_tab[CMD_MAX][HOST_CMD_TAB_IDX_COUNT] = {
//  [CMD_xxx]                                   = {availability, len_min, len_max}
    [CMD_RESET]                                 = { 1, 0, 0 },
    [CMD_SET_REGION]                            = { 1, 1, 1 },
    [CMD_GET_REGION]                            = { 1, 0, 0 },
    [CMD_JOIN_NETWORK]                          = { 1, 0, 0 },
    [CMD_REQUEST_UPLINK]                        = { 1, 2, 244 },
    [CMD_GET_EVENT]                             = { 1, 0, 0 },
    [CMD_GET_DOWNLINK_DATA]                     = { 1, 0, 0 },
    [CMD_GET_DOWNLINK_METADATA]                 = { 1, 0, 0 },
    [CMD_GET_JOIN_EUI]                          = { 1, 0, 0 },
    [CMD_SET_JOIN_EUI]                          = { 1, 8, 8 },
    [CMD_GET_DEV_EUI]                           = { 1, 0, 0 },
    [CMD_SET_DEV_EUI]                           = { 1, 8, 8 },
    [CMD_SET_NWKKEY]                            = { 1, 16, 16 },
    [CMD_GET_PIN]                               = { 1, 0, 0 },
    [CMD_GET_CHIP_EUI]                          = { 1, 0, 0 },
    [CMD_DERIVE_KEYS]                           = { 1, 0, 0 },
    [CMD_GET_MODEM_VERSION]                     = { 1, 0, 0 },
    [CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER]   = { 1, 0, 0 },
    [CMD_SET_CERTIFICATION_MODE]                = { 1, 1, 1 },
    [CMD_EMERGENCY_UPLINK]                      = { 1, 2, 244 },
    [CMD_REQUEST_EMPTY_UPLINK]                  = { 1, 3, 3 },
    [CMD_LEAVE_NETWORK]                         = { 1, 0, 0 },
    [CMD_ALARM_START_TIMER]                     = { 1, 4, 4 },
    [CMD_ALARM_CLEAR_TIMER]                     = { 1, 0, 0 },
    [CMD_ALARM_GET_REMAINING_TIME]              = { 1, 0, 0 },
    [CMD_GET_NEXT_TX_MAX_PAYLOAD]               = { 1, 0, 0 },
    [CMD_GET_DUTY_CYCLE_STATUS]                 = { 1, 0, 0 },
    [CMD_SET_NETWORK_TYPE]                      = { 1, 1, 1 },
    [CMD_SET_JOIN_DR_DISTRIBUTION]              = { 1, 16, 16 },
    [CMD_SET_ADR_PROFILE]                       = { 1, 1, 17 },
    [CMD_SET_NB_TRANS]                          = { 1, 1, 1 },
    [CMD_GET_NB_TRANS]                          = { 1, 0, 0 },
    [CMD_GET_ENABLED_DATARATE]                  = { 1, 0, 0 },
    [CMD_SET_ADR_ACK_LIMIT_DELAY]               = { 1, 2, 2},
    [CMD_GET_ADR_ACK_LIMIT_DELAY]               = { 1, 0, 0},
    [CMD_SET_CRYSTAL_ERR]                       = { 1, 4, 4 },
    [CMD_LBT_SET_PARAMS]                        = { 1, 10, 10 },
    [CMD_LBT_GET_PARAMS]                        = { 1, 0, 0 },
    [CMD_LBT_SET_STATE]                         = { 1, 1, 1 },
    [CMD_LBT_GET_STATE]                         = { 1, 0, 0 },
    [CMD_GET_CHARGE]                            = { 1, 0, 0 },
    [CMD_RESET_CHARGE]                          = { 1, 0, 0 },
    [CMD_SET_CLASS]                             = { 1, 1, 1 },
    [CMD_CLASS_B_SET_PING_SLOT_PERIODICITY]     = { 1, 1, 1, },
    [CMD_CLASS_B_GET_PING_SLOT_PERIODICITY]     = { 1, 0, 0, },
    [CMD_MULTICAST_SET_GROUP_CONFIG]            = { 1, 37, 37 },
    [CMD_MULTICAST_GET_GROUP_CONFIG]            = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_C_START_SESSION]       = { 1, 6, 6 },
    [CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS]  = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_C_STOP_SESSION]        = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS]   = { 1, 0, 0 },
    [CMD_MULTICAST_CLASS_B_START_SESSION]       = { 1, 7, 7 },
    [CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS]  = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_B_STOP_SESSION]        = { 1, 1, 1 },
    [CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS]   = { 1, 0, 0 },
    [CMD_START_ALCSYNC_SERVICE]                 = { 1, 0, 0 },
    [CMD_STOP_ALCSYNC_SERVICE]                  = { 1, 0, 0 },
    [CMD_GET_ALCSYNC_TIME]                      = { 1, 0, 0 },
    [CMD_TRIG_ALCSYNC_REQUEST]                  = { 1, 0, 0 },
    [CMD_LORAWAN_MAC_REQUEST]                   = { 1, 1, 1 },
    [CMD_GET_LORAWAN_TIME]                      = { 1, 0, 0},
    [CMD_GET_LINK_CHECK_DATA]                   = { 1, 0, 0 },
    [CMD_SET_DUTY_CYCLE_STATE]                  = { 1, 1, 1 },
    [CMD_DEBUG_CONNECT_WITH_ABP]                = { 1, 36, 36 },
    [CMD_TEST]                                  = { 1, 1, 255 },
    [CMD_GET_TX_POWER_OFFSET]                   = { 1, 0, 0 },
    [CMD_SET_TX_POWER_OFFSET]                   = { 1, 1, 1 },
    [CMD_CSMA_SET_STATE]                        = { 1, 1, 1 },
    [CMD_CSMA_GET_STATE]                        = { 1, 0, 0 },
    [CMD_CSMA_SET_PARAMETERS]                   = { 1, 3, 3 },
    [CMD_CSMA_GET_PARAMETERS]                   = { 1, 0, 0 },
    [CMD_STREAM_INIT]                           = { 1, 3, 3 },
    [CMD_STREAM_ADD_DATA]                       = { 1, 1, 255 },
    [CMD_STREAM_STATUS]                         = { 1, 0, 0 },
    [CMD_LFU_INIT]                              = { 1, 6, 6 },
    [CMD_LFU_DATA]                              = { 1, 0, 255 },
    [CMD_LFU_START]                             = { 1, 4, 4 },
    [CMD_LFU_RESET]                             = { 1, 0, 0 },
    [CMD_DM_ENABLE]                             = { 1, 1, 1 },
    [CMD_DM_GET_PORT]                           = { 1, 0, 0 },
    [CMD_DM_SET_PORT]                           = { 1, 1, 1 },
    [CMD_DM_GET_INFO_INTERVAL]                  = { 1, 0, 0 },
    [CMD_DM_SET_INFO_INTERVAL]                  = { 1, 1, 1 },
    [CMD_DM_GET_PERIODIC_INFO_FIELDS]           = { 1, 0, 0 },
    [CMD_DM_SET_PERIODIC_INFO_FIELDS]           = { 1, 0, MODEM_MAX_INFO_FIELD_SIZE },
    [CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS]      = { 1, 0, MODEM_MAX_INFO_FIELD_SIZE },
    [CMD_DM_SET_USER_DATA]                      = { 1, SMTC_MODEM_DM_USER_DATA_LENGTH, SMTC_MODEM_DM_USER_DATA_LENGTH },
    [CMD_DM_GET_USER_DATA]                      = { 1, 0, 0 },
    [CMD_GET_STATUS]                            = { 1, 0, 0 },
    [CMD_SUSPEND_RADIO_COMMUNICATIONS]          = { 1, 1, 1 },
    [CMD_DM_HANDLE_ALCSYNC]                     = { 1, 1, 1 },
    [CMD_SET_APPKEY]                            = { 1, 16, 16 },
    [CMD_GET_ADR_PROFILE]                       = { 1, 0, 0 },
    [CMD_GET_CERTIFICATION_MODE]                = { 1, 0, 0 },
#if defined( STM32L476xx )
    [CMD_STORE_AND_FORWARD_SET_STATE]           = { 1, 1, 1 },
    [CMD_STORE_AND_FORWARD_GET_STATE]           = { 1, 0, 0 },
    [CMD_STORE_AND_FORWARD_ADD_DATA]            = { 1, 2, 244 }, 
    [CMD_STORE_AND_FORWARD_CLEAR_DATA]          = { 1, 0, 0 },
    [CMD_STORE_AND_FORWARD_GET_FREE_SLOT]       = { 1, 0, 0 },
#endif  // STM32L476xx
#if defined( ADD_APP_GEOLOCATION ) && defined( STM32L476xx )
    [CMD_GNSS_SCAN]                             = { 1, 5, 5 },
    [CMD_GNSS_SCAN_CANCEL]                      = { 1, 0, 0 },
    [CMD_GNSS_GET_EVENT_DATA_SCAN_DONE]         = { 1, 0, 0 },
    [CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST]      = { 1, 0, 0 },
    [CMD_GNSS_GET_SCAN_DONE_METADATA_LIST]      = { 1, 0, 0 },
    [CMD_GNSS_GET_SCAN_DONE_SCAN_SV]            = { 1, 0, 0 },
    [CMD_GNSS_GET_EVENT_DATA_TERMINATED]        = { 1, 0, 0 },
    [CMD_GNSS_SET_CONST]                        = { 1, 1, 1 },
    [CMD_GNSS_SET_PORT]                         = { 1, 1, 1 },
    [CMD_GNSS_SCAN_AGGREGATE]                   = { 1, 1, 1 },
    [CMD_GNSS_SEND_MODE]                        = { 1, 1, 1 },
    [CMD_GNSS_ALM_DEMOD_START]                  = { 1, 0, 0 },
    [CMD_GNSS_ALM_DEMOD_SET_CONSTEL]            = { 1, 1, 1 },
    [CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD] = { 1, 0, 0 },
    [CMD_CLOUD_ALMANAC_START]                   = { 1, 0, 0 },
    [CMD_CLOUD_ALMANAC_STOP]                    = { 1, 0, 0 },
    [CMD_WIFI_SCAN_START]                       = { 1, 4, 4 },
    [CMD_WIFI_SCAN_CANCEL]                      = { 1, 0, 0 },
    [CMD_WIFI_GET_SCAN_DONE_SCAN_DATA]          = { 1, 0, 0 },
    [CMD_WIFI_GET_EVENT_DATA_TERMINATED]        = { 1, 0, 0 },
    [CMD_WIFI_SET_PORT]                         = { 1, 1, 1 },
    [CMD_WIFI_SEND_MODE]                        = { 1, 1, 1 },
    [CMD_WIFI_SET_PAYLOAD_FORMAT]               = { 1, 1, 1 },
    [CMD_LR11XX_RADIO_READ]                     = { 1, 0, 255 },
    [CMD_LR11XX_RADIO_WRITE]                    = { 1, 0, 255 },
#endif  // ADD_APP_GEOLOCATION && STM32L476xx
};

/**
 * @brief Test commands tab for availability, min length and max length
 *
 */
static const uint8_t host_cmd_test_tab[CMD_TST_MAX][HOST_CMD_TAB_IDX_COUNT] = {
    // [CMD_xxx] = {availability, len_min, len_max}
    [CMD_TST_START]                = { 1, 8, 8 },
    [CMD_TST_NOP]                  = { 1, 0, 0 },
    [CMD_TST_TX_SINGLE]            = { 1, 9, 9 },
    [CMD_TST_TX_CONT]              = { 1, 9, 9 },
    [CMD_TST_TX_HOP]               = { 1, 4, 4 },
    [CMD_TST_NA_1]                 = { 1, 0, 0 },
    [CMD_TST_TX_CW]                = { 1, 5, 5 },
    [CMD_TST_RX_CONT]              = { 1, 7, 7 },
    [CMD_TST_RSSI]                 = { 1, 7, 7 },
    [CMD_TST_RADIO_RST]            = { 1, 0, 0 },
    [CMD_TST_EXIT]                 = { 1, 0, 0 },
    [CMD_TST_BUSYLOOP]             = { 1, 0, 0 },
    [CMD_TST_PANIC]                = { 1, 0, 0 },
    [CMD_TST_WATCHDOG]             = { 1, 0, 0 },
    [CMD_TST_RADIO_READ]           = { 1, 0, 255 },
    [CMD_TST_RADIO_WRITE]          = { 1, 0, 255 },
    [CMD_TST_TX_SINGLE_PREAM]      = { 1, 11, 11 },
    [CMD_TST_RSSI_GET]             = { 1, 0, 0 },
    [CMD_TST_READ_NB_PKTS_RX_CONT] = { 1, 0, 0 },
};

#if HAL_DBG_TRACE == HAL_FEATURE_ON
/**
 * @brief Host command string names for print purpose
 *
 */
static const char* host_cmd_str[CMD_MAX] = {
    [CMD_RESET]                                = "CMD_RESET",
    [CMD_SET_REGION]                           = "CMD_SET_REGION",
    [CMD_GET_REGION]                           = "CMD_GET_REGION",
    [CMD_JOIN_NETWORK]                         = "CMD_JOIN_NETWORK",
    [CMD_REQUEST_UPLINK]                       = "CMD_REQUEST_UPLINK",
    [CMD_GET_EVENT]                            = "CMD_GET_EVENT",
    [CMD_GET_DOWNLINK_DATA]                    = "CMD_GET_DOWNLINK_DATA",
    [CMD_GET_DOWNLINK_METADATA]                = "CMD_GET_DOWNLINK_METADATA",
    [CMD_GET_JOIN_EUI]                         = "CMD_GET_JOIN_EUI",
    [CMD_SET_JOIN_EUI]                         = "CMD_SET_JOIN_EUI",
    [CMD_GET_DEV_EUI]                          = "CMD_GET_DEV_EUI",
    [CMD_SET_DEV_EUI]                          = "CMD_SET_DEV_EUI",
    [CMD_SET_NWKKEY]                           = "CMD_SET_NWKKEY",
    [CMD_GET_PIN]                              = "CMD_GET_PIN",
    [CMD_GET_CHIP_EUI]                         = "CMD_GET_CHIP_EUI",
    [CMD_DERIVE_KEYS]                          = "CMD_DERIVE_KEYS",
    [CMD_GET_MODEM_VERSION]                    = "CMD_GET_MODEM_VERSION",
    [CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER]  = "CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER",
    [CMD_SET_CERTIFICATION_MODE]               = "CMD_SET_CERTIFICATION_MODE",
    [CMD_EMERGENCY_UPLINK]                     = "CMD_EMERGENCY_UPLINK",
    [CMD_REQUEST_EMPTY_UPLINK]                 = "CMD_REQUEST_EMPTY_UPLINK",
    [CMD_LEAVE_NETWORK]                        = "CMD_LEAVE_NETWORK",
    [CMD_ALARM_START_TIMER]                    = "CMD_ALARM_START_TIMER",
    [CMD_ALARM_CLEAR_TIMER]                    = "CMD_ALARM_CLEAR_TIMER",
    [CMD_ALARM_GET_REMAINING_TIME]             = "CMD_ALARM_GET_REMAINING_TIME",
    [CMD_GET_NEXT_TX_MAX_PAYLOAD]              = "CMD_GET_NEXT_TX_MAX_PAYLOAD",
    [CMD_GET_DUTY_CYCLE_STATUS]                = "CMD_GET_DUTY_CYCLE_STATUS",
    [CMD_SET_NETWORK_TYPE]                     = "CMD_SET_NETWORK_TYPE",
    [CMD_SET_JOIN_DR_DISTRIBUTION]             = "CMD_SET_JOIN_DR_DISTRIBUTION",
    [CMD_SET_ADR_PROFILE]                      = "CMD_SET_ADR_PROFILE",
    [CMD_SET_NB_TRANS]                         = "CMD_SET_NB_TRANS",
    [CMD_GET_NB_TRANS]                         = "CMD_GET_NB_TRANS",
    [CMD_GET_ENABLED_DATARATE]                 = "CMD_GET_ENABLED_DATARATE",
    [CMD_SET_ADR_ACK_LIMIT_DELAY]              = "CMD_SET_ADR_ACK_LIMIT_DELAY",
    [CMD_GET_ADR_ACK_LIMIT_DELAY]              = "CMD_GET_ADR_ACK_LIMIT_DELAY",
    [CMD_SET_CRYSTAL_ERR]                      = "CMD_SET_CRYSTAL_ERR",
    [CMD_LBT_SET_PARAMS]                       = "CMD_LBT_SET_PARAMS",
    [CMD_LBT_GET_PARAMS]                       = "CMD_LBT_GET_PARAMS",
    [CMD_LBT_SET_STATE]                        = "CMD_LBT_SET_STATE",
    [CMD_LBT_GET_STATE]                        = "CMD_LBT_GET_STATE",
    [CMD_GET_CHARGE]                           = "CMD_GET_CHARGE",
    [CMD_RESET_CHARGE]                         = "CMD_RESET_CHARGE",
    [CMD_SET_CLASS]                            = "CMD_SET_CLASS",
    [CMD_CLASS_B_SET_PING_SLOT_PERIODICITY]    = "CMD_CLASS_B_SET_PING_SLOT_PERIODICITY",
    [CMD_CLASS_B_GET_PING_SLOT_PERIODICITY]    = "CMD_CLASS_B_GET_PING_SLOT_PERIODICITY",
    [CMD_MULTICAST_SET_GROUP_CONFIG]           = "CMD_MULTICAST_SET_GROUP_CONFIG",
    [CMD_MULTICAST_GET_GROUP_CONFIG]           = "CMD_MULTICAST_GET_GROUP_CONFIG",
    [CMD_MULTICAST_CLASS_C_START_SESSION]      = "CMD_MULTICAST_CLASS_C_START_SESSION",
    [CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS] = "CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS",
    [CMD_MULTICAST_CLASS_C_STOP_SESSION]       = "CMD_MULTICAST_CLASS_C_STOP_SESSION",
    [CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS]  = "CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS",
    [CMD_MULTICAST_CLASS_B_START_SESSION]      = "CMD_MULTICAST_CLASS_B_START_SESSION",
    [CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS] = "CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS",
    [CMD_MULTICAST_CLASS_B_STOP_SESSION]       = "CMD_MULTICAST_CLASS_B_STOP_SESSION",
    [CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS]  = "CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS",
    [CMD_START_ALCSYNC_SERVICE]                = "CMD_START_ALCSYNC_SERVICE",
    [CMD_STOP_ALCSYNC_SERVICE]                 = "CMD_STOP_ALCSYNC_SERVICE",
    [CMD_GET_ALCSYNC_TIME]                     = "CMD_GET_ALCSYNC_TIME",
    [CMD_TRIG_ALCSYNC_REQUEST]                 = "CMD_TRIG_ALCSYNC_REQUEST",
    [CMD_LORAWAN_MAC_REQUEST]                  = "CMD_LORAWAN_MAC_REQUEST",
    [CMD_GET_LORAWAN_TIME]                     = "CMD_GET_LORAWAN_TIME",
    [CMD_GET_LINK_CHECK_DATA]                  = "CMD_GET_LINK_CHECK_DATA",
    [CMD_SET_DUTY_CYCLE_STATE]                 = "CMD_SET_DUTY_CYCLE_STATE",
    [CMD_DEBUG_CONNECT_WITH_ABP]               = "CMD_DEBUG_CONNECT_WITH_ABP",
    [CMD_TEST]                                 = "CMD_TEST",
    [CMD_GET_TX_POWER_OFFSET]                  = "CMD_GET_TX_POWER_OFFSET",
    [CMD_SET_TX_POWER_OFFSET]                  = "CMD_SET_TX_POWER_OFFSET",
    [CMD_CSMA_SET_STATE]                       = "CMD_CSMA_SET_STATE",
    [CMD_CSMA_GET_STATE]                       = "CMD_CSMA_GET_STATE",
    [CMD_CSMA_SET_PARAMETERS]                  = "CMD_CSMA_SET_PARAMETERS",
    [CMD_CSMA_GET_PARAMETERS]                  = "CMD_CSMA_GET_PARAMETERS",
    [CMD_STREAM_INIT]                          = "CMD_STREAM_INIT",
    [CMD_STREAM_ADD_DATA]                      = "CMD_STREAM_ADD_DATA",
    [CMD_STREAM_STATUS]                        = "CMD_STREAM_STATUS",
    [CMD_LFU_INIT]                             = "CMD_LFU_INIT",
    [CMD_LFU_DATA]                             = "CMD_LFU_DATA",
    [CMD_LFU_START]                            = "CMD_LFU_START",
    [CMD_LFU_RESET]                            = "CMD_LFU_RESET",
    [CMD_DM_ENABLE]                            = "CMD_DM_ENABLE",
    [CMD_DM_GET_PORT]                          = "CMD_DM_GET_PORT",
    [CMD_DM_SET_PORT]                          = "CMD_DM_SET_PORT",
    [CMD_DM_GET_INFO_INTERVAL]                 = "CMD_DM_GET_INFO_INTERVAL",
    [CMD_DM_SET_INFO_INTERVAL]                 = "CMD_DM_SET_INFO_INTERVAL",
    [CMD_DM_GET_PERIODIC_INFO_FIELDS]          = "CMD_DM_GET_PERIODIC_INFO_FIELDS",
    [CMD_DM_SET_PERIODIC_INFO_FIELDS]          = "CMD_DM_SET_PERIODIC_INFO_FIELDS",
    [CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS]     = "CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS",
    [CMD_DM_SET_USER_DATA]                     = "CMD_DM_SET_USER_DATA",
    [CMD_DM_GET_USER_DATA]                     = "CMD_DM_GET_USER_DATA",
    [CMD_GET_STATUS]                           = "CMD_GET_STATUS",
    [CMD_SUSPEND_RADIO_COMMUNICATIONS]         = "CMD_SUSPEND_RADIO_COMMUNICATIONS",
    [CMD_DM_HANDLE_ALCSYNC]                    = "CMD_DM_HANDLE_ALCSYNC",
    [CMD_SET_APPKEY]                           = "CMD_SET_APPKEY",
    [CMD_GET_ADR_PROFILE]                      = "CMD_GET_ADR_PROFILE",
    [CMD_GET_CERTIFICATION_MODE]               = "CMD_GET_CERTIFICATION_MODE",
#if defined( STM32L476xx )
    [CMD_STORE_AND_FORWARD_SET_STATE]     = "CMD_STORE_AND_FORWARD_SET_STATE",
    [CMD_STORE_AND_FORWARD_GET_STATE]     = "CMD_STORE_AND_FORWARD_GET_STATE",
    [CMD_STORE_AND_FORWARD_ADD_DATA]      = "CMD_STORE_AND_FORWARD_ADD_DATA",
    [CMD_STORE_AND_FORWARD_CLEAR_DATA]    = "CMD_STORE_AND_FORWARD_CLEAR_DATA",
    [CMD_STORE_AND_FORWARD_GET_FREE_SLOT] = "CMD_STORE_AND_FORWARD_GET_FREE_SLOT",
#endif  // STM32L476xx
#if defined( ADD_APP_GEOLOCATION ) && defined( STM32L476xx )
    [CMD_GNSS_SCAN]                             = "CMD_GNSS_SCAN",
    [CMD_GNSS_SCAN_CANCEL]                      = "CMD_GNSS_SCAN_CANCEL",
    [CMD_GNSS_GET_EVENT_DATA_SCAN_DONE]         = "CMD_GNSS_GET_EVENT_DATA_SCAN_DONE",
    [CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST]      = "CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST",
    [CMD_GNSS_GET_SCAN_DONE_METADATA_LIST]      = "CMD_GNSS_GET_SCAN_DONE_METADATA_LIST",
    [CMD_GNSS_GET_SCAN_DONE_SCAN_SV]            = "CMD_GNSS_GET_SCAN_DONE_SCAN_SV",
    [CMD_GNSS_GET_EVENT_DATA_TERMINATED]        = "CMD_GNSS_GET_EVENT_DATA_TERMINATED",
    [CMD_GNSS_SET_CONST]                        = "CMD_GNSS_SET_CONST",
    [CMD_GNSS_SET_PORT]                         = "CMD_GNSS_SET_PORT",
    [CMD_GNSS_SCAN_AGGREGATE]                   = "CMD_GNSS_SCAN_AGGREGATE",
    [CMD_GNSS_SEND_MODE]                        = "CMD_GNSS_SEND_MODE",
    [CMD_GNSS_ALM_DEMOD_START]                  = "CMD_GNSS_ALM_DEMOD_START",
    [CMD_GNSS_ALM_DEMOD_SET_CONSTEL]            = "CMD_GNSS_ALM_DEMOD_SET_CONSTEL",
    [CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD] = "CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD",
    [CMD_CLOUD_ALMANAC_START]                   = "CMD_CLOUD_ALMANAC_START",
    [CMD_CLOUD_ALMANAC_STOP]                    = "CMD_CLOUD_ALMANAC_STOP",
    [CMD_WIFI_SCAN_START]                       = "CMD_MODEM_WIFI_SCAN_START",
    [CMD_WIFI_SCAN_CANCEL]                      = "CMD_MODEM_WIFI_SCAN_CANCEL",
    [CMD_WIFI_GET_SCAN_DONE_SCAN_DATA]          = "CMD_MODEM_WIFI_GET_SCAN_DONE_SCAN_DATA",
    [CMD_WIFI_GET_EVENT_DATA_TERMINATED]        = "CMD_MODEM_WIFI_GET_EVENT_DATA_TERMINATED",
    [CMD_WIFI_SET_PORT]                         = "CMD_MODEM_WIFI_SET_PORT",
    [CMD_WIFI_SEND_MODE]                        = "CMD_MODEM_WIFI_SEND_MODE",
    [CMD_WIFI_SET_PAYLOAD_FORMAT]               = "CMD_MODEM_WIFI_SET_PAYLOAD_FORMAT",
    [CMD_LR11XX_RADIO_READ]                     = "CMD_LR11XX_RADIO_READ",
    [CMD_LR11XX_RADIO_WRITE]                    = "CMD_LR11XX_RADIO_WRITE",

#endif  // ADD_APP_GEOLOCATION && STM32L476xx
};
#endif

#if HAL_DBG_TRACE == HAL_FEATURE_ON
/**
 * @brief Host test command names for print purpose
 *
 */
static const char* host_cmd_test_str[CMD_TST_MAX] = {
    [CMD_TST_START]                = "START",
    [CMD_TST_NOP]                  = "NOP",
    [CMD_TST_TX_SINGLE]            = "TX_SINGLE",
    [CMD_TST_TX_CONT]              = "TX_CONT",
    [CMD_TST_TX_HOP]               = "TX_HOP",
    [CMD_TST_NA_1]                 = "NA_1",
    [CMD_TST_TX_CW]                = "TX_CW",
    [CMD_TST_RX_CONT]              = "RX_CONT",
    [CMD_TST_RSSI]                 = "RSSI",
    [CMD_TST_RADIO_RST]            = "RADIO_RST",
    [CMD_TST_EXIT]                 = "EXIT",
    [CMD_TST_BUSYLOOP]             = "BUSYLOOP",
    [CMD_TST_PANIC]                = "PANIC",
    [CMD_TST_WATCHDOG]             = "WATCHDOG",
    [CMD_TST_RADIO_READ]           = "RADIO_READ",
    [CMD_TST_RADIO_WRITE]          = "RADIO_WRITE",
    [CMD_TST_TX_SINGLE_PREAM]      = "TX_SINGLE_PREAM",
    [CMD_TST_RSSI_GET]             = "TST_RSSI_GET",
    [CMD_TST_READ_NB_PKTS_RX_CONT] = "READ_NB_PKTS_RX_CONT",
};
#endif

/* clang-format off */

/**
 * @brief Spreading factor conversion table from hw modem command format to test modem api
 *
 */
static const smtc_modem_test_sf_t cmd_test_sf_table[SMTC_MODEM_TEST_LORA_SF_COUNT] = {
    SMTC_MODEM_TEST_FSK,
    SMTC_MODEM_TEST_LORA_SF7,
    SMTC_MODEM_TEST_LORA_SF8,
    SMTC_MODEM_TEST_LORA_SF9,
    SMTC_MODEM_TEST_LORA_SF10,
    SMTC_MODEM_TEST_LORA_SF11,
    SMTC_MODEM_TEST_LORA_SF12,
    SMTC_MODEM_TEST_LORA_SF5,
    SMTC_MODEM_TEST_LORA_SF6
};

/**
 * @brief Bandwidth conversion table from hw modem command format to test modem api
 *
 */
static const smtc_modem_test_bw_t cmd_test_bw_table[SMTC_MODEM_TEST_BW_COUNT] = {
    SMTC_MODEM_TEST_BW_125_KHZ,
    SMTC_MODEM_TEST_BW_250_KHZ,
    SMTC_MODEM_TEST_BW_500_KHZ,
    SMTC_MODEM_TEST_BW_200_KHZ,
    SMTC_MODEM_TEST_BW_400_KHZ,
    SMTC_MODEM_TEST_BW_800_KHZ,
    SMTC_MODEM_TEST_BW_1600_KHZ,
};

/**
 * @brief Coding rate conversion table from hw modem command format to test modem api
 *
 */
static const smtc_modem_test_cr_t cmd_test_cr_table[SMTC_MODEM_TEST_CR_COUNT] = {
    SMTC_MODEM_TEST_CR_4_5,
    SMTC_MODEM_TEST_CR_4_6,
    SMTC_MODEM_TEST_CR_4_7,
    SMTC_MODEM_TEST_CR_4_8,
    SMTC_MODEM_TEST_CR_LI_4_5,
    SMTC_MODEM_TEST_CR_LI_4_6,
    SMTC_MODEM_TEST_CR_LI_4_8
};

/**
 * @brief Modem class conversion table from hw modem command format to modem api
 *
 */
static const smtc_modem_class_t  cmd_modem_class_table[]={
    SMTC_MODEM_CLASS_A,
    SMTC_MODEM_CLASS_C,
    SMTC_MODEM_CLASS_B,
};

/**
 * @brief Look Up Table for hw modem event opcodes
 *
 */
static const uint8_t events_lut[SMTC_MODEM_EVENT_MAX] = {
    [SMTC_MODEM_EVENT_RESET]                             = 0x00,
    [SMTC_MODEM_EVENT_ALARM]                             = 0x01,
    [SMTC_MODEM_EVENT_JOINED]                            = 0x02,
    [SMTC_MODEM_EVENT_TXDONE]                            = 0x03,
    [SMTC_MODEM_EVENT_DOWNDATA]                          = 0x04,
    [SMTC_MODEM_EVENT_JOINFAIL]                          = 0x0A,
    [SMTC_MODEM_EVENT_ALCSYNC_TIME]                      = 0x0D,
    [SMTC_MODEM_EVENT_LINK_CHECK]                        = 0x10,
    [SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO]            = 0x13,
    [SMTC_MODEM_EVENT_CLASS_B_STATUS]                    = 0x14,
    [SMTC_MODEM_EVENT_LORAWAN_MAC_TIME]                  = 0x19,
    [SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE]                = 0x1A,
    [SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C] = 0x1B,
    [SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B] = 0x1C,
    [SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C]     = 0x1D,
    [SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B]     = 0x1E,
    [SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT]               = 0x1F,
    [SMTC_MODEM_EVENT_STREAM_DONE]                       = 0x08,
    [SMTC_MODEM_EVENT_UPLOAD_DONE]                       = 0x05,
    [SMTC_MODEM_EVENT_DM_SET_CONF]                       = 0x06,
    [SMTC_MODEM_EVENT_MUTE]                              = 0x07,
    [SMTC_MODEM_EVENT_GNSS_SCAN_DONE]                    = 0x20,
    [SMTC_MODEM_EVENT_GNSS_TERMINATED]                   = 0x21,
    [SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE]         = 0x22,
    [SMTC_MODEM_EVENT_WIFI_SCAN_DONE]                    = 0x23,
    [SMTC_MODEM_EVENT_WIFI_TERMINATED]                   = 0x24,


};

/**
 * @brief Look Up Table for hw modem return code opcodes
 *
 */
static const cmd_serial_rc_code_t rc_lut[] = {
    [SMTC_MODEM_RC_OK]               = CMD_RC_OK,
    [SMTC_MODEM_RC_NOT_INIT]         = CMD_RC_NOT_INIT,
    [SMTC_MODEM_RC_INVALID]          = CMD_RC_INVALID,
    [SMTC_MODEM_RC_BUSY]             = CMD_RC_BUSY,
    [SMTC_MODEM_RC_FAIL]             = CMD_RC_FAIL,
    [SMTC_MODEM_RC_NO_TIME]          = CMD_RC_NO_TIME,
    [SMTC_MODEM_RC_INVALID_STACK_ID] = CMD_RC_INVALID_STACK_ID,
    [SMTC_MODEM_RC_NO_EVENT]         = CMD_RC_NO_EVENT,
};

/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Check command size
 *
 * @param [in] cmd_id Received command id
 * @param [in] length Received command length
 * @return cmd_length_valid_t
 */
static cmd_length_valid_t cmd_parser_check_cmd_size( host_cmd_id_t cmd_id, uint8_t length );

/**
 * @brief
 *
 * @param [in] test_id Received test command id
 * @param [in] length  Received test command length
 * @return cmd_length_valid_t
 */
static cmd_length_valid_t cmd_test_parser_check_cmd_size( host_cmd_test_id_t test_id, uint8_t length );

/**
 * @brief Check if received sf, bw and cr are acceptable
 *
 * @param [in] sf  Received spreading factor
 * @param [in] bw  Received bandwidth
 * @param [in] cr  Received coding rate
 * @return cmd_parse_status_t
 */
static cmd_parse_status_t cmd_test_parser_check_sf_bw_cr( uint8_t sf, uint8_t bw, uint8_t cr );

/**
 * @brief Crc function used for LFU (Large File Upload)
 *
 * @param [in] buf Payload buffer
 * @param [in] len Length of the payload
 * @return uint32_t The calculated CRC
 */
uint32_t cmd_parser_crc( const uint8_t* buf, int len );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
cmd_parse_status_t parse_cmd( cmd_input_t* cmd_input, cmd_response_t* cmd_output )
{
    cmd_parse_status_t ret  = PARSE_OK;
    cmd_output->return_code = CMD_RC_OK;
    cmd_output->length      = 0;

    if( ( cmd_input->cmd_code >= CMD_MAX ) ||
        ( host_cmd_tab[cmd_input->cmd_code][HOST_CMD_TAB_IDX_AVAILABILITY] != 1 ) )
    {
        SMTC_HAL_TRACE_ERROR( "Unknown command (0x%x)\n", cmd_input->cmd_code );
        cmd_output->return_code = CMD_RC_UNKNOWN;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }

    if( cmd_parser_check_cmd_size( cmd_input->cmd_code, cmd_input->length ) == CMD_LENGTH_NOT_VALID )
    {
        cmd_output->return_code = CMD_RC_BAD_SIZE;
        cmd_output->length      = 0;
        return PARSE_ERROR;
    }
    SMTC_HAL_TRACE_WARNING( "CMD_%s (0x%02x)\n", host_cmd_str[cmd_input->cmd_code], cmd_input->cmd_code );
    switch( cmd_input->cmd_code )
    {
    case CMD_GET_EVENT:
    {
        smtc_modem_event_t current_event       = { 0 };
        uint8_t            event_pending_count = 0;
        cmd_output->return_code                = rc_lut[smtc_modem_get_event( &current_event, &event_pending_count )];
        if( cmd_output->return_code == CMD_RC_NO_EVENT )
        {
            // No event available
            cmd_output->length = 0;
            break;
        }

        // buffer[0]: event type
        cmd_output->buffer[0] = events_lut[current_event.event_type];

        // buffer[1]: missed event
        cmd_output->buffer[1] = current_event.missed_events;

        // buffer[2-N]; event data, depend on event_type
        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            cmd_output->buffer[2] = ( uint8_t )( current_event.event_data.reset.count >> 8 );
            cmd_output->buffer[3] = ( uint8_t )( current_event.event_data.reset.count );
            cmd_output->length    = 4;
            break;
        case SMTC_MODEM_EVENT_TXDONE:
            cmd_output->buffer[2] = current_event.event_data.txdone.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_LINK_CHECK:
            cmd_output->buffer[2] = current_event.event_data.link_check.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            cmd_output->buffer[2] = current_event.event_data.class_b_ping_slot_info.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            cmd_output->buffer[2] = current_event.event_data.class_b_status.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
            cmd_output->buffer[2] = current_event.event_data.lorawan_mac_time.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
            cmd_output->buffer[2] = current_event.event_data.fuota_status.successful;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
            cmd_output->buffer[2] = current_event.event_data.new_multicast_class_c.group_id;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
            cmd_output->buffer[2] = current_event.event_data.new_multicast_class_b.group_id;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
            cmd_output->buffer[2] = current_event.event_data.fmp.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_UPLOAD_DONE:
            cmd_output->buffer[2] = current_event.event_data.uploaddone.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_DM_SET_CONF:
            cmd_output->buffer[2] = current_event.event_data.setconf.opcode;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_MUTE:
            cmd_output->buffer[2] = current_event.event_data.mute.status;
            cmd_output->length    = 3;
            break;
        case SMTC_MODEM_EVENT_DOWNDATA:
        case SMTC_MODEM_EVENT_ALCSYNC_TIME:
        case SMTC_MODEM_EVENT_ALARM:
        case SMTC_MODEM_EVENT_JOINED:
        case SMTC_MODEM_EVENT_JOINFAIL:
        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
        case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
        case SMTC_MODEM_EVENT_STREAM_DONE:
        case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
        case SMTC_MODEM_EVENT_GNSS_TERMINATED:
        case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
        case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
        case SMTC_MODEM_EVENT_WIFI_TERMINATED:
            cmd_output->length = 2;
            break;

        default:
            cmd_output->length = 0;
            break;
        }

        // Handle event_pending_count
        if( event_pending_count == 0 )
        {
            // de-assert hw_modem irq line to indicate host that all events have been retrieved
            hal_gpio_set_value( HW_MODEM_EVENT_PIN, 0 );
        }
        break;
    }
    case CMD_GET_DOWNLINK_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_downlink_data( &cmd_output->buffer[2], &cmd_output->buffer[1],
                                                                       &last_dl_metadata, &cmd_output->buffer[0] )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 2 + cmd_output->buffer[1];
        }
        break;
    }
    case CMD_GET_DOWNLINK_METADATA:
    {
        cmd_output->return_code = CMD_RC_OK;

        cmd_output->buffer[0]  = last_dl_metadata.stack_id;
        cmd_output->buffer[1]  = last_dl_metadata.rssi;
        cmd_output->buffer[2]  = last_dl_metadata.snr;
        cmd_output->buffer[3]  = last_dl_metadata.window;
        cmd_output->buffer[4]  = last_dl_metadata.fport;
        cmd_output->buffer[5]  = last_dl_metadata.fpending_bit;
        cmd_output->buffer[6]  = ( last_dl_metadata.frequency_hz >> 24 ) & 0xff;
        cmd_output->buffer[7]  = ( last_dl_metadata.frequency_hz >> 16 ) & 0xff;
        cmd_output->buffer[8]  = ( last_dl_metadata.frequency_hz >> 8 ) & 0xff;
        cmd_output->buffer[9]  = ( last_dl_metadata.frequency_hz & 0xff );
        cmd_output->buffer[10] = last_dl_metadata.datarate;

        cmd_output->length = 11;
        break;
    }
    case CMD_RESET:
    {
        smtc_modem_hal_reset_mcu( );
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_RESET_CHARGE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_reset_charge( )];
        break;
    }
    case CMD_GET_CHARGE:
    {
        uint32_t charge         = 0;
        cmd_output->return_code = rc_lut[smtc_modem_get_charge( &charge )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( charge >> 24 ) & 0xFF;
            cmd_output->buffer[1] = ( charge >> 16 ) & 0xFF;
            cmd_output->buffer[2] = ( charge >> 8 ) & 0xFF;
            cmd_output->buffer[3] = ( charge & 0xFF );
            cmd_output->length    = 4;
        }
        break;
    }
    case CMD_GET_TX_POWER_OFFSET:
    {
        int8_t offset           = radio_utilities_get_tx_power_offset( );
        cmd_output->buffer[0]   = offset;
        cmd_output->length      = 1;
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_SET_TX_POWER_OFFSET:
    {
        radio_utilities_set_tx_power_offset( cmd_input->buffer[0] );
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_GET_ALCSYNC_TIME:
    {
        uint32_t gps_time_s = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_alcsync_time( STACK_ID, &gps_time_s )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( gps_time_s >> 24 ) & 0xFF;
            cmd_output->buffer[1] = ( gps_time_s >> 16 ) & 0xFF;
            cmd_output->buffer[2] = ( gps_time_s >> 8 ) & 0xFF;
            cmd_output->buffer[3] = ( gps_time_s & 0xFF );
            cmd_output->length    = 4;
        }
        break;
    }
    case CMD_ALARM_START_TIMER:
    {
        uint32_t alarm = 0;
        alarm |= cmd_input->buffer[0] << 24;
        alarm |= cmd_input->buffer[1] << 16;
        alarm |= cmd_input->buffer[2] << 8;
        alarm |= cmd_input->buffer[3];

        cmd_output->return_code = rc_lut[smtc_modem_alarm_start_timer( alarm )];
        break;
    }
    case CMD_GET_PIN:
    {
#if defined( USE_LR11XX_CRYPTO )
        uint8_t chip_pin[4];
        cmd_output->return_code = rc_lut[smtc_modem_get_pin( STACK_ID, chip_pin )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 4;
            // reverse endianess
            for( uint8_t i = 0; i < cmd_output->length; i++ )
            {
                cmd_output->buffer[i] = chip_pin[i];
            }
        }
#else
        cmd_output->return_code = CMD_RC_FAIL;
#endif
        break;
    }
    case CMD_GET_CHIP_EUI:
    {
#if defined( USE_LR11XX_CRYPTO )
        uint8_t chip_eui[8];
        cmd_output->return_code = rc_lut[smtc_modem_get_chip_eui( STACK_ID, chip_eui )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 8;
            // reverse endianess
            for( uint8_t i = 0; i < cmd_output->length; i++ )
            {
                cmd_output->buffer[i] = chip_eui[i];
            }
        }
#else
        cmd_output->return_code = CMD_RC_FAIL;
#endif
        break;
    }
    case CMD_GET_JOIN_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_joineui( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 8;
        }
        break;
    }
    case CMD_SET_JOIN_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_joineui( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_DEV_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_deveui( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 8;
        }
        break;
    }
    case CMD_SET_DEV_EUI:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_deveui( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_NWKKEY:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_nwkkey( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_CLASS:
    {
        if( cmd_input->buffer[0] > 2 )
        {
            cmd_output->return_code = CMD_RC_INVALID;
            cmd_output->length      = 0;
        }
        else
        {
            cmd_output->return_code =
                rc_lut[smtc_modem_set_class( STACK_ID, cmd_modem_class_table[cmd_input->buffer[0]] )];
        }
        break;
    }
    case CMD_GET_REGION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_region( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_SET_REGION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_region( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_ADR_PROFILE:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_adr_set_profile( STACK_ID, cmd_input->buffer[0], &cmd_input->buffer[1] )];
        break;
    }
    case CMD_GET_ADR_PROFILE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_adr_get_profile( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_JOIN_NETWORK:
    {
        cmd_output->return_code = rc_lut[smtc_modem_join_network( STACK_ID )];
        break;
    }
    case CMD_LEAVE_NETWORK:
    {
        cmd_output->return_code = rc_lut[smtc_modem_leave_network( STACK_ID )];
        break;
    }
    case CMD_GET_NEXT_TX_MAX_PAYLOAD:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_next_tx_max_payload( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_REQUEST_UPLINK:
    {
        // check if confirmed/not confirmed arg if different from 0/1 (modem api takes bool)
        if( ( cmd_input->buffer[1] != 0x00 ) && ( cmd_input->buffer[1] != 0x01 ) )
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_request_uplink(
                STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2], cmd_input->length - 2 )];
        }
        break;
    }
    case CMD_EMERGENCY_UPLINK:
    {
        // check if confirmed/not confirmed arg if different from 0/1 (modem api takes bool)
        if( ( cmd_input->buffer[1] != 0x00 ) && ( cmd_input->buffer[1] != 0x01 ) )
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_request_emergency_uplink(
                STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2], cmd_input->length - 2 )];
        }
        break;
    }
    case CMD_DERIVE_KEYS:
    {
#if defined( USE_LR11XX_CRYPTO )
        cmd_output->return_code = rc_lut[smtc_modem_derive_keys( STACK_ID )];
#else
        cmd_output->return_code = CMD_RC_FAIL;
#endif
        break;
    }
    case CMD_SET_CERTIFICATION_MODE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_certification_mode( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_CERTIFICATION_MODE:
    {
        bool certification_enabled = false;
        cmd_output->return_code    = rc_lut[smtc_modem_get_certification_mode( STACK_ID, &certification_enabled )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( uint8_t ) certification_enabled;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_TEST:
    {
        cmd_tst_input_t    cmd_tst_input;
        cmd_tst_response_t cmd_tst_output;

        cmd_tst_input.cmd_code = ( host_cmd_test_id_t ) cmd_input->buffer[0];
        cmd_tst_input.length   = cmd_input->length - 1;
        cmd_tst_input.buffer   = &cmd_input->buffer[1];
        cmd_tst_output.buffer  = &cmd_output->buffer[0];

        ret = cmd_test_parser( &cmd_tst_input, &cmd_tst_output );

        cmd_output->return_code = cmd_tst_output.return_code;
        cmd_output->length      = cmd_tst_output.length;
        break;
    }
    case CMD_GET_DUTY_CYCLE_STATUS:
    {
        int32_t next_free_dtc = 0;

        cmd_output->return_code = rc_lut[smtc_modem_get_duty_cycle_status( STACK_ID, &next_free_dtc )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( next_free_dtc >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( next_free_dtc >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( next_free_dtc >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( next_free_dtc & 0xff );

            cmd_output->length = 4;
        }
        break;
    }
    case CMD_SET_DUTY_CYCLE_STATE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_debug_set_duty_cycle_state( cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_ENABLED_DATARATE:
    {
        uint16_t enabled_datarate = 0;
        cmd_output->return_code   = rc_lut[smtc_modem_get_enabled_datarates( STACK_ID, &enabled_datarate )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( enabled_datarate >> 8 ) & 0xff;
            cmd_output->buffer[1] = ( enabled_datarate & 0xff );
            cmd_output->length    = 2;
        }
        break;
    }
    case CMD_SET_NETWORK_TYPE:
    {
        if( cmd_input->buffer[0] > 1 )
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_set_network_type( STACK_ID, cmd_input->buffer[0] )];
        }
        break;
    }
    case CMD_SET_NB_TRANS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_nb_trans( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_NB_TRANS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_get_nb_trans( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_SET_CRYSTAL_ERR:
    {
        uint32_t crystal_error = 0;
        crystal_error |= cmd_input->buffer[0] << 24;
        crystal_error |= cmd_input->buffer[1] << 16;
        crystal_error |= cmd_input->buffer[2] << 8;
        crystal_error |= cmd_input->buffer[3];
        cmd_output->return_code = rc_lut[smtc_modem_set_crystal_error_ppm( crystal_error )];
        break;
    }
    case CMD_MULTICAST_SET_GROUP_CONFIG:
    {
        uint32_t addr = 0;

        addr = cmd_input->buffer[1] << 24;
        addr |= cmd_input->buffer[2] << 16;
        addr |= cmd_input->buffer[3] << 8;
        addr |= cmd_input->buffer[4];

        cmd_output->return_code = rc_lut[smtc_modem_multicast_set_grp_config(
            STACK_ID, cmd_input->buffer[0], addr, &cmd_input->buffer[5], &cmd_input->buffer[21] )];
        break;
    }
    case CMD_MULTICAST_GET_GROUP_CONFIG:
    {
        uint32_t addr = 0;

        cmd_output->return_code = rc_lut[smtc_modem_multicast_get_grp_config( STACK_ID, cmd_input->buffer[0], &addr )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( addr >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( addr >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( addr >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( addr & 0xff );

            cmd_output->length = 4;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_C_START_SESSION:
    {
        uint32_t freq = 0;

        freq = cmd_input->buffer[1] << 24;
        freq |= cmd_input->buffer[2] << 16;
        freq |= cmd_input->buffer[3] << 8;
        freq |= cmd_input->buffer[4];

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_start_session( STACK_ID, cmd_input->buffer[0],
                                                                                     freq, cmd_input->buffer[5] )];
        break;
    }
    case CMD_MULTICAST_CLASS_C_GET_SESSION_STATUS:
    {
        uint32_t freq = 0;
        uint8_t  dr   = 0xFF;
        bool     is_session_started;

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_get_session_status(
            STACK_ID, cmd_input->buffer[0], &is_session_started, &freq, &dr )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = is_session_started;
            cmd_output->buffer[1] = ( freq >> 24 ) & 0xff;
            cmd_output->buffer[2] = ( freq >> 16 ) & 0xff;
            cmd_output->buffer[3] = ( freq >> 8 ) & 0xff;
            cmd_output->buffer[4] = ( freq & 0xff );
            cmd_output->buffer[5] = dr;

            cmd_output->length = 6;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_C_STOP_SESSION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_stop_session( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_MULTICAST_CLASS_C_STOP_ALL_SESSIONS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_c_stop_all_sessions( STACK_ID )];
        break;
    }
    case CMD_GET_MODEM_VERSION:
    {
        smtc_modem_version_t modem_version;
        cmd_output->return_code = rc_lut[smtc_modem_get_modem_version( &modem_version )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = modem_version.major;
            cmd_output->buffer[1] = modem_version.minor;
            cmd_output->buffer[2] = modem_version.patch;

            cmd_output->length = 3;
        }
        break;
    }
    case CMD_ALARM_CLEAR_TIMER:
    {
        cmd_output->return_code = rc_lut[smtc_modem_alarm_clear_timer( )];
        break;
    }
    case CMD_ALARM_GET_REMAINING_TIME:
    {
        uint32_t remaining_time = 0;

        cmd_output->return_code = rc_lut[smtc_modem_alarm_get_remaining_time( &remaining_time )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( remaining_time >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( remaining_time >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( remaining_time >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( remaining_time & 0xff );
            cmd_output->length    = 4;
        }
        break;
    }
    case CMD_REQUEST_EMPTY_UPLINK:
    {
        cmd_output->return_code = rc_lut[smtc_modem_request_empty_uplink( STACK_ID, cmd_input->buffer[0],
                                                                          cmd_input->buffer[1], cmd_input->buffer[2] )];
        break;
    }
    case CMD_LBT_SET_PARAMS:
    {
        uint32_t listening_duration_ms = 0;
        int16_t  threshold_dbm         = 0;
        uint32_t bw_hz                 = 0;

        listening_duration_ms |= cmd_input->buffer[0] << 24;
        listening_duration_ms |= cmd_input->buffer[1] << 16;
        listening_duration_ms |= cmd_input->buffer[2] << 8;
        listening_duration_ms |= cmd_input->buffer[3];

        threshold_dbm |= cmd_input->buffer[4] << 8;
        threshold_dbm |= cmd_input->buffer[5];

        bw_hz |= cmd_input->buffer[6] << 24;
        bw_hz |= cmd_input->buffer[7] << 16;
        bw_hz |= cmd_input->buffer[8] << 8;
        bw_hz |= cmd_input->buffer[9];

        cmd_output->return_code =
            rc_lut[smtc_modem_lbt_set_parameters( STACK_ID, listening_duration_ms, threshold_dbm, bw_hz )];
        break;
    }
    case CMD_LBT_GET_PARAMS:
    {
        uint32_t listening_duration_ms;
        int16_t  threshold_dbm;
        uint32_t bw_hz;
        cmd_output->return_code =
            rc_lut[smtc_modem_lbt_get_parameters( STACK_ID, &listening_duration_ms, &threshold_dbm, &bw_hz )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( listening_duration_ms >> 24 ) & 0xff;
            cmd_output->buffer[1] = ( listening_duration_ms >> 16 ) & 0xff;
            cmd_output->buffer[2] = ( listening_duration_ms >> 8 ) & 0xff;
            cmd_output->buffer[3] = ( listening_duration_ms & 0xff );
            cmd_output->buffer[4] = ( threshold_dbm >> 8 ) & 0xff;
            cmd_output->buffer[5] = ( threshold_dbm & 0xff );
            cmd_output->buffer[6] = ( bw_hz >> 24 ) & 0xff;
            cmd_output->buffer[7] = ( bw_hz >> 16 ) & 0xff;
            cmd_output->buffer[8] = ( bw_hz >> 8 ) & 0xff;
            cmd_output->buffer[9] = ( bw_hz & 0xff );

            cmd_output->length = 10;
        }
        break;
    }
    case CMD_LBT_SET_STATE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_lbt_set_state( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_LBT_GET_STATE:
    {
        bool enabled;
        cmd_output->return_code = rc_lut[smtc_modem_lbt_get_state( STACK_ID, &enabled )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = enabled;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_START_ALCSYNC_SERVICE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_start_alcsync_service( STACK_ID )];
        break;
    }
    case CMD_STOP_ALCSYNC_SERVICE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_stop_alcsync_service( STACK_ID )];
        break;
    }
    case CMD_TRIG_ALCSYNC_REQUEST:
    {
        cmd_output->return_code = rc_lut[smtc_modem_trigger_alcsync_request( STACK_ID )];
        break;
    }
    case CMD_CLASS_B_SET_PING_SLOT_PERIODICITY:
    {
        cmd_output->return_code = rc_lut[smtc_modem_class_b_set_ping_slot_periodicity(
            STACK_ID, ( smtc_modem_class_b_ping_slot_periodicity_t ) cmd_input->buffer[0] )];
        break;
    }
    case CMD_CLASS_B_GET_PING_SLOT_PERIODICITY:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_class_b_get_ping_slot_periodicity( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_B_START_SESSION:
    {
        uint32_t freq = 0;

        freq = cmd_input->buffer[1] << 24;
        freq |= cmd_input->buffer[2] << 16;
        freq |= cmd_input->buffer[3] << 8;
        freq |= cmd_input->buffer[4];

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_start_session(
            STACK_ID, cmd_input->buffer[0], freq, cmd_input->buffer[5], cmd_input->buffer[6] )];
        break;
    }
    case CMD_MULTICAST_CLASS_B_GET_SESSION_STATUS:
    {
        uint32_t                                   freq = 0;
        uint8_t                                    dr   = 0xFF;
        bool                                       is_session_started;
        bool                                       is_session_waiting_for_beacon;
        smtc_modem_class_b_ping_slot_periodicity_t ping_slot_periodicity;

        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_get_session_status(
            STACK_ID, cmd_input->buffer[0], &is_session_started, &is_session_waiting_for_beacon, &freq, &dr,
            &ping_slot_periodicity )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = is_session_started;
            cmd_output->buffer[1] = ( freq >> 24 ) & 0xff;
            cmd_output->buffer[2] = ( freq >> 16 ) & 0xff;
            cmd_output->buffer[3] = ( freq >> 8 ) & 0xff;
            cmd_output->buffer[4] = ( freq & 0xff );
            cmd_output->buffer[5] = dr;
            cmd_output->buffer[6] = is_session_waiting_for_beacon;
            cmd_output->buffer[7] = ping_slot_periodicity;

            cmd_output->length = 8;
        }
        break;
    }
    case CMD_MULTICAST_CLASS_B_STOP_SESSION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_stop_session( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_MULTICAST_CLASS_B_STOP_ALL_SESSIONS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_multicast_class_b_stop_all_sessions( STACK_ID )];
        break;
    }
    case CMD_LORAWAN_GET_LOST_CONNECTION_COUNTER:
    {
        uint16_t lost_connection_cnt = 0;

        cmd_output->return_code =
            rc_lut[smtc_modem_lorawan_get_lost_connection_counter( STACK_ID, &lost_connection_cnt )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( lost_connection_cnt >> 8 ) & 0xff;
            cmd_output->buffer[1] = ( lost_connection_cnt & 0xff );

            cmd_output->length = 2;
        }
        break;
    }
    case CMD_SET_ADR_ACK_LIMIT_DELAY:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_set_adr_ack_limit_delay( STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1] )];
        break;
    }
    case CMD_GET_ADR_ACK_LIMIT_DELAY:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_get_adr_ack_limit_delay( STACK_ID, &cmd_output->buffer[0], &cmd_output->buffer[1] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 2;
        }
        break;
    }
    case CMD_GET_LORAWAN_TIME:
    {
        uint32_t gps_time_s;
        uint32_t gps_fractional_s;

        cmd_output->return_code = rc_lut[smtc_modem_get_lorawan_mac_time( STACK_ID, &gps_time_s, &gps_fractional_s )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( gps_time_s >> 24 ) & 0xFF;
            cmd_output->buffer[1] = ( gps_time_s >> 16 ) & 0xFF;
            cmd_output->buffer[2] = ( gps_time_s >> 8 ) & 0xFF;
            cmd_output->buffer[3] = ( gps_time_s & 0xFF );
            cmd_output->buffer[4] = ( gps_fractional_s >> 24 ) & 0xFF;
            cmd_output->buffer[5] = ( gps_fractional_s >> 16 ) & 0xFF;
            cmd_output->buffer[6] = ( gps_fractional_s >> 8 ) & 0xFF;
            cmd_output->buffer[7] = ( gps_fractional_s & 0xFF );
            cmd_output->length    = 8;
        }
        break;
    }
    case CMD_SET_JOIN_DR_DISTRIBUTION:
    {
        cmd_output->return_code = rc_lut[smtc_modem_adr_set_join_distribution( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_LORAWAN_MAC_REQUEST:
    {
        cmd_output->return_code = rc_lut[smtc_modem_trig_lorawan_mac_request( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GET_LINK_CHECK_DATA:
    {
        uint8_t margin;
        uint8_t gw_cnt;
        cmd_output->return_code = rc_lut[smtc_modem_get_lorawan_link_check_data( STACK_ID, &margin, &gw_cnt )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = margin;
            cmd_output->buffer[1] = gw_cnt;
            cmd_output->length    = 2;
        }
        break;
    }
    case CMD_DEBUG_CONNECT_WITH_ABP:
    {
        uint32_t dev_addr = 0;

        dev_addr = cmd_input->buffer[0] << 24;
        dev_addr |= cmd_input->buffer[1] << 16;
        dev_addr |= cmd_input->buffer[2] << 8;
        dev_addr |= cmd_input->buffer[3];
        cmd_output->return_code = rc_lut[smtc_modem_debug_connect_with_abp( STACK_ID, dev_addr, &cmd_input->buffer[4],
                                                                            &cmd_input->buffer[20] )];
        break;
    }
    case CMD_CSMA_SET_STATE:
    {
#if defined( LR11XX ) || defined( SX126X )
        cmd_output->return_code = rc_lut[smtc_modem_csma_set_state( STACK_ID, cmd_input->buffer[0] )];
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_CSMA_GET_STATE:
    {
#if defined( LR11XX ) || defined( SX126X )
        bool enable;
        cmd_output->return_code = rc_lut[smtc_modem_csma_get_state( STACK_ID, &enable )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = enable;
            cmd_output->length    = 1;
        }
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_CSMA_SET_PARAMETERS:
    {
#if defined( LR11XX ) || defined( SX126X )
        if( cmd_input->buffer[1] > 1 )  // bo_enabled is a bool
        {
            cmd_output->return_code = CMD_RC_INVALID;
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_csma_set_parameters(
                STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], cmd_input->buffer[2] )];
        }
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_CSMA_GET_PARAMETERS:
    {
#if defined( LR11XX ) || defined( SX126X )
        uint8_t max_ch_change;
        bool    bo_enabled;
        uint8_t nb_bo_max;
        cmd_output->return_code =
            rc_lut[smtc_modem_csma_get_parameters( STACK_ID, &max_ch_change, &bo_enabled, &nb_bo_max )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = max_ch_change;
            cmd_output->buffer[1] = bo_enabled;
            cmd_output->buffer[2] = nb_bo_max;
            cmd_output->length    = 3;
        }
#else
        cmd_output->return_code = CMD_RC_NOT_IMPLEMENTED;
#endif
        break;
    }
    case CMD_STREAM_INIT:
    {
        uint8_t port             = cmd_input->buffer[0];
        uint8_t cipher           = cmd_input->buffer[1];
        uint8_t redundancy_ratio = cmd_input->buffer[2];

        cmd_output->return_code = rc_lut[smtc_modem_stream_init( STACK_ID, port, cipher, redundancy_ratio )];
        cmd_output->length      = 0;
        break;
    }
    case CMD_STREAM_ADD_DATA:
    {
        uint8_t* data           = &( cmd_input->buffer[0] );
        uint8_t  data_len       = cmd_input->length;
        cmd_output->return_code = rc_lut[smtc_modem_stream_add_data( STACK_ID, data, data_len )];
        cmd_output->length      = 0;
        break;
    }
    case CMD_STREAM_STATUS:
    {
        uint16_t pending = 0;
        uint16_t free    = 0;

        cmd_output->return_code = rc_lut[smtc_modem_stream_status( STACK_ID, &pending, &free )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length    = 4;
            cmd_output->buffer[0] = ( pending >> 8 ) & 0xff;
            cmd_output->buffer[1] = pending & 0xff;
            cmd_output->buffer[2] = ( free >> 8 ) & 0xff;
            cmd_output->buffer[3] = free & 0xff;
        }
        break;
    }
    case CMD_LFU_INIT:
    {
        uint16_t size          = 0;
        uint16_t average_delay = 0;

        size = cmd_input->buffer[2] << 8;
        size |= cmd_input->buffer[3];

        average_delay = cmd_input->buffer[4] << 8;
        average_delay |= cmd_input->buffer[5];

        file_size           = size;
        upload_status       = UPLOAD_NOT_INIT;
        upload_current_size = 0;
        // empty the file_storage buffer
        memset( file_store, 0, FILE_UPLOAD_MAX_SIZE );

        cmd_output->return_code = rc_lut[smtc_modem_file_upload_init(
            STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], file_store, file_size, average_delay )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            upload_status = UPLOAD_INIT;
        }
        break;
    }
    case CMD_LFU_DATA:
    {
        // First check if modem is in test mode
        if( modem_in_test_mode == true )
        {
            cmd_output->return_code = CMD_RC_BUSY;
        }
        else
        {
            // here is a emulation of upload data to keep compliant with modem-e
            if( ( upload_status != UPLOAD_INIT ) && ( upload_status != UPLOAD_DATA_ON_GOING ) )
            {
                cmd_output->return_code = CMD_RC_NOT_INIT;
                SMTC_HAL_TRACE_ERROR( "Upload file data, not init\n" );
            }
            else if( &cmd_input->buffer[0] == NULL )
            {
                cmd_output->return_code = CMD_RC_NOT_INIT;
                SMTC_HAL_TRACE_ERROR( "Upload file data, null\n" );
            }
            else if( ( upload_current_size + cmd_input->length ) > file_size )
            {
                cmd_output->return_code = CMD_RC_INVALID;
                SMTC_HAL_TRACE_ERROR( "Upload file data, size invalid\n" );
            }
            else if( upload_status == UPLOAD_STARTED )
            {
                cmd_output->return_code = CMD_RC_INVALID;
                SMTC_HAL_TRACE_ERROR( "Upload file still on going\n" );
            }
            else
            {
                memcpy( ( uint8_t* ) file_store + upload_current_size, &cmd_input->buffer[0], cmd_input->length );
                upload_current_size += cmd_input->length;

                upload_status = UPLOAD_DATA_ON_GOING;
            }
        }
        break;
    }
    case CMD_LFU_START:
    {
        uint32_t input_crc = 0;
        input_crc |= cmd_input->buffer[0] << 24;
        input_crc |= cmd_input->buffer[1] << 16;
        input_crc |= cmd_input->buffer[2] << 8;
        input_crc |= cmd_input->buffer[3];

        // check if file_size defined at upload_init cmd is equal to the actual received length
        if( file_size != upload_current_size )
        {
            cmd_output->return_code = CMD_RC_BAD_SIZE;
            SMTC_HAL_TRACE_ERROR( "Data size uploaded does not correspond to what was defined\n" );
            smtc_modem_file_upload_reset( STACK_ID );
        }
        else if( input_crc != cmd_parser_crc( file_store, file_size ) )
        {
            cmd_output->return_code = CMD_RC_BAD_CRC;
            SMTC_HAL_TRACE_ERROR( "Bad crc after uploading file data\n" );
            smtc_modem_file_upload_reset( STACK_ID );
        }
        else
        {
            cmd_output->return_code = rc_lut[smtc_modem_file_upload_start( STACK_ID )];
            if( cmd_output->return_code == CMD_RC_OK )
            {
                upload_status = UPLOAD_STARTED;
            }
        }
        break;
    }
    case CMD_LFU_RESET:
    {
        cmd_output->return_code = rc_lut[smtc_modem_file_upload_reset( STACK_ID )];
        break;
    }
    case CMD_DM_ENABLE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_enable( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_GET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_get_fport( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_DM_SET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_set_fport( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_GET_INFO_INTERVAL:
    {
        smtc_modem_dm_info_interval_format_t format;
        uint8_t                              interval;
        cmd_output->return_code = rc_lut[smtc_modem_dm_get_info_interval( STACK_ID, &format, &interval )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = interval & 0x3F;
            cmd_output->buffer[0] |= ( ( uint8_t ) format << 6 ) & 0xC0;
            cmd_output->length = 1;
        }
        break;
    }
    case CMD_DM_SET_INFO_INTERVAL:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_set_info_interval(
            STACK_ID, ( cmd_input->buffer[0] >> 6 ) & 0x03, cmd_input->buffer[0] & 0x3F )];
        break;
    }
    case CMD_DM_GET_PERIODIC_INFO_FIELDS:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_dm_get_periodic_info_fields( STACK_ID, &cmd_output->buffer[0], &cmd_output->length )];
        if( cmd_output->return_code != CMD_RC_OK )
        {
            cmd_output->length = 0;
        }
        break;
    }
    case CMD_DM_SET_PERIODIC_INFO_FIELDS:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_dm_set_periodic_info_fields( STACK_ID, &cmd_input->buffer[0], cmd_input->length )];
        break;
    }
    case CMD_DM_REQUEST_IMMEDIATE_INFO_FIELDS:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_dm_request_immediate_info_field( STACK_ID, &cmd_input->buffer[0], cmd_input->length )];
        break;
    }
    case CMD_DM_SET_USER_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_set_user_data( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_GET_USER_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_get_user_data( STACK_ID, &cmd_output->buffer[0] )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->length = SMTC_MODEM_DM_USER_DATA_LENGTH;
        }
        break;
    }
    case CMD_GET_STATUS:
    {
        smtc_modem_status_mask_t status_mask = { 0 };
        cmd_output->return_code              = rc_lut[smtc_modem_get_status( STACK_ID, &status_mask )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( uint8_t )( status_mask );
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_SUSPEND_RADIO_COMMUNICATIONS:
    {
        cmd_output->return_code = rc_lut[smtc_modem_suspend_radio_communications( cmd_input->buffer[0] )];
        break;
    }
    case CMD_DM_HANDLE_ALCSYNC:
    {
        cmd_output->return_code = rc_lut[smtc_modem_dm_handle_alcsync( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_SET_APPKEY:
    {
        cmd_output->return_code = rc_lut[smtc_modem_set_appkey( STACK_ID, &cmd_input->buffer[0] )];
        break;
    }
#if defined( STM32L476xx )
    case CMD_STORE_AND_FORWARD_SET_STATE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_set_state( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_STORE_AND_FORWARD_GET_STATE:
    {
        smtc_modem_store_and_forward_state_t state = { 0 };
        cmd_output->return_code                    = rc_lut[smtc_modem_store_and_forward_get_state( STACK_ID, &state )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = ( uint8_t )( state );
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_STORE_AND_FORWARD_ADD_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_flash_add_data(
            STACK_ID, cmd_input->buffer[0], cmd_input->buffer[1], &cmd_input->buffer[2], cmd_input->length - 2 )];
        break;
    }
    case CMD_STORE_AND_FORWARD_CLEAR_DATA:
    {
        cmd_output->return_code = rc_lut[smtc_modem_store_and_forward_flash_clear_data( STACK_ID )];
        break;
    }
    case CMD_STORE_AND_FORWARD_GET_FREE_SLOT:
    {
        uint32_t capacity  = 0;
        uint32_t free_slot = 0;
        cmd_output->return_code =
            rc_lut[smtc_modem_store_and_forward_flash_get_number_of_free_slot( STACK_ID, &capacity, &free_slot )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t idx               = 0;
            cmd_output->buffer[idx++] = ( capacity >> 24 ) & 0xFF;
            cmd_output->buffer[idx++] = ( capacity >> 16 ) & 0xFF;
            cmd_output->buffer[idx++] = ( capacity >> 8 ) & 0xFF;
            cmd_output->buffer[idx++] = ( capacity & 0xFF );
            cmd_output->buffer[idx++] = ( free_slot >> 24 ) & 0xFF;
            cmd_output->buffer[idx++] = ( free_slot >> 16 ) & 0xFF;
            cmd_output->buffer[idx++] = ( free_slot >> 8 ) & 0xFF;
            cmd_output->buffer[idx++] = ( free_slot & 0xFF );

            cmd_output->length = idx;
        }
        break;
    }
#endif  // STM32L476xx

#if defined( ADD_APP_GEOLOCATION ) && defined( STM32L476xx )
    case CMD_GNSS_SCAN:
    {
        smtc_modem_gnss_mode_t mode        = cmd_input->buffer[0];
        uint32_t               start_delay = 0;

        start_delay |= cmd_input->buffer[1] << 24;
        start_delay |= cmd_input->buffer[2] << 16;
        start_delay |= cmd_input->buffer[3] << 8;
        start_delay |= cmd_input->buffer[4];
        cmd_output->return_code = rc_lut[smtc_modem_gnss_scan( STACK_ID, mode, start_delay )];
        break;
    }
    case CMD_GNSS_SCAN_CANCEL:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_scan_cancel( STACK_ID )];
        break;
    }
    case CMD_GNSS_GET_EVENT_DATA_SCAN_DONE:
    {
        // Reset value of static scan data saved struct
        memset( &gnss_scan_data, 0, sizeof( smtc_modem_gnss_event_data_scan_done_t ) );

        // Get the value of the struct
        gnss_scan_done_rc       = rc_lut[smtc_modem_gnss_get_event_data_scan_done( STACK_ID, &gnss_scan_data )];
        cmd_output->return_code = gnss_scan_done_rc;

        if( gnss_scan_done_rc == CMD_RC_OK )
        {
            // Fill the output buffer
            uint8_t scan_done_offset = 0;

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.is_valid;

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.token;

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.nb_scans_valid;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.power_consumption_nah & 0xff );

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.context.mode;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.context.almanac_crc & 0xff );

            cmd_output->buffer[scan_done_offset++] = gnss_scan_data.indoor_detected;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.navgroup_duration_ms ) & 0xff;

            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp >> 24 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp >> 16 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp >> 8 ) & 0xff;
            cmd_output->buffer[scan_done_offset++] = ( gnss_scan_data.timestamp ) & 0xff;

            cmd_output->length = scan_done_offset;
        }
        break;
    }
    case CMD_GNSS_GET_SCAN_DONE_RAW_DATA_LIST:
    {
        cmd_output->return_code = gnss_scan_done_rc;
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t raw_data_offset = 0;
            for( uint8_t scan_index = 0; scan_index < gnss_scan_data.nb_scans_valid; scan_index++ )
            {
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp >> 24 ) & 0xff;
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp >> 16 ) & 0xff;
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp >> 8 ) & 0xff;
                cmd_output->buffer[raw_data_offset++] = ( gnss_scan_data.scans[scan_index].timestamp & 0xff );

                cmd_output->buffer[raw_data_offset++] = gnss_scan_data.scans[scan_index].nav_size;

                memcpy( &cmd_output->buffer[raw_data_offset], gnss_scan_data.scans[scan_index].nav,
                        gnss_scan_data.scans[scan_index].nav_size );

                raw_data_offset += gnss_scan_data.scans[scan_index].nav_size;
            }
            // At the end of the scan loop the size of the buff is known
            cmd_output->length = raw_data_offset;
        }
        break;
    }
    case CMD_GNSS_GET_SCAN_DONE_METADATA_LIST:
    {
        cmd_output->return_code = gnss_scan_done_rc;
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t metadata_offset = 0;
            for( uint8_t scan_index = 0; scan_index < gnss_scan_data.nb_scans_valid; scan_index++ )
            {
                uint32_t lat_x10000 = ( uint32_t )( gnss_scan_data.scans[scan_index].aiding_position.latitude * 10000 );

                cmd_output->buffer[metadata_offset++] = ( lat_x10000 >> 24 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( lat_x10000 >> 16 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( lat_x10000 >> 8 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( lat_x10000 & 0xff );

                uint32_t long_x10000 =
                    ( uint32_t )( gnss_scan_data.scans[scan_index].aiding_position.longitude * 10000 );

                cmd_output->buffer[metadata_offset++] = ( long_x10000 >> 24 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( long_x10000 >> 16 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( long_x10000 >> 8 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( long_x10000 & 0xff );

                cmd_output->buffer[metadata_offset++] = gnss_scan_data.scans[scan_index].scan_mode_launched;

                cmd_output->buffer[metadata_offset++] =
                    ( gnss_scan_data.scans[scan_index].scan_duration_ms >> 24 ) & 0xff;
                cmd_output->buffer[metadata_offset++] =
                    ( gnss_scan_data.scans[scan_index].scan_duration_ms >> 16 ) & 0xff;
                cmd_output->buffer[metadata_offset++] =
                    ( gnss_scan_data.scans[scan_index].scan_duration_ms >> 8 ) & 0xff;
                cmd_output->buffer[metadata_offset++] = ( gnss_scan_data.scans[scan_index].scan_duration_ms & 0xff );
            }
            // At the end of the scan loop the size of the buff is known
            cmd_output->length = metadata_offset;
        }
        break;
    }

    case CMD_GNSS_GET_SCAN_DONE_SCAN_SV:
    {
        cmd_output->return_code = gnss_scan_done_rc;
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t sv_index = 0;
            for( uint8_t scan_index = 0; scan_index < gnss_scan_data.nb_scans_valid; scan_index++ )
            {
                cmd_output->buffer[sv_index++] = gnss_scan_data.scans[scan_index].nb_svs;

                for( uint8_t nav_index = 0; nav_index < gnss_scan_data.scans[scan_index].nb_svs; nav_index++ )
                {
                    cmd_output->buffer[sv_index++] = gnss_scan_data.scans[scan_index].info_svs[nav_index].satellite_id;
                    cmd_output->buffer[sv_index++] = gnss_scan_data.scans[scan_index].info_svs[nav_index].cnr;
                    cmd_output->buffer[sv_index++] =
                        ( gnss_scan_data.scans[scan_index].info_svs[nav_index].doppler >> 8 ) & 0xff;
                    cmd_output->buffer[sv_index++] =
                        ( gnss_scan_data.scans[scan_index].info_svs[nav_index].doppler & 0xff );
                }
            }

            // At the end of the scan loop the size of the buff is known
            cmd_output->length = sv_index;
        }
        break;
    }
    case CMD_GNSS_GET_EVENT_DATA_TERMINATED:
    {
        smtc_modem_gnss_event_data_terminated_t gnss_data_terminated = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_gnss_get_event_data_terminated( STACK_ID, &gnss_data_terminated )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = gnss_data_terminated.nb_scans_sent;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_GNSS_SET_CONST:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_set_constellations( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_SET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_set_port( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_SCAN_AGGREGATE:
    {
        smtc_modem_gnss_scan_aggregate( STACK_ID, cmd_input->buffer[0] );
        // Above function do not have return code
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_GNSS_SEND_MODE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_gnss_send_mode( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_ALM_DEMOD_START:
    {
        cmd_output->return_code = rc_lut[smtc_modem_almanac_demodulation_start( STACK_ID )];
        break;
    }
    case CMD_GNSS_ALM_DEMOD_SET_CONSTEL:
    {
        cmd_output->return_code =
            rc_lut[smtc_modem_almanac_demodulation_set_constellations( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_GNSS_ALM_DEMOD_GET_EVENT_DATA_ALM_UPD:
    {
        smtc_modem_almanac_demodulation_event_data_almanac_update_t event_data_alm_update = { 0 };
        cmd_output->return_code =
            rc_lut[smtc_modem_almanac_demodulation_get_event_data_almanac_update( STACK_ID, &event_data_alm_update )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t index = 0;

            cmd_output->buffer[index++] = event_data_alm_update.status_gps;
            cmd_output->buffer[index++] = event_data_alm_update.status_beidou;
            cmd_output->buffer[index++] = event_data_alm_update.update_progress_gps;
            cmd_output->buffer[index++] = event_data_alm_update.update_progress_beidou;

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_done & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_update_from_sat_success & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_nb_aborted_by_rp & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.stat_cumulative_timings_s & 0xff );

            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( event_data_alm_update.power_consumption_nah & 0xff );

            cmd_output->length = index;
        }
        break;
    }
    case CMD_CLOUD_ALMANAC_START:
    {
        cmd_output->return_code = rc_lut[smtc_modem_almanac_start( STACK_ID )];
        break;
    }
    case CMD_CLOUD_ALMANAC_STOP:
    {
        cmd_output->return_code = rc_lut[smtc_modem_almanac_stop( STACK_ID )];
        break;
    }
    case CMD_WIFI_SCAN_START:
    {
        uint32_t start_delay = 0;

        start_delay |= cmd_input->buffer[0] << 24;
        start_delay |= cmd_input->buffer[1] << 16;
        start_delay |= cmd_input->buffer[2] << 8;
        start_delay |= cmd_input->buffer[3];
        cmd_output->return_code = rc_lut[smtc_modem_wifi_scan( STACK_ID, start_delay )];
        break;
    }
    case CMD_WIFI_SCAN_CANCEL:
    {
        cmd_output->return_code = rc_lut[smtc_modem_wifi_scan_cancel( STACK_ID )];
        break;
    }
    case CMD_WIFI_GET_SCAN_DONE_SCAN_DATA:
    {
        smtc_modem_wifi_event_data_scan_done_t wifi_scan_done_data = { 0 };
        cmd_output->return_code = rc_lut[smtc_modem_wifi_get_event_data_scan_done( STACK_ID, &wifi_scan_done_data )];
        if( cmd_output->return_code == CMD_RC_OK )
        {
            uint8_t index = 0;

            cmd_output->buffer[index++] = wifi_scan_done_data.nbr_results;

            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.power_consumption_nah & 0xff );

            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms >> 24 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms >> 16 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms >> 8 ) & 0xff;
            cmd_output->buffer[index++] = ( wifi_scan_done_data.scan_duration_ms & 0xff );

            for( uint8_t scan_index = 0; scan_index < wifi_scan_done_data.nbr_results; scan_index++ )
            {
                // copy LR11XX_WIFI_MAC_ADDRESS_LENGTH bytes of mac adress
                memcpy( &cmd_output->buffer[index], wifi_scan_done_data.results[scan_index].mac_address,
                        LR11XX_WIFI_MAC_ADDRESS_LENGTH );
                index += LR11XX_WIFI_MAC_ADDRESS_LENGTH;

                cmd_output->buffer[index++] = wifi_scan_done_data.results[scan_index].channel;
                cmd_output->buffer[index++] = wifi_scan_done_data.results[scan_index].type;
                cmd_output->buffer[index++] = wifi_scan_done_data.results[scan_index].rssi;
            }
            cmd_output->length = index;
        }
        break;
    }
    case CMD_WIFI_GET_EVENT_DATA_TERMINATED:
    {
        smtc_modem_wifi_event_data_terminated_t wifi_data_terminated = { 0 };

        cmd_output->return_code = rc_lut[smtc_modem_wifi_get_event_data_terminated( STACK_ID, &wifi_data_terminated )];

        if( cmd_output->return_code == CMD_RC_OK )
        {
            cmd_output->buffer[0] = wifi_data_terminated.nb_scans_sent;
            cmd_output->length    = 1;
        }
        break;
    }
    case CMD_WIFI_SET_PORT:
    {
        cmd_output->return_code = rc_lut[smtc_modem_wifi_set_port( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_WIFI_SEND_MODE:
    {
        cmd_output->return_code = rc_lut[smtc_modem_wifi_send_mode( STACK_ID, cmd_input->buffer[0] )];
        break;
    }
    case CMD_WIFI_SET_PAYLOAD_FORMAT:
    {
        smtc_modem_wifi_set_payload_format( STACK_ID, cmd_input->buffer[0] );
        // Above function do not have return code
        cmd_output->return_code = CMD_RC_OK;
        break;
    }
    case CMD_LR11XX_RADIO_READ:
    {
        // First check if modem is in test mode
        if( modem_in_test_mode == true )
        {
            cmd_output->return_code = CMD_RC_BUSY;
        }
        else
        {
#if defined( LR11XX_TRANSCEIVER )
            uint8_t command_length = cmd_input->buffer[0];
            uint8_t command[255]   = { 0 };
            memcpy( command, &cmd_input->buffer[1], command_length );

            uint8_t data_length = cmd_input->buffer[command_length + 1];
            uint8_t data[255]   = { 0 };
            if( lr11xx_hal_read( NULL, command, command_length, data, data_length ) != LR11XX_HAL_STATUS_OK )
            {
                cmd_output->return_code = CMD_RC_FAIL;
            }
            else
            {
                cmd_output->return_code = CMD_RC_OK;
            }
            memcpy( cmd_output->buffer, data, data_length );
            cmd_output->length = data_length;
#else
            cmd_output->return_code = CMD_RC_FAIL;
#endif
        }
        break;
    }
    case CMD_LR11XX_RADIO_WRITE:
    {
        // First check if modem is in test mode
        if( modem_in_test_mode == true )
        {
            cmd_output->return_code = CMD_RC_BUSY;
        }
        else
        {
#if defined( LR11XX_TRANSCEIVER )
            uint8_t command_length = cmd_input->buffer[0];
            uint8_t command[255]   = { 0 };
            memcpy( command, &cmd_input->buffer[1], command_length );

            uint8_t data_length = cmd_input->buffer[command_length + 1];
            uint8_t data[255]   = { 0 };
            memcpy( data, &cmd_input->buffer[command_length + 2], data_length );
            if( lr11xx_hal_write( NULL, command, command_length, data, data_length ) != LR11XX_HAL_STATUS_OK )
            {
                cmd_output->return_code = CMD_RC_FAIL;
            }
            else
            {
                cmd_output->return_code = CMD_RC_OK;
            }
            cmd_output->length = 0;
#else
            cmd_output->return_code = CMD_RC_FAIL;
#endif
        }
        break;
    }

#endif  // ADD_APP_GEOLOCATION && STM32L476xx
    default:
    {
        SMTC_HAL_TRACE_ERROR( "Unknown command (0x%x)\n", cmd_input->cmd_code );
        cmd_output->return_code = CMD_RC_UNKNOWN;
        cmd_output->length      = 0;
        return PARSE_ERROR;
        break;
    }
    }
    cmd_input->cmd_code = CMD_MAX;
    cmd_input->length   = 0;

    return ret;
}

cmd_parse_status_t cmd_test_parser( cmd_tst_input_t* cmd_tst_input, cmd_tst_response_t* cmd_tst_output )
{
    cmd_parse_status_t ret = PARSE_OK;

    if( ( cmd_tst_input->cmd_code >= CMD_TST_MAX ) ||
        ( host_cmd_test_tab[cmd_tst_input->cmd_code][HOST_CMD_TAB_IDX_AVAILABILITY] != 1 ) )
    {
        SMTC_HAL_TRACE_ERROR( "Unknown command test (0x%x)\n", cmd_tst_input->cmd_code );
        cmd_tst_output->return_code = CMD_RC_UNKNOWN;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    if( cmd_test_parser_check_cmd_size( cmd_tst_input->cmd_code, cmd_tst_input->length ) == CMD_LENGTH_NOT_VALID )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid size command test (0x%x)\n", cmd_tst_input->cmd_code );
        cmd_tst_output->return_code = CMD_RC_BAD_SIZE;
        cmd_tst_output->length      = 0;
        return PARSE_ERROR;
    }

    cmd_tst_output->return_code = CMD_RC_OK;  // by default the return code is ok and length is 0
    cmd_tst_output->length      = 0;

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_HAL_TRACE_WARNING( "\tCMD_TST_%s (0x%02x)\n", host_cmd_test_str[cmd_tst_input->cmd_code],
                            cmd_tst_input->cmd_code );
#endif
    switch( cmd_tst_input->cmd_code )
    {
    case CMD_TST_START:
    {
        if( strncmp( ( char* ) cmd_tst_input->buffer, "TESTTEST", 8 ) == 0 )
        {
            cmd_tst_output->return_code = rc_lut[smtc_modem_test_start( )];
            if( cmd_tst_output->return_code == CMD_RC_OK )
            {
                modem_in_test_mode = true;
            }
        }
        else
        {
            SMTC_HAL_TRACE_ERROR( "TST MODE: invalid enablement payload\n" );
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }

        break;
    }
    case CMD_TST_NOP:
    {
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_nop( )];
        break;
    }
    case CMD_TST_TX_SINGLE:
    {
        uint8_t raw_sf = cmd_tst_input->buffer[5];
        uint8_t raw_bw = cmd_tst_input->buffer[6];
        uint8_t raw_cr = cmd_tst_input->buffer[7];

        // integrity of the data shall be tested here as we perform a data change using LUT before calling api
        // function
        if( cmd_test_parser_check_sf_bw_cr( raw_sf, raw_bw, raw_cr ) == PARSE_OK )
        {
            smtc_modem_test_sf_t sf = cmd_test_sf_table[raw_sf];
            smtc_modem_test_bw_t bw = cmd_test_bw_table[raw_bw];
            smtc_modem_test_cr_t cr = cmd_test_cr_table[raw_cr];

            int8_t  pw  = cmd_tst_input->buffer[4];
            uint8_t len = cmd_tst_input->buffer[8];

            uint32_t freq = 0;
            freq |= cmd_tst_input->buffer[0] << 24;
            freq |= cmd_tst_input->buffer[1] << 16;
            freq |= cmd_tst_input->buffer[2] << 8;
            freq |= cmd_tst_input->buffer[3];

            cmd_tst_output->return_code = rc_lut[smtc_modem_test_tx( NULL, len, freq, pw, sf, bw, cr, 8, false )];
        }
        else
        {
            // one of the data is not in the accepted range => reject command
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_TX_CONT:
    {
        uint8_t raw_sf = cmd_tst_input->buffer[5];
        uint8_t raw_bw = cmd_tst_input->buffer[6];
        uint8_t raw_cr = cmd_tst_input->buffer[7];

        // integrity of the data shall be tested here as we perform a data change using LUT before calling api
        // function
        if( cmd_test_parser_check_sf_bw_cr( raw_sf, raw_bw, raw_cr ) == PARSE_OK )
        {
            smtc_modem_test_sf_t sf = cmd_test_sf_table[raw_sf];
            smtc_modem_test_bw_t bw = cmd_test_bw_table[raw_bw];
            smtc_modem_test_cr_t cr = cmd_test_cr_table[raw_cr];

            int8_t  pw  = cmd_tst_input->buffer[4];
            uint8_t len = cmd_tst_input->buffer[8];

            uint32_t freq = 0;
            freq |= cmd_tst_input->buffer[0] << 24;
            freq |= cmd_tst_input->buffer[1] << 16;
            freq |= cmd_tst_input->buffer[2] << 8;
            freq |= cmd_tst_input->buffer[3];

            cmd_tst_output->return_code = rc_lut[smtc_modem_test_tx( NULL, len, freq, pw, sf, bw, cr, 8, true )];
        }
        else
        {
            // one of the data is not in the accepted range => reject command
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_TX_HOP:
    {
        cmd_tst_output->return_code = CMD_RC_NOT_IMPLEMENTED;
        cmd_tst_output->length      = 0;
        break;
    }
    case CMD_TST_TX_CW:
    {
        int8_t pw = cmd_tst_input->buffer[4];

        uint32_t freq = 0;
        freq |= cmd_tst_input->buffer[0] << 24;
        freq |= cmd_tst_input->buffer[1] << 16;
        freq |= cmd_tst_input->buffer[2] << 8;
        freq |= cmd_tst_input->buffer[3];

        cmd_tst_output->return_code = rc_lut[smtc_modem_test_tx_cw( freq, pw )];
        break;
    }
    case CMD_TST_RX_CONT:
    {
        uint8_t raw_sf = cmd_tst_input->buffer[4];
        uint8_t raw_bw = cmd_tst_input->buffer[5];
        uint8_t raw_cr = cmd_tst_input->buffer[6];

        // integrity of the data shall be tested here as we perform a data change using LUT before calling api
        // function
        if( cmd_test_parser_check_sf_bw_cr( raw_sf, raw_bw, raw_cr ) == PARSE_OK )
        {
            smtc_modem_test_sf_t sf = cmd_test_sf_table[raw_sf];
            smtc_modem_test_bw_t bw = cmd_test_bw_table[raw_bw];
            smtc_modem_test_cr_t cr = cmd_test_cr_table[raw_cr];

            uint32_t freq = 0;
            freq |= cmd_tst_input->buffer[0] << 24;
            freq |= cmd_tst_input->buffer[1] << 16;
            freq |= cmd_tst_input->buffer[2] << 8;
            freq |= cmd_tst_input->buffer[3];

            cmd_tst_output->return_code = rc_lut[smtc_modem_test_rx_continuous( freq, sf, bw, cr )];
        }
        else
        {
            // one of the data is not in the accepted range => reject command
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_READ_NB_PKTS_RX_CONT:
    {
        uint32_t nb_read_pkt        = 0;
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_get_nb_rx_packets( &nb_read_pkt )];
        cmd_tst_output->buffer[0]   = ( nb_read_pkt >> 24 ) & 0xFF;
        cmd_tst_output->buffer[1]   = ( nb_read_pkt >> 16 ) & 0xFF;
        cmd_tst_output->buffer[2]   = ( nb_read_pkt >> 8 ) & 0xFF;
        cmd_tst_output->buffer[3]   = ( nb_read_pkt & 0xFF );
        cmd_tst_output->length      = 4;
        break;
    }
    case CMD_TST_RSSI:
    {
        uint8_t raw_bw = cmd_tst_input->buffer[6];
        if( raw_bw < SMTC_MODEM_TEST_BW_COUNT )
        {
            smtc_modem_test_bw_t bw = cmd_test_bw_table[raw_bw];

            uint32_t freq = 0;
            freq |= cmd_tst_input->buffer[0] << 24;
            freq |= cmd_tst_input->buffer[1] << 16;
            freq |= cmd_tst_input->buffer[2] << 8;
            freq |= cmd_tst_input->buffer[3];

            uint16_t time_ms = 0;
            time_ms |= cmd_tst_input->buffer[4] << 8;
            time_ms |= cmd_tst_input->buffer[5];

            cmd_tst_output->return_code = rc_lut[smtc_modem_test_rssi( freq, bw, time_ms )];
        }
        else
        {
            // bw is not in the accepted range => reject command
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_RSSI_GET:
    {
        int8_t rssi                 = 0;
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_get_rssi( &rssi )];
        cmd_tst_output->buffer[0]   = rssi;
        cmd_tst_output->length      = 1;
        break;
    }
    case CMD_TST_RADIO_RST:
    {
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_radio_reset( )];
        break;
    }
    case CMD_TST_EXIT:
    {
        cmd_tst_output->return_code = rc_lut[smtc_modem_test_stop( )];
        if( cmd_tst_output->return_code == CMD_RC_OK )
        {
            modem_in_test_mode = false;
        }
        break;
    }
    case CMD_TST_BUSYLOOP:
    {
        // First check if modem is in test mode
        if( modem_in_test_mode == true )
        {
            // Endless loop
            while( 1 )
            {
            };
        }
        else
        {
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_PANIC:
    {
        // First check if modem is in test mode
        if( modem_in_test_mode == true )
        {
            SMTC_MODEM_HAL_PANIC( "TEST PANIC" );
        }
        else
        {
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_WATCHDOG:
    {
        // First check if modem is in test mode
        if( modem_in_test_mode == true )
        {
            hal_mcu_disable_irq( );
            // Endless loop
            while( 1 )
            {
            };
            break;
        }
        else
        {
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    case CMD_TST_RADIO_READ:
    {
        uint8_t command_length = cmd_tst_input->buffer[0];
        uint8_t command[255]   = { 0 };
        memcpy( command, &cmd_tst_input->buffer[1], command_length );

        uint8_t data_length = cmd_tst_input->buffer[command_length + 1];
        uint8_t data[255]   = { 0 };

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_direct_radio_read( command, command_length, data, data_length )];
        memcpy( cmd_tst_output->buffer, data, data_length );
        cmd_tst_output->length = data_length;
        break;
    }
    case CMD_TST_RADIO_WRITE:
    {
        uint8_t command_length = cmd_tst_input->buffer[0];
        uint8_t command[255]   = { 0 };
        memcpy( command, &cmd_tst_input->buffer[1], command_length );

        uint8_t data_length = cmd_tst_input->buffer[command_length + 1];
        uint8_t data[255]   = { 0 };
        memcpy( data, &cmd_tst_input->buffer[command_length + 1], data_length );

        cmd_tst_output->return_code =
            rc_lut[smtc_modem_test_direct_radio_write( command, command_length, data, data_length )];
        cmd_tst_output->length = 0;
        break;
    }
    case CMD_TST_TX_SINGLE_PREAM:
    {
        uint8_t raw_sf = cmd_tst_input->buffer[5];
        uint8_t raw_bw = cmd_tst_input->buffer[6];
        uint8_t raw_cr = cmd_tst_input->buffer[7];

        // integrity of the data shall be tested here as we perform a data change using LUT before calling api
        // function
        if( cmd_test_parser_check_sf_bw_cr( raw_sf, raw_bw, raw_cr ) == PARSE_OK )
        {
            smtc_modem_test_sf_t sf  = cmd_test_sf_table[raw_sf];
            smtc_modem_test_bw_t bw  = cmd_test_bw_table[raw_bw];
            smtc_modem_test_cr_t cr  = cmd_test_cr_table[raw_cr];
            int8_t               pw  = cmd_tst_input->buffer[4];
            uint8_t              len = cmd_tst_input->buffer[8];

            uint32_t freq = 0;
            freq |= cmd_tst_input->buffer[0] << 24;
            freq |= cmd_tst_input->buffer[1] << 16;
            freq |= cmd_tst_input->buffer[2] << 8;
            freq |= cmd_tst_input->buffer[3];

            uint16_t preamble_length = 0;
            preamble_length |= cmd_tst_input->buffer[9] << 8;
            preamble_length |= cmd_tst_input->buffer[10];

            cmd_tst_output->return_code =
                rc_lut[smtc_modem_test_tx( NULL, len, freq, pw, sf, bw, cr, preamble_length, false )];
        }
        else
        {
            // one of the data is not in the accepted range => reject command
            cmd_tst_output->return_code = CMD_RC_INVALID;
        }
        break;
    }
    default:
    {
        cmd_tst_output->return_code = CMD_RC_UNKNOWN;
        cmd_tst_output->length      = 0;
        break;
    }
    }

    // Erase test command content to avoid twice calls
    cmd_tst_input->cmd_code = CMD_TST_MAX;
    cmd_tst_input->length   = 0;

    return ( ret );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static cmd_length_valid_t cmd_parser_check_cmd_size( host_cmd_id_t cmd_id, uint8_t length )
{
    // cmd len too small
    if( length < host_cmd_tab[cmd_id][HOST_CMD_TAB_IDX_MIN_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Command size too small\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( length > host_cmd_tab[cmd_id][HOST_CMD_TAB_IDX_MAX_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Command size too long\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    return CMD_LENGTH_VALID;
}

static cmd_length_valid_t cmd_test_parser_check_cmd_size( host_cmd_test_id_t tst_id, uint8_t length )
{
    // cmd len too small
    if( length < host_cmd_test_tab[tst_id][HOST_CMD_TAB_IDX_MIN_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid command test size (too small)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    // cmd len too long
    if( length > host_cmd_test_tab[tst_id][HOST_CMD_TAB_IDX_MAX_LENGTH] )
    {
        SMTC_HAL_TRACE_ERROR( "Invalid command test size (too long)\n" );
        return CMD_LENGTH_NOT_VALID;
    }
    return CMD_LENGTH_VALID;
}

static cmd_parse_status_t cmd_test_parser_check_sf_bw_cr( uint8_t sf, uint8_t bw, uint8_t cr )
{
    if( sf >= SMTC_MODEM_TEST_LORA_SF_COUNT )
    {
        return PARSE_ERROR;
    }
    else if( bw >= SMTC_MODEM_TEST_BW_COUNT )
    {
        return PARSE_ERROR;
    }
    else if( cr >= SMTC_MODEM_TEST_CR_COUNT )
    {
        return PARSE_ERROR;
    }
    else
    {
        return PARSE_OK;
    }
}

uint32_t cmd_parser_crc( const uint8_t* buf, int len )
{
    uint32_t crc = 0xFFFFFFFF;
    while( len-- > 0 )
    {
        crc = crc ^ *buf++;
        for( int i = 0; i < 8; i++ )
        {
            uint32_t mask = -( crc & 1 );
            crc           = ( crc >> 1 ) ^ ( 0xEDB88320 & mask );
        }
    }
    return ~crc;
}

/* --- EOF ------------------------------------------------------------------ */
