/**
 * @file      lorawan_certification.h
 *
 * @brief     LoRaWAN Certification Protocol Headers, Package Identifier 6, Package Version 1
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

#ifndef LORAWAN_CERTIFICATION_H
#define LORAWAN_CERTIFICATION_H

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
 * @brief Package information
 */
#define LORAWAN_CERTIFICATION_PACKAGE_IDENTIFIER 6
#define LORAWAN_CERTIFICATION_PACKAGE_VERSION 1

/**
 * @brief Commandes length in byte(s)
 */
#define LORAWAN_CERTIFICATION_PACKAGE_VERSION_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_PACKAGE_VERSION_ANS_SIZE 2
#define LORAWAN_CERTIFICATION_DUT_RESET_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_DUT_JOIN_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_TX_FRAME_CTRL_REQ_SIZE 242
#define LORAWAN_CERTIFICATION_ECHO_PLAY_REQ_SIZE 242
#define LORAWAN_CERTIFICATION_ECHO_PLAY_ANS_SIZE 242
#define LORAWAN_CERTIFICATION_RX_APP_CNT_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_RX_APP_CNT_ANS_SIZE 3
#define LORAWAN_CERTIFICATION_RX_APP_CNT_RESET_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_LINK_CHECK_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_DEVICE_TIME_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_PING_SLOT_INFO_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_TX_CW_REQ_SIZE 7
#define LORAWAN_CERTIFICATION_DUT_FPORT_224_DISABLE_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_DUT_VERSION_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_DUT_VERSION_ANS_SIZE 13

#define LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_CTRL_SIZE 2
#define LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_SIZE 22
#define LORAWAN_CERTIFICATION_BEACON_CNT_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_BEACON_CNT_ANS_SIZE 6
#define LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ_SIZE 1

#define LORAWAN_CERTIFICATION_FRAG_SESSION_CNT_REQ_SIZE 2

#define LORAWAN_CERTIFICATION_RELAY_MODE_CTRL_SIZE 2

#define LORAWAN_CERTIFICATION_FPORT 224
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LoRaWAN certification return type
 *
 */
typedef enum lorawan_certification_ret_e
{
    LORAWAN_CERTIFICATION_OK,
    LORAWAN_CERTIFICATION_ERROR,
} lorawan_certification_ret_t;

/**
 * @brief Command ID Ans/Req by the Device Under Test
 *
 * @enum lorawan_certification_cid_dut_t
 */
typedef enum lorawan_certification_cid_dut_e
{
    LORAWAN_CERTIFICATION_PACKAGE_VERSION_ANS  = 0x00,
    LORAWAN_CERTIFICATION_ECHO_PLAY_ANS        = 0x08,
    LORAWAN_CERTIFICATION_RX_APP_CNT_ANS       = 0x09,
    LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND = 0x41,
    LORAWAN_CERTIFICATION_BEACON_CNT_ANS       = 0x43,
    LORAWAN_CERTIFICATION_FRAG_SESSION_CNT_ANS = 0x52,
    LORAWAN_CERTIFICATION_DUT_VERSION_ANS      = 0x7F,
} lorawan_certification_cid_dut_t;

/**
 * @brief Command ID Ans/Req by Test Control Layer of the Test Harness
 *
 * @enum lorawan_certification_cid_tcl_t
 */
typedef enum lorawan_certification_cid_tcl_e
{
    LORAWAN_CERTIFICATION_PACKAGE_VERSION_REQ          = 0x00,
    LORAWAN_CERTIFICATION_DUT_RESET_REQ                = 0x01,
    LORAWAN_CERTIFICATION_DUT_JOIN_REQ                 = 0x02,
    LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ             = 0x03,
    LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ           = 0x04,
    LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ = 0x05,
    LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ    = 0x06,
    LORAWAN_CERTIFICATION_TX_FRAME_CTRL_REQ            = 0x07,
    LORAWAN_CERTIFICATION_ECHO_PLAY_REQ                = 0x08,
    LORAWAN_CERTIFICATION_RX_APP_CNT_REQ               = 0x09,
    LORAWAN_CERTIFICATION_RX_APP_CNT_RESET_REQ         = 0x0A,
    LORAWAN_CERTIFICATION_LINK_CHECK_REQ               = 0x20,
    LORAWAN_CERTIFICATION_DEVICE_TIME_REQ              = 0x21,
    LORAWAN_CERTIFICATION_PING_SLOT_INFO_REQ           = 0x22,
    LORAWAN_CERTIFICATION_BEACON_RX_STATUS_IND_CTRL    = 0x40,
    LORAWAN_CERTIFICATION_BEACON_CNT_REQ               = 0x42,
    LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ           = 0x44,
    LORAWAN_CERTIFICATION_FRAG_SESSION_CNT_REQ         = 0x52,
    LORAWAN_CERTIFICATION_RELAY_MODE_CTRL_REQ          = 0x53,
    LORAWAN_CERTIFICATION_TX_CW_REQ                    = 0x7D,
    LORAWAN_CERTIFICATION_DUT_FPORT_224_DISABLE_REQ    = 0x7E,
    LORAWAN_CERTIFICATION_DUT_VERSION_REQ              = 0x7F,
} lorawan_certification_cid_tcl_t;

/**
 * @brief LoRaWAN Certification Class Enumeration
 *
 */
typedef enum lorawan_certification_class_e
{
    LORAWAN_CERTIFICATION_CLASS_A = 0x00,
    LORAWAN_CERTIFICATION_CLASS_B,
    LORAWAN_CERTIFICATION_CLASS_C,
} lorawan_certification_class_t;

/**
 * @brief LoRaWAN Certification ADR Enumeration
 *
 */
typedef enum lorawan_certification_adr_e
{
    LORAWAN_CERTIFICATION_ADR_OFF = 0x00,
    LORAWAN_CERTIFICATION_ADR_ON,
} lorawan_certification_adr_t;

/**
 * @brief LoRaWAN Certification Duty-Cycle Enumeration
 *
 */
typedef enum lorawan_certification_duty_cycle_e
{
    LORAWAN_CERTIFICATION_DUTY_CYCLE_OFF = 0x00,
    LORAWAN_CERTIFICATION_DUTY_CYCLE_ON,
} lorawan_certification_duty_cycle_t;

/**
 * @brief LoRaWAN Certification Periodicity Enumeration
 *
 */
typedef enum lorawan_certification_periodicity_e
{
    LORAWAN_CERTIFICATION_PERIODICITY_0 = 0,  // Default DUT application behavior
    LORAWAN_CERTIFICATION_PERIODICITY_5,
    LORAWAN_CERTIFICATION_PERIODICITY_10,
    LORAWAN_CERTIFICATION_PERIODICITY_20,
    LORAWAN_CERTIFICATION_PERIODICITY_30,
    LORAWAN_CERTIFICATION_PERIODICITY_40,
    LORAWAN_CERTIFICATION_PERIODICITY_50,
    LORAWAN_CERTIFICATION_PERIODICITY_60,
    LORAWAN_CERTIFICATION_PERIODICITY_120,
    LORAWAN_CERTIFICATION_PERIODICITY_240,
    LORAWAN_CERTIFICATION_PERIODICITY_480,
    LORAWAN_CERTIFICATION_PERIODICITY_MAX,
} lorawan_certification_periodicity_t;

/**
 * @brief LoRaWAN Certification Periodicity Table converter
 *
 */
static const uint16_t lorawan_certification_periodicity_table_s[LORAWAN_CERTIFICATION_PERIODICITY_MAX] = {
    [LORAWAN_CERTIFICATION_PERIODICITY_0] = 0,     [LORAWAN_CERTIFICATION_PERIODICITY_5] = 5,
    [LORAWAN_CERTIFICATION_PERIODICITY_10] = 10,   [LORAWAN_CERTIFICATION_PERIODICITY_20] = 20,
    [LORAWAN_CERTIFICATION_PERIODICITY_30] = 30,   [LORAWAN_CERTIFICATION_PERIODICITY_40] = 40,
    [LORAWAN_CERTIFICATION_PERIODICITY_50] = 50,   [LORAWAN_CERTIFICATION_PERIODICITY_60] = 60,
    [LORAWAN_CERTIFICATION_PERIODICITY_120] = 120, [LORAWAN_CERTIFICATION_PERIODICITY_240] = 240,
    [LORAWAN_CERTIFICATION_PERIODICITY_480] = 480,
};

/**
 * @brief LoRaWAN Certification Frame Type Enumeration
 *
 */
typedef enum lorawan_certification_frame_type_e
{
    LORAWAN_CERTIFICATION_FRAME_TYPE_NO_CHANGE   = 0x00,  //!> No Change
    LORAWAN_CERTIFICATION_FRAME_TYPE_UNCONFIRMED = 0x01,  //!> Unconfirmed
    LORAWAN_CERTIFICATION_FRAME_TYPE_CONFIRMED   = 0x02,  //!> Confirmed
} lorawan_certification_frame_type_t;

typedef enum lorawan_certification_requested_tx_type_e
{
    LORAWAN_CERTIFICATION_NO_TX_REQ     = 0x00,
    LORAWAN_CERTIFICATION_TX_CERTIF_REQ = 0x01,
    LORAWAN_CERTIFICATION_TX_MAC_REQ    = 0x02,
    LORAWAN_CERTIFICATION_JOIN_REQ      = 0x03,
} lorawan_certification_requested_tx_type_t;

typedef enum lorawan_certification_relay_tx_enabled_e
{
    LORAWAN_CERTIFICATION_RELAY_TX_OFF = 0x00,
    LORAWAN_CERTIFICATION_RELAY_TX_ON,
} lorawan_certification_relay_tx_enabled_t;

/**
 * @brief LoRaWAN Certification Object
 *
 * @struct lorawan_certification_s
 *
 */
typedef struct lorawan_certification_s
{
    uint8_t  stack_id;
    uint8_t  task_id;
    bool     enabled;           //!> LoRaWAN Certification is enable or not
    uint16_t rx_app_cnt;        //!> Count each uplink frame
    uint16_t ul_periodicity_s;  //!> Uplink periodicity
    bool     frame_type;        //!> false = Unconfirmed, true = Confirmed
    bool     cw_running;
    uint16_t cw_timeout_s;
    uint32_t cw_frequency;
    int8_t   cw_tx_power;
    bool     beacon_rx_status_ind_ctrl;
    uint16_t rx_beacon_cnt;  //!> Count each new valid beacon frame

    uint8_t                                   tx_buffer_length;
    uint8_t                                   tx_buffer[242];
    uint8_t                                   tx_fport;
    lorawan_certification_requested_tx_type_t is_tx_requested;
    uint8_t                                   cid_req_list_size;
    uint8_t                                   cid_req_list[3];  // cid_from_device_t

} lorawan_certification_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Init a new LoRaWAN certification services object
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 * @return uint8_t
 */
void lorawan_certification_services_init( uint8_t* service_id, uint8_t task_id,
                                          uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                          void ( **on_launch_callback )( void* ),
                                          void ( **on_update_callback )( void* ), void** context_callback );

////////////////////////////////////////////////////////
//////////// Init certification object /////////////////
////////////////////////////////////////////////////////

/**
 * @brief Get the status of certification service
 *
 * @param [in]  stack_id Stack identifier
 * @param [out] enabled  True if enable, False if disable
 * @return lorawan_certification_ret_t
 */
lorawan_certification_ret_t lorawan_certification_get_enabled( uint8_t stack_id, bool* enabled );

/**
 * @brief Enable the certification
 *
 * @param [in]  stack_id Stack identifier
 * @param [in]  enabled  True to enable, False to disable
 * @return lorawan_certification_ret_t
 */
lorawan_certification_ret_t lorawan_certification_set_enabled( uint8_t stack_id, bool enabled );

#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_CERTIFICATION_H

/* --- EOF ------------------------------------------------------------------ */
