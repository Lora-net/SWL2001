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
#define LORAWAN_CERTIFICATION_DUT_REST_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_DUT_JOIN_REQ_SIZE 1
#define LORAWAN_CERTIFICATION_SWITCH_CLASS_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_ADR_BIT_CHANGE_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_REGIONAL_DUTY_CYCLE_CTRL_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_TX_PERIODICITY_CHANGE_REQ_SIZE 2
#define LORAWAN_CERTIFICATION_TX_FRAME_CTRL_REQ_SIZE 242  // TODO Must be checked
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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum lorawan_certification_parser_ret_e
{
    LORAWAN_CERTIFICATION_RET_NOTHING      = 0,
    LORAWAN_CERTIFICATION_RET_APP_UL       = 1,
    LORAWAN_CERTIFICATION_RET_CERTIF_UL    = 2,
    LORAWAN_CERTIFICATION_RET_LINK_CHECK   = 3,
    LORAWAN_CERTIFICATION_RET_DEVICE_TIME  = 4,
    LORAWAN_CERTIFICATION_RET_PING_SLOT    = 5,
    LORAWAN_CERTIFICATION_RET_TX_CW        = 6,
    LORAWAN_CERTIFICATION_RET_SWITCH_CLASS = 7,
} lorawan_certification_parser_ret_t;

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
    LORAWAN_CERTIFICATION_BEACON_CNT_ANS       = 0x42,
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
    LORAWAN_CERTIFICATION_DUT_REST_REQ                 = 0x01,
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
    LORAWAN_CERTIFICATION_BEACON_CNT_RST_REQ           = 0x43,
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
    LORAWAN_CERTIFICATION_PERIODICITY_1,
    LORAWAN_CERTIFICATION_PERIODICITY_2,
    LORAWAN_CERTIFICATION_PERIODICITY_3,
    LORAWAN_CERTIFICATION_PERIODICITY_4,
    LORAWAN_CERTIFICATION_PERIODICITY_5,
    LORAWAN_CERTIFICATION_PERIODICITY_6,
    LORAWAN_CERTIFICATION_PERIODICITY_7,
    LORAWAN_CERTIFICATION_PERIODICITY_8,
    LORAWAN_CERTIFICATION_PERIODICITY_9,
    LORAWAN_CERTIFICATION_PERIODICITY_10,
    LORAWAN_CERTIFICATION_PERIODICITY_MAX,
} lorawan_certification_periodicity_t;

/**
 * @brief LoRaWAN Certification Periodicity Table converter
 *
 */
static const uint16_t lorawan_certification_periodicity_table[LORAWAN_CERTIFICATION_PERIODICITY_MAX] = {
    [LORAWAN_CERTIFICATION_PERIODICITY_0] = 0,    [LORAWAN_CERTIFICATION_PERIODICITY_1] = 5,
    [LORAWAN_CERTIFICATION_PERIODICITY_2] = 10,   [LORAWAN_CERTIFICATION_PERIODICITY_3] = 20,
    [LORAWAN_CERTIFICATION_PERIODICITY_4] = 30,   [LORAWAN_CERTIFICATION_PERIODICITY_5] = 40,
    [LORAWAN_CERTIFICATION_PERIODICITY_6] = 50,   [LORAWAN_CERTIFICATION_PERIODICITY_7] = 60,
    [LORAWAN_CERTIFICATION_PERIODICITY_8] = 120,  [LORAWAN_CERTIFICATION_PERIODICITY_9] = 240,
    [LORAWAN_CERTIFICATION_PERIODICITY_10] = 480,
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

/**
 * @brief LoRaWAN Certification Object
 *
 * @struct lorawan_certification_s
 *
 */
typedef struct lorawan_certification_s
{
    bool                          enabled;         //!> LoRaWAN Certification is enable or not
    uint16_t                      rx_app_cnt;      //!> Count each uplink frame
    uint8_t                       ul_periodicity;  //!> Uplink periodicity
    bool                          frame_type;
    bool                          cw_running;
    uint16_t                      cw_timeout_s;
    uint32_t                      cw_frequency;
    int8_t                        cw_tx_power;
    bool                          beacon_rx_status_ind_ctrl;
    uint16_t                      rx_beacon_cnt;  //!> Count each new valid beacon frame
    lorawan_certification_class_t class_requested;

} lorawan_certification_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize the LoRaWAN Certification Package
 *
 * @param lorawan_certification
 */
void lorawan_certification_init( lorawan_certification_t* lorawan_certification );

/**
 * @brief
 *
 * @param lorawan_certification
 * @param enabled
 */
void lorawan_certification_set_enabled( lorawan_certification_t* lorawan_certification, bool enabled );

/**
 * @brief
 *
 * @param lorawan_certification
 * @return true
 * @return false
 */
bool lorawan_certification_get_enabled( lorawan_certification_t* lorawan_certification );

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t lorawan_certification_get_ul_periodicity( lorawan_certification_t* lorawan_certification );

/**
 * @brief
 *
 * @param lorawan_certification
 * @return true
 * @return false
 */
bool lorawan_certification_get_frame_type( lorawan_certification_t* lorawan_certification );

/**
 * @brief Return CW configuration requested by the testing tool
 *
 * @param [in] lorawan_certification
 * @param [out] timeout_s
 * @param [out] frequency
 * @param [out] tx_power
 */
void lorawan_certification_get_cw_config( lorawan_certification_t* lorawan_certification, uint16_t* timeout_s,
                                          uint32_t* frequency, int8_t* tx_power );

/**
 * @brief Return if the CW was requested by the TCL
 *
 * @param lorawan_certification
 * @return true
 * @return false
 */
bool lorawan_certification_is_cw_running( lorawan_certification_t* lorawan_certification );

/**
 * @brief Set CW as stopped
 *
 * @param lorawan_certification
 */
void lorawan_certification_cw_set_as_stopped( lorawan_certification_t* lorawan_certification );

/**
 * @brief Get the status of beacon rx status indication control
 *
 * @param lorawan_certification
 * @return true
 * @return false
 */
bool lorawan_certification_get_beacon_rx_status_ind_ctrl( lorawan_certification_t* lorawan_certification );

/**
 * @brief Return the class requested by the Testing tool
 *
 * @param lorawan_certification
 * @return lorawan_certification_class_t
 */
lorawan_certification_class_t lorawan_certification_get_requested_class(
    lorawan_certification_t* lorawan_certification );

/**
 * @brief
 *
 * @param lorawan_certification
 * @param rx_buffer
 * @param rx_buffer_length
 * @param tx_buffer
 * @param tx_buffer_length
 * @param tx_fport
 * @return lorawan_certification_parser_ret_t
 */
lorawan_certification_parser_ret_t lorawan_certification_parser( lorawan_certification_t* lorawan_certification,
                                                                 uint8_t* rx_buffer, uint8_t rx_buffer_length,
                                                                 uint8_t* tx_buffer, uint8_t* tx_buffer_length,
                                                                 uint8_t* tx_fport );
/**
 * @brief Build Class B Beacon Status Indication frame
 *
 * @param lorawan_certification
 * @param beacon_buffer
 * @param beacon_buffer_length
 * @param tx_buffer
 * @param tx_buffer_length
 * @param rssi
 * @param snr
 * @param beacon_dr
 * @param beacon_freq
 */
void lorawan_certification_build_beacon_rx_status_ind( lorawan_certification_t* lorawan_certification,
                                                       uint8_t* beacon_buffer, uint8_t beacon_buffer_length,
                                                       uint8_t* tx_buffer, uint8_t* tx_buffer_length, int8_t rssi,
                                                       int8_t snr, uint8_t beacon_dr, uint32_t beacon_freq );
#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_CERTIFICATION_H

/* --- EOF ------------------------------------------------------------------ */
