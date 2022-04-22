/**
 * @file      smtc_modem_test_api.h
 *
 * @brief     soft modem test API description
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

#ifndef SMTC_MODEM_TEST_API_H__
#define SMTC_MODEM_TEST_API_H__

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
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Test mode Spreading Factor type
 */
typedef enum smtc_modem_test_sf_e
{
    SMTC_MODEM_TEST_FSK = 0,        //!< FSK
    SMTC_MODEM_TEST_LORA_SF5,       //!< SF5
    SMTC_MODEM_TEST_LORA_SF6,       //!< SF6
    SMTC_MODEM_TEST_LORA_SF7,       //!< SF7
    SMTC_MODEM_TEST_LORA_SF8,       //!< SF8
    SMTC_MODEM_TEST_LORA_SF9,       //!< SF9
    SMTC_MODEM_TEST_LORA_SF10,      //!< SF10
    SMTC_MODEM_TEST_LORA_SF11,      //!< SF11
    SMTC_MODEM_TEST_LORA_SF12,      //!< SF12
    SMTC_MODEM_TEST_LORA_SF_COUNT,  //!< Count
} smtc_modem_test_sf_t;

/**
 * @brief Test mode Bandwith type
 */
typedef enum smtc_modem_test_bw_e
{
    SMTC_MODEM_TEST_BW_125_KHZ,   //!< BW125
    SMTC_MODEM_TEST_BW_250_KHZ,   //!< BW250
    SMTC_MODEM_TEST_BW_500_KHZ,   //!< BW500
    SMTC_MODEM_TEST_BW_200_KHZ,   //!< BW200
    SMTC_MODEM_TEST_BW_400_KHZ,   //!< BW400
    SMTC_MODEM_TEST_BW_800_KHZ,   //!< BW800
    SMTC_MODEM_TEST_BW_1600_KHZ,  //!< BW1600
    SMTC_MODEM_TEST_BW_COUNT,     //!< Count
} smtc_modem_test_bw_t;

/**
 * @brief Test mode Bandwith type (high bandwidth)
 */
typedef enum smtc_modem_test_bw_m_e
{
    SMTC_MODEM_TEST_BW_12M = 15,
    SMTC_MODEM_TEST_BW_18M,
    SMTC_MODEM_TEST_BW_24M,
} smtc_modem_test_bw_m_t;

/**
 * @brief Test mode Coding Rate type
 */
typedef enum smtc_modem_test_cr_e
{
    SMTC_MODEM_TEST_CR_4_5 = 0,  //!< CR 4/5
    SMTC_MODEM_TEST_CR_4_6,      //!< CR 4/6
    SMTC_MODEM_TEST_CR_4_7,      //!< CR 4/7
    SMTC_MODEM_TEST_CR_4_8,      //!< CR 4/8
    SMTC_MODEM_TEST_CR_LI_4_5,   //!< CR 4/5 long interleaved
    SMTC_MODEM_TEST_CR_LI_4_6,   //!< CR 4/6 long interleaved
    SMTC_MODEM_TEST_CR_LI_4_8,   //!< CR 4/8 long interleaved
    SMTC_MODEM_TEST_CR_COUNT,    //!< Count
} smtc_modem_test_cr_t;

/* clang-format on */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Put modem in test mode
 * @remark No other modem commands can be handled during modem test mode
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_start( void );

/**
 * @brief Exit modem test mode
 * @remark Exit test mode and perform a reset of modem
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_stop( void );

/**
 * @brief Perform no operation. This function can be used to terminate an ongoing continuous operation
 * @remark Abort the radio planner task
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_nop( void );

/**
 * @brief Test mode TX single or continue
 * @remark Transmit a single packet or continuously transmit packets as fast as possible.
 *
 * @param [in] payload*        Payload that will be sent. If NULL a randomly generated payload_length msg will be sent
 * @param [in] payload_length  Length of the payload
 * @param [in] frequency_hz    Frequency in Hz
 * @param [in] tx_power_dbm    Power in dbm
 * @param [in] sf              Spreading factor following smtc_modem_test_sf_t definition
 * @param [in] bw              Bandwith following smtc_modem_test_bw_t definition
 * @param [in] cr              Coding rate following smtc_modem_test_cr_t definition
 * @param [in] preamble_size   Size of the preamble
 * @param [in] continuous_tx   false: single transmission / true: continuous transmission
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_tx( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                             int8_t tx_power_dbm, smtc_modem_test_sf_t sf, smtc_modem_test_bw_t bw,
                                             smtc_modem_test_cr_t cr, uint32_t preamble_size, bool continuous_tx );

/**
 * @brief Test mode transmit a continuous wave.
 * @remark
 *
 * @param [in] frequency_hz  Frequency in Hz
 * @param [in] tx_power_dbm  Power in dbm
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_tx_cw( uint32_t frequency_hz, int8_t tx_power_dbm );

/**
 * @brief Test mode RX continue
 * @remark Continuously receive packets.
 *
 * @param [in] frequency_hz  Frequency in Hz
 * @param [in] sf            Spreading factor following smtc_modem_test_sf_t definition
 * @param [in] bw            Bandwith following smtc_modem_test_bw_t definition
 * @param [in] cr            Coding rate following smtc_modem_test_cr_t definition
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_rx_continuous( uint32_t frequency_hz, smtc_modem_test_sf_t sf,
                                                        smtc_modem_test_bw_t bw, smtc_modem_test_cr_t cr );

/**
 * @brief Read number of received packets during test RX continue
 * @remark
 *
 * @param [out] nb_rx_packets*  Number of received packet in Rx Continue
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_get_nb_rx_packets( uint32_t* nb_rx_packets );

/**
 * @brief Test mode RSSI
 * @remark Measure continuously the RSSI during a chosen time and give an average value
 *
 * @param [in] frequency_hz  Frequency in Hz
 * @param [in] bw            bandwidth following smtc_modem_test_bw_t definition
 * @param [in] time_ms       test duration in ms (1 rssi every 10 ms)

 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_rssi( uint32_t frequency_hz, smtc_modem_test_bw_t bw, uint16_t time_ms );

/**
 * @brief Get RSSI result (to be called when test rssi is finished)
 * @remark Returns the computed RSSI.
 *
 * @param [out]    rssi*  rssi + 64
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_get_rssi( int8_t* rssi );

/**
 * @brief Reset the Radio for test purpose
 * @remark
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_radio_reset( void );

/**
 * @brief Enable / disable the applicative dutycycle
 * @remark
 *
 * @param [in] status  0: disable, 1: enable
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t

 */
smtc_modem_return_code_t smtc_modem_test_duty_cycle_app_activate( bool status );

/**
 * @brief Direct access to radio command write
 *
 * @param [in] command         Pointer to the buffer to be transmitted
 * @param [in] command_length  Buffer size to be transmitted
 * @param [in] data            Pointer to the buffer to be transmitted
 * @param [in] data_length     Buffer size to be transmitted
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_direct_radio_write( uint8_t* command, uint16_t command_length, uint8_t* data,
                                                             uint16_t data_length );

/**
 * @brief Direct access to radio command read
 *
 * @param [in] command         Pointer to the buffer to be transmitted
 * @param [in] command_length  Buffer size to be transmitted
 * @param [out] data           Pointer to the buffer to be received
 * @param [in] data_length     Buffer size to be received
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_direct_radio_read( uint8_t* command, uint16_t command_length, uint8_t* data,
                                                            uint16_t data_length );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_TEST_API_H__

/* --- EOF ------------------------------------------------------------------ */
