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
#include "ral_defs.h"

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
typedef enum smtc_modem_test_mode_sync_word_e
{
    SYNC_WORD_0x12 = 0x12,
    SYNC_WORD_0x21 = 0x21,
    SYNC_WORD_0x34 = 0x34,
    SYNC_WORD_0x56 = 0x56,
} smtc_modem_test_mode_sync_word_t;

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
 * @param [in] reset_radio  Reset radio after NOP
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_nop( bool reset_radio );

/**
 * @brief  Test mode TX LoRa single or continue
 *
 * @param [in] payload*        Payload that will be sent. If NULL a randomly generated payload_length msg will be sent
 * @param [in] payload_length  Length of the payload
 * @param [in] frequency_hz    Frequency in Hz
 * @param [in] tx_power_dbm    Power in dbm
 * @param [in] sf              Spreading factor following ral_lora_sf_t definition
 * @param [in] bw              Bandwidth following ral_lora_bw_t definition
 * @param [in] cr              Coding rate following ral_lora_cr_t definition
 * @param [in] sync_word       sync_word
 * @param [in] invert_iq       invert iq parameter
 * @param [in] crc_is_on       include crc boolean
 * @param [in] header_type     header type (explicit/implicit)
 * @param [in] preamble_size   Size of the preamble
 * @param [in] nb_of_tx        nb of transmissions (0 means tx continuous)
 * @param [in] delay_ms        delay between two  transmissions
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */

smtc_modem_return_code_t smtc_modem_test_tx_lora( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                                  int8_t tx_power_dbm, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                                  ral_lora_cr_t cr, smtc_modem_test_mode_sync_word_t sync_word,
                                                  bool invert_iq, bool crc_is_on, ral_lora_pkt_len_modes_t header_type,
                                                  uint32_t preamble_size, uint32_t nb_of_tx, uint32_t delay_ms );

/**
 * @brief  Test mode TX FSK single or continue
 *
 * @param [in] payload*        Payload that will be sent. If NULL a randomly generated payload_length msg will be
 * sent
 * @param [in] payload_length  Length of the payload
 * @param [in] frequency_hz    Frequency in Hz
 * @param [in] tx_power_dbm    Power in dbm
 * @param [in] nb_of_tx        nb of transmissions (0 means tx continuous)
 * @param [in] delay_ms        delay between two  transmissions
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_tx_fsk( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                                 int8_t tx_power_dbm, uint32_t nb_of_tx, uint32_t delay_ms );

/**
 * @brief  Test mode TX LR_FHSS single or continue
 *
 * @param [in] payload*        Payload that will be sent. If NULL a randomly generated payload_length msg will be sent
 * @param [in] payload_length  Length of the payload
 * @param [in] frequency_hz    Frequency in Hz
 * @param [in] tx_power_dbm    Power in dbm
 * @param [in] cr              Coding rate following lr_fhss_v1_cr_t definition
 * @param [in] bw              Bandwidth following lr_fhss_v1_bw_t definition
 * @param [in] grid            Grid following lr_fhss_v1_grid_t definition
 * @param [in] enable_hopping  Enable channel hopping
 * @param [in] nb_of_tx        nb of transmissions (0 means tx continuous)
 * @param [in] delay_ms        delay between two  transmissions
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */

smtc_modem_return_code_t smtc_modem_test_tx_lrfhss( uint8_t* payload, uint8_t payload_length, uint32_t frequency_hz,
                                                    int8_t tx_power_dbm, lr_fhss_v1_cr_t tx_cr, lr_fhss_v1_bw_t tx_bw,
                                                    lr_fhss_v1_grid_t tx_grid, bool enable_hopping, uint32_t nb_of_tx,
                                                    uint32_t delay_ms );

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
 * @brief Test mode RX LoRa continue
 * @remark Continuously receive packets.
 *
 * @param [in] frequency_hz    Frequency in Hz
 * @param [in] sf              Spreading factor following ral_lora_sf_t definition
 * @param [in] bw              Bandwidth following ral_lora_bw_t definition
 * @param [in] cr              Coding rate following ral_lora_cr_t definition
 * @param [in] sync_word       sync_word
 * @param [in] invert_iq       invert iq parameter
 * @param [in] crc_is_on       include crc boolean
 * @param [in] header_type     header type (explicit/implicit)
 * @param [in] preamble_size   Size of the preamble
 * @param [in] symb_nb_timeout Number of symbols before timeout (0 means no timeout)
 * 
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */

smtc_modem_return_code_t smtc_modem_test_rx_lora( uint32_t frequency_hz, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                                             ral_lora_cr_t                    cr,
                                                             smtc_modem_test_mode_sync_word_t sync_word, bool invert_iq,
                                                             bool crc_is_on, ral_lora_pkt_len_modes_t header_type,
                                                             uint32_t preamble_size, uint8_t symb_nb_timeout);

/**
 * @brief Test mode RX FSK continue
 * @remark Continuously receive packets.
 *
 * @param [in] frequency_hz  Frequency in Hz
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_rx_fsk_continuous( uint32_t frequency_hz );

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
 * @brief Read last received packet during test RX continue
 * @remark
 * @param [out] rssi*            RSSI in dBm
 * @param [out] snr*             SNR in dB
 * @param [out] rx_payload*      Pointer to the buffer to store the received payload
 * @param [out] rx_payload_length*  Length of the received payload
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */
smtc_modem_return_code_t smtc_modem_test_get_last_rx_packets( int16_t* rssi, int16_t* snr, uint8_t* rx_payload,
                                                              uint8_t* rx_payload_length );

/**
 * @brief Test mode RSSI LBT
 * @remark Measure continuously the RSSI during a chosen time and give an average value
 *
 * @param [in] frequency_hz  Frequency in Hz
 * @param [in] bw            bandwidth in Hz
 * @param [in] time_ms       test duration in ms (1 rssi every 10 ms)

 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 */

smtc_modem_return_code_t smtc_modem_test_rssi_lbt( uint32_t frequency_hz, uint32_t bw_hz, uint16_t time_ms );

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
