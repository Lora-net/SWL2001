/*!
 * @file      lr20xx_radio_flrc_types.h
 *
 * @brief     FLRC radio types driver definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#ifndef LR20XX_RADIO_FLRC_TYPES_H
#define LR20XX_RADIO_FLRC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Length in bytes of the FLRC syncword
 *
 */
#define LR20XX_RADIO_FLRC_SYNCWORD_LENGTH ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Combinations of bitrate and bandwidth for FLRC packet type
 *
 * @remark The bitrate is in Mb/s and the bandwidth in MHz (DSB)
 */
typedef enum lr20xx_radio_flrc_br_bw_e
{
    LR20XX_RADIO_FLRC_BR_2_600_BW_2_666 = 0x00,  //!< 2.600 Mb/s, 2.666 MHz
    LR20XX_RADIO_FLRC_BR_2_080_BW_2_222 = 0x01,  //!< 2.080 Mb/s, 2.222 MHz
    LR20XX_RADIO_FLRC_BR_1_300_BW_1_333 = 0x02,  //!< 1.300 Mb/s, 1.333 MHz
    LR20XX_RADIO_FLRC_BR_1_040_BW_1_333 = 0x03,  //!< 1.040 Mb/s, 1.333 MHz
    LR20XX_RADIO_FLRC_BR_0_650_BW_0_740 = 0x04,  //!< 0.650 Mb/s, 0.740 MHz
    LR20XX_RADIO_FLRC_BR_0_520_BW_0_571 = 0x05,  //!< 0.520 Mb/s, 0.571 MHz
    LR20XX_RADIO_FLRC_BR_0_325_BW_0_357 = 0x06,  //!< 0.325 Mb/s, 0.357 MHz
    LR20XX_RADIO_FLRC_BR_0_260_BW_0_307 = 0x07,  //!< 0.260 Mb/s, 0.307 MHz
} lr20xx_radio_flrc_br_bw_t;

/**
 * @brief Coding rates for FLRC packet type
 */
typedef enum lr20xx_radio_flrc_cr_e
{
    LR20XX_RADIO_FLRC_CR_1_2  = 0x00,  //!< Coding rate 1/2
    LR20XX_RADIO_FLRC_CR_3_4  = 0x01,  //!< Coding rate 3/4
    LR20XX_RADIO_FLRC_CR_NONE = 0x02,  //!< Coding rate 1 (no FEC)
    LR20XX_RADIO_FLRC_CR_2_3  = 0x03,  //!< Coding rate 2/3
} lr20xx_radio_flrc_cr_t;

/**
 * @brief Modulation shapings for FLRC packet type
 */
typedef enum lr20xx_radio_flrc_pulse_shape_e
{
    LR20XX_RADIO_FLRC_PULSE_SHAPE_OFF   = 0x00,
    LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_05 = 0x05,
    LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_07 = 0x06,
    LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_1  = 0x07,
} lr20xx_radio_flrc_pulse_shape_t;

/**
 * @brief Preamble lengths for FLRC packet type
 */
typedef enum lr20xx_radio_flrc_preamble_len_e
{
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_04_BITS = 0x00,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_08_BITS = 0x01,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_12_BITS = 0x02,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_16_BITS = 0x03,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_20_BITS = 0x04,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_24_BITS = 0x05,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_28_BITS = 0x06,
    LR20XX_RADIO_FLRC_PREAMBLE_LEN_32_BITS = 0x07,
} lr20xx_radio_flrc_preamble_len_t;

/**
 * @brief Combinations of SyncWord correlators activated for FLRC packet types
 *
 * @remark Each syncword (1,2 or 3) is configured thanks to @ref lr20xx_radio_flrc_set_syncword
 */
typedef enum lr20xx_radio_flrc_rx_match_sync_word_e
{
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_OFF         = 0x00,  //!< No syncword match
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1           = 0x01,  //!< Match syncword #1
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_2           = 0x02,  //!< Match syncword #2
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1_OR_2      = 0x03,  //!< Match syncword #1 or #2
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_3           = 0x04,  //!< Match syncword #3
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1_OR_3      = 0x05,  //!< Match syncword #1 or #3
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_2_OR_3      = 0x06,  //!< Match syncword #2 or #3
    LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1_OR_2_OR_3 = 0x07,  //!< Match syncword #1 or #2 or #3
} lr20xx_radio_flrc_rx_match_sync_word_t;

/**
 * @brief Syncword lengths for FLRC packet type
 */
typedef enum lr20xx_radio_flrc_sync_word_len_e
{
    LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_OFF     = 0x00,
    LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_2_BYTES = 0x01,
    LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_4_BYTES = 0x02,
} lr20xx_radio_flrc_sync_word_len_t;

/**
 * @brief Configuration of syncwords for Tx operations
 *
 */
typedef enum lr20xx_radio_flrc_tx_syncword_e
{
    LR20XX_RADIO_FLRC_TX_SYNCWORD_NONE = 0x00,  //!< Do not use syncword for Tx operation
    LR20XX_RADIO_FLRC_TX_SYNCWORD_1    = 0x01,  //!< Use syncword 1
    LR20XX_RADIO_FLRC_TX_SYNCWORD_2    = 0x02,  //!< Use syncword 2
    LR20XX_RADIO_FLRC_TX_SYNCWORD_3    = 0x03,  //!< Use syncword 3
} lr20xx_radio_flrc_tx_syncword_t;

/**
 * @brief Packet length mode for FLRC packet type
 */
typedef enum lr20xx_radio_flrc_pkt_len_modes_e
{
    LR20XX_RADIO_FLRC_PKT_VAR_LEN = 0x00,
    LR20XX_RADIO_FLRC_PKT_FIX_LEN = 0x01,
} lr20xx_radio_flrc_pkt_len_modes_t;

/**
 * @brief CRC lengths for FLRC packet type
 */
typedef enum lr20xx_radio_flrc_crc_types_e
{
    LR20XX_RADIO_FLRC_CRC_OFF     = 0x00,
    LR20XX_RADIO_FLRC_CRC_2_BYTES = 0x01,
    LR20XX_RADIO_FLRC_CRC_3_BYTES = 0x02,
    LR20XX_RADIO_FLRC_CRC_4_BYTES = 0x03,
} lr20xx_radio_flrc_crc_types_t;

/**
 * @brief Modulation configuration for LoRa packet
 *
 */
typedef struct lr20xx_radio_flrc_mod_params_s
{
    lr20xx_radio_flrc_br_bw_t       br_bw;  //!< Bitrate & bandwidth
    lr20xx_radio_flrc_cr_t          cr;     //!< Coding rate
    lr20xx_radio_flrc_pulse_shape_t shape;  //!< Shaping
} lr20xx_radio_flrc_mod_params_t;

/**
 * @brief Packet parameters for FLRC packet type
 */
typedef struct lr20xx_radio_flrc_pkt_params_s
{
    lr20xx_radio_flrc_preamble_len_t       preamble_len;     //!< FLRC preamble length
    lr20xx_radio_flrc_sync_word_len_t      sync_word_len;    //!< FLRC syncword length
    lr20xx_radio_flrc_tx_syncword_t        tx_syncword;      //!< FLRC syncword to use for Tx operation
    lr20xx_radio_flrc_rx_match_sync_word_t match_sync_word;  //!< FLRC syncword matcher
    lr20xx_radio_flrc_pkt_len_modes_t      header_type;      //!< FLRC header type
    uint16_t pld_len_in_bytes;  //!< FLRC payload length in byte - in [6:511] (note that for SX1280 compatibility, range
                                //!< is [6:127]). If a length error is detected while
                                //!< lr20xx_radio_flrc_pkt_params_t.header_type == LR20XX_RADIO_FLRC_PKT_VAR_LEN, the
                                //!< IRQ LR20XX_SYSTEM_IRQ_LEN_ERROR is raised, but the device remains in Rx mode.
    lr20xx_radio_flrc_crc_types_t crc_type;  //!< FLRC CRC type configuration
} lr20xx_radio_flrc_pkt_params_t;

/**
 * @brief FLRC statistics of received packets
 */
typedef struct lr20xx_radio_flrc_rx_stats_s
{
    uint16_t received_packets;  //!< Number of received packets
    uint16_t crc_errors;        //!< Number of received packets with CRC error
    uint16_t length_errors;     //!< Number of received packets with length error
} lr20xx_radio_flrc_rx_stats_t;

/**
 * @brief Packet status parameters for FLRC packet types
 */
typedef struct lr20xx_radio_flrc_pkt_status_s
{
    uint16_t packet_length_bytes;       //!< Length of last received packet in bytes
    uint8_t  syncword_index;            //!< Syncword index of the last received packet - in [1, 2, 3]
    int16_t  rssi_avg_in_dbm;           //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_avg_in_dbm value in dBm
    int16_t  rssi_sync_in_dbm;          //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_sync_in_dbm value in dBm
} lr20xx_radio_flrc_pkt_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FLRC_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
