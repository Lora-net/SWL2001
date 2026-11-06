/*!
 * @file      lr20xx_radio_oqpsk_15_4_types.h
 *
 * @brief     OQPSK 15.4 radio types driver definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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

#ifndef LR20XX_RADIO_OQPSK_15_4_TYPES_H
#define LR20XX_RADIO_OQPSK_15_4_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_radio_fsk_common_types.h"

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
 * @brief OQPSK 15.4 modulation configuration
 *
 */
typedef enum lr20xx_radio_oqpsk_15_4_modulation_e
{
    LR20XX_RADIO_OQPSK_15_4_OQPSK_250KBPS = 0x00,  //!< O-QPSK at 250 kbps
} lr20xx_radio_oqpsk_15_4_modulation_t;

/**
 * @brief OQPSK 15.4 Frame Check Sequence (FCS) configuration
 *
 */
typedef enum lr20xx_radio_oqpsk_15_4_fcs_mode_e
{
    LR20XX_RADIO_OQPSK_15_4_FCS_AUTO =
        0x00,  //!< The FCS is managed by the chip: it is automatically generated in Tx, and
               //!< automatically checked in Rx (it is not available in the FIFO in Rx)
    LR20XX_RADIO_OQPSK_15_4_FCS_MANUAL =
        0x01,  //!< The FCS is not managed by the chip: it must be provided in the FIFO in Tx, and it is not
               //!< automatically checked in Rx (it is available in the FIFO in Rx)
} lr20xx_radio_oqpsk_15_4_fcs_mode_t;

/**
 * @brief OQPSK 15.4 adress filtering enable/disable
 */
typedef enum
{
    LR20XX_RADIO_OQPSK_15_4_ADDRESS_FILTER_DISABLE = 0x00,  //!< Address filtering is disabled
    LR20XX_RADIO_OQPSK_15_4_ADDRESS_FILTER_ENABLE  = 0x01,  //!< Address filtering is enabled
} lr20xx_radio_oqpsk_15_4_address_filtering_configuration_t;

/**
 * @brief OQPSK 15.4 modulation and packet parameters
 *
 */
typedef struct lr20xx_radio_oqpsk_15_4_params_s
{
    lr20xx_radio_oqpsk_15_4_modulation_t modulation;  //!< Modulation configuration
    lr20xx_radio_fsk_common_bw_t         rx_bw;  //!< Receiver bandwidth. It is advised to set a bandwidth value so that
                                                 //!< \f$rx\_bw \ge occupied\_bandwidth + frequency\_error\f$
    uint8_t payload_length;  //!< In reception, this is the maximum accepted payload size. In transmission, this is the
                             //!< total number of bytes to transmit. The complete MPDU must be provided in the FIFO
                             //!< (except FCS if @ref LR20XX_RADIO_OQPSK_15_4_FCS_AUTO is used)
                             //!< Maximum value is 127 bytes.
    uint16_t                           pbl_len_tx_in_bit;  //!< Length in bit of the preamble to send.
    lr20xx_radio_oqpsk_15_4_fcs_mode_t fcs_mode;           //!< Frame Check Sequence mode configuration
    lr20xx_radio_oqpsk_15_4_address_filtering_configuration_t
        address_filtering;  //!< Enable/disable address OQPSK 15.4 filtering
} lr20xx_radio_oqpsk_15_4_params_t;

/**
 * @brief OQPSK 15.4 statistics of received packets
 */
typedef struct lr20xx_radio_oqpsk_15_4_rx_stats_s
{
    uint16_t received_packets;  //!< Number of received packets
    uint16_t crc_errors;        //!< Number of received packets with CRC error
    uint16_t length_errors;     //!< Number of received packets with length error
} lr20xx_radio_oqpsk_15_4_rx_stats_t;

/**
 * @brief OQPSK 15.4 packet status
 */
typedef struct lr20xx_radio_oqpsk_15_4_pkt_status_s
{
    uint16_t packet_length_bytes;       //!< Length of last received packet in bytes
    uint8_t  phr;                       //!< Content of the PHY header: received payload length including FCS
    int16_t  rssi_avg_in_dbm;           //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    int16_t  rssi_sync_in_dbm;          //!< RSSI in dBm - value after syncword detection
    uint8_t  rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    uint8_t  link_quality_indicator;    //!< Margin to the detection level, in 0.25dB
} lr20xx_radio_oqpsk_15_4_pkt_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_OQPSK_15_4_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
