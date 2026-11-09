/*!
 * @file      lr20xx_radio_bluetooth_le_types.h
 *
 * @brief     Bluetooth_LE radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_BLUETOOTH_LE_TYPES_H
#define LR20XX_RADIO_BLUETOOTH_LE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

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
 * @brief Bluetooth_LE modes
 */
typedef enum lr20xx_radio_bluetooth_le_phy_e
{
    LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_1M = 0x00,  //!< Bluetooth_LE PHY LE 1Mb/s
    LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_2M = 0x01,  //!< Bluetooth_LE PHY LE 2Mb/s
    LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB =
        0x02,  //!< Bluetooth_LE PHY LE coded 500kb/s (payload coded with S=2)
    LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB =
        0x03,  //!< Bluetooth_LE PHY LE coded 125kb/s (payload coded with S=8)
} lr20xx_radio_bluetooth_le_phy_t;

/**
 * @brief Bluetooth_LE channel types
 */
typedef enum lr20xx_radio_bluetooth_le_phy_channel_pdu_e
{
    LR20XX_RADIO_BLUETOOTH_LE_PHY_CHANNEL_PDU_ADVERTISING        = 0x00,  //!< Bluetooth_LE advertiser channel type
    LR20XX_RADIO_BLUETOOTH_LE_PHY_CHANNEL_PDU_DATA_16_BIT_HEADER = 0x01,  //!< Bluetooth_LE data16 channel type
    LR20XX_RADIO_BLUETOOTH_LE_PHY_CHANNEL_PDU_DATA_24_BIT_HEADER = 0x02,  //!< Bluetooth_LE data24 channel type
} lr20xx_radio_bluetooth_le_phy_channel_pdu_t;

/**
 * @brief Bluetooth_LE packet parameters
 */
typedef struct lr20xx_radio_bluetooth_le_pkt_params_s
{
    lr20xx_radio_bluetooth_le_phy_channel_pdu_t phy_channel_pdu;  //!< Bluetooth_LE physical channel PDU
    uint8_t whitening_init;  //!< Bluetooth_LE whitening initialization value (for instance 0x53 for channel 37).
    //!< Setting to 0 disable whitening
    bool     is_crc_in_fifo;  //!< Indicate wether the CRC field is available in FIFO or not
    uint32_t crc_init;        //!< Bluetooth_LE CRC initialization value (0x555555 for advertising)
    uint32_t sync_word;       //!< Bluetooth_LE syncword (0x8e89bed6 for advertising)
} lr20xx_radio_bluetooth_le_pkt_params_t;

/**
 * @brief Bluetooth_LE statistics of received packets
 */
typedef struct lr20xx_radio_bluetooth_le_rx_stats_s
{
    uint16_t received_packets;  //!< Number of received packets
    uint16_t crc_errors;        //!< Number of received packets with CRC error
    uint16_t length_errors;     //!< Number of received packets with length error
} lr20xx_radio_bluetooth_le_rx_stats_t;

/**
 * @brief Bluetooth_LE packet status
 */
typedef struct lr20xx_radio_bluetooth_le_pkt_status_s
{
    uint16_t packet_length_bytes;       //!< Length of last received packet in bytes
    int16_t  rssi_avg_in_dbm;           //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    int16_t  rssi_sync_in_dbm;          //!< RSSI in dBm - value after syncword detection
    uint8_t  rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    uint8_t  link_quality_indicator;    //!< Margin to the detection level, in 0.25dB
} lr20xx_radio_bluetooth_le_pkt_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_BLUETOOTH_LE_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
