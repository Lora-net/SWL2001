/*!
 * @file      lr20xx_radio_wi_sun_types.h
 *
 * @brief     Wi-SUN radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_WI_SUN_TYPES_H
#define LR20XX_RADIO_WI_SUN_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

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
 * @brief Wi-SUN operating modes
 */
typedef enum lr20xx_radio_wi_sun_operating_mode_e
{
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_1A = 0x00,  //!< Wi-SUN operating mode 1A
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_1B = 0x01,  //!< Wi-SUN operating mode 1B
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_2A = 0x02,  //!< Wi-SUN operating mode 2A
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_2B = 0x03,  //!< Wi-SUN operating mode 2B
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_3  = 0x04,  //!< Wi-SUN operating mode 3
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_4A = 0x05,  //!< Wi-SUN operating mode 4A
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_4B = 0x06,  //!< Wi-SUN operating mode 4B
    LR20XX_RADIO_WI_SUN_OPERATING_MODE_5  = 0x07,  //!< Wi-SUN operating mode 5
} lr20xx_radio_wi_sun_operating_mode_t;

/**
 * @brief Wi-SUN frame check sequence (FCS)
 */
typedef enum lr20xx_radio_wi_sun_fcs_e
{
    LR20XX_RADIO_WI_SUN_FCS_32_BIT_CRC = 0x00,  //!< 32-bit CRC as Wi-SUN FCS
    LR20XX_RADIO_WI_SUN_FCS_16_BIT_CRC = 0x01,  //!< 16-bit CRC as Wi-SUN FCS
} lr20xx_radio_wi_sun_fcs_t;

/**
 * @brief Wi-SUN whitening
 */
typedef enum lr20xx_radio_wi_sun_whitening_e
{
    LR20XX_RADIO_WI_SUN_WHITENING_DISABLED = 0x00,  //!< Wi-SUN whitening disabled - test mode
    LR20XX_RADIO_WI_SUN_WHITENING_ENABLED  = 0x01,  //!< Wi-SUN whitening enabled
} lr20xx_radio_wi_sun_whitening_t;

/**
 * @brief Wi-SUN CRC source
 */
typedef enum lr20xx_radio_wi_sun_crc_source_e
{
    LR20XX_RADIO_WI_SUN_CRC_SOURCE_PAYLOAD  = 0x00,  //!< Wi-SUN CRC provided in payload
    LR20XX_RADIO_WI_SUN_CRC_SOURCE_COMPUTED = 0x01,  //!< Wi-SUN CRC computed by the device
} lr20xx_radio_wi_sun_crc_source_t;

/**
 * @brief Wi-SUN payload mode
 */
typedef enum lr20xx_radio_wi_sun_payload_mode_e
{
    LR20XX_RADIO_WI_SUN_PAYLOAD_MODE_NORMAL = 0x00,  //!< Wi-SUN normal payload
    LR20XX_RADIO_WI_SUN_PAYLOAD_MODE_SWITCH = 0x01,  //!< Wi-SUN mode switch payload
} lr20xx_radio_wi_sun_payload_mode_t;

/**
 * @brief Wi-SUN FEC (Forward Error Correction) mode
 */
typedef enum lr20xx_radio_wi_sun_fec_mode_e
{
    LR20XX_RADIO_WI_SUN_FEC_MODE_NONE                   = 0x00,  //!< Wi-SUN FEC mode deactivated
    LR20XX_RADIO_WI_SUN_FEC_MODE_NRNSC_WITH_INTERLEAVER = 0x01,  //!< Wi-SUN FEC mode activated - NRNSC with interleaver
    LR20XX_RADIO_WI_SUN_FEC_MODE_RSC_WITHOUT_INTERLEAVER =
        0x02,                                                  //!< Wi-SUN FEC mode activated - RSC without interleaver
    LR20XX_RADIO_WI_SUN_FEC_MODE_RSC_WITH_INTERLEAVER = 0x03,  //!< Wi-SUN FEC mode activated - RSC with interleaver
} lr20xx_radio_wi_sun_fec_mode_t;

/**
 * @brief Wi-SUN packet parameters
 *
 * To expect a correct preamble detection, the @p rx_preamble_detect_len_in_bits of the receiver has to be lower than or
 * equal to the @p tx_preamble_len_in_bytes of the transmitter.
 */
typedef struct lr20xx_radio_wi_sun_pkt_params_s
{
    lr20xx_radio_wi_sun_fcs_t          fcs;                       //!< Wi-SUN FCS configuration
    lr20xx_radio_wi_sun_whitening_t    whitening;                 //!< Wi-SUN whitening configuration
    lr20xx_radio_wi_sun_crc_source_t   crc_source;                //!< Wi-SUN CRC configuration
    lr20xx_radio_wi_sun_payload_mode_t payload_mode;              //!< Wi-SUN payload mode
    lr20xx_radio_wi_sun_fec_mode_t     fec;                       //!< Wi-SUN FEC mode
    uint16_t                           tx_payload_len;            //!< Wi-SUN payload length for Tx - from 0 to 2047
    uint8_t                            tx_preamble_len_in_bytes;  //!< Wi-SUN preamble length to transmit in bytes
    uint8_t rx_preamble_detect_len_in_bits;  //!< Wi-SUN preamble length detector configuration for reception, in bits.
                                             //!< Possible values are:
                                             //!<   - 0x00: Disable preamble detection, and detect on syncword
                                             //!<   - [0x01:0x20]: Number of preamble bits to detect
                                             //!<   - [0x21:0x7F]: Detect 32 bits of preamble (equivalent to set 0x20)
                                             //!<   - [0x80:0xFF]: Detect 24 bits of preamble (equivalent to set 0x18)
} lr20xx_radio_wi_sun_pkt_params_t;

/**
 * @brief Wi-SUN statistics of received packets
 */
typedef struct lr20xx_radio_wi_sun_rx_stats_s
{
    uint16_t received_packets;  //!< Number of received packets
    uint16_t crc_errors;        //!< Number of received packets with CRC error
    uint16_t length_errors;     //!< Number of received packets with length error
} lr20xx_radio_wi_sun_rx_stats_t;

/**
 * @brief Wi-SUN packet status
 */
typedef struct lr20xx_radio_wi_sun_pkt_status_s
{
    uint16_t packet_length_bytes;       //!< Length of last received packet in bytes
    uint16_t rx_header_raw;             //!< Raw 16-bit header from the last received packet
    uint8_t  syncword_idx;              //!< Index of the syncword detected in the last packet received
    int16_t  rssi_avg_in_dbm;           //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    int16_t  rssi_sync_in_dbm;          //!< RSSI in dBm - value after syncword detection
    uint8_t  rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    uint8_t  link_quality_indicator;    //!< Margin to the detection level, in 0.25dB
} lr20xx_radio_wi_sun_pkt_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_WI_SUN_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
