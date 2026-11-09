/*!
 * @file      lr20xx_radio_wm_bus_types.h
 *
 * @brief     Wireless M-Bus radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_WM_BUS_TYPES_H
#define LR20XX_RADIO_WM_BUS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lr20xx_radio_fsk_common_types.h"

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
 * @brief Wireless M-Bus modes
 */
typedef enum lr20xx_radio_wm_bus_mode_e
{
    LR20XX_RADIO_WM_BUS_MODE_S           = 0x00,  //!< Wireless M-Bus mode S
    LR20XX_RADIO_WM_BUS_MODE_T1          = 0x01,  //!< Wireless M-Bus mode T1
    LR20XX_RADIO_WM_BUS_MODE_T2_O2M      = 0x02,  //!< Wireless M-Bus mode T2 O-2-M
    LR20XX_RADIO_WM_BUS_MODE_T2_M2O      = 0x03,  //!< Wireless M-Bus mode T2 M-2-O
    LR20XX_RADIO_WM_BUS_MODE_R2          = 0x04,  //!< Wireless M-Bus mode R2
    LR20XX_RADIO_WM_BUS_MODE_C1          = 0x05,  //!< Wireless M-Bus mode C1
    LR20XX_RADIO_WM_BUS_MODE_C2_O2M      = 0x06,  //!< Wireless M-Bus mode C2 O-2-M
    LR20XX_RADIO_WM_BUS_MODE_C2_M2O      = 0x07,  //!< Wireless M-Bus mode C2 M-2-O
    LR20XX_RADIO_WM_BUS_MODE_N_4_8_KBPS  = 0x08,  //!< Wireless M-Bus mode N 4.8kbps
    LR20XX_RADIO_WM_BUS_MODE_N_2_4_KBPS  = 0x09,  //!< Wireless M-Bus mode N 2.4kbps
    LR20XX_RADIO_WM_BUS_MODE_N_6_4_KBPS  = 0x0A,  //!< Wireless M-Bus mode N 6.4kbps
    LR20XX_RADIO_WM_BUS_MODE_N_19_2_KBPS = 0x0B,  //!< Wireless M-Bus mode N 19.2kbps
    LR20XX_RADIO_WM_BUS_MODE_F2          = 0x0C,  //!< Wireless M-Bus mode F2
} lr20xx_radio_wm_bus_mode_t;

/**
 * @brief Wireless M-Bus modes
 */
typedef enum lr20xx_radio_wm_bus_pkt_format_e
{
    LR20XX_RADIO_WM_BUS_PKT_FORMAT_A = 0x00,  //!< Wireless M-Bus packet format A
    LR20XX_RADIO_WM_BUS_PKT_FORMAT_B = 0x01,  //!< Wireless M-Bus packet format B
} lr20xx_radio_wm_bus_pkt_format_t;

/**
 * @brief Wireless M-Bus address filtering configuration
 */
typedef enum lr20xx_radio_wm_bus_addr_filtering_e
{
    LR20XX_RADIO_WM_BUS_ADDR_FILTERING_DISABLED = 0x00,  //!< Wireless M-Bus address filtering disabled
    LR20XX_RADIO_WM_BUS_ADDR_FILTERING_ENABLED  = 0x01,  //!< Wireless M-Bus address filtering enabled on reception
} lr20xx_radio_wm_bus_addr_filtering_t;

/**
 * @brief Wireless M-Bus modes
 */
typedef struct lr20xx_radio_wm_bus_params_s
{
    lr20xx_radio_wm_bus_mode_t   mode;   //!< Wireless M-Bus mode according to EN13757-4 2019
    lr20xx_radio_fsk_common_bw_t rx_bw;  //!< Wireless M-Bus reception bandwidth. It is strongly recommended to use the
                                         //!< value @ref LR20XX_RADIO_FSK_COMMON_RX_BW_AUTO
    lr20xx_radio_wm_bus_pkt_format_t     pkt_format;      //!< Wireless M-Bus packet format
    lr20xx_radio_wm_bus_addr_filtering_t addr_filtering;  //!< Wireless M-Bus address filtering configuration
    uint8_t                              payload_length;  //!< Wireless M-Bus application payload length in byte
    uint16_t                             preamble_len;    //!< Wireless M-Bus preamble length in bit
    uint8_t preamble_len_detect;  //!< Wireless M-Bus preamble detection length in bit. It is strongly recommended to
                                  //!< use the value 0xFF for automatic setting based on preamble_length value
} lr20xx_radio_wm_bus_params_t;

/**
 * @brief Wireless M-Bus statistics of received packets
 */
typedef struct lr20xx_radio_wm_bus_rx_stats_s
{
    uint16_t received_packets;  //!< Number of received packets
    uint16_t crc_errors;        //!< Number of received packets with CRC error
    uint16_t length_errors;     //!< Number of received packets with length error
} lr20xx_radio_wm_bus_rx_stats_t;

/**
 * @brief Wireless M-Bus packet status
 */
typedef struct lr20xx_radio_wm_bus_pkt_status_s
{
    uint16_t                         packet_length_bytes;  //!< Length of last received packet in bytes
    uint8_t                          l_field;              //!< Demodulated length field
    uint32_t                         crc_errors;           //!< Bitfield giving the position of CRC errors
    lr20xx_radio_wm_bus_pkt_format_t pkt_format;           //!< Format of the received packet
    int16_t                          rssi_avg_in_dbm;      //!< RSSI in dBm - averaged over the last received packet
    uint8_t rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    int16_t rssi_sync_in_dbm;          //!< RSSI in dBm - value after syncword detection
    uint8_t rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    uint8_t link_quality_indicator;    //!< Margin to the detection level, in 0.25dB
} lr20xx_radio_wm_bus_pkt_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_WM_BUS_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
