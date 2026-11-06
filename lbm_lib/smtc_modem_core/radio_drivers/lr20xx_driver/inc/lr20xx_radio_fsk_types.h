/*!
 * @file      lr20xx_radio_fsk_types.h
 *
 * @brief     FSK radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_FSK_TYPES_H
#define LR20XX_RADIO_FSK_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lr20xx_radio_fsk_common_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Length in bytes of the FSK syncword
 *
 */
#define LR20XX_RADIO_FSK_SYNCWORD_LENGTH ( 8 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Type of bitrate for FSK
 *
 * This type is a synonym of uint32_t. The meaning of the corresponding value depends on the value of the MSB:
 *   - If MSB is 0, then the 30 remaining LSBs represent the bitrate in bits per second
 *   - If MSB is 1, then the 30 remaining LSBs represent the bitrate in 1/256 bits per second
 */
typedef uint32_t lr20xx_radio_fsk_bitrate_t;

/**
 * @brief Values for FSK pulse shape configuration
 */
typedef enum
{
    LR20XX_RADIO_FSK_PULSE_SHAPE_DISABLED        = 0x00,  //!< Disable pulse shaping
    LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_3 = 0x04,  //!< Pulse shaping Gaussian with BT 0.3
    LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_5 = 0x05,  //!< Pulse shaping Gaussian with BT 0.5
    LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_7 = 0x06,  //!< Pulse shaping Gaussian with BT 0.7
    LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_1_0 = 0x07,  //!< Pulse shaping Gaussian with BT 1
} lr20xx_radio_fsk_pulse_shape_t;

/**
 * @brief Structure of configuration for FSK modulation
 */
typedef struct
{
    lr20xx_radio_fsk_bitrate_t     br;           //!< Bitrate
    lr20xx_radio_fsk_pulse_shape_t pulse_shape;  //!< Pulse shape filter
    lr20xx_radio_fsk_common_bw_t   bw;           //!< Bandwidth
    uint32_t                       fdev_in_hz;   //!< Frequency deviation [Hz]
} lr20xx_radio_fsk_mod_params_t;

/**
 * @brief Preamble detector configuration
 *
 * @remark If enabled, a packet detection will start after the given number of bits of preamble is detected.
 */
typedef enum
{
    LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_DISABLED = 0x00,  //!< Preamble detector disabled - detection on syncword
    LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_8_BITS   = 0x08,  //!< Preamble detector set to 8 bits
    LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_16_BITS  = 0x10,  //!< Preamble detector set to 16 bits
    LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_24_BITS  = 0x18,  //!< Preamble detector set to 24 bits
    LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_32_BITS  = 0x20,  //!< Preamble detector set to 32 bits
} lr20xx_radio_fsk_preamble_detector_t;

/**
 * @brief Configuration of address filtering
 */
typedef enum
{
    LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED       = 0x00,  //!< Disable address filtering
    LR20XX_RADIO_FSK_ADDRESS_FILTERING_NODE           = 0x01,  //!< Filter on node address
    LR20XX_RADIO_FSK_ADDRESS_FILTERING_NODE_BROADCAST = 0x02,  //!< Filter on both node and broadcast addresses
} lr20xx_radio_fsk_address_filtering_t;

/**
 * @brief Configure the header mode of FSK packet
 */
typedef enum
{
    LR20XX_RADIO_FSK_HEADER_IMPLICIT =
        0x00,  //!< (aka. fixed length packet) The packet is sent without header so the receiver must be configured to
               //!< receive the same payload length as the transmitted one
    LR20XX_RADIO_FSK_HEADER_8BITS = 0x01,  //!< The packet can have variable payload length encoded in a 8 bits
                                           //!< header. Compatible with SX126X and SX127X
    LR20XX_RADIO_FSK_HEADER_SX128X_COMPATIBLE =
        0x02,  //!< The packet can have a variable payload length up to 255 bytes. The packet header is compatible with
               //!< SX128X: payload length is encoded on 9 bits but MSB is RFU
    LR20XX_RADIO_FSK_HEADER_16BITS = 0x03,  //!< Variable payload length encoded on 15LSBs - MSB is RFU
} lr20xx_radio_fsk_header_mode_t;

/**
 * @brief Configuration of CRC field
 */
typedef enum
{
    LR20XX_RADIO_FSK_CRC_OFF              = 0x00,  //!< CRC is not added to packet
    LR20XX_RADIO_FSK_CRC_1_BYTE           = 0x01,  //!< CRC is 1-byte long
    LR20XX_RADIO_FSK_CRC_2_BYTES          = 0x02,  //!< CRC is 2-byte long
    LR20XX_RADIO_FSK_CRC_3_BYTES          = 0x03,  //!< CRC is 3-byte long
    LR20XX_RADIO_FSK_CRC_4_BYTES          = 0x04,  //!< CRC is 4-byte long
    LR20XX_RADIO_FSK_CRC_1_BYTE_INVERTED  = 0x09,  //!< CRC is 1-byte long and inverted
    LR20XX_RADIO_FSK_CRC_2_BYTES_INVERTED = 0x0A,  //!< CRC is 2-byte long and inverted
    LR20XX_RADIO_FSK_CRC_3_BYTES_INVERTED = 0x0B,  //!< CRC is 3-byte long and inverted
    LR20XX_RADIO_FSK_CRC_4_BYTES_INVERTED = 0x0C,  //!< CRC is 4-byte long and inverted
} lr20xx_radio_fsk_crc_t;

/**
 * @brief Configuration of whitening (aka DC-free)
 */
typedef enum
{
    LR20XX_RADIO_FSK_WHITENING_OFF = 0x00,  //!< Whitening is disabled
    LR20XX_RADIO_FSK_WHITENING_ON  = 0x01,  //!< Whitening is enabled
} lr20xx_radio_fsk_whitening_t;

/**
 * @brief Unit of the payload length field packet configuration
 */
typedef enum lr20xx_radio_fsk_payload_length_unit_e
{
    LR20XX_RADIO_FSK_PAYLOAD_LENGTH_IN_BYTE = 0x00,  //!< Payload length field is populated as number of bytes
    LR20XX_RADIO_FSK_PAYLOAD_LENGTH_IN_BIT  = 0x01,  //!< Payload length field is populated as number of bits
} lr20xx_radio_fsk_payload_length_unit_t;

/**
 * @brief Structure of configuration for FSK packet parameters
 *
 * The \p payload_length field meaning depends on the header_mode value:
 * - if header_mode is \p LR20XX_RADIO_FSK_HEADER_IMPLICIT, then \p payload_length is the number of payload byte to
 * receive (not including CRC configuration)
 * - otherwise, \p payload_length is the maximal length of packet to receive (not including CRC). In this case, if a
 * packet is received with a header indicating the payload length is superior to the one configured, the chip raises
 * the IRQ \p LR20XX_SYSTEM_IRQ_LEN_ERROR along the \p LR20XX_SYSTEM_IRQ_RX_DONE (if these IRQs have been configured
 * with \p lr20xx_system_set_dio_irq_cfg function).
 *
 * If \p header_mode is set to @ref LR20XX_RADIO_FSK_HEADER_SX128X_COMPATIBLE, then \p address_filtering must be set
 * to
 * @ref LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED. Using address filtering with SX128X compatible header mode is
 * not supported.
 */
typedef struct
{
    uint16_t                             pbl_length_in_bit;  //!< Preamble length in bits
    lr20xx_radio_fsk_preamble_detector_t preamble_detector;  //!< Preamble detector configuration
    bool long_preamble_enabled;  //!< Set to true to enable long preamble reception. This needs to be set to true to
                                 //!< receive packet with preamble length higher than 2000 bits. With this mode enabled,
                                 //!< the LR20xx will detect preamble, then switch to syncword detection. So detection
                                 //!< IRQ can occur twice: once for preamble detection, and once for syncword detection.
                                 //!< @note If this mode is enabled, packets where preamble's time on air is inferior to
                                 //!< the configured preamble detection length + 8bits + 10us cannot be received.
    lr20xx_radio_fsk_address_filtering_t   address_filtering;    //!< Address filtering configuration
    lr20xx_radio_fsk_header_mode_t         header_mode;          //!< Header mode
    lr20xx_radio_fsk_payload_length_unit_t payload_length_unit;  //!< Unit of the @p payload_length field
    uint16_t                     payload_length;  //!< Payload length (unit depends on the @p payload_length_unit field)
    lr20xx_radio_fsk_crc_t       crc;             //!< CRC configuration
    lr20xx_radio_fsk_whitening_t whitening;       //!< Whitening (aka. DC free) configuration
} lr20xx_radio_fsk_pkt_params_t;

/**
 * @brief Whitening type configuration
 *
 */
typedef enum
{
    LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX126X_LR11XX = 0x00,  //!< Whitening compatible with SX126X and LR11XX
    LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX128X        = 0x01,  //!< Whitening compatible with SX128X
} lr20xx_radio_fsk_whitening_compatibility_t;

/**
 * @brief Syncword endianess
 */
typedef enum
{
    LR20XX_RADIO_FSK_SYNCWORD_LSBF = 0x00,  //!< Syncword is transmitted Least Significant Bit First
    LR20XX_RADIO_FSK_SYNCWORD_MSBF =
        0x01,  //!< Syncword is transmitted Most Significant Bit First (compatible with SX126X, LR11XX, SX128X)
} lr20xx_radio_fsk_syncword_bit_order_t;

/**
 * @brief Rx statistics of FSK packets
 */
typedef struct lr20xx_radio_fsk_rx_statistics_s
{
    uint16_t n_received_packets;     //!< Number of received packets
    uint16_t n_crc_errors;           //!< Number of received packets with CRC error
    uint16_t n_length_errors;        //!< Number of received packets with length error
    uint16_t n_preamble_detections;  //!< Number of preamble detections
    uint16_t n_syncword_ok;          //!< Number of valid syncwords detections
    uint16_t n_syncword_fail;        //!< Number of failed syncwords detections
    uint16_t n_timeout;              //!< Number of timeouts
} lr20xx_radio_fsk_rx_statistics_t;

/**
 * @brief Structure of packet status for FSK
 */
typedef struct lr20xx_radio_fsk_packet_status_s
{
    uint16_t packet_length_bytes;       //!< Length of last received packet in bytes
    int16_t  rssi_avg_in_dbm;           //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    int16_t  rssi_sync_in_dbm;          //!< RSSI in dBm - value after syncword detection
    uint8_t  rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    bool     is_addr_match_broadcast;  //!< True if address filtering is enabled and match with broadcast address, false
                                       //!< otherwise
    bool is_addr_match_node;           //!< True if address filtering is enabled and match with node address, false
                                       //!< otherwise
    uint8_t link_quality_indicator;    //!< Margin to the detection level, in 0.25dB
} lr20xx_radio_fsk_packet_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FSK_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
