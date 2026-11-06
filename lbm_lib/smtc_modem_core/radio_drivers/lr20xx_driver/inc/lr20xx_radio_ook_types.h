/*!
 * @file      lr20xx_radio_ook_types.h
 *
 * @brief     On-Off Keying radio types driver definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#ifndef LR20XX_RADIO_OOK_TYPES_H
#define LR20XX_RADIO_OOK_TYPES_H

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
 * @brief Length in bytes of the OOK syncword
 *
 */
#define LR20XX_RADIO_OOK_SYNCWORD_LENGTH ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Type of bitrate for OOK
 *
 * This type is a synonym of uint32_t. The meaning of the corresponding value depends on the value of the MSB:
 *   - If MSB is 0, then the 31 remaining LSBs represent the bitrate in bits per second
 *   - If MSB is 1, then the 31 remaining LSBs represent the bitrate in 1/256 bits per second
 */
typedef uint32_t lr20xx_radio_ook_bitrate_t;

/**
 * @brief Filter configuration for the pulse shape of OOK modulation
 *
 */
typedef enum
{
    LR20XX_RADIO_OOK_PULSE_SHAPE_NO_FILTER       = 0x00,  //!< No filter
    LR20XX_RADIO_OOK_PULSE_SHAPE_FILTER_GAUSSIAN = 0x01,  //!< Gaussian filter
} lr20xx_radio_ook_pulse_shape_filter_t;

/**
 * @brief BT value for the pulse shape filter of OOK modulation
 */
typedef enum
{
    LR20XX_RADIO_OOK_PULSE_SHAPE_BT_0_3 = 0x00,
    LR20XX_RADIO_OOK_PULSE_SHAPE_BT_0_5 = 0x01,
    LR20XX_RADIO_OOK_PULSE_SHAPE_BT_0_7 = 0x02,
    LR20XX_RADIO_OOK_PULSE_SHAPE_BT_1_0 = 0x03,
} lr20xx_radio_ook_pulse_shape_bt_t;

/**
 * @brief Magnitude depth value
 */
typedef enum
{
    LR20XX_RADIO_OOK_MAG_DEPTH_FULL = 0x00,  //!< The magnitude depth is limited by the power amplifier and depends on
                                             //!< the output power, external components and bitrate
    LR20XX_RADIO_OOK_MAG_DEPTH_UP_TO_20DB =
        0x01,  //!< Maximal magnitude depth is 20dB. The exact value must be measured on the device. It depends on
               //!< external components and bitrate.
} lr20xx_radio_ook_mag_depth_t;

/**
 * @brief Configuration of address filtering
 */
typedef enum
{
    LR20XX_RADIO_OOK_ADDRESS_FILTERING_DISABLED       = 0x00,  //!< Disable address filtering
    LR20XX_RADIO_OOK_ADDRESS_FILTERING_NODE           = 0x01,  //!< Filter on node address
    LR20XX_RADIO_OOK_ADDRESS_FILTERING_NODE_BROADCAST = 0x02,  //!< Filter on both node and broadcast addresses
} lr20xx_radio_ook_address_filtering_t;

/**
 * @brief Configure the header mode of OOK packet
 */
typedef enum
{
    LR20XX_RADIO_OOK_HEADER_IMPLICIT =
        0x00,  //!< (aka. fixed length packet) The packet is sent without header so the receiver must be configured to
               //!< receive the same payload length as the transmitted one
    LR20XX_RADIO_OOK_HEADER_EXPLICIT = 0x01,  //!< Packet with variable payload length encoded in a 8-bit header
    LR20XX_RADIO_OOK_HEADER_16BITS   = 0x03,  //!< Variable payload length encoded on 15LSBs - MSB is RFU
} lr20xx_radio_ook_header_mode_t;

/**
 * @brief Configuration of CRC field
 */
typedef enum
{
    LR20XX_RADIO_OOK_CRC_OFF              = 0x00,  //!< CRC is not added to packet
    LR20XX_RADIO_OOK_CRC_1_BYTE           = 0x01,  //!< CRC is 1-byte long
    LR20XX_RADIO_OOK_CRC_2_BYTES          = 0x02,  //!< CRC is 2-byte long
    LR20XX_RADIO_OOK_CRC_3_BYTES          = 0x03,  //!< CRC is 3-byte long
    LR20XX_RADIO_OOK_CRC_4_BYTES          = 0x04,  //!< CRC is 4-byte long
    LR20XX_RADIO_OOK_CRC_1_BYTE_INVERTED  = 0x09,  //!< CRC is 1-byte long and inverted
    LR20XX_RADIO_OOK_CRC_2_BYTES_INVERTED = 0x0A,  //!< CRC is 2-byte long and inverted
    LR20XX_RADIO_OOK_CRC_3_BYTES_INVERTED = 0x0B,  //!< CRC is 3-byte long and inverted
    LR20XX_RADIO_OOK_CRC_4_BYTES_INVERTED = 0x0C,  //!< CRC is 4-byte long and inverted
} lr20xx_radio_ook_crc_t;

/**
 * @brief Configuration of CRC field
 */
typedef enum
{
    LR20XX_RADIO_OOK_ENCODING_OFF            = 0x00,  //!< No encoding
    LR20XX_RADIO_OOK_ENCODING_MANCHESTER     = 0x01,  //!< Manchester encoding
    LR20XX_RADIO_OOK_ENCODING_MANCHESTER_INV = 0x09,  //!< Inverted-manchester encoding
} lr20xx_radio_ook_encoding_t;

/**
 * @brief Syncword endianess
 */
typedef enum
{
    LR20XX_RADIO_OOK_SYNCWORD_LSBF = 0x00,  //!< Syncword is transmitted Least Significant Bit First
    LR20XX_RADIO_OOK_SYNCWORD_MSBF = 0x01,  //!< Syncword is transmitted Most Significant Bit First
} lr20xx_radio_ook_syncword_bit_order_t;

/**
 * @brief Start-of-Frame delimiter type for OOK Rx detector
 */
typedef enum
{
    LR20XX_RADIO_OOK_RX_DETECTOR_SFD_TYPE_FALLING_EDGE = 0x00,  //!<
    LR20XX_RADIO_OOK_RX_DETECTOR_SFD_TYPE_RISING_EDGE  = 0x01,  //!<
} lr20xx_radio_ook_rx_detector_sfd_type_t;

/**
 * @brief Structure of the pulse shape configuration for OOK
 */
typedef struct
{
    lr20xx_radio_ook_pulse_shape_filter_t filter;  //!< Filter configuration
    lr20xx_radio_ook_pulse_shape_bt_t     bt;      //!< BT value to configure for the filter
} lr20xx_radio_ook_pulse_shape_t;

/**
 * @brief Modulation configuration for OOK packet
 */
typedef struct lr20xx_radio_ook_mod_params_s
{
    lr20xx_radio_ook_bitrate_t     br;           //!< Bitrate
    lr20xx_radio_ook_pulse_shape_t pulse_shape;  //!< Pulse shape
    lr20xx_radio_fsk_common_bw_t   bw;           //!< Bandwidth
    lr20xx_radio_ook_mag_depth_t   mag_depth;    //!< Magnitude depth
} lr20xx_radio_ook_mod_params_t;

/**
 * @brief Structure of configuration for OOK packet parameters
 *
 * The \p payload_length field meaning depends on the header_mode value:
 * - if header_mode is \p LR20XX_RADIO_OOK_HEADER_IMPLICIT, then \p payload_length is the number of payload byte to
 * receive (not including CRC configuration)
 * - otherwise, \p payload_length is the maximal length of packet to receive (not including CRC). In this case, if a
 * packet is received with a header indicating the payload length is superior to the one configured, the chip raises the
 * IRQ \p LR20XX_SYSTEM_IRQ_LEN_ERROR (if this IRQ has been configured with \p lr20xx_system_set_dio_irq_cfg function).
 */
typedef struct
{
    uint16_t                             pbl_length_in_bit;  //!< Preamble length in bits
    lr20xx_radio_ook_address_filtering_t address_filtering;  //!< Address filtering configuration
    lr20xx_radio_ook_header_mode_t       header_mode;        //!< Header mode
    uint16_t                             payload_length;     //!< Payload length in bytes
    lr20xx_radio_ook_crc_t               crc;                //!< CRC configuration
    lr20xx_radio_ook_encoding_t          encoding;           //!< Encoding configuration
} lr20xx_radio_ook_pkt_params_t;

/**
 * @brief Rx statistics of OOK packets
 */
typedef struct lr20xx_radio_ook_rx_statistics_s
{
    uint16_t n_received_packets;  //!< Number of received packets
    uint16_t n_crc_errors;        //!< Number of received packets with CRC error
    uint16_t n_length_errors;     //!< Number of received packets with length error
} lr20xx_radio_ook_rx_statistics_t;

/**
 * @brief Structure of OOK packet status
 */
typedef struct lr20xx_radio_ook_packet_status_s
{
    uint16_t packet_length_bytes;      //!< Length of last received packet in bytes
    int16_t  rssi_avg_in_dbm;          //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;  //!< Count of 0.5 dBm to subtract to @p rssi_avg_in_dbm value in dBm
    int16_t  rssi_on_in_dbm;           //!< RSSI estimated during @a ON part in dBm
    uint8_t  rssi_on_half_dbm_count;   //!< Count of 0.5 dBm to subtract to @p rssi_on_in_dbm value in dBm
    bool     is_addr_match_broadcast;  //!< True if address filtering is enabled and match with broadcast address, false
                                       //!< otherwise
    bool is_addr_match_node;           //!< True if address filtering is enabled and match with node address, false
                                       //!< otherwise
    uint8_t link_quality_indicator;    //!< Average difference between @a 0 and @a 1 symbols, in dB
} lr20xx_radio_ook_packet_status_t;

/**
 * @brief Rx detector configuration for OOK packet
 */
typedef struct
{
    uint16_t                                pattern;                //!< Rx detector pattern
    uint8_t                                 pattern_length_in_bit;  //!< Rx detector pattern length - 1
    uint8_t                                 pattern_repeat_nb;  //!< Rx detector pattern repetition number - in [0:31]
    lr20xx_radio_ook_rx_detector_sfd_type_t sfd_type;           //!< Start-of-Frame Delimiter type
    uint8_t                                 sfd_length_in_bit;  //!< Start-of-Frame Delimiter length in bit - in [0:15]
    bool is_syncword_encoded;  //!< Configure the encoding of syncword. If set to \p true the syncword is encoded the
                               //!< same way as the payload
} lr20xx_radio_ook_rx_detector_t;

/**
 * @brief Rx detector configuration for OOK packet
 */
typedef struct
{
    uint8_t  bit_index;   //!< LFSR bit index - in [0:15]
    uint16_t polynomial;  //!< Whitening polynomial - 12-bit value
    uint16_t seed;        //!< Whitening seed - 12-bit value
} lr20xx_radio_ook_whitening_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_OOK_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
