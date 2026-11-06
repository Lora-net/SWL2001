/*!
 * @file      lr20xx_radio_ook.h
 *
 * @brief     On-Off Keying radio driver definition for LR20XX
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

#ifndef LR20XX_RADIO_OOK_H
#define LR20XX_RADIO_OOK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_ook_types.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Set the modulation parameters for OOK packets
 *
 * @param[in] context Chip implementation context
 * @param[in] params Structure of OOK modulation configuration
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_modulation_params( const void*                          context,
                                                        const lr20xx_radio_ook_mod_params_t* params );

/**
 * @brief Set the packet parameters for OOK packets
 *
 * The OOK packet configuration with header explicit and CRC disabled is known to generate incorrect packet reception.
 *
 * @param[in] context Chip implementation context
 * @param[in] params Structure of the OOK packet parameter to configure
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_packet_params( const void* context, const lr20xx_radio_ook_pkt_params_t* params );

/**
 * @brief Set the CRC configuration for OOK packets
 *
 * @param[in] context Chip implementation context
 * @param[in] crc_polynomial Polynomial to use for CRC LFSR
 * @param[in] crc_seed LFSR initial value
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_crc_params( const void* context, uint32_t crc_polynomial, uint32_t crc_seed );

/**
 * @brief Set the syncword for OOK packets
 *
 * The argument \p syncword is a 4-byte array. However, it should be understood as 32-bit variable, where Most
 * Significant Bit is the MSB of syncword[0] and Least Significant Bit is LSB of syncword[3].
 *
 * For instance:
 *
 * @code{.c}
 * syncword = 0x0000AA67
 * nb_bits  = 12
 * @endcode
 *
 * Then, syncword in bits is:
 *
 * @verbatim
 *  0...0 01010101 01100111
 *  ^         ^           ^
 * MSB    nb_bit'th      LSB
 * @endverbatim
 *
 * Then, the bit stream sent over the air will be
 * - if \p bit_order == LR20XX_RADIO_OOK_SYNCWORD_LSBF:
 *     @verbatim
 *     111001101010
 *     @endverbatim
 * - if \p bit_order == LR20XX_RADIO_OOK_SYNCWORD_MSBF:
 *     @verbatim
 *     010101100111
 *     @endverbatim
 *
 * @param[in] context Chip implementation context
 * @param[in] syncword Array holding the syncword value. It is up to the caller that this array holds at least
 * LR20XX_RADIO_OOK_SYNCWORD_LENGTH bytes, even if nb_bits is not 32
 * @param[in] nb_bits The number of significant bits in syncword to use for syncword. The significant bits are taken as
 * Least Significant Bits of \p syncword argument. Value in range of [0:32]
 * @param[in] bit_order The order of transmission of the selected bits of syncword over-the-air
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_syncword( const void*   context,
                                               const uint8_t syncword[LR20XX_RADIO_OOK_SYNCWORD_LENGTH],
                                               uint8_t nb_bits, lr20xx_radio_ook_syncword_bit_order_t bit_order );

/**
 * @brief Set the node and broadcast addresses for OOK packets
 *
 * @param[in] context Chip implementation context
 * @param[in] node_address Node address
 * @param[in] broadcast_address Broadcast address
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_addresses( const void* context, uint8_t node_address, uint8_t broadcast_address );

/**
 * @brief Get the internal statistics of received OOK packets
 *
 * The internal statistics are reset on:
 *   - Power On Reset (POR)
 *   - sleep without memory retention
 *   - call to lr20xx_radio_common_reset_rx_stats
 *
 * @param[in] context Chip implementation context
 * @param[out] statistics Pointer to a structure of statistic to populate with internal statistics
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_common_reset_rx_stats
 */
lr20xx_status_t lr20xx_radio_ook_get_rx_statistics( const void* context, lr20xx_radio_ook_rx_statistics_t* statistics );

/**
 * @brief Get the status of the last received OOK packet
 *
 * Availability of the packet status fields depend on the IRQ as follows:
 * - Available from LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID:
 *     - lr20xx_radio_ook_packet_status_t.rssi_on_in_dbm
 *     - lr20xx_radio_ook_packet_status_t.rssi_on_half_dbm_count
 * - Available from LR20XX_SYSTEM_IRQ_RX_DONE:
 *     - lr20xx_radio_ook_packet_status_t.rssi_avg_in_dbm
 *     - lr20xx_radio_ook_packet_status_t.rssi_avg_half_dbm_count
 *     - lr20xx_radio_ook_packet_status_t.is_addr_match_broadcast
 *     - lr20xx_radio_ook_packet_status_t.is_addr_match_node
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status Pointer to a structure of packet status to populate
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_ook_get_packet_status( const void* context, lr20xx_radio_ook_packet_status_t* pkt_status );

/**
 * @brief Configure the Rx detector OOK packet
 *
 * @param[in] context Chip implementation context
 * @param[in] rx_detector Rx detector configuration
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_rx_detector( const void*                           context,
                                                  const lr20xx_radio_ook_rx_detector_t* rx_detector );

/**
 * @brief Set whitening parameters for OOK packet
 *
 * @param[in] context Chip implementation context
 * @param[in] params Whitening parameters
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_ook_set_whitening_params( const void*                                context,
                                                       const lr20xx_radio_ook_whitening_params_t* params );

/**
 * @brief Get the time on air in ms for OOK transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the OOK packet parameters
 * @param [in] mod_p Pointer to a structure holding the OOK modulation parameters
 * @param [in] syncword_len_in_bit Syncword length in bit
 *
 * @returns Time-on-air value in ms for OOK transmission
 */
uint32_t lr20xx_radio_ook_get_time_on_air_in_ms( const lr20xx_radio_ook_pkt_params_t* pkt_p,
                                                 const lr20xx_radio_ook_mod_params_t* mod_p,
                                                 uint8_t                              syncword_len_in_bit );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_OOK_H

/* --- EOF ------------------------------------------------------------------ */
