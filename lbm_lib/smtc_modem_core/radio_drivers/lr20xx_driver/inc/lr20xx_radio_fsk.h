/*!
 * @file      lr20xx_radio_fsk.h
 *
 * @brief     Radio FSK driver definition for LR20XX
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

#ifndef LR20XX_RADIO_FSK_H
#define LR20XX_RADIO_FSK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_fsk_types.h"
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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Set the modulation parameters for FSK packets
 *
 * @param[in] context Chip implementation context
 * @param[in] mod_params Structure of the FSK modulation parameter to configure
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_set_modulation_params( const void*                          context,
                                                        const lr20xx_radio_fsk_mod_params_t* mod_params );

/**
 * @brief Set the packet parameters for FSK packets
 *
 * @remark If whitening is enabled, the whitening type used is the one configured by calling @ref
 * lr20xx_radio_fsk_set_whitening_params
 *
 * @param[in] context Chip implementation context
 * @param[in] pkt_params Structure of the FSK packet parameter to configure
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_set_packet_params( const void*                          context,
                                                    const lr20xx_radio_fsk_pkt_params_t* pkt_params );

/**
 * @brief Set the whitening parameters for FSK packets
 *
 * @remark The whitening parameters - \p whitening_type and \p whitening_seed - are applied if whitening is enabled when
 * calling @ref lr20xx_radio_fsk_set_packet_params
 *
 * @param[in] context Chip implementation context
 * @param[in] whitening_type The whitening compatibility mode to configure
 * @param[in] whitening_seed The whitening seed to use. Only the 12 LSBs are meaningful
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_set_whitening_params( const void*                                context,
                                                       lr20xx_radio_fsk_whitening_compatibility_t whitening_type,
                                                       uint16_t                                   whitening_seed );

/**
 * @brief Set the CRC configuration for FSK packets
 *
 * @param[in] context Chip implementation context
 * @param[in] crc_polynom The polynom to use for CRC LFSR
 * @param[in] crc_seed The LFSR initial value
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_set_crc_params( const void* context, uint32_t crc_polynom, uint32_t crc_seed );

/**
 * @brief Set the syncword for FSK packets
 *
 * The argument \p syncword is an array of 8 bytes. However, it should be understood a 64 bit variable, where Most
 * Significant Bit is the MSB of syncword[0] and Least Significant Bit is LSB of syncword[7].
 *
 * For instance:
 *
 * @code{.c}
 * syncword[LR20XX_RADIO_FSK_SYNCWORD_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0x67};
 * nb_bits  = 12;
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
 * - if \p bit_order == LR20XX_RADIO_FSK_SYNCWORD_LSBF:
 *     @verbatim
 *     111001101010
 *     @endverbatim
 * - if \p bit_order == LR20XX_RADIO_FSK_SYNCWORD_MSBF:
 *     @verbatim
 *     010101100111
 *     @endverbatim
 *
 * @param[in] context Chip implementation context
 * @param[in] syncword Array holding the syncword value. It is up to the caller that this array holds at least
 * LR20XX_RADIO_FSK_SYNCWORD_LENGTH bytes, even if nb_bits is not 64
 * @param[in] nb_bits The number of significant bits in syncword to use for syncword. The significant bits are taken as
 * Least Significant Bits of \p syncword argument. Value in range of [0:64]
 * @param[in] bit_order The order of transmission of the selected bits of syncword over-the-air
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_set_syncword( const void*   context,
                                               const uint8_t syncword[LR20XX_RADIO_FSK_SYNCWORD_LENGTH],
                                               uint8_t nb_bits, lr20xx_radio_fsk_syncword_bit_order_t bit_order );

/**
 * @brief Set the node and broadcast addresses for FSK packets
 *
 * @param[in] context Chip implementation context
 * @param[in] node_address Node address
 * @param[in] broadcast_address Broadcast address
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_set_addresses( const void* context, uint8_t node_address, uint8_t broadcast_address );

/**
 * @brief Get the internal statistics of received FSK packets
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
lr20xx_status_t lr20xx_radio_fsk_get_rx_statistics( const void* context, lr20xx_radio_fsk_rx_statistics_t* statistics );

/**
 * @brief Get the status of the last received packet
 *
 * Availability of the packet status fields depend on the IRQ as follows:
 * - Available from LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID:
 *     - lr20xx_radio_fsk_packet_status_t.rssi_sync_in_dbm
 *     - lr20xx_radio_fsk_packet_status_t.rssi_sync_half_dbm_count
 * - Available from LR20XX_SYSTEM_IRQ_RX_DONE:
 *     - lr20xx_radio_fsk_packet_status_t.rssi_avg_in_dbm
 *     - lr20xx_radio_fsk_packet_status_t.rssi_avg_half_dbm_count
 *     - lr20xx_radio_fsk_packet_status_t.is_addr_match_broadcast
 *     - lr20xx_radio_fsk_packet_status_t.is_addr_match_node
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status Pointer to a structure of packet status to populate
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_get_packet_status( const void* context, lr20xx_radio_fsk_packet_status_t* pkt_status );

/*!
 * @brief Get the radio bw parameter for a given bandwidth in Hz
 *
 * @param [in] bw_in_hz Requested GFSK Rx bandwidth
 * @param [out] bw_parameter Radio parameter immediately above requested bw_in_hz
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_radio_fsk_get_rx_bandwidth( uint32_t bw_in_hz, lr20xx_radio_fsk_common_bw_t* bw_parameter );

/**
 * @brief Compute the numerator for GFSK time-on-air computation.
 *
 * @remark To get the actual time-on-air in seconds, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * @param [in] pkt_p Pointer to the structure holding the GFSK packet parameters
 * @param [in] syncword_len_in_bit Syncword length in bit
 *
 * @returns GFSK time-on-air numerator
 */
uint32_t lr20xx_radio_fsk_get_time_on_air_numerator( const lr20xx_radio_fsk_pkt_params_t* pkt_p,
                                                     uint8_t                              syncword_len_in_bit );

/**
 * @brief Get the time on air in ms for GFSK transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the GFSK packet parameters
 * @param [in] mod_p Pointer to a structure holding the GFSK modulation parameters
 * @param [in] syncword_len_in_bit Syncword length in bit
 *
 * @returns Time-on-air value in ms for GFSK transmission
 */
uint32_t lr20xx_radio_fsk_get_time_on_air_in_ms( const lr20xx_radio_fsk_pkt_params_t* pkt_p,
                                                 const lr20xx_radio_fsk_mod_params_t* mod_p,
                                                 uint8_t                              syncword_len_in_bit );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FSK_H

/* --- EOF ------------------------------------------------------------------ */
