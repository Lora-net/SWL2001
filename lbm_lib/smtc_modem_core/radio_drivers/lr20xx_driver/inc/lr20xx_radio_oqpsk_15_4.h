/*!
 * @file      lr20xx_radio_oqpsk_15_4.h
 *
 * @brief     OQPSK 15.4 driver definition for LR20XX
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

#ifndef LR20XX_RADIO_OQPSK_15_4_H
#define LR20XX_RADIO_OQPSK_15_4_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr20xx_radio_oqpsk_15_4_types.h"
#include "lr20xx_status.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Length of OQPSK 15.4 physical device address in byte
 */
#define LR20XX_RADIO_OQPSK_15_4_EXTENDED_DEVICE_ADDRESS_LENGTH ( 8 )

/**
 * @brief Length of OQPSK 15.4 network device address in byte
 */
#define LR20XX_RADIO_OQPSK_15_4_NETWORK_DEVICE_ADDRESS_LENGTH ( 2 )

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
 * @brief Set the OQPSK 15.4 parameters
 *
 * @param[in] context Chip implementation context
 * @param[in] params OQPSK 15.4 parameters
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_oqpsk_15_4_set_params( const void*                             context,
                                                    const lr20xx_radio_oqpsk_15_4_params_t* params );

/**
 * @brief Set the payload length of OQPSK 15.4 parameters
 *
 * This command is equivalent to @ref lr20xx_radio_oqpsk_15_4_set_params where only the payload_length field is changed.
 * It is to be used in time constrained situations where calling @ref lr20xx_radio_oqpsk_15_4_set_params would take too
 * long.
 *
 * @param context Chip implementation context
 * @param payload_length Payload length to set, refer to lr20xx_radio_oqpsk_15_4_params_t.payload_length for details
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_oqpsk_15_4_set_params, lr20xx_radio_oqpsk_15_4_params_t
 */
lr20xx_status_t lr20xx_radio_oqpsk_15_4_set_payload_length( const void* context, uint8_t payload_length );

/**
 * @brief Get the internal statistics of received OQPSK 15.4 packets
 *
 * The internal statistics are reset on:
 *   - Power On Reset (POR)
 *   - sleep without memory retention
 *   - call to lr20xx_radio_common_reset_rx_stats
 *
 * @param[in] context Chip implementation context
 * @param[out] statistics OQPSK 15.4 received packet statistics
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_common_reset_rx_stats
 */
lr20xx_status_t lr20xx_radio_oqpsk_15_4_get_rx_stats( const void*                         context,
                                                      lr20xx_radio_oqpsk_15_4_rx_stats_t* statistics );

/**
 * @brief Get the status of the last OQPSK 15.4 received packet
 *
 * Availability of the packet status fields depend on the IRQ as follows:
 * - Available from LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID:
 *     - lr20xx_radio_oqpsk_15_4_pkt_status_t.rssi_sync_in_dbm
 *     - lr20xx_radio_oqpsk_15_4_pkt_status_t.rssi_sync_half_dbm_count
 * - Available from LR20XX_SYSTEM_IRQ_RX_DONE:
 *     - lr20xx_radio_oqpsk_15_4_pkt_status_t.phr
 *     - lr20xx_radio_oqpsk_15_4_pkt_status_t.rssi_avg_in_dbm
 *     - lr20xx_radio_oqpsk_15_4_pkt_status_t.rssi_avg_half_dbm_count
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status OQPSK 15.4 packet status structure
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_oqpsk_15_4_get_pkt_status( const void*                           context,
                                                        lr20xx_radio_oqpsk_15_4_pkt_status_t* pkt_status );

/**
 * @brief Set the long and short OQPSK 15.4 destination addresses
 *
 * Long destination address is also called *extended address* or MAC address. It the the 64-bit device address.
 * Short destination address is also called network device address. It is the 16-bit device address.
 *
 * Upon packet reception, the long or short address (whichever applies) and personal area network identifier are checked
 * against the configured ones. If it does not match, the @ref LR20XX_SYSTEM_IRQ_ADDR_ERROR is raised and the packet
 * reception is stopped.
 *
 * If a message with broadcast address is received, it is not filtered out. Multicast is not filtered.
 *
 * @param [in] context Chip implementation context
 * @param [in] extended_device_address The long destination address, also called MAC address. It is up to the caller to
 * ensure it is at least @ref LR20XX_RADIO_OQPSK_15_4_EXTENDED_DEVICE_ADDRESS_LENGTH byte long
 * @param [in] network_device_address The short destination address. It is up to the caller to ensure it is at least
 * @ref LR20XX_RADIO_OQPSK_15_4_NETWORK_DEVICE_ADDRESS_LENGTH byte long
 * @param [in] personal_area_network_id The personal area network identifier
 * @param [in] transaction_id The transaction identifier
 *
 * @return lr20xx_status_t
 */
lr20xx_status_t lr20xx_radio_oqpsk_15_4_set_addresses(
    const void* context, const uint8_t extended_device_address[LR20XX_RADIO_OQPSK_15_4_EXTENDED_DEVICE_ADDRESS_LENGTH],
    const uint8_t network_device_address[LR20XX_RADIO_OQPSK_15_4_NETWORK_DEVICE_ADDRESS_LENGTH],
    uint16_t personal_area_network_id, uint8_t transaction_id );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_OQPSK_15_4_H

/* --- EOF ------------------------------------------------------------------ */
