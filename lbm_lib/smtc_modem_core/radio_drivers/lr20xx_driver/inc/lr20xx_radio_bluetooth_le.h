/*!
 * @file      lr20xx_radio_bluetooth_le.h
 *
 * @brief     Bluetooth_LE radio driver definition for LR20XX
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

#ifndef LR20XX_RADIO_BLUETOOTH_LE_H
#define LR20XX_RADIO_BLUETOOTH_LE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_bluetooth_le_types.h"

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
 * @brief Set the modulation parameters for Bluetooth_LE packets
 *
 * The workaround function @ref lr20xx_workarounds_bluetooth_le_phy_coded_syncwords must be called after @ref
 * lr20xx_radio_bluetooth_le_set_modulation_params and @ref lr20xx_radio_bluetooth_le_set_pkt_params if @p phy is set to
 * @ref LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB or @ref LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB.
 *
 * The helper function @ref lr20xx_radio_bluetooth_le_set_modulation_pkt_params configures both modulation and packet
 * Bluetooth LE parameters, and calls the @ref lr20xx_workarounds_bluetooth_le_phy_coded_syncwords if necessary.
 *
 * @param[in] context Chip implementation context
 * @param[in] phy Bluetooth_LE PHY configuration
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_workarounds_bluetooth_le_phy_coded_syncwords, lr20xx_radio_bluetooth_le_set_pkt_params,
 * lr20xx_radio_bluetooth_le_set_modulation_pkt_params
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_set_modulation_params( const void*                           context,
                                                                 const lr20xx_radio_bluetooth_le_phy_t phy );

/**
 * @brief Set the modulation parameters for Bluetooth_LE packets
 *
 * The workaround function @ref lr20xx_workarounds_bluetooth_le_phy_coded_syncwords must be called after @ref
 * lr20xx_radio_bluetooth_le_set_modulation_params and @ref lr20xx_radio_bluetooth_le_set_pkt_params if @p phy is set to
 * @ref LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB or @ref LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB.
 *
 * The helper function @ref lr20xx_radio_bluetooth_le_set_modulation_pkt_params configures both modulation and packet
 * Bluetooth LE parameters, and calls the @ref lr20xx_workarounds_bluetooth_le_phy_coded_syncwords if necessary.
 *
 * @param[in] context Chip implementation context
 * @param[in] pkt_params Bluetooth_LE packet parameters
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_workarounds_bluetooth_le_phy_coded_syncwords, lr20xx_radio_bluetooth_le_set_modulation_params,
 * lr20xx_radio_bluetooth_le_set_modulation_pkt_params
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_set_pkt_params( const void*                                   context,
                                                          const lr20xx_radio_bluetooth_le_pkt_params_t* pkt_params );

/**
 * @brief Helper function to configure both modulation and packet Bluetooth LE parameters, and to apply PHY Coded
 * syncword workaround if necessary
 *
 * This helper function executes the following sequence:
 * 1. Call @ref lr20xx_radio_bluetooth_le_set_modulation_params
 * 2. Call @ref lr20xx_radio_bluetooth_le_set_pkt_params
 * 3. Call @ref lr20xx_workarounds_bluetooth_le_phy_coded_syncwords if the @p phy parameter is @ref
 * LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB or @ref LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB.
 *
 * @param[in] context Chip implementation context
 * @param[in] phy Bluetooth_LE PHY configuration
 * @param[in] pkt_params Bluetooth_LE packet parameters
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_workarounds_bluetooth_le_phy_coded_syncwords, lr20xx_radio_bluetooth_le_set_modulation_params,
 * lr20xx_radio_bluetooth_le_set_pkt_params
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_set_modulation_pkt_params(
    const void* context, const lr20xx_radio_bluetooth_le_phy_t phy,
    const lr20xx_radio_bluetooth_le_pkt_params_t* pkt_params );

/**
 * @brief Configure the PDU length and send a Bluetooth_LE packet
 *
 * @remark This command is the concatenation of @ref lr20xx_radio_bluetooth_le_set_pdu_length and @ref
 * lr20xx_radio_common_set_tx
 *
 * @param[in] context Chip implementation context
 * @param[in] len PDU length - including 16-bit header, excluding the 3-byte CRC
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_set_tx( const void* context, uint8_t len );

/**
 * @brief Get the internal statistics of received Bluetooth_LE packets
 *
 * The internal statistics are reset on:
 *   - Power On Reset (POR)
 *   - sleep without memory retention
 *   - call to lr20xx_radio_common_reset_rx_stats
 *
 * @param[in] context Chip implementation context
 * @param[out] statistics Bluetooth_LE received packet statistics
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_common_reset_rx_stats
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_get_rx_stats( const void*                           context,
                                                        lr20xx_radio_bluetooth_le_rx_stats_t* statistics );

/**
 * @brief Get the status of the last Bluetooth_LE received packet
 *
 * Availability of the packet status fields depend on the IRQ as follows:
 * - Available from LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID:
 *     - lr20xx_radio_bluetooth_le_pkt_status_t.rssi_sync_in_dbm
 *     - lr20xx_radio_bluetooth_le_pkt_status_t.rssi_sync_half_dbm_count
 * - Available from LR20XX_SYSTEM_IRQ_RX_DONE:
 *     - lr20xx_radio_bluetooth_le_pkt_status_t.rssi_avg_in_dbm
 *     - lr20xx_radio_bluetooth_le_pkt_status_t.rssi_avg_half_dbm_count
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status Bluetooth_LE packet status structure
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_get_pkt_status( const void*                             context,
                                                          lr20xx_radio_bluetooth_le_pkt_status_t* pkt_status );

/**
 * @brief Configure the PDU length
 *
 * @param[in] context Chip implementation context
 * @param[in] len PDU length - including 16-bit header, excluding the 3-byte CRC
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_bluetooth_le_set_pdu_length( const void* context, uint8_t len );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_BLUETOOTH_LE_H

/* --- EOF ------------------------------------------------------------------ */
