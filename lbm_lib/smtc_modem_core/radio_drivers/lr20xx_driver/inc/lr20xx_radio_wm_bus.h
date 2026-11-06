/*!
 * @file      lr20xx_radio_wm_bus.h
 *
 * @brief     Radio Wireless M-Bus driver definition for LR20XX
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

#ifndef LR20XX_RADIO_WM_BUS_H
#define LR20XX_RADIO_WM_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr20xx_radio_wm_bus_types.h"
#include "lr20xx_radio_fsk_common_types.h"
#include "lr20xx_status.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Length in byte of the Wireless M-Bus address
 */
#define LR20XX_RADIO_WM_BUS_ADDR_LEN 6

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
 * @brief Set the Wireless M-Bus parameters
 *
 * @param[in] context Chip implementation context
 * @param[in] params Wireless M-Bus parameters
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_wm_bus_set_params( const void* context, const lr20xx_radio_wm_bus_params_t* params );

/**
 * @brief Get the internal statistics of received Wireless M-Bus packets
 *
 * The internal statistics are reset on:
 *   - Power On Reset (POR)
 *   - sleep without memory retention
 *   - call to lr20xx_radio_common_reset_rx_stats
 *
 * @param[in] context Chip implementation context
 * @param[out] statistics Wireless M-Bus received packet statistics
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_common_reset_rx_stats
 */
lr20xx_status_t lr20xx_radio_wm_bus_get_rx_stats( const void* context, lr20xx_radio_wm_bus_rx_stats_t* statistics );

/**
 * @brief Get the status of the last Wireless M-Bus received packet
 *
 * Availability of the packet status fields depend on the IRQ as follows:
 * - Available from LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID:
 *     - lr20xx_radio_wm_bus_pkt_status_t.rssi_sync_in_dbm
 *     - lr20xx_radio_wm_bus_pkt_status_t.rssi_sync_half_dbm_count
 *     - lr20xx_radio_wm_bus_pkt_status_t.pkt_format
 * - Available from LR20XX_SYSTEM_IRQ_RX_DONE:
 *     - lr20xx_radio_wm_bus_pkt_status_t.rssi_avg_in_dbm
 *     - lr20xx_radio_wm_bus_pkt_status_t.rssi_avg_half_dbm_count
 *     - lr20xx_radio_wm_bus_pkt_status_t.l_field
 *     - lr20xx_radio_wm_bus_pkt_status_t.crc_errors
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status Wireless M-Bus packet status structure
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_wm_bus_get_pkt_status( const void* context, lr20xx_radio_wm_bus_pkt_status_t* pkt_status );

/**
 * @brief Set the Wireless M-Bus address for Rx filtering
 *
 * @remark This value is taken into account only if enabled when calling @ref lr20xx_radio_wm_bus_set_params
 *
 * @param[in] context Chip implementation context
 * @param[in] addr Wireless M-Bus address
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_wm_bus_set_addr( const void* context, const uint8_t addr[LR20XX_RADIO_WM_BUS_ADDR_LEN] );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_WM_BUS_H

/* --- EOF ------------------------------------------------------------------ */
