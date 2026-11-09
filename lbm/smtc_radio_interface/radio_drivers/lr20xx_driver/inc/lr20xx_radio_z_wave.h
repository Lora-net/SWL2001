/*!
 * @file      lr20xx_radio_z_wave.h
 *
 * @brief     Z-Wave driver definition for LR20XX
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

#ifndef LR20XX_RADIO_Z_WAVE_H
#define LR20XX_RADIO_Z_WAVE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr20xx_radio_z_wave_types.h"
#include "lr20xx_status.h"

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
 * @brief Set the Z-Wave parameters
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[in] params Z-Wave parameters
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_z_wave_set_params( const void* context, const lr20xx_radio_z_wave_params_t* params );

/**
 * @brief Set the Z-Wave HomeID
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[in] homeid Z-Wave HomeID
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_z_wave_set_homeid( const void* context, const uint32_t homeid );

/**
 * @brief Get the internal statistics of received Z-Wave packets
 *
 * The internal statistics are reset on:
 *   - Power On Reset (POR)
 *   - sleep without memory retention
 *   - call to lr20xx_radio_common_reset_rx_stats
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[out] statistics Z-Wave received packet statistics
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_common_reset_rx_stats
 */
lr20xx_status_t lr20xx_radio_z_wave_get_rx_stats( const void* context, lr20xx_radio_z_wave_rx_stats_t* statistics );

/**
 * @brief Get the status of the last Z-Wave received packet
 *
 * Availability of the packet status fields depend on the IRQ as follows:
 * - Available from LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID:
 *     - lr20xx_radio_z_wave_pkt_status_t.rssi_sync_in_dbm
 *     - lr20xx_radio_z_wave_pkt_status_t.rssi_sync_half_dbm_count
 * - Available from LR20XX_SYSTEM_IRQ_RX_DONE:
 *     - lr20xx_radio_z_wave_pkt_status_t.rssi_avg_in_dbm
 *     - lr20xx_radio_z_wave_pkt_status_t.rssi_avg_half_dbm_count
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status Z-Wave packet status structure
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_z_wave_get_pkt_status( const void* context, lr20xx_radio_z_wave_pkt_status_t* pkt_status );

/**
 * @brief Set the Z-Wave beam filtering parameters
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[in] params Beam filtering parameters
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_z_wave_set_beam_filtering_params(
    const void* context, const lr20xx_radio_z_wave_beam_filtering_params_t* params );

/**
 * @brief Set the Z-Wave scan parameters
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[in] params Z-Wave scan parameters
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_z_wave_execute_scan_operation
 */
lr20xx_status_t lr20xx_radio_z_wave_set_scan_params( const void*                              context,
                                                     const lr20xx_radio_z_wave_scan_params_t* params );

/**
 * @brief Start Z-Wave scan operation
 *
 * @remark The configuration has to be applied beforehands by calling @ref lr20xx_radio_z_wave_set_scan_params.
 *
 * The Z-Wave scan sequence is composed of two operations:
 *   - configuration: the LR20xx computes internal Z-Wave scan configuration parameters
 *   - scan execution: the LR20xx actually executes the Z-Wave scanning activity
 *
 * In order to save time in succession SCAN / TX sequences, it is possible to executes once the scan configuration
 * before the sequences, then the SCAN / TX operations:
 *
 * SCAN CONFIGURATION / SCAN EXECUTION / TX / SCAN EXECUTION / TX / ...
 *
 * The configuration computed remains valid while:
 * - there is no change of packet type (with @ref lr20xx_radio_common_set_pkt_type)
 * - there is no change of boost mode configuration (with @ref lr20xx_radio_common_set_rx_path)
 * - there is no change of the frequency band (for instance switching from 490 MHz to 915 MHz)
 *
 * If one of the above parameter change, the scan configuration operation must be executed again.
 *
 * Note that to exit scan execution, the command @ref lr20xx_system_set_standby_mode must be called.
 *
 * The @p operation argument controls the scan sequence to execute.
 *
 * @note This command is not available to LR2022
 *
 * @param[in] context Chip implementation context
 * @param[in] operation Scan operation to execute
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_z_wave_set_scan_params
 */
lr20xx_status_t lr20xx_radio_z_wave_execute_scan_operation( const void*                          context,
                                                            lr20xx_radio_z_wave_scan_operation_t operation );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_Z_WAVE_H

/* --- EOF ------------------------------------------------------------------ */
