/*!
 * @file      lr20xx_workarounds.h
 *
 * @brief     System driver workarounds definition for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#ifndef LR20XX_WORKAROUNDS_H
#define LR20XX_WORKAROUNDS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "lr20xx_status.h"
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
 * @brief Apply workaround for syncwords usage with BLE LE coded PHY
 *
 * This workaround is to be applied after configuring Bluetooth LE modulation, if phy @ref
 * LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_125KB or @ref LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_CODED_500KB are configured.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @see lr20xx_radio_bluetooth_le_set_modulation_params
 */
lr20xx_status_t lr20xx_workarounds_bluetooth_le_phy_coded_syncwords( const void* context );

/**
 * @brief Enable LoRa compatibility mode with SX1276
 *
 * If the SX1276 LoRa compatibility is required, this workaround must be called after calling @ref
 * lr20xx_radio_lora_set_modulation_params.
 *
 * SX1276 LoRa compatibility mode allows:
 *   - transmission to, and reception from, SX1276 LoRa packets at SF6 only in implicit mode (@ref
 * LR20XX_RADIO_LORA_PKT_IMPLICIT); and
 *   - syncword nibbles greater than 7 for all SF.
 *
 * @param context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_workarounds_lora_disable_sx1276_compatibility_mode,
 * lr20xx_workarounds_lora_sx1276_compatibility_mode_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_lora_enable_sx1276_compatibility_mode( const void* context );

/**
 * @brief Disable the LoRa compatibility mode with SX1276
 *
 * To disable the SX1276 LoRa compatibility mode, this workaround can be call either before or after @ref
 * lr20xx_radio_lora_set_modulation_params.
 *
 * @param context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_workarounds_lora_enable_sx1276_compatibility_mode,
 * lr20xx_workarounds_lora_sx1276_compatibility_mode_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_lora_disable_sx1276_compatibility_mode( const void* context );

/**
 * @brief Store the LoRa SX1276 compatibility mode in retention memory
 *
 * Calling this function allows to store the SX1276 LoRa compatible state during sleep mode.
 * This helper function internally calls @ref lr20xx_system_add_register_to_retention_mem with the appropriate register
 * address.
 *
 * @param context Chip implementation context
 * @param slot Index in the storage list. Allowed values [0:31]
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_lora_enable_sx1276_compatibility_mode,
 * lr20xx_workarounds_lora_disable_sx1276_compatibility_mode
 */
lr20xx_status_t lr20xx_workarounds_lora_sx1276_compatibility_mode_store_retention_mem( const void* context,
                                                                                       uint8_t     slot );

/**
 * @brief Enable the SX1276 compatibility mode for LoRa intra-packet frequency hopping
 *
 * If the LoRa intra-packet frequency hopping compatible with SX1276 is required, this function must be called after
 * @ref lr20xx_radio_lora_set_freq_hop.
 *
 * @param context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_radio_lora_set_freq_hop, lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatiblity_mode,
 * lr20xx_workarounds_lora_freq_hop_sx1276_compatiblity_mode_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatiblity_mode( const void* context );

/**
 * @brief Disable the SX1276 compatibility mode for LoRa intra-packet frequency hopping
 *
 * @param context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_radio_lora_set_freq_hop, lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatiblity_mode,
 * lr20xx_workarounds_lora_freq_hop_sx1276_compatiblity_mode_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatiblity_mode( const void* context );

/**
 * @brief Store the SX1276 compatibility mode for LoRa intra-packet frequency hopping in retention memory
 *
 * Calling this function allows to store the SX1276 LoRa intra-packet frequency hopping compatible state during sleep
 * mode. This helper function internally calls @ref lr20xx_system_add_register_to_retention_mem with the appropriate
 * register address.
 *
 * @param context Chip implementation context
 * @param slot Index in the storage list. Allowed values [0:31]
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatiblity_mode,
 * lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatiblity_mode
 */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_sx1276_compatiblity_mode_store_retention_mem( const void* context,
                                                                                               uint8_t     slot );

/**
 * @brief Override the OOK detection threshold level
 *
 * The OOK detection threshold level is automatically computed by the LR20xx depending on the modulation parameters.
 * However the computed value may be too conservative which increase the packet error rate.
 * The detection threshold level can be therefore modified with this function. The threshold to provide is typically the
 * noise level returned by @ref lr20xx_radio_common_get_rssi_inst using the same modulation parameters, if it is higher
 * than the LR20xx default computed value.
 *
 * Refer to @ref lr20xx_workarounds_ook_get_default_detection_threshold_level to obtain the default computed values
 * depending on modulation bandwidth.
 *
 * This function should be called after @ref lr20xx_radio_ook_set_modulation_params.
 *
 * @param context Chip implementation context
 * @param threshold_level_db The threshold level to set, in dB
 *
 * @return Operation status
 *
 * @see lr20xx_radio_ook_set_modulation_params, lr20xx_radio_common_get_rssi_inst,
 * lr20xx_workarounds_ook_get_default_detection_threshold_level
 */
lr20xx_status_t lr20xx_workarounds_ook_set_detection_threshold_level( const void* context, int16_t threshold_level_db );

/**
 * @brief Helper function that returns default OOK detection threshold level
 *
 * This helper function helps to determine if the workaround @ref lr20xx_workarounds_ook_set_detection_threshold_level
 * is to be applied.
 *
 * @param bw The bandwidth for which the detection threshold is to be computed
 *
 * @return The default OOK detection threshold level, or 0 if the bandwidth @p bw is unknown
 *
 * @see lr20xx_workarounds_ook_set_detection_threshold_level
 *
 */
int16_t lr20xx_workarounds_ook_get_default_detection_threshold_level( lr20xx_radio_fsk_common_bw_t bw );

/**
 * @brief Apply workaround to truncate internal PLL frequency step for RTToF operation
 *
 * Unexpected RTToF results may be obtained if the RF frequency is not set to a value multiple of 122Hz.
 * This workaround ensures internal RF frequency is configured to a multiple of 122Hz.
 *
 * This workaround must be applied after configuring the RF frequency of RTToF ranging operations with @ref
 * lr20xx_radio_common_set_rf_freq.
 * After applying the workaround, the RF frequency is therefore modified by a quantity inferior or equal to 122Hz
 * compared to the value set by last call to @ref lr20xx_radio_common_set_rf_freq.
 *
 * @param context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_radio_common_set_rf_freq
 */
lr20xx_status_t lr20xx_workarounds_rttof_truncate_pll_freq_step( const void* context );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_WORKAROUNDS_H

/* --- EOF ------------------------------------------------------------------ */