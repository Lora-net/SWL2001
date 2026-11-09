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
#include "lr20xx_radio_fifo_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#ifndef LR20XX_WORKAROUND_DISABLE_CLEAR_POST_TEMPERATURE_READ
#define LR20XX_WORKAROUND_CONDITIONNALY_APPLY_CLEAR_POST_TEMPERATURE_READ( cont ) \
    lr20xx_workaround_clear_post_temperature_read( cont )
#else
#define LR20XX_WORKAROUND_CONDITIONNALY_APPLY_CLEAR_POST_TEMPERATURE_READ( cont ) LR20XX_STATUS_OK
#endif

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
 * @see lr20xx_radio_lora_set_freq_hop, lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatibility_mode,
 * lr20xx_workarounds_lora_freq_hop_sx1276_compatibility_mode_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatibility_mode( const void* context );

/**
 * @brief Disable the SX1276 compatibility mode for LoRa intra-packet frequency hopping
 *
 * @param context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_radio_lora_set_freq_hop, lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatibility_mode,
 * lr20xx_workarounds_lora_freq_hop_sx1276_compatibility_mode_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatibility_mode( const void* context );

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
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatibility_mode,
 * lr20xx_workarounds_lora_freq_hop_disable_sx1276_compatibility_mode
 */
lr20xx_status_t lr20xx_workarounds_lora_freq_hop_sx1276_compatibility_mode_store_retention_mem( const void* context,
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
 * @brief Apply workaround to reduce standard deviation of RTToF results with fractional bandwidths
 *
 * This workaround reduces the standard deviation of observed RTToF result on the following bandwiths:
 * - @ref LR20XX_RADIO_LORA_BW_812
 * - @ref LR20XX_RADIO_LORA_BW_406
 * - @ref LR20XX_RADIO_LORA_BW_203
 * - @ref LR20XX_RADIO_LORA_BW_101
 * The workaround must be called only on these bandwidths, after calling @ref lr20xx_radio_lora_set_modulation_params.
 *
 * Note that a call to @ref lr20xx_radio_lora_set_modulation_params reset the changes executed by this workaround.
 *
 * @param context Chip implementation context
 * @param is_manager True if the device operate as manager, false if it operates as subordinate
 *
 * @return Operation status
 *
 * @see lr20xx_radio_lora_set_modulation_params
 */
lr20xx_status_t lr20xx_workarounds_rttof_results_deviation( const void* context, bool is_manager );

/**
 * @brief Store the registers for RTToF results deviation workaround in retention memory
 *
 * Calling this function allows to store the RTToF results deviation workaround registers during sleep mode. This helper
 * function internally calls @ref lr20xx_system_add_register_to_retention_mem with the appropriate register address.
 *
 * The @ref lr20xx_workarounds_rttof_results_deviation workaround addresses two registers, hence the two configurable
 * slots.
 *
 * @param context Chip implementation context
 * @param slot_1 Index in the storage list. Allowed values [0:31]
 * @param slot_2 Index in the storage list. Allowed values [0:31]
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_rttof_results_deviation
 */
lr20xx_status_t lr20xx_workarounds_rttof_results_deviation_store_retention_mem( const void* context, uint8_t slot_1,
                                                                                uint8_t slot_2 );

/*!
 * @brief Workaround helper to configure FIFO events and threshold levels with 1024-byte Tx/Rx FiFos
 *
 * This workaround wraps @ref lr20xx_radio_fifo_cfg_irq when using 1024-byte Rx/Tx FiFos to allow settings @p
 * tx_fifo_low_threshold and @p rx_fifo_low_threshold to value superior to 256.
 *
 * This workaround configures registers that are not maintained in memory during sleep mode. Call @ref
 * lr20xx_workarounds_1024_byte_fifo_cfg_irq_store_retention_mem to register these registers as maintained during sleep
 * mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_fifo_irq_enable FIFO events triggering an interrupt in Rx
 * @param [in] tx_fifo_irq_enable FIFO events triggering an interrupt in Tx
 * @param [in] rx_fifo_high_threshold Rx FIFO threshold above which an interrupt (if
 * LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH is enabled) is triggered
 * @param [in] tx_fifo_low_threshold Tx FIFO threshold below which an interrupt (if LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW
 * is enabled) is triggered
 * @param [in] rx_fifo_low_threshold Rx FIFO threshold below which an interrupt (if LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW
 * is enabled) is triggered
 * @param [in] tx_fifo_high_threshold Tx FIFO threshold above which an interrupt (if
 * LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH is enabled) is triggered
 *
 * @returns Operation status
 *
 * @see lr20xx_radio_fifo_cfg_irq, lr20xx_radio_fifo_configure_1024_byte_tx_fifo,
 * lr20xx_radio_fifo_configure_1024_byte_rx_fifo
 */
lr20xx_status_t lr20xx_workarounds_1024_byte_fifo_cfg_irq(
    const void* context, lr20xx_radio_fifo_flag_t rx_fifo_irq_enable, lr20xx_radio_fifo_flag_t tx_fifo_irq_enable,
    uint16_t rx_fifo_high_threshold, uint16_t tx_fifo_low_threshold, uint16_t rx_fifo_low_threshold,
    uint16_t tx_fifo_high_threshold );

/**
 * @brief Store the registers to configure the 1024 bytes Tx/Rx FiFos threshold related IRQs in retention memory
 *
 * Calling this function allows to store the workaround registers for IRQ FiFo configuration with 1024-byte Tx/Rx FiFos
 * during sleep mode. This helper function internally calls @ref lr20xx_system_add_register_to_retention_mem with the
 * appropriate register address.
 *
 * @param [in] context Chip implementation context
 * @param [in] retention_slot_rx_thresholds Retention memory slot to store Rx FiFo thresholds
 * @param [in] retention_slot_tx_thresholds Retention memory slot to store Tx FiFo thresholds
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_1024_byte_fifo_cfg_irq
 */
lr20xx_status_t lr20xx_workarounds_1024_byte_fifo_cfg_irq_store_retention_mem( const void* context,
                                                                               uint8_t     retention_slot_rx_thresholds,
                                                                               uint8_t retention_slot_tx_thresholds );

/**
 * @brief Disable the LoRa Rx length check
 *
 * The payload length configured in @ref lr20xx_radio_lora_pkt_params_s::pld_len_in_bytes is used by the chip as both
 * the maximal payload length of received packets and the length of trasmitted packets.
 * This workaround disables the Rx length check.
 * It can typically be used when AutoTxRx feature is enabled as the same payload length configuration is used in the
 * transmission and reception phase.
 *
 * This workaround is reset after a call to @ref lr20xx_radio_lora_set_packet_params. Therefore it must be applied
 * after calling @ref lr20xx_radio_lora_set_packet_params.
 *
 * This workaround is not maintained when entering sleep mode with retention. To maintain it, it must be stored in
 * retention memory with @ref lr20xx_workarounds_lora_disable_rx_length_check_store_retention_mem.
 *
 * @param [in] context Chip implementation context
 *
 * @return Operation status
 *
 * @see lr20xx_workarounds_lora_disable_rx_length_check_store_retention_mem, lr20xx_radio_lora_set_packet_params,
 * lr20xx_radio_common_configure_auto_tx_rx
 *
 */
lr20xx_status_t lr20xx_workarounds_lora_disable_rx_length_check( const void* context );

/**
 * @brief Store the disabling of LoRa Rx length check in retention memory
 *
 * Calling this function allows to store the disabling of Lora Rx Length check during sleep mode. This helper function
 * internally calls @ref lr20xx_system_add_register_to_retention_mem with the appropriate register address.
 *
 * @param context Chip implementation context
 * @param slot Index in the storage list. Allowed values [0:31]
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_lora_disable_rx_length_check
 */
lr20xx_status_t lr20xx_workarounds_lora_disable_rx_length_check_store_retention_mem( const void* context,
                                                                                     uint8_t     slot );

/**
 * @brief Enable/Disable double rate sampling for FLRC
 *
 * This function must be called after @ref lr20xx_radio_flrc_set_modulation_params.
 * It is typically used to enable double sampling rate for FLRC with datarate inferior or equal to 2.05Mbps
 * to avoid low power spurs.
 *
 * @param [in] context Chip implementation context
 * @param [in] enable True to enable the double sampling rate. False to disable it.
 *
 * @return Operation status
 *
 * @see lr20xx_radio_flrc_set_modulation_params,
 * lr20xx_workarounds_flrc_configure_double_rate_sampling_store_retention_mem
 */
lr20xx_status_t lr20xx_workarounds_flrc_configure_double_rate_sampling( const void* context, bool enable );

/**
 * @brief Store the configuration of FLRC double rate sampling in retention memory
 *
 * Calling this function allows to store the configuration of FLRC double rate sampling during sleep mode.
 * This helper function internally calls @ref lr20xx_system_add_register_to_retention_mem with the appropriate
 * register address.
 *
 * @param context Chip implementation context
 * @param slot Index in the storage list. Allowed values [0:31]
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_flrc_configure_double_rate_sampling
 */
lr20xx_status_t lr20xx_workarounds_flrc_configure_double_rate_sampling_store_retention_mem( const void* context,
                                                                                            uint8_t     slot );

/**
 * @brief Disable FLRC IF correlators
 *
 * Disabling the intermediate frequency FLRC correlators allows to reduce FLRC reception power consumption, and to
 * reduce the probability of false detection.
 * However the frequency error with transmitter must have been compensated.
 *
 * To re-enable the FLRC IF correlators, the command @ref lr20xx_radio_flrc_set_modulation_params has to be
 * called.
 *
 * This configuration is reset by a call to @ref lr20xx_radio_flrc_set_modulation_params, so this function must
 * be called after @ref lr20xx_radio_flrc_set_modulation_params.
 *
 * @param context Chip implementation context
 * @return Operation status
 *
 * @see lr20xx_regmem_write_regmem32, lr20xx_radio_flrc_set_modulation_params
 */
lr20xx_status_t lr20xx_workarounds_flrc_disable_if_correlators( const void* context );

/**
 * @brief Store the configuration of FLRC IF correlators workaround in retention memory
 *
 * Calling this function allows to store the configuration of FLRC IF correlators during sleep mode.
 * This helper function internally calls @ref lr20xx_system_add_register_to_retention_mem with the appropriate
 * register address.
 *
 * @param context Chip implementation context
 * @param slot Index in the storage list. Allowed values [0:31]
 *
 * @return Operation status
 *
 * @see lr20xx_system_add_register_to_retention_mem, lr20xx_workarounds_flrc_disable_if_correlators
 */
lr20xx_status_t lr20xx_workarounds_flrc_disable_if_correlators_store_retention_mem( const void* context, uint8_t slot );

/**
 * @brief Clear internal state after reading temperature value
 *
 * If @ref lr20xx_system_get_temp is called with @p src being @ref LR20XX_SYSTEM_TEMP_SRC_VBE or @ref
 * LR20XX_SYSTEM_TEMP_SRC_XOSC, an internal state is not reset so that next call to @ref lr20xx_system_get_vbat returns
 * an incorrect value.
 *
 * This workaround reset the appropriate internal state.
 *
 * @param context Chip implementation context
 * @return Operation status
 *
 * @see lr20xx_system_get_temp, lr20xx_system_get_vbat
 */
lr20xx_status_t lr20xx_workaround_clear_post_temperature_read( const void* context );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_WORKAROUNDS_H

/* --- EOF ------------------------------------------------------------------ */