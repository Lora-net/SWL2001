/*!
 * @file      lr20xx_radio_lora.h
 *
 * @brief     Radio LoRa driver definition for LR20XX
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

#ifndef LR20XX_RADIO_LORA_H
#define LR20XX_RADIO_LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"
#include "lr20xx_radio_lora_types.h"

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
 * @brief Set the modulation parameters for LoRa packets
 *
 * @param[in] context Chip implementation context
 * @param[in] mod_params Structure of LoRa modulation configuration
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_lora_get_recommended_ppm_offset
 */
lr20xx_status_t lr20xx_radio_lora_set_modulation_params( const void*                           context,
                                                         const lr20xx_radio_lora_mod_params_t* mod_params );

/**
 * @brief Set the packet parameters for LoRa packets
 *
 * The meaning of field pkt_params->pld_len_in_bytes depends on the packet mode selected:
 *   - If LR20XX_RADIO_LORA_PKT_EXPLICIT:
 *     - pld_len_in_bytes = 0 means that packets of all payload length will be accepted
 *     - pld_len_in_bytes > 0 means that packet with payload length in range [1:pld_len_in_bytes] will be accepted.
 *       Packet of payload length equals to 0 or strictly superior to pld_len_in_bytes will be rejected with IRQ
 *       LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR
 *
 * @param[in] context Chip implementation context
 * @param[in] pkt_params Structure of LoRa packet configuration
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_lora_set_packet_params( const void*                           context,
                                                     const lr20xx_radio_lora_pkt_params_t* pkt_params );

/**
 * @brief Configure a timeout given in number of LoRa symbols before stopping reception if no LoRa preamble symbols are
 * detected
 *
 * A timeout interrupt is triggered if no LoRa preamble symbol is detected during the given period.
 *
 * Setting @p n_symbols to 0 disables the mechanism.
 *
 * @param[in] context Chip implementation context
 * @param[in] n_symbols The number of symbols to search for.
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols
 */
lr20xx_status_t lr20xx_radio_lora_configure_timeout_by_number_of_symbols( const void* context, uint8_t n_symbols );

/**
 * @brief Configure a timeout given in number of LoRa symbols before stopping reception if no LoRa preamble symbols are
 * detected
 *
 * A timeout interrupt is triggered if no LoRa preamble symbol is detected during the given period.
 *
 * The number of symbol is computed as \f$ N_{symbols} = mantissa ^ {2 \times exponent + 1} \f$
 *
 * Setting @p mantissa and @p exponent to get a number of symbol equal to 0 disables the mechanism.
 *
 * @param[in] context Chip implementation context
 * @param[in] mantissa Mantissa - from 0 to 31 - to compute the number of symbols
 * @param[in] exponent Exponent - from 0 to 7 - to compute the number of symbols
 *
 * @return lr20xx_status_t
 *
 * @see lr20xx_radio_lora_configure_timeout_by_number_of_symbols
 */
lr20xx_status_t lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols( const void* context, uint8_t mantissa,
                                                                                  uint8_t exponent );

/**
 * @brief Configure the LoRa syncword.
 *
 * Default value is 0x12.
 * Example of typical values:
 *   - LoRaWAN public network: 0x34
 *   - LoRaWAN private network: 0x12
 *
 * The syncword here should be understood as the concatenation of two 4 bits blocks as follows:
 * @code{.c}
 * uint8_t sync_block_1 = BLOCK_1;
 * uint8_t sync_block_2 = BLOCK_2;
 * uint8_t syncword = ((sync_block_1 & 0x0F) << 4) | (sync_block_2 & 0x0F);
 * @endcode
 *
 * Here are some recommendations for syncword selection:
 *   - @p sync_block_1 must not be 0. So that syncword 0x0x must not be used;
 *   - avoid reusing a block value from another network
 *
 * Note that using different syncwords does not guarantee packet rejection. Receiver is just less likely to accept frame
 * of different syncword.
 *
 * The following table indicates the compatible block values with other chips. Note that the block values are to be
 * compared as signed integer when evaluating compatibility.
 * A line indicates a set of values that are compatible together depending on other chips.
 * Column <tt>SX126x/LR11xx/LR20xx syncword</tt> indicates block values used
 * with @ref lr20xx_radio_lora_set_syncword function and SX1276 LoRa compatibility disabled (@ref
 * lr20xx_workarounds_lora_disable_sx1276_compatibility_mode); the column <tt>LR20xx syncword SX127x compatibility</tt>
 * indicates block values used with @ref lr20xx_radio_lora_set_syncword and with SX1276 LoRa compatibility enabled
 * (@ref lr20xx_workarounds_lora_enable_sx1276_compatibility_mode).
 *
 * | SX126x/LR11xx/LR20xx syncword | LR20xx syncword SX127x compatibility | SX127x          |
 * | ----------------------------- | ------------------------------------ | --------------- |
 * | 4 bits signed                 | 4 bits unsigned                      | 4 bits unsigned |
 * | -8                            |                                      |                 |
 * | -7                            |                                      |                 |
 * | -6                            |                                      |                 |
 * | -5                            |                                      |                 |
 * | -4                            |                                      |                 |
 * | -3                            |                                      |                 |
 * | -2                            |                                      |                 |
 * | -1                            |                                      |                 |
 * | 0                             | 0                                    | 0               |
 * | 1                             | 1                                    | 1               |
 * | 2                             | 2                                    | 2               |
 * | 3                             | 3                                    | 3               |
 * | 4                             | 4                                    | 4               |
 * | 5                             | 5                                    | 5               |
 * | 6                             | 6                                    | 6               |
 * | 7                             | 7                                    | 7               |
 * |                               | 8                                    | 8               |
 * |                               | 9                                    | 9               |
 * |                               | 10                                   | 10              |
 * |                               | 11                                   | 11              |
 * |                               | 12                                   | 12              |
 * |                               | 13                                   | 13              |
 * |                               | 14                                   | 14              |
 * |                               | 15                                   | 15              |
 *
 * @param[in] context Chip implementation context
 * @param[in] syncword The syncword to configure
 *
 * @return lr20xx_status_t Operation status
 *
 * @see LR20XX_RADIO_LORA_SYNCWORD_LORAWAN_PUBLIC_NETWORK,
 * LR20XX_RADIO_LORA_SYNCWORD_LORAWAN_PRIVATE_NETWORK
 */
lr20xx_status_t lr20xx_radio_lora_set_syncword( const void* context, uint8_t syncword );

/**
 * @brief Configure the Channel Activity Detection (CAD) operation
 *
 * @param[in] context Chip implementation context
 * @param[in] cad_params Structure of CAD parameter configuration
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_lora_set_cad
 */
lr20xx_status_t lr20xx_radio_lora_configure_cad_params( const void*                           context,
                                                        const lr20xx_radio_lora_cad_params_t* cad_params );

/**
 * @brief Start Channel Activity Detection (CAD) operation
 *
 * The CAD operation is a special mode of operation where the chip is looking for the presence of LoRa preamble symbols
 * or for any Lora signal, depending on the setting in the @ref lr20xx_radio_lora_configure_cad_params command.
 *
 * At the end of the CAD operation a LR20XX_SYSTEM_IRQ_CAD_DONE is generated. If the CAD operation detects a signal, it
 * also generates a LR20XX_SYSTEM_IRQ_CAD_DETECTED.
 *
 * Depending on the CAD configuration, the chip may either go back to the configured fallback mode, or enter the
 * configured exit mode.
 *
 * If the exit mode is a radio operation the corresponding IRQ the CAD related IRQ(s) comes at the end CAD operation,
 * and radio operations IRQ(s) of exit modes comes at the end of this radio operation.
 *
 * @param[in] context Chip implementation context
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_radio_lora_configure_cad_params
 */
lr20xx_status_t lr20xx_radio_lora_set_cad( const void* context );

/**
 * @brief Get the internal statistics of received packets
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
lr20xx_status_t lr20xx_radio_lora_get_rx_statistics( const void*                        context,
                                                     lr20xx_radio_lora_rx_statistics_t* statistics );

/**
 * @brief Get the status of the last received LoRa packet
 *
 * Status is valid only after the end of a packet reception or CAD done and until the next LoRa packet configuration.
 *
 * CRC and coding rate source depends on the packet mode configured on the receiver:
 *   - If LR20XX_RADIO_LORA_PKT_EXPLICIT: it is obtained from the received payload
 *   - If LR20XX_RADIO_LORA_PKT_IMPLICIT: it is obtained from the receiver configuration
 *
 * @param[in] context Chip implementation context
 * @param[out] pkt_status Pointer to a structure of packet status to populate
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_lora_get_packet_status( const void*                        context,
                                                     lr20xx_radio_lora_packet_status_t* pkt_status );

/**
 * @brief Set the address for filtering in reception
 *
 * @param[in] context Chip implementation context
 * @param[in] address_offset Offset in byte of the address field in the payload (header not counted)
 * @param[in] address_length Address length in byte - in [0:8], 0 disables LoRa address filtering
 * @param[in] address Address
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_lora_set_address( const void* context, uint8_t address_offset, uint8_t address_length,
                                               const uint8_t* address );

/**
 * @brief Configure LoRa intra-packet frequency hopping
 *
 * If the intra-packet frequency hopping must be compatible with SX1276, then the workaround @ref
 * lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatiblity_mode must be called after calling @ref
 * lr20xx_radio_lora_set_freq_hop.
 *
 * @param[in] context Chip implementation context
 * @param[in] cfg Frequency hopping configuration
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_workarounds_lora_freq_hop_enable_sx1276_compatiblity_mode
 */
lr20xx_status_t lr20xx_radio_lora_set_freq_hop( const void* context, const lr20xx_radio_lora_hopping_cfg_t* cfg );

/**
 * @brief Configure the LoRa Channel Activity Detection (CAD) side detectors
 *
 * Up to three CAD side detectors can be configured.
 *
 * @param context Chip implementation context
 * @param side_detector_cad_configurations Array of side detector CAD configurations
 * @param n_side_detector_cad_configurations Number of CAD side detector configurations in @p
 * side_detector_cad_configurations. It is up to the caller to ensure that @p side_detector_cad_configurations contains
 * at least @p n_side_detector_cad_configurations elements
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_lora_configure_side_detector_cad(
    const void* context, const lr20xx_radio_lora_side_detector_cad_configuration_t* side_detector_cad_configurations,
    uint8_t n_side_detector_cad_configurations );

/**
 * @brief Configure LoRa side detectors
 *
 * The side detectors allow to receive on multiple spreading factors, but on the same bandwidth as main detector. Up to
 * three side detectors can be configured.
 *
 * To disable all side detectors, there are 2 options:
 *   - call this command with @p n_side_detector_cfgs set to 0.
 *   - call @ref lr20xx_radio_lora_set_modulation_params
 *
 * Once a packet is received, it is possible to know which SF has been demodulated thanks to @ref
 * lr20xx_radio_lora_get_packet_status.
 *
 * Specificities related to the side detector configuration:
 *   - For normal Rx operations, the SF configured with @ref lr20xx_radio_lora_set_modulation_params must be lower than
 * the SF of the side detectors
 *   - For CAD operations, the SF configured with @ref lr20xx_radio_lora_set_modulation_params must be higher than the
 * SF of the side detectors
 *   - With BW set to @ref LR20XX_RADIO_LORA_BW_500 or higher, maximum 2 side detectors are allowed except if the SF
 * configured with @ref lr20xx_radio_lora_set_modulation_params is @ref LR20XX_RADIO_LORA_SF10 or higher where only 1
 * side detector is allowed
 *   - All SF must be different
 *   - Difference between the highest and the lowest SF must be less than or equal to 4
 *
 * @param[in] context Chip implementation context
 * @param[in] side_detector_cfgs Array of side detector configuration to set. It is up to the caller to ensure
 * there are at least @p n_side_detector_cfgs
 * @param[in] n_side_detector_cfgs Number of side detector to configure. Un-configured side detectors are
 * disabled. Value must be in range [0:3] included.
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_lora_configure_side_detectors(
    const void* context, const lr20xx_radio_lora_side_detector_cfg_t* side_detector_cfgs,
    uint8_t n_side_detector_cfgs );

/**
 * @brief Configure the LoRa syncwords for side detectors
 *
 * @param[in] context Chip implementation context
 * @param[in] syncword Array of side detector syncword to set. It is up to the caller to ensure there are at least @p
 * n_syncword
 * @param[in] n_syncword Number of side detector syncword configure. Un-configured syncword are set to a default value.
 * Value must be in range [0:3] included.
 *
 * @return lr20xx_status_t Operation status
 */
lr20xx_status_t lr20xx_radio_lora_set_side_detector_syncwords( const void* context, const uint8_t* syncword,
                                                               uint8_t n_syncword );

/**
 * @brief Compute the numerator for LoRa time-on-air computation.
 *
 * @remark To get the actual time-on-air in seconds, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * @param [in] pkt_p Pointer to the structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to the structure holding the LoRa modulation parameters
 *
 * @returns LoRa time-on-air numerator
 */
uint32_t lr20xx_radio_lora_get_time_on_air_numerator( const lr20xx_radio_lora_pkt_params_t* pkt_p,
                                                      const lr20xx_radio_lora_mod_params_t* mod_p );

/**
 * @brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * @param [in] bw LoRa bandwidth parameter
 *
 * @returns Actual LoRa bandwidth in Hertz
 */
uint32_t lr20xx_radio_lora_get_bw_in_hz( lr20xx_radio_lora_bw_t bw );

/*!
 * @brief Get the time on air in ms for LoRa transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to a structure holding the LoRa modulation parameters
 *
 * @returns Time-on-air value in ms for LoRa transmission
 */
uint32_t lr20xx_radio_lora_get_time_on_air_in_ms( const lr20xx_radio_lora_pkt_params_t* pkt_p,
                                                  const lr20xx_radio_lora_mod_params_t* mod_p );

/**
 * @brief Helper function to compute recommended ppm offset value from SF and BW
 *
 * This helper function provides recommended PPM offset configuration based on the following rules:asm
 *   - @ref LR20XX_RADIO_LORA_NO_PPM for all spreading factors, except for @ref LR20XX_RADIO_LORA_SF11 and @ref
 * LR20XX_RADIO_LORA_SF12
 *   - @ref LR20XX_RADIO_LORA_PPM_1_4 for bandwidths @ref LR20XX_RADIO_LORA_BW_812, @ref LR20XX_RADIO_LORA_BW_406 and
 * @ref LR20XX_RADIO_LORA_BW_203 to ensure SX128x compatibility
 *   - @ref LR20XX_RADIO_LORA_NO_PPM for bandwidths @ref LR20XX_RADIO_LORA_BW_1000 and @ref LR20XX_RADIO_LORA_BW_500
 *   - @ref LR20XX_RADIO_LORA_NO_PPM for bandwidth @ref LR20XX_RADIO_LORA_BW_250 and spreading factor @ref
 * LR20XX_RADIO_LORA_SF11
 * *   - @ref LR20XX_RADIO_LORA_PPM_1_4 for bandwidth @ref LR20XX_RADIO_LORA_BW_250 and spreading factor @ref
 * LR20XX_RADIO_LORA_SF12
 *   - @ref LR20XX_RADIO_LORA_PPM_1_4 otherwise
 *
 * | Bandwidths | @ref LR20XX_RADIO_LORA_SF12 | @ref LR20XX_RADIO_LORA_SF11 | other spreading factors |
 * | -- | -- | -- | -- |
 * | @ref LR20XX_RADIO_LORA_BW_1000 | @ref LR20XX_RADIO_LORA_NO_PPM |||
 * | @ref LR20XX_RADIO_LORA_BW_500 | @ref LR20XX_RADIO_LORA_NO_PPM |||
 * | @ref LR20XX_RADIO_LORA_BW_250 | @ref LR20XX_RADIO_LORA_PPM_1_4 | @ref LR20XX_RADIO_LORA_NO_PPM ||
 * | @ref LR20XX_RADIO_LORA_BW_812 | @ref LR20XX_RADIO_LORA_PPM_1_4 || @ref LR20XX_RADIO_LORA_NO_PPM |
 * | @ref LR20XX_RADIO_LORA_BW_406 | @ref LR20XX_RADIO_LORA_PPM_1_4 || @ref LR20XX_RADIO_LORA_NO_PPM |
 * | @ref LR20XX_RADIO_LORA_BW_203 | @ref LR20XX_RADIO_LORA_PPM_1_4 || @ref LR20XX_RADIO_LORA_NO_PPM |
 * | other bandwidths | @ref LR20XX_RADIO_LORA_PPM_1_4 || @ref LR20XX_RADIO_LORA_NO_PPM |
 *
 * @param sf Spreading factor
 * @param bw Bandwidth
 *
 * @return The recommended PPM offset configuration for the given spreading factor and bandwidth
 *
 * @see lr20xx_radio_lora_set_modulation_params
 */
lr20xx_radio_lora_ppm_t lr20xx_radio_lora_get_recommended_ppm_offset( lr20xx_radio_lora_sf_t sf,
                                                                      lr20xx_radio_lora_bw_t bw );
#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_LORA_H

/* --- EOF ------------------------------------------------------------------ */
