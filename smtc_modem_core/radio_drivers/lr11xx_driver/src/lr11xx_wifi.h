/*!
 * @file      lr11xx_wifi.h
 *
 * @brief     Wi-Fi passive scan driver definition for LR11XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#ifndef LR11XX_WIFI_H
#define LR11XX_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_regmem.h"
#include "lr11xx_wifi_types.h"
#include "lr11xx_types.h"
#include "lr11xx_system_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#ifndef LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK
/*!
 * @brief The number of results max to fetch per SPI communication with the chip
 *
 * This macro is used by the internals of the driver to size the internal
 * buffers of the driver used in the *read results* functions.
 *
 * It can be defined externally at compile time, or just before including this file.
 *
 * Its value can be programmatically obtained at runtime by calling lr11xx_wifi_get_nb_results_max_per_chunk() function.
 *
 * Its default value is set to the maximum number of results saved by LR11XX chip.
 *
 * @warning Its value must be in the range [1,32] (inclusive). Defining out of this range leads to undefined behavior.
 */
#define LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK LR11XX_WIFI_MAX_RESULTS
#endif  // LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Start a Wi-Fi passive scan operation
 *
 * During the complete passive scan operation, the LR11XX remains busy and cannot receive any commands. Using this
 * command **DOES** reset the results already obtained by previous passive scan operations.
 *
 * The result can be read at the end of the passive scan issuing the command lr11xx_wifi_get_nb_results (to get the
 * number of results to read) and lr11xx_wifi_read_basic_complete_results or
 * lr11xx_wifi_read_basic_mac_type_channel_results to actually get the result bytes.
 *
 * @param [in] context Chip implementation context
 * @param [in] signal_type The type of Wi-Fi Signals to scan for. If LR11XX_WIFI_TYPE_SCAN_B_G_N is selected, the LR11XX
 * already starts by scanning all selected channels for Wi-Fi signals B. Then the LR11XX scans all selected channels for
 * Wi-Fi signals G/N.
 * @param [in] channels Mask of the Wi-Fi channels to scan
 * @param [in] scan_mode Scan mode to execute
 * @param [in] max_results The maximal number of results to gather. When this limit is reached, the passive scan
 * automatically stop. Range of allowed values is [1:32]. Note that value 0 is forbidden.
 * @param [in] nb_scan_per_channel The number of internal scan sequences per channel scanned. Range of accepted values
 * is [1:255]. Note that value 0 is forbidden.
 * @param [in] timeout_in_ms The maximal duration of a single preamble search. Expressed in ms. Range of allowed values
 * is [1:65535]. Note that value 0 is forbidden.
 * @param [in] abort_on_timeout If true, the beacon search jumps to next configured Wi-Fi channel (or stop if there is
 * no more channel to scan) as soon as a search timeout is encountered
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_read_basic_complete_results, lr11xx_wifi_read_basic_mac_type_channel_results
 */
lr11xx_status_t lr11xx_wifi_scan( const void* context, const lr11xx_wifi_signal_type_scan_t signal_type,
                                  const lr11xx_wifi_channel_mask_t channels, const lr11xx_wifi_mode_t scan_mode,
                                  const uint8_t max_results, const uint8_t nb_scan_per_channel,
                                  const uint16_t timeout_in_ms, const bool abort_on_timeout );

/*!
 * @brief Start a Wi-Fi passive scan for country codes extraction
 *
 * This command starts a Wi-Fi passive scan operation for Beacons and Probe Responses on Wi-Fi type B only. It is to be
 * used to extract the Country Code fields.
 *
 * During the passive scan, the results are filtered to keep only single MAC addresses.
 *
 * @param [in] context Chip implementation context
 * @param [in] channels_mask Mask of the Wi-Fi channels to scan
 * @param [in] nb_max_results The maximum number of country code to gather. When this limit is reached, the passive scan
 * automatically stops. Maximal value is 32
 * @param [in] nb_scan_per_channel Maximal number of scan attempts per channel. Maximal value is 255
 * @param [in] timeout_in_ms The maximal duration of a single beacon search. Expressed in ms. Maximal value is 65535 ms
 * @param [in] abort_on_timeout If true, the beacon search jumps to next configured Wi-Fi channel (or stop if there is
 * no more channel to scan) as soon as a search timeout is encountered
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_search_country_code( const void* context, const lr11xx_wifi_channel_mask_t channels_mask,
                                                 const uint8_t nb_max_results, const uint8_t nb_scan_per_channel,
                                                 const uint16_t timeout_in_ms, const bool abort_on_timeout );

/*!
 * @brief Start a Wi-Fi passive scan operation with duration stop conditions
 *
 * This passive scan API does not require the number of scan per channel, so
 * that it searches for Wi-Fi signals until it finds one, or until the
 * exhaustion of timeout_per_scan_ms or timeout_per_channel_ms.
 *
 * The maximal duration of a scan is determined by the number of channels to scan times the timeout_per_channel_ms
 * configured. However, this duration may be exceeded depending on the crystal drift of the clock source and on the
 * instant the last Wi-Fi signal is detected by the device.
 * Therefore the maximal duration of a Wi-Fi scan with this API is provided by the following equations:
 *
 * For signal type being `LR11XX_WIFI_TYPE_SCAN_B`, `LR11XX_WIFI_TYPE_SCAN_G` or `LR11XX_WIFI_TYPE_SCAN_N`:
 *
 * \f$ T_{max} = N_{channel} \times ((1 + Xtal_{precision})timeout\_per\_channel + T_{offset} ) \f$
 *
 * \f$ Xtal_{precision} \f$ depends on the crystal used as clock source.
 * If the clock source is configured with 32kHz internal RC, then \f$ Xtal_{precision} = 1/100 \f$
 *
 * \f$ T_{offset} \f$ depends on the \f$ signal\_type \f$ and the \f$scan\_mode\f$ selected:
 *
 *   - LR11XX_WIFI_TYPE_SCAN_B:
 *     - if \f$scan\_mode != LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$: 2.31 ms
 *     - if \f$scan\_mode == LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$: 9.59 ms
 *   - LR11XX_WIFI_TYPE_SCAN_G:
 *     - if \f$scan\_mode != LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$: 52.55 ms
 *     - if \f$scan\_mode == LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$: N/A
 *
 * For signal type being `LR11XX_WIFI_TYPE_SCAN_B_G_N`:
 *
 * \f$ T_{max} = 2 \times N_{channel} \times (1 + Xtal_{precision})timeout\_per\_channel + T_{offset} \f$
 *
 * \f$ T_{offset} \f$ depends on the \f$scan\_mode\f$ selected:
 * - \f$scan\_mode != LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$: 54.86 ms
 * - \f$scan\_mode == LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$: 9.59 ms.
 *
 * @note With \f$scan\_mode != LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$ the T_offset is actually the worst case of
 * Wi-Fi type B and Wi-Fi type G/N. Moreover, the Wi-Fi types G and N are scanned within the same steps (it is not two
 * different scans). So the T_offset is the addition of 2.31 + 52.55 = 54.86.
 *
 * @note With \f$scan\_mode == LR11XX\_WIFI\_SCAN\_MODE\_FULL\_BEACON\f$, only Wi-Fi types B can be scanned. So scans
 * for Wi-Fi types G/N are silently discarded. Therefore the T_offset is the same as for scan with Wi-Fi type B.
 *
 * @param [in] context Chip implementation context
 * @param [in] signal_type The type of Wi-Fi Signals to scan for. If LR11XX_WIFI_TYPE_SCAN_B_G_N is selected, the LR11XX
 * already starts by scanning all selected channels for Wi-Fi signals B. Then the LR11XX scans all selected channels for
 * Wi-Fi signals G/N.
 * @param [in] channels Mask of the Wi-Fi channels to scan
 * @param [in] scan_mode Scan mode to execute
 * @param [in] max_results The maximal number of results to gather. When this
 * limit is reached, the passive scan automatically stop. Maximal value is 32
 * @param [in] timeout_per_channel_ms The time to spend scanning one channel. Expressed in ms. Value 0 is forbidden and
 * will result in the raise of WIFI_SCAN_DONE interrupt, with stat1.command_status being set to
 * LR11XX_SYSTEM_CMD_STATUS_PERR
 * @param [in] timeout_per_scan_ms The maximal time to spend in preamble detection for each single scan. The time spent
 * on preamble search is reset at each new preamble search. If the time spent on preamble search reach this timeout, the
 * scan on the current channel stops and start on next channel. If set to 0, the command will keep listening until
 * exhaustion of timeout_per_channel_ms or until nb_max_results is reached. Expressed in ms. Range of allowed values is
 * [0:65535].
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_read_basic_results, lr11xx_wifi_read_extended_results
 */
lr11xx_status_t lr11xx_wifi_scan_time_limit( const void* context, const lr11xx_wifi_signal_type_scan_t signal_type,
                                             const lr11xx_wifi_channel_mask_t channels,
                                             const lr11xx_wifi_mode_t scan_mode, const uint8_t max_results,
                                             const uint16_t timeout_per_channel_ms,
                                             const uint16_t timeout_per_scan_ms );

/*!
 * @brief Start a Wi-Fi passive scan for country codes extraction with duration stop conditions
 *
 * This command starts a Wi-Fi passive scan operation for Beacons and Probe Responses on Wi-Fi type B only. It is to be
 * used to extract the Country Code fields.
 * This passive scan API does not require the number of scan per channel, so that it searches for Wi-Fi signals until it
 * finds one, or until the exhaustion of timeout_per_scan_ms or timeout_per_channel_ms.
 *
 * The maximal duration of a scan is determined by the number of channels to scan times the timeout_per_channel_ms
 * configured. However, this duration may be exceeded depending on the crystal drift of the clock source and on the
 * instant the last Wi-Fi signal is detected by the device.
 * Therefore the maximal duration of a Wi-Fi scan with this API is provided by the following equation:
 *
 * \f$ T_{max} = N_{channel} \times ((1 + Xtal_{precision})timeout\_per\_channel + T_{offset} ) \f$
 *
 * \f$ Xtal_{precision} \f$ depends on the crystal used as clock source.
 * If the clock source is configured with 32kHz internal RC, then \f$ Xtal_{precision} = 1/100 \f$
 *
 * \f$ T_{offset} \f$ is always the same: 9.59 ms.
 *
 * @param [in] context Chip implementation context
 * @param [in] channels_mask Mask of the Wi-Fi channels to scan
 * @param [in] nb_max_results The maximum number of country code to gather. When this limit is reached, the passive scan
 * automatically stops. Maximal value is 32
 * @param [in] timeout_per_channel_ms The time to spend scanning one channel. Expressed in ms. Value 0 is forbidden and
 * will result in the raise of WIFI_SCAN_DONE interrupt, with stat1.command_status being set to
 * LR11XX_SYSTEM_CMD_STATUS_PERR
 * @param [in] timeout_per_scan_ms The maximal time to spend in preamble detection for each single scan. The time spent
 * on preamble search is reset at each new preamble search. If the time spent on preamble search reach this timeout, the
 * scan on the current channel stops and start on next channel. If set to 0, the command will keep listening until
 * exhaustion of timeout_per_channel_ms or until nb_max_results is reached. Expressed in ms. Range of allowed values is
 * [0:65535].
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_search_country_code_time_limit( const void*                      context,
                                                            const lr11xx_wifi_channel_mask_t channels_mask,
                                                            const uint8_t                    nb_max_results,
                                                            const uint16_t                   timeout_per_channel_ms,
                                                            const uint16_t                   timeout_per_scan_ms );

/*!
 * @brief Returns the number of results currently available in LR11XX
 *
 * It can be called before lr11xx_wifi_read_basic_complete_results or lr11xx_wifi_read_basic_mac_type_channel_results to
 * know the number of results.
 *
 * @param [in] context Chip implementation context
 * @param [out] nb_results The number of results available in the LR11XX
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_read_basic_complete_results, lr11xx_wifi_read_basic_mac_type_channel_results
 */
lr11xx_status_t lr11xx_wifi_get_nb_results( const void* context, uint8_t* nb_results );

/*!
 * @brief Read basic complete results
 *
 * This function can be used to fetch all results in a row, or one after the other.
 * It corresponds to result format @ref ::LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE.
 *
 * An example of usage to fetch all results in a row is:
 * \code{.cpp}
 * uint8_t nb_results = 0;
 * lr11xx_wifi_get_nb_results(&radio, &nb_results);
 * lr11xx_wifi_basic_complete_result_t all_results[LR11XX_WIFI_MAX_RESULTS] = {0};
 * lr11xx_wifi_read_basic_complete_results(&radio, 0, nb_results, all_results);
 * \endcode
 *
 * On the other hand, fetching result one after the other:
 * \code{.cpp}
 * uint8_t nb_results = 0;
 * lr11xx_wifi_get_nb_results(&radio, &nb_results);
 * lr11xx_wifi_basic_complete_result_t single_results = {0};
 * for(uint8_t index_result = 0; index_result < nb_results; index_result++){
 *   lr11xx_wifi_read_basic_complete_results(&radio, index_result, 1, &single_results);
 *   // Do something with single_results
 * }
 * \endcode
 *
 * @remark This result fetching function **MUST** be used only if the scan function call was made with Scan Mode set to
 * @ref ::LR11XX_WIFI_SCAN_MODE_BEACON or @ref ::LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT.
 * Refer to @ref lr11xx_wifi_are_scan_mode_result_format_compatible to know which scan mode and result format are
 * compatible.
 *
 * @param [in] radio Radio abstraction
 * @param [in] start_result_index Result index from which starting to fetch the results
 * @param [in] nb_results Number of results to fetch
 * @param [out] results Pointer to an array of result structures to populate. It is up to the caller to ensure this
 * array can hold at least nb_results elements.
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_are_scan_mode_result_format_compatible, lr11xx_wifi_read_basic_mac_type_channel_results,
 * lr11xx_wifi_read_extended_full_results
 */
lr11xx_status_t lr11xx_wifi_read_basic_complete_results( const void* context, const uint8_t start_result_index,
                                                         const uint8_t                        nb_results,
                                                         lr11xx_wifi_basic_complete_result_t* results );

/*!
 * @brief Read basic MAC, Wi-Fi type and channel results
 *
 * This function can be used to fetch all results in a row, or one after the other.
 * It corresponds to result format @ref ::LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL.
 *
 * An example of usage to fetch all results in a row is:
 * \code{.cpp}
 * uint8_t nb_results = 0;
 * lr11xx_wifi_get_nb_results(&radio, &nb_results);
 * lr11xx_wifi_basic_mac_type_channel_result_t all_results[LR11XX_WIFI_MAX_RESULTS] = {0};
 * lr11xx_wifi_read_basic_mac_type_channel_results(&radio, 0, nb_results, all_results);
 * \endcode
 *
 * On the other hand, fetching result one after the other:
 * \code{.cpp}
 * uint8_t nb_results = 0;
 * lr11xx_wifi_get_nb_results(&radio, &nb_results);
 * lr11xx_wifi_basic_mac_type_channel_result_t single_results = {0};
 * for(uint8_t index_result = 0; index_result < nb_results; index_result++){
 *   lr11xx_wifi_read_basic_mac_type_channel_results(&radio, index_result, 1, &single_results);
 *   // Do something with single_results
 * }
 * \endcode
 *
 * @remark This result fetching function **MUST** be used only if the scan function call was made with Scan Mode set to
 * @ref ::LR11XX_WIFI_SCAN_MODE_BEACON or @ref ::LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT.
 * Refer to @ref lr11xx_wifi_are_scan_mode_result_format_compatible to know which scan mode and result format are
 * compatible.
 *
 * @param [in] radio Radio abstraction
 * @param [in] start_result_index Result index from which starting to fetch the results
 * @param [in] nb_results Number of results to fetch
 * @param [out] results Pointer to an array of result structures to populate. It is up to the caller to ensure this
 * array can hold at least nb_results elements.
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_are_scan_mode_result_format_compatible, lr11xx_wifi_read_basic_complete_results,
 * lr11xx_wifi_read_extended_full_results
 */
lr11xx_status_t lr11xx_wifi_read_basic_mac_type_channel_results( const void* context, const uint8_t start_result_index,
                                                                 const uint8_t nb_results,
                                                                 lr11xx_wifi_basic_mac_type_channel_result_t* results );

/*!
 * @brief Read extended complete results
 *
 * This function can be used to fetch all results in a row, or one after the other.
 * It corresponds to result format @ref ::LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL.
 *
 * An example of usage to fetch all results in a row is:
 * \code{.cpp}
 * uint8_t nb_results = 0;
 * lr11xx_wifi_get_nb_results(&radio, &nb_results);
 * lr11xx_wifi_extended_full_result_t all_results[LR11XX_WIFI_MAX_RESULTS] = {0};
 * lr11xx_wifi_read_extended_full_results(&radio, 0, nb_results, all_results);
 * \endcode
 *
 * On the other hand, fetching result one after the other:
 * \code{.cpp}
 * uint8_t nb_results = 0;
 * lr11xx_wifi_get_nb_results(&radio, &nb_results);
 * lr11xx_wifi_extended_full_result_t single_results = {0};
 * for(uint8_t index_result = 0; index_result < nb_results; index_result++){
 *   lr11xx_wifi_read_extended_full_results(&radio, index_result, 1, &single_results);
 *   // Do something with single_results
 * }
 * \endcode
 *
 * @remark This result fetching function **MUST** be used only if the scan function call was made with Scan Mode set to
 * @ref ::LR11XX_WIFI_SCAN_MODE_FULL_BEACON.
 * Refer to @ref lr11xx_wifi_are_scan_mode_result_format_compatible to know which scan mode and result format are
 * compatible.
 *
 * @param [in] radio Radio abstraction
 * @param [in] start_result_index Result index from which starting to fetch the results
 * @param [in] nb_results Number of results to fetch
 * @param [out] results Pointer to an array of result structures to populate. It is up to the caller to ensure this
 * array can hold at least nb_results elements.
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_are_scan_mode_result_format_compatible, lr11xx_wifi_read_basic_complete_results,
 * lr11xx_wifi_read_basic_mac_type_channel_results
 */
lr11xx_status_t lr11xx_wifi_read_extended_full_results( const void* radio, const uint8_t start_result_index,
                                                        const uint8_t                       nb_results,
                                                        lr11xx_wifi_extended_full_result_t* results );

/*!
 * @brief Reset the internal counters of cumulative timing
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_reset_cumulative_timing( const void* context );

/*!
 * @brief Read the internal counters of cumulative timing
 *
 * @param [in] context Chip implementation context
 * @param [out] timing A pointer to the cumulative timing structure to populate
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_read_cumulative_timing( const void* context, lr11xx_wifi_cumulative_timings_t* timing );

/*!
 * @brief Get size of country code search results
 *
 * @param [in] context Chip implementation context
 * @param [out] nb_country_code_results Number of country results to read
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_get_nb_country_code_results( const void* context, uint8_t* nb_country_code_results );

/*!
 * @brief Read country code results
 *
 * The total number of country code results to read is obtained from a previous call to
 * lr11xx_wifi_get_nb_country_code_results
 *
 * @param [in] context Chip implementation context
 * @param [in] start_result_index The result index to start reading results from
 * @param [in] nb_country_results Number of country code results to read
 * @param [out] country_code_results An array of lr11xx_wifi_country_code_t to be filled. It is up to the application to
 * ensure this array is big enough to hold nb_country_results elements
 *
 * @returns Operation status
 *
 * @see lr11xx_wifi_get_nb_country_code_results, lr11xx_wifi_search_country_code
 */
lr11xx_status_t lr11xx_wifi_read_country_code_results( const void* context, const uint8_t start_result_index,
                                                       const uint8_t               nb_country_results,
                                                       lr11xx_wifi_country_code_t* country_code_results );

/*!
 * @brief Configure the timestamp used to discriminate mobile access points from gateways.
 *
 * This filtering is based on the hypothesis that mobile access points have timestamp shorter than gateways.
 *
 * @param [in] context Chip implementation context
 * @param [in] timestamp_in_s Timestamp value in second
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_cfg_timestamp_ap_phone( const void* context, uint32_t timestamp_in_s );

/*!
 * @brief Get the internal wifi firmware version
 *
 * @param [in] context Chip implementation context
 * @param [out] wifi_version The wifi version structure populated with version numbers
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_read_version( const void* context, lr11xx_wifi_version_t* wifi_version );

/*!
 * @brief Retreive channel information from channel info byte
 *
 * This method is to be called with on the WiFi channel info byte of a scan result.
 *
 * As the WiFi passive scan allows to get Access Point MAC address from Packet WiFi frames, it is possible that the
 * frame does not comes from the Access Point, but from a device. In that case, the RSSI reported by LR11XX is the one
 * of the frame received from the device and not from the Access Point. The rssi_validity flag allows to detect that
 * case.
 *
 * It is possible for an Access Point to be a mobile AP, which is of low interest for location purpose. The LR11XX tries
 * to detect mobile AP based on Access Point up time and set the flag mac_origin_estimation accordingly.
 *
 * @param [in] channel_info The channel info byte to retrieve channel information from. It is obtained from WiFi
 * passive scan result
 * @param [out] channel The channel of the scanned mac address
 * @param [out] rssi_validity The validity of the scanned MAC address
 * @param [out] mac_origin_estimation Indicates the estimation of MAC address origin by LR11XX
 *
 * @see lr11xx_wifi_read_basic_complete_results, lr11xx_wifi_read_basic_mac_type_channel_results,
 * lr11xx_wifi_cfg_timestamp_ap_phone
 */
void lr11xx_wifi_parse_channel_info( const lr11xx_wifi_channel_info_byte_t channel_info, lr11xx_wifi_channel_t* channel,
                                     bool* rssi_validity, lr11xx_wifi_mac_origin_t* mac_origin_estimation );

/*!
 * @brief Helper method to retrieve channel from channel info byte
 *
 * @param [in] channel_info The chanel info byte from passive scan result
 *
 * @returns The channel of scanned MAC address
 *
 * @see lr11xx_wifi_parse_channel_info
 */
lr11xx_wifi_channel_t lr11xx_wifi_extract_channel_from_info_byte( const lr11xx_wifi_channel_info_byte_t channel_info );

/*!
 * @brief Retrieve the Frame Type, Frame Subtype, To/From DS fields from a frame info byte
 *
 * This method is intended to be called on the channel info byte of a passive scan result structure.
 *
 * The from_ds/to_ds (Distribution Station) fields have the following meaning:
 *
 * <table>
 * <tr><th> to_ds value </th><th> from_ds value </th><th> Meaning </th>
 * <tr><td>  False </td><td> False  </td><td> Frame was between two Stations
 * </td> <tr><td>  True </td><td> False  </td><td> Frame was from Station to
 * Access Point </td> <tr><td>  False </td><td> True  </td><td> Frame was sent
 * from Access Point or Distribution Stations </td> <tr><td>  True </td><td>
 * True  </td><td> Mesh network only, frame was between Stations </td>
 * </table>
 *
 * @param [in] frame_type_info The frame info byte from passive scan result
 * @param [out] frame_type The Frame Type of the received frame
 * @param [out] frame_sub_type The Frame SubType of the frame received
 * @param [out] to_ds to_ds field of the frame received
 * @param [out] from_ds from_ds field of the frame received
 */
void lr11xx_wifi_parse_frame_type_info( const lr11xx_wifi_frame_type_info_byte_t frame_type_info,
                                        lr11xx_wifi_frame_type_t*                frame_type,
                                        lr11xx_wifi_frame_sub_type_t* frame_sub_type, bool* to_ds, bool* from_ds );

/*!
 * @brief Retrieve the data rate information from data rate info byte
 *
 * This method is intended to be called on a data rate info byte of a passive scan result structure.
 *
 * @param [in] data_rate_info The data rate info byte from a passive scan result
 * @param [out] wifi_signal_type The wifi signal type of the scanned frame
 * @param [out] wifi_data_rate The data rate of the scanned frame
 */
void lr11xx_wifi_parse_data_rate_info( const lr11xx_wifi_datarate_info_byte_t data_rate_info,
                                       lr11xx_wifi_signal_type_result_t*      wifi_signal_type,
                                       lr11xx_wifi_datarate_t*                wifi_data_rate );

/*!
 * @brief Return the maximal number of results to read per SPI communication
 *
 * This function **DOES NOT** communicates with the LR11XX. It returns the driver maximal number of Wi-Fi results it can
 * retrieve per SPI communication.
 *
 * @remark It is a driver limitation, not a LR11XX limitation, that avoid allocating temporary buffers of size too big
 * when reading Wi-Fi passive scan results.
 *
 * @see LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK
 *
 * @returns The maximal number of results to fetch per SPI calls
 */
uint8_t lr11xx_wifi_get_nb_results_max_per_chunk( void );

/*!
 * @brief Helper method to retrieve the signal type from data rate info byte
 *
 * @param [in] data_rate_info The data rate info byte from a passive scan result
 *
 * @returns The Signal Type of the scanned frame
 */
lr11xx_wifi_signal_type_result_t lr11xx_wifi_extract_signal_type_from_data_rate_info(
    const lr11xx_wifi_datarate_info_byte_t data_rate_info );

/*!
 * @brief Helper function to check if a buffer is a well-formed UTF-8 byte sequence
 *
 * @param [in] buffer The buffer holding the bytes to be analyzed
 * @param [in] length The number of bytes in the buffer
 *
 * @returns The result of the check
 */
bool lr11xx_wifi_is_well_formed_utf8_byte_sequence( const uint8_t* buffer, const uint8_t length );

/*!
 * @brief Check that Wi-Fi scan mode and result format are compatible
 *
 * The possible combination of Wi-Fi scan modes and result format are the following:
 *
 * <table>
 * <tr> <th> Scan Mode <th> Type/Sub-type selected <th> Corresponding read result function
 * <tr> <td> @ref ::LR11XX_WIFI_SCAN_MODE_BEACON <td> Management/Beacon and Management/Probe Response <td rowspan="2">
 * @ref lr11xx_wifi_read_basic_complete_results, @ref lr11xx_wifi_read_basic_mac_type_channel_results <tr> <td>
 * @ref ::LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT <td> Some from Management, Control and Data Types <tr> <td>
 * @ref ::LR11XX_WIFI_SCAN_MODE_FULL_BEACON <td> Management/Beacon and Management/Probe Response <td rowspan="2"> @ref
 * lr11xx_wifi_read_extended_full_results <tr> <td> @ref ::LR11XX_WIFI_SCAN_MODE_UNTIL_SSID <td> Management/Beacon and
 * Management/Probe Response - until SSID field
 * </table>
 *
 * @param scan_mode The scan mode used when calling the scan API
 * @param result_format The result format used when calling the read result API
 * @retval true The scan mode and result format are compatible
 * @retval false The scan mode and result format are not compatible.
 */
bool lr11xx_wifi_are_scan_mode_result_format_compatible( lr11xx_wifi_mode_t          scan_mode,
                                                         lr11xx_wifi_result_format_t result_format );

/**
 * @brief Compute the power consumption in uAh based on the cumulative timing.
 *
 * @param [in] regulator The regulator used during last Wi-Fi passive scan
 * @param [in] timing  Cumulative timing structure to use for computation
 *
 * @returns Current consumption in uAh
 */
uint64_t lr11xx_wifi_get_consumption( lr11xx_system_reg_mode_t regulator, lr11xx_wifi_cumulative_timings_t timing );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_WIFI_H

/* --- EOF ------------------------------------------------------------------ */
