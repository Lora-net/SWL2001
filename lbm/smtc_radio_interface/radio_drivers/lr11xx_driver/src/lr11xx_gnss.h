/*!
 * @file      lr11xx_gnss.h
 *
 * @brief     GNSS scan driver definition for LR11XX
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

#ifndef LR11XX_GNSS_H
#define LR11XX_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_gnss_types.h"
#include "lr11xx_types.h"

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

/*!
 * @brief Get the size of results
 *
 * This method returns the size in bytes of the results available in LR11XX result buffer.
 *
 * @param [in] context Chip implementation context
 * @param [out] result_size Result size
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_result_size( const void* context, uint16_t* result_size );

/*!
 * @brief Read GNSS results
 *
 * The GNSS results are pushed into a buffer directly. This buffer is provided by the application using the driver. It
 * MUST be long enough to contains at least result_buffer_size bytes.
 *
 * @warning No check is done on result_buffer size. If this application provided buffer is too small, there will be a
 * buffer overflow bug!
 *
 * @param [in] context Chip implementation context
 * @param [out] result_buffer Application provided buffer to be filled with result
 * @param [in] result_buffer_size The number of bytes to read from the LR11XX
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_results( const void* context, uint8_t* result_buffer,
                                          const uint16_t result_buffer_size );

/*!
 * @brief Update almanacs given as parameter
 *
 * @remark Note that information header and almanacs for all 128 SV (i.e. 129 20-byte long blocks) must be updated in a
 * row for the whole operation to be successful. Therefore, this function must be called as many times as needed without
 * any other operations in between.
 *
 * @param [in] context Chip implementation context
 * @param [in] blocks Buffer containing at least (nb_of_blocks * LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE) bytes of almanac
 * @param [in] nb_of_blocks Number of blocks to transfer
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_almanac_update( const void* context, const uint8_t* blocks, const uint8_t nb_of_blocks );

/*!
 * @brief Read the almanac
 *
 * @param [in] context Chip implementation context
 * @param [out] almanac_bytestream The bytestream of the almanac
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_almanac( const void*                                context,
                                          lr11xx_gnss_almanac_full_read_bytestream_t almanac_bytestream );

/*!
 * @brief Function to read the frequency search space around the Doppler frequency
 *
 * @param [in] radio Radio abstraction
 * @param [out] freq_search_space Frequency search space configuration read from the chip
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_freq_search_space( const void*                      radio,
                                                    lr11xx_gnss_freq_search_space_t* freq_search_space );

/*!
 * @brief Function to set the frequency search space around the Doppler frequency
 *
 * @param [in] radio Radio abstraction
 * @param [in] freq_search_space Frequency search space configuration to be applied
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_set_freq_search_space( const void*                           radio,
                                                   const lr11xx_gnss_freq_search_space_t freq_search_space );

/*!
 * @brief Get almanac age for a satellite
 *
 * @param [in] context Chip implementation context
 * @param [in] sv_id ID of the satellite corresponding the to almanac requested
 * @param [out] almanac_age Almanac age in days since last GPS time overlap
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_almanac_age_for_satellite( const void* context, const lr11xx_gnss_satellite_id_t sv_id,
                                                           uint16_t* almanac_age );

/*!
 * @brief Push data received from solver to LR11XX
 *
 * @param [in] context Chip implementation context
 * @param [in] payload Payload received from solver
 * @param [in] payload_size Size of the payload received from solver (in bytes)
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_push_solver_msg( const void* context, const uint8_t* payload, const uint16_t payload_size );

/**
 * @brief Return the theoretical number of visible satellites based on the given parameters.
 *
 * @param [in] context Chip implementation context
 * @param [in] date The actual date of scan. Its format is the number of seconds elapsed since January the 6th 1980
 * 00:00:00 with leap seconds included.
 * @param [in] assistance_position, latitude 12 bits and longitude 12 bits
 * @param [in] constellation Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values
 * @param [out] nb_visible_sv theoretical number of visible satellites
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_nb_visible_satellites(
    const void* context, const lr11xx_gnss_date_t date,
    const lr11xx_gnss_solver_assistance_position_t* assistance_position,
    const lr11xx_gnss_constellation_t constellation, uint8_t* nb_visible_sv );

/**
 * @brief Return the theoretical doppler information of theoretical visible satellites, this function shall be called
 * after lr11xx_gnss_get_nb_visible_satellites function.
 *
 * @param [in] context Chip implementation context
 * @param [in] nb_visible_satellites number of visible satellites returned by lr11xx_gnss_get_nb_visible_satellites
 * function,
 * @param [out] visible_satellite_id_doppler Doppler information of each satellite.
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_visible_satellites( const void* context, const uint8_t nb_visible_satellites,
                                                    lr11xx_gnss_visible_satellite_t* visible_satellite_id_doppler );

/*!
 * @brief Activate the GNSS scan constellation
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_read_used_constellations
 */
lr11xx_status_t lr11xx_gnss_set_constellations_to_use( const void*                            context,
                                                       const lr11xx_gnss_constellation_mask_t constellation_mask );

/*!
 * @brief Read constellation used by the GNSS scanner from the almanac update configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] constellations_used Bit mask of the constellations used. See @ref lr11xx_gnss_constellation_t for the
 * possible values
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_set_constellations_to_use
 */
lr11xx_status_t lr11xx_gnss_read_used_constellations( const void*                       context,
                                                      lr11xx_gnss_constellation_mask_t* constellations_used );

/*!
 * @brief Activate the almanac update
 *
 * @param [in] context Chip implementation context
 * @param [in] constellations_to_update Bit mask of the constellations to mark to update. See @ref
 * lr11xx_gnss_constellation_t for the possible values
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_set_almanac_update( const void*                            context,
                                                const lr11xx_gnss_constellation_mask_t constellations_to_update );

/*!
 * @brief Function to read the almanac update configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] constellations_to_update Bit mask of the constellations to mark to update. See @ref
 * lr11xx_gnss_constellation_t for the possible values
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_almanac_update( const void*                       context,
                                                 lr11xx_gnss_constellation_mask_t* constellations_to_update );

/*!
 * @brief Function to read the GNSS firmware version
 *
 * @param [in] context Chip implementation context
 * @param [in] version GNSS Firmware version currently running on the chip
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_firmware_version( const void* context, lr11xx_gnss_version_t* version );

/*!
 * @brief Function to read the supported constellation, GPS or BEIDOU other constellations
 *
 * @param [in] context Chip implementation context
 * @param [out] supported_constellations Bit mask of the constellations used. See @ref lr11xx_gnss_constellation_t for
 * the possible values
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_supported_constellations( const void*                       context,
                                                           lr11xx_gnss_constellation_mask_t* supported_constellations );

/*!
 * @brief Function to set the GNSS scan mode configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] scan_mode  GNSS scan mode
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_scan_mode_t
 */
lr11xx_status_t lr11xx_gnss_set_scan_mode( const void* context, const lr11xx_gnss_scan_mode_t scan_mode );

/*!
 * @brief Start the gnss scan
 *
 * @param [in] context Chip implementation context
 * @param [in] effort_mode Effort mode @ref lr11xx_gnss_search_mode_t
 * @param [in] gnss_input_parameters Bit mask indicating which information is added in the output payload @ref
 * lr11xx_gnss_result_fields_e
 * @param [in] nb_sat The expected number of satellite to provide. This value must be in the range [0:128]
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_scan( const void* context, const lr11xx_gnss_search_mode_t effort_mode,
                                  const uint8_t gnss_input_parameters, const uint8_t nb_sat );

/*!
 * @brief Function to set the assistance position.
 *
 * @param [in] context Chip implementation context
 * @param [in] assistance_position, latitude 12 bits and longitude 12 bits
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_solver_assistance_position_t
 */
lr11xx_status_t lr11xx_gnss_set_assistance_position(
    const void* context, const lr11xx_gnss_solver_assistance_position_t* assistance_position );

/*!
 * @brief Function to read the assistance position.
 *
 * The assistance position read may be different from the one set beforehand with @ref
 * lr11xx_gnss_set_assistance_position due to a scaling computation.
 *
 * @param [in] context Chip implementation context
 * @param [in] assistance_position, latitude 12 bits and longitude 12 bits
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_solver_assistance_position_t
 */
lr11xx_status_t lr11xx_gnss_read_assistance_position( const void*                               context,
                                                      lr11xx_gnss_solver_assistance_position_t* assistance_position );

/*!
 * @brief Host receives an update from the network or assembles itself the update message and send it to the LR11XX.
 *
 * @param [in] context Chip implementation context
 * @param [in] dmc_msg buffer containing the update the network
 * @param [in] dmc_msg_len length of this buffer
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_push_dmc_msg( const void* context, uint8_t* dmc_msg, uint16_t dmc_msg_len );

/*!
 * @brief Get the GNSS context status
 *
 * This function returns the GNSS context status as a raw buffer. It is possible to use
 * lr11xx_gnss_parse_context_status_buffer to obtain the details of the context status.
 *
 * @param [in] context Chip implementation context
 * @param [out] context_status_buffer Pointer to a buffer to be filled with context status information. Must be at least
 * 7 bytes long. It is up to the caller to ensure there is enough place in this buffer.
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_parse_context_status_buffer
 */
lr11xx_status_t lr11xx_gnss_get_context_status( const void*                             context,
                                                lr11xx_gnss_context_status_bytestream_t context_status_buffer );

/*!
 * @brief Get the number of detected satellites during last scan
 *
 * @param [in] context Chip implementation context
 * @param [out] nb_detected_satellites Number of satellites detected
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_nb_detected_satellites( const void* context, uint8_t* nb_detected_satellites );

/*!
 * @brief Get the satellites detected on last scan with their IDs, C/N (aka CNR) and doppler
 *
 * @note Doppler is returned with 6ppm accuracy.
 *
 * @param [in] context Chip implementation context
 * @param [in] nb_detected_satellites Number of detected satellites on last scan (obtained by calling
 * lr11xx_gnss_get_nb_detected_satellites)
 * @param [out] detected_satellite_id_snr_doppler Pointer to an array of structures of size big enough to contain
 * nb_detected_satellites elements
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_detected_satellites(
    const void* context, const uint8_t nb_detected_satellites,
    lr11xx_gnss_detected_satellite_t* detected_satellite_id_snr_doppler );

/*!
 * @brief Read almanacs per satellite range
 *
 * @note Doppler is returned with 6ppm accuracy.
 *
 * @param [in] context Chip implementation context
 * @param [in] sv_id_init Index of the satellite to start reading almanac from
 * @param [in] n_sv Number of satellite almanac to read from sv_id_init
 * @param [out] almanacs Pointer to an array to be filled by almanac data. It is up to the caller to ensure the
 * available length of almanacs buffer is at least (n_sv * LR11XX_GNSS_SINGLE_ALMANAC_READ_SIZE)
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_almanac_per_satellites( const void* context, uint8_t sv_id_init, uint8_t n_sv,
                                                         uint8_t* almanacs );

/*!
 * @brief Read RSSI on GNSS path
 *
 * This is a test function to read RSSI on GNSS path.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi_gnss_dbm RSSI read on GNSS path in dbm
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_gnss_rssi_test( const void* context, int8_t* rssi_gnss_dbm );

/**
 * @brief Parse a raw buffer of context status
 *
 * @param [in] context_status_bytestream The raw buffer of context status to parse. It is up to the caller to ensure the
 * buffer is at least LR11XX_GNSS_CONTEXT_STATUS_LENGTH bytes long
 * @param [out] context_status Pointer to a structure of lr11xx_gnss_context_status_t to be filled with information from
 * context_status_bytestream
 *
 * @returns Operation status
 *
 * @see lr11xx_gnss_get_context_status
 */
lr11xx_status_t lr11xx_gnss_parse_context_status_buffer(
    const lr11xx_gnss_context_status_bytestream_t context_status_bytestream,
    lr11xx_gnss_context_status_t*                 context_status );

/**
 * @brief Extract the destination from the result returned by a GNSS scan
 *
 * @param [in]  result_buffer       Pointer to the buffer holding the result
 * @param [in]  result_buffer_size  Size of the result in byte
 * @param [out] destination         Destination of the result
 *
 * @returns  Operation status
 */
lr11xx_status_t lr11xx_gnss_get_result_destination( const uint8_t* result_buffer, const uint16_t result_buffer_size,
                                                    lr11xx_gnss_destination_t* destination );

/**
 * @brief Helper function that computes the age of an almanac.
 *
 * This function does not call the LR11XX.
 * The almanac age is computed based on the following elements:
 *     - almanac age as obtained from lr11xx_gnss_get_almanac_age_for_satellite
 *     - the number of days elapsed between Epoch (January 6th 1980) and the GPS rollover reference of the current
 * almanac
 *     - the GPS date of today expressed in number of days elapsed since Epoch
 *
 * @remark It is important to use for nb_days_between_epoch_and_corresponding_gps_time_rollover the GPS time rollover
 * corresponding to the reference of the almanac_date. This is especially true when current date is just after a GPS
 * time rollover.
 *
 * @param [in] almanac_date Almanac date as obtained from lr11xx_gnss_get_almanac_age_for_satellite
 * @param [in] nb_days_between_epoch_and_corresponding_gps_time_rollover Number of days elapsed between GPS Epoch and
 * the GPS rollover corresponding to the almanac_date
 * @param [in] nb_days_since_epoch Number of days elapsed between January 6th 1980 and now
 *
 * @returns Age of the almanac expressed in number of days between its start valid instant and now
 */
uint16_t lr11xx_gnss_compute_almanac_age( uint16_t almanac_date,
                                          uint16_t nb_days_between_epoch_and_corresponding_gps_time_rollover,
                                          uint16_t nb_days_since_epoch );

/*!
 * @brief Start the time acquisition/domulation.
 *
 * @param [in] context Chip implementation context
 * @param [in] effort_mode Effort mode @ref lr11xx_gnss_search_mode_t, note that LR11XX_GNSS_OPTION_HIGH_EFFORT is not
 * supported here
 * @param [in] option Fetch time option @ref lr11xx_gnss_fetch_time_option_t
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_fetch_time( const void* context, const lr11xx_gnss_search_mode_t effort_mode,
                                        const lr11xx_gnss_fetch_time_option_t option );
/*!
 * @brief Read time from LR11XX.
 *
 * @param [in] context Chip implementation context
 * @param [out] time Structure containing the time \ref lr11xx_gnss_time_t
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_time( const void* context, lr11xx_gnss_time_t* time );

/*!
 * @brief Reset the internal time.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_reset_time( const void* context );

/*!
 * @brief Reset the location and the history Doppler buffer.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_reset_position( const void* context );

/*!
 * @brief Read the week number rollover.
 *
 * @param [in] context Chip implementation context
 * @param [out] wn_rollover_status Week number rollover status \ref lr11xx_gnss_week_number_rollover_status_t
 * @param [out] wn_number_rollover Week number rollover since 1980
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_week_number_rollover( const void*                                context,
                                                       lr11xx_gnss_week_number_rollover_status_t* wn_rollover_status,
                                                       uint8_t*                                   wn_number_rollover );

/*!
 * @brief Read demod status.
 *
 * @param [in] context Chip implementation context
 * @param [out] demod_status Demodulation status \ref lr11xx_gnss_demod_status_t
 * @param [out] demod_info Demodulation info \ref lr11xx_gnss_demod_info_t
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_demod_status( const void* context, lr11xx_gnss_demod_status_t* demod_status,
                                               lr11xx_gnss_demod_info_t* demod_info );

/*!
 * @brief Read cumulative timing.
 *
 * @param [in] context Chip implementation context
 * @param [out] cumulative_timing Cumulative timing status \ref lr11xx_gnss_cumulative_timing_t, The value of time is in
 * counter of 32KhZ, to have it in second, the counter must be divided by 32768
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_cumulative_timing( const void*                      context,
                                                    lr11xx_gnss_cumulative_timing_t* cumulative_timing );

/*!
 * @brief Compute power consumption based on timings and instantaneous power consumption.
 *
 * @param [in] cumulative_timing Timings read from lr11xx_gnss_read_cumulative_timing API \ref
 * lr11xx_gnss_cumulative_timing_t
 * @param [in] instantaneous_power_consumption_ua Instantaneous power consumption associated to each timings \ref
 * lr11xx_gnss_instantaneous_power_consumption_ua_t
 * @param [out] power_consumption_nah Power consumption computed in nAh
 * @param [out] power_consumption_nwh Power consumption computed in nWh
 */
void lr11xx_gnss_compute_power_consumption(
    const lr11xx_gnss_cumulative_timing_t*                  cumulative_timing,
    const lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua,
    uint32_t* power_consumption_nah, uint32_t* power_consumption_nwh );

/*!
 * @brief Set the GPS time.
 *
 * This command is to be used when the 32kHz clock feeding the LR11xx is turned off.
 * The LR11xx needs the 32kHz clock to track the absolute time. However if the clock is turned off, it will attempt to
 * get the absolute time from GNSS SV demodulation on next GNSS scan, which is power consuming.
 * However, if the MCU has capability to keep the absolute time when 32kHz clock is turned off, then it can use this
 * command to configure the LR11xx, so that the LR11xx is more power efficient when fetching time from SV signal.
 *
 * Typical usage is:
 *   1. MCU get absolute GPS time from any (possibly not accurate) source (like LoRaWAN network for instance)
 *   2. On next scan, the MCU turns on the 32kHz clock, uses lr11xx_gnss_set_time to set the time, with an accuracy that
 * depends on its crystal drift, and start the scan
 *   3. MCU reads the time from LR11xx (lr11xx_gnss_read_time) and stores it internally
 *   4. MCU turns off 32kHz clock of the LR11xx
 *
 * @param [in] context Chip implementation context
 * @param [in] time GPS time in sec from 6 January 1980 00:00:00
 * @param [in] time_accuracy Accuracy in millisecond of the time given. If set to 0, the accuracy of time given is
 * considered to be unknown
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_set_time( const void* context, const uint32_t time, const uint16_t time_accuracy );

/*!
 * @brief Configures the time delay in sec. If the time elapsed from last Assistance position update is larger than this
 * delay and there is always no SV detected, LR11xx will reset the Assistance position and the GNSS scan switches from
 * assisted scan to autonomous scan.
 *
 * @param [in] context Chip implementation context
 * @param [in] delay Delay in second on 3 bytes
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_config_delay_reset_assistance_position( const void* context, const uint32_t delay );

/*!
 * @brief Read the assisted position based on the internal doppler solver executed during lr11xx_gnss_scan or
 * lr11xx_gnss_almanac_update_from_sat functions.
 *
 * @param [in] context Chip implementation context
 * @param [out] results \ref lr11xx_gnss_doppler_solver_result_t
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_doppler_solver_result( const void*                          context,
                                                        lr11xx_gnss_doppler_solver_result_t* results );

/*!
 * @brief Read the time delay in sec. If the time elapsed from last Assistance position update is larger than this
 * delay and there is always no SV detected, LR11xx will reset the Assistance position and the GNSS scan switches from
 * assisted scan to autonomous scan.
 *
 * @param [in] context Chip implementation context
 * @param [out] delay Delay in second on 3 bytes
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_delay_reset_assistance_position( const void* context, uint32_t* delay );

/*!
 * @brief This command launches one scan to download from satellite almanac parameters broadcasted in one page by one
 * constellation.
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values
 * @param [in] effort_mode Effort mode @ref lr11xx_gnss_search_mode_t, note that LR11XX_GNSS_OPTION_HIGH_EFFORT is not
 * supported here
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_almanac_update_from_sat( const void*                            context,
                                                     const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                     const lr11xx_gnss_search_mode_t        effort_mode );

/*!
 * @brief This command read the number of visible satellites and the time elapsed from last detected satellite list
 * update of this constellation.
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values. Only one constellation shall be selected otherwise the command will return an error
 * @param [out] nb_visible_sat number of visible satellites
 * @param [out] time_elapsed elapsed from last sv list update in ms
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_keep_sync_status( const void*                            context,
                                                   const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                   uint8_t* nb_visible_sat, uint32_t* time_elapsed );

/*!
 * @brief This command returns the actual state of almanac GPS and Beidou.
 *
 * @param [in] context Chip implementation context
 * @param [in] almanac_status almanac status for GPS and Beidou @ref lr11xx_gnss_read_almanac_status_t
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_almanac_status( const void*                        context,
                                                 lr11xx_gnss_read_almanac_status_t* almanac_status );

/*!
 * @brief Configures the almanac update period.
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values. Only one constellation shall be selected otherwise the command will return an error
 * @param [in] sv_type sv type to configure. See @ref lr11xx_gnss_sv_type_t for
 * the possible values. This parameter has no impact when constellation_mask is set to LR11XX_GNSS_GPS_MASK but is value
 * must be a valid lr11xx_gnss_sv_type_t one
 * @param [in] period delta in day computed between age of almanac in flash and current day and compared to this period
 * to indicate to the application during a lr11xx_gnss_read_almanac_status if it must be downloaded
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_config_almanac_update_period( const void*                            context,
                                                          const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                          const lr11xx_gnss_sv_type_t sv_type, const uint16_t period );

/*!
 * @brief Read the almanac update period.
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values. Only one constellation shall be selected otherwise the command will return an error
 * @param [in] sv_type sv type of satellites to read period from. See @ref lr11xx_gnss_sv_type_t for
 * the possible values. This parameter has no impact when constellation_mask is set to LR11XX_GNSS_GPS_MASK but is value
 * must be a valid lr11xx_gnss_sv_type_t one
 * @param [out] period delta in day computed between age of almanac in flash and current day and compared to this period
 * to indicate to the application during a lr11xx_gnss_read_almanac_status if it must be downloaded
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_almanac_update_period( const void*                            context,
                                                        const lr11xx_gnss_constellation_mask_t constellation_mask,
                                                        const lr11xx_gnss_sv_type_t sv_type, uint16_t* period );

/*!
 * @brief Returns the list of satellite for the next keep sync scan.
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for
 * the possible values. Only one constellation shall be selected otherwise the command will return an error
 * @param [in] nb_sv_to_get Number of sv to read, the user must call lr11xx_gnss_read_keep_sync_status to know exactly
 * the number of satellites in the list
 * @param [out] sv_sync_list list of sync. It is up to the caller to ensure it is at least nb_sv_to_get byte long
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_sv_sync( const void* context, const lr11xx_gnss_constellation_mask_t constellation_mask,
                                         const uint8_t nb_sv_to_get, uint8_t* sv_sync_list );

/*!
 * @brief Configures the ability of the LR11xx to search almanac for each GPS satellites.
 *
 * @param [in] context Chip implementation context
 * @param [in] gps_sat_activated_1_32 32-bit bit mask sat activated: sat 1-32 activated (default value: 0xFFFFFFFF)
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_set_gps_bit_mask_sat_activated( const void*    context,
                                                            const uint32_t gps_sat_activated_1_32 );

/*!
 * @brief Configures the ability of the LR11xx to search almanac for each Beidou satellites.
 *
 * @param [in] context Chip implementation context
 * @param [in] beidou_sat_activated_1_32 32-bit bit mask sat activated: sat 1-32 activated (default value: 0xBFFCBFFF))
 * @param [in] beidou_sat_activated_33_63 32-bit bit mask sat activated: sat 33-63 activated (default value: 0xC0007FF))
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_set_beidou_bit_mask_sat_activated( const void*    context,
                                                               const uint32_t beidou_sat_activated_1_32,
                                                               const uint32_t beidou_sat_activated_33_63 );

/*!
 * @brief Get the type of scan launched during the last scan
 *
 * @param [in] context Chip implementation context
 * @param [out] last_scan_mode last scan launched. See @ref lr11xx_gnss_scan_mode_launched_t for
 * the possible values.
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_last_scan_mode_launched( const void*                       context,
                                                          lr11xx_gnss_scan_mode_launched_t* last_scan_mode );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_GNSS_H

/* --- EOF ------------------------------------------------------------------ */
