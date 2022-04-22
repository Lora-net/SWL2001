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
#include "lr11xx_system_types.h"
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
 * @brief Get the time spent in signal acquisition and signal analysis
 *
 * These timings allow to compute the current consumption of the last GNSS scan.
 *
 * @param [in] context Chip implementation context
 * @param [out] timings GNSS timings of last GNSS scan
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_timings( const void* context, lr11xx_gnss_timings_t* timings );

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

/*!
 * @brief Activate the GNSS scan constellation
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr11xx_gnss_constellation_t for the
 * possible values
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
 * @ref lr11xx_gnss_scan_mode_t
 */
lr11xx_status_t lr11xx_gnss_set_scan_mode( const void* context, const lr11xx_gnss_scan_mode_t scan_mode );

/*!
 * @brief Gnss scan with no assisted parameters needed
 *
 * @param [in] context Chip implementation context
 * @param [in] date The actual date of scan. Its format is the number of seconds elapsed since January the 6th 1980
 * 00:00:00 with leap seconds included.
 * @param [in] effort_mode Effort mode @ref lr11xx_gnss_search_mode_t
 * @param [in] gnss_input_parameters Bit mask indicating which information is added in the output payload @ref
 * lr11xx_gnss_result_fields_legacy_e
 * @param [in] nb_sat The expected number of satellite to provide. This value must be in the range [0:128]
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_scan_autonomous( const void* context, const lr11xx_gnss_date_t date,
                                             const lr11xx_gnss_search_mode_t effort_mode,
                                             const uint8_t gnss_input_parameters, const uint8_t nb_sat );

/*!
 * @brief Gnss scan with assisted parameters.
 *
 * @param [in] context Chip implementation context
 * @param [in] date The actual date of scan. Its format is the number of seconds elapsed since January the 6th 1980
 * 00:00:00 with leap seconds included.
 * @param [in] effort_mode Effort mode @ref lr11xx_gnss_search_mode_t
 * @param [in] gnss_input_parameters Bit mask indicating which information is added in the output payload @ref
 * lr11xx_gnss_result_fields_legacy_e
 * @param [in] nb_sat The expected number of satellite to provide. This value must be in the range [0:128]
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_scan_assisted( const void* context, const lr11xx_gnss_date_t date,
                                           const lr11xx_gnss_search_mode_t effort_mode,
                                           const uint8_t gnss_input_parameters, const uint8_t nb_sat );

/*!
 * @brief Function to set the assistance position.
 *
 * @param [in] context Chip implementation context
 * @param [in] assistance_position, latitude 12 bits and longitude 12 bits
 *
 * @ref See lr11xx_gnss_solver_assistance_position_t
 *
 * @returns Operation status
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
 * @ref See lr11xx_gnss_solver_assistance_position_t
 *
 * @returns Operation status
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

/**
 * @brief Compute the power consumption in uAh based on the time spent in signal acquisition and signal analysis.
 *
 * @param [in] regulator The regulator used during last GNSS scan
 * @param [in] timings Timings allowing to compute the current consumption
 * @param [in] constellations_used Bit mask of the constellations used
 *
 * @returns Current consumption in uAh
 */
uint32_t lr11xx_gnss_get_consumption( lr11xx_system_reg_mode_t regulator, lr11xx_gnss_timings_t timings,
                                      lr11xx_gnss_constellation_mask_t constellations_used );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_GNSS_H

/* --- EOF ------------------------------------------------------------------ */
