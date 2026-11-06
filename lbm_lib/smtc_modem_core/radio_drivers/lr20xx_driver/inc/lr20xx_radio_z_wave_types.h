/*!
 * @file      lr20xx_radio_z_wave_types.h
 *
 * @brief     Z-Wave radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_Z_WAVE_TYPES_H
#define LR20XX_RADIO_Z_WAVE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lr20xx_radio_fsk_common_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

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

/**
 * @brief Z-Wave modes
 */
typedef enum lr20xx_radio_z_wave_datarate_type_e
{
    LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_LR1 = 0x00,  //!< Z-Wave datarate type LR1
    LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R1  = 0x01,  //!< Z-Wave datarate type R1
    LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R2  = 0x02,  //!< Z-Wave datarate type R2
    LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R3  = 0x03,  //!< Z-Wave datarate type R3
} lr20xx_radio_z_wave_datarate_type_t;

/**
 * @brief Z-Wave address filtering configuration
 */
typedef enum lr20xx_radio_z_wave_addr_filtering_e
{
    LR20XX_RADIO_Z_WAVE_ADDR_FILTERING_DISABLED = 0x00,  //!< Z-Wave address filtering disabled
    LR20XX_RADIO_Z_WAVE_ADDR_FILTERING_HOMEID   = 0x01,  //!< Z-Wave address filtering enabled on HomeID
    LR20XX_RADIO_Z_WAVE_ADDR_FILTERING_HOMEID_AND_BEAM =
        0x02,  //!< Z-Wave address filtering enabled on HomeID and bean frame
} lr20xx_radio_z_wave_addr_filtering_t;

/**
 * @brief Z-Wave (Frame Check Sequence) FCS mode
 */
typedef enum lr20xx_radio_z_wave_fcs_mode_e
{
    LR20XX_RADIO_Z_WAVE_FCS_MODE_ON   = 0x00,  //!< Z-Wave FCS handled by the chip in Tx and Rx
    LR20XX_RADIO_Z_WAVE_FCS_MODE_FIFO = 0x01,  //!< Z-Wave FCS provided in FIFO by the application in Tx, and returned
                                               //!< to the application in Rx. Moreover in Rx the FCS is not checked.
} lr20xx_radio_z_wave_fcs_mode_t;

/**
 * @brief Z-Wave Beam address length
 */
typedef enum lr20xx_radio_z_wave_beam_address_length_e
{
    LR20XX_RADIO_Z_WAVE_BEAM_ADDRESS_LENGTH_8_BITS  = 0x00,  //!< Z-Wave Beam address length for all modes except LR1
    LR20XX_RADIO_Z_WAVE_BEAM_ADDRESS_LENGTH_12_BITS = 0x01,  //!< Z-Wave Beam address length for mode LR1
} lr20xx_radio_z_wave_beam_address_length_t;

/**
 * @brief Z-Wave parameters
 */
typedef struct lr20xx_radio_z_wave_params_s
{
    lr20xx_radio_z_wave_datarate_type_t  datarate;        //!< Z-Wave datarate
    lr20xx_radio_fsk_common_bw_t         rx_bw;           //!< Rx bandwidth
    lr20xx_radio_z_wave_addr_filtering_t addr_filtering;  //!< Z-Wave address filtering mode
    uint8_t payload_length;  //!< In reception, this is the maximum accepted MPDU size, i.e. what is decoded from the
                             //!< header. In transmission, this is the total number of bytes to transmit.
    uint16_t pbl_len_tx_in_bit;  //!< Size in bits of the preamble to send, or 0 to automatically set the preamble size
                                 //!< to the minimum possible for the selected mode
    uint8_t pbl_detect_len_in_bit;  //!< Preamble size, in bits, that is used for the detection. The recommended value
                                    //!< is 32 in all modes, or 0xff to automatically set the size to the optimal value.
    lr20xx_radio_z_wave_fcs_mode_t fcs_mode;  //!< Frame Check Sequence (FCS) configuration
} lr20xx_radio_z_wave_params_t;

/**
 * @brief Z-Wave statistics of received packets
 */
typedef struct lr20xx_radio_z_wave_rx_stats_s
{
    uint16_t received_packets;  //!< Number of received packets
    uint16_t crc_errors;        //!< Number of received packets with CRC error
    uint16_t length_errors;     //!< Number of received packets with length error
} lr20xx_radio_z_wave_rx_stats_t;

/**
 * @brief Z-Wave beam filtering parameters
 */
typedef struct lr20xx_radio_z_wave_beam_filtering_params_s
{
    uint8_t                                   tag;             //!< Z-Wave beam tag
    lr20xx_radio_z_wave_beam_address_length_t address_length;  //!< Z-Wave address length
    uint16_t node_id;      //!< NodeId - 8-bit value, 12-bit value for LR1 mode. 0xFF is always accepted
    uint8_t  homeid_hash;  //!< HomeID hash to accept - 0x55, 0x0A and 0x4A are always accepted
} lr20xx_radio_z_wave_beam_filtering_params_t;

/**
 * @brief Z-Wave packet status
 */
typedef struct lr20xx_radio_z_wave_pkt_status_s
{
    uint16_t packet_length_bytes;      //!< Length of last received packet in bytes
    int16_t  rssi_avg_in_dbm;          //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    lr20xx_radio_z_wave_datarate_type_t last_pkt_datarate_type;  //!< Datarate type of last received packet
    int16_t                             rssi_sync_in_dbm;        //!< RSSI in dBm - value after syncword detection
    uint8_t rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    uint8_t link_quality_indicator;    //!< Margin to the detection level, in 0.25dB
} lr20xx_radio_z_wave_pkt_status_t;

/**
 * @brief Z-Wave scan channel parameters
 *
 * The recommended values for @ref lr20xx_radio_z_wave_scan_channel_params_s::timeout_in_30us_step are:
 *
 * - For 3 channels scan:
 * | @p datarate | @p timeout_in_30us_step | Timeout in us |
 * | -- | -- | -- |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R1 | 19 | 570 |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R2 | 34 | 1020 |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R3 | 8 | 240 |
 *
 * - For 4 channels scan:
 * | @p datarate | @p timeout_in_30us_step | Timeout in us |
 * | -- | -- | -- |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_LR1 | 12 | 360 |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R1 | 19 | 570 |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R2 | 30 | 900 |
 * | LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R3 | 7 | 210 |
 */
typedef struct lr20xx_radio_z_wave_scan_channel_params_s
{
    uint32_t                            rf_freq_in_hz;         //!< RF frequency in Hertz
    lr20xx_radio_z_wave_datarate_type_t datarate;              //!< Z-Wave datarate
    uint8_t                             timeout_in_30us_step;  //!< Timeout in 30 microsecond step
    bool                                enable_cca;  //!< Indicate wether Clear Channel Assesment should be enabled
} lr20xx_radio_z_wave_scan_channel_params_t;

/**
 * @brief Z-Wave scan parameters
 *
 * Recommended configurations are the following:
 * - @ref lr20xx_radio_z_wave_scan_channel_params_t.enable_cca should be false
 * - if @ref lr20xx_radio_z_wave_scan_params_t.number_of_channels_to_scan == 2, then @ref
 * lr20xx_radio_z_wave_scan_channel_params_t.timeout_in_30us_step should be set to 11 (=330us)
 * - if @ref lr20xx_radio_z_wave_scan_params_t.number_of_channels_to_scan == 3
 * <table>
 * <tr> <th> @ref lr20xx_radio_z_wave_scan_channel_params_t.datarate <th> @ref
 * lr20xx_radio_z_wave_scan_channel_params_t.timeout_in_30us_step <th> Timeout in microsecond <tr> <td> @ref
 * LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R1 <td> 19 <td> 570 <tr> <td> @ref LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R2 <td> 34
 * <td> 1020 <tr> <td> @ref LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R3 <td> 8 <td> 240
 * </table>
 * - if @ref lr20xx_radio_z_wave_scan_params_t.number_of_channels_to_scan == 4
 * <table>
 * <tr> <th> @ref lr20xx_radio_z_wave_scan_channel_params_t.datarate <th> @ref
 * lr20xx_radio_z_wave_scan_channel_params_t.timeout_in_30us_step <th> Timeout in microsecond <tr> <td> @ref
 * LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R1 <td> 19 <td> 570 <tr> <td> @ref LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R2 <td> 30
 * <td> 900 <tr> <td> @ref LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_R3 <td> 7 <td> 210 <tr> <td> @ref
 * LR20XX_RADIO_Z_WAVE_DATARATE_TYPE_LR1 <td> 12 <td> 360
 * </table>
 */
typedef struct lr20xx_radio_z_wave_scan_params_s
{
    uint8_t number_of_channels_to_scan;                        //!< Number of channel to scan. Valid values are in [2:4]
    lr20xx_radio_z_wave_scan_channel_params_t channel_cfg[4];  //!< Z-Wave scan channel configurations
    lr20xx_radio_z_wave_addr_filtering_t      addr_filtering;  //!< Z-Wave address filtering mode
    lr20xx_radio_z_wave_fcs_mode_t            fcs_mode;        //!< Frame Check Sequence (FCS) configuration
} lr20xx_radio_z_wave_scan_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_Z_WAVE_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
