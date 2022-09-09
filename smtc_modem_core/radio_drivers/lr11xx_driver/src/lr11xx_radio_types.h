/*!
 * @file      lr11xx_radio_types.h
 *
 * @brief     Radio driver types for LR11XX
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

#ifndef LR11XX_RADIO_TYPES_H
#define LR11XX_RADIO_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

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

/*!
 * @brief Power Amplifier Selection values
 *
 * - Low-power Power Amplifier can reach up to 14dBm
 * - High-power Power Amplifier can reach up to 22 dBm
 */
typedef enum
{
    LR11XX_RADIO_PA_SEL_LP = 0x00,  //!< Low-power Power Amplifier
    LR11XX_RADIO_PA_SEL_HP = 0x01,  //!< High-power Power Amplifier
    LR11XX_RADIO_PA_SEL_HF = 0x02,  //!< High-frequency Power Amplifier
} lr11xx_radio_pa_selection_t;

/*!
 * @brief GFSK Address Filtering configurations
 *
 * If Address Filtering is enabled but a wrong address is received, therefore the reception is aborted and the address
 * error flag of packet status is set.
 */
typedef enum
{
    LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE      = 0x00,  //!< Filter deactivated
    LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS = 0x01,  //!< Filter on Node Address
    LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES =
        0x02,  //!< Filtering on Node and Broadcast addresses
} lr11xx_radio_gfsk_address_filtering_t;

/*!
 * @brief Chip mode after successfull transmission or reception
 *
 * Unused for RX duty cycle and AutoTxRx operations
 */
typedef enum
{
    LR11XX_RADIO_FALLBACK_STDBY_RC   = 0x01,  //!< Standby RC (Default)
    LR11XX_RADIO_FALLBACK_STDBY_XOSC = 0x02,  //!< Standby XOSC
    LR11XX_RADIO_FALLBACK_FS         = 0x03   //!< FS
} lr11xx_radio_fallback_modes_t;

/*!
 * @brief Ramping time for PA
 *
 * This parameter is the ramping time of the PA. A high value improves spectral quality.
 */
typedef enum
{
    LR11XX_RADIO_RAMP_16_US  = 0x00,  //!< 16 us Ramp Time
    LR11XX_RADIO_RAMP_32_US  = 0x01,  //!< 32 us Ramp Time
    LR11XX_RADIO_RAMP_48_US  = 0x02,  //!< 48 us Ramp Time (Default)
    LR11XX_RADIO_RAMP_64_US  = 0x03,  //!< 64 us Ramp Time
    LR11XX_RADIO_RAMP_80_US  = 0x04,  //!< 80 us Ramp Time
    LR11XX_RADIO_RAMP_96_US  = 0x05,  //!< 96 us Ramp Time
    LR11XX_RADIO_RAMP_112_US = 0x06,  //!< 112 us Ramp Time
    LR11XX_RADIO_RAMP_128_US = 0x07,  //!< 128 us Ramp Time
    LR11XX_RADIO_RAMP_144_US = 0x08,  //!< 144 us Ramp Time
    LR11XX_RADIO_RAMP_160_US = 0x09,  //!< 160 us Ramp Time
    LR11XX_RADIO_RAMP_176_US = 0x0A,  //!< 176 us Ramp Time
    LR11XX_RADIO_RAMP_192_US = 0x0B,  //!< 192 us Ramp Time
    LR11XX_RADIO_RAMP_208_US = 0x0C,  //!< 208 us Ramp Time
    LR11XX_RADIO_RAMP_240_US = 0x0D,  //!< 240 us Ramp Time
    LR11XX_RADIO_RAMP_272_US = 0x0E,  //!< 272 us Ramp Time
    LR11XX_RADIO_RAMP_304_US = 0x0F,  //!< 304 us Ramp Time
} lr11xx_radio_ramp_time_t;

/*!
 * @brief LoRa network type configuration
 */
typedef enum
{
    LR11XX_RADIO_LORA_NETWORK_PRIVATE = 0x00,  //!< LoRa private network
    LR11XX_RADIO_LORA_NETWORK_PUBLIC  = 0x01,  //!< LoRa public network
} lr11xx_radio_lora_network_type_t;

/*!
 * @brief LoRa Spreading Factor configurations
 */
typedef enum
{
    LR11XX_RADIO_LORA_SF5  = 0x05,  //!< Spreading Factor 5
    LR11XX_RADIO_LORA_SF6  = 0x06,  //!< Spreading Factor 6
    LR11XX_RADIO_LORA_SF7  = 0x07,  //!< Spreading Factor 7
    LR11XX_RADIO_LORA_SF8  = 0x08,  //!< Spreading Factor 8
    LR11XX_RADIO_LORA_SF9  = 0x09,  //!< Spreading Factor 9
    LR11XX_RADIO_LORA_SF10 = 0x0A,  //!< Spreading Factor 10
    LR11XX_RADIO_LORA_SF11 = 0x0B,  //!< Spreading Factor 11
    LR11XX_RADIO_LORA_SF12 = 0x0C,  //!< Spreading Factor 12
} lr11xx_radio_lora_sf_t;

/*!
 * @brief LoRa Bandwidth configurations
 */
typedef enum
{
    LR11XX_RADIO_LORA_BW_10  = 0x08,  //!< Bandwidth 10.42 kHz
    LR11XX_RADIO_LORA_BW_15  = 0x01,  //!< Bandwidth 15.63 kHz
    LR11XX_RADIO_LORA_BW_20  = 0x09,  //!< Bandwidth 20.83 kHz
    LR11XX_RADIO_LORA_BW_31  = 0x02,  //!< Bandwidth 31.25 kHz
    LR11XX_RADIO_LORA_BW_41  = 0x0A,  //!< Bandwidth 41.67 kHz
    LR11XX_RADIO_LORA_BW_62  = 0x03,  //!< Bandwidth 62.50 kHz
    LR11XX_RADIO_LORA_BW_125 = 0x04,  //!< Bandwidth 125.00 kHz
    LR11XX_RADIO_LORA_BW_250 = 0x05,  //!< Bandwidth 250.00 kHz
    LR11XX_RADIO_LORA_BW_500 = 0x06,  //!< Bandwidth 500.00 kHz
    LR11XX_RADIO_LORA_BW_200 = 0x0D,  //!< Bandwidth 203.00 kHz, 2G4 and compatible with LR112x chips only
    LR11XX_RADIO_LORA_BW_400 = 0x0E,  //!< Bandwidth 406.00 kHz, 2G4 and compatible with LR112x chips only
    LR11XX_RADIO_LORA_BW_800 = 0x0F,  //!< Bandwidth 812.00 kHz, 2G4 and compatible with LR112x chips only
} lr11xx_radio_lora_bw_t;

/*!
 * @brief LoRa Coding Rate configurations
 */
typedef enum
{
    LR11XX_RADIO_LORA_NO_CR     = 0x00,  //!< No Coding Rate
    LR11XX_RADIO_LORA_CR_4_5    = 0x01,  //!< Coding Rate 4/5 Short Interleaver
    LR11XX_RADIO_LORA_CR_4_6    = 0x02,  //!< Coding Rate 4/6 Short Interleaver
    LR11XX_RADIO_LORA_CR_4_7    = 0x03,  //!< Coding Rate 4/7 Short Interleaver
    LR11XX_RADIO_LORA_CR_4_8    = 0x04,  //!< Coding Rate 4/8 Short Interleaver
    LR11XX_RADIO_LORA_CR_LI_4_5 = 0x05,  //!< Coding Rate 4/5 Long Interleaver
    LR11XX_RADIO_LORA_CR_LI_4_6 = 0x06,  //!< Coding Rate 4/6 Long Interleaver
    LR11XX_RADIO_LORA_CR_LI_4_8 = 0x07,  //!< Coding Rate 4/8 Long Interleaver
} lr11xx_radio_lora_cr_t;

/*!
 * @brief Values for intermediary mode
 */
typedef enum
{
    LR11XX_RADIO_MODE_SLEEP = 0x00,  //!< Sleep / Not recommended with LR1110 FW from 0x0303 to 0x0307 and LR1120 FW
                                     //!< 0x0101 in case of transition from Rx to Tx in LoRa
    LR11XX_RADIO_MODE_STANDBY_RC   = 0x01,  //!< Standby RC
    LR11XX_RADIO_MODE_STANDBY_XOSC = 0x02,  //!< Standby XOSC
    LR11XX_RADIO_MODE_FS           = 0x03   //!< Frequency Synthesis
} lr11xx_radio_intermediary_mode_t;

/*!
 * @brief GFSK Cyclic Redundancy Check configurations
 *
 * If this value is set to something other than CRC_OFF, a CRC is automatically computed and added after the end of the
 * payload on transmitter side. On receiver side, the CRC check is automatically processed.
 */
typedef enum
{
    LR11XX_RADIO_GFSK_CRC_OFF         = 0x01,  //!< CRC check deactivated
    LR11XX_RADIO_GFSK_CRC_1_BYTE      = 0x00,
    LR11XX_RADIO_GFSK_CRC_2_BYTES     = 0x02,
    LR11XX_RADIO_GFSK_CRC_1_BYTE_INV  = 0x04,
    LR11XX_RADIO_GFSK_CRC_2_BYTES_INV = 0x06,
} lr11xx_radio_gfsk_crc_type_t;

/*!
 * @brief GFSK data whitening configurations
 */
typedef enum
{
    LR11XX_RADIO_GFSK_DC_FREE_OFF                   = 0x00,  //!< Whitening deactivated
    LR11XX_RADIO_GFSK_DC_FREE_WHITENING             = 0x01,  //!< Whitening enabled
    LR11XX_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP = 0x03,  //!< Whitening enabled - SX128x compatibility
} lr11xx_radio_gfsk_dc_free_t;

/*!
 * @brief GFSK Header Type configurations
 *
 * This parameter indicates whether or not the payload length is sent and read over the air.
 *
 * If the payload length is known beforehand by both transmitter and receiver, therefore there is no need to send it
 * over the air. Otherwise, setting this parameter to LR11XX_RADIO_GFSK_PKT_VAR_LEN will make the modem to automatically
 * prepand a byte containing the payload length to the the payload on transmitter side. On receiver side, this first
 * byte is read to set the payload length to read.
 *
 * This configuration is only available for GFSK packet types.
 */
typedef enum
{
    LR11XX_RADIO_GFSK_PKT_FIX_LEN = 0x00,  //!< Payload length is not sent/read over the air
    LR11XX_RADIO_GFSK_PKT_VAR_LEN = 0x01,  //!< Payload length is sent/read over the air
    LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP =
        0x02,  //!< Payload length is sent/read over the air - SX128x compatibility
} lr11xx_radio_gfsk_pkt_len_modes_t;

/*!
 * @brief GFSK Preamble Detector Length configurations
 *
 * This parameter sets the minimum length of preamble bits to be received to continue reception of incoming packet. If a
 * packet with preamble length lower than this value is being received, the reception stops without generating IRQ.
 *
 * This parameter has no impact on TX operations.
 */
typedef enum
{
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07
} lr11xx_radio_gfsk_preamble_detector_t;

/*!
 * @brief LoRa Cyclic Redundancy Check configurations
 */
typedef enum
{
    LR11XX_RADIO_LORA_CRC_OFF = 0x00,  //!< CRC deactivated
    LR11XX_RADIO_LORA_CRC_ON  = 0x01,  //!< CRC activated
} lr11xx_radio_lora_crc_t;

/*!
 * @brief LoRa Header type configurations
 */
typedef enum
{
    LR11XX_RADIO_LORA_PKT_EXPLICIT = 0x00,  //!< Explicit header: transmitted over the air
    LR11XX_RADIO_LORA_PKT_IMPLICIT = 0x01,  //!< Implicit header: not transmitted over the air
} lr11xx_radio_lora_pkt_len_modes_t;

/*!
 * @brief LoRa IQ mode configurations
 *
 * LoRa IQ modes are mutually exclusives: a physical packet sent with standard IQ will not be received by a receiver
 * configured with inverted IQ.
 */
typedef enum
{
    LR11XX_RADIO_LORA_IQ_STANDARD = 0x00,  //!< IQ standard
    LR11XX_RADIO_LORA_IQ_INVERTED = 0x01,  //!< IQ inverted
} lr11xx_radio_lora_iq_t;

/*!
 * @brief Packet type values
 */
typedef enum
{
    LR11XX_RADIO_PKT_NONE      = 0x00,  //!< State after cold start, Wi-Fi or GNSS capture
    LR11XX_RADIO_PKT_TYPE_GFSK = 0x01,  //!< GFSK modulation
    LR11XX_RADIO_PKT_TYPE_LORA = 0x02,  //!< LoRa modulation
} lr11xx_radio_pkt_type_t;

/*!
 * @brief Select power amplifier supply source
 */
typedef enum
{
    LR11XX_RADIO_PA_REG_SUPPLY_VREG = 0x00,  //!< Power amplifier supplied by the main regulator
    LR11XX_RADIO_PA_REG_SUPPLY_VBAT = 0x01   //!< Power amplifier supplied by the battery
} lr11xx_radio_pa_reg_supply_t;

/*!
 * @brief RX Duty Cycle Modes
 */
typedef enum
{
    LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX  = 0x00,  //!< LoRa/GFSK: Uses Rx for listening to packets
    LR11XX_RADIO_RX_DUTY_CYCLE_MODE_CAD = 0x01,  //!< Only in LoRa: Uses CAD to listen for over-the-air activity
} lr11xx_radio_rx_duty_cycle_mode_t;

/*!
 * @brief GFSK Bandwidth configurations
 */
typedef enum
{
    LR11XX_RADIO_GFSK_BW_4800   = 0x1F,  //!< Bandwidth 4.8 kHz DSB
    LR11XX_RADIO_GFSK_BW_5800   = 0x17,  //!< Bandwidth 5.8 kHz DSB
    LR11XX_RADIO_GFSK_BW_7300   = 0x0F,  //!< Bandwidth 7.3 kHz DSB
    LR11XX_RADIO_GFSK_BW_9700   = 0x1E,  //!< Bandwidth 9.7 kHz DSB
    LR11XX_RADIO_GFSK_BW_11700  = 0x16,  //!< Bandwidth 11.7 kHz DSB
    LR11XX_RADIO_GFSK_BW_14600  = 0x0E,  //!< Bandwidth 14.6 kHz DSB
    LR11XX_RADIO_GFSK_BW_19500  = 0x1D,  //!< Bandwidth 19.5 kHz DSB
    LR11XX_RADIO_GFSK_BW_23400  = 0x15,  //!< Bandwidth 23.4 kHz DSB
    LR11XX_RADIO_GFSK_BW_29300  = 0x0D,  //!< Bandwidth 29.3 kHz DSB
    LR11XX_RADIO_GFSK_BW_39000  = 0x1C,  //!< Bandwidth 39.0 kHz DSB
    LR11XX_RADIO_GFSK_BW_46900  = 0x14,  //!< Bandwidth 46.9 kHz DSB
    LR11XX_RADIO_GFSK_BW_58600  = 0x0C,  //!< Bandwidth 58.6 kHz DSB
    LR11XX_RADIO_GFSK_BW_78200  = 0x1B,  //!< Bandwidth 78.2 kHz DSB
    LR11XX_RADIO_GFSK_BW_93800  = 0x13,  //!< Bandwidth 93.8 kHz DSB
    LR11XX_RADIO_GFSK_BW_117300 = 0x0B,  //!< Bandwidth 117.3 kHz DSB
    LR11XX_RADIO_GFSK_BW_156200 = 0x1A,  //!< Bandwidth 156.2 kHz DSB
    LR11XX_RADIO_GFSK_BW_187200 = 0x12,  //!< Bandwidth 187.2 kHz DSB
    LR11XX_RADIO_GFSK_BW_234300 = 0x0A,  //!< Bandwidth 232.3 kHz DSB
    LR11XX_RADIO_GFSK_BW_312000 = 0x19,  //!< Bandwidth 312.0 kHz DSB
    LR11XX_RADIO_GFSK_BW_373600 = 0x11,  //!< Bandwidth 373.6 kHz DSB
    LR11XX_RADIO_GFSK_BW_467000 = 0x09   //!< Bandwidth 467.0 kHz DSB
} lr11xx_radio_gfsk_bw_t;

/*!
 * @brief Possible automatic actions when Channel Activity Detection operations terminate
 *
 * For RADIO_EXIT_MODE_CAD_RX, LR11XX enters RX mode on activity detected. The timeout value for this RX operation is
 * defined as:
 *
 * \f$ 31.25us \times timeout \f$
 *
 * With \f$ timeout \f$ defined in RadioCadParams_t::timeout
 *
 * If the CAD operation is negative with RADIO_CAD_EXIT_MODE_RX or if CAD operation is positive with
 * RADIO_CAD_EXIT_MODE_TX, therefore the LR11XX enters Standby RC mode.
 */
typedef enum
{
    LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC = 0x00,  //!< Enter standby RC mode after CAD operation
    LR11XX_RADIO_CAD_EXIT_MODE_RX        = 0x01,  //!< Enter in RX mode if an activity is detected
    LR11XX_RADIO_CAD_EXIT_MODE_TX        = 0x10,  //!< Enter in TX mode if no activity is detected
} lr11xx_radio_cad_exit_mode_t;

/*!
 * @brief Pulse shape configurations
 */
typedef enum
{
    LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF   = 0x00,  //!< No filter applied
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_03 = 0x08,  //!< Gaussian BT 0.3
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05 = 0x09,  //!< Gaussian BT 0.5
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07 = 0x0A,  //!< Gaussian BT 0.7
    LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1  = 0x0B   //!< Gaussian BT 1.0
} lr11xx_radio_gfsk_pulse_shape_t;

/*!
 * @brief Channel Activity Detection parameters
 *
 * Parameters detPeak and detMin are to be used for tuning the sensitivity of Channel Activity Detection. It depends on
 * Spreading Factor, Bandwidth and symbolNum.
 *
 * For detPeak, the 5 MSBits are encoding the integer part, the 3 LSBits are encoding 1/8 of the decimal part. For
 * instance, \f$detPeak = 50\f$ (= 0x32) leads to a ratio being \f$6 + 2 * 1/8 = 6.25\f$.
 *
 * detMin is unit free and represents the ratio between the minimal power of a correlation peak and measurement gain
 * that can be considered as a peak detection. It helps to avoid detection on noise. Authorized values a from 0 to 181.
 */
typedef struct lr11xx_radio_cad_params_s
{
    uint8_t cad_symb_nb;                         //!< Number of symbols used for CAD detection
    uint8_t cad_detect_peak;                     //!< Ratio for CAD between correlator peak and average
                                                 //!< (Default 0x32)
    uint8_t cad_detect_min;                      //!< Minimum power of the correlation peak to be
                                                 //!< considered as a positive CAD (Default 0x0A)
    lr11xx_radio_cad_exit_mode_t cad_exit_mode;  //!< Automated action on CAD completion
    uint32_t                     cad_timeout;    //!< Value used to compute timeout
} lr11xx_radio_cad_params_t;

/*!
 * @brief Status of GFSK received packet
 */
typedef struct lr11xx_radio_pkt_status_gfsk_s
{
    int8_t  rssi_sync_in_dbm;  //!< RSSI value latched on detection of the last received packet Sync Address
    int8_t  rssi_avg_in_dbm;   //!< RSSI averaged over the payload of the last received packet
    uint8_t rx_len_in_bytes;   //!< Length of the last received packet [Bytes]
    bool    is_addr_err;  //!< Address filtering status. Asserted if received packet address does not match node address
                          //!< nor broadcast address
    bool is_crc_err;      //!< CRC status of the current packet (applicable only in RX, with CRC enabled)
    bool is_len_err;      //!< Asserted when the length of last received packet is greater than the maximal length
                          //!< (applicable only in RX with variable length packet)
    bool is_abort_err;    //!< Asserted when the current packet has been aborted (applicable in RX and TX)
    bool is_received;     //!< Asserted when packet reception is done (applicable in RX)
    bool is_sent;         //!< Asserted when packet transmission is done (applicable in TX)
} lr11xx_radio_pkt_status_gfsk_t;

/*!
 * @brief Status of received packet
 */
typedef struct lr11xx_radio_pkt_status_lora_s
{
    int8_t rssi_pkt_in_dbm;         //!< Average RSSI over last received packet.
    int8_t snr_pkt_in_db;           //!< SNR estimated on last received packet.
    int8_t signal_rssi_pkt_in_dbm;  //!< RSSI of last packet latched after
} lr11xx_radio_pkt_status_lora_t;

/*!
 * @brief Length and offset of received packet
 */
typedef struct lr11xx_radio_rx_buffer_status_s
{
    uint8_t pld_len_in_bytes;      //!< Length of received packet [Bytes]
    uint8_t buffer_start_pointer;  //!< Offset in the reception buffer of
                                   //!< first byte received [Bytes]
} lr11xx_radio_rx_buffer_status_t;

/*!
 * @brief GFSK packet statistic structure
 */
typedef struct lr11xx_radio_stats_gfsk_s
{
    uint16_t nb_pkt_received;   //!< Total number of received packets
    uint16_t nb_pkt_crc_error;  //!< Total number of received packets with CRC error
    uint16_t nb_pkt_len_error;  //!< Total number of received packets with a length error
} lr11xx_radio_stats_gfsk_t;

/*!
 * @brief LoRa packet statistic structure
 */
typedef struct lr11xx_radio_stats_lora_s
{
    uint16_t nb_pkt_received;      //!< Total number of received packets
    uint16_t nb_pkt_crc_error;     //!< Total number of received packets with CRC error
    uint16_t nb_pkt_header_error;  //!< Total number of packets with header error
    uint16_t nb_pkt_falsesync;     //!< Total number of false sync
} lr11xx_radio_stats_lora_t;

/*!
 * @brief Modulation configuration for GFSK packet
 */
typedef struct lr11xx_radio_mod_params_gfsk_s
{
    uint32_t                        br_in_bps;     //!< GFSK bitrate [bit/s]
    lr11xx_radio_gfsk_pulse_shape_t pulse_shape;   //!< GFSK pulse shape
    lr11xx_radio_gfsk_bw_t          bw_dsb_param;  //!< GFSK bandwidth
    uint32_t                        fdev_in_hz;    //!< GFSK frequency deviation [Hz]
} lr11xx_radio_mod_params_gfsk_t;

/*!
 * @brief Modulation configuration for LoRa packet
 */
typedef struct lr11xx_radio_mod_params_lora_s
{
    lr11xx_radio_lora_sf_t sf;    //!< LoRa spreading factor
    lr11xx_radio_lora_bw_t bw;    //!< LoRa bandwidth
    lr11xx_radio_lora_cr_t cr;    //!< LoRa coding rate
    uint8_t                ldro;  //!< LoRa LDRO
} lr11xx_radio_mod_params_lora_t;

/*!
 * @brief Packet parameter configuration for GFSK packets
 */
typedef struct lr11xx_radio_pkt_params_gfsk_s
{
    uint16_t                              preamble_len_in_bits;   //!< GFSK Preamble length [bits]
    lr11xx_radio_gfsk_preamble_detector_t preamble_detector;      //!< GFSK Preamble detection configuration
    uint8_t                               sync_word_len_in_bits;  //!< GFSK Syncword length [bits]
    lr11xx_radio_gfsk_address_filtering_t address_filtering;      //!< GFSK Address filtering/comparison configuration
    lr11xx_radio_gfsk_pkt_len_modes_t     header_type;            //!< GFSK Header type configuration
    uint8_t                               pld_len_in_bytes;       //!< GFSK Payload length [bytes]
    lr11xx_radio_gfsk_crc_type_t          crc_type;               //!< GFSK CRC configuration
    lr11xx_radio_gfsk_dc_free_t           dc_free;                //!< GFSK Whitening configuration
} lr11xx_radio_pkt_params_gfsk_t;

/*!
 * @brief Packet parameter configuration for LoRa packets
 */
typedef struct lr11xx_radio_pkt_params_lora_s
{
    uint16_t                          preamble_len_in_symb;  //!< LoRa Preamble length [symbols]
    lr11xx_radio_lora_pkt_len_modes_t header_type;           //!< LoRa Header type configuration
    uint8_t                           pld_len_in_bytes;      //!< LoRa Payload length [bytes]
    lr11xx_radio_lora_crc_t           crc;                   //!< LoRa CRC configuration
    lr11xx_radio_lora_iq_t            iq;                    //!< LoRa IQ configuration
} lr11xx_radio_pkt_params_lora_t;

/*!
 * @brief Configuration of Power Amplifier
 *
 * @ref pa_duty_cycle controls the duty cycle of Power Amplifier according to:
 * \f$ dutycycle = 0.2 + 0.04 \times pa_duty_cycle \f$
 * It can be used to adapt the TX multi-band operation using a single-matching network.
 *
 * The allowed duty cycle values for LPA are from 0.2 to 0.48 (by step of 0.04). Therefore possible values for
 * pa_duty_cycle go from 0 to 7.
 *
 * The allowed duty cycle values for HPA go from 0.2 to 0.36 (by step of 0.04). Therefore in this case, the possible
 * values for pa_duty_cycle go from 0 to 4.
 *
 * @ref pa_hp_sel controls the number of slices for HPA according to: \f$ \#slices = pa_hp_sel + 1 \f$
 */
typedef struct lr11xx_radio_pa_cfg_s
{
    lr11xx_radio_pa_selection_t  pa_sel;         //!< Power Amplifier selection
    lr11xx_radio_pa_reg_supply_t pa_reg_supply;  //!< Power Amplifier regulator supply source
    uint8_t                      pa_duty_cycle;  //!< Power Amplifier duty cycle (Default 0x04)
    uint8_t                      pa_hp_sel;      //!< Number of slices for HPA (Default 0x07)
} lr11xx_radio_pa_cfg_t;

/*!
 * @brief RSSI calibration table
 */
typedef struct lr11xx_radio_rssi_calibration_table_s
{
    struct
    {
        uint8_t g4;
        uint8_t g5;
        uint8_t g6;
        uint8_t g7;
        uint8_t g8;
        uint8_t g9;
        uint8_t g10;
        uint8_t g11;
        uint8_t g12;
        uint8_t g13;
        uint8_t g13hp1;
        uint8_t g13hp2;
        uint8_t g13hp3;
        uint8_t g13hp4;
        uint8_t g13hp5;
        uint8_t g13hp6;
        uint8_t g13hp7;
    } gain_tune;  //!< Used to set gain tune value for RSSI calibration

    int16_t gain_offset;  //!< Used to set gain offset value for RSSI calibration
} lr11xx_radio_rssi_calibration_table_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_RADIO_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
