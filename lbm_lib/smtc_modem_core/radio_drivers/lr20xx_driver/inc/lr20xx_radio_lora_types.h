/*!
 * @file      lr20xx_radio_lora_types.h
 *
 * @brief     LoRa radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_LORA_TYPES_H
#define LR20XX_RADIO_LORA_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief LoRa syncword value for LoRaWAN public networks
 */
#define LR20XX_RADIO_LORA_SYNCWORD_LORAWAN_PUBLIC_NETWORK ( 0x34 )

/**
 * @brief LoRa syncword value for LoRaWAN private networks
 */
#define LR20XX_RADIO_LORA_SYNCWORD_LORAWAN_PRIVATE_NETWORK ( 0x12 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LoRa Spreading Factor
 */
typedef enum
{
    LR20XX_RADIO_LORA_SF5  = 0x05,  //!< Spreading factor 5
    LR20XX_RADIO_LORA_SF6  = 0x06,  //!< Spreading factor 6
    LR20XX_RADIO_LORA_SF7  = 0x07,  //!< Spreading factor 7
    LR20XX_RADIO_LORA_SF8  = 0x08,  //!< Spreading factor 8
    LR20XX_RADIO_LORA_SF9  = 0x09,  //!< Spreading factor 9
    LR20XX_RADIO_LORA_SF10 = 0x0A,  //!< Spreading factor 10
    LR20XX_RADIO_LORA_SF11 = 0x0B,  //!< Spreading factor 11
    LR20XX_RADIO_LORA_SF12 = 0x0C,  //!< Spreading factor 12
} lr20xx_radio_lora_sf_t;

/**
 * @brief LoRa Bandwidth
 */
typedef enum
{
    LR20XX_RADIO_LORA_BW_7    = 0x00,  //!< Bandwidth 7.81 kHz
    LR20XX_RADIO_LORA_BW_10   = 0x08,  //!< Bandwidth 10.42 kHz
    LR20XX_RADIO_LORA_BW_15   = 0x01,  //!< Bandwidth 15.63 kHz
    LR20XX_RADIO_LORA_BW_20   = 0x09,  //!< Bandwidth 20.83 kHz
    LR20XX_RADIO_LORA_BW_31   = 0x02,  //!< Bandwidth 31.25 kHz
    LR20XX_RADIO_LORA_BW_41   = 0x0A,  //!< Bandwidth 41.67 kHz
    LR20XX_RADIO_LORA_BW_83   = 0x0B,  //!< Bandwidth 83.34 kHz
    LR20XX_RADIO_LORA_BW_62   = 0x03,  //!< Bandwidth 62.50 kHz
    LR20XX_RADIO_LORA_BW_101  = 0x0C,  //!< Bandwidth 101.5625 kHz
    LR20XX_RADIO_LORA_BW_125  = 0x04,  //!< Bandwidth 125 kHz
    LR20XX_RADIO_LORA_BW_203  = 0x0D,  //!< Bandwidth 203 kHz
    LR20XX_RADIO_LORA_BW_250  = 0x05,  //!< Bandwidth 250 kHz
    LR20XX_RADIO_LORA_BW_406  = 0x0E,  //!< Bandwidth 406 kHz
    LR20XX_RADIO_LORA_BW_500  = 0x06,  //!< Bandwidth 500 kHz
    LR20XX_RADIO_LORA_BW_812  = 0x0F,  //!< Bandwidth 812 kHz
    LR20XX_RADIO_LORA_BW_1000 = 0x07,  //!< Bandwidth 1000 kHz
} lr20xx_radio_lora_bw_t;

/**
 * @brief LoRa Coding Rate
 */
typedef enum
{
    LR20XX_RADIO_LORA_NO_CR                   = 0x00,  //!< No Coding Rate
    LR20XX_RADIO_LORA_CR_4_5                  = 0x01,  //!< Short Interleaver Parity code
    LR20XX_RADIO_LORA_CR_4_6                  = 0x02,  //!< Short Interleaver Hamming code 2/3
    LR20XX_RADIO_LORA_CR_4_7                  = 0x03,  //!< Short Interleaver Hamming code 7/5
    LR20XX_RADIO_LORA_CR_4_8                  = 0x04,  //!< Short Interleaver Hamming code 1/2
    LR20XX_RADIO_LORA_CR_LI_4_5               = 0x05,  //!< Long Interleaver Parity code
    LR20XX_RADIO_LORA_CR_LI_4_6               = 0x06,  //!< Long Interleaver Hamming code 2/3
    LR20XX_RADIO_LORA_CR_LI_4_8               = 0x07,  //!< Long Interleaver Hamming code 1/2
    LR20XX_RADIO_LORA_CR_LI_CONVOLUTIONAL_4_6 = 0x08,  //!< Long Interleaver Convolutional code 2/3
    LR20XX_RADIO_LORA_CR_LI_CONVOLUTIONAL_4_8 = 0x09,  //!< Long Interleaver Convolutional code 1/2
} lr20xx_radio_lora_cr_t;

/**
 * @brief LoRa PPM Offset
 */
typedef enum
{
    LR20XX_RADIO_LORA_NO_PPM  = 0x00,  //!< No PPM offset: use full range of modulation
    LR20XX_RADIO_LORA_PPM_1_4 = 0x01,  //!< 1 bin every 4
} lr20xx_radio_lora_ppm_t;

/**
 * @brief LoRa header packet configuration
 */
typedef enum
{
    LR20XX_RADIO_LORA_PKT_EXPLICIT = 0x00,  //!< (aka. variable length packet) The packet is sent with a header
                                            //!< containing payload length so the receiver adapts to the payload length
    LR20XX_RADIO_LORA_PKT_IMPLICIT =
        0x01,  //!< (aka. fixed length packet) The packet is sent without header so the receiver must be configured to
               //!< receive the same payload length as the transmitted one
} lr20xx_radio_lora_pkt_mode_t;

/**
 * @brief LoRa Cyclic Redundancy Check packet configuration
 */
typedef enum
{
    LR20XX_RADIO_LORA_CRC_DISABLED = 0x00,  //!< CRC is not appended to the packet sent over the air
    LR20XX_RADIO_LORA_CRC_ENABLED =
        0x01,  //!< CRC is appended to the packet sent over the air, and checked upon reception
} lr20xx_radio_lora_crc_t;

/**
 * @brief LoRa IQ packet configuration
 */
typedef enum
{
    LR20XX_RADIO_LORA_IQ_STANDARD = 0x00,  //!< IQ standard
    LR20XX_RADIO_LORA_IQ_INVERTED = 0x01,  //!< IQ inverted
} lr20xx_radio_lora_iq_t;

/**
 * @brief Exit mode of LoRa Channel Activity Detection (CAD) operation
 *
 * Refer to @ref lr20xx_radio_common_set_rx_tx_fallback_mode for details regarding configuration of fallback mode.
 *
 * @see lr20xx_radio_common_set_rx_tx_fallback_mode
 */
typedef enum
{
    LR20XX_RADIO_LORA_CAD_EXIT_MODE_STANDBYRC =
        0x00,  //!< The chip goes to fallback mode after CAD operation, no matter what the result of CAD is
    LR20XX_RADIO_LORA_CAD_EXIT_MODE_RX = 0x01,  //!< If the CAD operation detects an activity, the chip enters in RX
                                                //!< mode. Otherwise it enters in fallback mode
    LR20XX_RADIO_LORA_CAD_EXIT_MODE_TX = 0x10,  //!< If the CAD operation does not detect an activity, the chip enters
                                                //!< in TX mode. Otherwise it enters in fallback mode
} lr20xx_radio_lora_cad_exit_mode_t;

/**
 * @brief LoRa intra-packet frequency hopping control
 */
typedef enum
{
    LR20XX_RADIO_LORA_HOPPING_CTRL_DISABLED = 0x00,  //!< LoRa intra-packet frequency hopping disabled
    LR20XX_RADIO_LORA_HOPPING_CTRL_ENABLED  = 0x01,  //!< LoRa intra-packet frequency hopping enabled
} lr20xx_radio_lora_hooping_ctrl_t;

/**
 * @brief LoRa Channel Activity Detection (CAD) parameters
 *
 * Parameter @ref lr20xx_radio_lora_cad_params_s.cad_detect_peak is used to tune the sensitivity of Channel Activity
 * Detection. It depends on Spreading Factor and @ref lr20xx_radio_lora_cad_params_s.cad_symb_nb.
 * Increasing value of lr20xx_radio_lora_cad_params_s.cad_detect_peak decreases CAD sensitivity.
 * Decreasing value of lr20xx_radio_lora_cad_params_s.cad_detect_peak increases CAD sensitivity, but increase the false
 * detections.
 *
 * Recommended values for @ref lr20xx_radio_lora_cad_params_s.cad_detect_peak, depending on @ref
 * lr20xx_radio_lora_cad_params_s.cad_symb_nb and the configured Spreading Factor are:
 *
 * | Spreading factor            | 1 symbol | 2 symbols | 3 symbols | 4 symbols |
 * | --------------------------- | -------- | --------- | --------- | --------- |
 * | @ref LR20XX_RADIO_LORA_SF5  | 60       | 56        | 51        | 51        |
 * | @ref LR20XX_RADIO_LORA_SF6  | 60       | 56        | 51        | 51        |
 * | @ref LR20XX_RADIO_LORA_SF7  | 60       | 56        | 52        | 51        |
 * | @ref LR20XX_RADIO_LORA_SF8  | 64       | 58        | 54        | 54        |
 * | @ref LR20XX_RADIO_LORA_SF9  | 64       | 58        | 56        | 56        |
 * | @ref LR20XX_RADIO_LORA_SF10 | 66       | 60        | 60        | 60        |
 * | @ref LR20XX_RADIO_LORA_SF11 | 70       | 64        | 60        | 60        |
 * | @ref LR20XX_RADIO_LORA_SF12 | 74       | 68        | 65        | 64        |
 *
 * The CAD can be configured in best-effort CAD operation to allow early CAD operation to stop if a clear non-detection
 * is obtained. The best-effort CAD is enabled by setting @ref lr20xx_radio_lora_cad_params_s.pnr_delta to a non-zero
 * value. The recommended  @ref lr20xx_radio_lora_cad_params_s.pnr_delta value to use for best-effort CAD is 8, and 0 to
 * disable the best-effort CAD.
 *
 * @ref lr20xx_radio_lora_cad_params_s.cad_timeout_in_pll_step is given in PPL step of 31.25us.
 */
typedef struct lr20xx_radio_lora_cad_params_s
{
    uint8_t cad_symb_nb;  //!< Number of symbols to search for CAD operation
    uint8_t pnr_delta;    //!< Peak to Noise Ratio. Possible values are:
                          //!< - 0: then the exact number of requested symbols @p cad_symb_nb is used to determine the
                          //!< activity detection;
                          //!< - 8: best-effort CAD is activated.
    lr20xx_radio_lora_cad_exit_mode_t cad_exit_mode;  //!< Action taken automatically at the end of CAD operation
    uint32_t cad_timeout_in_pll_step;  //!< Timeout in PLL steps used while in exit mode, if applicable. Max value is
                                       //!< 0x00FFFFFF PLL steps
    uint8_t cad_detect_peak;  //!< Ratio for CAD between correlator peak and average to identify a peak as a detection
                              //!< (default: 0x32)
} lr20xx_radio_lora_cad_params_t;

/**
 * @brief Packet parameters for LoRa packet
 */
typedef struct
{
    uint16_t                     preamble_len_in_symb;  //!< LoRa Preamble length [symbols]
    lr20xx_radio_lora_pkt_mode_t pkt_mode;              //!< LoRa packet mode configuration
    uint8_t                      pld_len_in_bytes;      //!< LoRa Payload length [bytes]
    lr20xx_radio_lora_crc_t      crc;                   //!< LoRa CRC configuration
    lr20xx_radio_lora_iq_t       iq;                    //!< LoRa IQ configuration
} lr20xx_radio_lora_pkt_params_t;

/**
 * @brief Modulation configuration for LoRa packet
 *
 * Refer to @ref lr20xx_radio_lora_get_recommended_ppm_offset for recommended values concerning @p ppm field.
 */
typedef struct
{
    lr20xx_radio_lora_sf_t  sf;   //!< Spreading factor
    lr20xx_radio_lora_bw_t  bw;   //!< Bandwidth
    lr20xx_radio_lora_cr_t  cr;   //!< Coding rate
    lr20xx_radio_lora_ppm_t ppm;  //!< PPM offset
} lr20xx_radio_lora_mod_params_t;

/**
 * @brief Reception statistics for LoRa packet
 */
typedef struct lr20xx_radio_lora_rx_statistics_s
{
    uint16_t n_received_packets;       //!< Number of received packets
    uint16_t n_crc_errors;             //!< Number of received packets with CRC error
    uint16_t n_header_errors;          //!< Number of received packets with header error (Rx configured in
                                       //!< LR20XX_RADIO_LORA_PKT_EXPLICIT and header CRC check failed)
    uint16_t n_false_synchronisation;  //!< Number of false synchronisation (preamble detected but syncword not
                                       //!< detected, probably preamble detected on noise)
} lr20xx_radio_lora_rx_statistics_t;

/**
 * @brief LoRa packet status fields
 */
typedef struct lr20xx_radio_lora_packet_status_s
{
    uint8_t                 packet_length_bytes;  //!< Length of last received packet in bytes
    lr20xx_radio_lora_crc_t crc;                  //!< CRC presence of the received packet
    lr20xx_radio_lora_cr_t  cr;                   //!< Coding rate of the received packet
    uint8_t                 detector;             //!< Indentifier of detectors that received / detected the packet
    int16_t rssi_pkt_in_dbm;          //!< Average energy in dBm at the input of the chip over the last packet received
    uint8_t rssi_pkt_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_pkt_in_dbm value in dBm
    int8_t  snr_pkt_raw;              //!< Estimation of the SNR on last packet received expressed in 0.25dB
    int16_t
        rssi_signal_pkt_in_dbm;  //!< Estimation of the mean energy of the LoRa signal over the last packet received.
                                 //!< Equivalent to rssi_pkt_in_dbm if snr_pkt_raw is positive, to rssi_pkt_in_dbm
                                 //!< + (snr_pkt_raw/4) if snr_pkt_raw is negative
    uint8_t rssi_signal_pkt_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_signal_pkt_in_dbm value in dBm
} lr20xx_radio_lora_packet_status_t;

/**
 * @brief LoRa intra-packet hopping configuration
 */
typedef struct
{
    lr20xx_radio_lora_hooping_ctrl_t hop_ctrl;  //!< LoRa intra-packet frequency hopping control
    uint16_t  hop_period;  //!< Number of LoRa symbols between two RF frequency changes (valid values in [0:16383])
    uint32_t* freq_hop;    //!< List of frequencies. It is up to the caller to ensure that the array pointed to contains
                           //!< at least nb_freq_hop items
    uint8_t nb_freq_hop;   //!< Number of frequencies in @ref freq_hop. Possible value in [0:40] included
} lr20xx_radio_lora_hopping_cfg_t;

/**
 * @brief Configuration structure of Channel Activity Detection (CAD) for side detectors
 */
typedef struct
{
    uint8_t pnr_delta;
    uint8_t det_peak;
} lr20xx_radio_lora_side_detector_cad_configuration_t;

/**
 * @brief Configuration of LoRa side detector
 */
typedef struct
{
    lr20xx_radio_lora_sf_t  sf;   //!< Spreading factor
    lr20xx_radio_lora_ppm_t ppm;  //!< PPM offset
    lr20xx_radio_lora_iq_t  iq;   //!< LoRa IQ configuration
} lr20xx_radio_lora_side_detector_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_LORA_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
