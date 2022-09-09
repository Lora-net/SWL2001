/**
 * @file      sx126x.h
 *
 * @brief     SX126x radio driver definition
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

#ifndef SX126X_H
#define SX126X_H

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Maximum value for parameter timeout_in_rtc_step in both functions @ref sx126x_set_rx_with_timeout_in_rtc_step
 * and @ref sx126x_set_tx_with_timeout_in_rtc_step
 */
#define SX126X_MAX_TIMEOUT_IN_RTC_STEP 0x00FFFFFE

/**
 * @brief  Maximum value for parameter timeout_in_ms in both functions @ref sx126x_set_rx and @ref sx126x_set_tx
 */
#define SX126X_MAX_TIMEOUT_IN_MS ( SX126X_MAX_TIMEOUT_IN_RTC_STEP / 64 )

/**
 * @brief Timeout parameter in \ref sx126x_set_rx_with_timeout_in_rtc_step to set the chip in reception until a
 * reception occurs
 */
#define SX126X_RX_SINGLE_MODE 0x00000000

/**
 * @brief Timeout parameter in @ref sx126x_set_rx_with_timeout_in_rtc_step to launch a continuous reception
 */
#define SX126X_RX_CONTINUOUS 0x00FFFFFF

/**
 * @brief Over-current protection default value after @ref sx126x_set_pa_cfg is called with @ref device_sel set to 1
 */
#define SX126X_OCP_PARAM_VALUE_60_MA 0x18

/**
 * @brief Over-current protection default value after @ref sx126x_set_pa_cfg is called with @ref device_sel set to 0
 */
#define SX126X_OCP_PARAM_VALUE_140_MA 0x38

/**
 * @brief XTA and XTB trimming capacitor default value after the chip entered @ref SX126X_STANDBY_CFG_XOSC mode
 */
#define SX126X_XTAL_TRIMMING_CAPACITOR_DEFAULT_VALUE_STDBY_XOSC 0x12

/**
 * @brief  Maximum value for parameter nb_of_symbs in @ref sx126x_set_lora_symb_nb_timeout
 */
#define SX126X_MAX_LORA_SYMB_NUM_TIMEOUT 248

/**
 * @brief Maximum number of register that can be added to the retention list
 */
#define SX126X_MAX_NB_REG_IN_RETENTION 4

/*!
 * @brief Frequency step in MHz used to compute the image calibration parameter
 *
 * @see sx126x_cal_img_in_mhz
 */
#define SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ 4

#define SX126X_CHIP_MODES_POS ( 4U )
#define SX126X_CHIP_MODES_MASK ( 0x07UL << SX126X_CHIP_MODES_POS )

#define SX126X_CMD_STATUS_POS ( 1U )
#define SX126X_CMD_STATUS_MASK ( 0x07UL << SX126X_CMD_STATUS_POS )

#define SX126X_GFSK_RX_STATUS_PKT_SENT_POS ( 0U )
#define SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_SENT_POS )

#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS ( 1U )
#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS )

#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS ( 2U )
#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS ( 3U )
#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_CRC_ERROR_POS ( 4U )
#define SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_CRC_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS ( 5U )
#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SX126X APIs return status enumeration definition
 */
typedef enum sx126x_status_e
{
    SX126X_STATUS_OK = 0,
    SX126X_STATUS_UNSUPPORTED_FEATURE,
    SX126X_STATUS_UNKNOWN_VALUE,
    SX126X_STATUS_ERROR,
} sx126x_status_t;

/**
 * @brief SX126X sleep mode configurations definition
 */
typedef enum sx126x_sleep_cfgs_e
{
    SX126X_SLEEP_CFG_COLD_START = ( 0 << 2 ),
    SX126X_SLEEP_CFG_WARM_START = ( 1 << 2 ),
} sx126x_sleep_cfgs_t;

/**
 * @brief SX126X standby modes enumeration definition
 */
typedef enum sx126x_standby_cfgs_e
{
    SX126X_STANDBY_CFG_RC   = 0x00,
    SX126X_STANDBY_CFG_XOSC = 0x01,
} sx126x_standby_cfgs_t;

typedef uint8_t sx126x_standby_cfg_t;

/**
 * @brief SX126X power regulator modes enumeration definition
 */
typedef enum sx126x_reg_mods_e
{
    SX126X_REG_MODE_LDO  = 0x00,  // default
    SX126X_REG_MODE_DCDC = 0x01,
} sx126x_reg_mod_t;

/**
 * @brief SX126X power amplifier configuration parameters structure definition
 */
typedef struct sx126x_pa_cfg_params_s
{
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
} sx126x_pa_cfg_params_t;

/**
 * @brief SX126X fallback modes enumeration definition
 */
typedef enum sx126x_fallback_modes_e
{
    SX126X_FALLBACK_STDBY_RC   = 0x20,
    SX126X_FALLBACK_STDBY_XOSC = 0x30,
    SX126X_FALLBACK_FS         = 0x40,
} sx126x_fallback_modes_t;

/**
 * @brief SX126X interrupt masks enumeration definition
 */
enum sx126x_irq_masks_e
{
    SX126X_IRQ_NONE              = ( 0 << 0 ),
    SX126X_IRQ_TX_DONE           = ( 1 << 0 ),
    SX126X_IRQ_RX_DONE           = ( 1 << 1 ),
    SX126X_IRQ_PREAMBLE_DETECTED = ( 1 << 2 ),
    SX126X_IRQ_SYNC_WORD_VALID   = ( 1 << 3 ),
    SX126X_IRQ_HEADER_VALID      = ( 1 << 4 ),
    SX126X_IRQ_HEADER_ERROR      = ( 1 << 5 ),
    SX126X_IRQ_CRC_ERROR         = ( 1 << 6 ),
    SX126X_IRQ_CAD_DONE          = ( 1 << 7 ),
    SX126X_IRQ_CAD_DETECTED      = ( 1 << 8 ),
    SX126X_IRQ_TIMEOUT           = ( 1 << 9 ),
    SX126X_IRQ_LR_FHSS_HOP       = ( 1 << 14 ),
    SX126X_IRQ_ALL               = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                     SX126X_IRQ_SYNC_WORD_VALID | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_HEADER_ERROR |
                     SX126X_IRQ_CRC_ERROR | SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_TIMEOUT |
                     SX126X_IRQ_LR_FHSS_HOP,
};

typedef uint16_t sx126x_irq_mask_t;

/**
 * @brief Calibration settings
 */
enum sx126x_cal_mask_e
{
    SX126X_CAL_RC64K      = ( 1 << 0 ),
    SX126X_CAL_RC13M      = ( 1 << 1 ),
    SX126X_CAL_PLL        = ( 1 << 2 ),
    SX126X_CAL_ADC_PULSE  = ( 1 << 3 ),
    SX126X_CAL_ADC_BULK_N = ( 1 << 4 ),
    SX126X_CAL_ADC_BULK_P = ( 1 << 5 ),
    SX126X_CAL_IMAGE      = ( 1 << 6 ),
    SX126X_CAL_ALL        = SX126X_CAL_RC64K | SX126X_CAL_RC13M | SX126X_CAL_PLL | SX126X_CAL_ADC_PULSE |
                     SX126X_CAL_ADC_BULK_N | SX126X_CAL_ADC_BULK_P | SX126X_CAL_IMAGE,
};

typedef uint8_t sx126x_cal_mask_t;

/**
 * @brief SX126X TCXO control voltages enumeration definition
 */
typedef enum sx126x_tcxo_ctrl_voltages_e
{
    SX126X_TCXO_CTRL_1_6V = 0x00,
    SX126X_TCXO_CTRL_1_7V = 0x01,
    SX126X_TCXO_CTRL_1_8V = 0x02,
    SX126X_TCXO_CTRL_2_2V = 0x03,
    SX126X_TCXO_CTRL_2_4V = 0x04,
    SX126X_TCXO_CTRL_2_7V = 0x05,
    SX126X_TCXO_CTRL_3_0V = 0x06,
    SX126X_TCXO_CTRL_3_3V = 0x07,
} sx126x_tcxo_ctrl_voltages_t;

/**
 * @brief SX126X packet types enumeration definition
 */
typedef enum sx126x_pkt_types_e
{
    SX126X_PKT_TYPE_GFSK    = 0x00,
    SX126X_PKT_TYPE_LORA    = 0x01,
    SX126X_PKT_TYPE_LR_FHSS = 0x03,
} sx126x_pkt_type_t;

/**
 * @brief SX126X power amplifier ramp-up timings enumeration definition
 */
typedef enum sx126x_ramp_time_e
{
    SX126X_RAMP_10_US   = 0x00,
    SX126X_RAMP_20_US   = 0x01,
    SX126X_RAMP_40_US   = 0x02,
    SX126X_RAMP_80_US   = 0x03,
    SX126X_RAMP_200_US  = 0x04,
    SX126X_RAMP_800_US  = 0x05,
    SX126X_RAMP_1700_US = 0x06,
    SX126X_RAMP_3400_US = 0x07,
} sx126x_ramp_time_t;

/**
 * @brief SX126X GFSK modulation shaping enumeration definition
 */
typedef enum sx126x_gfsk_pulse_shape_e
{
    SX126X_GFSK_PULSE_SHAPE_OFF   = 0x00,
    SX126X_GFSK_PULSE_SHAPE_BT_03 = 0x08,
    SX126X_GFSK_PULSE_SHAPE_BT_05 = 0x09,
    SX126X_GFSK_PULSE_SHAPE_BT_07 = 0x0A,
    SX126X_GFSK_PULSE_SHAPE_BT_1  = 0x0B,
} sx126x_gfsk_pulse_shape_t;

/**
 * @brief SX126X GFSK Rx bandwidth enumeration definition
 */
typedef enum sx126x_gfsk_bw_e
{
    SX126X_GFSK_BW_4800   = 0x1F,
    SX126X_GFSK_BW_5800   = 0x17,
    SX126X_GFSK_BW_7300   = 0x0F,
    SX126X_GFSK_BW_9700   = 0x1E,
    SX126X_GFSK_BW_11700  = 0x16,
    SX126X_GFSK_BW_14600  = 0x0E,
    SX126X_GFSK_BW_19500  = 0x1D,
    SX126X_GFSK_BW_23400  = 0x15,
    SX126X_GFSK_BW_29300  = 0x0D,
    SX126X_GFSK_BW_39000  = 0x1C,
    SX126X_GFSK_BW_46900  = 0x14,
    SX126X_GFSK_BW_58600  = 0x0C,
    SX126X_GFSK_BW_78200  = 0x1B,
    SX126X_GFSK_BW_93800  = 0x13,
    SX126X_GFSK_BW_117300 = 0x0B,
    SX126X_GFSK_BW_156200 = 0x1A,
    SX126X_GFSK_BW_187200 = 0x12,
    SX126X_GFSK_BW_234300 = 0x0A,
    SX126X_GFSK_BW_312000 = 0x19,
    SX126X_GFSK_BW_373600 = 0x11,
    SX126X_GFSK_BW_467000 = 0x09,
} sx126x_gfsk_bw_t;

/**
 * @brief SX126X GFSK modulation parameters structure definition
 */
typedef struct sx126x_mod_params_gfsk_s
{
    uint32_t                  br_in_bps;
    uint32_t                  fdev_in_hz;
    sx126x_gfsk_pulse_shape_t pulse_shape;
    sx126x_gfsk_bw_t          bw_dsb_param;
} sx126x_mod_params_gfsk_t;

/**
 * @brief SX126X LoRa spreading factor enumeration definition
 */
typedef enum sx126x_lora_sf_e
{
    SX126X_LORA_SF5  = 0x05,
    SX126X_LORA_SF6  = 0x06,
    SX126X_LORA_SF7  = 0x07,
    SX126X_LORA_SF8  = 0x08,
    SX126X_LORA_SF9  = 0x09,
    SX126X_LORA_SF10 = 0x0A,
    SX126X_LORA_SF11 = 0x0B,
    SX126X_LORA_SF12 = 0x0C,
} sx126x_lora_sf_t;

/**
 * @brief SX126X LoRa bandwidth enumeration definition
 */
typedef enum sx126x_lora_bw_e
{
    SX126X_LORA_BW_500 = 6,
    SX126X_LORA_BW_250 = 5,
    SX126X_LORA_BW_125 = 4,
    SX126X_LORA_BW_062 = 3,
    SX126X_LORA_BW_041 = 10,
    SX126X_LORA_BW_031 = 2,
    SX126X_LORA_BW_020 = 9,
    SX126X_LORA_BW_015 = 1,
    SX126X_LORA_BW_010 = 8,
    SX126X_LORA_BW_007 = 0,
} sx126x_lora_bw_t;

/**
 * @brief SX126X LoRa coding rate enumeration definition
 */
typedef enum sx126x_lora_cr_e
{
    SX126X_LORA_CR_4_5 = 0x01,
    SX126X_LORA_CR_4_6 = 0x02,
    SX126X_LORA_CR_4_7 = 0x03,
    SX126X_LORA_CR_4_8 = 0x04,
} sx126x_lora_cr_t;

/**
 * @brief SX126X LoRa modulation parameters structure definition
 */
typedef struct sx126x_mod_params_lora_s
{
    sx126x_lora_sf_t sf;    //!< LoRa Spreading Factor
    sx126x_lora_bw_t bw;    //!< LoRa Bandwidth
    sx126x_lora_cr_t cr;    //!< LoRa Coding Rate
    uint8_t          ldro;  //!< Low DataRate Optimization configuration
} sx126x_mod_params_lora_t;

/**
 * @brief SX126X GFSK preamble length Rx detection size enumeration definition
 */
typedef enum sx126x_gfsk_preamble_detector_e
{
    SX126X_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07,
} sx126x_gfsk_preamble_detector_t;

/**
 * @brief SX126X GFSK address filtering configuration enumeration definition
 */
typedef enum sx126x_gfsk_address_filtering_e
{
    SX126X_GFSK_ADDRESS_FILTERING_DISABLE                      = 0x00,
    SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS                 = 0x01,
    SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES = 0x02,
} sx126x_gfsk_address_filtering_t;

/**
 * @brief SX126X GFSK packet length enumeration definition
 */
typedef enum sx126x_gfsk_pkt_len_modes_e
{
    SX126X_GFSK_PKT_FIX_LEN = 0x00,  //!< The packet length is known on both sides, no header included
    SX126X_GFSK_PKT_VAR_LEN = 0x01,  //!< The packet length is variable, header included
} sx126x_gfsk_pkt_len_modes_t;

/**
 * @brief SX126X GFSK CRC type enumeration definition
 */
typedef enum sx126x_gfsk_crc_types_e
{
    SX126X_GFSK_CRC_OFF         = 0x01,
    SX126X_GFSK_CRC_1_BYTE      = 0x00,
    SX126X_GFSK_CRC_2_BYTES     = 0x02,
    SX126X_GFSK_CRC_1_BYTE_INV  = 0x04,
    SX126X_GFSK_CRC_2_BYTES_INV = 0x06,
} sx126x_gfsk_crc_types_t;

/**
 * @brief SX126X GFSK whitening control enumeration definition
 */
typedef enum sx126x_gfsk_dc_free_e
{
    SX126X_GFSK_DC_FREE_OFF       = 0x00,
    SX126X_GFSK_DC_FREE_WHITENING = 0x01,
} sx126x_gfsk_dc_free_t;

/**
 * @brief SX126X LoRa packet length enumeration definition
 */
typedef enum sx126x_lora_pkt_len_modes_e
{
    SX126X_LORA_PKT_EXPLICIT = 0x00,  //!< Header included in the packet
    SX126X_LORA_PKT_IMPLICIT = 0x01,  //!< Header not included in the packet
} sx126x_lora_pkt_len_modes_t;

/**
 * @brief SX126X LoRa packet parameters structure definition
 */
typedef struct sx126x_pkt_params_lora_s
{
    uint16_t                    preamble_len_in_symb;  //!< Preamble length in symbols
    sx126x_lora_pkt_len_modes_t header_type;           //!< Header type
    uint8_t                     pld_len_in_bytes;      //!< Payload length in bytes
    bool                        crc_is_on;             //!< CRC activation
    bool                        invert_iq_is_on;       //!< IQ polarity setup
} sx126x_pkt_params_lora_t;

/**
 * @brief SX126X GFSK packet parameters structure definition
 */
typedef struct sx126x_pkt_params_gfsk_s
{
    uint16_t                        preamble_len_in_bits;   //!< Preamble length in bits
    sx126x_gfsk_preamble_detector_t preamble_detector;      //!< Preamble detection length
    uint8_t                         sync_word_len_in_bits;  //!< Sync word length in bits
    sx126x_gfsk_address_filtering_t address_filtering;      //!< Address filtering configuration
    sx126x_gfsk_pkt_len_modes_t     header_type;            //!< Header type
    uint8_t                         pld_len_in_bytes;       //!< Payload length in bytes
    sx126x_gfsk_crc_types_t         crc_type;               //!< CRC type configuration
    sx126x_gfsk_dc_free_t           dc_free;                //!< Whitening configuration
} sx126x_pkt_params_gfsk_t;

/**
 * @brief SX126X LoRa CAD number of symbols enumeration definition
 *
 * @note Represents the number of symbols to be used for a CAD operation
 */
typedef enum sx126x_cad_symbs_e
{
    SX126X_CAD_01_SYMB = 0x00,
    SX126X_CAD_02_SYMB = 0x01,
    SX126X_CAD_04_SYMB = 0x02,
    SX126X_CAD_08_SYMB = 0x03,
    SX126X_CAD_16_SYMB = 0x04,
} sx126x_cad_symbs_t;

/**
 * @brief SX126X LoRa CAD exit modes enumeration definition
 *
 * @note Represents the action to be performed after a CAD is done
 */
typedef enum sx126x_cad_exit_modes_e
{
    SX126X_CAD_ONLY = 0x00,
    SX126X_CAD_RX   = 0x01,
    SX126X_CAD_LBT  = 0x10,
} sx126x_cad_exit_modes_t;

/**
 * @brief SX126X CAD parameters structure definition
 */
typedef struct sx126x_cad_param_s
{
    sx126x_cad_symbs_t      cad_symb_nb;      //!< CAD number of symbols
    uint8_t                 cad_detect_peak;  //!< CAD peak detection
    uint8_t                 cad_detect_min;   //!< CAD minimum detection
    sx126x_cad_exit_modes_t cad_exit_mode;    //!< CAD exit mode
    uint32_t                cad_timeout;      //!< CAD timeout value
} sx126x_cad_params_t;

/**
 * @brief SX126X chip mode enumeration definition
 */
typedef enum sx126x_chip_modes_e
{
    SX126X_CHIP_MODE_UNUSED    = 0,
    SX126X_CHIP_MODE_RFU       = 1,
    SX126X_CHIP_MODE_STBY_RC   = 2,
    SX126X_CHIP_MODE_STBY_XOSC = 3,
    SX126X_CHIP_MODE_FS        = 4,
    SX126X_CHIP_MODE_RX        = 5,
    SX126X_CHIP_MODE_TX        = 6,
} sx126x_chip_modes_t;

/**
 * @brief SX126X command status enumeration definition
 */
typedef enum sx126x_cmd_status_e
{
    SX126X_CMD_STATUS_RESERVED          = 0,
    SX126X_CMD_STATUS_RFU               = 1,
    SX126X_CMD_STATUS_DATA_AVAILABLE    = 2,
    SX126X_CMD_STATUS_CMD_TIMEOUT       = 3,
    SX126X_CMD_STATUS_CMD_PROCESS_ERROR = 4,
    SX126X_CMD_STATUS_CMD_EXEC_FAILURE  = 5,
    SX126X_CMD_STATUS_CMD_TX_DONE       = 6,
} sx126x_cmd_status_t;

/**
 * @brief SX126X chip status structure definition
 */
typedef struct sx126x_chip_status_s
{
    sx126x_cmd_status_t cmd_status;  //!< Previous command status
    sx126x_chip_modes_t chip_mode;   //!< Current chip mode
} sx126x_chip_status_t;

/**
 * @brief SX126X RX buffer status structure definition
 */
typedef struct sx126x_rx_buffer_status_s
{
    uint8_t pld_len_in_bytes;      //!< Number of bytes available in the buffer
    uint8_t buffer_start_pointer;  //!< Position of the first byte in the buffer
} sx126x_rx_buffer_status_t;

typedef struct sx126x_rx_status_gfsk_s
{
    bool pkt_sent;
    bool pkt_received;
    bool abort_error;
    bool length_error;
    bool crc_error;
    bool adrs_error;
} sx126x_rx_status_gfsk_t;

/**
 * @brief SX126X GFSK packet status structure definition
 */
typedef struct sx126x_pkt_status_gfsk_s
{
    sx126x_rx_status_gfsk_t rx_status;
    int8_t                  rssi_sync;  //!< The RSSI measured on last packet
    int8_t                  rssi_avg;   //!< The averaged RSSI
} sx126x_pkt_status_gfsk_t;

/**
 * @brief SX126X LoRa packet status structure definition
 */
typedef struct sx126x_pkt_status_lora_s
{
    int8_t rssi_pkt_in_dbm;         //!< RSSI of the last packet
    int8_t snr_pkt_in_db;           //!< SNR of the last packet
    int8_t signal_rssi_pkt_in_dbm;  //!< Estimation of RSSI (after despreading)
} sx126x_pkt_status_lora_t;

/**
 * @brief SX126X GFSK reception statistics structure definition
 */
typedef struct sx126x_stats_gfsk_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_len_error;
} sx126x_stats_gfsk_t;

/**
 * @brief SX126X LoRa reception statistics structure definition
 */
typedef struct sx126x_stats_lora_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_header_error;
} sx126x_stats_lora_t;

/**
 * @brief SX126X errors enumeration definition
 */
enum sx126x_errors_e
{
    SX126X_ERRORS_RC64K_CALIBRATION = ( 1 << 0 ),
    SX126X_ERRORS_RC13M_CALIBRATION = ( 1 << 1 ),
    SX126X_ERRORS_PLL_CALIBRATION   = ( 1 << 2 ),
    SX126X_ERRORS_ADC_CALIBRATION   = ( 1 << 3 ),
    SX126X_ERRORS_IMG_CALIBRATION   = ( 1 << 4 ),
    SX126X_ERRORS_XOSC_START        = ( 1 << 5 ),
    SX126X_ERRORS_PLL_LOCK          = ( 1 << 6 ),
    SX126X_ERRORS_PA_RAMP           = ( 1 << 8 ),
};

typedef uint16_t sx126x_errors_mask_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

//
// Operational Modes Functions
//

/**
 * @brief Set the chip in sleep mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Sleep mode configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg );

/**
 * @brief Set the chip in stand-by mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Stand-by mode configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg );

/**
 * @brief Set the chip in frequency synthesis mode
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_fs( const void* context );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms The timeout configuration in millisecond for Tx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx( const void* context, const uint32_t timeout_in_ms );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration can be computed with the formula:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times * \frac{1}{64} \f$
 *
 * @remark Maximal value is SX126X_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms)
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step The timeout configuration for Tx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values        | Meaning                                                                               |
 * | ----------------------| --------------------------------------------------------------------------------------|
 * | SX126X_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
 *
 * @remark Refer to @ref sx126x_handle_rx_done
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms The timeout configuration in millisecond for Rx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx( const void* context, const uint32_t timeout_in_ms );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration is obtained by:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is SX126X_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms).
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values        | Meaning                                                                               |
 * | ----------------------| --------------------------------------------------------------------------------------|
 * | SX126X_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
 * | SX126X_RX_CONTINUOUS  | Continuous: the chip stays in RX mode even after reception of a packet                |
 *
 * @remark Refer to @ref sx126x_handle_rx_done
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step The timeout configuration for Rx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 * @brief Configure the event on which the Rx timeout is stopped
 *
 * @remark The two options are:
 *   - Syncword / Header detection (default)
 *   - Preamble detection
 *
 * @param [in] context Chip implementation context
 * @param [in] enable If true, the timer stops on Syncword / Header detection
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_time_in_ms The timeout of Rx period - in millisecond
 * @param [in] sleep_time_in_ms The length of sleep period - in millisecond
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @remark The Rx mode duration is defined by:
 * \f$ rx\_duration\_ms = rx_time \times \frac{1}{64} \f$
 *
 * @remark The sleep mode duration is defined by:
 * \f$ sleep\_duration\_ms = sleep_time \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is 0xFFFFFF (i.e. 511 seconds).
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_time The timeout of Rx period
 * @param [in] sleep_time The length of sleep period
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step );

/**
 * @brief Set the chip in CAD (Channel Activity Detection) mode
 *
 * @remark The LoRa packet type shall be selected with @ref sx126x_set_pkt_type before this function is called.
 *
 * @remark The fallback mode is configured with @ref sx126x_set_cad_params.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_cad( const void* context );

/**
 * @brief Set the chip in Tx continuous wave (RF tone).
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_cw( const void* context );

/**
 * @brief Set the chip in Tx infinite preamble (modulated signal).
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context );

/**
 * @brief Configure the regulator mode to be used
 *
 * @remark This function shall be called to set the correct regulator mode, depending on the usage of LDO or DC/DC on
 * the PCB implementation.
 *
 * @param [in] context Chip implementation context
 * @param [in] mode Regulator mode configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode );

/**
 * @brief Perform the calibration of the requested blocks
 *
 * @remark This function shall only be called in stand-by RC mode
 *
 * @remark The chip will return to stand-by RC mode on exit. Potential calibration issues can be read out with @ref
 * sx126x_get_device_errors command.
 *
 * @param [in] context Chip implementation context
 * @param [in] param Mask holding the blocks to be calibrated
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cal( const void* context, const sx126x_cal_mask_t param );

/**
 * @brief Launch an image calibration valid for all frequencies inside an interval, in steps
 *
 * @param [in] context Chip implementation context
 * @param [in] freq1 Image calibration interval lower bound, in steps
 * @param [in] freq2 Image calibration interval upper bound, in steps
 *
 * @remark freq1 must be less than or equal to freq2
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 );

/**
 * @brief Launch an image calibration valid for all frequencies inside an interval, in MHz
 *
 * @param [in] context Chip implementation context
 * @param [in] freq1_in_mhz Image calibration interval lower bound, in MHz
 * @param [in] freq2_in_mhz Image calibration interval upper bound, in MHz
 *
 * @remark freq1_in_mhz must be less than or equal to freq2_in_mhz
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz );

/**
 * @brief Configure the PA (Power Amplifier)
 *
 * @remark The parameters depend on the chip being used
 *
 * @param [in] context Chip implementation context
 * @param [in] params Power amplifier configuration parameters
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params );

/**
 * @brief Set chip mode to be used after successful transmission or reception.
 *
 * @remark This setting is not taken into account during Rx Duty Cycle mode or Auto TxRx.
 *
 * @param [in] context Chip implementation context
 * @param [in] fallback_mode Selected fallback mode
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode );

//
// Registers and Buffer Access
//

/**
 * @brief Write data into register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start writing operation
 * @param [in] buffer The buffer of bytes to write into memory
 * @param [in] size Number of bytes to write into memory, starting from address
 *
 * @returns Operation status
 *
 * @see sx126x_read_register
 */
sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size );

/**
 * @brief Read data from register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start reading operation
 * @param [in] buffer The buffer of bytes to be filled with data from registers
 * @param [in] size Number of bytes to read from memory, starting from address
 *
 * @returns Operation status
 *
 * @see sx126x_write_register
 */
sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint8_t size );

/**
 * @brief Write data into radio Tx buffer memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] offset Start address in the Tx buffer of the chip
 * @param [in] buffer The buffer of bytes to write into radio buffer
 * @param [in] size The number of bytes to write into Tx radio buffer
 *
 * @returns Operation status
 *
 * @see sx126x_read_buffer
 */
sx126x_status_t sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size );

/**
 * @brief Read data from radio Rx buffer memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] offset Start address in the Rx buffer of the chip
 * @param [in] buffer The buffer of bytes to be filled with content from Rx radio buffer
 * @param [in] size The number of bytes to read from the Rx radio buffer
 *
 * @returns Operation status
 *
 * @see sx126x_write_buffer
 */
sx126x_status_t sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );

//
// DIO and IRQ Control Functions
//

/**
 * @brief Set which interrupt signals are redirected to the dedicated DIO pin
 *
 * @remark By default, no interrupt signal is redirected.
 *
 * @remark An interrupt will not occur until it is enabled system-wide, even if it is redirected to a specific DIO.
 *
 * @remark The DIO pin will remain asserted until all redirected interrupt signals are cleared with a call to @ref
 * sx126x_clear_irq_status.
 *
 * @remark DIO2 and DIO3 are shared with other features. See @ref sx126x_set_dio2_as_rf_sw_ctrl and @ref
 * sx126x_set_dio3_as_tcxo_ctrl
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt mask
 * @param [in] dio1_mask Variable that holds the interrupt mask for dio1
 * @param [in] dio2_mask Variable that holds the interrupt mask for dio2
 * @param [in] dio3_mask Variable that holds the interrupt mask for dio3
 *
 * @returns Operation status
 *
 * @see sx126x_clear_irq_status, sx126x_get_irq_status, sx126x_set_dio2_as_rf_sw_ctrl, sx126x_set_dio3_as_tcxo_ctrl
 */
sx126x_status_t sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask );

/**
 * @brief Get system interrupt status
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status
 *
 * @returns Operation status
 *
 * @see sx126x_clear_irq_status
 */
sx126x_status_t sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq );

/**
 * @brief Clear selected system interrupts
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt to be cleared
 *
 * @returns Operation status
 *
 * @see sx126x_get_irq_status
 */
sx126x_status_t sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask );

/**
 * @brief Clears any radio irq status flags that are set and returns the flags that
 * were cleared.
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status; can be NULL
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq );

/**
 * @brief Configure the embedded RF switch control
 *
 * @param [in] context Chip implementation context
 * @param [in] enable Enable this feature if set to true
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable );

/**
 * @brief Configure the embedded TCXO switch control
 *
 * @remark This function shall only be called in standby RC mode.
 *
 * @remark The chip will wait for the timeout to happen before starting any operation that requires the TCXO.
 *
 * @param [in] context Chip implementation context
 * @param [in] tcxo_voltage Voltage used to power the TCXO
 * @param [in] timeout Time needed for the TCXO to be stable
 *
 * @returns Operation status
 *
 */
sx126x_status_t sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout );

//
// RF Modulation and Packet-Related Functions
//

/**
 * @brief Set the RF frequency for future radio operations.
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq_in_hz The frequency in Hz to set for radio operations
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz );

/**
 * @brief Set the RF frequency for future radio operations - parameter in PLL steps
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq The frequency in PLL steps to set for radio operations
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq );

/**
 * @brief Set the packet type
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] pkt_type Packet type to set
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type );

/**
 * @brief Get the current packet type
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_type Pointer to a variable holding the packet type
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type );

/**
 * @brief Set the parameters for TX power and power amplifier ramp time
 *
 * @param [in] context Chip implementation context
 * @param [in] pwr_in_dbm The Tx output power in dBm
 * @param [in] ramp_time The ramping time configuration for the PA
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                      const sx126x_ramp_time_t ramp_time );

/**
 * @brief Set the modulation parameters for GFSK packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of GFSK modulation configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params );

/**
 * @brief Set the modulation parameters for LoRa packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of LoRa modulation configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params );

/**
 * @brief Set the packet parameters for GFSK packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of GFSK packet configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params );

/**
 * @brief Set the packet parameters for LoRa packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of LoRa packet configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params );

/*!
 * @brief Set the Node and Broadcast address used for GFSK
 *
 * This setting is used only when filtering is enabled.
 *
 * @param [in] context Chip implementation context
 * @param [in] node_address The node address used as filter
 * @param [in] broadcast_address The broadcast address used as filter
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                             const uint8_t broadcast_address );

/**
 * @brief Set the parameters for CAD operation
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of CAD configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params );

/**
 * @brief Set buffer start addresses for both Tx and Rx operations
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_base_address The start address used for Tx operations
 * @param [in] rx_base_address The start address used for Rx operations
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address );

/**
 * @brief Set the timeout to be used when the chip is configured in Rx mode (only in LoRa)
 *
 * @remark The maximum timeout is \ref SX126X_MAX_LORA_SYMB_NUM_TIMEOUT
 * @remark The function is disabled if the timeout is set to 0
 *
 * @param [in] context Chip implementation context
 * @param [in] nb_of_symbs Timeout in number of symbol
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs );

//
// Communication Status Information
//

/**
 * @brief Get the chip status
 *
 * @param [in] context Chip implementation context
 * @param [out] radio_status Pointer to a structure holding the radio status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status );

/**
 * @brief Get the current Rx buffer status for both LoRa and GFSK Rx operations
 *
 * @details This function is used to get the length of the received payload and the start address to be used when
 * reading data from the Rx buffer.
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_buffer_status Pointer to a structure to store the current status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status );

/**
 * @brief Get the status of the last GFSK packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status );

/**
 * @brief Get the status of the last LoRa packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status );

/**
 * @brief Get the instantaneous RSSI value.
 *
 * @remark This function shall be called when in Rx mode.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi_in_dbm Pointer to a variable to store the RSSI value in dBm
 *
 * @returns Operation status
 *
 * @see sx126x_set_rx
 */
sx126x_status_t sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm );

/**
 * @brief Get the statistics about GFSK communication
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Pointer to a structure to store GFSK-related statistics
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats );

/**
 * @brief Get the statistics about LoRa communication
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Pointer to a structure to store LoRa-related statistics
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats );

/**
 * @brief Reset all the statistics for both Lora and GFSK communications
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_reset_stats( const void* context );

//
// Miscellaneous
//

/**
 * @brief Perform a hard reset of the chip
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_reset( const void* context );

/**
 * @brief Wake the radio up from sleep mode.
 *
 * @param [in]  context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_wakeup( const void* context );

/**
 * @brief Get the list of all active errors
 *
 * @param [in] context Chip implementation context
 * @param [out] errors Pointer to a variable to store the error list
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors );

/**
 * @brief Clear all active errors
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_clear_device_errors( const void* context );

/**
 * @brief Get the parameter corresponding to a GFSK Rx bandwith immediately above the minimum requested one.
 *
 * @param [in] bw Minimum required bandwith in Hz
 * @param [out] param Pointer to a value to store the parameter
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param );

/**
 * @brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * @param [in] bw LoRa bandwidth parameter
 *
 * @returns Actual LoRa bandwidth in Hertz
 */
uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw );

/**
 * @brief Compute the numerator for LoRa time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * @param [in] pkt_p Pointer to the structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to the structure holding the LoRa modulation parameters
 *
 * @returns LoRa time-on-air numerator
 */
uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p );

/**
 * @brief Get the time on air in ms for LoRa transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to a structure holding the LoRa modulation parameters
 *
 * @returns Time-on-air value in ms for LoRa transmission
 */
uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p );

/**
 * @brief Compute the numerator for GFSK time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * @param [in] pkt_p Pointer to the structure holding the GFSK packet parameters
 *
 * @returns GFSK time-on-air numerator
 */
uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p );

/**
 * @brief Get the time on air in ms for GFSK transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the GFSK packet parameters
 * @param [in] mod_p Pointer to a structure holding the GFSK modulation parameters
 *
 * @returns Time-on-air value in ms for GFSK transmission
 */
uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p );

/**
 * @brief Generate one or more 32-bit random numbers.
 *
 * @remark A valid packet type must have been configured with @ref sx126x_set_pkt_type
 *         before using this command.
 *
 * @param [in]  context Chip implementation context
 * @param [out] numbers Array where numbers will be stored
 * @param [in]  n Number of desired random numbers
 *
 * @returns Operation status
 *
 * This code can potentially result in interrupt generation. It is the responsibility of
 * the calling code to disable radio interrupts before calling this function,
 * and re-enable them afterwards if necessary, or be certain that any interrupts
 * generated during this process will not cause undesired side-effects in the software.
 *
 * Please note that the random numbers produced by the generator do not have a uniform or Gaussian distribution. If
 * uniformity is needed, perform appropriate software post-processing.
 */
sx126x_status_t sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n );

/**
 * @brief Get the number of PLL steps for a given frequency in Hertz
 *
 * @param [in] freq_in_hz Frequency in Hertz
 *
 * @returns Number of PLL steps
 */
uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz );

/**
 * @brief Get the number of RTC steps for a given timeout in millisecond
 *
 * @param [in] timeout_in_ms Timeout in millisecond
 *
 * @returns Number of RTC steps
 */
uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms );

/**
 * @brief Generic finalizing function after reception
 *
 * @remark This function can be called after any reception sequence and must be called after any reception with timeout
 * active sequence.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_handle_rx_done( const void* context );

//
// Registers access
//

/**
 * @brief Configure the boost mode in reception
 *
 * @remark This configuration is not kept in the retention memory. Rx boosted mode shall be enabled each time the chip
 * leaves sleep mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] state Boost mode activation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cfg_rx_boosted( const void* context, const bool state );

/**
 * @brief Configure the sync word used in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Buffer holding the sync word to be configured
 * @param [in] sync_word_len Sync word length in byte
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len );

/**
 * @brief Configure the sync word used in LoRa packet
 *
 * @remark In the case of a LoRaWAN use case, the two following values are specified:
 *   - 0x12 for a private LoRaWAN network (default)
 *   - 0x34 for a public LoRaWAN network
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Sync word to be configured
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word );

/**
 * @brief Configure the seed used to compute CRC in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used to compute the CRC value
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed );

/**
 * @brief Configure the polynomial used to compute CRC in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] polynomial Polynomial value used to compute the CRC value
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial );

/**
 * @brief Configure the whitening seed used in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used in data whitening
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed );

/**
 * @brief Configure the Tx PA clamp
 *
 * @remark Workaround - With a SX1262, during the chip initialization, calling this function optimizes the PA clamping
 * threshold. The call must be done after a Power On Reset or a wake-up from cold start (see DS_SX1261-2_V1.2 datasheet
 * chapter 15.2)
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cfg_tx_clamp( const void* context );

/**
 * @brief Stop the RTC and clear the related event
 *
 * @remark Workaround - It is advised to call this function after ANY reception with timeout active sequence, which
 * stop the RTC and clear the timeout event, if any (see DS_SX1261-2_V1.2 datasheet chapter 15.4)
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_stop_rtc( const void* context );

/**
 * @brief Configure the Over Current Protection (OCP) value
 *
 * @remark The maximum value that can be configured is 63 (i.e. 157.5 mA)
 *
 * @param [in] context Chip implementation context
 * @param [in] ocp_in_step_of_2_5_ma OCP value given in steps of 2.5 mA
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma );

/**
 * @brief Configure the internal trimming capacitor values
 *
 * @remark The device is fitted with internal programmable capacitors connected independently to the pins XTA and XTB of
 * the device. Each capacitor can be controlled independently in steps of 0.47 pF added to the minimal value 11.3pF.
 *
 * @param [in] context Chip implementation context
 * @param [in] trimming_cap_xta Value for the trimming capacitor connected to XTA pin
 * @param [in] trimming_cap_xtb Value for the trimming capacitor connected to XTB pin
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb );

/**
 * @brief Add registers to the retention list
 *
 * @remark Up to 4 registers can be added to the retention list
 * @remark This function actually appends registers to the list until it is full
 * @remark Registers already added to the list cannot be removed unless the chip goes in sleep mode without retention or
 * a reset is issued
 *
 * @param [in] context Chip implementation context
 * @param [in] register_address The array with addresses of the register to be kept in retention
 * @param [in] register_nb The number of register to be kept in retention
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
                                                        uint8_t register_nb );

/**
 * @brief Add SX126X_REG_RXGAIN, SX126X_REG_TX_MODULATION and SX126X_REG_IQ_POLARITY registers to the retention list
 *
 * @remark These registers are used in workarounds implemented in this driver
 * @remark This function adds 3 registers out of the 4 available slots to the retention list
 * @remark It is recommended to call this function once during initialization phase if the application requires the chip
 * to enter sleep mode without retention
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @see sx126x_add_registers_to_retention_list
 */
sx126x_status_t sx126x_init_retention_list( const void* context );

/**
 * @brief Get LoRa coding rate and CRC configurations from received header
 *
 * @remark The output of this function is only valid if the field header_type of pkt_params is equal to @ref
 * SX126X_LORA_PKT_EXPLICIT when calling @ref sx126x_set_lora_pkt_params()
 * @remark The values for cr and crc_is_on are extracted from the header of the received LoRa packet
 *
 * @param [in]  context    Chip implementation context
 * @param [out] cr         LoRa coding rate
 * @param [out] crc_is_on  LoRa CRC configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_lora_params_from_header( const void* context, sx126x_lora_cr_t* cr, bool* crc_is_on );

#ifdef __cplusplus
}
#endif

#endif  // SX126X_H

/* --- EOF ------------------------------------------------------------------ */
