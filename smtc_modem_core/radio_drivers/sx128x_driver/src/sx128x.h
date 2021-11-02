/**
 * @file      sx128x.h
 *
 * @brief     SX128X radio driver definition
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

#ifndef SX128X_H
#define SX128X_H

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
 * @brief Minimum Tx power
 */
#define SX128X_PWR_MIN ( -18 )

/**
 * @brief Maximum Tx power
 */
#define SX128X_PWR_MAX ( 13 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief API return status
 */
typedef enum sx128x_status_e
{
    SX128X_STATUS_OK = 0,
    SX128X_STATUS_UNSUPPORTED_FEATURE,
    SX128X_STATUS_UNKNOWN_VALUE,
    SX128X_STATUS_ERROR,
} sx128x_status_t;

/**
 * @brief Sleep modes configurations
 */
enum sx128x_sleep_cfg_e
{
    SX128X_SLEEP_CFG_DATA_RETENTION        = ( 1 << 0 ),
    SX128X_SLEEP_CFG_DATA_BUFFER_RETENTION = ( 1 << 1 ),
};

/**
 * @brief Standby mode configurations
 */
typedef enum sx128x_standby_cfg_e
{
    SX128X_STANDBY_CFG_RC   = 0x00,
    SX128X_STANDBY_CFG_XOSC = 0x01,
} sx128x_standby_cfg_t;

/**
 * @brief Power regulations used to power the device
 */
typedef enum sx128x_reg_mod_e
{
    SX128X_REG_MODE_LDO  = 0x00,  // default
    SX128X_REG_MODE_DCDC = 0x01,
} sx128x_reg_mod_t;

/**
 * @brief Tick sizes available for Rx/Tx timeout operations
 */
typedef enum sx128x_tick_size_e
{
    SX128X_TICK_SIZE_0015_US = 0x00,
    SX128X_TICK_SIZE_0062_US = 0x01,
    SX128X_TICK_SIZE_1000_US = 0x02,
    SX128X_TICK_SIZE_4000_US = 0x03,
} sx128x_tick_size_t;

/**
 * @brief IRQ definition
 */
enum sx128x_irq_mask_e
{
    SX128X_IRQ_NONE                      = ( 0 << 0 ),
    SX128X_IRQ_TX_DONE                   = ( 1 << 0 ),
    SX128X_IRQ_RX_DONE                   = ( 1 << 1 ),
    SX128X_IRQ_SYNC_WORD_VALID           = ( 1 << 2 ),
    SX128X_IRQ_SYNC_WORD_ERROR           = ( 1 << 3 ),
    SX128X_IRQ_HEADER_VALID              = ( 1 << 4 ),
    SX128X_IRQ_HEADER_ERROR              = ( 1 << 5 ),
    SX128X_IRQ_CRC_ERROR                 = ( 1 << 6 ),
    SX128X_IRQ_RANGING_SLV_RES_DONE      = ( 1 << 7 ),
    SX128X_IRQ_RANGING_SLV_REQ_DISCARDED = ( 1 << 8 ),
    SX128X_IRQ_RANGING_MST_RES_VALID     = ( 1 << 9 ),
    SX128X_IRQ_RANGING_MST_TIMEOUT       = ( 1 << 10 ),
    SX128X_IRQ_RANGING_SLV_REQ_VALID     = ( 1 << 11 ),
    SX128X_IRQ_CAD_DONE                  = ( 1 << 12 ),
    SX128X_IRQ_CAD_DETECTED              = ( 1 << 13 ),
    SX128X_IRQ_TIMEOUT                   = ( 1 << 14 ),
    SX128X_IRQ_PREAMBLE_DETECTED         = ( 1 << 15 ),
    SX128X_IRQ_ADV_RANGING_VALID         = ( 1 << 15 ),
    SX128X_IRQ_ALL = SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_DONE | SX128X_IRQ_SYNC_WORD_VALID | SX128X_IRQ_SYNC_WORD_ERROR |
                     SX128X_IRQ_HEADER_VALID | SX128X_IRQ_HEADER_ERROR | SX128X_IRQ_CRC_ERROR |
                     SX128X_IRQ_RANGING_SLV_RES_DONE | SX128X_IRQ_RANGING_SLV_REQ_DISCARDED |
                     SX128X_IRQ_RANGING_MST_RES_VALID | SX128X_IRQ_RANGING_MST_TIMEOUT |
                     SX128X_IRQ_RANGING_SLV_REQ_VALID | SX128X_IRQ_CAD_DONE | SX128X_IRQ_CAD_DETECTED |
                     SX128X_IRQ_TIMEOUT | SX128X_IRQ_PREAMBLE_DETECTED | SX128X_IRQ_ADV_RANGING_VALID,
};

/**
 * @brief IRQ bit mask definition.
 *
 * @see sx128x_irq_mask_e
 */
typedef uint16_t sx128x_irq_mask_t;

/**
 * @brief Possible packet types (i.e. modem)
 */
typedef enum sx128x_pkt_type_e
{
    SX128X_PKT_TYPE_GFSK    = 0x00,
    SX128X_PKT_TYPE_LORA    = 0x01,
    SX128X_PKT_TYPE_RANGING = 0x02,
    SX128X_PKT_TYPE_FLRC    = 0x03,
    SX128X_PKT_TYPE_BLE     = 0x04,
} sx128x_pkt_type_t;

/**
 * @brief Ramping times for the power amplifier
 */
typedef enum sx128x_ramp_time_e
{
    SX128X_RAMP_02_US = 0x00,
    SX128X_RAMP_04_US = 0x20,
    SX128X_RAMP_06_US = 0x40,
    SX128X_RAMP_08_US = 0x60,
    SX128X_RAMP_10_US = 0x80,
    SX128X_RAMP_12_US = 0xA0,
    SX128X_RAMP_16_US = 0xC0,
    SX128X_RAMP_20_US = 0xE0,
} sx128x_ramp_time_t;

/**
 * @brief Combinations of bitrate and bandwidth for GFSK and BLE packet types
 *
 * @remark The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef enum sx128x_gfsk_ble_br_bw_e
{
    SX128X_GFSK_BLE_BR_2_000_BW_2_4 = 0x04,
    SX128X_GFSK_BLE_BR_1_600_BW_2_4 = 0x28,
    SX128X_GFSK_BLE_BR_1_000_BW_2_4 = 0x4C,
    SX128X_GFSK_BLE_BR_1_000_BW_1_2 = 0x45,
    SX128X_GFSK_BLE_BR_0_800_BW_2_4 = 0x70,
    SX128X_GFSK_BLE_BR_0_800_BW_1_2 = 0x69,
    SX128X_GFSK_BLE_BR_0_500_BW_1_2 = 0x8D,
    SX128X_GFSK_BLE_BR_0_500_BW_0_6 = 0x86,
    SX128X_GFSK_BLE_BR_0_400_BW_1_2 = 0xB1,
    SX128X_GFSK_BLE_BR_0_400_BW_0_6 = 0xAA,
    SX128X_GFSK_BLE_BR_0_250_BW_0_6 = 0xCE,
    SX128X_GFSK_BLE_BR_0_250_BW_0_3 = 0xC7,
    SX128X_GFSK_BLE_BR_0_125_BW_0_3 = 0xEF,
} sx128x_gfsk_ble_br_bw_t;

/**
 * @brief Modulation indexes for GFSK and BLE packet types
 */
typedef enum sx128x_gfsk_ble_mod_ind_s
{
    SX128X_GFSK_BLE_MOD_IND_0_35 = 0,
    SX128X_GFSK_BLE_MOD_IND_0_50 = 1,
    SX128X_GFSK_BLE_MOD_IND_0_75 = 2,
    SX128X_GFSK_BLE_MOD_IND_1_00 = 3,
    SX128X_GFSK_BLE_MOD_IND_1_25 = 4,
    SX128X_GFSK_BLE_MOD_IND_1_50 = 5,
    SX128X_GFSK_BLE_MOD_IND_1_75 = 6,
    SX128X_GFSK_BLE_MOD_IND_2_00 = 7,
    SX128X_GFSK_BLE_MOD_IND_2_25 = 8,
    SX128X_GFSK_BLE_MOD_IND_2_50 = 9,
    SX128X_GFSK_BLE_MOD_IND_2_75 = 10,
    SX128X_GFSK_BLE_MOD_IND_3_00 = 11,
    SX128X_GFSK_BLE_MOD_IND_3_25 = 12,
    SX128X_GFSK_BLE_MOD_IND_3_50 = 13,
    SX128X_GFSK_BLE_MOD_IND_3_75 = 14,
    SX128X_GFSK_BLE_MOD_IND_4_00 = 15,
} sx128x_gfsk_ble_mod_ind_t;

/**
 * @brief Modulation shapings for GFSK, FLRC and BLE packet types
 */
typedef enum sx128x_gfsk_flrc_ble_pulse_shape_e
{
    SX128X_GFSK_FLRC_BLE_PULSE_SHAPE_OFF   = 0x00,
    SX128X_GFSK_FLRC_BLE_PULSE_SHAPE_BT_1  = 0x10,
    SX128X_GFSK_FLRC_BLE_PULSE_SHAPE_BT_05 = 0x20,
} sx128x_gfsk_flrc_ble_pulse_shape_t;

/**
 * @brief Combinations of bitrate and bandwidth for GFSK packet type
 *
 * @remark The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef sx128x_gfsk_ble_br_bw_t sx128x_gfsk_br_bw_t;

/**
 * @brief Modulation indexes for GFSK packet type
 */
typedef sx128x_gfsk_ble_mod_ind_t sx128x_gfsk_mod_ind_t;

/**
 * @brief Modulation shaping for GFSK packet type
 */
typedef sx128x_gfsk_flrc_ble_pulse_shape_t sx128x_gfsk_pulse_shape_t;

/**
 * @brief Modulation parameters for GFSK packet type
 */
typedef struct sx128x_mod_params_gfsk_s
{
    sx128x_gfsk_br_bw_t       br_bw;        //!< GFSK bitrate bandwidth configuration
    sx128x_gfsk_mod_ind_t     mod_ind;      //!< GFSK modulation index configuration
    sx128x_gfsk_pulse_shape_t pulse_shape;  //!< GFSK pulse shape configuration
} sx128x_mod_params_gfsk_t;

/**
 * @brief Spreading factor values for LoRa and Ranging packet types
 */
typedef enum sx128x_lora_ranging_sf_e
{
    SX128X_LORA_RANGING_SF5  = 0x50,
    SX128X_LORA_RANGING_SF6  = 0x60,
    SX128X_LORA_RANGING_SF7  = 0x70,
    SX128X_LORA_RANGING_SF8  = 0x80,
    SX128X_LORA_RANGING_SF9  = 0x90,
    SX128X_LORA_RANGING_SF10 = 0xA0,
    SX128X_LORA_RANGING_SF11 = 0xB0,
    SX128X_LORA_RANGING_SF12 = 0xC0,
} sx128x_lora_ranging_sf_t;

/**
 * @brief Bandwidth values for LoRa and Ranging packet types
 */
typedef enum sx128x_lora_ranging_bw_e
{
    SX128X_LORA_RANGING_BW_200  = 0x34,
    SX128X_LORA_RANGING_BW_400  = 0x26,
    SX128X_LORA_RANGING_BW_800  = 0x18,
    SX128X_LORA_RANGING_BW_1600 = 0x0A,
} sx128x_lora_ranging_bw_t;

/**
 * @brief Coding rate values for LoRa and Ranging packet types
 */
typedef enum sx128x_lora_ranging_cr_e
{
    SX128X_LORA_RANGING_CR_4_5    = 0x01,
    SX128X_LORA_RANGING_CR_4_6    = 0x02,
    SX128X_LORA_RANGING_CR_4_7    = 0x03,
    SX128X_LORA_RANGING_CR_4_8    = 0x04,
    SX128X_LORA_RANGING_CR_LI_4_5 = 0x05,
    SX128X_LORA_RANGING_CR_LI_4_6 = 0x06,
    SX128X_LORA_RANGING_CR_LI_4_8 = 0x07,
} sx128x_lora_ranging_cr_t;

/**
 * @brief Spreading factors for LoRa packet types
 */
typedef sx128x_lora_ranging_sf_t sx128x_lora_sf_t;

/**
 * @brief Bandwidths for LoRa packet type
 */
typedef sx128x_lora_ranging_bw_t sx128x_lora_bw_t;

/**
 * @brief Coding rates for LoRa packet type
 */
typedef sx128x_lora_ranging_cr_t sx128x_lora_cr_t;

/**
 * @brief Modulation parameters for LoRa and Ranging packet types
 */
typedef struct sx128x_mod_params_lora_ranging_s
{
    sx128x_lora_sf_t sf;  //!< LoRa spreading factor configuration
    sx128x_lora_bw_t bw;  //!< LoRa bandwidth configuration
    sx128x_lora_cr_t cr;  //!< LoRa coding rate configuration
} sx128x_mod_params_lora_ranging_t;

/**
 * @brief Modulation parameters for LoRa packet type
 */
typedef sx128x_mod_params_lora_ranging_t sx128x_mod_params_lora_t;

/**
 * @brief Spreading factors for Ranging packet types
 */
typedef sx128x_lora_ranging_sf_t sx128x_ranging_sf_t;

/**
 * @brief Bandwidths for Ranging packet type
 */
typedef sx128x_lora_ranging_bw_t sx128x_ranging_bw_t;

/**
 * @brief Coding rates for Ranging packet type
 */
typedef sx128x_lora_ranging_cr_t sx128x_ranging_cr_t;

/**
 * @brief Modulation parameters for Ranging packet type
 */
typedef sx128x_mod_params_lora_ranging_t sx128x_mod_params_ranging_t;

/**
 * @brief Number of Ranging address bits checked
 */
typedef enum sx128x_ranging_address_len_e
{
    SX128X_RANGING_ADDRESS_LEN_8  = 0x00,
    SX128X_RANGING_ADDRESS_LEN_16 = 0x01,
    SX128X_RANGING_ADDRESS_LEN_24 = 0x02,
    SX128X_RANGING_ADDRESS_LEN_32 = 0x03,
} sx128x_ranging_address_len_t;

/**
 * @brief Ranging result type
 */
typedef enum sx128x_ranging_result_type_e
{
    SX128X_RANGING_RESULT_TYPE_RAW      = 0x00,
    SX128X_RANGING_RESULT_TYPE_FILTERED = 0x01,
} sx128x_ranging_result_type_t;

/**
 * @brief Combinations of bitrate and bandwidth for FLRC packet type
 *
 * @remark The bitrate is in Mb/s and the bandwidth in MHz
 */
typedef enum sx128x_flrc_br_bw_e
{
    SX128X_FLRC_BR_1_300_BW_1_2 = 0x45,
    SX128X_FLRC_BR_1_040_BW_1_2 = 0x69,
    SX128X_FLRC_BR_0_650_BW_0_6 = 0x86,
    SX128X_FLRC_BR_0_520_BW_0_6 = 0xAA,
    SX128X_FLRC_BR_0_325_BW_0_3 = 0xC7,
    SX128X_FLRC_BR_0_260_BW_0_3 = 0xEB,
} sx128x_flrc_br_bw_t;

/**
 * @brief Coding rates for FLRC packet type
 */
typedef enum sx128x_flrc_cr_e
{
    SX128X_FLRC_CR_1_2 = 0x00,
    SX128X_FLRC_CR_3_4 = 0x02,
    SX128X_FLRC_CR_1_1 = 0x04,
} sx128x_flrc_cr_t;

/**
 * @brief Modulation shapings for FLRC packet types
 */
typedef sx128x_gfsk_flrc_ble_pulse_shape_t sx128x_flrc_pulse_shape_t;

/**
 * @brief Modulation parameters for FLRC packet type
 */
typedef struct sx128x_mod_params_flrc_s
{
    sx128x_flrc_br_bw_t       br_bw;        //!< FLRC bitrate bandwidth configuration
    sx128x_flrc_cr_t          cr;           //!< FLCR coding rate configuration
    sx128x_flrc_pulse_shape_t pulse_shape;  //!< FLRC pulse shape configuration
} sx128x_mod_params_flrc_t;

/**
 * @brief Combinations of bitrate and bandwidth for BLE packet types
 *
 * @remark The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef sx128x_gfsk_ble_br_bw_t sx128x_ble_br_bw_t;

/**
 * @brief Modulation indexes for BLE packet types
 */
typedef sx128x_gfsk_ble_mod_ind_t sx128x_ble_mod_ind_t;

/**
 * @brief Modulation shapings for BLE packet types
 */
typedef sx128x_gfsk_flrc_ble_pulse_shape_t sx128x_ble_pulse_shape_t;

/**
 * @brief Modulation parameters for BLE packet type
 */
typedef struct sx128x_mod_params_ble_s
{
    sx128x_ble_br_bw_t       br_bw;        //!< BLE bitrate and bandwidth configuration
    sx128x_ble_mod_ind_t     mod_ind;      //!< BLE Modulation index configuration
    sx128x_ble_pulse_shape_t pulse_shape;  //!< BLE Pulse shape configuration
} sx128x_mod_params_ble_t;

/**
 * @brief Preamble lengths for GFSK and FLRC packet types
 */
typedef enum sx128x_gfsk_flrc_preamble_len_e
{
    SX128X_GFSK_FLRC_PREAMBLE_LEN_04_BITS = 0x00,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_08_BITS = 0x10,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_12_BITS = 0x20,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_16_BITS = 0x30,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_20_BITS = 0x40,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_24_BITS = 0x50,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_28_BITS = 0x60,
    SX128X_GFSK_FLRC_PREAMBLE_LEN_32_BITS = 0x70,
} sx128x_gfsk_flrc_preamble_len_t;

/**
 * @brief SyncWord lengths for GFSK packet type
 */
typedef enum sx128x_gfsk_sync_word_len_e
{
    SX128X_GFSK_SYNC_WORD_LEN_1_BYTE = 0x00,
    SX128X_GFSK_SYNC_WORD_LEN_2_BYTE = 0x02,
    SX128X_GFSK_SYNC_WORD_LEN_3_BYTE = 0x04,
    SX128X_GFSK_SYNC_WORD_LEN_4_BYTE = 0x06,
    SX128X_GFSK_SYNC_WORD_LEN_5_BYTE = 0x08,
} sx128x_gfsk_sync_word_len_t;

/**
 * @brief Combinations of SyncWord correlators activated for GFSK and FLRC packet
 * types
 */
typedef enum sx128x_gfsk_flrc_rx_match_sync_word_e
{
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_OFF   = 0x00,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_1     = 0x10,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_2     = 0x20,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_1_2   = 0x30,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_3     = 0x40,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_1_3   = 0x50,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_2_3   = 0x60,
    SX128X_GFSK_FLRC_RX_MATCH_SYNCWORD_1_2_3 = 0x70,
} sx128x_gfsk_flrc_rx_match_sync_word_t;

/**
 * @brief Packet length mode for GFSK and FLRC packet types
 */
typedef enum sx128x_gfsk_flrc_pkt_len_modes_e
{
    SX128X_GFSK_FLRC_PKT_FIX_LEN = 0x00,
    SX128X_GFSK_FLRC_PKT_VAR_LEN = 0x20,
} sx128x_gfsk_flrc_pkt_len_modes_t;

/**
 * @brief CRC lengths for GFSK packet types
 *
 * @remark Not all configurations are available for both GFSK and FLRC packet types. Please refer to the datasheet for
 * possible configuration.
 */
typedef enum sx128x_gfsk_crc_types_e
{
    SX128X_GFSK_CRC_OFF     = 0x00,
    SX128X_GFSK_CRC_1_BYTES = 0x10,
    SX128X_GFSK_CRC_2_BYTES = 0x20,
    SX128X_GFSK_CRC_3_BYTES = 0x30,
} sx128x_gfsk_crc_types_t;

/**
 * @brief Whitening modes for GFSK, FLRC and BLE packet types
 */
typedef enum sx128x_gfsk_flrc_ble_dc_free_e
{
    SX128X_GFSK_FLRC_BLE_DC_FREE_OFF = 0x08,
    SX128X_GFSK_FLRC_BLE_DC_FREE_ON  = 0x00,
} sx128x_gfsk_flrc_ble_dc_free_t;

/**
 * @brief Preamble lengths for GFS packet type
 */
typedef sx128x_gfsk_flrc_preamble_len_t sx128x_gfsk_preamble_len_t;

/**
 * @brief Combinations of SyncWord correlators activated for GFSK packet type
 */
typedef sx128x_gfsk_flrc_rx_match_sync_word_t sx128x_gfsk_rx_match_sync_word_t;

/**
 * @brief Packet length mode for GFSK packet type
 */
typedef sx128x_gfsk_flrc_pkt_len_modes_t sx128x_gfsk_pkt_len_modes_t;

/**
 * @brief Packet parameters for GFSK modulation
 */
typedef struct sx128x_pkt_params_gfsk_s
{
    sx128x_gfsk_preamble_len_t       preamble_len;      //!< GFSK preamble length configuration
    sx128x_gfsk_sync_word_len_t      sync_word_len;     //!< GFSK syncword length configuration
    sx128x_gfsk_rx_match_sync_word_t match_sync_word;   //!< GFSK syncword matcher configuration
    sx128x_gfsk_pkt_len_modes_t      header_type;       //!< GFSK header type configuration
    uint8_t                          pld_len_in_bytes;  //!< GFSK payload length in byte configuration
    sx128x_gfsk_crc_types_t          crc_type;          //!< GFSK CRC type configuration
    sx128x_gfsk_flrc_ble_dc_free_t   dc_free;           //!< GFSK whitening configuration
} sx128x_pkt_params_gfsk_t;

/**
 * @brief Packet lengths for LoRa and Ranging packet types
 */
typedef enum sx128x_lora_ranging_pkt_len_modes_e
{
    SX128X_LORA_RANGING_PKT_EXPLICIT = 0x00,
    SX128X_LORA_RANGING_PKT_IMPLICIT = 0x80,
} sx128x_lora_ranging_pkt_len_modes_t;

/**
 * @brief Packet lengths for LoRa packet type
 */
typedef sx128x_lora_ranging_pkt_len_modes_t sx128x_lora_pkt_len_modes_t;

/**
 * @brief Preamble length for LoRa and Ranging packet types
 *
 * It is defined with a mantissa part and and an exponential part. So that the length of the preamble is defined by:
 *
 * \f$ preamble\_length = mant * 2 ^ {exp} \f$
 */
typedef struct sx128x_lora_ranging_preamble_len_s
{
    uint8_t mant;  //!< Mantissa part of the preamble length definition
    uint8_t exp;   //!< Exponenential part of the preamble length definition
} sx128x_lora_ranging_preamble_len_t;

/**
 * @brief Preamble length for LoRa packet type
 */
typedef sx128x_lora_ranging_preamble_len_t sx128x_lora_preamble_len_t;

/**
 * @brief Packet parameters for LoRa and Ranging packet types
 */
typedef struct sx128x_pkt_params_lora_ranging_s
{
    sx128x_lora_preamble_len_t  preamble_len;      //!< LoRa preamble length configuration
    sx128x_lora_pkt_len_modes_t header_type;       //!< LoRa header type configuration
    uint8_t                     pld_len_in_bytes;  //!< LoRa payload length in byte configuration
    bool                        crc_is_on;         //!< LoRa CRC configuration
    bool                        invert_iq_is_on;   //!< LoRa IQ swapping configuration
} sx128x_pkt_params_lora_ranging_t;

/**
 * @brief Packet parameters for LoRa packet type
 */
typedef sx128x_pkt_params_lora_ranging_t sx128x_pkt_params_lora_t;

/**
 * @brief Packet lengths for Ranging packet type
 */
typedef sx128x_lora_ranging_pkt_len_modes_t sx128x_ranging_pkt_len_modes_t;

/**
 * @brief Preamble length for Ranging packet type
 */
typedef sx128x_lora_ranging_preamble_len_t sx128x_ranging_preamble_len_t;

/**
 * @brief Packet parameters for Ranging packet type
 */
typedef sx128x_pkt_params_lora_ranging_t sx128x_pkt_params_ranging_t;

/**
 * @brief Preamble length values for FLRC packet type
 */
typedef sx128x_gfsk_flrc_preamble_len_t sx128x_flrc_preamble_len_t;

/**
 * @brief SyncWord lengths for FLRC packet type
 */
typedef enum sx128x_flrc_sync_word_len_e
{
    SX128X_FLRC_SYNC_WORD_OFF = 0x00,
    SX128X_FLRC_SYNC_WORD_ON  = 0x04,
} sx128x_flrc_sync_word_len_t;

/**
 * @brief Combinations of SyncWord correlators activated for FLRC packet type
 */
typedef sx128x_gfsk_flrc_rx_match_sync_word_t sx128x_flrc_rx_match_sync_word_t;

/**
 * @brief Packet lengths for FLRC packet type
 */
typedef sx128x_gfsk_flrc_pkt_len_modes_t sx128x_flrc_pkt_len_modes_t;

/**
 * @brief CRC lengths for FLRC packet types
 *
 * @remark Not all configurations are available for both GFSK and FLRC packet types. Please refer to the datasheet for
 * possible configuration.
 */
typedef enum sx128x_flrc_crc_types_e
{
    SX128X_FLRC_CRC_OFF     = 0x00,
    SX128X_FLRC_CRC_2_BYTES = 0x10,
    SX128X_FLRC_CRC_3_BYTES = 0x20,
    SX128X_FLRC_CRC_4_BYTES = 0x30,
} sx128x_flrc_crc_types_t;

/**
 * @brief Packet parameters for FLRC packet type
 */
typedef struct sx128x_pkt_params_flrc_s
{
    sx128x_flrc_preamble_len_t       preamble_len;      //!< FLRC preamble length configuration
    sx128x_flrc_sync_word_len_t      sync_word_len;     //!< FLRC syncword length configuration
    sx128x_flrc_rx_match_sync_word_t match_sync_word;   //!< FLRC syncword matcher configuration
    sx128x_flrc_pkt_len_modes_t      header_type;       //!< FLRC header type configuration
    uint8_t                          pld_len_in_bytes;  //!< FLRC payload length in byte configuration
    sx128x_flrc_crc_types_t          crc_type;          //!< FLRC CRC type configuration
} sx128x_pkt_params_flrc_t;

/**
 * @brief Connection states for BLE packet type
 */
typedef enum sx128x_ble_con_states_e
{
    SX128X_BLE_PLD_LEN_MAX_31_BYTES  = 0x00,
    SX128X_BLE_PLD_LEN_MAX_37_BYTES  = 0x20,
    SX128X_BLE_TX_TEST_MODE          = 0x40,
    SX128X_BLE_RX_TEST_MODE          = 0x60,
    SX128X_BLE_PLD_LEN_MAX_255_BYTES = 0x80,
} sx128x_ble_con_states_t;

/**
 * @brief CRC lengths for BLE packet type
 */
typedef enum sx128x_ble_crc_type_e
{
    SX128X_BLE_CRC_OFF = 0x00,
    SX128X_BLE_CRC_3B  = 0x10,
} sx128x_ble_crc_type_t;

/**
 * @brief BLE packet types for BLE packet type
 */
typedef enum sx128x_ble_pkt_types_e
{
    SX128X_BLE_PKT_TYPE_PRBS_9       = 0x00,  //!< Pseudo Random Binary Sequence based on 9th degree polynomial
    SX128X_BLE_PKT_TYPE_PRBS_15      = 0x0C,  //!< Pseudo Random Binary Sequence based on 15th degree polynomial
    SX128X_BLE_PKT_TYPE_EYELONG_1_0  = 0x04,  //!< Repeated '11110000' sequence
    SX128X_BLE_PKT_TYPE_EYELONG_0_1  = 0x18,  //!< Repeated '00001111' sequence
    SX128X_BLE_PKT_TYPE_EYESHORT_1_0 = 0x08,  //!< Repeated '10101010' sequence
    SX128X_BLE_PKT_TYPE_EYESHORT_0_1 = 0x1C,  //!< Repeated '01010101' sequence
    SX128X_BLE_PKT_TYPE_ALL_1        = 0x10,  //!< Repeated '11111111' sequence
    SX128X_BLE_PKT_TYPE_ALL_0        = 0x14,  //!< Repeated '00000000' sequence
} sx128x_ble_pkt_type_t;

/**
 * @brief Whitening modes for BLE packet type
 */
typedef sx128x_gfsk_flrc_ble_dc_free_t sx128x_ble_dc_free_t;

/**
 * @brief Packet parameters for BLE packet type
 */
typedef struct sx128x_pkt_params_ble_s
{
    sx128x_ble_con_states_t con_state;  //!< BLE connection state configuration
    sx128x_ble_crc_type_t   crc_type;   //!< BLE CRC configuration
    sx128x_ble_pkt_type_t   pkt_type;   //!< BLE test packet payload to use when con_state == SX128X_BLE_TX_TEST_MODE
    sx128x_ble_dc_free_t    dc_free;    //!< BLE whitening configuration
} sx128x_pkt_params_ble_t;

/**
 * @brief CAD number of symbols
 */
typedef enum sx128x_lora_cad_symbs_e
{
    SX128X_LORA_CAD_01_SYMB = 0x00,
    SX128X_LORA_CAD_02_SYMB = 0x20,
    SX128X_LORA_CAD_04_SYMB = 0x40,
    SX128X_LORA_CAD_08_SYMB = 0x60,
    SX128X_LORA_CAD_16_SYMB = 0x80,
} sx128x_lora_cad_symbs_t;

/**
 * @brief CAD parameters.
 */
typedef struct sx128x_lora_cad_param_s
{
    sx128x_lora_cad_symbs_t cad_symb_nb;  //!< Configuration of the number of LoRa symbols for CAD operation
} sx128x_lora_cad_params_t;

/**
 * @brief Chip mode status
 */
typedef enum sx128x_chip_modes_e
{
    SX128X_CHIP_MODE_STBY_RC   = 2,
    SX128X_CHIP_MODE_STBY_XOSC = 3,
    SX128X_CHIP_MODE_FS        = 4,
    SX128X_CHIP_MODE_RX        = 5,
    SX128X_CHIP_MODE_TX        = 6,
} sx128x_chip_modes_t;

/**
 * @brief Chip command status
 */
typedef enum sx128x_cmd_status_e
{
    SX128X_CMD_STATUS_CMD_OK            = 1,
    SX128X_CMD_STATUS_DATA_AVAILABLE    = 2,
    SX128X_CMD_STATUS_CMD_TIMEOUT       = 3,
    SX128X_CMD_STATUS_CMD_PROCESS_ERROR = 4,
    SX128X_CMD_STATUS_CMD_EXEC_FAILURE  = 5,
    SX128X_CMD_STATUS_CMD_TX_DONE       = 6,
} sx128x_cmd_status_t;

/**
 * @brief Chip status parameters
 */
typedef struct sx128x_chip_status_s
{
    sx128x_cmd_status_t cmd_status;  //!< Status of the last command
    sx128x_chip_modes_t chip_mode;   //!< Current mode of the chip
} sx128x_chip_status_t;

/**
 * @brief Rx buffer status parameters for GFSK, LoRa, FLRC and BLE packet types
 */
typedef struct sx128x_rx_buffer_status_s
{
    uint8_t pld_len_in_bytes;      //!< Received payload buffer length in bytes
    uint8_t buffer_start_pointer;  //!< Received payload buffer offset
} sx128x_rx_buffer_status_t;

/**
 * @brief Bit mask for packet status error
 */
enum sx128x_pkt_status_errors_e
{
    SX128X_PKT_STATUS_ERROR_PKT_CTRL_BUSY = ( 1 << 0 ),
    SX128X_PKT_STATUS_ERROR_PKT_RX        = ( 1 << 1 ),
    SX128X_PKT_STATUS_ERROR_HEADER_RX     = ( 1 << 2 ),
    SX128X_PKT_STATUS_ERROR_TX_RX_ABORTED = ( 1 << 3 ),
    SX128X_PKT_STATUS_ERROR_PKT_CRC       = ( 1 << 4 ),
    SX128X_PKT_STATUS_ERROR_PKT_LEN       = ( 1 << 5 ),
    SX128X_PKT_STATUS_ERROR_PKT_SYNC      = ( 1 << 6 ),
};

/**
 * @brief Mask for status errors
 *
 * @see sx128x_pkt_status_errors_e
 */
typedef uint8_t sx128x_pkt_status_errors_t;

/**
 * @brief Bit mask for packet status
 */
enum sx128x_pkt_status_e
{
    SX128X_PKT_STATUS_PKT_SENT  = ( 1 << 0 ),
    SX128X_PKT_STATUS_RX_NO_ACK = ( 1 << 5 ),
};

/**
 * @brief Mask for packet statuses
 *
 * @see sx128x_pkt_status_e
 */
typedef uint8_t sx128x_pkt_status_t;

/**
 * @brief Bit mask for packet status sync address
 */
enum sx128x_pkt_status_sync_e
{
    SX128X_PKT_STATUS_SYNC_ADDRESS_1 = ( 1 << 0 ),
    SX128X_PKT_STATUS_SYNC_ADDRESS_2 = ( 1 << 1 ),
    SX128X_PKT_STATUS_SYNC_ADDRESS_3 = ( 1 << 2 ),
};

/**
 * @brief Mask for packet status sync addresses
 *
 * @see sx128x_pkt_status_sync_e
 */
typedef uint8_t sx128x_pkt_status_sync_t;

/**
 * @brief Packet status parameters for GFSK, FLRC and BLE packet types
 */
typedef struct sx128x_pkt_status_gfsk_flrc_ble_s
{
    int8_t                     rssi;    //!< FSK/FLRC/BLE packet RSSI (dBm)
    sx128x_pkt_status_errors_t errors;  //!< FSK/FLRC/BLE packet error bit mask
    sx128x_pkt_status_t        status;  //!< FSK/FLRC/BLE packet status bit mask
    sx128x_pkt_status_sync_t   sync;    //!< FSK/FLRC/BLE packet sync bit mask
} sx128x_pkt_status_gfsk_flrc_ble_t;

/**
 * @brief Packet status parameters for GFSK packet type
 */
typedef sx128x_pkt_status_gfsk_flrc_ble_t sx128x_pkt_status_gfsk_t;

/**
 * @brief Packet status parameters for FLRC packet type
 */
typedef sx128x_pkt_status_gfsk_flrc_ble_t sx128x_pkt_status_flrc_t;

/**
 * @brief Packet status parameters for BLE packet type
 */
typedef sx128x_pkt_status_gfsk_flrc_ble_t sx128x_pkt_status_ble_t;

/**
 * @brief Packet status for LoRa and Ranging packet types
 */
typedef struct sx128x_pkt_status_lora_ranging_s
{
    int8_t rssi;  //!< LoRa packet RSSI (dBm)
    int8_t snr;   //!< LoRa packet SNR (dB)
} sx128x_pkt_status_lora_ranging_t;

/**
 * @brief Packet status for LoRa packet types
 */
typedef sx128x_pkt_status_lora_ranging_t sx128x_pkt_status_lora_t;

/**
 * @brief Packet status for Ranging packet types
 */
typedef sx128x_pkt_status_lora_ranging_t sx128x_pkt_status_ranging_t;

/**
 * @brief Ranging roles for ranging packet type
 */
typedef enum sx128x_ranging_role_e
{
    SX128X_RANGING_ROLE_SLV = 0,
    SX128X_RANGING_ROLE_MST = 1,
} sx128x_ranging_role_t;

/**
 * @brief Selector values to configure LNA regime
 */
typedef enum sx128x_lna_settings_e
{
    //!< Allow maximum efficiency of sx128x (default)
    SX128X_LNA_LOW_POWER_MODE,
    //!< Allow to use highest three steps of LNA gain and increase current
    //!< consumption
    SX128X_LNA_HIGH_SENSITIVITY_MODE,
} sx128x_lna_settings_t;

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
 * @param [in]  retain_data_buffer Retain the data buffer during sleep
 * @param [in]  retain_radio_configuration Retain the radio configuration during sleep
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_sleep( const void* context, const bool retain_data_buffer,
                                  const bool retain_radio_configuration );

/**
 * @brief Set the chip in stand-by mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Stand-by mode configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_standby( const void* context, const sx128x_standby_cfg_t cfg );

/**
 * @brief Wake the radio up from sleep mode.
 *
 * @param [in]  context Chip implementation context
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_wakeup( const void* context );

/**
 * @brief Set the chip in frequency synthesis mode
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_fs( const void* context );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref sx128x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout.
 *
 * @remark The timeout duration can be computed with the formula:
 * \f$ timeout\_duration = period_base_count \times period_base \f$
 *
 * @remark Maximal value is 0xFFFF
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] period_base Base time scale for the timeout duration
 * @param [in] period_base_count The number of period_base to count for timeout
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_tx( const void* context, sx128x_tick_size_t period_base, const uint16_t period_base_count );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref sx128x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout.
 *
 * @remark The timeout duration is obtained by:
 * \f$ timeout\_duration = period_base_count \times period_base \f$
 *
 * @remark Maximal ordinary timeout value is 0xFFFE.
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values | Meaning                                                                                         |
 * | -----------    | ------------------------------------------------------------------------------------------------|
 * | 0x0000         | RX single: the chip stays in RX mode until a packet is received, then switch to standby RC mode |
 * | 0xFFFF         | RX continuous: the chip stays in RX mode even after reception of a packet                       |
 *
 * @param [in] context Chip implementation context
 * @param [in] period_base Base time scale for the timeout duration
 * @param [in] period_base_count The number of period_base to count for timeout
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_rx( const void* context, sx128x_tick_size_t period_base, const uint16_t period_base_count );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @remark The Rx mode duration is defined by:
 * \f$ rx\_duration = rx_period_base_count \times period_base \f$
 *
 * @remark The sleep mode duration is defined by:
 * \f$ sleep\_duration = sleep_period_base_count \times period_base \f$
 *
 * @remark Maximal timeout value is 0xFFFF.
 *
 * @param [in] context Chip implementation context
 * @param [in] period_base Base time scale for the timeout duration
 * @param [in] rx_period_base_count The number of period_base to count for Rx operation timeout
 * @param [in] sleep_period_base_count The number of period_base to count for Sleep operation timeout
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_rx_duty_cycle( const void* context, sx128x_tick_size_t period_base,
                                          const uint16_t rx_period_base_count, const uint16_t sleep_period_base_count );

/**
 * @brief Set the chip in CAD (Channel Activity Detection) mode
 *
 * @remark The LoRa packet type shall be selected with @ref sx128x_set_pkt_type before this function is called.
 *
 * @remark The CAD mode is configured with @ref sx128x_set_cad_params.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_cad( const void* context );

/**
 * @brief Set packet transmission to occur a programmable time after packet reception
 *
 * @param [in] context Chip implementation context
 * @param [in] time_in_us The time delay between reception and transmission, in microseconds
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_auto_tx( const void* context, uint16_t time_in_us );

/**
 * @brief Configure the chip to enter FS mode after transmission or reception
 *
 * @param [in] context Chip implementation context
 * @param [in] is_enabled Set to 1 to enable this feature, or 0 to disable
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_auto_fs( const void* context, bool is_enabled );

/**
 * @brief Set the chip in Tx continuous wave (RF tone).
 *
 * @remark The packet type shall be configured with @ref sx128x_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_tx_cw( const void* context );

/**
 * @brief Set the chip in Tx infinite preamble (modulated signal).
 *
 * @remark The packet type shall be configured with @ref sx128x_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @warning This may be called SetTxContinuousPreamble in the device documentation.
 */
sx128x_status_t sx128x_set_tx_infinite_preamble( const void* context );

//
// Register and Buffer Access
//

/**
 * @brief Write data into register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start writing operation
 * @param [in] buffer The buffer of bytes to write into memory
 * @param [in] size Number of bytes to write into memory, starting from addr
 *
 * @see sx128x_read_register
 */
sx128x_status_t sx128x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint16_t size );

/**
 * @brief Read data from register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start reading operation
 * @param [in] buffer The buffer of bytes to be filled with data from registers
 * @param [in] size Number of bytes to read from memory, starting from addr
 *
 * @see sx128x_write_register
 */
sx128x_status_t sx128x_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint16_t size );

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
 * @see sx128x_read_buffer
 */
sx128x_status_t sx128x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint16_t size );

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
 * @see sx128x_write_buffer
 */
sx128x_status_t sx128x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint16_t size );

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
 * sx128x_clear_irq_status.
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt mask
 * @param [in] dio1_mask Variable that holds the interrupt mask for dio1
 * @param [in] dio2_mask Variable that holds the interrupt mask for dio2
 * @param [in] dio3_mask Variable that holds the interrupt mask for dio3
 *
 * @returns Operation status
 *
 * @see sx128x_clear_irq_status, sx128x_get_irq_status
 */
sx128x_status_t sx128x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask );

/**
 * @brief Get system interrupt status
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status
 *
 * @returns Operation status
 *
 * @see sx128x_clear_irq_status
 */
sx128x_status_t sx128x_get_irq_status( const void* context, sx128x_irq_mask_t* irq );

/**
 * @brief Clear selected system interrupts
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt to be cleared
 *
 * @returns Operation status
 *
 * @see sx128x_get_irq_status
 */
sx128x_status_t sx128x_clear_irq_status( const void* context, const sx128x_irq_mask_t irq_mask );

/**
 * @brief Clears any radio irq status flags that are set and returns the flags that were cleared.
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status. Can be NULL.
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_and_clear_irq_status( const void* context, sx128x_irq_mask_t* irq );

//
// RF Modulation and Packet-Related Functions
//

/**
 * @brief Set the RF frequency for future radio operations.
 *
 * @remark This command shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq_in_hz The frequency in Hz to set for radio operations
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_rf_freq( const void* context, const uint32_t freq_in_hz );

/**
 * @brief Set the RF frequency for future radio operations - parameter in PLL steps
 *
 * @remark This command shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq The frequency in PLL steps to set for radio operations
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq );

/**
 * @brief Set the packet type
 *
 * @param [in] context Chip implementation context
 * @param [in] pkt_type Packet type to set
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_pkt_type( const void* context, const sx128x_pkt_type_t pkt_type );

/**
 * @brief Get the current packet type
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_type Pointer to a variable holding the packet type
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_pkt_type( const void* context, sx128x_pkt_type_t* pkt_type );

/**
 * @brief Set the parameters for TX power and power amplifier ramp time
 *
 * @param [in] context Chip implementation context
 * @param [in] pwr_in_dbm The Tx output power in dBm
 * @param [in] ramp_time The ramping time configuration for the PA
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                      const sx128x_ramp_time_t ramp_time );

/**
 * @brief Set the modulation parameters for GFSK packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The GFSK modulation configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_mod_params( const void* context, const sx128x_mod_params_gfsk_t* params );

/**
 * @brief Set the modulation parameters for LoRa packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The LoRa modulation configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_lora_mod_params( const void* context, const sx128x_mod_params_lora_t* params );

/**
 * @brief Set the modulation parameters for ranging packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The ranging modulation configuration
 *
 * @returns Operation status
 */
static inline sx128x_status_t sx128x_set_ranging_mod_params( const void*                        context,
                                                             const sx128x_mod_params_ranging_t* params )
{
    return sx128x_set_lora_mod_params( context, params );
}

/**
 * @brief Set the modulation parameters for FLRC packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The FLRC modulation configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_flrc_mod_params( const void* context, const sx128x_mod_params_flrc_t* params );

/**
 * @brief Set the modulation parameters for BLE packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The BLE modulation configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ble_mod_params( const void* context, const sx128x_mod_params_ble_t* params );

/**
 * @brief Set the packet parameters for GFSK packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The GFSK packet configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_pkt_params( const void* context, const sx128x_pkt_params_gfsk_t* params );

/**
 * @brief Set the packet parameters for LoRa packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The LoRa packet configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_lora_pkt_params( const void* context, const sx128x_pkt_params_lora_t* params );

/**
 * @brief Set the packet parameters for ranging packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The ranging packet configuration
 *
 * @returns Operation status
 */
static inline sx128x_status_t sx128x_set_ranging_pkt_params( const void*                        context,
                                                             const sx128x_pkt_params_ranging_t* params )
{
    return sx128x_set_lora_pkt_params( context, params );
}

/**
 * @brief Set the packet parameters for FLRC packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The FLRC packet configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_flrc_pkt_params( const void* context, const sx128x_pkt_params_flrc_t* params );

/**
 * @brief Set the packet parameters for BLE packets
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The BLE packet configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ble_pkt_params( const void* context, const sx128x_pkt_params_ble_t* params );

/**
 * @brief Set the parameters for CAD operation
 *
 * @remark The command @ref sx128x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The CAD configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_cad_params( const void* context, const sx128x_lora_cad_params_t* params );

/**
 * @brief Set buffer start addresses for both Tx and Rx operations
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_base_address The start address used for Tx operations
 * @param [in] rx_base_address The start address used for Rx operations
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address );

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
sx128x_status_t sx128x_get_status( const void* context, sx128x_chip_status_t* radio_status );

/**
 * @brief Get the current Rx buffer status
 *
 * @details This function is used to get the length of the received payload and the start address to be used when
 * reading data from the Rx buffer.
 * If the radio is configured in LoRa packet type with implicit header, then the payload length returned by this call
 * is not the one configured.
 * In that situation, use sx128x_get_lora_implicit_payload_len
 *
 * @note This command returns meaningful value ONLY if it is called after the SX128x raises SX128X_IRQ_RX_DONE.
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_buffer_status Pointer to a structure to store the current status
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_rx_buffer_status( const void* context, sx128x_rx_buffer_status_t* rx_buffer_status );

/**
 * @brief May be used when using implicit header mode to retrieve the previously-configured payload length.
 *
 * @remark This may be used if the previously-configured payload length wasn't stored by the application.
 *
 * @param [in] context Chip implementation context
 * @param [out] payload_len Pointer to the variable for storing the previously-configured payload length
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_lora_implicit_payload_len( const void* context, uint8_t* payload_len );

/**
 * @brief Get the status of the last GFSK packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_gfsk_pkt_status( const void* context, sx128x_pkt_status_gfsk_t* pkt_status );

/**
 * @brief Get the status of the last LoRa packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_lora_pkt_status( const void* context, sx128x_pkt_status_lora_t* pkt_status );

/**
 * @brief Get the status of the last ranging packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
static inline sx128x_status_t sx128x_get_ranging_pkt_status( const void*                  context,
                                                             sx128x_pkt_status_ranging_t* pkt_status )
{
    return sx128x_get_lora_pkt_status( context, pkt_status );
}

/**
 * @brief Get the status of the last FLRC packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_flrc_pkt_status( const void* context, sx128x_pkt_status_flrc_t* pkt_status );

/**
 * @brief Get the status of the last BLE packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_ble_pkt_status( const void* context, sx128x_pkt_status_ble_t* pkt_status );

/**
 * @brief Get the instantaneous RSSI value.
 *
 * @remark This function shall be called when in Rx mode.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi Pointer to a variable to store the RSSI value in dBm
 *
 * @returns Operation status
 *
 * @see sx128x_set_rx
 */
sx128x_status_t sx128x_get_rssi_inst( const void* context, int16_t* rssi );

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
sx128x_status_t sx128x_reset( const void* context );

/**
 * @brief Put the transceiver in long preamble mode
 *
 * @param [in] context Chip implementation context
 * @param [in] state Set to 1 to enable this feature, or 0 to disable
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_long_preamble( const void* context, const bool state );

/**
 * @brief Configure the regulator mode to be used
 *
 * @remark This function shall be called to set the regulator mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] mode Regulator mode configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_reg_mode( const void* context, const sx128x_reg_mod_t mode );

/**
 * @brief Configure the LNA
 *
 * @param [in] context Chip implementation context
 * @param [in] settings LNA configuration
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_lna_settings( const void* context, sx128x_lna_settings_t settings );

/**
 * @brief Save the transceiver registers, to permit restoration after wakeup
 *
 * @remark Must be called before calling sx128x_set_sleep, with SX128X_SLEEP_CFG_DATA_RETENTION.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @see sx128x_set_sleep
 */
sx128x_status_t sx128x_save_context( const void* context );

/**
 * @brief Select either master or slave ranging role
 *
 * @param [in] context Chip implementation context
 * @param [in] role The desired ranging role
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ranging_role( const void* context, const sx128x_ranging_role_t role );

/**
 * @brief Activate or deactivate the advanced ranging mode
 *
 * @param [in] context Chip implementation context
 * @param [in] state Set to 1 to enable this feature, or 0 to disable
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_adv_ranging( const void* context, const bool state );

/**
 * @brief Set the number of bit-errors tolerated in the GFSK or FLRC sync word
 *
 * @param [in] context Chip implementation context
 * @param [in] tolerance The number of tolerated bit-errors
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_flrc_sync_word_tolerance( const void* context, uint8_t tolerance );

/**
 * @brief Get the sx128x_gfsk_ble_br_bw_t value with bitrate and bandwidth immediately above the minimum requested one.
 *
 * @param [in] br Largest desired bitrate
 * @param [in] bw Largest desired bandwidth
 * @param [out] param Value for sx128x_gfsk_ble_br_bw_t
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_gfsk_br_bw_param( const uint32_t br, const uint32_t bw, sx128x_gfsk_ble_br_bw_t* param );

/**
 * @brief Retrieve the largest modulation index param such that \f$ param <= 2 \times fdev / br \f$
 *
 * @param [in] br bitrate
 * @param [in] fdev Frequency deviation
 * @param [out] param Modulation index
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_gfsk_mod_ind_param( const uint32_t br, const uint32_t fdev,
                                               sx128x_gfsk_ble_mod_ind_t* param );

/**
 * @brief Get the sx128x_flrc_br_bw_t value with bitrate and bandwidth immediately above the minimum requested one.
 *
 * @param [in] br Largest desired bitrate
 * @param [in] bw Largest desired bandwidth
 * @param [out] param Value for sx128x_flrc_br_bw_t
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_flrc_br_bw_param( const uint32_t br, const uint32_t bw, sx128x_flrc_br_bw_t* param );

/**
 * @brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * @param [in] bw LoRa bandwidth parameter
 *
 * @returns Actual LoRa bandwidth in Hertz
 */
uint32_t sx128x_get_lora_bw_in_hz( sx128x_lora_bw_t bw );

/**
 * @brief Compute the numerator for LoRa time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * @param [in] pkt_params Pointer to the structure holding the LoRa packet parameters
 * @param [in] mod_params Pointer to the structure holding the LoRa modulation parameters
 *
 * @returns LoRa time-on-air numerator
 */
uint32_t sx128x_get_lora_time_on_air_numerator( const sx128x_pkt_params_lora_t* pkt_params,
                                                const sx128x_mod_params_lora_t* mod_params );

/**
 * @brief Get the time on air in ms for LoRa transmission
 *
 * @param [in] pkt_params Pointer to a structure holding the LoRa packet parameters
 * @param [in] mod_params Pointer to a structure holding the LoRa modulation parameters
 *
 * @returns Time-on-air value in ms for LoRa transmission
 */
uint32_t sx128x_get_lora_time_on_air_in_ms( const sx128x_pkt_params_lora_t* pkt_params,
                                            const sx128x_mod_params_lora_t* mod_params );

/**
 * @brief Get the bitrate in bit per second for a given sx128x_gfsk_br_bw_t parameter
 *
 * @param [in] br_bw GFSK bitrate/bandwidth parameter
 *
 * @returns Actual GFSK bitrate in bit per second
 */
uint32_t sx128x_get_gfsk_br_in_bps( sx128x_gfsk_br_bw_t br_bw );

/**
 * @brief Compute the numerator for GFSK time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * @param [in] pkt_params Pointer to the structure holding the GFSK packet parameters
 *
 * @returns GFSK time-on-air numerator
 */
uint32_t sx128x_get_gfsk_time_on_air_numerator( const sx128x_pkt_params_gfsk_t* pkt_params );

/**
 * @brief Get the time on air in ms for GFSK transmission
 *
 * @param [in] pkt_params Pointer to a structure holding the GFSK packet parameters
 * @param [in] mod_params Pointer to a structure holding the GFSK modulation parameters
 *
 * @returns Time-on-air value in ms for GFSK transmission
 */
uint32_t sx128x_get_gfsk_time_on_air_in_ms( const sx128x_pkt_params_gfsk_t* pkt_params,
                                            const sx128x_mod_params_gfsk_t* mod_params );

/**
 * @brief Get the bitrate in bits per second for a given sx128x_flrc_br_bw_t parameter
 *
 * @param [in] br_bw FLRC bitrate/bandwidth parameter
 *
 * @returns Actual FLRC bitrate in bits per second
 */
uint32_t sx128x_get_flrc_br_in_bps( sx128x_flrc_br_bw_t br_bw );

/**
 * @brief Compute the numerator for FLRC time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the FLRC bitrate in bits per
 * second.
 *
 * @param [in] pkt_params Pointer to the structure holding the FLRC packet parameters
 * @param [in] mod_params Pointer to a structure holding the FLRC modulation parameters
 *
 * @returns FLRC time-on-air numerator or 0 if it fails to compute it
 */
uint32_t sx128x_get_flrc_time_on_air_numerator( const sx128x_pkt_params_flrc_t* pkt_params,
                                                const sx128x_mod_params_flrc_t* mod_params );

/**
 * @brief Get the time on air in ms for FLRC transmission
 *
 * @param [in] pkt_params Pointer to a structure holding the FLRC packet parameters
 * @param [in] mod_params Pointer to a structure holding the FLRC modulation parameters
 *
 * @returns Time-on-air value in ms for FLRC transmission
 */
uint32_t sx128x_get_flrc_time_on_air_in_ms( const sx128x_pkt_params_flrc_t* pkt_params,
                                            const sx128x_mod_params_flrc_t* mod_params );

/**
 * @brief Get the number of PLL steps for a given frequency in Hertz
 *
 * @param [in] freq_in_hz Frequency in Hertz
 *
 * @returns Number of PLL steps
 */
uint32_t sx128x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz );

//
// Register access
//

/**
 * @brief Configure one of the GFSK sync words
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word_id Syncword ID to configure. Possible value between 1 and 3
 * @param [in] sync_word Buffer holding the sync word to be configured. Must be of length at least sync_word_len.
 * @param [in] sync_word_len Sync word length in bytes. Must be between 1 and 5.
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_sync_word( const void* context, const uint8_t sync_word_id, const uint8_t* sync_word,
                                           const uint8_t sync_word_len );

/**
 * @brief Configure the LoRa sync word
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Sync word to be configured
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_lora_sync_word( const void* context, const uint8_t sync_word );

/**
 * @brief Configure the sync word used in FLRC packet
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word_id Syncword ID to configure. Possible value between 1 and 3
 * @param [in] sync_word Buffer holding the sync word to be configured. It is up to the integrator to ensure it is at
 * least 4 bytes long. Failing to do so may result in program crash.
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_flrc_sync_word( const void* context, const uint8_t sync_word_id,
                                           const uint8_t sync_word[4] );

/**
 * @brief Configure the BLE sync word
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Buffer holding the 4-byte sync word to be configured. It is up to the integrator to ensure it
 * is at least 4 bytes long. Failing to do so may result in program crash.
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ble_sync_word( const void* context, const uint8_t sync_word[4] );

/**
 * @brief Configure the seed used to compute the GFSK packet CRC
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used to compute the CRC value
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_crc_seed( const void* context, uint16_t seed );

/**
 * @brief Configure the seed used to compute the FLRC packet CRC
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used to compute the CRC value
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_flrc_crc_seed( const void* context, uint32_t seed );

/**
 * @brief Configure the seed used to compute the BLE packet CRC
 *
 * @param [in] context Chip implementation context
 * @param [in] seed seed value used to compute the 3-byte CRC value
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ble_crc_seed( const void* context, uint32_t seed );

/**
 * @brief Configure the polynomial used to compute the GFSK packet CRC
 *
 * @param [in] context Chip implementation context
 * @param [in] polynomial Polynomial value used to compute the CRC value
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_crc_polynomial( const void* context, uint16_t polynomial );

/**
 * @brief Configure the GFSK or BLE whitening seed
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used in data whitening
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_gfsk_ble_whitening_seed( const void* context, uint8_t seed );

/**
 * @brief Get the current packet length mode (header type)
 *
 * @param [in] context Chip implementation context
 * @param [out] header_type Pointer to a structure holding the header type
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_lora_pkt_len_mode( const void* context, sx128x_lora_pkt_len_modes_t* header_type );

/**
 * @brief Get the LoRa packet length, as configured in the radio
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_len Pointer to a variable for storing the length
 *
 * @returns Operation status
 *
 * @remark For LoRa packets with no header, the length returned by sx128x_get_rx_buffer_status will be zero, so this can
 * be used instead
 */
sx128x_status_t sx128x_get_lora_pkt_len( const void* context, uint8_t* pkt_len );

/**
 * @brief Determine the coding rate of the last received packet
 *
 * @param [in] context Chip implementation context
 * @param [out] lora_incoming_cr Pointer to a variable for storing the coding rate
 *
 * @returns Operation status
 *
 * @remark If called after receiving a LoRa packet containing a valid header, the coding rate will be obtained from the
 * header. If called after receiving a LoRa packet in implicit mode, the coding rate will be obtained from the coding
 * rate configuration previously programmed through @ref sx128x_set_lora_pkt_params.
 */
sx128x_status_t sx128x_get_lora_rx_pkt_cr( const void* context, sx128x_lora_ranging_cr_t* lora_incoming_cr );

/**
 * @brief Determine if the last received packet was sent with a CRC
 *
 * @param [in] context Chip implementation context
 * @param [out] is_lora_incoming_crc_present Pointer to a boolean for storing the CRC presence information
 *
 * @returns Operation status
 *
 * @remark If called after receiving a LoRa packet containing a valid header, the CRC presence will be obtained from the
 * header. If called after receiving a LoRa packet in implicit mode, the CRC presence will be obtained from the CRC
 * configuration previously programmed through @ref sx128x_set_lora_pkt_params.
 */
sx128x_status_t sx128x_get_lora_rx_pkt_crc_present( const void* context, bool* is_lora_incoming_crc_present );

/**
 * @brief Set the ranging address for ranging master operations
 *
 * @param [in] context Chip implementation context
 * @param [in] address The desired ranging address
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ranging_master_address( const void* context, uint32_t address );

/**
 * @brief Set the ranging address for ranging slave operations
 *
 * @param [in] context Chip implementation context
 * @param [in] address The desired ranging address
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ranging_slave_address( const void* context, uint32_t address );

/**
 * @brief Set the number of bits of the ranging address that will be checked on ranging packet reception
 *
 * @param [in] context Chip implementation context
 * @param [in] len Number of Ranging address bits checked
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ranging_address_len( const void* context, sx128x_ranging_address_len_t len );

/**
 * @brief Set the ranging calibration value that will be used when ranging
 *
 * @param [in] context Chip implementation context
 * @param [in] calibration The ranging calibration value
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ranging_calibration_value( const void* context, uint16_t calibration );

/**
 * @brief Preserve the ranging result for reading
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_ranging_results_freeze( const void* context );

/**
 * @brief Select whether raw or filtered ranging results are desired
 *
 * @param [in] context Chip implementation context
 * @param [in] type The desired ranging result type
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_set_ranging_result_type( const void* context, sx128x_ranging_result_type_t type );

/**
 * @brief Read the contents of the ranging result registers
 *
 * @param [in] context Chip implementation context
 * @param [in] type Type of ranging result to read
 * @param [out] result Pointer to a variable for storing the measurement
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_ranging_result( const void* context, sx128x_ranging_result_type_t type, int32_t* result );

/**
 * @brief Read the contents of the ranging result, converted to centimeters
 *
 * @param [in] context Chip implementation context
 * @param [in] type Type of ranging result to read
 * @param [in] bw LoRa BW used during the ranging operation
 * @param [out] result Pointer to a variable for storing the measurement
 *
 * @returns Operation status
 */
sx128x_status_t sx128x_get_ranging_result_in_cm( const void* context, sx128x_ranging_result_type_t type,
                                                 sx128x_lora_bw_t bw, int32_t* result );

#ifdef __cplusplus
}
#endif

#endif  // SX128X_H

/* --- EOF ------------------------------------------------------------------ */
