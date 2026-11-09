/**
 * @file      ral_defs.h
 *
 * @brief     Radio abstraction layer types definition
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

#ifndef RAL_DEFS_H
#define RAL_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "lr_fhss_v1_base_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Number of state bytes necessary to guarantee functionality for all radios
 */
#define RAL_LR_FHSS_STATE_MAXSIZE ( 24 )

/**
 * @brief Length, in bytes, of a LR-FHSS sync word
 */
#define LR_FHSS_SYNC_WORD_BYTES ( 4 )

/**
 * @brief Reserved value used to configure a reception in continuous mode
 */
#define RAL_RX_TIMEOUT_CONTINUOUS_MODE 0xFFFFFFFF

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

#define RAL_STANDBY_CFG( X )   \
    X( RAL_STANDBY_CFG_RC, 0 ) \
    X( RAL_STANDBY_CFG_XOSC, 1 )

typedef enum ral_standby_cfg_e
{
#define X( identifier, value ) identifier = value,
    RAL_STANDBY_CFG( X )
#undef X
} ral_standby_cfg_t;

static inline const char* ral_standby_cfg_to_str( const ral_standby_cfg_t cfg )
{
    switch( cfg )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_STANDBY_CFG( X )
#undef X
    }
    return "UNKNOWN_RAL_STANDBY_CFG";
}

#define RAL_FALLBACK_MODES( X )     \
    X( RAL_FALLBACK_STDBY_RC, 0 )   \
    X( RAL_FALLBACK_STDBY_XOSC, 1 ) \
    X( RAL_FALLBACK_FS, 2 )

typedef enum ral_fallback_modes_e
{
#define X( identifier, value ) identifier = value,
    RAL_FALLBACK_MODES( X )
#undef X
} ral_fallback_modes_t;

static inline const char* ral_fallback_modes_to_str( const ral_fallback_modes_t mode )
{
    switch( mode )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FALLBACK_MODES( X )
#undef X
    }
    return "UNKNOWN_RAL_FALLBACK_MODE";
}

#define RAL_XOSC_CFG( X )                \
    X( RAL_XOSC_CFG_XTAL, 0 )            \
    X( RAL_XOSC_CFG_TCXO_RADIO_CTRL, 1 ) \
    X( RAL_XOSC_CFG_TCXO_EXT_CTRL, 2 )

typedef enum ral_xosc_cfg_e
{
#define X( identifier, value ) identifier = value,
    RAL_XOSC_CFG( X )
#undef X
} ral_xosc_cfg_t;

static inline const char* ral_xosc_cfg_to_str( const ral_xosc_cfg_t cfg )
{
    switch( cfg )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_XOSC_CFG( X )
#undef X
    }
    return "UNKNOWN_RAL_XOSC_CFG";
}

/**
 * API return status
 */
#define RAL_STATUS( X )                    \
    X( RAL_STATUS_OK, 0 )                  \
    X( RAL_STATUS_UNSUPPORTED_FEATURE, 1 ) \
    X( RAL_STATUS_UNKNOWN_VALUE, 2 )       \
    X( RAL_STATUS_ERROR, 3 )

typedef enum ral_status_e
{
#define X( identifier, value ) identifier = value,
    RAL_STATUS( X )
#undef X
} ral_status_t;

static inline const char* ral_status_to_str( const ral_status_t status )
{
    switch( status )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_STATUS( X )
#undef X
    }
    return "UNKNOWN_RAL_STATUS";
}

#define RAL_GFSK_CRC_TYPE( X )       \
    X( RAL_GFSK_CRC_OFF, 1 )         \
    X( RAL_GFSK_CRC_1_BYTE, 2 )      \
    X( RAL_GFSK_CRC_2_BYTES, 3 )     \
    X( RAL_GFSK_CRC_1_BYTE_INV, 4 )  \
    X( RAL_GFSK_CRC_2_BYTES_INV, 5 ) \
    X( RAL_GFSK_CRC_3_BYTES, 6 )     \
    X( RAL_GFSK_CRC_3_BYTES_INV, 7 ) \
    X( RAL_GFSK_CRC_4_BYTES, 8 )     \
    X( RAL_GFSK_CRC_4_BYTES_INV, 9 )

typedef enum ral_gfsk_crc_type_e
{
#define X( identifier, value ) identifier = value,
    RAL_GFSK_CRC_TYPE( X )
#undef X
} ral_gfsk_crc_type_t;

static inline const char* ral_gfsk_crc_type_to_str( const ral_gfsk_crc_type_t type )
{
    switch( type )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_GFSK_CRC_TYPE( X )
#undef X
    }
    return "UNKNOWN_RAL_GFSK_CRC_TYPE";
}

#define RAL_GFSK_PULSE_SHAPE( X )      \
    X( RAL_GFSK_PULSE_SHAPE_OFF, 0 )   \
    X( RAL_GFSK_PULSE_SHAPE_BT_03, 1 ) \
    X( RAL_GFSK_PULSE_SHAPE_BT_05, 2 ) \
    X( RAL_GFSK_PULSE_SHAPE_BT_07, 3 ) \
    X( RAL_GFSK_PULSE_SHAPE_BT_1, 4 )

typedef enum ral_gfsk_pulse_shape_e
{
#define X( identifier, value ) identifier = value,
    RAL_GFSK_PULSE_SHAPE( X )
#undef X
} ral_gfsk_pulse_shape_t;

static inline const char* ral_gfsk_pulse_shape_to_str( const ral_gfsk_pulse_shape_t shape )
{
    switch( shape )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_GFSK_PULSE_SHAPE( X )
#undef X
    }
    return "UNKNOWN_RAL_GFSK_PULSE_SHAPE";
}

#define RAL_LORA_SF( X )   \
    X( RAL_LORA_SF5, 5 )   \
    X( RAL_LORA_SF6, 6 )   \
    X( RAL_LORA_SF7, 7 )   \
    X( RAL_LORA_SF8, 8 )   \
    X( RAL_LORA_SF9, 9 )   \
    X( RAL_LORA_SF10, 10 ) \
    X( RAL_LORA_SF11, 11 ) \
    X( RAL_LORA_SF12, 12 )

typedef enum ral_lora_sf_e
{
#define X( identifier, value ) identifier = value,
    RAL_LORA_SF( X )
#undef X
} ral_lora_sf_t;

static inline const char* ral_lora_sf_to_str( const ral_lora_sf_t sf )
{
    switch( sf )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_LORA_SF( X )
#undef X
    }
    return "UNKNOWN_RAL_LORA_SF";
}

#define RAL_LORA_BW( X )                                           \
    X( RAL_LORA_BW_007_KHZ, 0 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_010_KHZ, 1 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_015_KHZ, 2 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_020_KHZ, 3 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_031_KHZ, 4 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_041_KHZ, 5 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_062_KHZ, 6 /* All except SX128X and SX1272 */ ) \
    X( RAL_LORA_BW_125_KHZ, 7 /* All except SX128X */ )            \
    X( RAL_LORA_BW_200_KHZ, 8 /* LR112X and SX128X only */ )       \
    X( RAL_LORA_BW_250_KHZ, 9 /* All except SX128X */ )            \
    X( RAL_LORA_BW_400_KHZ, 10 /* LR112X and SX128X only */ )      \
    X( RAL_LORA_BW_500_KHZ, 11 /* All except SX128X */ )           \
    X( RAL_LORA_BW_800_KHZ, 12 /* LR112X and SX128X only */ )      \
    X( RAL_LORA_BW_1000_KHZ, 13 /* LR20XX only */ )                \
    X( RAL_LORA_BW_1600_KHZ, 14 /* SX128X only */ )                \
    X( RAL_LORA_BW_083_KHZ, 15 /* LR20XX only */ )                 \
    X( RAL_LORA_BW_101_KHZ, 16 /* LR20XX only */ )

typedef enum ral_lora_bw_e
{
#define X( identifier, value ) identifier = value,
    RAL_LORA_BW( X )
#undef X
} ral_lora_bw_t;

static inline const char* ral_lora_bw_to_str( const ral_lora_bw_t bw )
{
    switch( bw )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_LORA_BW( X )
#undef X
    }
    return "UNKNOWN_RAL_LORA_BW";
}

#define RAL_LORA_CR( X )                     \
    X( RAL_LORA_CR_4_5, 1 )                  \
    X( RAL_LORA_CR_4_6, 2 )                  \
    X( RAL_LORA_CR_4_7, 3 )                  \
    X( RAL_LORA_CR_4_8, 4 )                  \
    X( RAL_LORA_CR_LI_4_5, 5 )               \
    X( RAL_LORA_CR_LI_4_6, 6 )               \
    X( RAL_LORA_CR_LI_4_8, 7 )               \
    X( RAL_LORA_CR_LI_CONVOLUTIONAL_4_6, 8 ) \
    X( RAL_LORA_CR_LI_CONVOLUTIONAL_4_8, 9 )
typedef enum ral_lora_cr_e
{
#define X( identifier, value ) identifier = value,
    RAL_LORA_CR( X )
#undef X
} ral_lora_cr_t;

static inline const char* ral_lora_cr_to_str( const ral_lora_cr_t cr )
{
    switch( cr )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_LORA_CR( X )
#undef X
    }
    return "UNKNOWN_RAL_LORA_CR";
}

#define RAL_FLRC_CR( X )    \
    X( RAL_FLRC_CR_1_2, 0 ) \
    X( RAL_FLRC_CR_3_4, 1 ) \
    X( RAL_FLRC_CR_1_1, 2 ) \
    X( RAL_FLRC_CR_2_3, 3 )

typedef enum ral_flrc_cr_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_CR( X )
#undef X
} ral_flrc_cr_t;

static inline const char* ral_flrc_cr_to_str( const ral_flrc_cr_t cr )
{
    switch( cr )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_CR( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_CR";
}

#define RAL_FLRC_CRC_TYPE( X )   \
    X( RAL_FLRC_CRC_OFF, 0 )     \
    X( RAL_FLRC_CRC_2_BYTES, 2 ) \
    X( RAL_FLRC_CRC_3_BYTES, 3 ) \
    X( RAL_FLRC_CRC_4_BYTES, 4 )

typedef enum ral_flrc_crc_type_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_CRC_TYPE( X )
#undef X
} ral_flrc_crc_type_t;

static inline const char* ral_flrc_crc_type_to_str( const ral_flrc_crc_type_t type )
{
    switch( type )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_CRC_TYPE( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_CRC_TYPE";
}

#define RAL_FLRC_PULSE_SHAPE( X )      \
    X( RAL_FLRC_PULSE_SHAPE_OFF, 0 )   \
    X( RAL_FLRC_PULSE_SHAPE_BT_05, 1 ) \
    X( RAL_FLRC_PULSE_SHAPE_BT_1, 2 )

typedef enum ral_flrc_pulse_shape_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_PULSE_SHAPE( X )
#undef X
} ral_flrc_pulse_shape_t;

static inline const char* ral_flrc_pulse_shape_to_str( const ral_flrc_pulse_shape_t shape )
{
    switch( shape )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_PULSE_SHAPE( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_PULSE_SHAPE";
}

#define RAL_FLRC_SYNCWORD_LENGTH( X )        \
    X( RAL_FLRC_SYNCWORD_LENGTH_OFF, 0 )     \
    X( RAL_FLRC_SYNCWORD_LENGTH_2_BYTES, 1 ) \
    X( RAL_FLRC_SYNCWORD_LENGTH_4_BYTES, 2 )

typedef enum ral_flrc_sync_word_len_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_SYNCWORD_LENGTH( X )
#undef X
} ral_flrc_sync_word_len_t;

static inline const char* ral_flrc_sync_word_len_to_str( const ral_flrc_sync_word_len_t len )
{
    switch( len )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_SYNCWORD_LENGTH( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_SYNCWORD_LENGTH";
}

#define RAL_FLRC_TX_SYNCWORD( X )     \
    X( RAL_FLRC_TX_SYNCWORD_NONE, 0 ) \
    X( RAL_FLRC_TX_SYNCWORD_1, 1 )    \
    X( RAL_FLRC_TX_SYNCWORD_2, 2 )    \
    X( RAL_FLRC_TX_SYNCWORD_3, 3 )

typedef enum ral_flrc_tx_syncword_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_TX_SYNCWORD( X )
#undef X
} ral_flrc_tx_syncword_t;

static inline const char* ral_flrc_tx_syncword_to_str( const ral_flrc_tx_syncword_t tx_syncword )
{
    switch( tx_syncword )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_TX_SYNCWORD( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_TX_SYNCWORD";
}

#define RAL_FLRC_RX_MATCH_SYNCWORD( X )       \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_OFF, 0 )    \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_1, 1 )      \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_2, 2 )      \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_1_OR_2, 3 ) \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_3, 4 )      \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_1_OR_3, 5 ) \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_2_OR_3, 6 ) \
    X( RAL_FLRC_RX_MATCH_SYNCWORD_1_OR_2_OR_3, 7 )

typedef enum ral_flrc_rx_match_sync_word_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_RX_MATCH_SYNCWORD( X )
#undef X
} ral_flrc_rx_match_sync_word_t;

static inline const char* ral_flrc_rx_match_sync_word_to_str( const ral_flrc_rx_match_sync_word_t match_sync_word )
{
    switch( match_sync_word )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_RX_MATCH_SYNCWORD( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_RX_MATCH_SYNCWORD";
}

#define RAL_FLRC_PREAMBLE_LENGTH( X )        \
    X( RAL_FLRC_PREAMBLE_LENGTH_4_BITS, 0 )  \
    X( RAL_FLRC_PREAMBLE_LENGTH_8_BITS, 1 )  \
    X( RAL_FLRC_PREAMBLE_LENGTH_12_BITS, 2 ) \
    X( RAL_FLRC_PREAMBLE_LENGTH_16_BITS, 3 ) \
    X( RAL_FLRC_PREAMBLE_LENGTH_20_BITS, 4 ) \
    X( RAL_FLRC_PREAMBLE_LENGTH_24_BITS, 5 ) \
    X( RAL_FLRC_PREAMBLE_LENGTH_28_BITS, 6 ) \
    X( RAL_FLRC_PREAMBLE_LENGTH_32_BITS, 7 )

typedef enum ral_flrc_preamble_length_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_PREAMBLE_LENGTH( X )
#undef X
} ral_flrc_preamble_length_t;

static inline const char* ral_flrc_preamble_length_to_str( const ral_flrc_preamble_length_t len )
{
    switch( len )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_PREAMBLE_LENGTH( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_PREAMBLE_LENGTH";
}

/**
 * @brief GFSK preamble length Rx detection size enumeration definition
 */
#define RAL_GFSK_PREAMBLE_DETECTOR( X )            \
    X( RAL_GFSK_PREAMBLE_DETECTOR_OFF, 0 )         \
    X( RAL_GFSK_PREAMBLE_DETECTOR_MIN_8BITS, 8 )   \
    X( RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS, 16 ) \
    X( RAL_GFSK_PREAMBLE_DETECTOR_MIN_24BITS, 24 ) \
    X( RAL_GFSK_PREAMBLE_DETECTOR_MIN_32BITS, 32 )

typedef enum ral_gfsk_preamble_detector_e
{
#define X( identifier, value ) identifier = value,
    RAL_GFSK_PREAMBLE_DETECTOR( X )
#undef X
} ral_gfsk_preamble_detector_t;

static inline const char* ral_gfsk_preamble_detector_to_str( const ral_gfsk_preamble_detector_t detector )
{
    switch( detector )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_GFSK_PREAMBLE_DETECTOR( X )
#undef X
    }
    return "UNKNOWN_RAL_GFSK_PREAMBLE_DETECTOR";
}

/**
 * @brief LoRa packet length enumeration definition
 */
#define RAL_LORA_PKT_LEN_MODES( X )                                        \
    X( RAL_LORA_PKT_EXPLICIT /* LoRa header included in the packet */, 0 ) \
    X( RAL_LORA_PKT_IMPLICIT /* LoRa header not included in the packet */, 1 )

typedef enum ral_lora_pkt_len_modes_e
{
#define X( identifier, value ) identifier = value,
    RAL_LORA_PKT_LEN_MODES( X )
#undef X
} ral_lora_pkt_len_modes_t;

static inline const char* ral_lora_pkt_len_modes_to_str( const ral_lora_pkt_len_modes_t mode )
{
    switch( mode )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_LORA_PKT_LEN_MODES( X )
#undef X
    }
    return "UNKNOWN_RAL_LORA_PKT_LEN_MODE";
}

#define RAL_PKT_TYPE( X )     \
    X( RAL_PKT_TYPE_GFSK, 0 ) \
    X( RAL_PKT_TYPE_LORA, 1 ) \
    X( RAL_PKT_TYPE_FLRC, 2 ) \
    X( RAL_PKT_TYPE_RTTOF, 3 )

typedef enum ral_pkt_types_e
{
#define X( identifier, value ) identifier = value,
    RAL_PKT_TYPE( X )
#undef X
} ral_pkt_type_t;

static inline const char* ral_pkt_type_to_str( const ral_pkt_type_t type )
{
    switch( type )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_PKT_TYPE( X )
#undef X
    }
    return "UNKNOWN_RAL_PKT_TYPE";
}

/**
 * @brief GFSK address filtering configuration enumeration definition
 */
#define RAL_GFSK_ADDRESS_FILTERING( X )             \
    X( RAL_GFSK_ADDRESS_FILTERING_DISABLE, 0 )      \
    X( RAL_GFSK_ADDRESS_FILTERING_NODE_ADDRESS, 1 ) \
    X( RAL_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES, 2 )

typedef enum ral_gfsk_address_filtering_e
{
#define X( identifier, value ) identifier = value,
    RAL_GFSK_ADDRESS_FILTERING( X )
#undef X
} ral_gfsk_address_filtering_t;

static inline const char* ral_gfsk_address_filtering_to_str( const ral_gfsk_address_filtering_t filtering )
{
    switch( filtering )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_GFSK_ADDRESS_FILTERING( X )
#undef X
    }
    return "UNKNOWN_RAL_GFSK_ADDRESS_FILTERING";
}

/*!
 * \brief GFSK data whitening configurations
 */
#define RAL_GFSK_DC_FREE( X )                                            \
    X( RAL_GFSK_DC_FREE_OFF /* GFSK data whitening deactivated */, 0 )   \
    X( RAL_GFSK_DC_FREE_WHITENING /* GFSK data whitening enabled */, 1 ) \
    X( RAL_GFSK_DC_FREE_WHITENING_SX128X_COMP /* GFSK data whitening enabled - SX128x compatibility */, 2 )

typedef enum ral_gfsk_dc_free_e
{
#define X( identifier, value ) identifier = value,
    RAL_GFSK_DC_FREE( X )
#undef X
} ral_gfsk_dc_free_t;

static inline const char* ral_gfsk_dc_free_to_str( const ral_gfsk_dc_free_t dc )
{
    switch( dc )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_GFSK_DC_FREE( X )
#undef X
    }
    return "UNKNOWN_RAL_GFSK_DC_FREE";
}

#define RAL_RX_STATUS_GFSK( X )                 \
    X( RAL_RX_STATUS_PKT_SENT, ( 1 << 0 ) )     \
    X( RAL_RX_STATUS_PKT_RECEIVED, ( 1 << 1 ) ) \
    X( RAL_RX_STATUS_ABORT_ERROR, ( 1 << 2 ) )  \
    X( RAL_RX_STATUS_LENGTH_ERROR, ( 1 << 3 ) ) \
    X( RAL_RX_STATUS_CRC_ERROR, ( 1 << 4 ) )    \
    X( RAL_RX_STATUS_ADDR_ERROR, ( 1 << 5 ) )

enum ral_rx_status_gfsk_e
{
#define X( identifier, value ) identifier = value,
    RAL_RX_STATUS_GFSK( X )
#undef X
};

static inline const char* ral_rx_status_gfsk_to_str( const enum ral_rx_status_gfsk_e status )
{
    switch( status )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_RX_STATUS_GFSK( X )
#undef X
    }
    return "UNKNOWN_RAL_RX_STATUS_GFSK";
}

/*!
 * @brief GFSK Header Type configurations
 *
 * This parameter indicates whether or not the payload length is sent and read over the air.
 *
 * If the payload length is known beforehand by both transmitter and receiver, therefore there is no need to send it
 * over the air and the parameter can be set to RAL_GFSK_PKT_FIX_LEN. Otherwise, setting this parameter to
 * RAL_GFSK_PKT_VAR_LEN will make the modem to automatically prepend a byte containing the payload length to the payload
 * on transmitter side. On receiver side, this first byte is read to set the payload length to read.
 *
 * This configuration is only available for GFSK packet types.
 */
#define RAL_GFSK_PKT_LEN_MODES( X )                                                 \
    X( RAL_GFSK_PKT_FIX_LEN /* Payload length is not sent/read over the air */, 0 ) \
    X( RAL_GFSK_PKT_VAR_LEN /* Payload length is sent/read over the air */, 1 )     \
    X( RAL_GFSK_PKT_VAR_LEN_SX128X_COMP /* Payload length is sent/read over the air - SX128x compatibility */, 2 )

typedef enum ral_gfsk_pkt_len_modes_e
{
#define X( identifier, value ) identifier = value,
    RAL_GFSK_PKT_LEN_MODES( X )
#undef X
} ral_gfsk_pkt_len_modes_t;

static inline const char* ral_gfsk_pkt_len_modes_to_str( const ral_gfsk_pkt_len_modes_t mode )
{
    switch( mode )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_GFSK_PKT_LEN_MODES( X )
#undef X
    }
    return "UNKNOWN_RAL_GFSK_PKT_LEN_MODE";
}

typedef uint16_t ral_rx_status_gfsk_t;

typedef struct ral_gfsk_rx_pkt_status_s
{
    ral_rx_status_gfsk_t rx_status;
    int16_t              rssi_sync_in_dbm;
    int16_t              rssi_avg_in_dbm;
} ral_gfsk_rx_pkt_status_t;

typedef struct ral_lora_rx_pkt_status_s
{
    int16_t rssi_pkt_in_dbm;
    int16_t snr_pkt_in_db;
    int16_t signal_rssi_pkt_in_dbm;  // Last packet received LoRa signal RSSI power in dB (after despreading)
    int32_t freq_offset_hz;          // Frequency offset of the last packet received in Hz
} ral_lora_rx_pkt_status_t;

typedef struct ral_flrc_rx_pkt_status_s
{
    uint16_t packet_length_bytes;       //!< Length of last received packet in bytes
    uint8_t  syncword_index;            //!< Syncword index of the last received packet - in [1, 2, 3]
    int16_t  rssi_avg_in_dbm;           //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_avg_half_dbm_count;   //!< Count of 0.5 dBm to subtract to rssi_avg_in_dbm value in dBm
    int16_t  rssi_sync_in_dbm;          //!< RSSI in dBm - averaged over the last received packet
    uint8_t  rssi_sync_half_dbm_count;  //!< Count of 0.5 dBm to subtract to rssi_sync_in_dbm value in dBm
    int32_t  freq_offset_hz;            //!< Frequency offset of the last packet received in Hz
} ral_flrc_rx_pkt_status_t;

/*!
 *  Represents the number of symbs to be used for channel activity detection operation
 */
typedef enum ral_lora_cad_symbs_e
{
    RAL_LORA_CAD_01_SYMB,
    RAL_LORA_CAD_02_SYMB,
    RAL_LORA_CAD_04_SYMB,
    RAL_LORA_CAD_08_SYMB,
    RAL_LORA_CAD_16_SYMB,
} ral_lora_cad_symbs_t;

/*!
 *  Represents the Channel Activity Detection actions after the CAD operation is finished
 */
typedef enum ral_lora_cad_exit_modes_e
{
    RAL_LORA_CAD_ONLY,
    RAL_LORA_CAD_RX,
    RAL_LORA_CAD_LBT,
} ral_lora_cad_exit_modes_t;

#define RAL_FLRC_RAW_BIT_RATE( X )           \
    X( RAL_FLRC_RAW_BIT_RATE_0_260_MBPS, 0 ) \
    X( RAL_FLRC_RAW_BIT_RATE_0_325_MBPS, 1 ) \
    X( RAL_FLRC_RAW_BIT_RATE_0_520_MBPS, 2 ) \
    X( RAL_FLRC_RAW_BIT_RATE_0_650_MBPS, 3 ) \
    X( RAL_FLRC_RAW_BIT_RATE_1_040_MBPS, 4 ) \
    X( RAL_FLRC_RAW_BIT_RATE_1_300_MBPS, 5 ) \
    X( RAL_FLRC_RAW_BIT_RATE_2_080_MBPS, 6 ) \
    X( RAL_FLRC_RAW_BIT_RATE_2_600_MBPS, 7 )

typedef enum ral_flrc_raw_bit_rate_e
{
#define X( identifier, value ) identifier = value,
    RAL_FLRC_RAW_BIT_RATE( X )
#undef X
} ral_flrc_raw_bit_rate_t;

static inline const char* ral_flrc_raw_bit_rate_to_str( const ral_flrc_raw_bit_rate_t raw_bit_rate )
{
    switch( raw_bit_rate )
    {
#define X( identifier, _ ) \
    case identifier:       \
        return #identifier;
        RAL_FLRC_RAW_BIT_RATE( X )
#undef X
    }
    return "UNKNOWN_RAL_FLRC_RAW_BIT_RATE";
}

/**
 * @brief GFSK modulation parameters structure definition
 */
typedef struct ral_gfsk_mod_params_s
{
    uint32_t               br_in_bps;
    uint32_t               fdev_in_hz;
    uint32_t               bw_dsb_in_hz;
    ral_gfsk_pulse_shape_t pulse_shape;
} ral_gfsk_mod_params_t;

/**
 * @brief GFSK packet parameters structure definition
 */
typedef struct ral_gfsk_pkt_params_s
{
    uint16_t                     preamble_len_in_bits;   //!< GFSK Preamble length in bits
    ral_gfsk_preamble_detector_t preamble_detector;      //!< GFSK Preamble detection length
    uint8_t                      sync_word_len_in_bits;  //!< GFSK Sync word length in bits
    ral_gfsk_address_filtering_t address_filtering;      //!< GFSK Address filtering configuration
    ral_gfsk_pkt_len_modes_t     header_type;            //!< GFSK Header type
    uint16_t                     pld_len_in_bytes;       //!< GFSK Payload length in bytes
    ral_gfsk_crc_type_t          crc_type;               //!< GFSK CRC type configuration
    ral_gfsk_dc_free_t           dc_free;                //!< GFSK Whitening configuration
} ral_gfsk_pkt_params_t;

/**
 * @brief LoRa modulation parameters structure definition
 */
typedef struct ral_lora_mod_params_s
{
    ral_lora_sf_t sf;    //!< LoRa Spreading Factor
    ral_lora_bw_t bw;    //!< LoRa Bandwidth
    ral_lora_cr_t cr;    //!< LoRa Coding Rate
    uint8_t       ldro;  //!< LoRa Low DataRate Optimization configuration
} ral_lora_mod_params_t;

/**
 * @brief LoRa packet parameters structure definition
 */
typedef struct ral_lora_pkt_params_s
{
    uint16_t                 preamble_len_in_symb;  //!< LoRa preamble length in symbols
    ral_lora_pkt_len_modes_t header_type;           //!< LoRa header type
    uint8_t                  pld_len_in_bytes;      //!< LoRa payload length in bytes
    bool                     crc_is_on;             //!< LoRa CRC activation
    bool                     invert_iq_is_on;       //!< LoRa IQ polarity setup
} ral_lora_pkt_params_t;

/*!
 * Defines \ref ral_set_cad_params function parameters.
 */
typedef struct ral_lora_cad_param_s
{
    ral_lora_cad_symbs_t      cad_symb_nb;
    uint8_t                   cad_det_peak_in_symb;
    uint8_t                   cad_det_min_in_symb;
    ral_lora_cad_exit_modes_t cad_exit_mode;
    uint32_t                  cad_timeout_in_ms;
} ral_lora_cad_params_t;

/**
 * @brief FLRC modulation parameters structure definition
 */
typedef struct ral_flrc_mod_params_s
{
    ral_flrc_raw_bit_rate_t raw_bit_rate;
    ral_flrc_cr_t           cr;
    ral_flrc_pulse_shape_t  pulse_shape;
} ral_flrc_mod_params_t;

/**
 * @brief FLRC packet parameters structure definition
 */
typedef struct ral_flrc_pkt_params_s
{
    ral_flrc_preamble_length_t    preamble_len;
    ral_flrc_sync_word_len_t      sync_word_len;
    ral_flrc_tx_syncword_t        tx_syncword;
    ral_flrc_rx_match_sync_word_t match_sync_word;
    bool                          pld_is_fix;
    uint16_t                      pld_len_in_bytes;
    ral_flrc_crc_type_t           crc_type;
} ral_flrc_pkt_params_t;

/*!
 * @brief LR FHSS parameters
 */
typedef struct ral_lr_fhss_params_s
{
    lr_fhss_v1_params_t lr_fhss_params;
    uint32_t            center_frequency_in_hz;
    int8_t              device_offset;
} ral_lr_fhss_params_t;

/*!
 * @brief Memory allocated to hold LR-FHSS state
 *
 * This memory is to be allocated by the caller. It is used by the radio driver to store internal
 * LR-FHSS state. This is not needed by all radio drivers, but in order to support all transceivers,
 * this should point to a block of at least RAL_LR_FHSS_STATE_MAXSIZE bytes of RAM. If you are only
 * targeting LR11XX, for instance, you may use NULL.
 */
typedef void* ral_lr_fhss_memory_state_t;

/**
 * @brief IRQ definitions
 *
 * @remark This enumeration cannot accept more than 32 entries in addition to @ref RAL_IRQ_NONE and @ref RAL_IRQ_ALL -
 * this is related to the fact that @ref ral_irq_t is 32-bit long
 */
enum ral_irq_e
{
    RAL_IRQ_NONE                           = ( 0 << 0 ),
    RAL_IRQ_TX_DONE                        = ( 1 << 1 ),
    RAL_IRQ_RX_DONE                        = ( 1 << 2 ),
    RAL_IRQ_RX_TIMEOUT                     = ( 1 << 3 ),
    RAL_IRQ_RX_PREAMBLE_DETECTED           = ( 1 << 4 ),
    RAL_IRQ_RX_HDR_OK                      = ( 1 << 5 ),
    RAL_IRQ_RX_HDR_ERROR                   = ( 1 << 6 ),
    RAL_IRQ_RX_CRC_ERROR                   = ( 1 << 7 ),
    RAL_IRQ_CAD_DONE                       = ( 1 << 8 ),
    RAL_IRQ_CAD_OK                         = ( 1 << 9 ),
    RAL_IRQ_LR_FHSS_HOP                    = ( 1 << 10 ),
    RAL_IRQ_WIFI_SCAN_DONE                 = ( 1 << 11 ),
    RAL_IRQ_GNSS_SCAN_DONE                 = ( 1 << 12 ),
    RAL_IRQ_RX_FIFO_LEVEL                  = ( 1 << 13 ),
    RAL_IRQ_TX_FIFO_LEVEL                  = ( 1 << 14 ),
    RAL_IRQ_RX_TIMESTAMP                   = ( 1 << 15 ),
    RAL_IRQ_TX_TIMESTAMP                   = ( 1 << 16 ),
    RAL_IRQ_RTTOF_REQ_DISCARDED            = ( 1 << 17 ),
    RAL_IRQ_RTTOF_RESP_DONE                = ( 1 << 18 ),
    RAL_IRQ_RTTOF_EXCH_VALID               = ( 1 << 19 ),
    RAL_IRQ_RTTOF_TIMEOUT                  = ( 1 << 20 ),
    RAL_IRQ_CMD_ERROR                      = ( 1 << 21 ),
    RAL_IRQ_ERROR                          = ( 1 << 22 ),
    RAL_IRQ_RX_LEN_ERROR                   = ( 1 << 23 ),
    RAL_IRQ_ADDR_ERROR                     = ( 1 << 24 ),
    RAL_IRQ_LOW_BATTERY                    = ( 1 << 25 ),
    RAL_IRQ_LORA_RX_HEADER_TIMESTAMP       = ( 1 << 26 ),
    RAL_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE = ( 1 << 27 ),
    RAL_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD    = ( 1 << 28 ),
    RAL_IRQ_PA_OVP_OCP                     = ( 1 << 29 ),
    RAL_IRQ_RTTOF_RESPONDER_REQUEST_VALID  = ( 1 << 30 ),
    RAL_IRQ_ALL = RAL_IRQ_TX_DONE | RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_PREAMBLE_DETECTED |
                  RAL_IRQ_RX_HDR_OK | RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK |
                  RAL_IRQ_LR_FHSS_HOP | RAL_IRQ_WIFI_SCAN_DONE | RAL_IRQ_GNSS_SCAN_DONE | RAL_IRQ_RX_FIFO_LEVEL |
                  RAL_IRQ_TX_FIFO_LEVEL | RAL_IRQ_RX_TIMESTAMP | RAL_IRQ_TX_TIMESTAMP | RAL_IRQ_RTTOF_REQ_DISCARDED |
                  RAL_IRQ_RTTOF_RESP_DONE | RAL_IRQ_RTTOF_EXCH_VALID | RAL_IRQ_RTTOF_TIMEOUT | RAL_IRQ_CMD_ERROR |
                  RAL_IRQ_ERROR | RAL_IRQ_RX_LEN_ERROR | RAL_IRQ_ADDR_ERROR | RAL_IRQ_LOW_BATTERY |
                  RAL_IRQ_LORA_RX_HEADER_TIMESTAMP | RAL_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE |
                  RAL_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD | RAL_IRQ_PA_OVP_OCP | RAL_IRQ_RTTOF_RESPONDER_REQUEST_VALID,

};

typedef uint32_t ral_irq_t;

enum ral_radio_fifo_flag_e
{
    RAL_RADIO_FIFO_FLAG_NONE           = ( 0 << 0 ),
    RAL_RADIO_FIFO_FLAG_EMPTY          = ( 1 << 0 ),
    RAL_RADIO_FIFO_FLAG_THRESHOLD_LOW  = ( 1 << 1 ),
    RAL_RADIO_FIFO_FLAG_THRESHOLD_HIGH = ( 1 << 2 ),
    RAL_RADIO_FIFO_FLAG_FULL           = ( 1 << 3 ),
    RAL_RADIO_FIFO_FLAG_OVERFLOW       = ( 1 << 4 ),
    RAL_RADIO_FIFO_FLAG_UNDERFLOW      = ( 1 << 5 ),

    RAL_RADIO_FIFO_FLAG_ALL = RAL_RADIO_FIFO_FLAG_EMPTY | RAL_RADIO_FIFO_FLAG_THRESHOLD_LOW |
                              RAL_RADIO_FIFO_FLAG_THRESHOLD_HIGH | RAL_RADIO_FIFO_FLAG_FULL |
                              RAL_RADIO_FIFO_FLAG_OVERFLOW | RAL_RADIO_FIFO_FLAG_UNDERFLOW,
};

typedef uint8_t ral_radio_fifo_flag_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION PROTOTYPES ----------------------------------------------
 */

static inline uint8_t ral_compute_lora_ldro( const ral_lora_sf_t sf, const ral_lora_bw_t bw )
{
    // PPM offset is activated, except for the cases that follow
    uint8_t ppm_offset = 1;  // 0: no_ppm; 1: ppm_offset_activated

    if( ( sf != RAL_LORA_SF11 ) && ( sf != RAL_LORA_SF12 ) )
    {
        // 1. If sf is not SF11 nor SF12: no ppm offset
        ppm_offset = 0;
    }
    else
    {
        // 2. Else it depends on the bandwidth
        switch( bw )
        {
        case RAL_LORA_BW_1000_KHZ:
        case RAL_LORA_BW_500_KHZ:
        {
            ppm_offset = 0;
            break;
        }
        case RAL_LORA_BW_250_KHZ:
        {
            if( sf == RAL_LORA_SF11 )
            {
                ppm_offset = 0;
            }
            else if( sf == RAL_LORA_SF12 )
            {
                ppm_offset = 1;
            }
            break;
        }
        case RAL_LORA_BW_800_KHZ:
        case RAL_LORA_BW_400_KHZ:
        case RAL_LORA_BW_200_KHZ:
        {
            // For SX1280 compatibility
            ppm_offset = 1;
            break;
        }
        default:
        {
            // All other is set to 1
        }
        }
    }
    return ppm_offset;
}

#ifdef __cplusplus
}
#endif

#endif  // RAL_DEFS_H

/* --- EOF ------------------------------------------------------------------ */
