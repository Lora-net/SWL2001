/*!
 * @file      lr20xx_radio_bpsk_types.h
 *
 * @brief     BPSK radio types driver definition for LR20XX
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

#ifndef LR20XX_RADIO_BPSK_TYPES_H
#define LR20XX_RADIO_BPSK_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

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

/*!
 * @brief Bitrate unit
 */
typedef enum lr20xx_radio_bpsk_mod_params_br_unit_e
{
    LR20XX_RADIO_BPSK_MOD_PARAMS_BR_IN_BPS         = 0x00,  //!<
    LR20XX_RADIO_BPSK_MOD_PARAMS_BR_IN_BPS_DIV_256 = 0x01,  //!<
} lr20xx_radio_bpsk_mod_params_br_unit_t;

/*!
 * @brief Shape filter
 */
typedef enum lr20xx_radio_bpsk_mod_params_shape_filter_e
{
    LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_NONE            = 0x00,  //!< No filter
    LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_0_3 = 0x04,  //!< Gaussian - BT 0.3
    LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_0_5 = 0x05,  //!< Gaussian - BT 0.5
    LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_0_7 = 0x06,  //!< Gaussian - BT 0.7
    LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_1_0 = 0x07,  //!< Gaussian - BT 1.0
} lr20xx_radio_bpsk_mod_params_shape_filter_t;

/**
 * @brief Differential encoding enable value
 */
typedef enum
{
    LR20XX_RADIO_BPSK_DIFFERENTIAL_ENCODING_DISABLE = 0x00,  //!< Disable differential encoding
    LR20XX_RADIO_BPSK_DIFFERENTIAL_ENCODING_ENABLE  = 0x01,  //!< Enable differential encoding (DBPSK)
} lr20xx_radio_bpsk_differential_encoding_enable_t;

/**
 * @brief BPSK modulation parameters
 */
typedef struct
{
    lr20xx_radio_bpsk_mod_params_br_unit_t      bitrate_unit;  //!< Unit of @ref bitrate field
    uint32_t                                    bitrate;       //!< Bitrate, unit depends on @ref bitrate_unit value
    lr20xx_radio_bpsk_mod_params_shape_filter_t pulse_shape;   //!< Signal shape
    lr20xx_radio_bpsk_differential_encoding_enable_t
         differential_encoding_enable;         //!< Enable or disable the differential encoding
    bool differential_encoding_init_previous;  //!< Initial value for previous value of differential encoding. Only
                                               //!< meaningful if @ref differential_encoding_enable is @ref
                                               //!< LR20XX_RADIO_BPSK_DIFFERENTIAL_ENCODING_ENABLE
    bool differential_encoding_init_parity;  //!< Initial value for the parity of differential encoding. Only meaningful
                                             //!< if @ref differential_encoding_enable is @ref
                                             //!< LR20XX_RADIO_BPSK_DIFFERENTIAL_ENCODING_ENABLE
} lr20xx_radio_bpsk_mod_params_t;

/**
 * @brief PHY mode of BPSK
 */
typedef enum
{
    LR20XX_RADIO_BPSK_PHY_MODE_RAW    = 0x00,  //!< The data written in FIFO is the packet to send over-the-air
    LR20XX_RADIO_BPSK_PHY_MODE_SIGFOX = 0x01,  //!< The data written in FIFO is the UL-CONTAINER. CRC, convolution,
                                               //!< frame type and preamble are added by the chip
} lr20xx_radio_bpsk_phy_mode_t;

/**
 * @brief Sigfox message type
 */
typedef enum
{
    LR20XX_RADIO_BPSK_SIGFOX_MESSAGE_TYPE_APPLICATION = 0x00,
    LR20XX_RADIO_BPSK_SIGFOX_MESSAGE_TYPE_CONTROL     = 0x01,
} lr20xx_radio_bpsk_sigfox_message_type_t;

/**
 * @brief Sigfox frame emission rank
 */
typedef enum
{
    LR20XX_RADIO_BPSK_SIGFOX_FRAME_RANK_FIRST_LAST = 0x00,
    LR20XX_RADIO_BPSK_SIGFOX_FRAME_RANK_SECOND     = 0x01,
    LR20XX_RADIO_BPSK_SIGFOX_FRAME_RANK_THIRD      = 0x02,
} lr20xx_radio_bpsk_sigfox_frame_rank_t;

/**
 * @brief BPSK packet parameters
 */
typedef struct
{
    uint8_t payload_length;                 //!< Payload length. Unit depends on @p phy_mode:
                                            //!< | phy_mode | payload_length unit |
                                            //!< | -- | -- |
                                            //!< | LR20XX_RADIO_BPSK_PHY_MODE_RAW | bits |
                                            //!< | LR20XX_RADIO_BPSK_PHY_MODE_SIGFOX | bytes |
    lr20xx_radio_bpsk_phy_mode_t phy_mode;  //!< Selector for PHY mode
    lr20xx_radio_bpsk_sigfox_message_type_t
        sigfox_message_type;  //!< Sigfox message type, only meaningful if @ref
                              //!< phy_mode is @ref LR20XX_RADIO_BPSK_PHY_MODE_SIGFOX
    lr20xx_radio_bpsk_sigfox_frame_rank_t
        sigfox_frame_rank;  //!< Sigfox frame rank, only meaningful if @ref phy_mode is
                            //!< @ref LR20XX_RADIO_BPSK_PHY_MODE_SIGFOX
} lr20xx_radio_bpsk_pkt_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_BPSK_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
