/*!
 * @file      lr20xx_radio_lr_fhss_types.h
 *
 * @brief     LR-FHSS types definition for LR20XX
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

#ifndef LR20XX_RADIO_LR_FHSS_TYPES_H
#define LR20XX_RADIO_LR_FHSS_TYPES_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

#define LR20XX_RADIO_LR_FHSS_MAX_HOPPING_FREQUENCIES ( 40 )

/**
 * @brief LR-FHSS modulation type
 */
typedef enum lr20xx_radio_lr_fhss_modulation_type_e
{
    LR20XX_RADIO_LR_FHSS_MODULATION_TYPE_GMSK_488 = 0,
} lr20xx_radio_lr_fhss_modulation_type_t;

/**
 * @brief LR-FHSS coding rate
 */
typedef enum lr20xx_radio_lr_fhss_cr_e
{
    LR20XX_RADIO_LR_FHSS_CR_5_6 = 0x00,
    LR20XX_RADIO_LR_FHSS_CR_2_3 = 0x01,
    LR20XX_RADIO_LR_FHSS_CR_1_2 = 0x02,
    LR20XX_RADIO_LR_FHSS_CR_1_3 = 0x03,
} lr20xx_radio_lr_fhss_cr_t;

/**
 * @brief LR-FHSS grid
 */
typedef enum lr20xx_radio_lr_fhss_grid_e
{
    LR20XX_RADIO_LR_FHSS_GRID_25391_HZ = 0x00,
    LR20XX_RADIO_LR_FHSS_GRID_3906_HZ  = 0x01,
} lr20xx_radio_lr_fhss_grid_t;

/**
 * @brief LR-FHSS bandwidth
 */
typedef enum lr20xx_radio_lr_fhss_bw_e
{
    LR20XX_RADIO_LR_FHSS_BW_39063_HZ   = 0x00,
    LR20XX_RADIO_LR_FHSS_BW_85938_HZ   = 0x01,
    LR20XX_RADIO_LR_FHSS_BW_136719_HZ  = 0x02,
    LR20XX_RADIO_LR_FHSS_BW_183594_HZ  = 0x03,
    LR20XX_RADIO_LR_FHSS_BW_335938_HZ  = 0x04,
    LR20XX_RADIO_LR_FHSS_BW_386719_HZ  = 0x05,
    LR20XX_RADIO_LR_FHSS_BW_722656_HZ  = 0x06,
    LR20XX_RADIO_LR_FHSS_BW_773438_HZ  = 0x07,
    LR20XX_RADIO_LR_FHSS_BW_1523438_HZ = 0x08,
    LR20XX_RADIO_LR_FHSS_BW_1574219_HZ = 0x09,
} lr20xx_radio_lr_fhss_bw_t;

/**
 * @brief LR-FHSS hopping configuration
 *
 */
typedef enum lr20xx_radio_lr_fhss_hopping_e
{
    LR20XX_RADIO_LR_FHSS_HOPPING_OFF = 0x00,  //!< Hopping disabled
    LR20XX_RADIO_LR_FHSS_HOPPING_ON  = 0x01,  //!< Hopping enabled
    LR20XX_RADIO_LR_FHSS_HOPPING_TEST_PAYLOAD =
        0x02,  //!< Test mode: hopping disabled but payload is encoded as for hopping
    LR20XX_RADIO_LR_FHSS_HOPPING_TEST_PA =
        0x03,  //!< Test mode: hopping disabled but power amplifier ramps up/down as it would do for hopping
} lr20xx_radio_lr_fhss_hopping_t;

/*!
 * @brief LR-FHSS parameter structure
 */
typedef struct
{
    uint8_t syncword_header_count;              //!< Number of syncword header sent. Possible values in [1,4] included
    lr20xx_radio_lr_fhss_cr_t              cr;  //!< Coding rate for Forward Error Correction
    lr20xx_radio_lr_fhss_modulation_type_t modulation_type;  //!< Physical modulation layer
    lr20xx_radio_lr_fhss_grid_t            grid;             //!< Frequency grid space for hopping
    lr20xx_radio_lr_fhss_hopping_t         hopping_mode;     //!< Hopping mode
    lr20xx_radio_lr_fhss_bw_t              bw;               //!< Bandwidth of the hopping sequence
    uint16_t hop_sequence_id;  //< Seed of hopping sequence. Only the 9 LSBs are meaningful
    int8_t   device_offset;    //!< Per device offset to avoid collisions over the air. Possible values:
                               //!< - if lr_fhss_params.grid == LR20XX_RADIO_LR_FHSS_GRID_25391_HZ:
                               //!<     [-26, 25]
                               //!< - if lr_fhss_params.grid == LR20XX_RADIO_LR_FHSS_GRID_3906_HZ:
                               //!<     [-4, 3]
} lr20xx_radio_lr_fhss_params_t;

/**
 * @brief Hop frequency/symbols structure
 */
typedef struct
{
    uint32_t frequency;  //!< Frequency of the hop
    uint16_t n_symbols;  //!< Number of symbols in the hop
} lr20xx_radio_lr_fhss_hopping_frequency_t;

/**
 * @brief LR-FHSS hopping table configuration structure
 */
typedef struct
{
    bool hopping_enabled;    //!< True if hopping is enabled
    bool frequencies_in_hz;  //!< True if frequencies are provided in Hz, false if frequencies are provided in PLL steps
    uint16_t                                 pkt_length;
    uint8_t                                  n_frequencies;
    uint8_t                                  n_hopping_blocks;
    uint8_t                                  n_hopping_table;
    lr20xx_radio_lr_fhss_hopping_frequency_t hopping_frequencies[LR20XX_RADIO_LR_FHSS_MAX_HOPPING_FREQUENCIES];
} lr20xx_radio_lr_fhss_hopping_table_t;

#endif  // LR20XX_RADIO_LR_FHSS_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
