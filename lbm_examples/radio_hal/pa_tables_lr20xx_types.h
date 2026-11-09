/*!
 * \file      smtc_shield_lr20xx_types.h
 *
 * \brief     Types common to all lr20xx-based shields
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

#ifndef SMTC_SHIELD_LR20XX_TYPES_H
#define SMTC_SHIELD_LR20XX_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_lora_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief lower boundary for a frequency to be considered sub-ghz
 */
#define SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MIN 150000000

/**
 * @brief lower boundary for a frequency to be considered sub-ghz
 */
#define SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MED 600000000

/**
 * @brief upper boundary for a frequency to be considered sub-ghz
 */
#define SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MAX 960000000

/**
 * @brief lower boundary for a frequency to be considered "high-frequency"
 */
#define SMTC_SHIELD_LR20XX_LF_HF_FREQ_THRESHOLD 1900000000

/**
 * @brief lower boundary for a frequency to be considered in the 2ghz range
 */
#define SMTC_SHIELD_LR20XX_2GHZ_FREQ_MIN 2000000000

/**
 * @brief upper boundary for a frequency to be considered in the 2ghz range
 */
#define SMTC_SHIELD_LR20XX_2GHZ_FREQ_MAX 2100000000

/**
 * @brief lower boundary for a frequency to be considered in the 2.4ghz range
 */
#define SMTC_SHIELD_LR20XX_2_4GHZ_FREQ_MIN 2400000000

/**
 * @brief upper boundary for a frequency to be considered in the 2ghz range
 */
#define SMTC_SHIELD_LR20XX_2_4GHZ_FREQ_MAX 2500000000

/**
 * @brief lower output power limit for low-frequency path in dbm
 */
#define SMTC_SHIELD_LR20XX_MIN_PWR -17

/**
 * @brief upper output power limit for low-frequency path in dbm
 */
#define SMTC_SHIELD_LR20XX_MAX_PWR 22

/**
 * @brief lower output power limit for high-frequency path in dbm
 */
#define SMTC_SHIELD_LR20XX_MIN_PWR_HF -17

/**
 * @brief upper output power limit for high-frequency path in dbm
 */
#define SMTC_SHIELD_LR20XX_MAX_PWR_HF 12

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Power amplifier and output power configurations structure definition
 */
typedef struct smtc_shield_lr20xx_pa_pwr_cfg_s
{
    int8_t                       power;
    lr20xx_radio_common_pa_cfg_t pa_config;
} smtc_shield_lr20xx_pa_pwr_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_LR20XX_TYPES_H
