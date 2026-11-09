/*!
 * @file      smtc_shield_lr20xx_common.c
 *
 * @brief     Implementation common to LR20xx shields
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "pa_tables_lr20xx_types.h"
#include "pa_tables_lr20xx_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const smtc_shield_lr20xx_pa_pwr_cfg_t smtc_shield_lr20xx_common_pa_lf_default_pwr_cfg = {
    .power     = 0,
    .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                   .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                   .pa_lf_duty_cycle = 6,
                   .pa_lf_slices     = 7,
                   .pa_hf_duty_cycle = 16 }
};

static const smtc_shield_lr20xx_pa_pwr_cfg_t smtc_shield_lr20xx_common_pa_hf_default_pwr_cfg = {
    .power     = 0,
    .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                   .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                   .pa_lf_duty_cycle = 7,
                   .pa_lf_slices     = 6,
                   .pa_hf_duty_cycle = 16 }
};

const smtc_shield_lr20xx_pa_pwr_cfg_t
    smtc_shield_lr2021_common_pa_low_lf_pwr_cfg[2 * ( SMTC_SHIELD_LR20XX_MAX_PWR - SMTC_SHIELD_LR20XX_MIN_PWR ) + 1] = {

        /* Expected output power = -17dBm */
        {
            .power     = -16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -16.5dBm */
        {
            .power     = -14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -16dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -15.5dBm */
        {
            .power     = -6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -15dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -14.5dBm */
        {
            .power     = -10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -14dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -13.5dBm */
        {
            .power     = -14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -13dBm */
        {
            .power     = -18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -12.5dBm */
        {
            .power     = 14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -12dBm */
        {
            .power     = -12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 5,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -11.5dBm */
        {
            .power     = -8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -11dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -10.5dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -10dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -9.5dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -9dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -8.5dBm */
        {
            .power     = -8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -8dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -7.5dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -7dBm */
        {
            .power     = 12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -6.5dBm */
        {
            .power     = 18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -6dBm */
        {
            .power     = 14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -5.5dBm */
        {
            .power     = 20,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -5dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -4.5dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -4dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -3.5dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -3dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -2.5dBm */
        {
            .power     = 26,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -2dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -1.5dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -1dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -0.5dBm */
        {
            .power     = 36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 0dBm */
        {
            .power     = 12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 0.5dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 1dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 1.5dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 2dBm */
        {
            .power     = 18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 2.5dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 3dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 3.5dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 4dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 4.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 5dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 5.5dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 6dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 6.5dBm */
        {
            .power     = 26,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 7dBm */
        {
            .power     = 26,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 7.5dBm */
        {
            .power     = 26,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 8dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 8.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 9dBm */
        {
            .power     = 34,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 9.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 10dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 10.5dBm */
        {
            .power     = 34,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 11dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 5,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 11.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 12dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 12.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 13dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 13.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 14dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 5,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 14.5dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 15dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 5,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 15.5dBm */
        {
            .power     = 34,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 16dBm */
        {
            .power     = 36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 5,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 16.5dBm */
        {
            .power     = 36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 17dBm */
        {
            .power     = 38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 5,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 17.5dBm */
        {
            .power     = 38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 18dBm */
        {
            .power     = 38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 6,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 18.5dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 19dBm */
        {
            .power     = 40,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 6,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 19.5dBm */
        {
            .power     = 42,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 20dBm */
        {
            .power     = 42,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 20.5dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 21dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 21.5dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 22dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        }
    };

const smtc_shield_lr20xx_pa_pwr_cfg_t
    smtc_shield_lr2021_common_pa_lf_pwr_cfg[2 * ( SMTC_SHIELD_LR20XX_MAX_PWR - SMTC_SHIELD_LR20XX_MIN_PWR ) + 1] = {

        /* Expected output power = -17dBm */
        {
            .power     = -18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -16.5dBm */
        {
            .power     = -18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -16dBm */
        {
            .power     = -16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -15.5dBm */
        {
            .power     = -16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -15dBm */
        {
            .power     = -16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -14.5dBm */
        {
            .power     = -16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -14dBm */
        {
            .power     = -10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -13.5dBm */
        {
            .power     = -14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -13dBm */
        {
            .power     = -8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -12.5dBm */
        {
            .power     = -16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -12dBm */
        {
            .power     = -6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -11.5dBm */
        {
            .power     = -18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -11dBm */
        {
            .power     = -10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -10.5dBm */
        {
            .power     = -12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -10dBm */
        {
            .power     = -8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -9.5dBm */
        {
            .power     = -10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -9dBm */
        {
            .power     = -6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -8.5dBm */
        {
            .power     = -8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -8dBm */
        {
            .power     = 0,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -7.5dBm */
        {
            .power     = -6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -7dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -6.5dBm */
        {
            .power     = -2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -6dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -5.5dBm */
        {
            .power     = 8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -5dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -4.5dBm */
        {
            .power     = 10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -4dBm */
        {
            .power     = 8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -3.5dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -3dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -2.5dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -2dBm */
        {
            .power     = 6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -1.5dBm */
        {
            .power     = 14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -1dBm */
        {
            .power     = 12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = -0.5dBm */
        {
            .power     = 12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 0dBm */
        {
            .power     = 16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 0.5dBm */
        {
            .power     = 14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 1dBm */
        {
            .power     = 18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 1.5dBm */
        {
            .power     = 16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 2dBm */
        {
            .power     = 18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 2.5dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 3dBm */
        {
            .power     = 20,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 3.5dBm */
        {
            .power     = 26,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 4dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 4.5dBm */
        {
            .power     = 20,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 5dBm */
        {
            .power     = 16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 5.5dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 6dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 6.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 7dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 0,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 7.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 8dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 0,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 8.5dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 2,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 9dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 9.5dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 10dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 1,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 10.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 3,
                           .pa_lf_slices     = 1,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 11dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 11.5dBm */
        {
            .power     = 28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 12dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 12.5dBm */
        {
            .power     = 30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 13dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 13.5dBm */
        {
            .power     = 32,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 14dBm */
        {
            .power     = 34,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 14.5dBm */
        {
            .power     = 34,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 15dBm */
        {
            .power     = 36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 2,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 15.5dBm */
        {
            .power     = 36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 16dBm */
        {
            .power     = 34,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 3,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 16.5dBm */
        {
            .power     = 36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 17dBm */
        {
            .power     = 38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 17.5dBm */
        {
            .power     = 38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 18dBm */
        {
            .power     = 38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 18.5dBm */
        {
            .power     = 40,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 4,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 19dBm */
        {
            .power     = 40,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 19.5dBm */
        {
            .power     = 42,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 4,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 20dBm */
        {
            .power     = 42,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 6,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 20.5dBm */
        {
            .power     = 42,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 21dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 5,
                           .pa_lf_slices     = 6,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 21.5dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        },

        /* Expected output power = 22dBm */
        {
            .power     = 44,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 7,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 0 },
        }
    };

const smtc_shield_lr20xx_pa_pwr_cfg_t
    smtc_shield_lr2021_common_pa_hf_pwr_cfg[2 * ( SMTC_SHIELD_LR20XX_MAX_PWR_HF - SMTC_SHIELD_LR20XX_MIN_PWR_HF ) +
                                            1] = {
        /* Expected output power = -17dBm */
        {
            .power     = -39,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = -16.5dBm */
        {
            .power     = -38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 27 },
        },

        /* Expected output power = -16dBm */
        {
            .power     = -38,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 18 },
        },

        /* Expected output power = -15.5dBm */
        {
            .power     = -36,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 19 },
        },

        /* Expected output power = -15dBm */
        {
            .power     = -35,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 19 },
        },

        /* Expected output power = -14.5dBm */
        {
            .power     = -33,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 19 },
        },

        /* Expected output power = -14dBm */
        {
            .power     = -31,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 24 },
        },

        /* Expected output power = -13.5dBm */
        {
            .power     = -30,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 23 },
        },

        /* Expected output power = -13dBm */
        {
            .power     = -29,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 19 },
        },

        /* Expected output power = -12.5dBm */
        {
            .power     = -28,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 18 },
        },

        /* Expected output power = -12dBm */
        {
            .power     = -26,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 24 },
        },

        /* Expected output power = -11.5dBm */
        {
            .power     = -25,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 20 },
        },

        /* Expected output power = -11dBm */
        {
            .power     = -24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 17 },
        },

        /* Expected output power = -10.5dBm */
        {
            .power     = -22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 24 },
        },

        /* Expected output power = -10dBm */
        {
            .power     = -21,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 23 },
        },

        /* Expected output power = -9.5dBm */
        {
            .power     = -20,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 20 },
        },

        /* Expected output power = -9dBm */
        {
            .power     = -19,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 18 },
        },

        /* Expected output power = -8.5dBm */
        {
            .power     = -18,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = -8dBm */
        {
            .power     = -17,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = -7.5dBm */
        {
            .power     = -15,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 23 },
        },

        /* Expected output power = -7dBm */
        {
            .power     = -14,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 21 },
        },

        /* Expected output power = -6.5dBm */
        {
            .power     = -13,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 21 },
        },

        /* Expected output power = -6dBm */
        {
            .power     = -12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 18 },
        },

        /* Expected output power = -5.5dBm */
        {
            .power     = -11,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 17 },
        },

        /* Expected output power = -5dBm */
        {
            .power     = -10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = -4.5dBm */
        {
            .power     = -9,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = -4dBm */
        {
            .power     = -8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = -3.5dBm */
        {
            .power     = -6,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 24 },
        },

        /* Expected output power = -3dBm */
        {
            .power     = -5,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 24 },
        },

        /* Expected output power = -2.5dBm */
        {
            .power     = -4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 22 },
        },

        /* Expected output power = -2dBm */
        {
            .power     = -2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 27 },
        },

        /* Expected output power = -1.5dBm */
        {
            .power     = 0,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 29 },
        },

        /* Expected output power = -1dBm */
        {
            .power     = 1,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 29 },
        },

        /* Expected output power = -0.5dBm */
        {
            .power     = 3,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 0dBm */
        {
            .power     = 2,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 26 },
        },

        /* Expected output power = 0.5dBm */
        {
            .power     = 3,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 25 },
        },

        /* Expected output power = 1dBm */
        {
            .power     = 4,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 25 },
        },

        /* Expected output power = 1.5dBm */
        {
            .power     = 5,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 25 },
        },

        /* Expected output power = 2dBm */
        {
            .power     = 7,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = 2.5dBm */
        {
            .power     = 8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = 3dBm */
        {
            .power     = 8,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 20 },
        },

        /* Expected output power = 3.5dBm */
        {
            .power     = 10,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = 4dBm */
        {
            .power     = 11,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = 4.5dBm */
        {
            .power     = 12,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = 5dBm */
        {
            .power     = 13,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 27 },
        },

        /* Expected output power = 5.5dBm */
        {
            .power     = 13,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 25 },
        },

        /* Expected output power = 6dBm */
        {
            .power     = 16,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 6.5dBm */
        {
            .power     = 15,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 25 },
        },

        /* Expected output power = 7dBm */
        {
            .power     = 17,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 28 },
        },

        /* Expected output power = 7.5dBm */
        {
            .power     = 19,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 8dBm */
        {
            .power     = 17,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = 8.5dBm */
        {
            .power     = 21,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 9dBm */
        {
            .power     = 22,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 9.5dBm */
        {
            .power     = 23,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 10dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 30 },
        },

        /* Expected output power = 10.5dBm */
        {
            .power     = 23,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 26 },
        },

        /* Expected output power = 11dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 26 },
        },

        /* Expected output power = 11.5dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

        /* Expected output power = 12dBm */
        {
            .power     = 24,
            .pa_config = { .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
                           .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
                           .pa_lf_duty_cycle = 6,
                           .pa_lf_slices     = 7,
                           .pa_hf_duty_cycle = 16 },
        },

    };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const smtc_shield_lr20xx_pa_pwr_cfg_t* smtc_shield_lr20xx_common_get_pa_pwr_cfg(
    const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_half_dbm )
{
    if( rf_freq_in_hz < SMTC_SHIELD_LR20XX_LF_HF_FREQ_THRESHOLD )
    {
        // SubGHz frequencies

        // - Below 600MHz
        if( ( SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MIN <= rf_freq_in_hz ) &&
            ( rf_freq_in_hz <= SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MED ) )
        {
            int8_t index = expected_output_pwr_in_half_dbm - ( 2 * SMTC_SHIELD_LR20XX_MIN_PWR );
            if( ( index < 0 ) || ( index > 2 * ( SMTC_SHIELD_LR20XX_MAX_PWR - SMTC_SHIELD_LR20XX_MIN_PWR ) ) )
            {
                return &smtc_shield_lr20xx_common_pa_lf_default_pwr_cfg;
            }
            else
            {
                return &smtc_shield_lr2021_common_pa_low_lf_pwr_cfg[index];
            }
        }
        // - From 600MHz to 960MHz
        else
        {
            if( SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MED <= rf_freq_in_hz &&
                rf_freq_in_hz <= SMTC_SHIELD_LR20XX_SUBGHZ_FREQ_MAX )
            {
                int8_t index = expected_output_pwr_in_half_dbm - ( 2 * SMTC_SHIELD_LR20XX_MIN_PWR );
                if( ( index < 0 ) || ( index > 2 * ( SMTC_SHIELD_LR20XX_MAX_PWR - SMTC_SHIELD_LR20XX_MIN_PWR ) ) )
                {
                    return &smtc_shield_lr20xx_common_pa_lf_default_pwr_cfg;
                }
                else
                {
                    return &smtc_shield_lr2021_common_pa_lf_pwr_cfg[index];
                }
            }
        }
    }
    else
    {
        if( ( ( SMTC_SHIELD_LR20XX_2GHZ_FREQ_MIN <= rf_freq_in_hz ) &&
              ( rf_freq_in_hz <= SMTC_SHIELD_LR20XX_2GHZ_FREQ_MAX ) ) ||
            ( ( SMTC_SHIELD_LR20XX_2_4GHZ_FREQ_MIN <= rf_freq_in_hz ) &&
              ( rf_freq_in_hz <= SMTC_SHIELD_LR20XX_2_4GHZ_FREQ_MAX ) ) )

        {
            int8_t index = expected_output_pwr_in_half_dbm - ( 2 * SMTC_SHIELD_LR20XX_MIN_PWR_HF );
            if( ( index < 0 ) || ( index > 2 * ( SMTC_SHIELD_LR20XX_MAX_PWR_HF - SMTC_SHIELD_LR20XX_MIN_PWR_HF ) ) )
            {
                return &smtc_shield_lr20xx_common_pa_hf_default_pwr_cfg;
            }
            else
            {
                return &smtc_shield_lr2021_common_pa_hf_pwr_cfg[index];
            }
        }
    }
    return NULL;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
