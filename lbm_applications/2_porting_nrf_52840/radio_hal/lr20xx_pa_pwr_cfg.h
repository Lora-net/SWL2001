/*!
 * @file      lr20xx_pa_pwr_cfg.h
 *
 * @brief     lr20xx power amplifier configuration.
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
#ifndef LR20XX_PA_PWR_CFG_H
#define LR20XX_PA_PWR_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

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

#define LR20XX_PA_LF_CFG_TABLE                   \
    {                                            \
        {                                        \
            /* Expected output power = -10dBm */ \
            .half_power    = -18,                \
            .pa_duty_cycle = 3,                  \
            .pa_lf_slices  = 6,                  \
        },                                       \
        {                                        \
            /* Expected output power = -9dBm */  \
            .half_power    = -13,                \
            .pa_duty_cycle = 2,                  \
            .pa_lf_slices  = 5,                  \
        },                                       \
        {                                        \
            /* Expected output power = -8dBm */  \
            .half_power    = -13,                \
            .pa_duty_cycle = 6,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = -7dBm */  \
            .half_power    = -6,                 \
            .pa_duty_cycle = 6,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = -6dBm */  \
            .half_power    = 4,                  \
            .pa_duty_cycle = 1,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = -5dBm */  \
            .half_power    = 4,                  \
            .pa_duty_cycle = 2,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = -4dBm */  \
            .half_power    = 2,                  \
            .pa_duty_cycle = 1,                  \
            .pa_lf_slices  = 3,                  \
        },                                       \
        {                                        \
            /* Expected output power = -3dBm */  \
            .half_power    = 14,                 \
            .pa_duty_cycle = 0,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = -2dBm */  \
            .half_power    = 9,                  \
            .pa_duty_cycle = 0,                  \
            .pa_lf_slices  = 3,                  \
        },                                       \
        {                                        \
            /* Expected output power = -1dBm */  \
            .half_power    = 11,                 \
            .pa_duty_cycle = 3,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 0dBm */   \
            .half_power    = 16,                 \
            .pa_duty_cycle = 1,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 1dBm */   \
            .half_power    = 11,                 \
            .pa_duty_cycle = 7,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 2dBm */   \
            .half_power    = 18,                 \
            .pa_duty_cycle = 2,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 3dBm */   \
            .half_power    = 16,                 \
            .pa_duty_cycle = 5,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 4dBm */   \
            .half_power    = 17,                 \
            .pa_duty_cycle = 7,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 5dBm */   \
            .half_power    = 21,                 \
            .pa_duty_cycle = 1,                  \
            .pa_lf_slices  = 2,                  \
        },                                       \
        {                                        \
            /* Expected output power = 6dBm */   \
            .half_power    = 25,                 \
            .pa_duty_cycle = 3,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 7dBm */   \
            .half_power    = 32,                 \
            .pa_duty_cycle = 0,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = 8dBm */   \
            .half_power    = 32,                 \
            .pa_duty_cycle = 2,                  \
            .pa_lf_slices  = 0,                  \
        },                                       \
        {                                        \
            /* Expected output power = 9dBm */   \
            .half_power    = 27,                 \
            .pa_duty_cycle = 3,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = 10dBm */  \
            .half_power    = 32,                 \
            .pa_duty_cycle = 2,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = 11dBm */  \
            .half_power    = 28,                 \
            .pa_duty_cycle = 5,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = 12dBm */  \
            .half_power    = 30,                 \
            .pa_duty_cycle = 5,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = 13dBm */  \
            .half_power    = 34,                 \
            .pa_duty_cycle = 4,                  \
            .pa_lf_slices  = 1,                  \
        },                                       \
        {                                        \
            /* Expected output power = 14dBm */  \
            .half_power    = 31,                 \
            .pa_duty_cycle = 5,                  \
            .pa_lf_slices  = 4,                  \
        },                                       \
        {                                        \
            /* Expected output power = 15dBm */  \
            .half_power    = 34,                 \
            .pa_duty_cycle = 4,                  \
            .pa_lf_slices  = 4,                  \
        },                                       \
        {                                        \
            /* Expected output power = 16dBm */  \
            .half_power    = 34,                 \
            .pa_duty_cycle = 5,                  \
            .pa_lf_slices  = 6,                  \
        },                                       \
        {                                        \
            /* Expected output power = 17dBm */  \
            .half_power    = 39,                 \
            .pa_duty_cycle = 3,                  \
            .pa_lf_slices  = 5,                  \
        },                                       \
        {                                        \
            /* Expected output power = 18dBm */  \
            .half_power    = 37,                 \
            .pa_duty_cycle = 6,                  \
            .pa_lf_slices  = 6,                  \
        },                                       \
        {                                        \
            /* Expected output power = 19dBm */  \
            .half_power    = 40,                 \
            .pa_duty_cycle = 5,                  \
            .pa_lf_slices  = 5,                  \
        },                                       \
        {                                        \
            /* Expected output power = 20dBm */  \
            .half_power    = 41,                 \
            .pa_duty_cycle = 7,                  \
            .pa_lf_slices  = 4,                  \
        },                                       \
        {                                        \
            /* Expected output power = 21dBm */  \
            .half_power    = 43,                 \
            .pa_duty_cycle = 7,                  \
            .pa_lf_slices  = 4,                  \
        },                                       \
        {                                        \
            /* Expected output power = 22dBm */  \
            .half_power    = 44,                 \
            .pa_duty_cycle = 7,                  \
            .pa_lf_slices  = 7,                  \
        },                                       \
    }

// TODO .half_power and .pa_duty_cycle must be updated
#define LR20XX_PA_HF_CFG_TABLE                   \
    {                                            \
        {                                        \
            /* Expected output power = -17dBm */ \
            .half_power    = -39,                \
            .pa_duty_cycle = 29,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -16dBm */ \
            .half_power    = -39,                \
            .pa_duty_cycle = 16,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -15dBm */ \
            .half_power    = -35,                \
            .pa_duty_cycle = 19,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -14dBm */ \
            .half_power    = -32,                \
            .pa_duty_cycle = 19,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -13dBm */ \
            .half_power    = -29,                \
            .pa_duty_cycle = 19,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -12dBm */ \
            .half_power    = -27,                \
            .pa_duty_cycle = 16,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -11dBm */ \
            .half_power    = -24,                \
            .pa_duty_cycle = 17,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -10dBm */ \
            .half_power    = -22,                \
            .pa_duty_cycle = 16,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -9dBm */  \
            .half_power    = -19,                \
            .pa_duty_cycle = 18,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -8dBm */  \
            .half_power    = -17,                \
            .pa_duty_cycle = 16,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -7dBm */  \
            .half_power    = -14,                \
            .pa_duty_cycle = 21,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -6dBm */  \
            .half_power    = -12,                \
            .pa_duty_cycle = 18,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -5dBm */  \
            .half_power    = -7,                 \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -4dBm */  \
            .half_power    = -8,                 \
            .pa_duty_cycle = 16,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -3dBm */  \
            .half_power    = -5,                 \
            .pa_duty_cycle = 24,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -2dBm */  \
            .half_power    = -2,                 \
            .pa_duty_cycle = 27,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = -1dBm */  \
            .half_power    = 1,                  \
            .pa_duty_cycle = 29,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 0dBm */   \
            .half_power    = 4,                  \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 1dBm */   \
            .half_power    = 6,                  \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 2dBm */   \
            .half_power    = 7,                  \
            .pa_duty_cycle = 28,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 3dBm */   \
            .half_power    = 8,                  \
            .pa_duty_cycle = 25,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 4dBm */   \
            .half_power    = 10,                 \
            .pa_duty_cycle = 25,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 5dBm */   \
            .half_power    = 15,                 \
            .pa_duty_cycle = 31,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 6dBm */   \
            .half_power    = 16,                 \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 7dBm */   \
            .half_power    = 18,                 \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 8dBm */   \
            .half_power    = 21,                 \
            .pa_duty_cycle = 31,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 9dBm */   \
            .half_power    = 22,                 \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 10dBm */  \
            .half_power    = 24,                 \
            .pa_duty_cycle = 30,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 11dBm */  \
            .half_power    = 24,                 \
            .pa_duty_cycle = 26,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
        {                                        \
            /* Expected output power = 12dBm */  \
            .half_power    = 24,                 \
            .pa_duty_cycle = 16,                 \
            .pa_lf_slices  = 7,                  \
        },                                       \
                                                 \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_PA_PWR_CFG_H

/* --- EOF ------------------------------------------------------------------ */
