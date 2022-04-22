/**
 * @file      ralf_defs.h
 *
 * @brief     Radio abstraction layer
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

#ifndef RALF_DEFS_H__
#define RALF_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "ral_defs.h"

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

typedef struct ralf_params_gfsk_s
{
    ral_gfsk_mod_params_t mod_params;
    ral_gfsk_pkt_params_t pkt_params;
    const uint8_t*        sync_word;
    uint32_t              rf_freq_in_hz;
    uint16_t              crc_seed;
    uint16_t              crc_polynomial;
    uint16_t              whitening_seed;
    bool                  dc_free_is_on;
    int8_t                output_pwr_in_dbm;
} ralf_params_gfsk_t;

typedef struct ralf_params_lora_s
{
    ral_lora_mod_params_t mod_params;
    ral_lora_pkt_params_t pkt_params;
    uint32_t              rf_freq_in_hz;
    int8_t                output_pwr_in_dbm;
    uint8_t               sync_word;
    uint8_t               symb_nb_timeout;  //! Rx only parameters
} ralf_params_lora_t;

typedef struct ralf_params_flrc_s
{
    ral_flrc_mod_params_t mod_params;
    ral_flrc_pkt_params_t pkt_params;
    const uint8_t*        sync_word;
    uint32_t              rf_freq_in_hz;
    uint32_t              crc_seed;
    int8_t                output_pwr_in_dbm;
} ralf_params_flrc_t;

typedef struct ralf_params_lr_fhss_s
{
    ral_lr_fhss_params_t ral_lr_fhss_params;
    int8_t               output_pwr_in_dbm;
    uint16_t             hop_sequence_id;
} ralf_params_lr_fhss_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // RALF_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */
