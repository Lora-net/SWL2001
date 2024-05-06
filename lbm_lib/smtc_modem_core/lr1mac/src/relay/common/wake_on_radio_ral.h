/*!
 * \file    wake_on_radio_ral.h
 *
 * \brief   WOR and WOR ACK radio ral function
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
#ifndef WAKE_ON_RADIO_RAL_H
#define WAKE_ON_RADIO_RAL_H

#ifdef _cplusplus
extern "C" {
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */
#include <stdint.h>
#include <stdbool.h>
#include "radio_planner.h"
#include "radio_planner_types.h"
#include "ral_defs.h"
#include "smtc_real_defs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS ----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES -----------------------------------------------------------
 */

typedef struct wor_tx_param_s
{
    uint32_t      start_time_ms;
    uint32_t      freq_hz;
    uint8_t*      payload;
    uint16_t      preamble_len_symb;
    uint8_t       payload_len;
    ral_lora_bw_t bw;
    ral_lora_cr_t cr;
    uint8_t       sf;
    int8_t        output_pwr_in_dbm;
    uint8_t       sync_word;
    bool          wor_at_time;
} wor_tx_param_t;

typedef struct wor_ack_rx_param_s
{
    uint32_t      start_time_ms;
    uint32_t      toa;
    uint32_t      freq_hz;
    uint8_t*      payload;
    uint8_t       payload_len;
    ral_lora_bw_t bw;
    ral_lora_cr_t cr;
    uint8_t       sf;
    uint8_t       sync_word;
    uint16_t      preamble_len_in_symb;
} wor_ack_rx_param_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC VARIABLES -------------------------------------------------------------
 */

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DECLARATION -------------------------------------------------
 */

/**
 * @brief Schedule a radio planner task to send a WOR frame
 *
 * @param[in]   hook_id     relay tx hook id
 * @param[in]   rp          radio planner pointer
 * @param[in]   wor_param   WOR param to send WOR frame
 * @return true     WOR successfully loaded in radio planner
 * @return false    Failed
 */
bool wor_schedule_tx_wor( uint8_t hook_id, radio_planner_t* rp, wor_tx_param_t* wor_param );

/**
 * @brief Fill rp_radio_params_t struct to send a WOR ACK
 *
 * @param[in]   real        Regional Abstraction Layer object
 * @param[in]   dr          Datarate of the WOR ACK
 * @param[in]   freq_hz     Frequency of the WOR ACK
 * @param[in]   payload_len Len of the WOR ACK
 * @param[out]  param       Radio parameter structure with WOR ACK
 */
void wor_ral_init_tx_ack( smtc_real_t* real, uint8_t dr, uint32_t freq_hz, uint8_t payload_len,
                          rp_radio_params_t* param );

/**
 * @brief Fill rp_radio_params_t struct to receive a WOR ACK
 *
 * @param[in]   hook_id       Relay TX hook id
 * @param[in]   rp            Radio planner pointer
 * @param[in]   wor_ack_param WOR ACK param to open RX windows
 */
bool wor_schedule_rx_wor_ack( uint8_t hook_id, radio_planner_t* rp, wor_ack_rx_param_t* wor_ack_param );

#ifdef _cplusplus
}
#endif
#endif  // WAKE_ON_RADIO_RAL_H
