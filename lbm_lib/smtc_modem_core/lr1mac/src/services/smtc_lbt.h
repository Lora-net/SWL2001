/*!
 * \file      smtc_lbt.h
 *
 * \brief     Listen Before Talk Object
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

#ifndef __SMTC_LBT_H__
#define __SMTC_LBT_H__

#include <stdint.h>
#include <stdbool.h>
#include "radio_planner.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * API definitions
 * ============================================================================
 */
#define LAP_OF_TIME_TO_GET_A_RSSI_VALID 2  // duration to stabilize the radio after rx cmd in ms
typedef struct smtc_lbt_s
{
    radio_planner_t* rp;
    uint8_t          lbt_id4rp;
    uint32_t         listen_duration_ms;
    int16_t          threshold;
    uint32_t         bw_hz;
    bool             is_at_time;
    void ( *free_callback )( void* );
    void* free_context;
    void ( *busy_callback )( void* );
    void* busy_context;
    void ( *abort_callback )( void* );
    void*    abort_context;
    int16_t  rssi_inst;
    int32_t  rssi_accu;
    uint32_t rssi_nb_of_meas;
    bool     enabled;
    /* data */
} smtc_lbt_t;

/**
 * @brief smtc_lbt_init function to init lbt object
 *
 * @param lbt_obj pointer to lbt_obj itself
 * @param rp      pointer to associated radio planner
 * @param lbt_id_rp task id inside the radio planner
 * @param free_callback callback attached if channel is free
 * @param free_context  context link to the previous callback
 * @param busy_callback callback attached if channel is busy
 * @param busy_context  context link to the previous callback
 * @param abort_callback callback attached if channel is aborted
 * @param abort_context  context link to the previous callback
 */
void smtc_lbt_init( smtc_lbt_t* lbt_obj, radio_planner_t* rp, uint8_t lbt_id_rp,
                    void ( *free_callback )( void* free_context ), void*   free_context,
                    void ( *busy_callback )( void* busy_context ), void*   busy_context,
                    void ( *abort_callback )( void* abort_context ), void* abort_context );

/**
 * @brief Set the LBT parameters
 *
 * @param [in] lbt_obj pointer to lbt_obj itself
 * @param [in] listen_duration_ms duration of the listen task
 * @param [in] threshold_dbm threshold in dbm to decide if the channel is free or busy
 * @param [in] bw_hz bandwith in hertz to listen a channel
 */
void smtc_lbt_set_parameters( smtc_lbt_t* lbt_obj, uint32_t listen_duration_ms, int16_t threshold_dbm, uint32_t bw_hz );

/**
 * @brief Get the configured lbt parameters
 *
 * @param [in]  lbt_obj pointer to lbt_obj itself
 * @param [out] listen_duration_ms duration of the listen task
 * @param [out] threshold_dbm threshold in dbm
 * @param [out] bw_hz  bandwith in hertz
 */
void smtc_lbt_get_parameters( smtc_lbt_t* lbt_obj, uint32_t* listen_duration_ms, int16_t* threshold_dbm,
                              uint32_t* bw_hz );

/**
 * @brief Enable/Disable LBT service
 *
 * @param [in] lbt_obj pointer to lbt_obj itself
 * @param [in] enable true to enable lbt service, false to disable it
 */
void smtc_lbt_set_state( smtc_lbt_t* lbt_obj, bool enable );

/**
 * @brief Return the current enabled state of the lbt service
 *
 * @param [in] lbt_obj pointer to lbt_obj itself
 * @return true if service is currently enabled
 * @return false if service is currently disabled
 */
bool smtc_lbt_get_state( smtc_lbt_t* lbt_obj );

/**
 * @brief smtc_lbt_listen_channel this function is called each time who want to listen the channel
 *
 * @param lbt_obj pointer to lbt_obj itself
 * @param freq    targeted listen frequency in hertz
 * @param is_at_time is a listen at time or asap
 * @param target_time_ms time to start the listening
 * @param tx_duration_ms duration of the transmission if channel is free ( allow to book the radio planer )
 */
void smtc_lbt_listen_channel( smtc_lbt_t* lbt_obj, uint32_t freq, bool is_at_time, uint32_t target_time_ms,
                              uint32_t tx_duration_ms );

/**
 * @brief smtc_lbt_rp_callback this function is call by the radio planer when lbt task is finished
 *
 * @param lbt_obj  pointer to lbt_obj itself
 */
void smtc_lbt_rp_callback( smtc_lbt_t* lbt_obj );

/**
 * @brief smtc_lbt_launch_callback_for_rp this function is call by the radio planer when it is time to launch listen
 * task
 *
 * @param rp_void pointer to lbt_obj itself
 */
void smtc_lbt_launch_callback_for_rp( void* rp_void );
#ifdef __cplusplus
}
#endif

#endif  //  __SMTC_LBT_H__
