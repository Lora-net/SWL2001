/*!
 * \file      smtc_lora_cad_bt.h
 *
 * \brief     LoRa Channel Activities Detection Before Talk Object (CSMA)
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

#ifndef __SMTC_LORA_CAD_BT_H__
#define __SMTC_LORA_CAD_BT_H__

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "radio_planner.h"
#ifdef __cplusplus
extern "C" {
#endif

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
typedef enum smtc_lora_cad_status_e
{
    SMTC_LORA_CAD_OK = 0,
    SMTC_LORA_CAD_ERROR
} smtc_lora_cad_status_t;

typedef enum smtc_lora_cad_state_e
{
    SMTC_LORA_CAD_DIFS = 0,
    SMTC_LORA_CAD_BO,
    SMTC_LORA_CAD_NB_STATE
} smtc_lora_cad_state_t;

typedef struct smtc_lora_cad_bt_s
{
    radio_planner_t* rp;
    uint8_t          cad_id4rp;
    bool             enabled;
    bool             is_at_time;

    /* Callback */
    void ( *ch_free_callback )( void* );
    void* ch_free_context;
    void ( *ch_busy_callback_update_channel )( void* );
    void* ch_busy_context_update_channel;
    void ( *ch_free_callback_on_back_off )( void* );
    void* ch_free_context_on_back_off;
    void ( *abort_callback )( void* );
    void* abort_context;

    /* data */
    smtc_lora_cad_state_t cad_state;               // State machine SMTC_LORA_CAD_DIFS or SMTC_LORA_CAD_BO
    smtc_lora_cad_state_t cad_state_prev;          // previous state
    bool                  is_cad_running;          // CAD is running on radio
    bool                  bo_enabled;              // is back off enabled
    uint8_t               nb_bo_max_conf;          // number of back off configured by the user
    uint8_t               nb_bo;                   // number of back off
    uint8_t               max_ch_change_cnt_conf;  // number of try to cad configured by the user
    uint8_t               max_ch_change_cnt;       // number of try to cad
    int8_t                detect_peak_offset;

} smtc_lora_cad_bt_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief smtc_lora_cad_bt_init function to init lbt object
 *
 * @param cad_obj pointer to cad_obj itself
 * @param rp      pointer to associated radio planner
 * @param cad_id_rp task id inside the radio planner
 * @param ch_free_callback callback attached if channel is free
 * @param ch_free_context  context link to the previous callback
 * @param ch_busy_callback_update_channel callback attached if channel is busy and must be changed
 * @param ch_busy_context_update_channel  context link to the previous callback
 * @param ch_free_callback_on_back_off callback attached if channel Back Off is enabled, channel is kept
 * @param ch_free_context_on_back_off  context link to the previous callback
 * @param abort_callback callback attached if channel is aborted
 * @param abort_context  context link to the previous callback
 */
void smtc_lora_cad_bt_init( smtc_lora_cad_bt_t* cad_obj, radio_planner_t* rp, uint8_t cad_id_rp,
                            void ( *ch_free_callback )( void* ch_free_context ), void* ch_free_context,
                            void ( *ch_busy_callback_update_channel )( void* ch_busy_context_update_channel ),
                            void* ch_busy_context_update_channel,
                            void ( *ch_free_callback_on_back_off )( void* ch_free_context_on_back_off ),
                            void* ch_free_context_on_back_off, void ( *abort_callback )( void* abort_context ),
                            void* abort_context );

/**
 * @brief Set the LoRa CAD parameters
 *
 * @param [in] cad_obj pointer to cad_obj itself
 * @param [in] max_ch_change Number of channel changed before send the packet in ALOHA mode
 * @param [in] bo_enabled Enable the back off to run multiple little CAD before transmit
 * @param [in] nb_bo_max Set the number max of multiple little CAD
 * @return smtc_lora_cad_status_t
 */
smtc_lora_cad_status_t smtc_lora_cad_bt_set_parameters( smtc_lora_cad_bt_t* cad_obj, uint8_t max_ch_change,
                                                        bool bo_enabled, uint8_t nb_bo_max );

/**
 * @brief Get the configured LoRa CAD parameters
 *
 * @param [in]  cad_obj pointer to cad_obj itself
 * @param [out] max_ch_change Number of channel changed before send the packet in ALOHA mode
 * @param [out] bo_enabled Is back off enable to run multiple little CAD before transmit
 * @param [out] nb_bo_max Get the number max of multiple little CAD
 */
void smtc_lora_cad_bt_get_parameters( smtc_lora_cad_bt_t* cad_obj, uint8_t* max_ch_change, bool* bo_enabled,
                                      uint8_t* nb_bo_max );

/**
 * @brief Enable/Disable CAD before talk service
 *
 * @param [in] cad_obj pointer to cad_obj itself
 * @param [in] enable true to enable cad before talk service, false to disable it
 */
void smtc_lora_cad_bt_set_state( smtc_lora_cad_bt_t* cad_obj, bool enable );

/**
 * @brief Return the current enabled state of the CAD before talk service
 *
 * @param [in] cad_obj pointer to cad_obj itself
 * @return true if service is currently enabled
 * @return false if service is currently disabled
 */
bool smtc_lora_cad_bt_get_state( smtc_lora_cad_bt_t* cad_obj );

/**
 * @brief smtc_lora_cad_bt_listen_channel this function is called each time who want to listen LoRa activity on a
 * channel
 *
 * @param cad_obj               pointer to cad_obj itself
 * @param freq_hz               targeted listen frequency in hertz
 * @param sf                    targeted listen spreading factor (ral_lora_sf_t)
 * @param bandwidth             targeted listen bandwidth (ral_lora_bw_t)
 * @param is_at_time            is a listen at time or asap
 * @param target_time_ms        time to start the listening
 * @param tx_duration_ms        duration of the transmission if channel is free ( allow to book the radio planer )
 * @param nb_available_channel  the number of available channels in region for max_change
 * @param invert_iq_is_on       target iq invert
 */
void smtc_lora_cad_bt_listen_channel( smtc_lora_cad_bt_t* cad_obj, uint32_t freq_hz, ral_lora_sf_t sf,
                                      ral_lora_bw_t bandwidth, bool is_at_time, uint32_t target_time_ms,
                                      uint32_t tx_duration_ms, uint8_t nb_available_channel, bool invert_iq_is_on );

/**
 * @brief return the cad obj pointer with stack id as parameter
 * task
 *
 * @param stack_id  stack_id
 * @return smtc_lora_cad_bt_t object pointer
 */
smtc_lora_cad_bt_t* smtc_cad_get_obj( uint8_t stack_id );

#ifdef __cplusplus
}
#endif

#endif  //  __SMTC_LORA_CAD_BT_H__

/* --- EOF ------------------------------------------------------------------ */
