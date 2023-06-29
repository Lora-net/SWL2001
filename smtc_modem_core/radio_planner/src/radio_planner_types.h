/*!
 * \file      radio_planner_types.h
 *
 * \brief     Radio planner types definition
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
#ifndef __RADIO_PLANNER_TYPES_H__
#define __RADIO_PLANNER_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "radio_planner_hook_id_defs.h"

// Include radio abstraction layer
#include "ralf.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

// clang-format off

/*
 * Maximum number of objects that can be attached to the scheduler
 */
#define RP_NB_HOOKS                                 RP_HOOK_ID_MAX

#define RP_NB_USER_HOOK                             3



/*!
 *
 */
#define RP_NO_MORE_TASK                             0

/*!
 *
 */
#define RP_SOMETHING_TO_DO                          1

/*!
 * for 8 ms : 5MS FOR WAKE UP (2MS) + CONFIG TIMER (3MS FIX !) + 3 ms interrupt routine
 */
#ifndef RP_MARGIN_DELAY
#define RP_MARGIN_DELAY                             8
#endif



/*!
 *
 */
#define RP_TASK_ASAP_TO_SCHEDULE_TRIG_TIME          120000  // for 120 seconds

/*!
 *
 */
#define RP_TASK_RE_SCHEDULE_OFFSET_TIME             2000  // for 2 seconds

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 *
 */
typedef struct rp_radio_params_s
{
    ral_pkt_type_t pkt_type;
    struct
    {
        union
        {
            ralf_params_gfsk_t    gfsk;
            ralf_params_lora_t    lora;
            ralf_params_lr_fhss_t lr_fhss;
        };
    } tx;
    struct
    {
        union
        {
            ralf_params_gfsk_t gfsk;
            ralf_params_lora_t lora;
        };
        uint32_t timeout_in_ms;
        union
        {
            ral_gfsk_rx_pkt_status_t gfsk_pkt_status;
            ral_lora_rx_pkt_status_t lora_pkt_status;
        };
    } rx;
    int16_t lbt_threshold;
    uint8_t lr_fhss_state[RAL_LR_FHSS_STATE_MAXSIZE];
} rp_radio_params_t;

/*!
 *
 */
typedef enum rp_task_types_e
{
    RP_TASK_TYPE_RX_LORA,
    RP_TASK_TYPE_RX_FSK,
    RP_TASK_TYPE_TX_LORA,
    RP_TASK_TYPE_TX_FSK,
    RP_TASK_TYPE_TX_LR_FHSS,
    RP_TASK_TYPE_CAD,
    RP_TASK_TYPE_CAD_TO_TX,
    RP_TASK_TYPE_CAD_TO_RX,
    RP_TASK_TYPE_GNSS_SNIFF,
    RP_TASK_TYPE_WIFI_SNIFF,
    RP_TASK_TYPE_GNSS_RSSI,
    RP_TASK_TYPE_WIFI_RSSI,
    RP_TASK_TYPE_LBT,
    RP_TASK_TYPE_USER,
    RP_TASK_TYPE_NONE,
} rp_task_types_t;

/*!
 *
 */
typedef enum rp_task_states_e
{
    RP_TASK_STATE_SCHEDULE,
    RP_TASK_STATE_ASAP,
    RP_TASK_STATE_RUNNING,
    RP_TASK_STATE_ABORTED,
    RP_TASK_STATE_FINISHED,
} rp_task_states_t;

/*!
 *
 */
typedef enum rp_status_e
{
    RP_STATUS_RX_CRC_ERROR,
    RP_STATUS_CAD_POSITIVE,
    RP_STATUS_CAD_NEGATIVE,
    RP_STATUS_TX_DONE,
    RP_STATUS_RX_PACKET,
    RP_STATUS_RX_TIMEOUT,
    RP_STATUS_LBT_FREE_CHANNEL,
    RP_STATUS_LBT_BUSY_CHANNEL,
    RP_STATUS_WIFI_SCAN_DONE,
    RP_STATUS_GNSS_SCAN_DONE,
    RP_STATUS_TASK_ABORTED,
    RP_STATUS_TASK_INIT,
    RP_STATUS_LR_FHSS_HOP,
} rp_status_t;

typedef enum rp_next_state_status_e
{
    RP_STATUS_NO_MORE_TASK_SCHEDULE,
    RP_STATUS_HAVE_TO_SET_TIMER
} rp_next_state_status_t;

/*!
 *
 */
typedef struct rp_task_s
{
    uint8_t         hook_id;
    rp_task_types_t type;
    void ( *launch_task_callbacks )( void* );
    uint8_t          priority;
    bool             schedule_task_low_priority;
    rp_task_states_t state;
    // absolute Ms
    uint32_t start_time_ms;
    uint32_t start_time_100us;
    // Have to keep the initial start time to be able to switch asap task to
    // schedule task after long period
    uint32_t start_time_init_ms;
    uint32_t duration_time_ms;
} rp_task_t;

/*!
 *
 */
typedef enum rp_hook_status_e
{
    RP_HOOK_STATUS_OK,
    RP_HOOK_STATUS_ID_ERROR,
    RP_TASK_STATUS_ALREADY_RUNNING,
    RP_TASK_STATUS_SCHEDULE_TASK_IN_PAST,
    RP_TASK_STATUS_TASK_TOO_FAR_IN_FUTURE,
} rp_hook_status_t;

/*!
 *
 */

#ifdef __cplusplus
}
#endif

#endif  // __RADIO_PLANNER_TYPES_H__
/* --- EOF ------------------------------------------------------------------ */
