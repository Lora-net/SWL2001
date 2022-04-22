/*!
 * \file      smtc_d2d.h
 *
 * \brief     device 2 device class b implementation , manage the device transmission in class b multicast
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SMTC_D2D_H__
#define __SMTC_D2D_H__

#include <stdint.h>
#include <stdbool.h>

#include "lr1_stack_mac_layer.h"
#include "lr1mac_defs.h"
#include "smtc_real.h"

#include "smtc_ping_slot.h"
#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * API definitions
 * ============================================================================
 */
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define D2D_HEADER_LORAWAN_SIZE 9
#define D2D_MIC_SIZE 4
#define USER_BUFFER_SIZE 255
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACRO---- --------------------------------------------------------
 */
#define SMTC_D2D_HAL_TRACE_PRINTF( msg ) SMTC_MODEM_HAL_TRACE_PRINTF( msg )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum smtc_class_b_d2d_status_e
{
    SMTC_CLASS_B_D2D_ERROR = 0,
    SMTC_CLASS_B_D2D_OK
} smtc_class_b_d2d_status_t;

typedef struct smtc_class_b_d2d_s
{
    smtc_ping_slot_t* ping_slot_obj;                         //!< the beacon object embeds the ping slot object
    uint8_t           tx_payload[USER_BUFFER_SIZE];          //!< the beacon payload
    uint8_t           tx_payload_encrypt[USER_BUFFER_SIZE];  //!< the beacon payload encrypted
    uint8_t           tx_payload_size;
    uint8_t           classb_d2d_id_rp;
    uint8_t           nb_trans_cnt;
    uint8_t           nb_trans_trial_cnt;
    uint8_t           nb_trans_max_retry;

    uint8_t           tx_priority;
    rx_session_type_t multi_cast_group_id;
    bool              tx_on_going;

    void ( *tx_event_callback )( void* );  //!< this call back is used to push an event to the user layer ,
    void* tx_event_context;  //!< the context given by the upper layer to transmit with the previous push_callback
    //!< function

    uint8_t ping_slots_mask[16];
} smtc_class_b_d2d_t;

void smtc_class_b_d2d_init( smtc_class_b_d2d_t* class_b_d2d_obj, smtc_ping_slot_t* ping_slot_obj,
                            uint8_t classb_d2d_id_rp, void ( *tx_event_callback )( void* tx_event_context ),
                            void*   tx_event_context );

smtc_class_b_d2d_status_t smtc_class_b_d2d_request_tx( smtc_class_b_d2d_t* class_b_d2d_obj,
                                                       rx_session_type_t multi_cast_group_id, uint8_t fport,
                                                       uint8_t priority, const uint8_t* payload, uint8_t payload_size,
                                                       uint8_t nb_rep, uint8_t nb_trans_max_retry,
                                                       uint8_t* ping_slots_mask, uint8_t ping_slots_mask_size );

uint8_t smtc_class_b_d2d_next_max_payload_length_get( smtc_class_b_d2d_t* class_b_d2d_obj,
                                                      rx_session_type_t   multi_cast_group_id );

status_lorawan_t smtc_class_b_d2d_fcnt_down( void* ping_slot_obj_void, uint32_t* fcnt_dwn_stack_tmp, uint32_t mic_in );

#ifdef __cplusplus
}
#endif

#endif  //  __SMTC_D2D_H__