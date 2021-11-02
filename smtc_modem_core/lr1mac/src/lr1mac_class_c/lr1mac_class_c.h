/*!
 * \file      lr1mac_class_c.h
 *
 * \brief     LoRaWAN Class C API
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
#ifndef __LR1MAC_CLASS_C_H__
#define __LR1MAC_CLASS_C_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "lr1_stack_mac_layer.h"
#include "lr1mac_defs.h"
#include "radio_planner.h"
#include "smtc_secure_element.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
// clang-format off
#define LR1MAC_RCX_MIN_DURATION_MS   20
#define LR1MAC_NUMBER_OF_MC_SESSION  4
#define LR1MAC_NUMBER_OF_RXC_SESSION (1 + LR1MAC_NUMBER_OF_MC_SESSION) // 1 RxC + 4 Multicast address

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct lr1mac_rx_session_param_e
{
    bool                     enabled;
    uint32_t                 dev_addr;
    uint32_t                 fcnt_dwn;
    smtc_se_key_identifier_t nwk_skey;
    smtc_se_key_identifier_t app_skey;
    uint8_t                  rx_data_rate;
    uint32_t                 rx_frequency;
} lr1mac_rx_session_param_t;

typedef struct lr1mac_class_c_s
{
    bool             enabled;
    bool             started;
    lr1_stack_mac_t* lr1_mac;
    uint8_t          class_c_id4rp;
    radio_planner_t* rp;
    rp_status_t      planner_status;

    void ( *rx_callback )( void* );
    void* rx_context;
    void ( *push_callback )( void* );
    void* push_context;

    lr1mac_down_metadata_t rx_metadata;
    uint8_t                rx_payload_size;
    uint8_t                rx_payload[255];

    rx_session_type_t          rx_session_type;
    lr1mac_rx_session_param_t  rx_session_param[LR1MAC_NUMBER_OF_RXC_SESSION];
    lr1mac_rx_session_param_t* rx_session_param_ptr;

    receive_win_t    receive_window_type;
    rx_packet_type_t valid_rx_packet;
    uint8_t          tx_ack_bit;
    uint8_t          tx_mtype;
    uint8_t          rx_ftype;
    uint8_t          rx_major;
    uint8_t          rx_fctrl;
    uint8_t          rx_fopts[15];
    uint8_t          rx_fopts_length;
    uint8_t          rx_payload_empty;

    user_rx_packet_type_t available_app_packet;

} lr1mac_class_c_t;

typedef enum lr1mac_multicast_config_rc_e
{
    LR1MAC_MC_RC_OK,
    LR1MAC_MC_RC_ERROR_BAD_ID,
    LR1MAC_MC_RC_ERROR_BUSY,
    LR1MAC_MC_RC_ERROR_CRYPTO,
    LR1MAC_MC_RC_ERROR_PARAM,
    LR1MAC_MC_RC_ERROR_INCOMPATIBLE_SESSION,
    LR1MAC_MC_RC_ERROR_NOT_INIT,
} lr1mac_multicast_config_rc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

void lr1mac_class_c_init( lr1mac_class_c_t* class_c_obj, lr1_stack_mac_t* lr1_mac, radio_planner_t* rp,
                          uint8_t class_c_id_rp, void ( *rx_callback )( void* rx_context ), void* rx_context,
                          void ( *push_callback )( void* push_context ), void* push_context );

void lr1mac_class_c_enabled( lr1mac_class_c_t* class_c_obj, bool enable );

void lr1mac_class_c_stop( lr1mac_class_c_t* class_c_obj );
void lr1mac_class_c_start( lr1mac_class_c_t* class_c_obj );
void lr1mac_class_c_mac_rp_callback( lr1mac_class_c_t* class_c_obj );

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_set_group_config( lr1mac_class_c_t* class_c_obj,
                                                                        uint8_t mc_group_id, uint32_t mc_group_address,
                                                                        const uint8_t mc_ntw_skey[SMTC_SE_KEY_SIZE],
                                                                        const uint8_t mc_app_skey[SMTC_SE_KEY_SIZE] );

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_get_group_config( lr1mac_class_c_t* class_c_obj,
                                                                        uint8_t           mc_group_id,
                                                                        uint32_t*         mc_group_address );

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_start_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id,
                                                                     uint32_t freq, uint8_t dr );

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_get_session_status( lr1mac_class_c_t* class_c_obj,
                                                                          uint8_t mc_group_id, bool* is_session_started,
                                                                          uint32_t* freq, uint8_t* dr );

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_stop_session( lr1mac_class_c_t* class_c_obj,
                                                                    uint8_t           mc_group_id );

lr1mac_multicast_config_rc_t lr1mac_class_c_multicast_stop_all_sessions( lr1mac_class_c_t* class_c_obj );

#ifdef __cplusplus
}
#endif

#endif  // __LR1MAC_CLASS_C_H__

/* --- EOF ------------------------------------------------------------------ */
