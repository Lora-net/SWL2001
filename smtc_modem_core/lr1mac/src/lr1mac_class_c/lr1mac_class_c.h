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
#include "smtc_multicast.h"
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
#define LR1MAC_NUMBER_OF_RXC_SESSION RX_SESSION_COUNT // Unicast + Multicast

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct lr1mac_class_c_s
{
    bool             enabled;        // Service is enabled/disabled
    bool             started;        // Class C window is opened/stopped
    lr1_stack_mac_t* lr1_mac;        // lr1mac object
    uint8_t          class_c_id4rp;  // Hook ID for radio planner
    radio_planner_t* rp;             // Radio planner object
    rp_status_t      planner_status;

    void ( *rx_callback )( void* );  // radio planner callback to set the Rx windows parameters
    void* rx_context;
    void ( *push_callback )( void* );  // Callback to handle received downlink
    void* push_context;

    lr1mac_down_metadata_t rx_metadata;
    uint8_t                rx_payload_size;
    uint8_t                rx_payload[255];

    rx_session_type_t rx_session_index;

    // Contains All Rx Session, Unicast and Multicast
    lr1mac_rx_session_param_t  rx_session_param_unicast;
    lr1mac_rx_session_param_t* rx_session_param[LR1MAC_NUMBER_OF_RXC_SESSION];

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Init the class C and the callback to push downlink
 *
 * @param class_c_obj   // Class C object
 * @param lr1_mac       // lr1mac object
 * @param rp            // Radio planner object
 * @param class_c_id_rp // Hook ID for radio planner
 * @param rx_callback   // radio planner callback to set the Rx windows parameters
 * @param rx_context
 * @param push_callback // Callback to handle received downlink
 * @param push_context
 */
void lr1mac_class_c_init( lr1mac_class_c_t* class_c_obj, lr1_stack_mac_t* lr1_mac, smtc_multicast_t* multicast_obj,
                          radio_planner_t* rp, uint8_t class_c_id_rp, void ( *rx_callback )( void* rx_context ),
                          void* rx_context, void ( *push_callback )( void* push_context ), void* push_context );

/**
 * @brief Class C service enablement
 *
 * @remark this function does not start the class C, just enable the service
 *
 * @param class_c_obj
 * @param enable
 */
void lr1mac_class_c_enabled( lr1mac_class_c_t* class_c_obj, bool enable );

/**
 * @brief Stop class C windows
 *
 * @param class_c_obj
 */
void lr1mac_class_c_stop( lr1mac_class_c_t* class_c_obj );

/**
 * @brief Start class C windows
 *
 * @param class_c_obj
 */
void lr1mac_class_c_start( lr1mac_class_c_t* class_c_obj );

/**
 * @brief Callback called by radio planner on interrupt
 *
 * @param class_c_obj
 */
void lr1mac_class_c_mac_rp_callback( lr1mac_class_c_t* class_c_obj );

/**
 * @brief Start the class C multicast session
 *
 * @remark need to configured before
 *
 * @param class_c_obj
 * @param mc_group_id
 * @param freq
 * @param dr
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t lr1mac_class_c_multicast_start_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id,
                                                                   uint32_t freq, uint8_t dr );

/**
 * @brief Stop the class C multicast session
 *
 * @param class_c_obj
 * @param mc_group_id
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t lr1mac_class_c_multicast_stop_session( lr1mac_class_c_t* class_c_obj, uint8_t mc_group_id );

/**
 * @brief Stop all class C multicast session
 *
 * @param class_c_obj
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t lr1mac_class_c_multicast_stop_all_sessions( lr1mac_class_c_t* class_c_obj );

/**
 * @brief Get class C multicast status for a session
 *
 * @param class_c_obj
 * @param mc_group_id
 * @param is_session_started
 * @param freq
 * @param dr
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t lr1mac_class_c_multicast_get_session_status( lr1mac_class_c_t* class_c_obj,
                                                                        uint8_t mc_group_id, bool* is_session_started,
                                                                        uint32_t* freq, uint8_t* dr );

#ifdef __cplusplus
}
#endif

#endif  // __LR1MAC_CLASS_C_H__

/* --- EOF ------------------------------------------------------------------ */
