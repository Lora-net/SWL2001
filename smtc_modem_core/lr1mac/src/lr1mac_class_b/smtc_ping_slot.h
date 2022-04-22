/*!
 * \file      smtc_ping_slot.h
 *
 * \brief     Ping Slot management for LoRaWAN class B devices
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
#ifndef __SMTC_PING_SLOT_H__
#define __SMTC_PING_SLOT_H__

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
#define MIN_PING_SLOT_WINDOW_SYMB 6
#define MAX_PING_SLOT_WINDOW_MS 500
#define RX_BEACON_TIMESTAMP_ERROR 0

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define LR1MAC_NUMBER_OF_CLASS_B_SESSION RX_SESSION_COUNT  // Unicast + Multicast

/**
 * @brief Not defined datarate value
 */
#define LR1MAC_CLASS_B_MC_NO_DATARATE 0xFF

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Rx session context
 *
 */
typedef struct lr1mac_rx_b_session_param_e
{
    bool                                         enabled;                // Is the session enabled
    uint32_t                                     dev_addr;               // Device address
    uint32_t                                     fcnt_dwn;               // downlink frame counter
    smtc_se_key_identifier_t                     nwk_skey;               // Newtork session key
    smtc_se_key_identifier_t                     app_skey;               // Applicative session key
    uint8_t                                      rx_data_rate;           // Rx datarate
    uint32_t                                     rx_frequency;           // Rx Frequency
    smtc_multicast_fpending_bit_prioritization_t fpending_bit;           // FPending bit status
    uint8_t                                      ping_slot_periodicity;  // Value set by the user [0 to 7]
    smtc_ping_slot_parameters_t                  ping_slot_parameters;   // ping slot parameters
    uint16_t                                     rx_window_symb;  // Number of Rx window symboles to listen preamble
} lr1mac_rx_session_b_param_t;

/**
 * @brief Ping Slot context
 *
 */
typedef struct smtc_ping_slot_s
{
    lr1_stack_mac_t* lr1_mac;
    uint8_t          ping_slot_id4rp;
    radio_planner_t* rp;
    rp_status_t      planner_status;
    bool             enabled;  // is ping slot service enabled

    void ( *rx_callback )( void* );    // Callback to setup the radio with Rx parameters
    void* rx_context;                  // Context of the callback to setup the radio with Rx parameters
    void ( *push_callback )( void* );  // Callback to push the downlink to the upper layer
    void* push_context;                // Context to the callback to push the downlink to the upper layer

    void ( *d2d_callback )( void* );  // Callback used by the Device To Device to send a downlink to others devices
    void* d2d_context;  // Context of the callback used by the Device To Device to send a downlink to others devices

    status_lorawan_t ( *d2d_check_fcnt_down_callback )( void*, uint32_t* fcnt_dwn_stack_tmp, uint32_t mic_in );

    lr1mac_down_metadata_t rx_metadata;      // Downlink metadata
    uint8_t                rx_payload_size;  //@note Have to by replace by a fifo objet to manage class b
    uint8_t                rx_payload[255];  //@note Have to by replace by a fifo objet to manage class b

    rx_session_type_t          rx_session_index;          // Current running Rx session  (unicast, multicast0, ...)
    lr1mac_rx_session_param_t  rx_session_param_unicast;  // Unicast session context
    lr1mac_rx_session_param_t* rx_session_param[LR1MAC_NUMBER_OF_CLASS_B_SESSION];  // Array of pointer to address
                                                                                    // Unicast and Multicast session

    uint32_t next_beacon_timestamp;
    uint32_t beacon_reserved_ms;
    uint32_t beacon_guard_ms;

    rx_packet_type_t      valid_rx_packet;
    uint8_t               tx_ack_bit;
    uint8_t               tx_mtype;
    uint8_t               rx_ftype;
    uint8_t               rx_major;
    uint8_t               rx_fctrl;
    uint8_t               rx_fopts[15];
    uint8_t               rx_fopts_length;
    uint8_t               rx_payload_empty;
    uint32_t              last_valid_rx_beacon_ms;
    uint32_t              last_valid_rx_ping_slot_toa;
    user_rx_packet_type_t available_app_packet;

    uint32_t last_toa;  // Last downlink Time On Air

} smtc_ping_slot_t;

/**
 * @brief Init the class B ping slot object and the callback to push downlink
 *
 * @param [in,out] ping_slot_obj    // Ping slot object
 * @param [in] lr1_mac              // lr1mac object
 * @param [in] multicast_obj        // multicast object
 * @param [in] rp                   // Radio planner object
 * @param [in] ping_slot_id_rp      // Hook ID for radio planner
 * @param [in] rx_callback          // radio planner callback to set the Rx windows parameters
 * @param [in] rx_context           // callback context
 * @param [in] push_callback        // Callback to handle received downlink
 * @param [in] push_context         // callback context
 */
void smtc_ping_slot_init( smtc_ping_slot_t* ping_slot_obj, lr1_stack_mac_t* lr1_mac, smtc_multicast_t* multicast_obj,
                          radio_planner_t* rp, uint8_t ping_slot_id_rp, void ( *rx_callback )( void* rx_context ),
                          void* rx_context, void ( *push_callback )( void* push_context ), void* push_context );

/**
 * @brief init all class B sessions when a beacon is received
 *
 * @remark Must be called for each beacon interrupt ( received or not )
 *
 * @param [in,out] ping_slot_obj         // Ping slot object
 * @param [in] beacon_timestamp          // Beacon timestamp
 * @param [in] next_beacon_timestamp     // Next beacon timestamp
 * @param [in] beacon_reserved_ms        // beacon reserved ms
 * @param [in] beacon_guard_ms           // beacon guard ms
 * @param [in] beacon_epoch_time         // beacon epoch time
 */
void smtc_ping_slot_init_after_beacon( smtc_ping_slot_t* ping_slot_obj, uint32_t beacon_timestamp,
                                       uint32_t next_beacon_timestamp, uint32_t beacon_reserved_ms,
                                       uint32_t beacon_guard_ms, uint32_t beacon_epoch_time );

/**
 * @brief Start the ping slot windows and radio configuration
 *
 * @param [in,out] ping_slot_obj    // Ping slot object
 */
void smtc_ping_slot_start( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Stop the ping slots windows
 *
 * @param [in,out] ping_slot_obj    // Ping slot object
 */
void smtc_ping_slot_stop( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Radio planner callback on radio planner IT
 *
 * @param [in,out] ping_slot_obj    // Ping slot object
 */
void smtc_ping_slot_rp_callback( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Handle the received downlink
 *
 * @param [in,out] ping_slot_obj    // Ping slot object
 */
void smtc_ping_slot_mac_rp_callback( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Compute the first ping slot timing
 *
 * @param [in] beacon_time_received_ms   The beacon timestamp at the begin of the transmission (without TOA)
 * @param [in] beacon_reserved_ms        No ping slit could be start in this period
 * @param [in] beacon_epoch_time         Epoch Time decoded in beacon
 * @param [in] dev_addr                  Devaddr of the requested session
 * @param [in] ping_period               number of ping period
 * @return uint32_t
 */

uint32_t smtc_ping_slot_compute_first_slot( uint32_t beacon_time_received_100us, uint32_t beacon_reserved_ms,
                                            uint32_t beacon_epoch_time, uint32_t dev_addr, uint16_t ping_period );

/**
 * @brief Start a ping slot multicast session
 *
 * @remark need to configured before
 * @remark Will be started on beacon reception
 *
 * @param [in,out] ping_slot_obj         // Ping slot object
 * @param [in] mc_group_id               // multicast group ID
 * @param [in] freq                      // Frequency MHz
 * @param [in] dr                        // Datarate
 * @param [in] ping_slot_periodicity     // ping slot periodicity
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_start_session( smtc_ping_slot_t* ping_slot_obj,
                                                                     uint8_t mc_group_id, uint32_t freq, uint8_t dr,
                                                                     uint8_t ping_slot_periodicity );

/**
 * @brief Stop a ping slot multicast session
 *
 * @param [in,out] ping_slot_obj    // Ping slot object
 * @param [in] mc_group_id          // multicast group ID
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_stop_session( smtc_ping_slot_t* ping_slot_obj,
                                                                    uint8_t           mc_group_id );

/**
 * @brief Stop all ping slot multicast session
 *
 * @param [in] ping_slot_obj    // Ping Slot object
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_stop_all_sessions( smtc_ping_slot_t* ping_slot_obj );

/**
 * @brief Get class B multicast status for a session
 *
 * @param [in,out] ping_slot_obj         // Ping slot object
 * @param [in] mc_group_id               // multicast group ID
 * @param [out] is_session_started       // Is the multicast session launched
 * @param [out] waiting_beacon_to_start  // Is waiting a beacon to be started
 * @param [out] freq                     // Frequency MHs
 * @param [out] dr                       // Datarate
 * @param [out] ping_slot_periodicity    // ping slot periodicity
 * @return smtc_multicast_config_rc_t
 */
smtc_multicast_config_rc_t smtc_ping_slot_multicast_b_get_session_status( smtc_ping_slot_t* ping_slot_obj,
                                                                          uint8_t mc_group_id, bool* is_session_started,
                                                                          bool* waiting_beacon_to_start, uint32_t* freq,
                                                                          uint8_t* dr, uint8_t* ping_slot_periodicity );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_PING_SLOT_H__

/* --- EOF ------------------------------------------------------------------ */
