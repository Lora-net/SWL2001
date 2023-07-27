/**
 * @file      lorawan_alcsync.h
 *
 * @brief     LoRaWAN Application Layer Clock Synchronization V1.0.0 Definitions
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

#ifndef LORAWAN_ALCSYNC_H
#define LORAWAN_ALCSYNC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

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

typedef enum alc_sync_ret_e
{
    ALC_SYNC_OK   = 0,
    ALC_SYNC_FAIL = 1,
} alc_sync_ret_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Init a new LoRaWAN ALCSync services object
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 */
void lorawan_alcsync_services_init( uint8_t* service_id, uint8_t task_id,
                                    uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                    void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                    void** context_callback );

/**
 * @brief Callback called at task launch
 *
 * @param context_callback
 */
void lorawan_alcsync_service_on_launch( void* service_id );

/**
 * @brief Callback called at the end of the task
 *
 * @param context_callback
 */
void lorawan_alcsync_service_on_update( void* service_id );

/**
 * @brief Callback to handle the downlink received by the LoRaWAN layer
 *
 * @param rx_down_data
 */
uint8_t lorawan_alcsync_service_downlink_handler( lr1_stack_mac_down_data_t* rx_down_data );

/**
 * @brief Enable ALCSync service
 *
 * @param stack_id
 * @param enabled
 */
void lorawan_alcsync_set_enabled( uint8_t stack_id, bool enabled );

/**
 * @brief Request to time sync now
 *
 * @param [in] stack_id
 * @param [in] ans_required     If the AnsRequired bit is set to 1 the end-device expects an answer whether its clock is
 *                              well synchronized or not. If this bit is set to 0, this signals to the AS that it only
 *                              needs to answer if the end-device clock is de-synchronized. sync
 * @return alc_sync_ret_t
 */
alc_sync_ret_t lorawan_alcsync_request_sync( uint8_t stack_id, bool ans_required );

/**
 * @brief Get the GPS time in second
 *
 * @param stack_id
 * @param gps_time_s         GPS Time in seconds
 * @return alc_sync_ret_t
 */
alc_sync_ret_t lorawan_alcsync_get_gps_time_second( uint8_t stack_id, uint32_t* gps_time_s );
void lorawan_alcsync_service_get_id (uint8_t *pkt_id, uint8_t * pkt_version, uint8_t* pkt_port );
bool lorawan_alcsync_mpa_injector( uint8_t stack_id, uint8_t* payload_in, receive_win_t rx_window, uint8_t* payload_out,
                               uint8_t* payload_out_length );
#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_ALCSYNC_H

/* --- EOF ------------------------------------------------------------------ */
