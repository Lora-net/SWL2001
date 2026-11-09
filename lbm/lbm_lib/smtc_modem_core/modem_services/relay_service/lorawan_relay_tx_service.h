/**
 * @file      lorawan_relay_tx_service.h
 *
 * @brief     Relay TX service. Handle automatic activation of relay mode.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#ifndef LORAWAN_RELAY_TX_SERVICE_H
#define LORAWAN_RELAY_TX_SERVICE_H

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Init a the relay TX service
 *
 * @param[in]   service_id          Service ID (provided by supervisor)
 * @param[in]   task_id             Task ID (provided by supervisor)
 * @param[out]  downlink_callback   Callbak called after RX2
 * @param[out]  on_lunch_callback   Callback called to launched the service
 * @param[out]  on_update_callback  Callback called when service has finished
 * @param[out]  context_callback    Context for every callback
 */
void lorawan_relay_tx_services_init( uint8_t* service_id, uint8_t task_id,
                                     uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                     void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                     void** context_callback );

#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_RELAY_TX_SERVICE_H

/* --- EOF ------------------------------------------------------------------ */
