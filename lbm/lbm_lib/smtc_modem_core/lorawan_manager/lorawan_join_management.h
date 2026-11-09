/**
 * @file      lorawan_join_management.h
 *
 * @brief     LoRaWAN join_management
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

#ifndef LORAWAN_JOIN_MANAGEMENT_H
#define LORAWAN_JOIN_MANAGEMENT_H

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
enum 
{
    SUCCESS_TO_LAUNCH_JOIN   = 0,
    FAIL_TO_LAUNCH_JOIN = 1
} ;

/**
 * @brief LoRaWAN join_management Object
 *
 * @struct lorawan_join_management_s
 *
 */
typedef struct lorawan_join_management_s
{
    uint8_t stack_id;
    uint8_t task_id;
    bool    enabled;

} lorawan_join_management_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief add generic task join
 *
 * @param stack_id
 */
void lorawan_join_add_task( uint8_t stack_id );

/**
 * @brief remove generic tack join
 *
 * @param stack_id
 */
void lorawan_join_remove_task( uint8_t stack_id );

/**
 * @brief Init a new LoRaWAN join_management services object
 *
 * @param service_id
 * @param task_id
 * @param downlink_callback
 * @param on_launch_callback
 * @param on_update_callback
 * @return uint8_t
 */
void lorawan_join_management_services_init( uint8_t* service_id, uint8_t task_id,
                                            uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                            void ( **on_launch_callback )( void* ),
                                            void ( **on_update_callback )( void* ), void** context_callback );

#ifdef __cplusplus
}
#endif

#endif  // LORAWAN_JOIN_MANAGEMENT_H

/* --- EOF ------------------------------------------------------------------ */
