/**
 * @file      almanac_update.h
 *
 * @brief     Almanac Update service API
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

#ifndef ALMANAC_UPDATE_H
#define ALMANAC_UPDATE_H

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
#define ALM_UPDATE_UPLINK_PAYLOAD_LENGTH 8

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Almanac update services return codes
 *
 * @enum almanac_update_return_code_t
 */

typedef enum almanac_update_return_code_e
{
    ALMANAC_OK = 0,
    ALMANAC_ERROR,
} almanac_update_return_code_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */
/**
 * @brief This function create the payload that can be sent to DAS for Almanac update service
 * @remark This function make a direct access to lr11xx radio
 *
 * @param [in] lr11xx_context lr11xx implementation context
 * @param [out] payload the created 8 bytes payload
 * @attention Please provide at least a 8 bytes buffer (ALM_UPDATE_UPLINK_PAYLOAD_LENGTH)
 *
 * @return Almanac service operation status
 */

almanac_update_return_code_t almanac_update_create_uplink_payload( const void* lr11xx_context, uint8_t payload[8] );

/**
 * @brief This function parse and process the payload that has been sent by DAS for Almanac update service
 * @remark This function make a direct access to lr11xx radio

 * @param [in] lr11xx_context lr11xx implementation context
 * @param [in] payload payload received from DAS
 * @param [in] payload_len length of the received payload
 *
 * @return Almanac service operation status
 */

almanac_update_return_code_t almanac_update_process_downlink_payload( const void* lr11xx_context, uint8_t* payload,
                                                                      uint8_t payload_len );

#ifdef __cplusplus
}
#endif

#endif  // ALMANAC_UPDATE_H

/* --- EOF ------------------------------------------------------------------ */
