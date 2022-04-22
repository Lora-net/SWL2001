/**
 * @file      almanac_update.c
 *
 * @brief     Almanac Update service implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include <stddef.h>  //NULL
#include <string.h>  //memcpy

#include "almanac_update.h"
#include "modem_services_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define SERVICE_LR11XX_GNSS_CONTEXT_STATUS_LENGTH 9
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

almanac_update_return_code_t almanac_update_create_uplink_payload( const void* lr11xx_context, uint8_t payload[8] )
{
    uint8_t local_buff[SERVICE_LR11XX_GNSS_CONTEXT_STATUS_LENGTH];
    // ask lr11xx to get gnss context status
    if( smtc_modem_services_lr11xx_gnss_get_context_status( lr11xx_context, local_buff ) != MODEM_SERVICES_RADIO_OK )
    {
        return ALMANAC_ERROR;
    }

    // Do not take hte first byte from the lr11xx response
    memcpy( payload, &local_buff[1], SERVICE_LR11XX_GNSS_CONTEXT_STATUS_LENGTH - 1 );

    return ALMANAC_OK;
}

almanac_update_return_code_t almanac_update_process_downlink_payload( const void* lr11xx_context, uint8_t* payload,
                                                                      uint8_t payload_len )
{
    // take buffer received from DAS and push it to lr11xx removing the first 2 bytes (upcount, updelay)
    if( smtc_modem_services_lr11xx_gnss_push_dmc_msg( lr11xx_context, &payload[2], payload_len - 2 ) !=
        MODEM_SERVICES_RADIO_OK )
    {
        return ALMANAC_ERROR;
    }
    return ALMANAC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
