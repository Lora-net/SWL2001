/*!
 * @file      lr20xx_rttof.c
 *
 * @brief     Round-Trip Time of Flight (RTToF) driver implementation for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#include "lr20xx_rttof.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_RTTOF_SET_RESPONDER_ADDRESS_CMD_LENGTH ( 2 + 5 )
#define LR20XX_RTTOF_SET_INITIATOR_ADDRESS_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RTTOF_SET_TX_RX_DELAY_CMD_LENGTH ( 2 + 4 )
#define LR20XX_RTTOF_SET_PARAMETERS_CMD_LENGTH ( 2 + 1 )
#define LR20XX_RTTOF_GET_STATS_CMD_LENGTH ( 2 )
#define LR20XX_RTTOF_CONFIGURE_TIMING_SYNCHRONIZATION_CMD_LENGTH ( 2 + 1 )

#define LR20XX_RTTOF_RESULT_SIZE_IN_BYTE ( 4 )
#define LR20XX_RTTOF_STATS_SIZE_IN_BYTE ( 10 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for RTToF related operations
 */
enum
{
    LR20XX_RTTOF_SET_RESPONDER_ADDRESS_OC            = 0x0278,
    LR20XX_RTTOF_SET_INITIATOR_ADDRESS_OC            = 0x0279,
    LR20XX_RTTOF_GET_RESULTS_OC                      = 0x027A,
    LR20XX_RTTOF_SET_TX_RX_DELAY_OC                  = 0x027B,
    LR20XX_RTTOF_SET_PARAMETERS_OC                   = 0x027C,
    LR20XX_RTTOF_GET_STATS_OC                        = 0x027D,
    LR20XX_RTTOF_CONFIGURE_TIMING_SYNCHRONIZATION_OC = 0x021D,
};

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

lr20xx_status_t lr20xx_rttof_set_responder_address( const void* context, const uint32_t address, const uint8_t length )
{
    const uint8_t cbuffer[LR20XX_RTTOF_SET_RESPONDER_ADDRESS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_SET_RESPONDER_ADDRESS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_SET_RESPONDER_ADDRESS_OC >> 0 ),
        ( uint8_t ) ( address >> 24 ),
        ( uint8_t ) ( address >> 16 ),
        ( uint8_t ) ( address >> 8 ),
        ( uint8_t ) ( address >> 0 ),
        length,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RTTOF_SET_RESPONDER_ADDRESS_CMD_LENGTH, NULL,
                                                 0 );
}

lr20xx_status_t lr20xx_rttof_set_initiator_address( const void* context, const uint32_t address )
{
    const uint8_t cbuffer[LR20XX_RTTOF_SET_INITIATOR_ADDRESS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_SET_INITIATOR_ADDRESS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_SET_INITIATOR_ADDRESS_OC >> 0 ),
        ( uint8_t ) ( address >> 24 ),
        ( uint8_t ) ( address >> 16 ),
        ( uint8_t ) ( address >> 8 ),
        ( uint8_t ) ( address >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RTTOF_SET_INITIATOR_ADDRESS_CMD_LENGTH, NULL,
                                                 0 );
}

lr20xx_status_t lr20xx_rttof_get_results( const void* context, lr20xx_rttof_results_t* result )
{
    const uint8_t cbuffer[LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_GET_RESULTS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_GET_RESULTS_OC >> 0 ),
        0x00,
    };

    uint8_t data[LR20XX_RTTOF_RESULT_SIZE_IN_BYTE] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH, data, LR20XX_RTTOF_RESULT_SIZE_IN_BYTE );

    if( status == LR20XX_STATUS_OK )
    {
        result->val  = ( int32_t ) ( ( ( uint32_t ) data[0] << 16 ) + ( ( uint32_t ) data[1] << 8 ) +
                                   ( ( uint32_t ) data[2] << 0 ) );
        result->rssi = ( int8_t )( -( ( int8_t )( data[3] >> 1 ) ) );
    }

    return status;
}

lr20xx_status_t lr20xx_rttof_get_results_extended( const void* context, lr20xx_rttof_results_extended_t* result )
{
    const uint8_t cbuffer[LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_GET_RESULTS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_GET_RESULTS_OC >> 0 ),
        0x01,
    };

    uint8_t data[LR20XX_RTTOF_RESULT_SIZE_IN_BYTE * 2] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH, data, LR20XX_RTTOF_RESULT_SIZE_IN_BYTE * 2 );

    if( status == LR20XX_STATUS_OK )
    {
        result->res1.val =
            ( int32_t ) ( ( ( uint32_t ) data[0] << 16 ) + ( ( uint32_t ) data[1] << 8 ) + ( ( uint32_t ) data[2] << 0 ) );
        result->res1.rssi = ( int8_t )( -( ( int8_t )( data[3] >> 1 ) ) );
        result->res2.val =
            ( int32_t ) ( ( ( uint32_t ) data[4] << 16 ) + ( ( uint32_t ) data[5] << 8 ) + ( ( uint32_t ) data[6] << 0 ) );
        result->res2.rssi = ( int8_t )( -( ( int8_t )( data[7] >> 1 ) ) );
    }

    return status;
}

lr20xx_status_t lr20xx_rttof_get_gain_steps_extended( const void* context, uint8_t* gain_step_1, uint8_t* gain_step_2 )
{
    const uint8_t cbuffer[LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_GET_RESULTS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_GET_RESULTS_OC >> 0 ),
        0x02,
    };

    uint8_t data[2] = { 0 };

    const lr20xx_status_t status =
        ( lr20xx_status_t ) lr20xx_hal_read( context, cbuffer, LR20XX_RTTOF_GET_RESULTS_CMD_LENGTH, data, 2 );

    if( status == LR20XX_STATUS_OK )
    {
        *gain_step_1 = data[0];
        *gain_step_2 = data[1];
    }

    return status;
}

lr20xx_status_t lr20xx_rttof_set_tx_rx_delay( const void* context, uint32_t delay_in_rtc_step )
{
    const uint8_t cbuffer[LR20XX_RTTOF_SET_TX_RX_DELAY_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_SET_TX_RX_DELAY_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_SET_TX_RX_DELAY_OC >> 0 ),
        ( uint8_t ) ( delay_in_rtc_step >> 24 ),
        ( uint8_t ) ( delay_in_rtc_step >> 16 ),
        ( uint8_t ) ( delay_in_rtc_step >> 8 ),
        ( uint8_t ) ( delay_in_rtc_step >> 0 ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RTTOF_SET_TX_RX_DELAY_CMD_LENGTH, NULL, 0 );
}

lr20xx_status_t lr20xx_rttof_set_params( const void* context, const lr20xx_rttof_params_t* params )
{
    const uint8_t cbuffer[LR20XX_RTTOF_SET_PARAMETERS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_SET_PARAMETERS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_SET_PARAMETERS_OC >> 0 ),
        ( uint8_t ) ( ( ( uint8_t ) params->mode << 7 ) | ( ( uint8_t ) params->spy_mode << 6 ) | params->nb_symbol ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_RTTOF_SET_PARAMETERS_CMD_LENGTH, NULL, 0 );
}

lr20xx_status_t lr20xx_rttof_get_stats( const void* context, lr20xx_rttof_stats_t* stats )
{
    const uint8_t cbuffer[LR20XX_RTTOF_GET_STATS_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_GET_STATS_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_GET_STATS_OC >> 0 ),
    };

    uint8_t data[LR20XX_RTTOF_STATS_SIZE_IN_BYTE] = { 0 };

    const lr20xx_status_t status = ( lr20xx_status_t ) lr20xx_hal_read(
        context, cbuffer, LR20XX_RTTOF_GET_STATS_CMD_LENGTH, data, LR20XX_RTTOF_STATS_SIZE_IN_BYTE );

    if( status == LR20XX_STATUS_OK )
    {
        stats->exchange_valid    = ( uint16_t ) ( ( ( uint16_t ) data[0] << 8 ) + ( ( uint16_t ) data[1] << 0 ) );
        stats->request_valid     = ( uint16_t ) ( ( ( uint16_t ) data[2] << 8 ) + ( ( uint16_t ) data[3] << 0 ) );
        stats->response_done     = ( uint16_t ) ( ( ( uint16_t ) data[4] << 8 ) + ( ( uint16_t ) data[5] << 0 ) );
        stats->timeout           = ( uint16_t ) ( ( ( uint16_t ) data[6] << 8 ) + ( ( uint16_t ) data[7] << 0 ) );
        stats->request_discarded = ( uint16_t ) ( ( ( uint16_t ) data[8] << 8 ) + ( ( uint16_t ) data[9] << 0 ) );
    }

    return status;
}

lr20xx_status_t lr20xx_rttof_configure_timing_synchronization( const void*                                context,
                                                               lr20xx_rttof_timing_synchronization_role_t role,
                                                               lr20xx_rttof_dio_t                         dio )
{
    const uint8_t cbuffer[LR20XX_RTTOF_CONFIGURE_TIMING_SYNCHRONIZATION_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_RTTOF_CONFIGURE_TIMING_SYNCHRONIZATION_OC >> 8 ),
        ( uint8_t ) ( LR20XX_RTTOF_CONFIGURE_TIMING_SYNCHRONIZATION_OC >> 0 ),
        ( uint8_t ) ( ( ( ( uint8_t ) role ) << 6 ) | dio ),
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer,
                                                 LR20XX_RTTOF_CONFIGURE_TIMING_SYNCHRONIZATION_CMD_LENGTH, 0, 0 );
}

int32_t lr20xx_rttof_distance_raw_to_meter( lr20xx_radio_lora_bw_t rttof_bw, const int32_t raw_distance )
{
    int32_t       retval;
    const uint8_t bitcnt = 24u;

    retval = raw_distance;
    /* Convert the signed 24-bit integer into a signed 32-bit integer. */
    if( raw_distance >= ( int32_t )( 1 << ( bitcnt - 1 ) ) )
    {
        retval -= ( 1 << bitcnt );
    }

    const int32_t numerator = 150 * retval;
    const int32_t denominator = ( int32_t )(( lr20xx_radio_lora_get_bw_in_hz( rttof_bw ) * 4096 ) / 1000000 );

    return ( numerator / denominator );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
