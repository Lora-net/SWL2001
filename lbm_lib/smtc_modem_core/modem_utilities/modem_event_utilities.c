/*!
 * \file      modem_event_utilities.c
 *
 * \brief     manage asynchronous event to notify the host layer.
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

#include "modem_event_utilities.h"
#include "smtc_modem_hal_dbg_trace.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
struct
{
    uint8_t asynchronous_msgnumber;
    uint8_t modem_event_stack_id[MODEM_NUMBER_OF_EVENTS];
    uint8_t modem_event_count[MODEM_NUMBER_OF_EVENTS];
    uint8_t modem_event_status[MODEM_NUMBER_OF_EVENTS];
    uint8_t asynch_msg[MODEM_NUMBER_OF_EVENTS];
    void ( *app_callback )( void );
} modem_event_ctx;

#define asynchronous_msgnumber modem_event_ctx.asynchronous_msgnumber
#define modem_event_count modem_event_ctx.modem_event_count
#define modem_event_stack_id modem_event_ctx.modem_event_stack_id
#define modem_event_status modem_event_ctx.modem_event_status
#define asynch_msg modem_event_ctx.asynch_msg
#define app_callback modem_event_ctx.app_callback

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
void modem_event_init( void ( *callback )( void ) )
{
    for( int i = 0; i < MODEM_NUMBER_OF_EVENTS; i++ )
    {
        set_modem_event_count_and_status( i, 0, 0 );
    }
    asynchronous_msgnumber = 0;
    app_callback           = callback;
}
uint8_t get_modem_event_count( smtc_modem_event_type_t event_type )
{
    if( event_type >= MODEM_NUMBER_OF_EVENTS )
    {
        SMTC_MODEM_HAL_PANIC( );
    }

    return ( modem_event_count[event_type] );
}

uint8_t get_modem_event_status( uint8_t event_type )
{
    if( event_type >= MODEM_NUMBER_OF_EVENTS )
    {
        SMTC_MODEM_HAL_PANIC( );
    }
    return ( modem_event_status[event_type] );
}

void set_modem_event_count_and_status( uint8_t event_type, uint8_t value, uint8_t status )
{
    if( event_type < MODEM_NUMBER_OF_EVENTS )
    {
        modem_event_count[event_type]  = value;
        modem_event_status[event_type] = status;
    }
}

void increment_modem_event_count_and_status( uint8_t event_type, uint8_t status )
{
    if( event_type < MODEM_NUMBER_OF_EVENTS )
    {
        if( modem_event_count[event_type] < 255 )
        {
            modem_event_count[event_type]++;
        }
        // Set last status even if the number of event max is reached
        modem_event_status[event_type] = status;
    }
}

void decrement_asynchronous_msgnumber( void )
{
    if( asynchronous_msgnumber > 0 )
    {
        asynchronous_msgnumber--;
    }
    else
    {
        asynchronous_msgnumber = 0;
    }
}

uint8_t get_asynchronous_msgnumber( void )
{
    return ( asynchronous_msgnumber );
}

void increment_asynchronous_msgnumber( uint8_t event_type, uint8_t status, uint8_t stack_id )
{
    // Next condition should never append because only one asynch msg by type of message
    if( asynchronous_msgnumber >= MODEM_NUMBER_OF_EVENTS )
    {
        return;
    }
    uint8_t tmp;
    tmp = get_modem_event_count( event_type );

    if( tmp == 0 )
    {
        asynch_msg[asynchronous_msgnumber] = event_type;
        asynchronous_msgnumber++;
    }

    increment_modem_event_count_and_status( event_type, status );
    modem_event_stack_id[event_type] = stack_id;

    if( *app_callback != NULL )
    {
        app_callback( );
    }
}

uint8_t get_last_msg_event( uint8_t* stack_id )
{
    if( asynchronous_msgnumber > 0 )
    {
        *stack_id = modem_event_stack_id[asynch_msg[asynchronous_msgnumber - 1]];
        return asynch_msg[asynchronous_msgnumber - 1];
    }
    *stack_id = 0xFF;
    return 0xFF;
}
