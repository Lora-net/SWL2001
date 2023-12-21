/*!
 * \file      fifo_ctrl.c
 *
 * \brief
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
#include <string.h>

#include "fifo_ctrl.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LEN_DATA_SIZE ( 2 )
#define LEN_METADATA_SIZE ( 1 )

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
static fifo_return_status_t ctrl_set( fifo_ctrl_t* ctrl, const uint8_t* buffer, const uint16_t buffer_len,
                                      const void* metadata, const uint8_t metadata_len );

static fifo_return_status_t ctrl_get( fifo_ctrl_t* ctrl, uint8_t* buffer, uint16_t* data_len,
                                      const uint16_t data_buffer_size, void* metadata, uint8_t* metadata_len,
                                      const uint8_t metadata_buffer_size );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void fifo_ctrl_init( fifo_ctrl_t* ctrl, uint8_t* buffer, const uint16_t buffer_size )
{
    ctrl->buffer      = buffer;
    ctrl->buffer_size = buffer_size;
    fifo_ctrl_clear( ctrl );
}

void fifo_ctrl_clear( fifo_ctrl_t* ctrl )
{
    ctrl->read_offset  = 0;
    ctrl->write_offset = 0;
    ctrl->nb_element   = 0;
    ctrl->write_cnt    = 0;
    ctrl->read_cnt     = 0;
    ctrl->drop_cnt     = 0;
    ctrl->free_space   = ctrl->buffer_size;
}

void fifo_ctrl_print_stat( const fifo_ctrl_t* ctrl )
{
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "----------------------------------\n" );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "fifo_ctrl_print_stat\n" );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "Buffer size : %d\n", ctrl->buffer_size );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "Current elt : %d\n", ctrl->nb_element );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "Free space  : %d\n", ctrl->free_space );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "Write       : %d\n", ctrl->write_cnt );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "Read        : %d\n", ctrl->read_cnt - ctrl->drop_cnt );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "Drop        : %d\n", ctrl->drop_cnt );
    SMTC_MODEM_HAL_TRACE_INFO_DEBUG( "----------------------------------\n" );
}

uint16_t fifo_ctrl_get_nb_elt( const fifo_ctrl_t* ctrl )
{
    return ctrl->nb_element;
}

uint16_t fifo_ctrl_get_free_space( const fifo_ctrl_t* ctrl )
{
    return ctrl->free_space;
}

fifo_return_status_t fifo_ctrl_get( fifo_ctrl_t* ctrl, uint8_t* buffer, uint16_t* data_len,
                                    const uint16_t data_buffer_size, void* metadata, uint8_t* metadata_len,
                                    const uint8_t metadata_buffer_size )
{
    smtc_modem_hal_disable_modem_irq( );
    fifo_return_status_t ret =
        ctrl_get( ctrl, buffer, data_len, data_buffer_size, metadata, metadata_len, metadata_buffer_size );
    smtc_modem_hal_enable_modem_irq( );

    return ret;
}

fifo_return_status_t fifo_ctrl_set( fifo_ctrl_t* ctrl, const uint8_t* buffer, const uint16_t buffer_len,
                                    const void* metadata, const uint8_t metadata_len )
{
    smtc_modem_hal_disable_modem_irq( );
    fifo_return_status_t ret = ctrl_set( ctrl, buffer, buffer_len, metadata, metadata_len );
    smtc_modem_hal_enable_modem_irq( );
    return ret;
}

static fifo_return_status_t ctrl_set( fifo_ctrl_t* ctrl, const uint8_t* buffer, const uint16_t buffer_len,
                                      const void* metadata, const uint8_t metadata_len )
{
    uint16_t total_write_len = LEN_DATA_SIZE + LEN_METADATA_SIZE + metadata_len + buffer_len;

    if( total_write_len > ctrl->buffer_size )
    {
        return FIFO_STATUS_BUFFER_TOO_SMALL;
    }

    while( ctrl->free_space < total_write_len )
    {
        // Not enough free space --> Remove oldest
        ctrl_get( ctrl, NULL, NULL, 0, NULL, NULL, 0 );
        ctrl->drop_cnt += 1;
    }

    // Write data length - 2 bytes MSB first
    ctrl->buffer[ctrl->write_offset] = ( uint8_t )( buffer_len >> 8 );
    ctrl->write_offset += 1;
    ctrl->write_offset %= ctrl->buffer_size;
    ctrl->buffer[ctrl->write_offset] = ( uint8_t )( buffer_len );
    ctrl->write_offset += 1;
    ctrl->write_offset %= ctrl->buffer_size;

    // Write metadata length
    ctrl->buffer[ctrl->write_offset] = metadata_len;
    ctrl->write_offset += 1;
    ctrl->write_offset %= ctrl->buffer_size;

    // Write metadata
    if( metadata_len != 0 )
    {
        if( ( ctrl->write_offset + metadata_len ) > ctrl->buffer_size )
        {
            memcpy( ctrl->buffer + ctrl->write_offset, ( uint8_t* ) metadata, ctrl->buffer_size - ctrl->write_offset );
            memcpy( ctrl->buffer, ( uint8_t* ) metadata + ctrl->buffer_size - ctrl->write_offset,
                    metadata_len - ( ctrl->buffer_size - ctrl->write_offset ) );
        }
        else
        {
            memcpy( ctrl->buffer + ctrl->write_offset, ( uint8_t* ) metadata, metadata_len );
        }
        ctrl->write_offset += metadata_len;
        ctrl->write_offset %= ctrl->buffer_size;
    }

    // Write data
    if( buffer_len != 0 )
    {
        if( ( ctrl->write_offset + buffer_len ) > ctrl->buffer_size )
        {
            memcpy( ctrl->buffer + ctrl->write_offset, buffer, ctrl->buffer_size - ctrl->write_offset );
            memcpy( ctrl->buffer, buffer + ctrl->buffer_size - ctrl->write_offset,
                    buffer_len - ( ctrl->buffer_size - ctrl->write_offset ) );
        }
        else
        {
            memcpy( ctrl->buffer + ctrl->write_offset, buffer, buffer_len );
        }

        ctrl->write_offset += buffer_len;
        ctrl->write_offset %= ctrl->buffer_size;
    }

    ctrl->free_space -= total_write_len;
    ctrl->nb_element += 1;
    ctrl->write_cnt += 1;

    return FIFO_STATUS_OK;
}

static fifo_return_status_t ctrl_get( fifo_ctrl_t* ctrl, uint8_t* buffer, uint16_t* data_len,
                                      const uint16_t data_buffer_size, void* metadata, uint8_t* metadata_len,
                                      const uint8_t metadata_buffer_size )
{
    if( ctrl->nb_element == 0 )
    {
        return FIFO_STATUS_BUFFER_EMPTY;
    }

    // Read data & metadata size (read_offset update is done later if input param are ok)
    uint16_t read_data_len = ( ( uint16_t ) ctrl->buffer[ctrl->read_offset] ) << 8;
    read_data_len += ( ( uint16_t ) ctrl->buffer[( ctrl->read_offset + 1 ) % ctrl->buffer_size] );
    uint8_t read_metadata_len = ctrl->buffer[( ctrl->read_offset + 2 ) % ctrl->buffer_size];

    // Buffer & metadata are NULL --> drop old message --> don't check/update size of buffer
    if( ( buffer != NULL ) && ( metadata != NULL ) )
    {
        if( ( data_len == NULL ) || ( metadata_len == NULL ) )
        {
            return FIFO_STATUS_PARAM_ERROR;
        }

        // Buffer length are ok -> save length infos
        *data_len     = read_data_len;
        *metadata_len = read_metadata_len;

        if( ( read_data_len > data_buffer_size ) || ( read_metadata_len > metadata_buffer_size ) )
        {
            return FIFO_STATUS_BUFFER_TOO_SMALL;
        }
    }

    // Update read offset (only if there is no error)
    ctrl->read_offset += ( LEN_DATA_SIZE + LEN_METADATA_SIZE );
    ctrl->read_offset %= ctrl->buffer_size;

    // Copy metadata (if required)
    if( ( metadata != NULL ) && ( read_metadata_len != 0 ) )
    {
        if( ( ctrl->read_offset + read_metadata_len ) > ctrl->buffer_size )
        {
            memcpy( ( uint8_t* ) metadata, ctrl->buffer + ctrl->read_offset, ctrl->buffer_size - ctrl->read_offset );
            memcpy( ( uint8_t* ) metadata + ctrl->buffer_size - ctrl->read_offset, ctrl->buffer,
                    read_metadata_len - ( ctrl->buffer_size - ctrl->read_offset ) );
        }
        else
        {
            memcpy( ( uint8_t* ) metadata, ctrl->buffer + ctrl->read_offset, read_metadata_len );
        }
    }
    ctrl->read_offset += read_metadata_len;
    ctrl->read_offset %= ctrl->buffer_size;

    // Copy data (if required)
    if( ( buffer != NULL ) && ( read_data_len != 0 ) )
    {
        if( ( ctrl->read_offset + read_data_len ) > ctrl->buffer_size )
        {
            memcpy( buffer, ctrl->buffer + ctrl->read_offset, ctrl->buffer_size - ctrl->read_offset );
            memcpy( buffer + ctrl->buffer_size - ctrl->read_offset, ctrl->buffer,
                    read_data_len - ( ctrl->buffer_size - ctrl->read_offset ) );
        }
        else
        {
            memcpy( buffer, ctrl->buffer + ctrl->read_offset, read_data_len );
        }
    }
    ctrl->read_offset += read_data_len;
    ctrl->read_offset %= ctrl->buffer_size;

    ctrl->free_space += ( LEN_DATA_SIZE + LEN_METADATA_SIZE + read_metadata_len + read_data_len );
    ctrl->nb_element -= 1;
    ctrl->read_cnt += 1;

    return FIFO_STATUS_OK;
}
