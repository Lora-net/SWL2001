/*!
 * \file      stream.c
 *
 * \brief     streaming code implementation
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

#include "stream.h"

#include <string.h>

#include "modem_services_common.h"
#include "rose.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

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

/*
 * TODO Stream low level management should be here and not in modem_context
 */

stream_return_code_t stream_init( rose_t* ROSE )
{
    int rose_rc;
    // prepare stream module
    rose_rc = ROSE_init( ROSE, ROSE_DEFAULT_WL, ROSE_DEFAULT_MINFREE, ROSE_DEFAULT_RR, 1 );

    switch( rose_rc )
    {
    case ROSE_OK:
        return STREAM_OK;
    case ROSE_BAD_UNITSZ:
    case ROSE_NOMEM:
    default:
        return STREAM_FAIL;
    }
}

stream_return_code_t stream_enable_encryption( rose_t* ROSE )
{
    int rose_rc;
    // prepare stream module
    rose_rc = ROSE_enable_encryption( ROSE );

    switch( rose_rc )
    {
    case ROSE_OK:
        return STREAM_OK;
    case ROSE_BUSY:
    default:
        return STREAM_BUSY;
    }
}

bool stream_data_pending( rose_t* ROSE )
{
    return ROSE_getStatus( ROSE ) == ROSE_PENDTX;
}

// (only allowed when joined)
stream_return_code_t stream_add_data( rose_t* ROSE, const uint8_t* data, uint8_t len )
{
    // for now only one stream supported
    int err = 0;

    if( data == NULL )
    {
        return STREAM_FAIL;
    }

    if( len == 0 )
    {
        return STREAM_BADSIZE;
    }

    // check data record length
    err = ROSE_addRecord( ROSE, &data[0], len );
    if( err == ROSE_BAD_DATALEN )
    {
        return STREAM_BADSIZE;
    }
    if( err == ROSE_OVERRUN )
    {
        return STREAM_BUSY;
    }
    if( err != ROSE_OK )
    {
        return STREAM_FAIL;
    }

    // LOG_PRINTF( "stream_add_data (%d) [", len );
    // LOG_PACKARRAY( "", data, len );
    // LOG_MSG( "]\n" );
    return STREAM_OK;
}

stream_return_code_t stream_get_fragment( rose_t* ROSE, uint8_t* buf, uint32_t frag_ctn, uint8_t* len )
{
    int rose_rc;

    if( buf == NULL || len == NULL )
    {
        return STREAM_FAIL;
    }

    if( *len == 0 )
    {
        return STREAM_BADSIZE;
    }

    // Initialize return buffer
    memset( buf, 0xff, *len );

    rose_rc = ROSE_getData( ROSE, frag_ctn, buf, len );

    switch( rose_rc )
    {
    case ROSE_LFRAME_SIZE:
        return STREAM_BADSIZE;
    case ROSE_OVERRUN:
        return STREAM_OVERRUN;
    }

    // LOG_PRINTF( "stream_get_fragment (%d) [", *len );
    // LOG_PACKARRAY( "", buf, *len );
    // LOG_MSG( "]\n" );
    return STREAM_OK;
}

void stream_status( rose_t* ROSE, uint16_t* pending, uint16_t* free )
{
    if( pending != NULL )
    {
        *pending = ROSE_getPending( ROSE );
    }
    if( free != NULL )
    {
        *free = ROSE_getFree( ROSE );
    }
}

stream_return_code_t stream_process_dn_frame( rose_t* ROSE, const uint8_t* payload, uint8_t len )
{
    int rc;
    if( payload == NULL )
    {
        return STREAM_FAIL;
    }

    rc = ROSE_processDnFrame( ROSE, payload, len );
    if( rc == ROSE_NOTFORME )
    {
        return STREAM_UNKNOWN_SCMD;
    }
    return STREAM_OK;
}

uint8_t stream_get_rr( rose_t* ROSE )
{
    return ( ROSE->rr );
}

void stream_set_rr( rose_t* ROSE, uint8_t stream_rr )
{
    ROSE->rr = stream_rr;
}

stream_return_code_t stream_reset( rose_t* ROSE )
{
    memset( ROSE, 0, sizeof( rose_t ) );
    return STREAM_OK;
}

/* --- EOF ------------------------------------------------------------------ */
