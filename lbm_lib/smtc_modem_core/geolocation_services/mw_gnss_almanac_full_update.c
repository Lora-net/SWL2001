/**
 * @file      mw_gnss_almanac_full_update.c
 *
 * @brief     Helper function for full almanac update
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

#include "smtc_modem_hal_dbg_trace.h"
#include "modem_core.h"

#include "lr11xx_system.h"
#include "lr11xx_gnss.h"

#include "mw_gnss_almanac_full_update.h"

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

static uint32_t get_almanac_crc( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_modem_return_code_t mw_gnss_almanac_full_update( const uint8_t* almanac, uint16_t almanac_size )
{
    /* Ensure that the given almanac image is a multiple of block size */
    if( ( almanac_size % LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE ) != 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_almanac_full_update: invalid almanac size %u (should be multiple of %d)\n",
                                    almanac_size, LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE );
        return SMTC_MODEM_RC_INVALID;
    }

#if ( MODEM_HAL_DBG_TRACE )
    uint32_t t_start_ms = smtc_modem_hal_get_time_in_ms( );
#endif

    /* Compare current almanac CRC with the given one */
    uint32_t update_almanac_crc = ( almanac[6] << 24 ) + ( almanac[5] << 16 ) + ( almanac[4] << 8 ) + ( almanac[3] );
    if( update_almanac_crc != get_almanac_crc( ) )
    {
        /* Load almanac in LR11xx flash */
        lr11xx_status_t err;
        for( int i = 0; i < almanac_size; i += LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE )
        {
            err = lr11xx_gnss_almanac_update( modem_get_radio_ctx( ), almanac + i, 1 );
            if( err != LR11XX_STATUS_OK )
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_almanac_full_update: failed at block %d\n", i );
                return SMTC_MODEM_RC_FAIL;
            }
        }
        /* Check CRC again to confirm proper update */
        if( get_almanac_crc( ) != update_almanac_crc )
        {
            SMTC_MODEM_HAL_TRACE_ERROR( "mw_gnss_almanac_full_update: CRC mismatch\n" );
            return SMTC_MODEM_RC_FAIL;
        }
        else
        {
#if ( MODEM_HAL_DBG_TRACE )
            uint32_t t_end_ms = smtc_modem_hal_get_time_in_ms( );
            SMTC_MODEM_HAL_TRACE_INFO( "mw_gnss_almanac_full_update: SUCCESS (%u ms)\n", t_end_ms - t_start_ms );
#endif
        }
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_INFO( "mw_gnss_almanac_full_update: already up-to-date\n" );
    }

    return SMTC_MODEM_RC_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t get_almanac_crc( void )
{
    lr11xx_status_t                         err;
    lr11xx_gnss_context_status_bytestream_t context_status_bytestream;
    lr11xx_gnss_context_status_t            context_status;

    err = lr11xx_gnss_get_context_status( modem_get_radio_ctx( ), context_status_bytestream );
    if( err != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to get gnss context status\n" );
        return 0;
    }

    err = lr11xx_gnss_parse_context_status_buffer( context_status_bytestream, &context_status );
    if( err != LR11XX_STATUS_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Failed to parse gnss context status to get almanac status\n" );
        return 0;
    }

    return context_status.global_almanac_crc;
}

/* --- EOF ------------------------------------------------------------------ */