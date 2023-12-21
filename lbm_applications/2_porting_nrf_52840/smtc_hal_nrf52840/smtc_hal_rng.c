/*!
 * \file      smtc_hal_rng.c
 *
 * \brief     Random Number Generator Hardware Abstraction Layer implementation
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
#include <string.h>   // memcpy

#include "nrf_log.h"
#include "nrf_drv_rng.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_rng.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
#define RANDOM_BUFF_SIZE 4 /**< Random numbers buffer size. */

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

/** @brief Function for getting vector of random numbers.
 *
 * @param[out] p_buff       Pointer to unit8_t buffer for storing the bytes.
 * @param[in]  length       Number of bytes to take from pool and place in p_buff.
 *
 * @retval     Number of bytes actually placed in p_buff.
 */
static uint8_t random_vector_generate( uint8_t* p_buff, uint8_t size );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rng_init( void )
{
    uint32_t err_code;

    err_code = nrf_drv_rng_init( NULL );
    APP_ERROR_CHECK( err_code );
    nrfx_rng_start( );
    NRF_LOG_INFO( "RNG init." );
}

void hal_rng_uinit( void )
{
    nrf_drv_rng_uninit( );
}

void hal_rng_start( void )
{
    nrfx_rng_start( );
}

void hal_rng_stop( void )
{
    nrfx_rng_stop( );
}

uint32_t hal_rng_get_random( void )
{
    uint32_t rand_nb = 0;

    uint8_t p_buff[RANDOM_BUFF_SIZE] = { 0 };
    uint8_t length                   = random_vector_generate( p_buff, RANDOM_BUFF_SIZE );

    memcpy( ( uint8_t* ) &rand_nb, p_buff, length );

    return rand_nb;
}

uint32_t hal_rng_get_random_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    if( val_1 <= val_2 )
    {
        return ( uint32_t )( ( hal_rng_get_random( ) % ( val_2 - val_1 + 1 ) ) + val_1 );
    }
    else
    {
        return ( uint32_t )( ( hal_rng_get_random( ) % ( val_1 - val_2 + 1 ) ) + val_2 );
    }
}

int32_t hal_rng_get_signed_random_in_range( const int32_t val_1, const int32_t val_2 )
{
    uint32_t tmp_range = 0;  // ( val_1 <= val_2 ) ? ( val_2 - val_1 ) : ( val_1 - val_2 );

    if( val_1 <= val_2 )
    {
        tmp_range = ( val_2 - val_1 );
        return ( int32_t )( ( val_1 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
    else
    {
        tmp_range = ( val_1 - val_2 );
        return ( int32_t )( ( val_2 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint8_t random_vector_generate( uint8_t* p_buff, uint8_t size )
{
    uint32_t err_code;
    uint8_t  available;

    do
    {
        nrf_drv_rng_bytes_available( &available );
    } while( available < size );

    uint8_t length = MIN( size, available );

    err_code = nrf_drv_rng_rand( p_buff, length );
    APP_ERROR_CHECK( err_code );

    return length;
}

/* --- EOF ------------------------------------------------------------------ */
