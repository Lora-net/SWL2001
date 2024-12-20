/*!
 * \file      smtc_hal_flash.c
 *
 * \brief     FLASH Hardware Abstraction Layer implementation
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

#include "smtc_hal_flash.h"
#include "nrfx_nvmc.h"
#include "smtc_hal_dbg_trace.h"

#include <string.h>
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef FAIL
#define FAIL 0
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
static uint8_t copy_page[4096] = { 0x00 };
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief  Gets the page of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The page of a given address
 */
static uint32_t flash_get_page( uint32_t Address );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint16_t hal_flash_get_page_size( )
{
    return nrfx_nvmc_flash_page_size_get( );
}

uint8_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page )
{
    if( ( addr + ( nrfx_nvmc_flash_page_size_get( ) * ( nb_page - 1 ) ) ) >
        ( FLASH_PAGE_NUMBER * nrfx_nvmc_flash_page_size_get( ) ) )
    {
        SMTC_HAL_TRACE_ERROR( "FLASH_OPERATION_BAD PAGE ADDR\r\n" );
        return FAIL;
    }

    uint32_t first_page      = flash_get_page( addr );
    uint32_t start_addr_page = ADDR_FLASH_PAGE_0 + ( nrfx_nvmc_flash_page_size_get( ) * first_page );

    for( uint16_t i = 0; i < nb_page; i++ )
    {
        if( nrfx_nvmc_page_erase( start_addr_page ) != NRFX_SUCCESS )
        {
            return FAIL;
        }
        start_addr_page += nrfx_nvmc_flash_page_size_get( );
    }

    return SUCCESS;
}

uint32_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    nrfx_nvmc_bytes_write( addr, buffer, size );

    return SUCCESS;
}

void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    uint32_t     FlashIndex = 0;
    __IO uint8_t data8      = 0;

    while( FlashIndex < size )
    {
        data8 = *( __IO uint8_t* ) ( addr + FlashIndex );

        buffer[FlashIndex] = data8;

        FlashIndex++;
    }
}

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    uint32_t       remaining_size = size;
    const uint32_t page_size      = nrfx_nvmc_flash_page_size_get( );
    do
    {
        memset( copy_page, 0xFF, sizeof( copy_page ) );

        // Get the page number
        uint32_t num_page = flash_get_page( addr );

        // Get start address of this NVM page
        uint32_t start_addr_page = ADDR_FLASH_PAGE_0 + ( page_size * num_page );

        // Read data on this NVM page
        hal_flash_read_buffer( start_addr_page, copy_page, page_size );

        // Compute the index where data need to be updated in RAM copy page
        uint32_t index = ( addr - start_addr_page ) % page_size;

        // Compute the size of the data need to be copied without overflow on the next page
        uint32_t cpy_size = MIN( remaining_size, page_size - index );
        memcpy( &copy_page[index], &buffer[size - remaining_size], cpy_size );

        // Erase NVM page
        hal_flash_erase_page( addr, 1 );

        // Write the RAM buffer on this NVM page
        hal_flash_write_buffer( start_addr_page, copy_page, page_size );

        // increment address to the next page
        addr += page_size - index;

        // Reduce the amount of data remaining to be written
        remaining_size -= cpy_size;

    } while( remaining_size != 0 );
}
#endif  // USE_FLASH_READ_MODIFY_WRITE or MULTISTACK

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t flash_get_page( uint32_t Addr )
{
    return ( Addr - ADDR_FLASH_PAGE_0 ) / nrfx_nvmc_flash_page_size_get( );
}

/* --- EOF ------------------------------------------------------------------ */
