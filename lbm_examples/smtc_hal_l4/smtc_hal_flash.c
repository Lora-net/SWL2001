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
#include "stm32l4xx_hal.h"
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
static uint8_t copy_page[FLASH_PAGE_SIZE] = { 0x00 };
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

/**
 * @brief  Gets the bank of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The bank of a given address
 */
static uint32_t flash_get_bank( uint32_t Address );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
uint16_t hal_flash_get_page_size( )
{
    return FLASH_PAGE_SIZE;
}

uint8_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page )
{
    uint8_t  status                = SUCCESS;
    uint8_t  hal_status            = SUCCESS;
    uint32_t FirstUserPage         = 0;
    uint32_t bank_number           = 0;
    uint32_t PageError             = 0;
    uint8_t  flash_operation_retry = 0;

    FLASH_EraseInitTypeDef EraseInitStruct;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock( );

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG( FLASH_SR_OPTVERR );

    /* Get the 1st page to erase */
    FirstUserPage = flash_get_page( addr );

    /* Get the bank */
    bank_number = flash_get_bank( addr );

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = FirstUserPage;
    EraseInitStruct.NbPages   = nb_page;
    EraseInitStruct.Banks     = bank_number;

    // SMTC_HAL_TRACE_INFO( "Erase page %u bank %u\r\n", FirstUserPage, bank_number );

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
    do
    {
        hal_status = HAL_FLASHEx_Erase( &EraseInitStruct, &PageError );
        flash_operation_retry++;
    } while( ( hal_status != HAL_OK ) && ( flash_operation_retry < FLASH_OPERATION_MAX_RETRY ) );

    if( flash_operation_retry >= FLASH_OPERATION_MAX_RETRY )
    {
        /*
          Error occurred while  erase.
          User can add here some code to deal with this error.
          PageError will contain the faulty  and then to know the code error on this ,
          user can call function 'HAL_FLASH_GetError()'
        */
        SMTC_HAL_TRACE_ERROR( "FLASH_OPERATION_MAX_RETRY\r\n" );
        /* Infinite loop */
        while( 1 )
        {
        }
    }
    else
    {
        flash_operation_retry = 0;
    }

    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock( );

    return status;
}

uint32_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    uint8_t  status      = SUCCESS;
    uint8_t  hal_status  = SUCCESS;
    uint32_t BufferIndex = 0, real_size = 0, AddrEnd = 0;
    uint64_t data64                = 0;
    uint8_t  flash_operation_retry = 0;

    /* Complete size for FLASH_TYPEPROGRAM_DOUBLEWORD operation*/
    if( ( size % 8 ) != 0 )
    {
        real_size = size + ( 8 - ( size % 8 ) );
    }
    else
    {
        real_size = size;
    }

    AddrEnd = addr + real_size;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock( );

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG( FLASH_SR_OPTVERR );

    /* Don't draw outside the lines */
    if( AddrEnd > ( FLASH_USER_END_ADDR + 1 ) )
    {
        status = FAIL;
        return status;
    }

    /* Program the user Flash area word by word
    (area defined by FlashUserStartAddr and FLASH_USER_END_ADDR) ***********/

    while( addr < AddrEnd )
    {
        data64 = 0;
        for( uint8_t i = 0; i < 8; i++ )
        {
            data64 += ( ( ( uint64_t ) buffer[BufferIndex + i] ) << ( i * 8 ) );
        }

        do
        {
            hal_status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data64 );
            flash_operation_retry++;
        } while( ( hal_status != HAL_OK ) && ( flash_operation_retry < FLASH_OPERATION_MAX_RETRY ) );

        if( flash_operation_retry >= FLASH_OPERATION_MAX_RETRY )
        {
            /* Error occurred while writing data in Flash memory.
            User can add here some code to deal with this error */
            /* Infinite loop */
            while( 1 )
            {
            }
        }
        else
        {
            flash_operation_retry = 0;
            /* increment to next double word*/
            addr        = addr + 8;
            BufferIndex = BufferIndex + 8;
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock( );

    return real_size;
}

void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size )
{
    uint32_t     FlashIndex = 0;
    __IO uint8_t data8      = 0;

    while( FlashIndex < size )
    {
        data8 = *( __IO uint32_t* ) ( addr + FlashIndex );

        buffer[FlashIndex] = data8;

        FlashIndex++;
    }
}

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size )
{
    uint32_t remaining_size = size;
    do
    {
        memset( copy_page, 0xFF, sizeof( copy_page ) );

        // Get the page number
        uint32_t num_page = flash_get_page( addr );

        // Get start address of this NVM page
        uint32_t start_addr_page = FLASH_BASE + ( FLASH_PAGE_SIZE * num_page );

        // Read data on this NVM page
        hal_flash_read_buffer( start_addr_page, copy_page, FLASH_PAGE_SIZE );

        // Compute the index where data need to be updated in RAM copy page
        uint32_t index = ( addr - start_addr_page ) % FLASH_PAGE_SIZE;

        // Compute the size of the data need to be copied without overflow on the next page
        uint32_t cpy_size = MIN( remaining_size, FLASH_PAGE_SIZE - index );
        memcpy( &copy_page[index], &buffer[size - remaining_size], cpy_size );

        // Erase NVM page
        hal_flash_erase_page( addr, 1 );

        // Write the RAM buffer on this NVM page
        hal_flash_write_buffer( start_addr_page, copy_page, FLASH_PAGE_SIZE );

        // increment address to the next page
        addr += FLASH_PAGE_SIZE - index;

        // Reduce the amount of data remaining to be written
        remaining_size -= cpy_size;

    } while( remaining_size != 0 );
}
#endif  // USE_FLASH_READ_MODIFY_WRITE || MULTISTACK

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t flash_get_page( uint32_t Addr )
{
    return ( Addr - FLASH_BASE ) / FLASH_PAGE_SIZE;
}

/**
 * @brief  Gets the bank of a given address
 * @param  Addr: Address of the FLASH Memory
 * @retval The bank of a given address
 */
static uint32_t flash_get_bank( uint32_t Addr )
{
    uint32_t page = flash_get_page( Addr );
    if( page >= FLASH_PAGE_NUMBER )
    {
        SMTC_HAL_TRACE_ERROR( "Address out of range: 0x%X\r\n", Addr );
    }
    return FLASH_BANK_1 + ( page / FLASH_PAGE_PER_BANK );
}

/* --- EOF ------------------------------------------------------------------ */
