/*!
 * @file      lr20xx_pram.c
 *
 * @brief     Implementation of patching commands for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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

#include <stdbool.h>  // true, false

#include "lr20xx_pram.h"
#include "lr20xx_regmem.h"
#include "lr20xx_hal.h"

#include "lr20xx_pram_lr2021.h"
#include "lr20xx_pram_lr20x2.h"
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_PRAM_MAGIC_WORD_ADDRESS ( 0x800FF8 )
#define LR20XX_PRAM_TYPE_VERSION_ADDRESS ( 0x800FFC )
#define LR20XX_PRAM_MAGIC_WORD_EXPECTED ( 0x600DB002 )
#define LR20XX_PRAM_BASE_ADDRESS ( 0x801000 )

#define LR20XX_PRAM_ENABLE_PRAM_CMD_LENGTH ( 2 + 1 )

#define LR20XX_PRAM_BASE_ADDRESS ( 0x801000 )

#define LR20XX_PRAM_CHECK_VERSION_MASK 0x0000FF00
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for patch RAM related operations
 */
enum
{
    LR20XX_PRAM_ENABLE_PRAM_OC = 0x012D,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Load and enable a PRAM
 *
 * This function can be called to integrate and enable PRAM.
 *
 * @param context Chip implementation context
 * @param buffer Pointer to the buffer being the PRAM data to transfer. It is up to the caller to ensure it contains at
 * least @p length elements
 * @param length Number of elements fro @p buffer array to write
 * @return lr20xx_status_t Operation status
 *
 * @see  lr20xx_pram_load_pram_lr2021, lr20xx_pram_load_pram_lr20x2
 */
static lr20xx_status_t lr20xx_pram_load_and_enable( const void* context, const uint32_t* buffer,
                                                    const uint32_t length );

/**
 * @brief Check Patch RAM (PRAM) version integrate in chip
 *
 * This function can be called after @ref lr20xx_pram_load_pram to check that PRAM has been correctly loaded and
 * the version is the same compared to the software.
 *
 * @param context Chip implementation context
 * @param buffer Pointer to the buffer being the PRAM.
 * @return lr20xx_status_t Operation status
 *
 * @see  lr20xx_pram_get_version
 */
static lr20xx_status_t lr20xx_pram_check_version( const void* context, const uint32_t* pram_buffer );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_status_t lr20xx_pram_get_version( const void* context, lr20xx_patch_version_t* pram_version )
{
    // 1. Read the magic word register
    uint32_t              magic_word_read = 0;
    const lr20xx_status_t magic_word_read_status =
        lr20xx_regmem_read_regmem32( context, LR20XX_PRAM_MAGIC_WORD_ADDRESS, &magic_word_read, 1 );
    if( magic_word_read_status == LR20XX_STATUS_OK )
    {
        // 2. If magic word register content match expectation, a PRAM is loaded
        if( magic_word_read == LR20XX_PRAM_MAGIC_WORD_EXPECTED )
        {
            // 3. Read the PRAM type and version register
            uint32_t              pram_type_version_raw = 0;
            const lr20xx_status_t pram_type_version_read_status =
                lr20xx_regmem_read_regmem32( context, LR20XX_PRAM_TYPE_VERSION_ADDRESS, &pram_type_version_raw, 1 );
            if( pram_type_version_read_status == LR20XX_STATUS_OK )
            {
                pram_version->is_pram_loaded = true;
                pram_version->pram_type      = ( uint8_t ) ( pram_type_version_raw >> 16 );
                pram_version->pram_version   = ( uint8_t ) ( pram_type_version_raw >> 8 );
            }
            return pram_type_version_read_status;
        }
        else
        {
            pram_version->is_pram_loaded = false;
            pram_version->pram_type      = 0;
            pram_version->pram_version   = 0;
        }
    }
    return magic_word_read_status;
}

lr20xx_status_t lr20xx_pram_load_pram( const void* context, const uint32_t address, const uint32_t* buffer,
                                       const uint32_t length )
{
    // The pram is written by blocks of 32 words
    const uint32_t n_32_word_blocks = length / 32u;

    // Write all blocks of 32 words
    for( uint32_t index_32_word_block = 0; index_32_word_block < n_32_word_blocks; index_32_word_block++ )
    {
        const uint32_t*       local_buffer  = buffer + ( index_32_word_block * 32u );
        const uint32_t        local_address = address + ( index_32_word_block * 32u * 4u );
        const lr20xx_status_t status        = lr20xx_regmem_write_regmem32( context, local_address, local_buffer, 32u );
        if( status != LR20XX_STATUS_OK )
        {
            return status;
        }
    }

    // Check if there are remaining words to write and if so, write it in a single call
    const uint8_t n_remaining_words = ( uint8_t ) ( length - ( n_32_word_blocks * 32u ) );
    if( n_remaining_words > 0 )
    {
        const lr20xx_status_t status =
            lr20xx_regmem_write_regmem32( context, address + ( n_32_word_blocks * 32u * 4u ),
                                          buffer + ( n_32_word_blocks * 32u ), n_remaining_words );
        return status;
    }
    else
    {
        return LR20XX_STATUS_OK;
    }
}

lr20xx_status_t lr20xx_pram_enable_pram( const void* context )
{
    const uint8_t cbuffer[LR20XX_PRAM_ENABLE_PRAM_CMD_LENGTH] = {
        ( uint8_t ) ( LR20XX_PRAM_ENABLE_PRAM_OC >> 8 ),
        ( uint8_t ) ( LR20XX_PRAM_ENABLE_PRAM_OC >> 0 ),
        0,
    };

    return ( lr20xx_status_t ) lr20xx_hal_write( context, cbuffer, LR20XX_PRAM_ENABLE_PRAM_CMD_LENGTH, 0, 0 );
}

lr20xx_status_t lr20xx_pram_load_pram_lr2021( const void* context )
{
    return lr20xx_pram_load_and_enable( context, pram_lr2021, pram_lr2021_size );
}

lr20xx_status_t lr20xx_pram_load_pram_lr20x2( const void* context )
{
    return lr20xx_pram_load_and_enable( context, pram_lr20x2, pram_lr20x2_size );
}

lr20xx_status_t lr20xx_pram_check_pram_lr2021( const void* context )
{
    return lr20xx_pram_check_version( context, pram_lr2021 );
}

lr20xx_status_t lr20xx_pram_check_pram_lr20x2( const void* context )
{
    return lr20xx_pram_check_version( context, pram_lr20x2 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr20xx_status_t lr20xx_pram_check_version( const void* context, const uint32_t* pram_buffer )
{
    lr20xx_patch_version_t pram_version;
    RETURN_STATUS_ON_NOT_OK( lr20xx_pram_get_version( context, &pram_version ) );
    if( ( ( pram_buffer[1] & LR20XX_PRAM_CHECK_VERSION_MASK ) >> 8 ) != pram_version.pram_version )
    {
        return LR20XX_STATUS_ERROR;
    }

    return LR20XX_STATUS_OK;
}

static lr20xx_status_t lr20xx_pram_load_and_enable( const void* context, const uint32_t* buffer, const uint32_t length )
{
    RETURN_STATUS_ON_NOT_OK( lr20xx_pram_load_pram( context, LR20XX_PRAM_BASE_ADDRESS, buffer, length ) );
    return lr20xx_pram_enable_pram( context );
}

/* --- EOF ------------------------------------------------------------------ */
