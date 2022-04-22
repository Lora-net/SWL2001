/*!
 * @file      lr11xx_bootloader.c
 *
 * @brief     Bootloader driver implementation for LR11XX
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

#include "lr11xx_bootloader.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 ( 64 )
#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT8 ( LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 * 4 )

#define LR11XX_BL_CMD_NO_PARAM_LENGTH ( 2 )
#define LR11XX_BL_GET_STATUS_CMD_LENGTH ( 2 + 4 )
#define LR11XX_BL_VERSION_CMD_LENGTH LR11XX_BL_CMD_NO_PARAM_LENGTH
#define LR11XX_BL_ERASE_FLASH_CMD_LENGTH LR11XX_BL_CMD_NO_PARAM_LENGTH
#define LR11XX_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH + 4 )
#define LR11XX_BL_REBOOT_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH + 1 )
#define LR11XX_BL_GET_PIN_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH )
#define LR11XX_BL_READ_CHIP_EUI_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH )
#define LR11XX_BL_READ_JOIN_EUI_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for bootloader-related operations
 */
enum
{
    LR11XX_BL_GET_STATUS_OC            = 0x0100,
    LR11XX_BL_GET_VERSION_OC           = 0x0101,
    LR11XX_BL_ERASE_FLASH_OC           = 0x8000,
    LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC = 0x8003,
    LR11XX_BL_REBOOT_OC                = 0x8005,
    LR11XX_BL_GET_PIN_OC               = 0x800B,
    LR11XX_BL_READ_CHIP_EUI_OC         = 0x800C,
    LR11XX_BL_READ_JOIN_EUI_OC         = 0x800D,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Returns the minimum of the operand given as parameter and the maximum allowed block size
 *
 * @param [in] operand Size to compare
 *
 * @returns Minimum between operand and @ref LR11XX_FLASH_DATA_MAX_LENGTH_UINT32
 */
static uint8_t lr11xx_bootloader_get_min_from_operand_and_max_block_size( uint32_t operand );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_bootloader_get_status( const void* context, lr11xx_bootloader_stat1_t* stat1,
                                              lr11xx_bootloader_stat2_t*    stat2,
                                              lr11xx_bootloader_irq_mask_t* irq_status )
{
    uint8_t data[LR11XX_BL_GET_STATUS_CMD_LENGTH];

    const lr11xx_status_t status =
        ( lr11xx_status_t ) lr11xx_hal_direct_read( context, data, LR11XX_BL_GET_STATUS_CMD_LENGTH );

    if( status == LR11XX_STATUS_OK )
    {
        stat1->is_interrupt_active = ( ( data[0] & 0x01 ) != 0 ) ? true : false;
        stat1->command_status      = ( lr11xx_bootloader_command_status_t ) ( data[0] >> 1 );

        stat2->is_running_from_flash = ( ( data[1] & 0x01 ) != 0 ) ? true : false;
        stat2->chip_mode             = ( lr11xx_bootloader_chip_modes_t ) ( ( data[1] & 0x0F ) >> 1 );
        stat2->reset_status          = ( lr11xx_bootloader_reset_status_t ) ( ( data[1] & 0xF0 ) >> 4 );

        *irq_status =
            ( ( lr11xx_bootloader_irq_mask_t ) data[2] << 24 ) + ( ( lr11xx_bootloader_irq_mask_t ) data[3] << 16 ) +
            ( ( lr11xx_bootloader_irq_mask_t ) data[4] << 8 ) + ( ( lr11xx_bootloader_irq_mask_t ) data[5] << 0 );
    }

    return status;
}

lr11xx_status_t lr11xx_bootloader_clear_reset_status_info( const void* context )
{
    const uint8_t cbuffer[LR11XX_BL_CMD_NO_PARAM_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_GET_STATUS_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_GET_STATUS_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_CMD_NO_PARAM_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_bootloader_get_version( const void* context, lr11xx_bootloader_version_t* version )
{
    const uint8_t cbuffer[LR11XX_BL_VERSION_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_GET_VERSION_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_GET_VERSION_OC >> 0 ),
    };
    uint8_t rbuffer[LR11XX_BL_VERSION_LENGTH] = { 0x00 };

    const lr11xx_status_t status = ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_BL_VERSION_CMD_LENGTH,
                                                                        rbuffer, LR11XX_BL_VERSION_LENGTH );

    if( status == LR11XX_STATUS_OK )
    {
        version->hw   = rbuffer[0];
        version->type = rbuffer[1];
        version->fw   = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];
    }

    return status;
}

lr11xx_status_t lr11xx_bootloader_erase_flash( const void* context )
{
    const uint8_t cbuffer[LR11XX_BL_ERASE_FLASH_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_ERASE_FLASH_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_ERASE_FLASH_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_ERASE_FLASH_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_bootloader_write_flash_encrypted( const void* context, const uint32_t offset,
                                                         const uint32_t* data, uint8_t length )
{
    const uint8_t cbuffer[LR11XX_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC >> 0 ),
        ( uint8_t ) ( offset >> 24 ),
        ( uint8_t ) ( offset >> 16 ),
        ( uint8_t ) ( offset >> 8 ),
        ( uint8_t ) ( offset >> 0 ),
    };

    uint8_t cdata[256] = { 0 };
    for( uint8_t index = 0; index < length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t ) ( data[index] >> 24 );
        cdata_local[1] = ( uint8_t ) ( data[index] >> 16 );
        cdata_local[2] = ( uint8_t ) ( data[index] >> 8 );
        cdata_local[3] = ( uint8_t ) ( data[index] >> 0 );
    }

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH, cdata,
                                                 length * sizeof( uint32_t ) );
}

lr11xx_status_t lr11xx_bootloader_write_flash_encrypted_full( const void* context, const uint32_t offset,
                                                              const uint32_t* buffer, const uint32_t length )
{
    uint32_t remaining_length = length;
    uint32_t local_offset     = offset;
    uint32_t loop             = 0;

    while( remaining_length != 0 )
    {
        const lr11xx_status_t status = lr11xx_bootloader_write_flash_encrypted(
            context, local_offset, buffer + loop * LR11XX_FLASH_DATA_MAX_LENGTH_UINT32,
            lr11xx_bootloader_get_min_from_operand_and_max_block_size( remaining_length ) );

        if( status != LR11XX_STATUS_OK )
        {
            return status;
        }

        local_offset += LR11XX_FLASH_DATA_MAX_LENGTH_UINT8;
        remaining_length = ( remaining_length < LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 )
                               ? 0
                               : ( remaining_length - LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 );

        loop++;
    }

    return LR11XX_STATUS_OK;
}

lr11xx_status_t lr11xx_bootloader_reboot( const void* context, const bool stay_in_bootloader )
{
    const uint8_t cbuffer[LR11XX_BL_REBOOT_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_REBOOT_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_REBOOT_OC >> 0 ),
        ( stay_in_bootloader == true ) ? 0x03 : 0x00,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_REBOOT_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_bootloader_read_pin( const void* context, lr11xx_bootloader_pin_t pin )
{
    const uint8_t cbuffer[LR11XX_BL_GET_PIN_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_GET_PIN_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_GET_PIN_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_BL_GET_PIN_CMD_LENGTH, pin,
                                                LR11XX_BL_PIN_LENGTH );
}

lr11xx_status_t lr11xx_bootloader_read_chip_eui( const void* context, lr11xx_bootloader_chip_eui_t chip_eui )
{
    const uint8_t cbuffer[LR11XX_BL_READ_CHIP_EUI_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_READ_CHIP_EUI_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_READ_CHIP_EUI_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_BL_READ_CHIP_EUI_CMD_LENGTH, chip_eui,
                                                LR11XX_BL_CHIP_EUI_LENGTH );
}

lr11xx_status_t lr11xx_bootloader_read_join_eui( const void* context, lr11xx_bootloader_join_eui_t join_eui )
{
    const uint8_t cbuffer[LR11XX_BL_READ_JOIN_EUI_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_BL_READ_JOIN_EUI_OC >> 8 ),
        ( uint8_t ) ( LR11XX_BL_READ_JOIN_EUI_OC >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_BL_READ_JOIN_EUI_CMD_LENGTH, join_eui,
                                                LR11XX_BL_JOIN_EUI_LENGTH );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint8_t lr11xx_bootloader_get_min_from_operand_and_max_block_size( uint32_t operand )
{
    if( operand > LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 )
    {
        return LR11XX_FLASH_DATA_MAX_LENGTH_UINT32;
    }
    else
    {
        return ( uint8_t ) operand;
    }
}

/* --- EOF ------------------------------------------------------------------ */
