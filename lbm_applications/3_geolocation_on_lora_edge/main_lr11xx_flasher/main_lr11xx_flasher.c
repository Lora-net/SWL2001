/*!
 * \file      main_lr11xx_flasher.c
 *
 * \brief     main program for LR11xx flasher
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

#include "main.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"
#include "smtc_hal_watchdog.h"

#include "lr11xx_system.h"
#include "lr11xx_bootloader.h"
#include "lr11xx_hal.h"

#include "modem_pinout.h"

#include "lr11xx_fw.h"

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

static void lr1110_modem_hal_enter_dfu( const void* context );

/*
 * -----------------------------------------------------------------------------
 * --- LR11XX DRIVER EXTENSION FUNCTIONS DECLARATION ---------------------------
 */

static lr11xx_status_t lr11xx_bootloader_write_flash_full( const void* context, const uint32_t offset,
                                                           const uint32_t* buffer, const uint32_t length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_lr11xx_flasher( void )
{
    lr11xx_bootloader_version_t bootloader_version;
    lr11xx_system_version_t     lr11xx_fw_version;
    lr11xx_status_t             status;

    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "LR11xx flasher is starting \n" );

    SMTC_HAL_TRACE_PRINTF( "Enter DFU mode\n" );
    lr1110_modem_hal_enter_dfu( NULL );

    lr11xx_bootloader_get_version( NULL, &bootloader_version );
    SMTC_HAL_TRACE_PRINTF( "LR11xx: hw:0x%02X / type:0x%02X fw:0x%04X\n", bootloader_version.hw,
                           bootloader_version.type, bootloader_version.fw );

    SMTC_HAL_TRACE_PRINTF( "Erasing flash...\n" );
    lr11xx_bootloader_erase_flash( NULL );

    SMTC_HAL_TRACE_PRINTF( "Writing flash...\n" );
    if( bootloader_version.fw == 0 )
    {
        lr11xx_bootloader_write_flash_full( NULL, 0, lr11xx_firmware_image, LR11XX_FIRMWARE_IMAGE_SIZE );
    }
    else
    {
        lr11xx_bootloader_write_flash_encrypted_full( NULL, 0, lr11xx_firmware_image, LR11XX_FIRMWARE_IMAGE_SIZE );
    }

    SMTC_HAL_TRACE_PRINTF( "Resetting...\n" );
    lr11xx_hal_reset( NULL );

    status = lr11xx_system_get_version( NULL, &lr11xx_fw_version );
    if( status != LR11XX_STATUS_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get LR11XX firmware version\n" );
        mcu_panic( );
    }
    SMTC_HAL_TRACE_INFO( "LR11XX FW: 0x%04X\n", lr11xx_fw_version.fw );

    SMTC_HAL_TRACE_INFO( "###### DONE ######\n\n" );

    while( 1 )
    {
        hal_mcu_disable_irq( );
        hal_mcu_set_sleep_for_ms( 10000 );
        hal_watchdog_reload( );
        hal_mcu_enable_irq( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr1110_modem_hal_enter_dfu( const void* context )
{
    /* Force dio0 to 0 */
    hal_gpio_init_out( RADIO_BUSY_PIN, 0 );

    /* reset the chip */
    lr11xx_hal_reset( context );

    /* reinit dio0 */
    hal_gpio_init_in( RADIO_BUSY_PIN, BSP_GPIO_PULL_MODE_NONE, BSP_GPIO_IRQ_MODE_OFF, NULL );
}

/*
 * -----------------------------------------------------------------------------
 * --- LR11XX DRIVER EXTENSION FUNCTIONS DEFINITION (for development board (unencrypted)
 */

#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 ( 64 )
#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT8 ( LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 * 4 )

#define LR11XX_BL_CMD_NO_PARAM_LENGTH ( 2 )
#define LR11XX_BL_WRITE_FLASH_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH + 4 )

#define LR11XX_BL_WRITE_FLASH_OC 0x8002

static lr11xx_status_t lr11xx_bootloader_write_flash( const void* context, const uint32_t offset, const uint32_t* data,
                                                      uint8_t length )
{
    const uint8_t cbuffer[LR11XX_BL_WRITE_FLASH_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_BL_WRITE_FLASH_OC >> 8 ),
        ( uint8_t )( LR11XX_BL_WRITE_FLASH_OC >> 0 ),
        ( uint8_t )( offset >> 24 ),
        ( uint8_t )( offset >> 16 ),
        ( uint8_t )( offset >> 8 ),
        ( uint8_t )( offset >> 0 ),
    };

    uint8_t cdata[256] = { 0 };
    for( uint8_t index = 0; index < length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t )( data[index] >> 24 );
        cdata_local[1] = ( uint8_t )( data[index] >> 16 );
        cdata_local[2] = ( uint8_t )( data[index] >> 8 );
        cdata_local[3] = ( uint8_t )( data[index] >> 0 );
    }

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_WRITE_FLASH_CMD_LENGTH, cdata,
                                                 length * sizeof( uint32_t ) );
}

static uint8_t lr11xx_bootloader_get_min_from_operand_and_max_block_size( uint32_t operand )
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

static lr11xx_status_t lr11xx_bootloader_write_flash_full( const void* context, const uint32_t offset,
                                                           const uint32_t* buffer, const uint32_t length )
{
    uint32_t remaining_length = length;
    uint32_t local_offset     = offset;
    uint32_t loop             = 0;

    while( remaining_length != 0 )
    {
        const lr11xx_status_t status = lr11xx_bootloader_write_flash(
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

/* --- EOF ------------------------------------------------------------------ */
