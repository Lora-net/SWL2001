/*!
 * \file      sx126x_hal.c
 *
 * \brief     Implements the sx126x radio HAL functions
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

#include "sx126x_hal.h"

#include "smtc_hal_gpio.h"
#include "smtc_hal_spi.h"
#include "smtc_hal_mcu.h"
#include "modem_pinout.h"

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

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_sleep_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

// This variable will hold the current sleep status of the radio
static radio_sleep_mode_t radio_mode = RADIO_AWAKE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Wait until radio busy pin returns to 0
 */
static void sx126x_hal_wait_on_busy( void );

/**
 * @brief Check if device is ready to receive spi transaction.
 * @remark If the device is in sleep mode, it will awake it and wait until it is ready
 */
static void sx126x_hal_check_device_ready( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    sx126x_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }
    for( uint16_t i = 0; i < data_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, data[i] );
    }
    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    // 0x84 - SX126x_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1 => do not test it
    if( command[0] != 0x84 )
    {
        sx126x_hal_check_device_ready( );
    }
    else
    {
        radio_mode = RADIO_SLEEP;
    }

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    sx126x_hal_check_device_ready( );

    // Put NSS low to start spi transaction
    hal_gpio_set_value( RADIO_NSS, 0 );
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( RADIO_SPI_ID, command[i] );
    }
    for( uint16_t i = 0; i < data_length; i++ )
    {
        data[i] = hal_spi_in_out( RADIO_SPI_ID, 0 );
    }
    // Put NSS high as the spi transaction is finished
    hal_gpio_set_value( RADIO_NSS, 1 );

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset( const void* context )
{
    hal_gpio_set_value( RADIO_NRST, 0 );
    hal_mcu_wait_us( 5000 );
    hal_gpio_set_value( RADIO_NRST, 1 );
    hal_mcu_wait_us( 5000 );
    radio_mode = RADIO_AWAKE;
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
    sx126x_hal_check_device_ready( );
    return SX126X_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void sx126x_hal_wait_on_busy( void )
{
    while( hal_gpio_get_value( RADIO_BUSY_PIN ) == 1 )
    {
    };
}

static void sx126x_hal_check_device_ready( void )
{
    if( radio_mode != RADIO_SLEEP )
    {
        sx126x_hal_wait_on_busy( );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device
        hal_gpio_set_value( RADIO_NSS, 0 );
        sx126x_hal_wait_on_busy( );
        hal_gpio_set_value( RADIO_NSS, 1 );
        radio_mode = RADIO_AWAKE;
    }
}