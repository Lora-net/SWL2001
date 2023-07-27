/**
 * @file      lr11xx_ranging.c
 *
 * @brief     Ranging driver implementation for LR11XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#include <stdint.h>
#include "lr11xx_ranging.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_RANGING_SET_ADDRESS_CMD_LENGTH ( 2 + 5 )
#define LR11XX_RANGING_SET_REQUEST_ADDRESS_CMD_LENGTH ( 2 + 4 )
#define LR11XX_RANGING_SET_RX_TX_DELAY_CMD_LENGTH ( 2 + 4 )
#define LR11XX_RANGING_SET_PARAMETERS_CMD_LENGTH ( 2 + 2 )
#define LR11XX_RANGING_GET_RESULT_CMD_LENGTH ( 2 + 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Operating codes for ranging-related operations
 */
enum
{
    LR11XX_RANGING_SET_ADDRESS         = 0x021C,
    LR11XX_RANGING_SET_REQUEST_ADDRESS = 0x021D,
    LR11XX_RANGING_SET_RX_TX_DELAY     = 0x021F,
    LR11XX_RANGING_SET_PARAMETERS      = 0x0228,
    LR11XX_RANGING_GET_RESULT          = 0x021E,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

lr11xx_status_t lr11xx_ranging_set_address( const void* context, const uint32_t address, const uint8_t check_length )
{
    const uint8_t cbuffer[LR11XX_RANGING_SET_ADDRESS_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_RANGING_SET_ADDRESS >> 8 ),
        ( uint8_t )( LR11XX_RANGING_SET_ADDRESS >> 0 ),
        ( uint8_t )( address >> 24 ),
        ( uint8_t )( address >> 16 ),
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
        check_length,
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RANGING_SET_ADDRESS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_ranging_set_request_address( const void* context, const uint32_t request_address )
{
    const uint8_t cbuffer[LR11XX_RANGING_SET_REQUEST_ADDRESS_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_RANGING_SET_REQUEST_ADDRESS >> 8 ),
        ( uint8_t )( LR11XX_RANGING_SET_REQUEST_ADDRESS >> 0 ),
        ( uint8_t )( request_address >> 24 ),
        ( uint8_t )( request_address >> 16 ),
        ( uint8_t )( request_address >> 8 ),
        ( uint8_t )( request_address >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RANGING_SET_REQUEST_ADDRESS_CMD_LENGTH, 0,
                                                 0 );
}

lr11xx_status_t lr11xx_ranging_set_rx_tx_delay_indicator( const void* context, const uint32_t delay_indicator )
{
    const uint8_t cbuffer[LR11XX_RANGING_SET_RX_TX_DELAY_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_RANGING_SET_RX_TX_DELAY >> 8 ),
        ( uint8_t )( LR11XX_RANGING_SET_RX_TX_DELAY >> 0 ),
        ( uint8_t )( delay_indicator >> 24 ),
        ( uint8_t )( delay_indicator >> 16 ),
        ( uint8_t )( delay_indicator >> 8 ),
        ( uint8_t )( delay_indicator >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RANGING_SET_RX_TX_DELAY_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_ranging_set_parameters( const void* context, const uint8_t nb_symbols )
{
    const uint8_t cbuffer[LR11XX_RANGING_SET_PARAMETERS_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_RANGING_SET_PARAMETERS >> 8 ), ( uint8_t )( LR11XX_RANGING_SET_PARAMETERS >> 0 ), 0x00,
        nb_symbols
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_RANGING_SET_PARAMETERS_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_ranging_get_raw_result( const void* context, const lr11xx_ranging_result_type_t type,
                                               uint8_t result[LR11XX_RANGING_RESULT_LENGTH] )
{
    const uint8_t cbuffer[LR11XX_RANGING_GET_RESULT_CMD_LENGTH] = {
        ( uint8_t )( LR11XX_RANGING_GET_RESULT >> 8 ),
        ( uint8_t )( LR11XX_RANGING_GET_RESULT >> 0 ),
        ( uint8_t ) type,
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_RANGING_GET_RESULT_CMD_LENGTH, result,
                                                LR11XX_RANGING_RESULT_LENGTH );
}

lr11xx_status_t lr11xx_ranging_get_recommended_rx_tx_delay_indicator( lr11xx_radio_lora_bw_t bw,
                                                                      lr11xx_radio_lora_sf_t sf,
                                                                      uint32_t*              delay_indicator )
{
    lr11xx_status_t status = LR11XX_STATUS_OK;

    *delay_indicator = 0u;

    if( bw == LR11XX_RADIO_LORA_BW_500 )
    {
        if( sf == LR11XX_RADIO_LORA_SF5 )
        {
            *delay_indicator = 20149u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF6 )
        {
            *delay_indicator = 20227u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF7 )
        {
            *delay_indicator = 20258u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF8 )
        {
            *delay_indicator = 20277u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF9 )
        {
            *delay_indicator = 20286u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF10 )
        {
            *delay_indicator = 20292u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF11 )
        {
            *delay_indicator = 20295u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            *delay_indicator = 20298u;
        }
        else
        {
            status = LR11XX_STATUS_ERROR;
        }
    }
    else if( bw == LR11XX_RADIO_LORA_BW_250 )
    {
        if( sf == LR11XX_RADIO_LORA_SF5 )
        {
            *delay_indicator = 20235u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF6 )
        {
            *delay_indicator = 20239u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF7 )
        {
            *delay_indicator = 20238u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF8 )
        {
            *delay_indicator = 20237u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF9 )
        {
            *delay_indicator = 20236u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF10 )
        {
            *delay_indicator = 20235u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF11 )
        {
            *delay_indicator = 20236u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            *delay_indicator = 20232u;
        }
        else
        {
            status = LR11XX_STATUS_ERROR;
        }
    }
    else if( bw == LR11XX_RADIO_LORA_BW_125 )
    {
        if( sf == LR11XX_RADIO_LORA_SF5 )
        {
            *delay_indicator = 19035u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF6 )
        {
            *delay_indicator = 19040u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF7 )
        {
            *delay_indicator = 19040u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF8 )
        {
            *delay_indicator = 19039u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF9 )
        {
            *delay_indicator = 19036u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF10 )
        {
            *delay_indicator = 19038u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF11 )
        {
            *delay_indicator = 19036u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            *delay_indicator = 19024u;
        }
        else
        {
            status = LR11XX_STATUS_ERROR;
        }
    }
    else
    {
        status = LR11XX_STATUS_ERROR;
    }
    return status;
}

lr11xx_status_t lr11xx_ranging_set_recommended_rx_tx_delay_indicator( const void* context, lr11xx_radio_lora_bw_t bw,
                                                                      lr11xx_radio_lora_sf_t sf )
{
    uint32_t        delay_indicator = 0;
    lr11xx_status_t status          = lr11xx_ranging_get_recommended_rx_tx_delay_indicator( bw, sf, &delay_indicator );
    if( status == LR11XX_STATUS_OK )
    {
        status = lr11xx_ranging_set_rx_tx_delay_indicator( context, delay_indicator );
    }
    return status;
}

int32_t lr11xx_ranging_distance_raw_to_meter( lr11xx_radio_lora_bw_t ranging_bw,
                                              const uint8_t          raw_distance_buf[LR11XX_RANGING_RESULT_LENGTH] )
{
    const uint8_t bitcnt     = 24u;
    uint8_t       bw_scaling = 0u;

    const uint32_t raw_distance =
        ( ( uint32_t ) raw_distance_buf[3] << 0 ) + ( ( uint32_t ) raw_distance_buf[2] << 8 ) +
        ( ( uint32_t ) raw_distance_buf[1] << 16 ) + ( ( uint32_t ) raw_distance_buf[0] << 24 );

    if( ranging_bw == LR11XX_RADIO_LORA_BW_500 )
    {
        bw_scaling = 1u;
    }
    else if( ranging_bw == LR11XX_RADIO_LORA_BW_250 )
    {
        bw_scaling = 2u;
    }
    else if( ranging_bw == LR11XX_RADIO_LORA_BW_125 )
    {
        bw_scaling = 4u;
    }

    int32_t retval = raw_distance;
    if( raw_distance >= ( 1 << ( bitcnt - 1 ) ) )
    {
        retval -= ( 1 << bitcnt );
    }

    return 300 * bw_scaling * retval / 4096;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
