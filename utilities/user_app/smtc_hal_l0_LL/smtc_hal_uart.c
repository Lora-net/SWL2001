/*!
 * \file      smtc_hal_uart.c
 *
 * \brief     UART Hardware Abstraction Layer implementation
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

#include "smtc_hal_uart.h"

#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_dma.h"

#include "modem_pinout.h"
#include "smtc_hal_mcu.h"

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

void uart2_init( void )
{
    // Configure DEBUG Uart RX/TX pins: PA2 and PA3
    // enable clock for gpio A port
    LL_IOP_GRP1_EnableClock( LL_IOP_GRP1_PERIPH_GPIOA );

    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE );
    LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH );
    LL_GPIO_SetAFPin_0_7( GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4 );
    // LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP );

    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE );
    LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH );
    LL_GPIO_SetAFPin_0_7( GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4 );
    // LL_GPIO_SetPinPull( GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP );

    // Enable USART2 clock
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART2 );

    // USART2 Init
    LL_USART_InitTypeDef usart_init = { 0 };

    usart_init.BaudRate            = 115200;
    usart_init.DataWidth           = LL_USART_DATAWIDTH_8B;
    usart_init.StopBits            = LL_USART_STOPBITS_1;
    usart_init.Parity              = LL_USART_PARITY_NONE;
    usart_init.TransferDirection   = LL_USART_DIRECTION_TX;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart_init.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init( USART2, &usart_init );
    LL_USART_ConfigAsyncMode( USART2 );
    LL_USART_Enable( USART2 );
    // Polling USART initialisation

    while( !( LL_USART_IsActiveFlag_TEACK( USART2 ) ) )
    {
    }
}

void uart2_deinit( void )
{
    // Disable USART2
    LL_USART_Disable( USART2 );

    // Disable USART2 clock
    LL_APB1_GRP1_DisableClock( LL_APB1_GRP1_PERIPH_USART2 );

    // De-init DEBUG Uart RX/TX pins: PA2 and PA3
    LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_LOW );
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG );

    LL_GPIO_SetPinSpeed( GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH );
    LL_GPIO_SetPinMode( GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ANALOG );
}

void uart2_tx( uint8_t* buff, uint8_t len )
{
    uint8_t index = 0;

    // Send characters one per one, until last char to be sent
    while( index < len )
    {
        // Wait for TXE (Transmit Data Register Empty) flag to be raised
        while( !LL_USART_IsActiveFlag_TXE( USART2 ) )
        {
        }
        // If last char to be sent, clear TC (Transmission Complete) flag
        if( index == ( len - 1 ) )
        {
            LL_USART_ClearFlag_TC( USART2 );
        }
        // Write character in Transmit Data register.
        // TXE flag is cleared by writing data in TDR register
        LL_USART_TransmitData8( USART2, buff[index++] );
    }

    // Wait for TC flag to be raised for last char
    while( !LL_USART_IsActiveFlag_TC( USART2 ) )
    {
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
