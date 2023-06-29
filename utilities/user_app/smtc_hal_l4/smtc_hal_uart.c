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
#include "stm32l4xx_hal.h"

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

static DMA_HandleTypeDef hdma_usart4_rx;

static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart4;

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
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if( HAL_UART_Init( &huart2 ) != HAL_OK )
    {
        mcu_panic( );
    }
}

void uart2_deinit( void )
{
    HAL_UART_DeInit( &huart2 );
}

void uart2_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart2, ( uint8_t* ) buff, len, 0xffffff );
}

void HAL_UART_MspInit( UART_HandleTypeDef* huart )
{
    GPIO_InitTypeDef GPIO_InitStruct;

        if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_CLK_ENABLE( );

        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        GPIO_InitStruct.Pin       = ( 1 << ( DEBUG_UART_TX & 0x0F ) ) | ( 1 << ( DEBUG_UART_RX & 0x0F ) );
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

        __HAL_RCC_GPIOA_CLK_ENABLE( );
        HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );
    }
    else
    {
        mcu_panic( );
    }
}

void HAL_UART_MspDeInit( UART_HandleTypeDef* huart )
{
    if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_CLK_DISABLE( );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( DEBUG_UART_TX & 0x0F ) ) );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( DEBUG_UART_RX & 0x0F ) ) );
    }
}

void DMA2_Channel5_IRQHandler( void )
{
    HAL_DMA_IRQHandler( &hdma_usart4_rx );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
