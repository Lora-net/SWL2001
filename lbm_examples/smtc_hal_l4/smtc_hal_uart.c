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
#include "smtc_hal_gpio.h"

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

void hw_modem_uart_init( void )
{
    __HAL_RCC_DMA2_CLK_ENABLE( );
    HAL_NVIC_SetPriority( DMA2_Channel5_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( DMA2_Channel5_IRQn );

    huart4.Instance                    = UART4;
    huart4.Init.BaudRate               = 115200;
    huart4.Init.WordLength             = UART_WORDLENGTH_8B;
    huart4.Init.StopBits               = UART_STOPBITS_1;
    huart4.Init.Parity                 = UART_PARITY_NONE;
    huart4.Init.Mode                   = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if( HAL_UART_Init( &huart4 ) != HAL_OK )
    {
        mcu_panic( );
    }
}

void hw_modem_uart_deinit( void )
{
    HAL_UART_DeInit( &huart4 );
}

void trace_uart_init( void )
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

void trace_uart_deinit( void )
{
    HAL_UART_DeInit( &huart2 );
}

void hw_modem_uart_dma_start_rx( uint8_t* buff, uint16_t size )
{
    HAL_UART_DMAStop( &huart4 );
    HAL_UART_Receive_DMA( &huart4, buff, size );
}

void hw_modem_uart_dma_stop_rx( void )
{
    HAL_UART_DMAStop( &huart4 );
}

void hw_modem_uart_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart4, ( uint8_t* ) buff, len, 0xffffff );
}

void trace_uart_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart2, ( uint8_t* ) buff, len, 0xffffff );
}

void HAL_UART_MspInit( UART_HandleTypeDef* huart )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if( huart->Instance == UART4 )
    {
        __HAL_RCC_UART4_CLK_ENABLE( );
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

        hal_gpio_enable_clock( HW_MODEM_RX_LINE );
        GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( HW_MODEM_RX_LINE & 0xF0 ) << 6 ) );
        GPIO_InitStruct.Pin     = ( 1 << ( HW_MODEM_RX_LINE & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &GPIO_InitStruct );

        hal_gpio_enable_clock( HW_MODEM_TX_LINE );
        gpio_port           = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( HW_MODEM_TX_LINE & 0xF0 ) << 6 ) );
        GPIO_InitStruct.Pin = ( 1 << ( HW_MODEM_TX_LINE & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &GPIO_InitStruct );

        hdma_usart4_rx.Instance                 = DMA2_Channel5;
        hdma_usart4_rx.Init.Request             = DMA_REQUEST_2;
        hdma_usart4_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart4_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart4_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart4_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart4_rx.Init.Mode                = DMA_NORMAL;
        hdma_usart4_rx.Init.Priority            = DMA_PRIORITY_LOW;

        if( HAL_DMA_Init( &hdma_usart4_rx ) != HAL_OK )
        {
            mcu_panic( );
        }
        __HAL_LINKDMA( huart, hdmarx, hdma_usart4_rx );
    }
    else if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_CLK_ENABLE( );

        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

        hal_gpio_enable_clock( DEBUG_UART_TX );
        GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( DEBUG_UART_TX & 0xF0 ) << 6 ) );
        GPIO_InitStruct.Pin     = ( 1 << ( DEBUG_UART_TX & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &GPIO_InitStruct );

        hal_gpio_enable_clock( DEBUG_UART_RX );
        gpio_port           = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( DEBUG_UART_RX & 0xF0 ) << 6 ) );
        GPIO_InitStruct.Pin = ( 1 << ( DEBUG_UART_RX & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &GPIO_InitStruct );
    }
    else
    {
        mcu_panic( );
    }
}

void HAL_UART_MspDeInit( UART_HandleTypeDef* huart )
{
    if( huart->Instance == UART4 )
    {
        __HAL_RCC_UART4_CLK_DISABLE( );

        GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( HW_MODEM_TX_LINE & 0xF0 ) << 6 ) );
        HAL_GPIO_DeInit( gpio_port, ( 1 << ( HW_MODEM_TX_LINE & 0x0F ) ) );
        gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( HW_MODEM_RX_LINE & 0xF0 ) << 6 ) );
        HAL_GPIO_DeInit( gpio_port, ( 1 << ( HW_MODEM_RX_LINE & 0x0F ) ) );

        HAL_DMA_DeInit( &hdma_usart4_rx );

        __HAL_RCC_DMA2_CLK_DISABLE( );
    }
    if( huart->Instance == USART2 )
    {
        __HAL_RCC_USART2_CLK_DISABLE( );

        GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( DEBUG_UART_TX & 0xF0 ) << 6 ) );
        HAL_GPIO_DeInit( gpio_port, ( 1 << ( DEBUG_UART_TX & 0x0F ) ) );
        gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( DEBUG_UART_RX & 0xF0 ) << 6 ) );
        HAL_GPIO_DeInit( gpio_port, ( 1 << ( DEBUG_UART_RX & 0x0F ) ) );
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
