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
#include "stm32u5xx_hal.h"

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

static DMA_HandleTypeDef hdma_usart3_rx;

// UART1 is connected to ST-link on Nucleo U575ZI
static UART_HandleTypeDef huart1;

// UART3 is available on Nucleo U575ZI on CN10 PB10 (tx) PB11 (rx)
static UART_HandleTypeDef huart3;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void uart3_init( void )
{
    __HAL_RCC_GPDMA1_CLK_ENABLE( );
    HAL_NVIC_SetPriority( GPDMA1_Channel0_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( GPDMA1_Channel0_IRQn );

    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 115200;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if( HAL_UART_Init( &huart3 ) != HAL_OK )
    {
        mcu_panic( );
    }
}

void uart3_deinit( void )
{
    HAL_UART_DeInit( &huart3 );
}

void uart1_init( void )
{
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if( HAL_UART_Init( &huart1 ) != HAL_OK )
    {
        mcu_panic( );
    }
}

void uart1_deinit( void )
{
    HAL_UART_DeInit( &huart1 );
}

void uart3_dma_start_rx( uint8_t* buff, uint16_t size )
{
    HAL_UART_DMAStop( &huart3 );
    HAL_UART_Receive_DMA( &huart3, buff, size );
}

void uart3_dma_stop_rx( void )
{
    HAL_UART_DMAStop( &huart3 );
}

void uart3_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart3, ( uint8_t* ) buff, len, 0xffffff );
}

void uart1_tx( uint8_t* buff, uint8_t len )
{
    HAL_UART_Transmit( &huart1, ( uint8_t* ) buff, len, 0xffffff );
}

void HAL_UART_MspInit( UART_HandleTypeDef* huart )
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if( huart->Instance == USART3 )
    {
        __HAL_RCC_USART3_CLK_ENABLE( );
        __HAL_RCC_GPIOB_CLK_ENABLE( );

        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        GPIO_InitStruct.Pin       = ( 1 << ( HW_MODEM_RX_LINE & 0x0F ) ) | ( 1 << ( HW_MODEM_TX_LINE & 0x0F ) );
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

        hdma_usart3_rx.Instance                   = GPDMA1_Channel1;
        hdma_usart3_rx.Init.Request               = GPDMA1_REQUEST_USART3_RX;
        hdma_usart3_rx.Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
        hdma_usart3_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.SrcInc                = DMA_SINC_FIXED;
        hdma_usart3_rx.Init.DestInc               = DMA_DINC_INCREMENTED;
        hdma_usart3_rx.Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
        hdma_usart3_rx.Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
        hdma_usart3_rx.Init.Priority              = DMA_HIGH_PRIORITY;
        hdma_usart3_rx.Init.SrcBurstLength        = 1;
        hdma_usart3_rx.Init.DestBurstLength       = 1;
        hdma_usart3_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
        hdma_usart3_rx.Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
        hdma_usart3_rx.Init.Mode                  = DMA_NORMAL;

        if( HAL_DMA_Init( &hdma_usart3_rx ) != HAL_OK )
        {
            mcu_panic( );
        }
        __HAL_LINKDMA( huart, hdmarx, hdma_usart3_rx );

        if( HAL_DMA_ConfigChannelAttributes( &hdma_usart3_rx, DMA_CHANNEL_NPRIV ) != HAL_OK )
        {
            mcu_panic( );
        }
        HAL_NVIC_SetPriority( USART3_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( USART3_IRQn );
    }
    else if( huart->Instance == USART1 )
    {
        __HAL_RCC_USART1_CLK_ENABLE( );

        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
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
    if( huart->Instance == USART3 )
    {
        __HAL_RCC_USART3_CLK_DISABLE( );
        HAL_GPIO_DeInit( GPIOB, ( 1 << ( HW_MODEM_TX_LINE & 0x0F ) ) );
        HAL_GPIO_DeInit( GPIOB, ( 1 << ( HW_MODEM_RX_LINE & 0x0F ) ) );

        HAL_DMA_DeInit( &hdma_usart3_rx );

        __HAL_RCC_GPDMA1_CLK_DISABLE( );
    }
    if( huart->Instance == USART1 )
    {
        __HAL_RCC_USART1_CLK_DISABLE( );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( DEBUG_UART_TX & 0x0F ) ) );
        HAL_GPIO_DeInit( GPIOA, ( 1 << ( DEBUG_UART_RX & 0x0F ) ) );
    }
}

void DMA2_Channel5_IRQHandler( void )
{
    HAL_DMA_IRQHandler( &hdma_usart3_rx );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
