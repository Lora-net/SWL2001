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

void trace_uart_init( void )
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

void trace_uart_deinit( void )
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

void trace_uart_tx( uint8_t* buff, uint8_t len )
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

void hw_modem_uart_init( void )
{
    // Configure Modem RX/TX pins: PC11 and PC10
    // enable clock for gpio C port
    LL_IOP_GRP1_EnableClock( LL_IOP_GRP1_PERIPH_GPIOC );

    LL_GPIO_SetPinSpeed( GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH );
    LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE );
    LL_GPIO_SetAFPin_8_15( GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_6 );
    LL_GPIO_SetPinOutputType( GPIOC, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_PUSHPULL );
    LL_GPIO_SetPinPull( GPIOC, LL_GPIO_PIN_9, LL_GPIO_PULL_UP );

    LL_GPIO_SetPinSpeed( GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH );
    LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE );
    LL_GPIO_SetAFPin_8_15( GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_6 );
    LL_GPIO_SetPinOutputType( GPIOC, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL );
    LL_GPIO_SetPinPull( GPIOC, LL_GPIO_PIN_10, LL_GPIO_PULL_UP );

    // Enable DMA clock
    LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_DMA1 );

    // Init DMA interrupt
    NVIC_SetPriority( DMA1_Channel2_3_IRQn, 0 );
    NVIC_EnableIRQ( DMA1_Channel2_3_IRQn );

    // USART4 DMA Init
    LL_DMA_SetPeriphRequest( DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_12 );
    LL_DMA_SetDataTransferDirection( DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY );
    LL_DMA_SetChannelPriorityLevel( DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH );
    LL_DMA_SetMode( DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL );
    LL_DMA_SetPeriphIncMode( DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT );
    LL_DMA_SetMemoryIncMode( DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT );
    LL_DMA_SetPeriphSize( DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE );
    LL_DMA_SetMemorySize( DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE );

    // Enable USART4 clock
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART4 );

    // USART4 Init
    LL_USART_InitTypeDef usart_init = { 0 };

    usart_init.BaudRate            = 115200;
    usart_init.DataWidth           = LL_USART_DATAWIDTH_8B;
    usart_init.StopBits            = LL_USART_STOPBITS_1;
    usart_init.Parity              = LL_USART_PARITY_NONE;
    usart_init.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart_init.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init( USART4, &usart_init );
    LL_USART_ConfigAsyncMode( USART4 );
    LL_USART_Enable( USART4 );

    // Polling USART initialisation
    while( ( !( LL_USART_IsActiveFlag_TEACK( USART4 ) ) ) || ( !( LL_USART_IsActiveFlag_REACK( USART4 ) ) ) )
    {
    }
}

void hw_modem_uart_deinit( void )
{
    // Disable USART4
    LL_USART_Disable( USART4 );

    // Disable USART4 clock
    LL_APB1_GRP1_DisableClock( LL_APB1_GRP1_PERIPH_USART4 );

    // De-init Modem RX/TX pins: PC11 and PC10
    LL_GPIO_SetPinSpeed( GPIOC, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_LOW );
    LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ANALOG );

    LL_GPIO_SetPinSpeed( GPIOC, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_LOW );
    LL_GPIO_SetPinMode( GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ANALOG );

    // De init DMA1  LL_DMA_CHANNEL_2
    LL_DMA_DeInit( DMA1, LL_DMA_CHANNEL_2 );
}

void hw_modem_uart_dma_start_rx( uint8_t* buff, uint16_t size )
{
    // First stop and disable any on going transfert
    LL_DMA_DisableIT_TC( DMA1, LL_DMA_CHANNEL_2 );
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_2 );
    LL_DMA_ClearFlag_GI2( DMA1 );

    LL_DMA_ConfigAddresses( DMA1, LL_DMA_CHANNEL_2, LL_USART_DMA_GetRegAddr( USART4, LL_USART_DMA_REG_DATA_RECEIVE ),
                            ( uint32_t ) buff, LL_DMA_GetDataTransferDirection( DMA1, LL_DMA_CHANNEL_2 ) );
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_2, size );

    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_2 );

    LL_USART_EnableDMAReq_RX( USART4 );
}

void hw_modem_uart_dma_stop_rx( void )
{
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_2 );
}

void hw_modem_uart_tx( uint8_t* buff, uint8_t len )
{
    uint8_t index = 0;

    // Send characters one per one, until last char to be sent
    while( index < len )
    {
        // Wait for TXE (Transmit Data Register Empty) flag to be raised
        while( !LL_USART_IsActiveFlag_TXE( USART4 ) )
        {
        }
        // If last char to be sent, clear TC (Transmission Complete) flag
        if( index == ( len - 1 ) )
        {
            LL_USART_ClearFlag_TC( USART4 );
        }
        // Write character in Transmit Data register.
        // TXE flag is cleared by writing data in TDR register
        LL_USART_TransmitData8( USART4, buff[index++] );
    }

    // Wait for TC flag to be raised for last char
    while( !LL_USART_IsActiveFlag_TC( USART4 ) )
    {
    }
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
