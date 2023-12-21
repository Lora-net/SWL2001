/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
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

#include "smtc_hal_gpio.h"

#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"

#include "smtc_hal_dbg_trace.h"

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

/*!
 * Array holding attached IRQ gpio data context
 */
static hal_gpio_irq_t const* gpio_irq[16];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void config_exti_line( GPIO_TypeDef* port, uint8_t line );

static void call_gpio_irq_cb( uint8_t index );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

//
// MCU input pin Handling
//
void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq )
{
    const uint32_t pulls[] = { LL_GPIO_PULL_NO, LL_GPIO_PULL_UP, LL_GPIO_PULL_DOWN };

    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    // enable gpio port clock
    LL_IOP_GRP1_EnableClock( 0x1UL << ( ( pin & 0xF0 ) >> 4 ) );

    // Init gpio
    LL_GPIO_SetPinPull( gpio_port, ( 1 << ( pin & 0x0F ) ), pulls[pull_mode] );
    LL_GPIO_SetPinMode( gpio_port, ( 1 << ( pin & 0x0F ) ), LL_GPIO_MODE_INPUT );

    // Init Exti of needed
    if( irq_mode != BSP_GPIO_IRQ_MODE_OFF )
    {
        uint32_t exti_line = 0x1UL << ( pin & 0x0F );

        // Connect External Line to the GPIO
        config_exti_line( gpio_port, pin & 0x0F );

        // Disable Event and enable IT on exti line
        LL_EXTI_DisableEvent_0_31( exti_line );
        LL_EXTI_EnableIT_0_31( exti_line );

        // Configure irq mode
        switch( irq_mode )
        {
        case BSP_GPIO_IRQ_MODE_RISING:
            LL_EXTI_DisableFallingTrig_0_31( exti_line );
            LL_EXTI_EnableRisingTrig_0_31( exti_line );
            break;
        case BSP_GPIO_IRQ_MODE_FALLING:
            LL_EXTI_DisableRisingTrig_0_31( exti_line );
            LL_EXTI_EnableFallingTrig_0_31( exti_line );

            break;
        case BSP_GPIO_IRQ_MODE_RISING_FALLING:
            LL_EXTI_EnableRisingTrig_0_31( exti_line );
            LL_EXTI_EnableFallingTrig_0_31( exti_line );
            break;
        default:
            // never reached
            break;
        }

        // Attach callback to local tab
        hal_gpio_irq_attach( irq );

        // Configure NVIC for corresponding EXTI irq line
        switch( pin & 0x0F )
        {
        case 0:
        case 1:
            NVIC_SetPriority( EXTI0_1_IRQn, 0 );
            NVIC_EnableIRQ( EXTI0_1_IRQn );
            break;
        case 2:
        case 3:
            NVIC_SetPriority( EXTI2_3_IRQn, 0 );
            NVIC_EnableIRQ( EXTI2_3_IRQn );
            break;
        default:
            NVIC_SetPriority( EXTI4_15_IRQn, 0 );
            NVIC_EnableIRQ( EXTI4_15_IRQn );
            break;
        }
    }
}

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    // enable gpio port clock
    LL_IOP_GRP1_EnableClock( 0x1UL << ( ( pin & 0xF0 ) >> 4 ) );

    // Set init value
    if( value == 0 )
    {
        LL_GPIO_ResetOutputPin( gpio_port, 1 << ( pin & 0x0F ) );
    }
    else  // set io to high
    {
        LL_GPIO_SetOutputPin( gpio_port, 1 << ( pin & 0x0F ) );
    }

    LL_GPIO_SetPinSpeed( gpio_port, 1 << ( pin & 0x0F ), LL_GPIO_SPEED_FREQ_HIGH );
    LL_GPIO_SetPinOutputType( gpio_port, 1 << ( pin & 0x0F ), LL_GPIO_OUTPUT_PUSHPULL );
    LL_GPIO_SetPinPull( gpio_port, 1 << ( pin & 0x0F ), LL_GPIO_PULL_NO );
    LL_GPIO_SetPinMode( gpio_port, 1 << ( pin & 0x0F ), LL_GPIO_MODE_OUTPUT );
}

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    if( ( irq != NULL ) && ( irq->callback != NULL ) )
    {
        gpio_irq[( irq->pin ) & 0x0F] = irq;
    }
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
    if( irq != NULL )
    {
        gpio_irq[( irq->pin ) & 0x0F] = NULL;
    }
}

void hal_gpio_irq_enable( void )
{
    NVIC_EnableIRQ( EXTI0_1_IRQn );
    NVIC_EnableIRQ( EXTI2_3_IRQn );
    NVIC_EnableIRQ( EXTI4_15_IRQn );
}

void hal_gpio_irq_disable( void )
{
    NVIC_DisableIRQ( EXTI0_1_IRQn );
    NVIC_DisableIRQ( EXTI2_3_IRQn );
    NVIC_DisableIRQ( EXTI4_15_IRQn );
}

//
// MCU pin state control
//

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );
    if( value == 0 )
    {
        LL_GPIO_ResetOutputPin( gpio_port, 1 << ( pin & 0x0F ) );
    }
    else  // set io to high
    {
        LL_GPIO_SetOutputPin( gpio_port, 1 << ( pin & 0x0F ) );
    }
}
uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( IOPPERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );
    return LL_GPIO_IsInputPinSet( gpio_port, 1 << ( pin & 0x0F ) );
}

void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin )
{
    switch( pin & 0x0F )
    {
    case 0:
    case 1:
        NVIC_ClearPendingIRQ( EXTI0_1_IRQn );
        break;
    case 2:
    case 3:
        NVIC_ClearPendingIRQ( EXTI2_3_IRQn );
        break;
    default:
        NVIC_ClearPendingIRQ( EXTI4_15_IRQn );
        break;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void config_exti_line( GPIO_TypeDef* port, uint8_t line )
{
    LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_SYSCFG );

    uint32_t sysconf_exti_line = ( uint32_t )( ( uint32_t )( ( 4 * line ) % 16 ) << 16U | ( uint32_t )( line >> 2 ) );

    if( port == GPIOA )
    {
        LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTA, sysconf_exti_line );
    }
    else if( port == GPIOB )
    {
        LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTB, sysconf_exti_line );
    }
    else if( port == GPIOC )
    {
        LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTC, sysconf_exti_line );
    }
    else if( port == GPIOD )
    {
        LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTD, sysconf_exti_line );
    }
    else if( port == GPIOE )
    {
        LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTE, sysconf_exti_line );
    }
    else if( port == GPIOH )
    {
        LL_SYSCFG_SetEXTISource( LL_SYSCFG_EXTI_PORTH, sysconf_exti_line );
    }
}

static void call_gpio_irq_cb( uint8_t index )
{
    if( ( gpio_irq[index] != NULL ) && ( gpio_irq[index]->callback != NULL ) )
    {
        gpio_irq[index]->callback( gpio_irq[index]->context );
    }
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line[0:1] interrupts.
 */
void EXTI0_1_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_0 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_0 );
        call_gpio_irq_cb( 0 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_1 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_1 );
        call_gpio_irq_cb( 1 );
    }
}

/**
 * @brief This function handles EXTI line[2:3] interrupts.
 */
void EXTI2_3_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_2 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_2 );
        call_gpio_irq_cb( 2 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_3 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_3 );
        call_gpio_irq_cb( 3 );
    }
}

/**
 * @brief This function handles EXTI line[4:15] interrupts.
 */
void EXTI4_15_IRQHandler( void )
{
    // check all irq lines
    for( uint8_t i = 4; i <= 15; i++ )
    {
        if( LL_EXTI_IsActiveFlag_0_31( 0x1UL << i ) != RESET )
        {
            LL_EXTI_ClearFlag_0_31( 0x1UL << i );
            call_gpio_irq_cb( i );
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
