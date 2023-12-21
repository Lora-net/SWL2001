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
#include "stm32u5xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define DEFINE_EXTI_IRQ_HANDLER( line_number )              \
    void EXTI##line_number##_IRQHandler( void )             \
    {                                                       \
        HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_##line_number ); \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * GPIO setup data structure
 */
typedef struct bsp_gpio_s
{
    hal_gpio_pin_names_t pin;
    uint32_t             mode;
    uint32_t             pull;
    uint32_t             speed;
    uint32_t             alternate;
} gpio_t;

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

/*!
 * Generic gpio initialization
 *
 * \param [in/out] gpio  Holds MCU gpio parameters
 * \param [in]     value Initial MCU pit value
 * \param [in/out] irq   Pointer to IRQ data context.
 *                         NULL when setting gpio as output
 */
static void gpio_init( const gpio_t* gpio, const uint32_t value, const hal_gpio_irq_t* irq );

/*!
 * Generic call for rising and falling exti callback
 */
static void exti_callback_call( uint16_t gpio_pin );

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
    const uint32_t modes[] = { GPIO_MODE_INPUT, GPIO_MODE_IT_RISING, GPIO_MODE_IT_FALLING,
                               GPIO_MODE_IT_RISING_FALLING };
    const uint32_t pulls[] = { GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN };

    gpio_t gpio = {
        .pin = pin, .mode = modes[irq_mode], .pull = pulls[pull_mode], .speed = GPIO_SPEED_FREQ_LOW, .alternate = 0
    };

    if( irq != NULL )
    {
        irq->pin = pin;
    }

    gpio_init( &gpio, GPIO_PIN_RESET, irq );
}

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    gpio_t gpio = {
        .pin = pin, .mode = GPIO_MODE_OUTPUT_PP, .pull = GPIO_NOPULL, .speed = GPIO_SPEED_FREQ_LOW, .alternate = 0
    };
    gpio_init( &gpio, ( value != 0 ) ? GPIO_PIN_SET : GPIO_PIN_RESET, NULL );
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
    HAL_NVIC_EnableIRQ( EXTI0_IRQn );
    HAL_NVIC_EnableIRQ( EXTI1_IRQn );
    HAL_NVIC_EnableIRQ( EXTI2_IRQn );
    HAL_NVIC_EnableIRQ( EXTI3_IRQn );
    HAL_NVIC_EnableIRQ( EXTI4_IRQn );
    HAL_NVIC_EnableIRQ( EXTI5_IRQn );
    HAL_NVIC_EnableIRQ( EXTI6_IRQn );
    HAL_NVIC_EnableIRQ( EXTI7_IRQn );
    HAL_NVIC_EnableIRQ( EXTI8_IRQn );
    HAL_NVIC_EnableIRQ( EXTI9_IRQn );
    HAL_NVIC_EnableIRQ( EXTI10_IRQn );
    HAL_NVIC_EnableIRQ( EXTI11_IRQn );
    HAL_NVIC_EnableIRQ( EXTI12_IRQn );
    HAL_NVIC_EnableIRQ( EXTI13_IRQn );
    HAL_NVIC_EnableIRQ( EXTI14_IRQn );
    HAL_NVIC_EnableIRQ( EXTI15_IRQn );
}

void hal_gpio_irq_disable( void )
{
    HAL_NVIC_DisableIRQ( EXTI0_IRQn );
    HAL_NVIC_DisableIRQ( EXTI1_IRQn );
    HAL_NVIC_DisableIRQ( EXTI2_IRQn );
    HAL_NVIC_DisableIRQ( EXTI3_IRQn );
    HAL_NVIC_DisableIRQ( EXTI4_IRQn );
    HAL_NVIC_DisableIRQ( EXTI5_IRQn );
    HAL_NVIC_DisableIRQ( EXTI6_IRQn );
    HAL_NVIC_DisableIRQ( EXTI7_IRQn );
    HAL_NVIC_DisableIRQ( EXTI8_IRQn );
    HAL_NVIC_DisableIRQ( EXTI9_IRQn );
    HAL_NVIC_DisableIRQ( EXTI10_IRQn );
    HAL_NVIC_DisableIRQ( EXTI11_IRQn );
    HAL_NVIC_DisableIRQ( EXTI12_IRQn );
    HAL_NVIC_DisableIRQ( EXTI13_IRQn );
    HAL_NVIC_DisableIRQ( EXTI14_IRQn );
    HAL_NVIC_DisableIRQ( EXTI15_IRQn );
}

//
// MCU pin state control
//

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    HAL_GPIO_WritePin( gpio_port, ( 1 << ( pin & 0x0F ) ), ( value != 0 ) ? GPIO_PIN_SET : GPIO_PIN_RESET );
}

uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{
    GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( pin & 0xF0 ) << 6 ) );

    return ( HAL_GPIO_ReadPin( gpio_port, ( ( 1 << ( pin & 0x0F ) ) ) ) != GPIO_PIN_RESET ) ? 1 : 0;
}

void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin )
{
    switch( pin & 0x0F )
    {
    case 0:
        NVIC_ClearPendingIRQ( EXTI0_IRQn );
        break;
    case 1:
        NVIC_ClearPendingIRQ( EXTI1_IRQn );

        break;
    case 2:
        NVIC_ClearPendingIRQ( EXTI2_IRQn );
        break;
    case 3:
        NVIC_ClearPendingIRQ( EXTI3_IRQn );
        break;
    case 4:
        NVIC_ClearPendingIRQ( EXTI4_IRQn );
        break;
    case 5:
        NVIC_ClearPendingIRQ( EXTI5_IRQn );
        break;
    case 6:
        NVIC_ClearPendingIRQ( EXTI6_IRQn );
        break;
    case 7:
        NVIC_ClearPendingIRQ( EXTI7_IRQn );
        break;
    case 8:
        NVIC_ClearPendingIRQ( EXTI8_IRQn );
        break;
    case 9:
        NVIC_ClearPendingIRQ( EXTI9_IRQn );
        break;
    case 10:
        NVIC_ClearPendingIRQ( EXTI10_IRQn );
        break;
    case 11:
        NVIC_ClearPendingIRQ( EXTI11_IRQn );
        break;
    case 12:
        NVIC_ClearPendingIRQ( EXTI12_IRQn );
        break;
    case 13:
        NVIC_ClearPendingIRQ( EXTI13_IRQn );
        break;
    case 14:
        NVIC_ClearPendingIRQ( EXTI14_IRQn );
        break;
    case 15:
        NVIC_ClearPendingIRQ( EXTI15_IRQn );
        break;
    default:
        break;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void gpio_init( const gpio_t* gpio, const uint32_t value, const hal_gpio_irq_t* irq )
{
    GPIO_InitTypeDef gpio_local;
    GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( gpio->pin & 0xF0 ) << 6 ) );

    gpio_local.Pin       = ( 1 << ( gpio->pin & 0x0F ) );
    gpio_local.Mode      = gpio->mode;
    gpio_local.Pull      = gpio->pull;
    gpio_local.Speed     = gpio->speed;
    gpio_local.Alternate = gpio->alternate;

    if( gpio_port == GPIOA )
    {
        __HAL_RCC_GPIOA_CLK_ENABLE( );
    }
    else if( gpio_port == GPIOB )
    {
        __HAL_RCC_GPIOB_CLK_ENABLE( );
    }
    else if( gpio_port == GPIOC )
    {
        __HAL_RCC_GPIOC_CLK_ENABLE( );
    }
    else if( gpio_port == GPIOD )
    {
        __HAL_RCC_GPIOD_CLK_ENABLE( );
    }
    else if( gpio_port == GPIOE )
    {
        __HAL_RCC_GPIOE_CLK_ENABLE( );
    }
    else if( gpio_port == GPIOF )
    {
        __HAL_RCC_GPIOF_CLK_ENABLE( );
    }
    else if( gpio_port == GPIOH )
    {
        __HAL_RCC_GPIOH_CLK_ENABLE( );
    }

    HAL_GPIO_WritePin( gpio_port, gpio_local.Pin, ( GPIO_PinState ) value );
    HAL_GPIO_Init( gpio_port, &gpio_local );

    if( ( gpio->mode == GPIO_MODE_IT_RISING ) || ( gpio->mode == GPIO_MODE_IT_FALLING ) ||
        ( gpio->mode == GPIO_MODE_IT_RISING_FALLING ) )
    {
        hal_gpio_irq_attach( irq );
        switch( gpio->pin & 0x0F )
        {
        case 0:
            HAL_NVIC_SetPriority( EXTI0_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI0_IRQn );
            break;
        case 1:
            HAL_NVIC_SetPriority( EXTI1_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI1_IRQn );
            break;
        case 2:
            HAL_NVIC_SetPriority( EXTI2_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI2_IRQn );
            break;
        case 3:
            HAL_NVIC_SetPriority( EXTI3_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI3_IRQn );
            break;
        case 4:
            HAL_NVIC_SetPriority( EXTI4_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI4_IRQn );
            break;
        case 5:
            HAL_NVIC_SetPriority( EXTI5_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI5_IRQn );
            break;
        case 6:
            HAL_NVIC_SetPriority( EXTI6_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI6_IRQn );
            break;
        case 7:
            HAL_NVIC_SetPriority( EXTI7_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI7_IRQn );
            break;
        case 8:
            HAL_NVIC_SetPriority( EXTI8_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI8_IRQn );
            break;
        case 9:
            HAL_NVIC_SetPriority( EXTI9_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI9_IRQn );
            break;
        case 10:
            HAL_NVIC_SetPriority( EXTI10_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI10_IRQn );
            break;
        case 11:
            HAL_NVIC_SetPriority( EXTI11_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI11_IRQn );
            break;
        case 12:
            HAL_NVIC_SetPriority( EXTI12_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI12_IRQn );
            break;
        case 13:
            HAL_NVIC_SetPriority( EXTI13_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI13_IRQn );
            break;
        case 14:
            HAL_NVIC_SetPriority( EXTI14_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI14_IRQn );
            break;
        case 15:
            HAL_NVIC_SetPriority( EXTI15_IRQn, 0, 0 );
            HAL_NVIC_EnableIRQ( EXTI15_IRQn );
            break;
        default:
            break;
        }
    }
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/
DEFINE_EXTI_IRQ_HANDLER( 1 )
DEFINE_EXTI_IRQ_HANDLER( 2 )
DEFINE_EXTI_IRQ_HANDLER( 3 )
DEFINE_EXTI_IRQ_HANDLER( 4 )
DEFINE_EXTI_IRQ_HANDLER( 5 )
DEFINE_EXTI_IRQ_HANDLER( 6 )
DEFINE_EXTI_IRQ_HANDLER( 7 )
DEFINE_EXTI_IRQ_HANDLER( 8 )
DEFINE_EXTI_IRQ_HANDLER( 9 )
DEFINE_EXTI_IRQ_HANDLER( 10 )
DEFINE_EXTI_IRQ_HANDLER( 11 )
DEFINE_EXTI_IRQ_HANDLER( 12 )
DEFINE_EXTI_IRQ_HANDLER( 13 )
DEFINE_EXTI_IRQ_HANDLER( 14 )
DEFINE_EXTI_IRQ_HANDLER( 15 )

void HAL_GPIO_EXTI_Rising_Callback( uint16_t gpio_pin )
{
    exti_callback_call( gpio_pin );
}

void HAL_GPIO_EXTI_Falling_Callback( uint16_t gpio_pin )
{
    exti_callback_call( gpio_pin );
}

void exti_callback_call( uint16_t gpio_pin )
{
    uint8_t callback_index = 0;

    if( gpio_pin > 0 )
    {
        while( gpio_pin != 0x01 )
        {
            gpio_pin = gpio_pin >> 1;
            callback_index++;
        }
    }

    if( ( gpio_irq[callback_index] != NULL ) && ( gpio_irq[callback_index]->callback != NULL ) )
    {
        gpio_irq[callback_index]->callback( gpio_irq[callback_index]->context );
    }
}

/* --- EOF ------------------------------------------------------------------ */
