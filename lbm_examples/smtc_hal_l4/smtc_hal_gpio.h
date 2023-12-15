/*!
 * \file      smtc_hal_gpio.h
 *
 * \brief     GPIO Hardware Abstraction Layer definition
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
#ifndef __SMTC_HAL_GPIO_H__
#define __SMTC_HAL_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_gpio_pin_names.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * GPIO IRQ data context
 */
typedef struct gpio_irq_s
{
    hal_gpio_pin_names_t pin;
    void*                context;
    void                 ( *callback )( void* context );
} hal_gpio_irq_t;

/*!
 * GPIO Pull modes
 */
typedef enum gpio_pull_mode_e
{
    BSP_GPIO_PULL_MODE_NONE = 0,
    BSP_GPIO_PULL_MODE_UP   = 1,
    BSP_GPIO_PULL_MODE_DOWN = 2,
} hal_gpio_pull_mode_t;

/*!
 * GPIO IRQ modes
 */
typedef enum gpio_irq_mode_e
{
    BSP_GPIO_IRQ_MODE_OFF            = 0,
    BSP_GPIO_IRQ_MODE_RISING         = 1,
    BSP_GPIO_IRQ_MODE_FALLING        = 2,
    BSP_GPIO_IRQ_MODE_RISING_FALLING = 3,
} hal_gpio_irq_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * Initializes given pin as output with given initial value
 *
 * \param [in] pin   MCU pin to be initialized
 * \param [in] value MCU initial pit state
 *
 */
void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value );

/*!
 * Initializes given pin as input
 *
 * \param [in]     pin       MCU pin to be initialized
 * \param [in]     pull_mode MCU pin pull mode [BSP_GPIO_PULL_MODE_NONE,
 *                                              BSP_GPIO_PULL_MODE_UP,
 *                                              BSP_GPIO_PULL_MODE_DOWN]
 * \param [in]     irq_mode  MCU IRQ mode [BSP_GPIO_IRQ_MODE_OFF,
 *                                         BSP_GPIO_IRQ_MODE_RISING,
 *                                         BSP_GPIO_IRQ_MODE_FALLING,
 *                                         BSP_GPIO_IRQ_MODE_RISING_FALLING]
 * \param [in/out] irq       Pointer to IRQ data context.
 *                              NULL when BSP_GPIO_IRQ_MODE_OFF
 *                              pin parameter is initialized
 */
void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq );

/*!
 * Attaches given callback to the MCU IRQ handler
 *
 * \param [in] irq     Pointer to IRQ data context
 */
void hal_gpio_irq_attach( const hal_gpio_irq_t* irq );

/*!
 * Detattaches callback from the MCU IRQ handler
 *
 * \param [in] irq     Pointer to IRQ data context
 */
void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq );

/*!
 * Enables all GPIO MCU interrupts
 */
void hal_gpio_irq_enable( void );

/*!
 * Disables all GPIO MCU interrupts
 */
void hal_gpio_irq_disable( void );

/*!
 * Sets MCU pin to given value
 *
 * \param [in] pin   MCU pin to be set
 * \param [in] value MCU pin state to be set
 */
void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value );

/*!
 * Gets MCU pin state value
 *
 * \param [in] pin   MCU pin to be read
 *
 * \retval value Current MCU pin state
 */
uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin );

/*
 * Clears a pending irq on a pin
 *
 * \param [in] pin   pin for which pending state is to be cleared
 */
void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin );

/**
 * @brief Enable gpio clock according to gpio pin
 *
 * @param [in] pin gpio pin
 */
void hal_gpio_enable_clock( const hal_gpio_pin_names_t pin );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_GPIO_H__

/* --- EOF ------------------------------------------------------------------ */
