/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
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

#include "smtc_hal_lp_timer.h"
#include "stm32l4xx_hal.h"
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

static LPTIM_HandleTypeDef lptim_handle;

static hal_lp_timer_irq_t lptim_tmr_irq = { .context = NULL, .callback = NULL };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( void )
{
    lptim_handle.Instance             = LPTIM1;
    lptim_handle.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    lptim_handle.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV16;
    lptim_handle.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;
    lptim_handle.Init.OutputPolarity  = LPTIM_OUTPUTPOLARITY_HIGH;
    lptim_handle.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
    lptim_handle.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;

    if( HAL_LPTIM_Init( &lptim_handle ) != HAL_OK )
    {
        mcu_panic( );
    }
    lptim_tmr_irq = ( hal_lp_timer_irq_t ){ .context = NULL, .callback = NULL };
}

void hal_lp_timer_start( const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    uint32_t delay_ms_2_tick = 0;

    // Remark LSE_VALUE / LPTIM_PRESCALER_DIV16
    delay_ms_2_tick = ( uint32_t )( ( ( uint64_t ) milliseconds * ( LSE_VALUE >> 4 ) ) / 1000 );

    // check if delay_ms_2_tick is not greater than 0xFFFF and clamp it if it is the case
    if( delay_ms_2_tick > 0xFFFF )
    {
        delay_ms_2_tick = 0xFFFF;
    }

    // Auto reload period is set to max value 0xFFFF
    HAL_LPTIM_TimeOut_Start_IT( &lptim_handle, 0xFFFF, delay_ms_2_tick );
    lptim_tmr_irq = *tmr_irq;
}

void hal_lp_timer_stop( void )
{
    HAL_LPTIM_TimeOut_Stop_IT( &lptim_handle );
}

void hal_lp_timer_irq_enable( void )
{
    HAL_NVIC_EnableIRQ( LPTIM1_IRQn );
}

void hal_lp_timer_irq_disable( void )
{
    HAL_NVIC_DisableIRQ( LPTIM1_IRQn );
}

void LPTIM1_IRQHandler( void )
{
    HAL_LPTIM_IRQHandler( &lptim_handle );
    HAL_LPTIM_TimeOut_Stop( &lptim_handle );

    if( lptim_tmr_irq.callback != NULL )
    {
        lptim_tmr_irq.callback( lptim_tmr_irq.context );
    }
}

void HAL_LPTIM_MspInit( LPTIM_HandleTypeDef* lptimhandle )
{
    if( lptimhandle->Instance == LPTIM1 )
    {
        __HAL_RCC_LPTIM1_CLK_ENABLE( );
        HAL_NVIC_SetPriority( LPTIM1_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( LPTIM1_IRQn );
    }
}

void HAL_LPTIM_MspDeInit( LPTIM_HandleTypeDef* lptimhandle )
{
    if( lptimhandle->Instance == LPTIM1 )
    {
        __HAL_RCC_LPTIM1_CLK_DISABLE( );
        HAL_NVIC_DisableIRQ( LPTIM1_IRQn );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
