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
#include <stddef.h>

#include "smtc_hal_lp_timer.h"
#include "smtc_hal_mcu.h"
#include "stm32l0xx_ll_lptim.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_rcc.h"

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

static hal_lp_timer_irq_t lptim_tmr_irq = { .context = NULL, .callback = NULL };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }
    // Configure Irq
    NVIC_SetPriority( LPTIM1_IRQn, 0 );
    NVIC_EnableIRQ( LPTIM1_IRQn );

    // Enable LPIM clock
    LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_LPTIM1 );

    // Select LSE as lptim clock source
    LL_RCC_SetLPTIMClockSource( LL_RCC_LPTIM1_CLKSOURCE_LSE );

    // Config prescaler
    LL_LPTIM_SetPrescaler( LPTIM1, LL_LPTIM_PRESCALER_DIV16 );

    lptim_tmr_irq = ( hal_lp_timer_irq_t ){ .context = NULL, .callback = NULL };
}

void hal_lp_timer_start( hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    uint32_t delay_ms_2_tick = 0;

    // Remark LSE_VALUE / LPTIM_PRESCALER_DIV16
    delay_ms_2_tick = ( uint32_t ) ( ( ( uint64_t ) milliseconds * ( LSE_VALUE >> 4 ) ) / 1000 );

    // check if delay_ms_2_tick is not greater than 0xFFFF and clamp it if it is the case
    if( delay_ms_2_tick > 0xFFFF )
    {
        delay_ms_2_tick = 0xFFFF;
    }

    // Save irq callback
    lptim_tmr_irq = *tmr_irq;

    // Enable compare match interrupt
    LL_LPTIM_EnableIT_CMPM( LPTIM1 );

    // Enable periph
    LL_LPTIM_Enable( LPTIM1 );

    // Clear ARR OK flag before trying to update ARR register
    LL_LPTIM_ClearFlag_ARROK( LPTIM1 );

    // Set autoreload to max value
    LL_LPTIM_SetAutoReload( LPTIM1, 0xFFFF );

    // Wait for the completion of the write operation to the LPTIM_ARR register
    while( !LL_LPTIM_IsActiveFlag_ARROK( LPTIM1 ) )
    {
    }

    // Clear CMP OK flag before trying to update CMP register
    LL_LPTIM_ClearFlag_CMPOK( LPTIM1 );

    // Set the compare value
    LL_LPTIM_SetCompare( LPTIM1, delay_ms_2_tick );

    // Wait for the completion of the write operation to the LPTIM_CMP register
    while( !LL_LPTIM_IsActiveFlag_CMPOK( LPTIM1 ) )
    {
    }

    // Start the LPTIM counter in continuous mode
    LL_LPTIM_StartCounter( LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS );
}

void hal_lp_timer_stop( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    LL_LPTIM_Disable( LPTIM1 );
}

void hal_lp_timer_irq_enable( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    NVIC_EnableIRQ( LPTIM1_IRQn );
}

void hal_lp_timer_irq_disable( hal_lp_timer_id_t id )
{
    if( id != HAL_LP_TIMER_ID_1 )
    {
        mcu_panic( );
    }

    NVIC_DisableIRQ( LPTIM1_IRQn );
}

void LPTIM1_IRQHandler( void )
{
    // Check interrupt source

    if( LL_LPTIM_IsActiveFlag_CMPM( LPTIM1 ) == 1 )
    {
        LL_LPTIM_ClearFLAG_CMPM( LPTIM1 );
        if( lptim_tmr_irq.callback != NULL )
        {
            lptim_tmr_irq.callback( lptim_tmr_irq.context );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
