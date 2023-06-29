/*!
 * \file      smtc_hal_adc.c
 *
 * \brief     ADC Hardware Abstraction Layer implementation
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

#include "smtc_hal_adc.h"
#include "stm32l4xx_hal.h"

#include "modem_pinout.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */
static uint16_t adc_read( uint32_t channel, uint32_t sampling_time );
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

static ADC_HandleTypeDef hal_adc_handle;
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_adc_init( void )
{
    hal_adc_handle.Instance                   = ADC1;
    hal_adc_handle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV12;
    hal_adc_handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hal_adc_handle.Init.Resolution            = ADC_RESOLUTION_12B;
    hal_adc_handle.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hal_adc_handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hal_adc_handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hal_adc_handle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hal_adc_handle.Init.NbrOfConversion       = 1;
    hal_adc_handle.Init.NbrOfDiscConversion   = 0;
    hal_adc_handle.Init.ContinuousConvMode    = DISABLE;
    hal_adc_handle.Init.DiscontinuousConvMode = DISABLE;
    hal_adc_handle.Init.LowPowerAutoWait      = DISABLE;
    hal_adc_handle.Init.DMAContinuousRequests = DISABLE;
    hal_adc_handle.Init.OversamplingMode      = DISABLE;
    hal_adc_handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;

    if( HAL_ADC_Init( &hal_adc_handle ) != HAL_OK )
    {
        mcu_panic( );
    }

    if( HAL_ADCEx_Calibration_Start( &hal_adc_handle, ADC_SINGLE_ENDED ) != HAL_OK )
    {
        mcu_panic( );  // Calibration Error
    }
}

uint16_t hal_adc_get_vref_int( void )
{
    // 6.5 cycle == 1µs (6.5 x 12 / 80)
    uint16_t adc_val = adc_read( ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_6CYCLES_5 );
    if( adc_val > 0 )
    {
        return ( uint16_t ) __HAL_ADC_CALC_VREFANALOG_VOLTAGE( adc_val, ADC_RESOLUTION_12B );
    }
    else
    {
        return 0;
    }
}

int8_t hal_adc_get_vbat( void )
{
    // Update vref for more precise measure
    uint16_t vref_int_mv = hal_adc_get_vref_int( );

    // 6.5 cycle == 1µs (6.5 x 12 / 80)
    uint16_t adc_val = adc_read( ADC_CHANNEL_VBAT, ADC_SAMPLETIME_6CYCLES_5 );
    uint16_t vbat    = __HAL_ADC_CALC_DATA_TO_VOLTAGE( vref_int_mv, adc_val, ADC_RESOLUTION_12B );
    return vbat;
}

int8_t hal_adc_get_temp( void )
{
    // Update vref for more precise measure
    uint16_t vref_int_mv = hal_adc_get_vref_int( );

    // Internal temperature sensor needs at least 5µs to be measured properly
    // ADC is clock at 6.66 MHz (80Mhz / 12) so 5µs is 33.3 cycle --> we choose 47.5 cycles (7.125µs)
    uint16_t adc_val     = adc_read( ADC_CHANNEL_TEMPSENSOR, ADC_SAMPLETIME_47CYCLES_5 );
    int32_t  temperature = __HAL_ADC_CALC_TEMPERATURE( vref_int_mv, adc_val, ADC_RESOLUTION_12B );

    return ( int8_t ) temperature;
}

void hal_adc_deinit( void )
{
    HAL_ADC_DeInit( &hal_adc_handle );
}

void HAL_ADC_MspInit( ADC_HandleTypeDef* adc_handle )
{
    __HAL_RCC_ADC_CLK_ENABLE( );
}

void HAL_ADC_MspDeInit( ADC_HandleTypeDef* adc_handle )
{
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Read channel from adc
 *
 * @param channel       channel to read ( see to @defgroup ADC_HAL_EC_CHANNEL)
 * @param sampling_time sampling time   ( see to @defgroup ADC_HAL_EC_CHANNEL_SAMPLINGTIME)
 * @return uint16_t     adc raw value
 */
static uint16_t adc_read( uint32_t channel, uint32_t sampling_time )
{
    ADC_ChannelConfTypeDef adc_channel_conf = { 0 };
    uint16_t               adc_value        = 0;

    adc_channel_conf.Channel      = channel;
    adc_channel_conf.SamplingTime = sampling_time;
    adc_channel_conf.Rank         = ADC_REGULAR_RANK_1;

    if( HAL_ADC_ConfigChannel( &hal_adc_handle, &adc_channel_conf ) != HAL_OK )
    {
        return 0;
    }

    if( HAL_ADC_Start( &hal_adc_handle ) != HAL_OK )
    {
        return 0;
    }

    if( HAL_ADC_PollForConversion( &hal_adc_handle, 10 ) != HAL_OK )
    {
        return 0;
    }

    if( ( HAL_ADC_GetState( &hal_adc_handle ) & HAL_ADC_STATE_REG_EOC ) == HAL_ADC_STATE_REG_EOC )
    {
        adc_value = HAL_ADC_GetValue( &hal_adc_handle );
    }

    if( HAL_ADC_Stop( &hal_adc_handle ) != HAL_OK )
    {
        return 0;
    }

    return adc_value;
}
