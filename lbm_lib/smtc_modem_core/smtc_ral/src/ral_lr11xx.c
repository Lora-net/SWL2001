/**
 * @file      ral_lr11xx.c
 *
 * @brief     Radio abstraction layer definition
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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_lr_fhss.h"
#include "ral_lr11xx.h"
#include "ral_lr11xx_bsp.h"

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

/**
 * @brief Convert interrupt flags from LR11XX context to RAL context
 *
 * @param [in] lr11xx_irq  LR11XX interrupt status
 *
 * @returns RAL interrupt status
 */
ral_irq_t ral_lr11xx_convert_irq_flags_to_ral( lr11xx_system_irq_mask_t lr11xx_irq );

/**
 * @brief Convert interrupt flags from RAL context to LR11XX context
 *
 * @param [in] ral_irq RAL interrupt status
 *
 * @returns LR11XX interrupt status
 */
lr11xx_system_irq_mask_t ral_lr11xx_convert_irq_flags_from_ral( ral_irq_t ral_irq );

/**
 * @brief Convert GFSK modulation parameters from RAL context to LR11XX context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 * @returns Operation status
 */
ral_status_t ral_lr11xx_convert_gfsk_mod_params_from_ral( const ral_gfsk_mod_params_t*    ral_mod_params,
                                                          lr11xx_radio_mod_params_gfsk_t* radio_mod_params );

/**
 * @brief Convert GFSK packet parameters from RAL context to LR11XX context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 *
 * @returns Operation status
 */
ral_status_t ral_lr11xx_convert_gfsk_pkt_params_from_ral( const ral_gfsk_pkt_params_t*    ral_pkt_params,
                                                          lr11xx_radio_pkt_params_gfsk_t* radio_pkt_params );

/**
 * @brief Convert LoRa modulation parameters from RAL context to LR11XX context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 * @returns Operation status
 */
ral_status_t ral_lr11xx_convert_lora_mod_params_from_ral( const ral_lora_mod_params_t*    ral_mod_params,
                                                          lr11xx_radio_mod_params_lora_t* radio_mod_params );

/**
 * @brief Convert LoRa packet parameters from RAL context to LR11XX context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 *
 * @returns Operation status
 */
ral_status_t ral_lr11xx_convert_lora_pkt_params_from_ral( const ral_lora_pkt_params_t*    ral_pkt_params,
                                                          lr11xx_radio_pkt_params_lora_t* radio_pkt_params );

/**
 * @brief Convert LoRa CAD parameters from RAL context to LR11XX context
 *
 * @param [in] ral_lora_cad_params     RAL LoRa CAD parameters
 * @param [out] radio_lora_cad_params  Radio LoRa CAD parameters
 *
 * @returns Operation status
 */
ral_status_t ral_lr11xx_convert_lora_cad_params_from_ral( const ral_lora_cad_params_t* ral_lora_cad_params,
                                                          lr11xx_radio_cad_params_t*   radio_lora_cad_params );

/**
 * @brief Convert LR-FHSS parameters from RAL context to LR11XX context
 *
 * @param [in] ral_lr_fhss_params     RAL LR-FHSS parameters
 * @param [out] radio_lr_fhss_params  Radio LR-FHSS parameters
 */
void ral_lr11xx_convert_lr_fhss_params_from_ral( const ral_lr_fhss_params_t* ral_lr_fhss_params,
                                                 lr11xx_lr_fhss_params_t*    radio_lr_fhss_params );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool ral_lr11xx_handles_part( const char* part_number )
{
    return ( strcmp( "lr1110", part_number ) == 0 ) || ( strcmp( "lr1120", part_number ) == 0 );
}

ral_status_t ral_lr11xx_reset( const void* context )
{
    return ( ral_status_t ) lr11xx_system_reset( context );
}

ral_status_t ral_lr11xx_init( const void* context )
{
    ral_status_t                        status = RAL_STATUS_ERROR;
    lr11xx_system_tcxo_supply_voltage_t tcxo_supply_voltage;
    lr11xx_system_reg_mode_t            reg_mode;
    lr11xx_system_rfswitch_cfg_t        system_rf_switch_cfg = { 0 };
    ral_xosc_cfg_t                      xosc_cfg;
    bool                                crc_is_activated     = false;
    uint32_t                            startup_time_in_tick = 0;
    bool                                rx_boost_is_activated;

    ral_lr11xx_bsp_get_crc_state( context, &crc_is_activated );
    if( crc_is_activated == true )
    {
        status = ( ral_status_t ) lr11xx_system_enable_spi_crc( context, true );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    ral_lr11xx_bsp_get_reg_mode( context, &reg_mode );
    status = ( ral_status_t ) lr11xx_system_set_reg_mode( context, reg_mode );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_lr11xx_bsp_get_rf_switch_cfg( context, &system_rf_switch_cfg );
    status = ( ral_status_t ) lr11xx_system_set_dio_as_rf_switch( context, &system_rf_switch_cfg );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_lr11xx_bsp_get_xosc_cfg( context, &xosc_cfg, &tcxo_supply_voltage, &startup_time_in_tick );
    if( xosc_cfg == RAL_XOSC_CFG_TCXO_RADIO_CTRL )
    {
        status = ( ral_status_t ) lr11xx_system_set_tcxo_mode( context, tcxo_supply_voltage, startup_time_in_tick );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }

        status = ( ral_status_t ) lr11xx_system_calibrate(
            context, LR11XX_SYSTEM_CALIB_LF_RC_MASK | LR11XX_SYSTEM_CALIB_HF_RC_MASK | LR11XX_SYSTEM_CALIB_PLL_MASK |
                         LR11XX_SYSTEM_CALIB_ADC_MASK | LR11XX_SYSTEM_CALIB_IMG_MASK |
                         LR11XX_SYSTEM_CALIB_PLL_TX_MASK );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    ral_lr11xx_bsp_get_rx_boost_cfg( context, &rx_boost_is_activated );
    status = ( ral_status_t ) lr11xx_radio_cfg_rx_boosted( context, rx_boost_is_activated );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_lr11xx_wakeup( const void* context )
{
    return ( ral_status_t ) lr11xx_system_wakeup( context );
}

ral_status_t ral_lr11xx_set_sleep( const void* context, const bool retain_config )
{
    ral_status_t status = RAL_STATUS_ERROR;
    bool         lfclk_is_running;

    ral_lr11xx_bsp_get_lfclk_cfg_in_sleep( context, &lfclk_is_running );

    const lr11xx_system_sleep_cfg_t radio_sleep_cfg = {
        .is_warm_start  = retain_config,
        .is_rtc_timeout = lfclk_is_running,
    };

    // Workaround: force LF clock to RC otherwise the wake-up sequence will not work
    if( lfclk_is_running == false )
    {
        status = ( ral_status_t ) lr11xx_system_cfg_lfclk( context, LR11XX_SYSTEM_LFCLK_RC, true );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    return ( ral_status_t ) lr11xx_system_set_sleep( context, radio_sleep_cfg, 0 );
}

ral_status_t ral_lr11xx_set_standby( const void* context, ral_standby_cfg_t ral_standby_cfg )
{
    lr11xx_system_standby_cfg_t radio_standby_cfg;

    switch( ral_standby_cfg )
    {
    case RAL_STANDBY_CFG_RC:
    {
        radio_standby_cfg = LR11XX_SYSTEM_STANDBY_CFG_RC;
        break;
    }
    case RAL_STANDBY_CFG_XOSC:
    {
        radio_standby_cfg = LR11XX_SYSTEM_STANDBY_CFG_XOSC;
        break;
    }
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return ( ral_status_t ) lr11xx_system_set_standby( context, radio_standby_cfg );
}

ral_status_t ral_lr11xx_set_fs( const void* context )
{
    return ( ral_status_t ) lr11xx_system_set_fs( context );
}

ral_status_t ral_lr11xx_set_tx( const void* context )
{
    return ( ral_status_t ) lr11xx_radio_set_tx( context, 0 );
}

ral_status_t ral_lr11xx_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms == RAL_RX_TIMEOUT_CONTINUOUS_MODE )
    {
        return ( ral_status_t ) lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, 0x00FFFFFF );
    }
    else
    {  // max timeout is 0xFFFFFE -> 511999 ms (0xFFFFFE / 32768 * 1000) - Single reception mode set if timeout_ms is 0
        if( timeout_in_ms < 512000 )
        {
            return ( ral_status_t ) lr11xx_radio_set_rx( context, timeout_in_ms );
        }
        else
        {
            return RAL_STATUS_ERROR;
        }
    }

    return RAL_STATUS_ERROR;
}

ral_status_t ral_lr11xx_cfg_rx_boosted( const void* context, const bool enable_boost_mode )
{
    return ( ral_status_t ) lr11xx_radio_cfg_rx_boosted( context, enable_boost_mode );
}

ral_status_t ral_lr11xx_set_rx_tx_fallback_mode( const void* context, const ral_fallback_modes_t ral_fallback_mode )
{
    lr11xx_radio_fallback_modes_t radio_fallback_mode;

    switch( ral_fallback_mode )
    {
    case RAL_FALLBACK_STDBY_RC:
    {
        radio_fallback_mode = LR11XX_RADIO_FALLBACK_STDBY_RC;
        break;
    }
    case RAL_FALLBACK_STDBY_XOSC:
    {
        radio_fallback_mode = LR11XX_RADIO_FALLBACK_STDBY_XOSC;
        break;
    }
    case RAL_FALLBACK_FS:
    {
        radio_fallback_mode = LR11XX_RADIO_FALLBACK_FS;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return ( ral_status_t ) lr11xx_radio_set_rx_tx_fallback_mode( context, radio_fallback_mode );
}

ral_status_t ral_lr11xx_stop_timer_on_preamble( const void* context, const bool enable )
{
    return ( ral_status_t ) lr11xx_radio_stop_timeout_on_preamble( context, enable );
}

ral_status_t ral_lr11xx_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                           const uint32_t sleep_time_in_ms )
{
    return ( ral_status_t ) lr11xx_radio_set_rx_duty_cycle( context, rx_time_in_ms, sleep_time_in_ms,
                                                            LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX );
}

ral_status_t ral_lr11xx_set_lora_cad( const void* context )
{
    return ( ral_status_t ) lr11xx_radio_set_cad( context );
}

ral_status_t ral_lr11xx_set_tx_cw( const void* context )
{
    return ( ral_status_t ) lr11xx_radio_set_tx_cw( context );
}

ral_status_t ral_lr11xx_set_tx_infinite_preamble( const void* context )
{
    return ( ral_status_t ) lr11xx_radio_set_tx_infinite_preamble( context );
}

ral_status_t ral_lr11xx_cal_img( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    return ( ral_status_t ) lr11xx_system_calibrate_image_in_mhz( context, freq1_in_mhz, freq2_in_mhz );
}

ral_status_t ral_lr11xx_set_tx_cfg( const void* context, const int8_t output_pwr_in_dbm, const uint32_t rf_freq_in_hz )
{
    ral_status_t                               status = RAL_STATUS_ERROR;
    ral_lr11xx_bsp_tx_cfg_output_params_t      tx_cfg_output_params;
    const ral_lr11xx_bsp_tx_cfg_input_params_t tx_cfg_input_params = {
        .freq_in_hz               = rf_freq_in_hz,
        .system_output_pwr_in_dbm = output_pwr_in_dbm,
    };

    ral_lr11xx_bsp_get_tx_cfg( context, &tx_cfg_input_params, &tx_cfg_output_params );

    status = ( ral_status_t ) lr11xx_radio_set_pa_cfg( context, &tx_cfg_output_params.pa_cfg );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ( ral_status_t ) lr11xx_radio_set_tx_params(
        context, tx_cfg_output_params.chip_output_pwr_in_dbm_configured, tx_cfg_output_params.pa_ramp_time );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_lr11xx_set_pkt_payload( const void* context, const uint8_t* buffer, const uint16_t size )
{
    return ( ral_status_t ) lr11xx_regmem_write_buffer8( context, buffer, size );
}

ral_status_t ral_lr11xx_get_pkt_payload( const void* context, uint16_t max_size_in_bytes, uint8_t* buffer,
                                         uint16_t* size_in_bytes )
{
    ral_status_t                    status                 = RAL_STATUS_ERROR;
    lr11xx_radio_rx_buffer_status_t radio_rx_buffer_status = { 0 };

    status = ( ral_status_t ) lr11xx_radio_get_rx_buffer_status( context, &radio_rx_buffer_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    if( size_in_bytes != 0 )
    {
        *size_in_bytes = radio_rx_buffer_status.pld_len_in_bytes;
    }

    if( radio_rx_buffer_status.pld_len_in_bytes <= max_size_in_bytes )
    {
        status = ( ral_status_t ) lr11xx_regmem_read_buffer8(
            context, buffer, radio_rx_buffer_status.buffer_start_pointer, radio_rx_buffer_status.pld_len_in_bytes );
    }
    else
    {
        status = RAL_STATUS_ERROR;
    }

    return status;
}

ral_status_t ral_lr11xx_get_irq_status( const void* context, ral_irq_t* irq )
{
    ral_status_t             status         = RAL_STATUS_ERROR;
    lr11xx_system_irq_mask_t radio_irq_mask = LR11XX_SYSTEM_IRQ_NONE;

    status = ( ral_status_t ) lr11xx_system_get_irq_status( context, &radio_irq_mask );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    *irq = ral_lr11xx_convert_irq_flags_to_ral( radio_irq_mask );

    return status;
}

ral_status_t ral_lr11xx_clear_irq_status( const void* context, const ral_irq_t irq )
{
    const lr11xx_system_irq_mask_t radio_irq = ral_lr11xx_convert_irq_flags_from_ral( irq );

    return ( ral_status_t ) lr11xx_system_clear_irq_status( context, radio_irq );
}

ral_status_t ral_lr11xx_get_and_clear_irq_status( const void* context, ral_irq_t* irq )
{
    ral_status_t             status    = RAL_STATUS_ERROR;
    lr11xx_system_irq_mask_t radio_irq = LR11XX_SYSTEM_IRQ_NONE;

    status = ( ral_status_t ) lr11xx_system_get_and_clear_irq_status( context, &radio_irq );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    if( irq != 0 )
    {
        *irq = ral_lr11xx_convert_irq_flags_to_ral( radio_irq );
    }

    return status;
}

ral_status_t ral_lr11xx_set_dio_irq_params( const void* context, const ral_irq_t irq )
{
    lr11xx_system_irq_mask_t lr11xx_irq = ral_lr11xx_convert_irq_flags_from_ral( irq );

    return ( ral_status_t ) lr11xx_system_set_dio_irq_params( context, lr11xx_irq, LR11XX_SYSTEM_IRQ_NONE );
}

ral_status_t ral_lr11xx_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    ral_status_t                          status = RAL_STATUS_ERROR;
    lr11xx_radio_rssi_calibration_table_t rssi_calibration_table;

    status = ( ral_status_t ) lr11xx_radio_set_rf_freq( context, freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_lr11xx_bsp_get_rssi_calibration_table( context, freq_in_hz, &rssi_calibration_table );

    status = ( ral_status_t ) lr11xx_radio_set_rssi_calibration( context, &rssi_calibration_table );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_lr11xx_set_pkt_type( const void* context, const ral_pkt_type_t ral_pkt_type )
{
    lr11xx_radio_pkt_type_t radio_pkt_type;

    switch( ral_pkt_type )
    {
    case RAL_PKT_TYPE_GFSK:
    {
        radio_pkt_type = LR11XX_RADIO_PKT_TYPE_GFSK;
        break;
    }
    case RAL_PKT_TYPE_LORA:
    {
        radio_pkt_type = LR11XX_RADIO_PKT_TYPE_LORA;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return ( ral_status_t ) lr11xx_radio_set_pkt_type( context, radio_pkt_type );
}

ral_status_t ral_lr11xx_get_pkt_type( const void* context, ral_pkt_type_t* pkt_type )
{
    ral_status_t            status = RAL_STATUS_ERROR;
    lr11xx_radio_pkt_type_t radio_pkt_type;

    status = ( ral_status_t ) lr11xx_radio_get_pkt_type( context, &radio_pkt_type );
    if( status == RAL_STATUS_OK )
    {
        switch( radio_pkt_type )
        {
        case LR11XX_RADIO_PKT_TYPE_GFSK:
        {
            *pkt_type = RAL_PKT_TYPE_GFSK;
            break;
        }
        case LR11XX_RADIO_PKT_TYPE_LORA:
        {
            *pkt_type = RAL_PKT_TYPE_LORA;
            break;
        }
        default:
        {
            return RAL_STATUS_UNKNOWN_VALUE;
        }
        }
    }

    return status;
}

ral_status_t ral_lr11xx_set_gfsk_mod_params( const void* context, const ral_gfsk_mod_params_t* ral_mod_params )
{
    ral_status_t                   status           = RAL_STATUS_ERROR;
    lr11xx_radio_mod_params_gfsk_t radio_mod_params = { 0 };

    status = ral_lr11xx_convert_gfsk_mod_params_from_ral( ral_mod_params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr11xx_radio_set_gfsk_mod_params( context, &radio_mod_params );
}

ral_status_t ral_lr11xx_set_gfsk_pkt_params( const void* context, const ral_gfsk_pkt_params_t* ral_pkt_params )
{
    ral_status_t                   status           = RAL_STATUS_ERROR;
    lr11xx_radio_pkt_params_gfsk_t radio_pkt_params = { 0 };

    status = ral_lr11xx_convert_gfsk_pkt_params_from_ral( ral_pkt_params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr11xx_radio_set_gfsk_pkt_params( context, &radio_pkt_params );
}

ral_status_t ral_lr11xx_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                              const uint8_t braodcast_address )
{
    return ( ral_status_t ) lr11xx_radio_set_pkt_address( context, node_address, braodcast_address );
}

ral_status_t ral_lr11xx_set_lora_mod_params( const void* context, const ral_lora_mod_params_t* ral_mod_params )
{
    ral_status_t                   status = RAL_STATUS_ERROR;
    lr11xx_radio_mod_params_lora_t radio_mod_params;

    status = ral_lr11xx_convert_lora_mod_params_from_ral( ral_mod_params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr11xx_radio_set_lora_mod_params( context, &radio_mod_params );
}

ral_status_t ral_lr11xx_set_lora_pkt_params( const void* context, const ral_lora_pkt_params_t* ral_pkt_params )
{
    ral_status_t                   status           = RAL_STATUS_ERROR;
    lr11xx_radio_pkt_params_lora_t radio_pkt_params = { 0 };

    status = ral_lr11xx_convert_lora_pkt_params_from_ral( ral_pkt_params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr11xx_radio_set_lora_pkt_params( context, &radio_pkt_params );
}

ral_status_t ral_lr11xx_set_lora_cad_params( const void* context, const ral_lora_cad_params_t* ral_lora_cad_params )
{
    ral_status_t              status = RAL_STATUS_ERROR;
    lr11xx_radio_cad_params_t radio_lora_cad_params;

    status = ral_lr11xx_convert_lora_cad_params_from_ral( ral_lora_cad_params, &radio_lora_cad_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr11xx_radio_set_cad_params( context, &radio_lora_cad_params );
}

ral_status_t ral_lr11xx_set_lora_symb_nb_timeout( const void* context, const uint16_t nb_of_symbs )
{
    return ( ral_status_t ) lr11xx_radio_set_lora_sync_timeout( context, nb_of_symbs );
}

ral_status_t ral_lr11xx_set_flrc_mod_params( const void* context, const ral_flrc_mod_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_set_flrc_pkt_params( const void* context, const ral_flrc_pkt_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_get_gfsk_rx_pkt_status( const void* context, ral_gfsk_rx_pkt_status_t* ral_rx_pkt_status )
{
    ral_status_t                   status = RAL_STATUS_ERROR;
    lr11xx_radio_pkt_status_gfsk_t radio_rx_pkt_status;

    status = ( ral_status_t ) lr11xx_radio_get_gfsk_pkt_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    uint8_t rx_status = 0;
    rx_status |= ( radio_rx_pkt_status.is_sent == true ) ? RAL_RX_STATUS_PKT_SENT : 0x00;
    rx_status |= ( radio_rx_pkt_status.is_received == true ) ? RAL_RX_STATUS_PKT_RECEIVED : 0x00;
    rx_status |= ( radio_rx_pkt_status.is_abort_err == true ) ? RAL_RX_STATUS_ABORT_ERROR : 0x00;
    rx_status |= ( radio_rx_pkt_status.is_len_err == true ) ? RAL_RX_STATUS_LENGTH_ERROR : 0x00;
    rx_status |= ( radio_rx_pkt_status.is_crc_err == true ) ? RAL_RX_STATUS_CRC_ERROR : 0x00;
    rx_status |= ( radio_rx_pkt_status.is_addr_err == true ) ? RAL_RX_STATUS_ADDR_ERROR : 0x00;

    ral_rx_pkt_status->rx_status = rx_status;

    ral_rx_pkt_status->rssi_sync_in_dbm = radio_rx_pkt_status.rssi_sync_in_dbm;
    ral_rx_pkt_status->rssi_avg_in_dbm  = radio_rx_pkt_status.rssi_avg_in_dbm;

    return status;
}

ral_status_t ral_lr11xx_get_lora_rx_pkt_status( const void* context, ral_lora_rx_pkt_status_t* ral_rx_pkt_status )
{
    ral_status_t                   status = RAL_STATUS_ERROR;
    lr11xx_radio_pkt_status_lora_t radio_rx_pkt_status;

    status = ( ral_status_t ) lr11xx_radio_get_lora_pkt_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_rx_pkt_status->rssi_pkt_in_dbm        = radio_rx_pkt_status.rssi_pkt_in_dbm;
    ral_rx_pkt_status->snr_pkt_in_db          = radio_rx_pkt_status.snr_pkt_in_db;
    ral_rx_pkt_status->signal_rssi_pkt_in_dbm = radio_rx_pkt_status.signal_rssi_pkt_in_dbm;

    return status;
}

ral_status_t ral_lr11xx_get_flrc_rx_pkt_status( const void* context, ral_flrc_rx_pkt_status_t* rx_pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    int8_t          local_rssi_in_dbm;
    lr11xx_status_t status = lr11xx_radio_get_rssi_inst( context, &local_rssi_in_dbm );
    *rssi_in_dbm           = ( int16_t ) local_rssi_in_dbm;

    return ( ral_status_t ) status;
}

uint32_t ral_lr11xx_get_lora_time_on_air_in_ms( const ral_lora_pkt_params_t* pkt_p, const ral_lora_mod_params_t* mod_p )
{
    lr11xx_radio_mod_params_lora_t radio_mod_params;
    lr11xx_radio_pkt_params_lora_t radio_pkt_params;

    ral_lr11xx_convert_lora_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_lr11xx_convert_lora_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return lr11xx_radio_get_lora_time_on_air_in_ms( &radio_pkt_params, &radio_mod_params );
}

uint32_t ral_lr11xx_get_gfsk_time_on_air_in_ms( const ral_gfsk_pkt_params_t* pkt_p, const ral_gfsk_mod_params_t* mod_p )
{
    lr11xx_radio_mod_params_gfsk_t radio_mod_params;
    lr11xx_radio_pkt_params_gfsk_t radio_pkt_params;

    ral_lr11xx_convert_gfsk_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_lr11xx_convert_gfsk_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return lr11xx_radio_get_gfsk_time_on_air_in_ms( &radio_pkt_params, &radio_mod_params );
}

uint32_t ral_lr11xx_get_flrc_time_on_air_in_ms( const ral_flrc_pkt_params_t* pkt_p, const ral_flrc_mod_params_t* mod_p )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    uint8_t sync_word_loc[8] = { 0 };

    for( uint8_t i = 0; i < sync_word_len; i++ )
    {
        sync_word_loc[i] = sync_word[i];
    }

    return ( ral_status_t ) lr11xx_radio_set_gfsk_sync_word( context, sync_word_loc );
}

ral_status_t ral_lr11xx_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    return ( ral_status_t ) lr11xx_radio_set_lora_sync_word( context, sync_word );
}

ral_status_t ral_lr11xx_set_flrc_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_set_gfsk_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    return ( ral_status_t ) lr11xx_radio_set_gfsk_crc_params( context, seed, polynomial );
}

ral_status_t ral_lr11xx_set_flrc_crc_params( const void* context, const uint32_t seed )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    return ( ral_status_t ) lr11xx_radio_set_gfsk_whitening_seed( context, seed );
}

ral_status_t ral_lr11xx_lr_fhss_init( const void* context, const ral_lr_fhss_params_t* lr_fhss_params )
{
    ( void ) lr_fhss_params;  // Unused parameter
    return ( ral_status_t ) lr11xx_lr_fhss_init( context );
}

ral_status_t ral_lr11xx_lr_fhss_build_frame( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                             ral_lr_fhss_memory_state_t state, uint16_t hop_sequence_id,
                                             const uint8_t* payload, uint16_t payload_length )
{
    ( void ) state;  // Unused argument

    lr11xx_lr_fhss_params_t lr11xx_params;
    ral_lr11xx_convert_lr_fhss_params_from_ral( lr_fhss_params, &lr11xx_params );

    lr11xx_status_t status = lr11xx_radio_set_rf_freq( context, lr_fhss_params->center_frequency_in_hz );
    if( status != LR11XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    status = lr11xx_lr_fhss_build_frame( context, &lr11xx_params, hop_sequence_id, payload, payload_length );
    if( status != LR11XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    lr11xx_system_stat1_t stat1;
    status = lr11xx_system_get_status( context, &stat1, 0, 0 );
    if( status != LR11XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    if( stat1.command_status != LR11XX_SYSTEM_CMD_STATUS_OK )
    {
        return RAL_STATUS_ERROR;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_lr_fhss_handle_hop( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                            ral_lr_fhss_memory_state_t state )
{
    ( void ) context;         // Unused arguments
    ( void ) state;           // Unused arguments
    ( void ) lr_fhss_params;  // Unused arguments
    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_lr_fhss_handle_tx_done( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                ral_lr_fhss_memory_state_t state )
{
    ( void ) context;         // Unused arguments
    ( void ) state;           // Unused arguments
    ( void ) lr_fhss_params;  // Unused arguments
    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_lr_fhss_get_time_on_air_in_ms( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                       uint16_t payload_length, uint32_t* time_on_air )
{
    lr11xx_lr_fhss_params_t lr11xx_params;
    ral_lr11xx_convert_lr_fhss_params_from_ral( lr_fhss_params, &lr11xx_params );

    *time_on_air = lr11xx_lr_fhss_get_time_on_air_in_ms( &lr11xx_params, payload_length );

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_lr_fhss_get_hop_sequence_count( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                        unsigned int* hop_sequence_count )
{
    lr11xx_lr_fhss_params_t lr11xx_params;
    ral_lr11xx_convert_lr_fhss_params_from_ral( lr_fhss_params, &lr11xx_params );

    *hop_sequence_count = lr11xx_lr_fhss_get_hop_sequence_count( &lr11xx_params );

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_lr_fhss_get_bit_delay_in_us( const void* context, const ral_lr_fhss_params_t* params,
                                                     uint16_t payload_length, uint16_t* delay )
{
    lr11xx_lr_fhss_params_t lr11xx_params;

    ral_lr11xx_convert_lr_fhss_params_from_ral( params, &lr11xx_params );

    *delay = lr11xx_lr_fhss_get_bit_delay_in_us( &lr11xx_params, payload_length );

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_get_lora_rx_pkt_cr_crc( const void* context, ral_lora_cr_t* cr, bool* is_crc_present )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_get_tx_consumption_in_ua( const void* context, const int8_t output_pwr_in_dbm,
                                                  const uint32_t rf_freq_in_hz, uint32_t* pwr_consumption_in_ua )
{
    // 1. Get the LR11xx BSP configuration corresponding to the input parameters
    lr11xx_system_reg_mode_t                   radio_reg_mode;
    ral_lr11xx_bsp_tx_cfg_output_params_t      tx_cfg_output_params;
    const ral_lr11xx_bsp_tx_cfg_input_params_t tx_cfg_input_params = {
        .freq_in_hz               = rf_freq_in_hz,
        .system_output_pwr_in_dbm = output_pwr_in_dbm,
    };
    ral_lr11xx_bsp_get_tx_cfg( context, &tx_cfg_input_params, &tx_cfg_output_params );
    ral_lr11xx_bsp_get_reg_mode( context, &radio_reg_mode );

    // 2. Refer to the BSP to get the instantaneous power consumption corresponding to the LR1xx BSP configuration
    return ral_lr11xx_bsp_get_instantaneous_tx_power_consumption( context, &tx_cfg_output_params, radio_reg_mode,
                                                                  pwr_consumption_in_ua );
}

ral_status_t ral_lr11xx_get_gfsk_rx_consumption_in_ua( const void* context, const uint32_t br_in_bps,
                                                       const uint32_t bw_dsb_in_hz, const bool rx_boosted,
                                                       uint32_t* pwr_consumption_in_ua )
{
    // 1. Get the regulator configured
    lr11xx_system_reg_mode_t radio_reg_mode;
    ral_lr11xx_bsp_get_reg_mode( context, &radio_reg_mode );

    // 2. Refer to BSP to get the instantaneous GFSK Rx Power consumption
    return ral_lr11xx_bsp_get_instantaneous_gfsk_rx_power_consumption( context, radio_reg_mode, rx_boosted,
                                                                       pwr_consumption_in_ua );
}

ral_status_t ral_lr11xx_get_lora_rx_consumption_in_ua( const void* context, const ral_lora_bw_t bw,
                                                       const bool rx_boosted, uint32_t* pwr_consumption_in_ua )
{
    // 1. Get the regulator configured
    lr11xx_system_reg_mode_t radio_reg_mode;
    ral_lr11xx_bsp_get_reg_mode( context, &radio_reg_mode );

    // 2. Refer to BSP to get the instantaneous LoRa Rx Power consumption
    return ral_lr11xx_bsp_get_instantaneous_lora_rx_power_consumption( context, radio_reg_mode, rx_boosted, pwr_consumption_in_ua );
}

ral_status_t ral_lr11xx_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    ral_status_t status = RAL_STATUS_ERROR;

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = ( ral_status_t ) lr11xx_system_get_random_number( context, numbers + i );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    return status;
}

ral_status_t ral_lr11xx_handle_rx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_handle_tx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                               ral_lora_cad_symbs_t nb_symbol, uint8_t* cad_det_peak )
{
    if( bw >= RAL_LORA_BW_500_KHZ )
    {
        switch( sf )
        {
        case RAL_LORA_SF5:
            *cad_det_peak = 65;
            break;
        case RAL_LORA_SF6:
            *cad_det_peak = 70;
            break;
        case RAL_LORA_SF7:
            *cad_det_peak = 77;
            break;
        case RAL_LORA_SF8:
            *cad_det_peak = 85;
            break;
        case RAL_LORA_SF9:
            *cad_det_peak = 78;
            break;
        case RAL_LORA_SF10:
            *cad_det_peak = 80;
            break;
        case RAL_LORA_SF11:
            *cad_det_peak = 79;
            break;
        case RAL_LORA_SF12:
            *cad_det_peak = 82;
            break;
        default:
            return RAL_STATUS_UNKNOWN_VALUE;
            break;
        }
    }
    else if( bw >= RAL_LORA_BW_125_KHZ )
    {
        switch( sf )
        {
        case RAL_LORA_SF5:
            *cad_det_peak = 56;
            break;
        case RAL_LORA_SF6:
            *cad_det_peak = 52;
            break;
        case RAL_LORA_SF7:
            *cad_det_peak = 52;
            break;
        case RAL_LORA_SF8:
            *cad_det_peak = 58;
            break;
        case RAL_LORA_SF9:
            *cad_det_peak = 58;
            break;
        case RAL_LORA_SF10:
            *cad_det_peak = 62;
            break;
        case RAL_LORA_SF11:
            *cad_det_peak = 66;
            break;
        case RAL_LORA_SF12:
            *cad_det_peak = 68;
            break;
        default:
            return RAL_STATUS_UNKNOWN_VALUE;
            break;
        }
    }
    else
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    // Nothing To do when nb_symbol == RAL_LORA_CAD_01_SYMB or nb_symbol == RAL_LORA_CAD_02_SYMB
    // with more symbols detection the sensibility could be increased
    switch( nb_symbol )
    {
    case RAL_LORA_CAD_01_SYMB:
    case RAL_LORA_CAD_02_SYMB:
        break;
    case RAL_LORA_CAD_04_SYMB:
        *cad_det_peak -= 1;
        break;
    case RAL_LORA_CAD_08_SYMB:
    case RAL_LORA_CAD_16_SYMB:
        *cad_det_peak -= 2;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
        break;
    }

    ral_lr11xx_bsp_get_lora_cad_det_peak( context, sf, bw, nb_symbol, cad_det_peak );

    return RAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

ral_irq_t ral_lr11xx_convert_irq_flags_to_ral( lr11xx_system_irq_mask_t lr11xx_irq_status )
{
    ral_irq_t ral_irq = RAL_IRQ_NONE;

    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_TX_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_TX_DONE;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_RX_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_DONE;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_PREAMBLE_DETECTED;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_TIMEOUT ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_TIMEOUT;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_HDR_OK;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_HEADER_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_HDR_ERROR;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_CRC_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_CRC_ERROR;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_CAD_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_CAD_DONE;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_CAD_DETECTED ) != 0 )
    {
        ral_irq |= RAL_IRQ_CAD_OK;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP ) != 0 )
    {
        ral_irq |= RAL_IRQ_LR_FHSS_HOP;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_WIFI_SCAN_DONE;
    }
    if( ( lr11xx_irq_status & LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_GNSS_SCAN_DONE;
    }

    return ral_irq;
}

lr11xx_system_irq_mask_t ral_lr11xx_convert_irq_flags_from_ral( ral_irq_t ral_irq )
{
    lr11xx_system_irq_mask_t lr11xx_irq_status = LR11XX_SYSTEM_IRQ_NONE;

    if( ( ral_irq & RAL_IRQ_TX_DONE ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_TX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_DONE ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_RX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_PREAMBLE_DETECTED ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED;
    }
    if( ( ral_irq & RAL_IRQ_RX_TIMEOUT ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_TIMEOUT;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_OK ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_ERROR ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_HEADER_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_RX_CRC_ERROR ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_CRC_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_CAD_DONE ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_CAD_DONE;
    }
    if( ( ral_irq & RAL_IRQ_CAD_OK ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_CAD_DETECTED;
    }
    if( ( ral_irq & RAL_IRQ_LR_FHSS_HOP ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP;
    }
    if( ( ral_irq & RAL_IRQ_WIFI_SCAN_DONE ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE;
    }
    if( ( ral_irq & RAL_IRQ_GNSS_SCAN_DONE ) != 0 )
    {
        lr11xx_irq_status |= LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE;
    }

    return lr11xx_irq_status;
}

ral_status_t ral_lr11xx_convert_gfsk_mod_params_from_ral( const ral_gfsk_mod_params_t*    ral_mod_params,
                                                          lr11xx_radio_mod_params_gfsk_t* radio_mod_params )
{
    ral_status_t           status = RAL_STATUS_ERROR;
    lr11xx_radio_gfsk_bw_t bw_dsb_param;

    status = ( ral_status_t ) lr11xx_radio_get_gfsk_rx_bandwidth( ral_mod_params->bw_dsb_in_hz, &bw_dsb_param );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    radio_mod_params->br_in_bps    = ral_mod_params->br_in_bps;
    radio_mod_params->fdev_in_hz   = ral_mod_params->fdev_in_hz;
    radio_mod_params->bw_dsb_param = bw_dsb_param;

    switch( ral_mod_params->pulse_shape )
    {
    case RAL_GFSK_PULSE_SHAPE_OFF:
    {
        radio_mod_params->pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_03:
    {
        radio_mod_params->pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_03;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_05:
    {
        radio_mod_params->pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_07:
    {
        radio_mod_params->pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_1:
    {
        radio_mod_params->pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return status;
}

ral_status_t ral_lr11xx_convert_gfsk_pkt_params_from_ral( const ral_gfsk_pkt_params_t*    ral_pkt_params,
                                                          lr11xx_radio_pkt_params_gfsk_t* radio_pkt_params )
{
    radio_pkt_params->preamble_len_in_bits = ral_pkt_params->preamble_len_in_bits;

    switch( ral_pkt_params->preamble_detector )
    {
    case RAL_GFSK_PREAMBLE_DETECTOR_OFF:
    {
        radio_pkt_params->preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_OFF;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_8BITS:
    {
        radio_pkt_params->preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS:
    {
        radio_pkt_params->preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_24BITS:
    {
        radio_pkt_params->preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_32BITS:
    {
        radio_pkt_params->preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_pkt_params->sync_word_len_in_bits = ral_pkt_params->sync_word_len_in_bits;

    switch( ral_pkt_params->address_filtering )
    {
    case RAL_GFSK_ADDRESS_FILTERING_DISABLE:
    {
        radio_pkt_params->address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE;
        break;
    }
    case RAL_GFSK_ADDRESS_FILTERING_NODE_ADDRESS:
    {
        radio_pkt_params->address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS;
        break;
    }
    case RAL_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES:
    {
        radio_pkt_params->address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    switch( ral_pkt_params->header_type )
    {
    case RAL_GFSK_PKT_FIX_LEN:
    {
        radio_pkt_params->header_type = LR11XX_RADIO_GFSK_PKT_FIX_LEN;
        break;
    }
    case RAL_GFSK_PKT_VAR_LEN:
    {
        radio_pkt_params->header_type = LR11XX_RADIO_GFSK_PKT_VAR_LEN;
        break;
    }
    case RAL_GFSK_PKT_VAR_LEN_SX128X_COMP:
    {
        radio_pkt_params->header_type = LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_pkt_params->pld_len_in_bytes = ral_pkt_params->pld_len_in_bytes;

    switch( ral_pkt_params->crc_type )
    {
    case RAL_GFSK_CRC_OFF:
    {
        radio_pkt_params->crc_type = LR11XX_RADIO_GFSK_CRC_OFF;
        break;
    }
    case RAL_GFSK_CRC_1_BYTE:
    {
        radio_pkt_params->crc_type = LR11XX_RADIO_GFSK_CRC_1_BYTE;
        break;
    }
    case RAL_GFSK_CRC_2_BYTES:
    {
        radio_pkt_params->crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES;
        break;
    }
    case RAL_GFSK_CRC_1_BYTE_INV:
    {
        radio_pkt_params->crc_type = LR11XX_RADIO_GFSK_CRC_1_BYTE_INV;
        break;
    }
    case RAL_GFSK_CRC_2_BYTES_INV:
    {
        radio_pkt_params->crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES_INV;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    switch( ral_pkt_params->dc_free )
    {
    case RAL_GFSK_DC_FREE_OFF:
    {
        radio_pkt_params->dc_free = LR11XX_RADIO_GFSK_DC_FREE_OFF;
        break;
    }
    case RAL_GFSK_DC_FREE_WHITENING:
    {
        radio_pkt_params->dc_free = LR11XX_RADIO_GFSK_DC_FREE_WHITENING;
        break;
    }
    case RAL_GFSK_DC_FREE_WHITENING_SX128X_COMP:
    {
        radio_pkt_params->dc_free = LR11XX_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_convert_lora_mod_params_from_ral( const ral_lora_mod_params_t*    ral_mod_params,
                                                          lr11xx_radio_mod_params_lora_t* radio_mod_params )
{
    radio_mod_params->sf = ( lr11xx_radio_lora_sf_t ) ral_mod_params->sf;

    switch( ral_mod_params->bw )
    {
    case RAL_LORA_BW_010_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_10;
        break;
    }
    case RAL_LORA_BW_015_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_15;
        break;
    }
    case RAL_LORA_BW_020_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_20;
        break;
    }
    case RAL_LORA_BW_031_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_31;
        break;
    }
    case RAL_LORA_BW_041_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_41;
        break;
    }
    case RAL_LORA_BW_062_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_62;
        break;
    }
    case RAL_LORA_BW_125_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_125;
        break;
    }
    case RAL_LORA_BW_200_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_200;
        break;
    }
    case RAL_LORA_BW_250_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_250;
        break;
    }
    case RAL_LORA_BW_400_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_400;
        break;
    }
    case RAL_LORA_BW_500_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_500;
        break;
    }
    case RAL_LORA_BW_800_KHZ:
    {
        radio_mod_params->bw = LR11XX_RADIO_LORA_BW_800;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_mod_params->cr = ( lr11xx_radio_lora_cr_t ) ral_mod_params->cr;

    radio_mod_params->ldro = ral_mod_params->ldro;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_convert_lora_pkt_params_from_ral( const ral_lora_pkt_params_t*    ral_pkt_params,
                                                          lr11xx_radio_pkt_params_lora_t* radio_pkt_params )
{
    radio_pkt_params->preamble_len_in_symb = ral_pkt_params->preamble_len_in_symb;

    switch( ral_pkt_params->header_type )
    {
    case( RAL_LORA_PKT_EXPLICIT ):
    {
        radio_pkt_params->header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT;
        break;
    }
    case( RAL_LORA_PKT_IMPLICIT ):
    {
        radio_pkt_params->header_type = LR11XX_RADIO_LORA_PKT_IMPLICIT;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_pkt_params->pld_len_in_bytes = ral_pkt_params->pld_len_in_bytes;
    radio_pkt_params->crc =
        ( ral_pkt_params->crc_is_on == false ) ? LR11XX_RADIO_LORA_CRC_OFF : LR11XX_RADIO_LORA_CRC_ON;
    radio_pkt_params->iq =
        ( ral_pkt_params->invert_iq_is_on == false ) ? LR11XX_RADIO_LORA_IQ_STANDARD : LR11XX_RADIO_LORA_IQ_INVERTED;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr11xx_convert_lora_cad_params_from_ral( const ral_lora_cad_params_t* ral_lora_cad_params,
                                                          lr11xx_radio_cad_params_t*   radio_lora_cad_params )
{
    switch( ral_lora_cad_params->cad_symb_nb )
    {
    case RAL_LORA_CAD_01_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = 1;
        break;
    }
    case RAL_LORA_CAD_02_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = 2;
        break;
    }
    case RAL_LORA_CAD_04_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = 4;
        break;
    }
    case RAL_LORA_CAD_08_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = 8;
        break;
    }
    case RAL_LORA_CAD_16_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = 16;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_lora_cad_params->cad_detect_peak = ral_lora_cad_params->cad_det_peak_in_symb;
    radio_lora_cad_params->cad_detect_min  = ral_lora_cad_params->cad_det_min_in_symb;

    switch( ral_lora_cad_params->cad_exit_mode )
    {
    case RAL_LORA_CAD_ONLY:
    {
        radio_lora_cad_params->cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC;
        break;
    }
    case RAL_LORA_CAD_RX:
    {
        radio_lora_cad_params->cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_RX;
        break;
    }
    case RAL_LORA_CAD_LBT:
    {
        radio_lora_cad_params->cad_exit_mode = LR11XX_RADIO_CAD_EXIT_MODE_TX;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_lora_cad_params->cad_timeout =
        lr11xx_radio_convert_time_in_ms_to_rtc_step( ral_lora_cad_params->cad_timeout_in_ms );

    return RAL_STATUS_OK;
}

void ral_lr11xx_convert_lr_fhss_params_from_ral( const ral_lr_fhss_params_t* ral_lr_fhss_params,
                                                 lr11xx_lr_fhss_params_t*    radio_lr_fhss_params )
{
    *radio_lr_fhss_params = ( lr11xx_lr_fhss_params_t ){
        .lr_fhss_params = ral_lr_fhss_params->lr_fhss_params,
        .device_offset  = ral_lr_fhss_params->device_offset,
    };
}

/* --- EOF ------------------------------------------------------------------ */
