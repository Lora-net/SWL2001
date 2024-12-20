/**
 * @file      ral_sx126x.c
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
#include "sx126x.h"
#include "sx126x_lr_fhss.h"
#include "ral_sx126x.h"
#include "ral_sx126x_bsp.h"

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
 * @brief Convert interrupt flags from SX126x context to RAL context
 *
 * @param [in] sx126x_irq  SX126x interrupt status
 *
 * @returns RAL interrupt status
 */
ral_irq_t ral_sx126x_convert_irq_flags_to_ral( sx126x_irq_mask_t sx126x_irq );

/**
 * @brief Convert interrupt flags from RAL context to SX126x context
 *
 * @param [in] ral_irq RAL interrupt status
 *
 * @returns SX126x interrupt status
 */
sx126x_irq_mask_t ral_sx126x_convert_irq_flags_from_ral( ral_irq_t ral_irq );

/**
 * @brief Convert GFSK modulation parameters from RAL context to SX126x context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 * @returns Operation status
 */
ral_status_t ral_sx126x_convert_gfsk_mod_params_from_ral( const ral_gfsk_mod_params_t* ral_mod_params,
                                                          sx126x_mod_params_gfsk_t*    radio_mod_params );

/**
 * @brief Convert GFSK packet parameters from RAL context to SX126x context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 *
 * @returns Operation status
 */
ral_status_t ral_sx126x_convert_gfsk_pkt_params_from_ral( const ral_gfsk_pkt_params_t* ral_pkt_params,
                                                          sx126x_pkt_params_gfsk_t*    radio_pkt_params );

/**
 * @brief Convert LoRa modulation parameters from RAL context to SX126x context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 * @returns Operation status
 */
ral_status_t ral_sx126x_convert_lora_mod_params_from_ral( const ral_lora_mod_params_t* ral_mod_params,
                                                          sx126x_mod_params_lora_t*    radio_mod_params );

/**
 * @brief Convert LoRa packet parameters from RAL context to SX126x context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 *
 * @returns Operation status
 */
ral_status_t ral_sx126x_convert_lora_pkt_params_from_ral( const ral_lora_pkt_params_t* ral_pkt_params,
                                                          sx126x_pkt_params_lora_t*    radio_pkt_params );

/**
 * @brief Convert LoRa coding rate value from SX126x context to RAL context
 *
 * @param [in] radio_cr  Radio LoRa coding rate value
 * @param [out] ral_cr   RAL LoRa coding rate value
 *
 * @returns Operation status
 */
ral_status_t ral_sx126x_convert_lora_cr_to_ral( const sx126x_lora_cr_t radio_cr, ral_lora_cr_t* ral_cr );

/**
 * @brief Convert LoRa CAD parameters from RAL context to SX126x context
 *
 * @param [in] ral_lora_cad_params     RAL LoRa CAD parameters
 * @param [out] radio_lora_cad_params  Radio LoRa CAD parameters
 *
 * @returns Operation status
 */
ral_status_t ral_sx126x_convert_lora_cad_params_from_ral( const ral_lora_cad_params_t* ral_lora_cad_params,
                                                          sx126x_cad_params_t*         radio_lora_cad_params );

/**
 * @brief Convert LR FHSS params structure from RAL context to SX126x context
 *
 * @param [in] ral_lr_fhss_params     RAL LR FHSS parameters
 *
 * \returns SX126X LR FHSS parameters
 *
 * @returns Operation status
 */
void ral_sx126x_convert_lr_fhss_params_from_ral( const ral_lr_fhss_params_t* ral_lr_fhss_params,
                                                 sx126x_lr_fhss_params_t*    radio_lr_fhss_params );

/**
 * @brief Configure the crystal trimming capacitor
 *
 * @param [in] context  Radio context
 *
 * @returns Operation status
 */
static ral_status_t ral_sx126x_cfg_trim_cap( const void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool ral_sx126x_handles_part( const char* part_number )
{
    return ( strcmp( "sx1261", part_number ) == 0 ) || ( strcmp( "sx1262", part_number ) == 0 );
}

ral_status_t ral_sx126x_reset( const void* context )
{
    return ( ral_status_t ) sx126x_reset( context );
}

ral_status_t ral_sx126x_init( const void* context )
{
    ral_status_t                status = RAL_STATUS_ERROR;
    sx126x_tcxo_ctrl_voltages_t tcxo_supply_voltage;
    sx126x_reg_mod_t            reg_mode;
    bool                        dio2_is_set_as_rf_switch = false;
    ral_xosc_cfg_t              xosc_cfg;
    uint32_t                    startup_time_in_tick = 0;
    bool                        rx_boost_is_activated;

    status = ( ral_status_t ) sx126x_init_retention_list( context );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_sx126x_bsp_get_reg_mode( context, &reg_mode );
    status = ( ral_status_t ) sx126x_set_reg_mode( context, reg_mode );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_sx126x_bsp_get_rf_switch_cfg( context, &dio2_is_set_as_rf_switch );
    status = ( ral_status_t ) sx126x_set_dio2_as_rf_sw_ctrl( context, dio2_is_set_as_rf_switch );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_sx126x_bsp_get_xosc_cfg( context, &xosc_cfg, &tcxo_supply_voltage, &startup_time_in_tick );
    if( xosc_cfg == RAL_XOSC_CFG_TCXO_RADIO_CTRL )
    {
        status = ( ral_status_t ) sx126x_set_dio3_as_tcxo_ctrl( context, tcxo_supply_voltage, startup_time_in_tick );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }

        status = ( ral_status_t ) sx126x_cal( context, SX126X_CAL_ALL );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    ral_sx126x_bsp_get_rx_boost_cfg( context, &rx_boost_is_activated );
    status = ( ral_status_t ) sx126x_cfg_rx_boosted( context, rx_boost_is_activated );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_sx126x_wakeup( const void* context )
{
    return ( ral_status_t ) sx126x_wakeup( context );
}

ral_status_t ral_sx126x_set_sleep( const void* context, const bool retain_config )
{
    const sx126x_sleep_cfgs_t radio_sleep_cfg =
        ( retain_config == true ) ? SX126X_SLEEP_CFG_WARM_START : SX126X_SLEEP_CFG_COLD_START;

    return ( ral_status_t ) sx126x_set_sleep( context, radio_sleep_cfg );
}

ral_status_t ral_sx126x_set_standby( const void* context, ral_standby_cfg_t standby_cfg )
{
    sx126x_standby_cfg_t radio_standby_cfg;

    switch( standby_cfg )
    {
    case RAL_STANDBY_CFG_RC:
    {
        radio_standby_cfg = SX126X_STANDBY_CFG_RC;
        break;
    }
    case RAL_STANDBY_CFG_XOSC:
    {
        radio_standby_cfg = SX126X_STANDBY_CFG_XOSC;
        break;
    }
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return ( ral_status_t ) sx126x_set_standby( context, radio_standby_cfg );
}

ral_status_t ral_sx126x_set_fs( const void* context )
{
    return ( ral_status_t ) sx126x_set_fs( context );
}

ral_status_t ral_sx126x_set_tx( const void* context )
{
    ral_status_t status = ral_sx126x_cfg_trim_cap( context );

    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_tx( context, 0 );
    }

    return status;
}

ral_status_t ral_sx126x_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    ral_status_t status = ral_sx126x_cfg_trim_cap( context );

    if( status == RAL_STATUS_OK )
    {
        if( timeout_in_ms == RAL_RX_TIMEOUT_CONTINUOUS_MODE )
        {
            return ( ral_status_t ) sx126x_set_rx_with_timeout_in_rtc_step( context, 0x00FFFFFF );
        }
        else
        {  // max timeout is 0xFFFFFE -> 262143 ms (0xFFFFFE / 64000 * 1000) - Single reception mode set if timeout_ms
           // is 0
            if( timeout_in_ms < 262144 )
            {
                return ( ral_status_t ) sx126x_set_rx( context, timeout_in_ms );
            }
            else
            {
                return RAL_STATUS_ERROR;
            }
        }
    }

    return RAL_STATUS_ERROR;
}

ral_status_t ral_sx126x_cfg_rx_boosted( const void* context, const bool enable_boost_mode )
{
    return ( ral_status_t ) sx126x_cfg_rx_boosted( context, enable_boost_mode );
}

ral_status_t ral_sx126x_set_rx_tx_fallback_mode( const void* context, const ral_fallback_modes_t ral_fallback_mode )
{
    sx126x_fallback_modes_t radio_fallback_mode;

    switch( ral_fallback_mode )
    {
    case RAL_FALLBACK_STDBY_RC:
    {
        radio_fallback_mode = SX126X_FALLBACK_STDBY_RC;
        break;
    }
    case RAL_FALLBACK_STDBY_XOSC:
    {
        radio_fallback_mode = SX126X_FALLBACK_STDBY_XOSC;
        break;
    }
    case RAL_FALLBACK_FS:
    {
        radio_fallback_mode = SX126X_FALLBACK_FS;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return ( ral_status_t ) sx126x_set_rx_tx_fallback_mode( context, radio_fallback_mode );
}

ral_status_t ral_sx126x_stop_timer_on_preamble( const void* context, const bool enable )
{
    return ( ral_status_t ) sx126x_stop_timer_on_preamble( context, enable );
}

ral_status_t ral_sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                           const uint32_t sleep_time_in_ms )
{
    return ( ral_status_t ) sx126x_set_rx_duty_cycle( context, rx_time_in_ms, sleep_time_in_ms );
}

ral_status_t ral_sx126x_set_lora_cad( const void* context )
{
    ral_status_t status = ral_sx126x_cfg_trim_cap( context );

    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_cad( context );
    }

    return status;
}

ral_status_t ral_sx126x_set_tx_cw( const void* context )
{
    ral_status_t status = ral_sx126x_cfg_trim_cap( context );

    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_tx_cw( context );
    }

    return status;
}

ral_status_t ral_sx126x_set_tx_infinite_preamble( const void* context )
{
    ral_status_t status = ral_sx126x_cfg_trim_cap( context );

    if( status == RAL_STATUS_OK )
    {
        status = ( ral_status_t ) sx126x_set_tx_infinite_preamble( context );
    }

    return status;
}

ral_status_t ral_sx126x_cal_img( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    return ( ral_status_t ) sx126x_cal_img_in_mhz( context, freq1_in_mhz, freq2_in_mhz );
}

ral_status_t ral_sx126x_set_tx_cfg( const void* context, const int8_t output_pwr_in_dbm, const uint32_t rf_freq_in_hz )
{
    ral_status_t                               status = RAL_STATUS_ERROR;
    ral_sx126x_bsp_tx_cfg_output_params_t      tx_cfg_output_params;
    const ral_sx126x_bsp_tx_cfg_input_params_t tx_cfg_input_params = {
        .freq_in_hz               = rf_freq_in_hz,
        .system_output_pwr_in_dbm = output_pwr_in_dbm,
    };

    ral_sx126x_bsp_get_tx_cfg( context, &tx_cfg_input_params, &tx_cfg_output_params );

    if( tx_cfg_output_params.pa_cfg.device_sel == 0x00 )
    {
        status = ( ral_status_t ) sx126x_cfg_tx_clamp( context );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    status = ( ral_status_t ) sx126x_set_pa_cfg( context, &tx_cfg_output_params.pa_cfg );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    uint8_t ocp_in_step_of_2_5_ma = ( tx_cfg_output_params.pa_cfg.device_sel == 0x00 ) ? 0x38 : 0x18;

    ral_sx126x_bsp_get_ocp_value( context, &ocp_in_step_of_2_5_ma );

    if( ( ( tx_cfg_output_params.pa_cfg.device_sel == 0x00 ) && ( ocp_in_step_of_2_5_ma != 0x38 ) ) ||
        ( ( tx_cfg_output_params.pa_cfg.device_sel == 0x01 ) && ( ocp_in_step_of_2_5_ma != 0x18 ) ) )
    {
        status = ( ral_status_t ) sx126x_set_ocp_value( context, ocp_in_step_of_2_5_ma );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    status = ( ral_status_t ) sx126x_set_tx_params( context, tx_cfg_output_params.chip_output_pwr_in_dbm_configured,
                                                    tx_cfg_output_params.pa_ramp_time );

    return status;
}

ral_status_t ral_sx126x_set_pkt_payload( const void* context, const uint8_t* buffer, const uint16_t size )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ( ral_status_t ) sx126x_set_buffer_base_address( context, 0x00, 0x00 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ( ral_status_t ) sx126x_write_buffer( context, 0x00, buffer, size );

    return status;
}

ral_status_t ral_sx126x_get_pkt_payload( const void* context, uint16_t max_size_in_bytes, uint8_t* buffer,
                                         uint16_t* size_in_bytes )
{
    ral_status_t              status = RAL_STATUS_ERROR;
    sx126x_rx_buffer_status_t radio_rx_buffer_status;

    status = ( ral_status_t ) sx126x_get_rx_buffer_status( context, &radio_rx_buffer_status );
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
        status = ( ral_status_t ) sx126x_read_buffer( context, radio_rx_buffer_status.buffer_start_pointer, buffer,
                                                      radio_rx_buffer_status.pld_len_in_bytes );
    }
    else
    {
        status = RAL_STATUS_ERROR;
    }

    return status;
}

ral_status_t ral_sx126x_get_irq_status( const void* context, ral_irq_t* irq )
{
    ral_status_t      status          = RAL_STATUS_ERROR;
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    status = ( ral_status_t ) sx126x_get_irq_status( context, &sx126x_irq_mask );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    *irq = ral_sx126x_convert_irq_flags_to_ral( sx126x_irq_mask );

    return status;
}

ral_status_t ral_sx126x_clear_irq_status( const void* context, const ral_irq_t irq )
{
    const sx126x_irq_mask_t sx126x_irq_mask = ral_sx126x_convert_irq_flags_from_ral( irq );

    return ( ral_status_t ) sx126x_clear_irq_status( context, sx126x_irq_mask );
}

ral_status_t ral_sx126x_get_and_clear_irq_status( const void* context, ral_irq_t* irq )
{
    ral_status_t      status          = RAL_STATUS_ERROR;
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    status = ( ral_status_t ) sx126x_get_and_clear_irq_status( context, &sx126x_irq_mask );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    if( irq != 0 )
    {
        *irq = ral_sx126x_convert_irq_flags_to_ral( sx126x_irq_mask );
    }

    return status;
}

ral_status_t ral_sx126x_set_dio_irq_params( const void* context, const ral_irq_t irq )
{
    const uint16_t sx126x_irq = ral_sx126x_convert_irq_flags_from_ral( irq );

    return ( ral_status_t ) sx126x_set_dio_irq_params( context, SX126X_IRQ_ALL, sx126x_irq, SX126X_IRQ_NONE,
                                                       SX126X_IRQ_NONE );
}

ral_status_t ral_sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    return ( ral_status_t ) sx126x_set_rf_freq( context, freq_in_hz );
}

ral_status_t ral_sx126x_set_pkt_type( const void* context, const ral_pkt_type_t pkt_type )
{
    sx126x_pkt_type_t radio_pkt_type;

    switch( pkt_type )
    {
    case RAL_PKT_TYPE_GFSK:
    {
        radio_pkt_type = SX126X_PKT_TYPE_GFSK;
        break;
    }
    case RAL_PKT_TYPE_LORA:
    {
        radio_pkt_type = SX126X_PKT_TYPE_LORA;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return ( ral_status_t ) sx126x_set_pkt_type( context, radio_pkt_type );
}

ral_status_t ral_sx126x_get_pkt_type( const void* context, ral_pkt_type_t* pkt_type )
{
    ral_status_t      status = RAL_STATUS_ERROR;
    sx126x_pkt_type_t radio_pkt_type;

    status = ( ral_status_t ) sx126x_get_pkt_type( context, &radio_pkt_type );
    if( status == RAL_STATUS_OK )
    {
        switch( radio_pkt_type )
        {
        case SX126X_PKT_TYPE_GFSK:
        {
            *pkt_type = RAL_PKT_TYPE_GFSK;
            break;
        }
        case SX126X_PKT_TYPE_LORA:
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

ral_status_t ral_sx126x_set_gfsk_mod_params( const void* context, const ral_gfsk_mod_params_t* params )
{
    ral_status_t             status           = RAL_STATUS_ERROR;
    sx126x_mod_params_gfsk_t radio_mod_params = { 0 };

    status = ral_sx126x_convert_gfsk_mod_params_from_ral( params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) sx126x_set_gfsk_mod_params( context, &radio_mod_params );
}

ral_status_t ral_sx126x_set_gfsk_pkt_params( const void* context, const ral_gfsk_pkt_params_t* params )
{
    ral_status_t             status           = RAL_STATUS_ERROR;
    sx126x_pkt_params_gfsk_t radio_pkt_params = { 0 };

    status = ral_sx126x_convert_gfsk_pkt_params_from_ral( params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) sx126x_set_gfsk_pkt_params( context, &radio_pkt_params );
}

ral_status_t ral_sx126x_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                              const uint8_t braodcast_address )
{
    return ( ral_status_t ) sx126x_set_gfsk_pkt_address( context, node_address, braodcast_address );
}

ral_status_t ral_sx126x_set_lora_mod_params( const void* context, const ral_lora_mod_params_t* params )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    sx126x_mod_params_lora_t radio_mod_params;

    status = ral_sx126x_convert_lora_mod_params_from_ral( params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) sx126x_set_lora_mod_params( context, &radio_mod_params );
}

ral_status_t ral_sx126x_set_lora_pkt_params( const void* context, const ral_lora_pkt_params_t* params )
{
    ral_status_t             status           = RAL_STATUS_ERROR;
    sx126x_pkt_params_lora_t radio_pkt_params = { 0 };

    status = ral_sx126x_convert_lora_pkt_params_from_ral( params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) sx126x_set_lora_pkt_params( context, &radio_pkt_params );
}

ral_status_t ral_sx126x_set_lora_cad_params( const void* context, const ral_lora_cad_params_t* params )
{
    ral_status_t        status = RAL_STATUS_ERROR;
    sx126x_cad_params_t radio_lora_cad_params;

    status = ral_sx126x_convert_lora_cad_params_from_ral( params, &radio_lora_cad_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) sx126x_set_cad_params( context, &radio_lora_cad_params );
}

ral_status_t ral_sx126x_set_lora_symb_nb_timeout( const void* context, const uint16_t nb_of_symbs )
{
    if( nb_of_symbs <= SX126X_MAX_LORA_SYMB_NUM_TIMEOUT )
    {
        return ( ral_status_t ) sx126x_set_lora_symb_nb_timeout( context, nb_of_symbs );
    }
    else
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
}

ral_status_t ral_sx126x_set_flrc_mod_params( const void* context, const ral_flrc_mod_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}
ral_status_t ral_sx126x_set_flrc_pkt_params( const void* context, const ral_flrc_pkt_params_t* params )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_get_gfsk_rx_pkt_status( const void* context, ral_gfsk_rx_pkt_status_t* rx_pkt_status )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    sx126x_pkt_status_gfsk_t radio_rx_pkt_status;

    status = ( ral_status_t ) sx126x_get_gfsk_pkt_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    uint8_t rx_status = 0;
    rx_status |= ( radio_rx_pkt_status.rx_status.pkt_sent == true ) ? RAL_RX_STATUS_PKT_SENT : 0x00;
    rx_status |= ( radio_rx_pkt_status.rx_status.pkt_received == true ) ? RAL_RX_STATUS_PKT_RECEIVED : 0x00;
    rx_status |= ( radio_rx_pkt_status.rx_status.abort_error == true ) ? RAL_RX_STATUS_ABORT_ERROR : 0x00;
    rx_status |= ( radio_rx_pkt_status.rx_status.length_error == true ) ? RAL_RX_STATUS_LENGTH_ERROR : 0x00;
    rx_status |= ( radio_rx_pkt_status.rx_status.crc_error == true ) ? RAL_RX_STATUS_CRC_ERROR : 0x00;
    rx_status |= ( radio_rx_pkt_status.rx_status.adrs_error == true ) ? RAL_RX_STATUS_ADDR_ERROR : 0x00;

    rx_pkt_status->rx_status = rx_status;

    rx_pkt_status->rssi_sync_in_dbm = radio_rx_pkt_status.rssi_sync;
    rx_pkt_status->rssi_avg_in_dbm  = radio_rx_pkt_status.rssi_avg;

    return status;
}

ral_status_t ral_sx126x_get_lora_rx_pkt_status( const void* context, ral_lora_rx_pkt_status_t* rx_pkt_status )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    sx126x_pkt_status_lora_t radio_rx_pkt_status;

    status = ( ral_status_t ) sx126x_get_lora_pkt_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    rx_pkt_status->rssi_pkt_in_dbm        = radio_rx_pkt_status.rssi_pkt_in_dbm;
    rx_pkt_status->snr_pkt_in_db          = radio_rx_pkt_status.snr_pkt_in_db;
    rx_pkt_status->signal_rssi_pkt_in_dbm = radio_rx_pkt_status.signal_rssi_pkt_in_dbm;

    return status;
}

ral_status_t ral_sx126x_get_flrc_rx_pkt_status( const void* context, ral_flrc_rx_pkt_status_t* rx_pkt_status )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    return ( ral_status_t ) sx126x_get_rssi_inst( context, rssi_in_dbm );
}

uint32_t ral_sx126x_get_lora_time_on_air_in_ms( const ral_lora_pkt_params_t* pkt_p, const ral_lora_mod_params_t* mod_p )
{
    sx126x_mod_params_lora_t radio_mod_params;
    sx126x_pkt_params_lora_t radio_pkt_params;

    ral_sx126x_convert_lora_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_sx126x_convert_lora_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return sx126x_get_lora_time_on_air_in_ms( &radio_pkt_params, &radio_mod_params );
}

uint32_t ral_sx126x_get_gfsk_time_on_air_in_ms( const ral_gfsk_pkt_params_t* pkt_p, const ral_gfsk_mod_params_t* mod_p )
{
    sx126x_mod_params_gfsk_t radio_mod_params;
    sx126x_pkt_params_gfsk_t radio_pkt_params;

    ral_sx126x_convert_gfsk_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_sx126x_convert_gfsk_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return sx126x_get_gfsk_time_on_air_in_ms( &radio_pkt_params, &radio_mod_params );
}

uint32_t ral_sx126x_get_flrc_time_on_air_in_ms( const ral_flrc_pkt_params_t* pkt_p, const ral_flrc_mod_params_t* mod_p )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    return ( ral_status_t ) sx126x_set_gfsk_sync_word( context, sync_word, sync_word_len );
}

ral_status_t ral_sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    return ( ral_status_t ) sx126x_set_lora_sync_word( context, sync_word );
}

ral_status_t ral_sx126x_set_flrc_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_set_gfsk_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ( ral_status_t ) sx126x_set_gfsk_crc_seed( context, ( uint16_t ) seed );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ( ral_status_t ) sx126x_set_gfsk_crc_polynomial( context, ( uint16_t ) polynomial );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_sx126x_set_flrc_crc_params( const void* context, const uint32_t seed )
{
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    return ( ral_status_t ) sx126x_set_gfsk_whitening_seed( context, seed );
}

ral_status_t ral_sx126x_lr_fhss_init( const void* context, const ral_lr_fhss_params_t* lr_fhss_params )
{
    sx126x_lr_fhss_params_t sx126x_params;
    ral_sx126x_convert_lr_fhss_params_from_ral( lr_fhss_params, &sx126x_params );

    return ( ral_status_t ) sx126x_lr_fhss_init( context, &sx126x_params );
}

ral_status_t ral_sx126x_lr_fhss_build_frame( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                             ral_lr_fhss_memory_state_t state, uint16_t hop_sequence_id,
                                             const uint8_t* payload, uint16_t payload_length )
{
    sx126x_lr_fhss_params_t sx126x_params;
    ral_sx126x_convert_lr_fhss_params_from_ral( lr_fhss_params, &sx126x_params );

    return ( ral_status_t ) sx126x_lr_fhss_build_frame( context, &sx126x_params, ( sx126x_lr_fhss_state_t* ) state,
                                                        hop_sequence_id, payload, payload_length, NULL );
}

ral_status_t ral_sx126x_lr_fhss_handle_hop( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                            ral_lr_fhss_memory_state_t state )
{
    sx126x_lr_fhss_params_t sx126x_params;
    ral_sx126x_convert_lr_fhss_params_from_ral( lr_fhss_params, &sx126x_params );

    return ( ral_status_t ) sx126x_lr_fhss_handle_hop( context, &sx126x_params, ( sx126x_lr_fhss_state_t* ) state );
}

ral_status_t ral_sx126x_lr_fhss_handle_tx_done( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                ral_lr_fhss_memory_state_t state )
{
    sx126x_lr_fhss_params_t sx126x_params;
    ral_sx126x_convert_lr_fhss_params_from_ral( lr_fhss_params, &sx126x_params );

    return ( ral_status_t ) sx126x_lr_fhss_handle_tx_done( context, &sx126x_params, ( sx126x_lr_fhss_state_t* ) state );
}

ral_status_t ral_sx126x_lr_fhss_get_time_on_air_in_ms( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                       uint16_t payload_length, uint32_t* time_on_air )
{
    sx126x_lr_fhss_params_t sx126x_params;
    ral_sx126x_convert_lr_fhss_params_from_ral( lr_fhss_params, &sx126x_params );

    *time_on_air = sx126x_lr_fhss_get_time_on_air_in_ms( &sx126x_params, payload_length );

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_lr_fhss_get_hop_sequence_count( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                        unsigned int* hop_sequence_count )
{
    sx126x_lr_fhss_params_t sx126x_params;
    ral_sx126x_convert_lr_fhss_params_from_ral( lr_fhss_params, &sx126x_params );

    *hop_sequence_count = sx126x_lr_fhss_get_hop_sequence_count( &sx126x_params );

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_lr_fhss_get_bit_delay_in_us( const void* context, const ral_lr_fhss_params_t* params,
                                                     uint16_t payload_length, uint16_t* delay )
{
    sx126x_lr_fhss_params_t sx126x_params;

    ral_sx126x_convert_lr_fhss_params_from_ral( params, &sx126x_params );

    *delay = sx126x_lr_fhss_get_bit_delay_in_us( &sx126x_params, payload_length );

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_get_lora_rx_pkt_cr_crc( const void* context, ral_lora_cr_t* cr, bool* is_crc_present )
{
    ral_status_t     status = RAL_STATUS_ERROR;
    sx126x_lora_cr_t radio_cr;

    status = ( ral_status_t ) sx126x_get_lora_params_from_header( context, &radio_cr, is_crc_present );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ral_sx126x_convert_lora_cr_to_ral( radio_cr, cr );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_sx126x_get_tx_consumption_in_ua( const void* context, const int8_t output_pwr_in_dbm,
                                                  const uint32_t rf_freq_in_hz, uint32_t* pwr_consumption_in_ua )
{
    sx126x_reg_mod_t                           radio_reg_mode;
    ral_sx126x_bsp_tx_cfg_output_params_t      tx_cfg_output_params;
    const ral_sx126x_bsp_tx_cfg_input_params_t tx_cfg_input_params = {
        .freq_in_hz               = rf_freq_in_hz,
        .system_output_pwr_in_dbm = output_pwr_in_dbm,
    };
    ral_sx126x_bsp_get_tx_cfg( context, &tx_cfg_input_params, &tx_cfg_output_params );
    ral_sx126x_bsp_get_reg_mode( context, &radio_reg_mode );

    return ral_sx126x_bsp_get_instantaneous_tx_power_consumption( context, &tx_cfg_output_params, radio_reg_mode,
                                                                  pwr_consumption_in_ua );
}

ral_status_t ral_sx126x_get_gfsk_rx_consumption_in_ua( const void* context, const uint32_t br_in_bps,
                                                       const uint32_t bw_dsb_in_hz, const bool rx_boosted,
                                                       uint32_t* pwr_consumption_in_ua )
{
    sx126x_reg_mod_t radio_reg_mode;
    ral_sx126x_bsp_get_reg_mode( context, &radio_reg_mode );

    return ral_sx126x_bsp_get_instantaneous_gfsk_rx_power_consumption( context, radio_reg_mode, rx_boosted,
                                                                       pwr_consumption_in_ua );
}

ral_status_t ral_sx126x_get_lora_rx_consumption_in_ua( const void* context, const ral_lora_bw_t bw,
                                                       const bool rx_boosted, uint32_t* pwr_consumption_in_ua )
{
    sx126x_reg_mod_t radio_reg_mode;
    ral_sx126x_bsp_get_reg_mode( context, &radio_reg_mode );

    return ral_sx126x_bsp_get_instantaneous_lora_rx_power_consumption( context, radio_reg_mode, rx_boosted,
                                                                       pwr_consumption_in_ua );
}

ral_status_t ral_sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    return ( ral_status_t ) sx126x_get_random_numbers( context, numbers, n );
}

ral_status_t ral_sx126x_handle_rx_done( const void* context )
{
    return ( ral_status_t ) sx126x_handle_rx_done( context );
}

ral_status_t ral_sx126x_handle_tx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                               ral_lora_cad_symbs_t nb_symbol, uint8_t* cad_det_peak )
{
    if( bw >= RAL_LORA_BW_500_KHZ )
    {
        switch( sf )
        {
        case RAL_LORA_SF5:
            *cad_det_peak = 19;
            break;
        case RAL_LORA_SF6:
            *cad_det_peak = 20;
            break;
        case RAL_LORA_SF7:
            *cad_det_peak = 21;
            break;
        case RAL_LORA_SF8:
            *cad_det_peak = 22;
            break;
        case RAL_LORA_SF9:
            *cad_det_peak = 23;
            break;
        case RAL_LORA_SF10:
            *cad_det_peak = 23;
            break;
        case RAL_LORA_SF11:
            *cad_det_peak = 26;
            break;
        case RAL_LORA_SF12:
            *cad_det_peak = 30;
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
            *cad_det_peak = 19;
            break;
        case RAL_LORA_SF6:
            *cad_det_peak = 20;
            break;
        case RAL_LORA_SF7:
            *cad_det_peak = 22;
            break;
        case RAL_LORA_SF8:
            *cad_det_peak = 22;
            break;
        case RAL_LORA_SF9:
            *cad_det_peak = 24;
            break;
        case RAL_LORA_SF10:
            *cad_det_peak = 25;
            break;
        case RAL_LORA_SF11:
            *cad_det_peak = 26;
            break;
        case RAL_LORA_SF12:
            *cad_det_peak = 29;
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

    ral_sx126x_bsp_get_lora_cad_det_peak( context, sf, bw, nb_symbol, cad_det_peak );

    return RAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

ral_irq_t ral_sx126x_convert_irq_flags_to_ral( sx126x_irq_mask_t sx126x_irq )
{
    ral_irq_t ral_irq = RAL_IRQ_NONE;

    if( ( sx126x_irq & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
    {
        ral_irq |= RAL_IRQ_TX_DONE;
    }
    if( ( sx126x_irq & SX126X_IRQ_RX_DONE ) == SX126X_IRQ_RX_DONE )
    {
        ral_irq |= RAL_IRQ_RX_DONE;
    }
    if( ( sx126x_irq & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT )
    {
        ral_irq |= RAL_IRQ_RX_TIMEOUT;
    }
    if( ( ( sx126x_irq & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID ) ||
        ( ( sx126x_irq & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID ) )
    {
        ral_irq |= RAL_IRQ_RX_HDR_OK;
    }
    if( ( sx126x_irq & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
    {
        ral_irq |= RAL_IRQ_RX_HDR_ERROR;
    }
    if( ( sx126x_irq & SX126X_IRQ_CRC_ERROR ) == SX126X_IRQ_CRC_ERROR )
    {
        ral_irq |= RAL_IRQ_RX_CRC_ERROR;
    }
    if( ( sx126x_irq & SX126X_IRQ_CAD_DONE ) == SX126X_IRQ_CAD_DONE )
    {
        ral_irq |= RAL_IRQ_CAD_DONE;
    }
    if( ( sx126x_irq & SX126X_IRQ_CAD_DETECTED ) == SX126X_IRQ_CAD_DETECTED )
    {
        ral_irq |= RAL_IRQ_CAD_OK;
    }
    if( ( sx126x_irq & SX126X_IRQ_PREAMBLE_DETECTED ) == SX126X_IRQ_PREAMBLE_DETECTED )
    {
        ral_irq |= RAL_IRQ_RX_PREAMBLE_DETECTED;
    }
    if( ( sx126x_irq & SX126X_IRQ_LR_FHSS_HOP ) == SX126X_IRQ_LR_FHSS_HOP )
    {
        ral_irq |= RAL_IRQ_LR_FHSS_HOP;
    }
    return ral_irq;
}

sx126x_irq_mask_t ral_sx126x_convert_irq_flags_from_ral( ral_irq_t ral_irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    if( ( ral_irq & RAL_IRQ_TX_DONE ) == RAL_IRQ_TX_DONE )
    {
        sx126x_irq_mask |= SX126X_IRQ_TX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_DONE ) == RAL_IRQ_RX_DONE )
    {
        sx126x_irq_mask |= SX126X_IRQ_RX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_TIMEOUT ) == RAL_IRQ_RX_TIMEOUT )
    {
        sx126x_irq_mask |= SX126X_IRQ_TIMEOUT;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_OK ) == RAL_IRQ_RX_HDR_OK )
    {
        sx126x_irq_mask |= SX126X_IRQ_SYNC_WORD_VALID;
        sx126x_irq_mask |= SX126X_IRQ_HEADER_VALID;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_ERROR ) == RAL_IRQ_RX_HDR_ERROR )
    {
        sx126x_irq_mask |= SX126X_IRQ_HEADER_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_RX_CRC_ERROR ) == RAL_IRQ_RX_CRC_ERROR )
    {
        sx126x_irq_mask |= SX126X_IRQ_CRC_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_CAD_DONE ) == RAL_IRQ_CAD_DONE )
    {
        sx126x_irq_mask |= SX126X_IRQ_CAD_DONE;
    }
    if( ( ral_irq & RAL_IRQ_CAD_OK ) == RAL_IRQ_CAD_OK )
    {
        sx126x_irq_mask |= SX126X_IRQ_CAD_DETECTED;
    }
    if( ( ral_irq & RAL_IRQ_RX_PREAMBLE_DETECTED ) == RAL_IRQ_RX_PREAMBLE_DETECTED )
    {
        sx126x_irq_mask |= SX126X_IRQ_PREAMBLE_DETECTED;
    }
    if( ( ral_irq & RAL_IRQ_LR_FHSS_HOP ) == RAL_IRQ_LR_FHSS_HOP )
    {
        sx126x_irq_mask |= SX126X_IRQ_LR_FHSS_HOP;
    }

    return sx126x_irq_mask;
}

ral_status_t ral_sx126x_convert_gfsk_mod_params_from_ral( const ral_gfsk_mod_params_t* ral_mod_params,
                                                          sx126x_mod_params_gfsk_t*    radio_mod_params )
{
    ral_status_t status       = RAL_STATUS_ERROR;
    uint8_t      bw_dsb_param = 0;

    status = ( ral_status_t ) sx126x_get_gfsk_bw_param( ral_mod_params->bw_dsb_in_hz, &bw_dsb_param );
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
        radio_mod_params->pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_03:
    {
        radio_mod_params->pulse_shape = SX126X_GFSK_PULSE_SHAPE_BT_03;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_05:
    {
        radio_mod_params->pulse_shape = SX126X_GFSK_PULSE_SHAPE_BT_05;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_07:
    {
        radio_mod_params->pulse_shape = SX126X_GFSK_PULSE_SHAPE_BT_07;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_1:
    {
        radio_mod_params->pulse_shape = SX126X_GFSK_PULSE_SHAPE_BT_1;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return status;
}

ral_status_t ral_sx126x_convert_gfsk_pkt_params_from_ral( const ral_gfsk_pkt_params_t* ral_pkt_params,
                                                          sx126x_pkt_params_gfsk_t*    radio_pkt_params )
{
    radio_pkt_params->preamble_len_in_bits = ral_pkt_params->preamble_len_in_bits;

    switch( ral_pkt_params->preamble_detector )
    {
    case RAL_GFSK_PREAMBLE_DETECTOR_OFF:
    {
        radio_pkt_params->preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_OFF;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_8BITS:
    {
        radio_pkt_params->preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS:
    {
        radio_pkt_params->preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_24BITS:
    {
        radio_pkt_params->preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_32BITS:
    {
        radio_pkt_params->preamble_detector = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS;
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
        radio_pkt_params->address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE;
        break;
    }
    case RAL_GFSK_ADDRESS_FILTERING_NODE_ADDRESS:
    {
        radio_pkt_params->address_filtering = SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS;
        break;
    }
    case RAL_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES:
    {
        radio_pkt_params->address_filtering = SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES;
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
        radio_pkt_params->header_type = SX126X_GFSK_PKT_FIX_LEN;
        break;
    }
    case RAL_GFSK_PKT_VAR_LEN:
    {
        radio_pkt_params->header_type = SX126X_GFSK_PKT_VAR_LEN;
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
        radio_pkt_params->crc_type = SX126X_GFSK_CRC_OFF;
        break;
    }
    case RAL_GFSK_CRC_1_BYTE:
    {
        radio_pkt_params->crc_type = SX126X_GFSK_CRC_1_BYTE;
        break;
    }
    case RAL_GFSK_CRC_2_BYTES:
    {
        radio_pkt_params->crc_type = SX126X_GFSK_CRC_2_BYTES;
        break;
    }
    case RAL_GFSK_CRC_1_BYTE_INV:
    {
        radio_pkt_params->crc_type = SX126X_GFSK_CRC_1_BYTE_INV;
        break;
    }
    case RAL_GFSK_CRC_2_BYTES_INV:
    {
        radio_pkt_params->crc_type = SX126X_GFSK_CRC_2_BYTES_INV;
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
        radio_pkt_params->dc_free = SX126X_GFSK_DC_FREE_OFF;
        break;
    }
    case RAL_GFSK_DC_FREE_WHITENING:
    {
        radio_pkt_params->dc_free = SX126X_GFSK_DC_FREE_WHITENING;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_convert_lora_mod_params_from_ral( const ral_lora_mod_params_t* ral_mod_params,
                                                          sx126x_mod_params_lora_t*    radio_mod_params )
{
    radio_mod_params->sf = ( sx126x_lora_sf_t ) ral_mod_params->sf;

    switch( ral_mod_params->bw )
    {
    case RAL_LORA_BW_007_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_007;
        break;
    }
    case RAL_LORA_BW_010_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_010;
        break;
    }
    case RAL_LORA_BW_015_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_015;
        break;
    }
    case RAL_LORA_BW_020_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_020;
        break;
    }
    case RAL_LORA_BW_031_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_031;
        break;
    }
    case RAL_LORA_BW_041_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_041;
        break;
    }
    case RAL_LORA_BW_062_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_062;
        break;
    }
    case RAL_LORA_BW_125_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_125;
        break;
    }
    case RAL_LORA_BW_250_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_250;
        break;
    }
    case RAL_LORA_BW_500_KHZ:
    {
        radio_mod_params->bw = SX126X_LORA_BW_500;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_mod_params->cr = ( sx126x_lora_cr_t ) ral_mod_params->cr;

    radio_mod_params->ldro = ral_mod_params->ldro;

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_convert_lora_pkt_params_from_ral( const ral_lora_pkt_params_t* ral_pkt_params,
                                                          sx126x_pkt_params_lora_t*    radio_pkt_params )
{
    radio_pkt_params->preamble_len_in_symb = ral_pkt_params->preamble_len_in_symb;

    switch( ral_pkt_params->header_type )
    {
    case( RAL_LORA_PKT_EXPLICIT ):
    {
        radio_pkt_params->header_type = SX126X_LORA_PKT_EXPLICIT;
        break;
    }
    case( RAL_LORA_PKT_IMPLICIT ):
    {
        radio_pkt_params->header_type = SX126X_LORA_PKT_IMPLICIT;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_pkt_params->pld_len_in_bytes = ral_pkt_params->pld_len_in_bytes;
    radio_pkt_params->crc_is_on        = ral_pkt_params->crc_is_on;
    radio_pkt_params->invert_iq_is_on  = ral_pkt_params->invert_iq_is_on;

    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_convert_lora_cr_to_ral( const sx126x_lora_cr_t radio_cr, ral_lora_cr_t* ral_cr )
{
    switch( radio_cr )
    {
    case SX126X_LORA_CR_4_5:
        *ral_cr = RAL_LORA_CR_4_5;
        break;
    case SX126X_LORA_CR_4_6:
        *ral_cr = RAL_LORA_CR_4_6;
        break;
    case SX126X_LORA_CR_4_7:
        *ral_cr = RAL_LORA_CR_4_7;
        break;
    case SX126X_LORA_CR_4_8:
        *ral_cr = RAL_LORA_CR_4_8;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    return RAL_STATUS_OK;
}

ral_status_t ral_sx126x_convert_lora_cad_params_from_ral( const ral_lora_cad_params_t* ral_lora_cad_params,
                                                          sx126x_cad_params_t*         radio_lora_cad_params )
{
    switch( ral_lora_cad_params->cad_symb_nb )
    {
    case RAL_LORA_CAD_01_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = SX126X_CAD_01_SYMB;
        break;
    }
    case RAL_LORA_CAD_02_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = SX126X_CAD_02_SYMB;
        break;
    }
    case RAL_LORA_CAD_04_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = SX126X_CAD_04_SYMB;
        break;
    }
    case RAL_LORA_CAD_08_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = SX126X_CAD_08_SYMB;
        break;
    }
    case RAL_LORA_CAD_16_SYMB:
    {
        radio_lora_cad_params->cad_symb_nb = SX126X_CAD_16_SYMB;
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
        radio_lora_cad_params->cad_exit_mode = SX126X_CAD_ONLY;
        break;
    }
    case RAL_LORA_CAD_RX:
    {
        radio_lora_cad_params->cad_exit_mode = SX126X_CAD_RX;
        break;
    }
    case RAL_LORA_CAD_LBT:
    {
        radio_lora_cad_params->cad_exit_mode = SX126X_CAD_LBT;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_lora_cad_params->cad_timeout =
        sx126x_convert_timeout_in_ms_to_rtc_step( ral_lora_cad_params->cad_timeout_in_ms );

    return RAL_STATUS_OK;
}

void ral_sx126x_convert_lr_fhss_params_from_ral( const ral_lr_fhss_params_t* ral_lr_fhss_params,
                                                 sx126x_lr_fhss_params_t*    radio_lr_fhss_params )
{
    *radio_lr_fhss_params = ( sx126x_lr_fhss_params_t ){
        .lr_fhss_params           = ral_lr_fhss_params->lr_fhss_params,
        .center_freq_in_pll_steps = sx126x_convert_freq_in_hz_to_pll_step( ral_lr_fhss_params->center_frequency_in_hz ),
        .device_offset            = ral_lr_fhss_params->device_offset,
    };
}

static ral_status_t ral_sx126x_cfg_trim_cap( const void* context )
{
    sx126x_tcxo_ctrl_voltages_t tcxo_supply_voltage;
    ral_xosc_cfg_t              xosc_cfg;
    uint32_t                    startup_time_in_tick = 0;

    ral_sx126x_bsp_get_xosc_cfg( context, &xosc_cfg, &tcxo_supply_voltage, &startup_time_in_tick );

    if( xosc_cfg == RAL_XOSC_CFG_XTAL )
    {
        uint8_t trimming_cap_xta = SX126X_XTAL_TRIMMING_CAPACITOR_DEFAULT_VALUE_STDBY_XOSC;
        uint8_t trimming_cap_xtb = SX126X_XTAL_TRIMMING_CAPACITOR_DEFAULT_VALUE_STDBY_XOSC;

        ral_sx126x_bsp_get_trim_cap( context, &trimming_cap_xta, &trimming_cap_xtb );

        if( ( trimming_cap_xta != SX126X_XTAL_TRIMMING_CAPACITOR_DEFAULT_VALUE_STDBY_XOSC ) ||
            ( trimming_cap_xtb != SX126X_XTAL_TRIMMING_CAPACITOR_DEFAULT_VALUE_STDBY_XOSC ) )
        {
            sx126x_status_t      status;
            sx126x_chip_status_t chip_status;

            status = sx126x_get_status( context, &chip_status );
            if( status != SX126X_STATUS_OK )
            {
                return ( ral_status_t ) status;
            }

            if( chip_status.chip_mode == SX126X_CHIP_MODE_STBY_RC )
            {
                status = sx126x_set_standby( context, SX126X_STANDBY_CFG_XOSC );
                if( status != SX126X_STATUS_OK )
                {
                    return ( ral_status_t ) status;
                }
            }

            status = sx126x_set_trimming_capacitor_values( context, trimming_cap_xta, trimming_cap_xtb );
            if( status != SX126X_STATUS_OK )
            {
                return ( ral_status_t ) status;
            }
        }
    }

    return RAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
