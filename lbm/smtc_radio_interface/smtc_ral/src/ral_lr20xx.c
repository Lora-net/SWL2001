/**
 * @file      ral_lr20xx.c
 *
 * @brief     Radio abstraction layer definition
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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
#include "lr20xx_pram.h"
#include "lr20xx_system.h"
#include "lr20xx_system_types.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_common_types.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_fsk.h"
#include "lr20xx_radio_flrc.h"
#include "lr20xx_radio_lr_fhss.h"
#include "lr20xx_rttof.h"
#include "lr20xx_workarounds.h"
#include "ral_lr20xx.h"
#include "ral_defs.h"
#include "ral_lr20xx_bsp.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const ral_lora_bw_t RAL_LR20XX_SUPPORTED_BW[] = { RAL_LORA_BW_031_KHZ,  RAL_LORA_BW_041_KHZ, RAL_LORA_BW_062_KHZ,
                                                  RAL_LORA_BW_125_KHZ,  RAL_LORA_BW_200_KHZ, RAL_LORA_BW_250_KHZ,
                                                  RAL_LORA_BW_400_KHZ,  RAL_LORA_BW_500_KHZ, RAL_LORA_BW_800_KHZ,
                                                  RAL_LORA_BW_1000_KHZ, RAL_LORA_BW_083_KHZ, RAL_LORA_BW_101_KHZ };

const uint8_t RAL_LR20XX_SUPPORTED_BW_SIZE =
    ( sizeof( RAL_LR20XX_SUPPORTED_BW ) / sizeof( RAL_LR20XX_SUPPORTED_BW[0] ) );

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
 * @brief Convert interrupt flags from Lr20xx context to RAL context
 *
 * @param [in] lr20xx_irq  Lr20xx interrupt status
 *
 * @returns RAL interrupt status
 */
static ral_irq_t ral_lr20xx_convert_irq_flags_to_ral( lr20xx_system_irq_mask_t lr20xx_irq );

/**
 * @brief Convert interrupt flags from RAL context to Lr20xx context
 *
 * @param [in] ral_irq RAL interrupt status
 *
 * @returns Lr20xx interrupt status
 */
static lr20xx_system_irq_mask_t ral_lr20xx_convert_irq_flags_from_ral( ral_irq_t ral_irq );

/**
 * @brief Convert fifo interrupt flags from Lr20xx context to RAL context
 *
 * @param [in] lr20xx_fifo_irq_status Lr20xx fifo interrupt status
 *
 * @returns Lr20xx interrupt status
 */
static ral_radio_fifo_flag_t ral_lr20xx_convert_fifo_irq_flags_to_ral(
    lr20xx_radio_fifo_flag_t lr20xx_fifo_irq_status );

/**
 * @brief Convert fifo interrupt flags from RAL context to Lr20xx context
 *
 * @param [in] ral_fifo_irq RAL fifo interrupt status
 *
 * @returns Lr20xx interrupt status
 */
static lr20xx_radio_fifo_flag_t ral_lr20xx_convert_irq_fifo_flags_from_ral( ral_radio_fifo_flag_t ral_fifo_irq );

/**
 * @brief Convert GFSK modulation parameters from RAL context to Lr20xx context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 * @returns Operation status
 */
static ral_status_t ral_lr20xx_convert_gfsk_mod_params_from_ral( const ral_gfsk_mod_params_t*   ral_mod_params,
                                                                 lr20xx_radio_fsk_mod_params_t* radio_mod_params );

/**
 * @brief Convert GFSK packet parameters from RAL context to Lr20xx context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 *
 * @returns Operation status
 */
static ral_status_t ral_lr20xx_convert_gfsk_pkt_params_from_ral( const ral_gfsk_pkt_params_t*   ral_pkt_params,
                                                                 lr20xx_radio_fsk_pkt_params_t* radio_pkt_params );

/**
 * @brief Convert LoRa modulation parameters from RAL context to Lr20xx context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 * @returns Operation status
 */
static ral_status_t ral_lr20xx_convert_lora_mod_params_from_ral( const ral_lora_mod_params_t*    ral_mod_params,
                                                                 lr20xx_radio_lora_mod_params_t* radio_mod_params );

/**
 * @brief Convert LoRa packet parameters from RAL context to Lr20xx context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 *
 * @returns Operation status
 */
static ral_status_t ral_lr20xx_convert_lora_pkt_params_from_ral( const ral_lora_pkt_params_t*    ral_pkt_params,
                                                                 lr20xx_radio_lora_pkt_params_t* radio_pkt_params );

/**
 * @brief Convert LoRa CAD parameters from RAL context to Lr20xx context
 *
 * @param [in] ral_lora_cad_params     RAL LoRa CAD parameters
 * @param [out] radio_lora_cad_params  Radio LoRa CAD parameters
 *
 * @returns Operation status
 */
static ral_status_t ral_lr20xx_convert_lora_cad_params_from_ral(
    const ral_lora_cad_params_t* ral_lora_cad_params, lr20xx_radio_lora_cad_params_t* radio_lora_cad_params );

/**
 * @brief Convert LR-FHSS parameters from RAL context to Lr20xx context
 *
 * @param [in] ral_lr_fhss_params     RAL LR-FHSS parameters
 * @param [in] hop_sequence_id The hop sequence ID to set in the radio LR-FHSS parameters structure
 * @param [out] radio_lr_fhss_params  Radio LR-FHSS parameters
 *
 * @returns Operation status
 */
static ral_status_t ral_lr20xx_convert_lr_fhss_params_from_ral( const ral_lr_fhss_params_t*    ral_lr_fhss_params,
                                                                uint16_t                       hop_sequence_id,
                                                                lr20xx_radio_lr_fhss_params_t* radio_lr_fhss_params );

/**
 * @brief Convert FLRC modulation parameters from RAL context to LR20xx context
 *
 * @param [in] ral_mod_params     RAL modulation parameters
 * @param [out] radio_mod_params  Radio modulation parameters
 *
 */
static ral_status_t ral_lr20xx_convert_flrc_mod_params_from_ral( const ral_flrc_mod_params_t*    ral_mod_params,
                                                                 lr20xx_radio_flrc_mod_params_t* radio_mod_params );

/**
 * @brief Convert FLRC packet parameters from RAL context to LR20xx context
 *
 * @param [in] ral_pkt_params     RAL packet parameters
 * @param [out] radio_pkt_params  Radio packet parameters
 */
static ral_status_t ral_lr20xx_convert_flrc_pkt_params_from_ral( const ral_flrc_pkt_params_t*    ral_pkt_params,
                                                                 lr20xx_radio_flrc_pkt_params_t* radio_pkt_params );

/**
 * @brief Load and enable LR20xx PRAM
 *
 * The behavior of this function is compile definition dependent. It needs to have either LR2012, LR2021 or LR2022
 * macro defined.
 */
static ral_status_t ral_lr20xx_load_pram( const void* context );

/**
 * @brief Check LR20xx PRAM version integrate in chip
 *
 * The behavior of this function is compile definition dependent. It needs to have either LR2012, LR2021 or LR2022
 * macro defined.
 */
static ral_status_t ral_lr20xx_check_pram_version( const void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool ral_lr20xx_handles_part( const char* part_number )
{
    return ( strcmp( "lr20xx", part_number ) == 0 );
}

ral_status_t ral_lr20xx_reset( const void* context )
{
    return ( ral_status_t ) lr20xx_system_reset( context );
}

ral_status_t ral_lr20xx_wakeup( const void* context )
{
    return ( ral_status_t ) lr20xx_system_wakeup( context );
}

ral_status_t ral_lr20xx_init( const void* context )
{
    ral_status_t status = RAL_STATUS_ERROR;

    status = ral_lr20xx_load_pram( context );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ral_lr20xx_check_pram_version( context );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    lr20xx_system_reg_mode_t reg_mode;
    ral_lr20xx_bsp_get_reg_mode( context, &reg_mode );
    status = ( ral_status_t ) lr20xx_system_set_reg_mode( context, reg_mode );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_xosc_cfg_t                      xosc_cfg;
    lr20xx_system_tcxo_supply_voltage_t tcxo_supply_voltage;
    uint32_t                            startup_time_in_tick = 0;
    ral_lr20xx_bsp_get_xosc_cfg( context, &xosc_cfg, &tcxo_supply_voltage, &startup_time_in_tick );
    if( xosc_cfg == RAL_XOSC_CFG_TCXO_RADIO_CTRL )
    {
        status = ( ral_status_t ) lr20xx_system_set_tcxo_mode( context, tcxo_supply_voltage, startup_time_in_tick );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
        // Calibration
        lr20xx_system_clear_errors( context );     // Clear the errors first
        lr20xx_system_calibrate( context, 0x6F );  // Must do the calibration after the TCXO works steadily
    }

    lr20xx_system_lfclk_cfg_t lfclk_cfg;
    ral_bsp_lr20xx_get_lfclk_cfg( context, &lfclk_cfg );
    status = ( ral_status_t ) lr20xx_system_cfg_lfclk( context, lfclk_cfg );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    uint16_t errors;
    lr20xx_system_get_errors( context, &errors );
    if( errors != 0 )
    {
        lr20xx_system_clear_errors( context );
    }

    uint8_t dio_count = lr20xx_system_dio_get_count( );
    for( uint8_t dio_nth = 0; dio_nth < dio_count; dio_nth++ )
    {
        lr20xx_system_dio_t       dio;
        lr20xx_system_dio_func_t  dio_function = LR20XX_SYSTEM_DIO_FUNC_NONE;
        lr20xx_system_dio_drive_t dio_drive    = LR20XX_SYSTEM_DIO_DRIVE_NONE;
        if( !lr20xx_system_dio_get_nth( dio_nth, &dio ) )
        {
            return RAL_STATUS_ERROR;
        }

        ral_lr20xx_bsp_get_dio_function( context, dio, &dio_function );
        ral_lr20xx_bsp_get_dio_sleep_drive( context, dio, &dio_drive );

        status = ( ral_status_t ) lr20xx_system_set_dio_function( context, dio, dio_function, dio_drive );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }

        if( dio_function == LR20XX_SYSTEM_DIO_FUNC_IRQ )
        {
            /* FIXME: We might not want to set the IRQs at init, but wait for
             * ral_lr20xx_set_dio_irq_params to be called.
            lr20xx_system_irq_mask_t irq_mask;
            ral_lr20xx_bsp_get_dio_irq_mask( context, dio, &irq_mask);
            status = lr20xx_system_set_dio_irq_cfg( context, dio, irq_mask );
            if( status != RAL_STATUS_OK )
            {
                return status;
            }
            */
        }

        if( dio_function == LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH )
        {
            lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg = 0;
            ral_lr20xx_bsp_get_dio_rf_switch_cfg( context, dio, &rf_switch_cfg );
            status = ( ral_status_t ) lr20xx_system_set_dio_rf_switch_cfg( context, dio, rf_switch_cfg );
            if( status != RAL_STATUS_OK )
            {
                return status;
            }
        }

        if( dio_function == LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT )
        {
            lr20xx_system_hf_clk_scaling_t hf_clk_scaling = LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ;
            ral_lr20xx_bsp_get_dio_hf_clk_scaling_cfg( context, &hf_clk_scaling );
            status = ( ral_status_t ) lr20xx_system_cfg_clk_output( context, hf_clk_scaling );
            if( status != RAL_STATUS_OK )
            {
                return status;
            }
        }
    }

    // Front end calibration
    lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures[3] = { 0 };
    ral_lr20xx_bsp_get_front_end_calibration_cfg( context, front_end_calibration_structures );
    status =
        ( ral_status_t ) lr20xx_radio_common_calibrate_front_end_helper( context, front_end_calibration_structures, 3 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    // RSSI calibration
    const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_calibration_table_hf = NULL;
    const lr20xx_radio_common_rssi_calibration_gain_table_t* rssi_calibration_table_lf = NULL;
    ral_lr20xx_bsp_get_rssi_calibration_table( context, &rssi_calibration_table_lf, &rssi_calibration_table_hf );
    status = ( ral_status_t ) lr20xx_radio_common_set_rssi_calibration( context, rssi_calibration_table_lf,
                                                                        rssi_calibration_table_hf );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

#if defined( LR20XX_FIFO_SIZE_INCREASE_WORKAROUND_ENABLE )
    status = ( ral_status_t ) lr20xx_radio_fifo_configure_1024_byte_tx_fifo( context );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ( ral_status_t ) lr20xx_radio_fifo_configure_1024_byte_rx_fifo( context );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ( ral_status_t ) lr20xx_radio_fifo_1024_byte_tx_fifo_store_retention_mem( context, 0, 1 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ( ral_status_t ) lr20xx_radio_fifo_1024_byte_rx_fifo_store_retention_mem( context, 2, 3 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    status = ( ral_status_t ) lr20xx_workarounds_1024_byte_fifo_cfg_irq_store_retention_mem( context, 4, 5 );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
#endif  // LR20XX_FIFO_SIZE_INCREASE_WORKAROUND_ENABLE

    return status;
}

ral_status_t ral_lr20xx_set_sleep( const void* context, const bool retain_config )
{
    const lr20xx_system_sleep_cfg_t radio_sleep_cfg = {
        .is_ram_retention_enabled = retain_config,
        .is_clk_32k_enabled       = false,
    };

    return ( ral_status_t ) lr20xx_system_set_sleep_mode( context, &radio_sleep_cfg, 0 );
}

ral_status_t ral_lr20xx_set_standby( const void* context, ral_standby_cfg_t ral_standby_cfg )
{
    lr20xx_system_standby_mode_t radio_standby_cfg;

    switch( ral_standby_cfg )
    {
    case RAL_STANDBY_CFG_RC:
    {
        radio_standby_cfg = LR20XX_SYSTEM_STANDBY_MODE_RC;
        break;
    }
    case RAL_STANDBY_CFG_XOSC:
    {
        radio_standby_cfg = LR20XX_SYSTEM_STANDBY_MODE_XOSC;
        break;
    }
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return ( ral_status_t ) lr20xx_system_set_standby_mode( context, radio_standby_cfg );
}

ral_status_t ral_lr20xx_set_fs( const void* context )
{
    return ( ral_status_t ) lr20xx_system_set_fs_mode( context );
}

ral_status_t ral_lr20xx_set_tx( const void* context )
{
    return ( ral_status_t ) lr20xx_radio_common_set_tx( context, 0 );
}

ral_status_t ral_lr20xx_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    lr20xx_radio_fifo_clear_rx( context );
    if( timeout_in_ms == RAL_RX_TIMEOUT_CONTINUOUS_MODE )
    {
        return ( ral_status_t ) lr20xx_radio_common_set_rx_with_timeout_in_rtc_step( context, 0x00FFFFFF );
    }
    else
    {  // max timeout is 0xFFFFFE -> 511999 ms (0xFFFFFE / 32768 * 1000) - Single reception mode set if timeout_ms is 0
        if( timeout_in_ms < 512000 )
        {
            return ( ral_status_t ) lr20xx_radio_common_set_rx( context, timeout_in_ms );
        }
        else
        {
            return RAL_STATUS_ERROR;
        }
    }

    return RAL_STATUS_ERROR;
}

ral_status_t ral_lr20xx_cfg_rx_boosted( const void* context, const bool enable_boost_mode )
{
    // return ( ral_status_t ) lr20xx_radio_common_set_rx_boost_mode( context, enable_boost_mode, enable_boost_mode );
    return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr20xx_set_rx_tx_fallback_mode( const void* context, const ral_fallback_modes_t ral_fallback_mode )
{
    lr20xx_radio_common_fallback_modes_t radio_fallback_mode;

    switch( ral_fallback_mode )
    {
    case RAL_FALLBACK_STDBY_RC:
    {
        radio_fallback_mode = LR20XX_RADIO_FALLBACK_STDBY_RC;
        break;
    }
    case RAL_FALLBACK_STDBY_XOSC:
    {
        radio_fallback_mode = LR20XX_RADIO_FALLBACK_STDBY_XOSC;
        break;
    }
    case RAL_FALLBACK_FS:
    {
        radio_fallback_mode = LR20XX_RADIO_FALLBACK_FS;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return ( ral_status_t ) lr20xx_radio_common_set_rx_tx_fallback_mode( context, radio_fallback_mode );
}

ral_status_t ral_lr20xx_stop_timer_on_preamble( const void* context, const bool enable )
{
    return ( ral_status_t ) lr20xx_radio_common_set_rx_timeout_stop_event( context, enable );
}

ral_status_t ral_lr20xx_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                           const uint32_t sleep_time_in_ms )
{
    return ( ral_status_t ) lr20xx_radio_common_set_rx_duty_cycle( context, rx_time_in_ms, sleep_time_in_ms,
                                                                   LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_RX );
}

ral_status_t ral_lr20xx_set_lora_cad( const void* context )
{
    return ( ral_status_t ) lr20xx_radio_lora_set_cad( context );
}

ral_status_t ral_lr20xx_set_tx_cw( const void* context )
{
    return ( ral_status_t ) lr20xx_radio_common_set_tx_test_mode( context,
                                                                  LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE );
}

ral_status_t ral_lr20xx_set_tx_infinite_preamble( const void* context )
{
    return ( ral_status_t ) lr20xx_radio_common_set_tx_test_mode( context,
                                                                  LR20XX_RADIO_COMMON_TX_TEST_MODE_INFINITE_PREAMBLE );
}

ral_status_t ral_lr20xx_cal_img( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    const lr20xx_radio_common_front_end_calibration_value_t image_calibration_structures[2] = {
        { .rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF, .frequency_in_hertz = freq1_in_mhz * 1e6 },
        { .rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF, .frequency_in_hertz = freq2_in_mhz * 1e6 }
    };
    return ( ral_status_t ) lr20xx_radio_common_calibrate_front_end_helper( context, image_calibration_structures, 2 );
}

ral_status_t ral_lr20xx_set_tx_cfg( const void* context, const int8_t output_pwr_in_dbm, const uint32_t rf_freq_in_hz )
{
    ral_status_t                               status = RAL_STATUS_ERROR;
    ral_lr20xx_bsp_tx_cfg_output_params_t      tx_cfg_output_params;
    const ral_lr20xx_bsp_tx_cfg_input_params_t tx_cfg_input_params = {
        .freq_in_hz               = rf_freq_in_hz,
        .system_output_pwr_in_dbm = output_pwr_in_dbm,
    };

    ral_lr20xx_bsp_get_tx_cfg( context, &tx_cfg_input_params, &tx_cfg_output_params );

    status = ( ral_status_t ) lr20xx_radio_common_set_pa_cfg( context, &tx_cfg_output_params.pa_cfg );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    status = ( ral_status_t ) lr20xx_radio_common_set_tx_params(
        context, tx_cfg_output_params.chip_output_half_pwr_in_dbm_configured, tx_cfg_output_params.pa_ramp_time );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return status;
}

ral_status_t ral_lr20xx_set_pkt_payload( const void* context, const uint8_t* buffer, const uint16_t size )
{
    return ( ral_status_t ) lr20xx_radio_fifo_write_tx( context, buffer, size );
}

ral_status_t ral_lr20xx_get_pkt_payload( const void* context, uint16_t max_size_in_bytes, uint8_t* buffer,
                                         uint16_t* size_in_bytes )
{
    ral_status_t status = RAL_STATUS_ERROR;
    uint16_t     pkt_len;

    status = ( ral_status_t ) lr20xx_radio_common_get_rx_packet_length( context, &pkt_len );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    if( size_in_bytes != 0 )
    {
        *size_in_bytes = pkt_len;
    }

    if( ( pkt_len <= max_size_in_bytes ) && ( pkt_len <= RAL_LR20XX_MAX_DATA_FIFO_BYTES ) )
    {
        status = ( ral_status_t ) lr20xx_radio_fifo_read_rx( context, buffer, pkt_len );
    }
    else
    {
        status = RAL_STATUS_ERROR;
    }

    return status;
}

ral_status_t ral_lr20xx_get_pkt_size( const void* context, uint16_t* size_in_bytes )
{
    if( size_in_bytes == 0 )
    {
        return RAL_STATUS_ERROR;
    }

    return ( ral_status_t ) lr20xx_radio_common_get_rx_packet_length( context, size_in_bytes );
}

ral_status_t ral_lr20xx_get_data_rx_buffer( const void* context, uint8_t* buffer, uint16_t size_in_bytes )
{
    if( size_in_bytes > RAL_LR20XX_MAX_DATA_FIFO_BYTES )
    {
        return RAL_STATUS_ERROR;
    }
    return ( ral_status_t ) lr20xx_radio_fifo_read_rx( context, buffer, size_in_bytes );
}

ral_status_t ral_lr20xx_clear_rx_fifo( const void* context )
{
    return ( ral_status_t ) lr20xx_radio_fifo_clear_rx( context );
}

ral_status_t ral_lr20xx_clear_tx_fifo( const void* context )
{
    return ( ral_status_t ) lr20xx_radio_fifo_clear_tx( context );
}

ral_status_t ral_lr20xx_get_tx_fifo_level( const void* context, uint16_t* fifo_level )
{
    return ( ral_status_t ) lr20xx_radio_fifo_get_tx_level( context, fifo_level );
}

ral_status_t ral_lr20xx_get_rx_fifo_level( const void* context, uint16_t* fifo_level )
{
    return ( ral_status_t ) lr20xx_radio_fifo_get_rx_level( context, fifo_level );
}

ral_status_t ral_lr20xx_cfg_fifo_irq( const void* context, ral_radio_fifo_flag_t rx_fifo_irq_enable,
                                      ral_radio_fifo_flag_t tx_fifo_irq_enable, uint16_t rx_fifo_high_threshold,
                                      uint16_t tx_fifo_low_threshold, uint16_t rx_fifo_low_threshold,
                                      uint16_t tx_fifo_high_threshold )
{
#if defined( LR20XX_FIFO_SIZE_INCREASE_WORKAROUND_ENABLE )
    return ( ral_status_t ) lr20xx_workarounds_1024_byte_fifo_cfg_irq(
        context, ral_lr20xx_convert_irq_fifo_flags_from_ral( rx_fifo_irq_enable ),
        ral_lr20xx_convert_irq_fifo_flags_from_ral( tx_fifo_irq_enable ), rx_fifo_high_threshold, tx_fifo_low_threshold,
        rx_fifo_low_threshold, tx_fifo_high_threshold );
#else
    return ( ral_status_t ) lr20xx_radio_fifo_cfg_irq(
        context, ral_lr20xx_convert_irq_fifo_flags_from_ral( rx_fifo_irq_enable ),
        ral_lr20xx_convert_irq_fifo_flags_from_ral( tx_fifo_irq_enable ), rx_fifo_high_threshold, tx_fifo_low_threshold,
        rx_fifo_low_threshold, tx_fifo_high_threshold );
#endif  // LR20XX_FIFO_SIZE_INCREASE_WORKAROUND_ENABLE
}

ral_status_t ral_lr20xx_get_fifo_irq( const void* context, ral_radio_fifo_flag_t* rx_fifo_flags,
                                      ral_radio_fifo_flag_t* tx_fifo_flags )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    lr20xx_radio_fifo_flag_t lr20xx_rx_fifo_flags;
    lr20xx_radio_fifo_flag_t lr20xx_tx_fifo_flags;

    status = ( ral_status_t ) lr20xx_radio_fifo_get_irq( context, &lr20xx_rx_fifo_flags, &lr20xx_tx_fifo_flags );
    if( status == RAL_STATUS_OK )
    {
        *rx_fifo_flags = ral_lr20xx_convert_fifo_irq_flags_to_ral( lr20xx_rx_fifo_flags );
        *tx_fifo_flags = ral_lr20xx_convert_fifo_irq_flags_to_ral( lr20xx_tx_fifo_flags );
    }

    return status;
}

ral_status_t ral_lr20xx_clear_fifo_irq( const void* context, ral_radio_fifo_flag_t rx_fifo_flags_to_clear,
                                        ral_radio_fifo_flag_t tx_fifo_flags_to_clear )
{
    return ( ral_status_t ) lr20xx_radio_fifo_clear_irq_flags(
        context, ral_lr20xx_convert_irq_fifo_flags_from_ral( rx_fifo_flags_to_clear ),
        ral_lr20xx_convert_irq_fifo_flags_from_ral( tx_fifo_flags_to_clear ) );
}

ral_status_t ral_lr20xx_get_and_clear_fifo_irq( const void* context, ral_radio_fifo_flag_t* rx_fifo_flags,
                                                ral_radio_fifo_flag_t* tx_fifo_flags )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    lr20xx_radio_fifo_flag_t lr20xx_rx_fifo_flags;
    lr20xx_radio_fifo_flag_t lr20xx_tx_fifo_flags;

    status = ( ral_status_t ) lr20xx_radio_fifo_get_and_clear_irq_flags( context, &lr20xx_rx_fifo_flags,
                                                                         &lr20xx_tx_fifo_flags );
    if( status == RAL_STATUS_OK )
    {
        *rx_fifo_flags = ral_lr20xx_convert_fifo_irq_flags_to_ral( lr20xx_rx_fifo_flags );
        *tx_fifo_flags = ral_lr20xx_convert_fifo_irq_flags_to_ral( lr20xx_tx_fifo_flags );
    }

    return status;
}

ral_status_t ral_lr20xx_get_irq_status( const void* context, ral_irq_t* irq )
{
    ral_status_t             status = RAL_STATUS_ERROR;
    lr20xx_system_irq_mask_t radio_irq_mask;
    lr20xx_system_stat1_t    stat1;
    lr20xx_system_stat2_t    stat2;

    status = ( ral_status_t ) lr20xx_system_get_status( context, &stat1, &stat2, &radio_irq_mask );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    *irq = ral_lr20xx_convert_irq_flags_to_ral( radio_irq_mask );

    return status;
}

ral_status_t ral_lr20xx_clear_irq_status( const void* context, const ral_irq_t irq )
{
    const lr20xx_system_irq_mask_t radio_irq = ral_lr20xx_convert_irq_flags_from_ral( irq );

    return ( ral_status_t ) lr20xx_system_clear_irq_status( context, radio_irq );
}

ral_status_t ral_lr20xx_get_and_clear_irq_status( const void* context, ral_irq_t* irq )
{
    ral_status_t             status    = RAL_STATUS_ERROR;
    lr20xx_system_irq_mask_t radio_irq = LR20XX_SYSTEM_IRQ_NONE;

    status = ( ral_status_t ) lr20xx_system_get_and_clear_irq_status( context, &radio_irq );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    if( irq != 0 )
    {
        *irq = ral_lr20xx_convert_irq_flags_to_ral( radio_irq );
    }

    return status;
}

ral_status_t ral_lr20xx_set_dio_irq_params( const void* context, const ral_irq_t irq )
{
    ral_status_t             status            = RAL_STATUS_ERROR;
    lr20xx_system_irq_mask_t lr20xx_irq        = ral_lr20xx_convert_irq_flags_from_ral( irq );
    lr20xx_system_irq_mask_t all_dios_irq_mask = 0;

    uint8_t dio_count = lr20xx_system_dio_get_count( );
    for( uint8_t dio_nth = 0; dio_nth < dio_count; dio_nth++ )
    {
        lr20xx_system_dio_t      dio;
        lr20xx_system_dio_func_t dio_function = LR20XX_SYSTEM_DIO_FUNC_NONE;
        lr20xx_system_irq_mask_t dio_irq_mask = LR20XX_SYSTEM_IRQ_NONE;

        if( !lr20xx_system_dio_get_nth( dio_nth, &dio ) )
        {
            return RAL_STATUS_ERROR;
        }
        ral_lr20xx_bsp_get_dio_function( context, dio, &dio_function );
        if( dio_function != LR20XX_SYSTEM_DIO_FUNC_IRQ )
        {
            continue;
        }

        ral_lr20xx_bsp_get_dio_irq_mask( context, dio, &dio_irq_mask );

        dio_irq_mask &= lr20xx_irq;

        all_dios_irq_mask |= dio_irq_mask;
        status = ( ral_status_t ) lr20xx_system_set_dio_irq_cfg( context, dio, dio_irq_mask );
        if( status != RAL_STATUS_OK )
        {
            return RAL_STATUS_ERROR;
        }
    }

    if( all_dios_irq_mask != lr20xx_irq )
    {
        /* Couldn't apply the expected IRQ mask, the BSP is misconfigured */
        return RAL_STATUS_ERROR;
    }

    return ( ral_status_t ) status;
}

ral_status_t ral_lr20xx_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    ral_status_t status = RAL_STATUS_ERROR;
    status              = ( ral_status_t ) lr20xx_radio_common_set_rf_freq( context, freq_in_hz );
    if( status != RAL_STATUS_OK )
    {
        return RAL_STATUS_ERROR;
    }

    lr20xx_radio_common_rx_path_t            rx_path    = LR20XX_RADIO_COMMON_RX_PATH_LF;
    lr20xx_radio_common_rx_path_boost_mode_t boost_mode = LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE;

    ral_lr20xx_bsp_get_rx_cfg( context, freq_in_hz, &rx_path, &boost_mode );

    return ( ral_status_t ) lr20xx_radio_common_set_rx_path( context, rx_path, boost_mode );
}

ral_status_t ral_lr20xx_set_pkt_type( const void* context, const ral_pkt_type_t ral_pkt_type )
{
    lr20xx_radio_common_pkt_type_t radio_pkt_type;

    switch( ral_pkt_type )
    {
    case RAL_PKT_TYPE_GFSK:
    {
        radio_pkt_type = LR20XX_RADIO_COMMON_PKT_TYPE_FSK;
        break;
    }
    case RAL_PKT_TYPE_LORA:
    {
        radio_pkt_type = LR20XX_RADIO_COMMON_PKT_TYPE_LORA;
        break;
    }
    case RAL_PKT_TYPE_FLRC:
    {
        radio_pkt_type = LR20XX_RADIO_COMMON_PKT_TYPE_FLRC;
        break;
    }
    case RAL_PKT_TYPE_RTTOF:
    {
        radio_pkt_type = LR20XX_RADIO_COMMON_PKT_TYPE_RTTOF;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return ( ral_status_t ) lr20xx_radio_common_set_pkt_type( context, radio_pkt_type );
}

ral_status_t ral_lr20xx_get_pkt_type( const void* context, ral_pkt_type_t* pkt_type )
{
    ral_status_t                   status = RAL_STATUS_ERROR;
    lr20xx_radio_common_pkt_type_t radio_pkt_type;

    status = ( ral_status_t ) lr20xx_radio_common_get_pkt_type( context, &radio_pkt_type );
    if( status == RAL_STATUS_OK )
    {
        switch( radio_pkt_type )
        {
        case LR20XX_RADIO_COMMON_PKT_TYPE_FSK:
        {
            *pkt_type = RAL_PKT_TYPE_GFSK;
            break;
        }
        case LR20XX_RADIO_COMMON_PKT_TYPE_LORA:
        {
            *pkt_type = RAL_PKT_TYPE_LORA;
            break;
        }
        case LR20XX_RADIO_COMMON_PKT_TYPE_FLRC:
        {
            *pkt_type = RAL_PKT_TYPE_FLRC;
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

ral_status_t ral_lr20xx_set_gfsk_mod_params( const void* context, const ral_gfsk_mod_params_t* ral_mod_params )
{
    ral_status_t                  status           = RAL_STATUS_ERROR;
    lr20xx_radio_fsk_mod_params_t radio_mod_params = { 0 };

    status = ral_lr20xx_convert_gfsk_mod_params_from_ral( ral_mod_params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr20xx_radio_fsk_set_modulation_params( context, &radio_mod_params );
}

ral_status_t ral_lr20xx_set_gfsk_pkt_params( const void* context, const ral_gfsk_pkt_params_t* ral_pkt_params )
{
    ral_status_t                  status           = RAL_STATUS_ERROR;
    lr20xx_radio_fsk_pkt_params_t radio_pkt_params = { 0 };

    status = ral_lr20xx_convert_gfsk_pkt_params_from_ral( ral_pkt_params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr20xx_radio_fsk_set_packet_params( context, &radio_pkt_params );
}

ral_status_t ral_lr20xx_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                              const uint8_t braodcast_address )
{
    return ( ral_status_t ) lr20xx_radio_fsk_set_addresses( context, node_address, braodcast_address );
}

ral_status_t ral_lr20xx_set_lora_mod_params( const void* context, const ral_lora_mod_params_t* ral_mod_params )
{
    ral_status_t                   status = RAL_STATUS_ERROR;
    lr20xx_radio_lora_mod_params_t radio_mod_params;

    status = ral_lr20xx_convert_lora_mod_params_from_ral( ral_mod_params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr20xx_radio_lora_set_modulation_params( context, &radio_mod_params );
}

ral_status_t ral_lr20xx_set_lora_pkt_params( const void* context, const ral_lora_pkt_params_t* ral_pkt_params )
{
    ral_status_t                   status           = RAL_STATUS_ERROR;
    lr20xx_radio_lora_pkt_params_t radio_pkt_params = { 0 };

    status = ral_lr20xx_convert_lora_pkt_params_from_ral( ral_pkt_params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr20xx_radio_lora_set_packet_params( context, &radio_pkt_params );
}

ral_status_t ral_lr20xx_set_lora_cad_params( const void* context, const ral_lora_cad_params_t* ral_lora_cad_params )
{
    ral_status_t                   status = RAL_STATUS_ERROR;
    lr20xx_radio_lora_cad_params_t radio_lora_cad_params;

    status = ral_lr20xx_convert_lora_cad_params_from_ral( ral_lora_cad_params, &radio_lora_cad_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    return ( ral_status_t ) lr20xx_radio_lora_configure_cad_params( context, &radio_lora_cad_params );
}

ral_status_t ral_lr20xx_set_lora_symb_nb_timeout( const void* context, const uint16_t nb_of_symbs )
{
    return ( ral_status_t ) lr20xx_radio_lora_configure_timeout_by_number_of_symbols( context, nb_of_symbs );
}

ral_status_t ral_lr20xx_set_flrc_mod_params( const void* context, const ral_flrc_mod_params_t* params )
{
    ral_status_t                   status           = RAL_STATUS_ERROR;
    lr20xx_radio_flrc_mod_params_t radio_mod_params = { 0 };

    status = ral_lr20xx_convert_flrc_mod_params_from_ral( params, &radio_mod_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    return ( ral_status_t ) lr20xx_radio_flrc_set_modulation_params( context, &radio_mod_params );
}

ral_status_t ral_lr20xx_set_flrc_pkt_params( const void* context, const ral_flrc_pkt_params_t* params )
{
    ral_status_t                   status           = RAL_STATUS_ERROR;
    lr20xx_radio_flrc_pkt_params_t radio_pkt_params = { 0 };

    status = ral_lr20xx_convert_flrc_pkt_params_from_ral( params, &radio_pkt_params );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }
    return ( ral_status_t ) lr20xx_radio_flrc_set_pkt_params( context, &radio_pkt_params );
}

ral_status_t ral_lr20xx_get_gfsk_rx_pkt_status( const void* context, ral_gfsk_rx_pkt_status_t* ral_rx_pkt_status )
{
    ral_status_t                     status              = RAL_STATUS_ERROR;
    lr20xx_radio_fsk_packet_status_t radio_rx_pkt_status = { 0 };

    status = ( ral_status_t ) lr20xx_radio_fsk_get_packet_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_rx_pkt_status->rx_status = 0;

    ral_rx_pkt_status->rssi_sync_in_dbm = radio_rx_pkt_status.rssi_sync_in_dbm;
    ral_rx_pkt_status->rssi_avg_in_dbm  = radio_rx_pkt_status.rssi_avg_in_dbm;

    return status;
}

ral_status_t ral_lr20xx_get_lora_rx_pkt_status( const void* context, ral_lora_rx_pkt_status_t* ral_rx_pkt_status )
{
    ral_status_t                      status = RAL_STATUS_ERROR;
    lr20xx_radio_lora_packet_status_t radio_rx_pkt_status;

    status = ( ral_status_t ) lr20xx_radio_lora_get_packet_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    ral_rx_pkt_status->rssi_pkt_in_dbm        = radio_rx_pkt_status.rssi_pkt_in_dbm;
    ral_rx_pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) radio_rx_pkt_status.snr_pkt_raw ) + 2 ) >> 2;
    ral_rx_pkt_status->signal_rssi_pkt_in_dbm = radio_rx_pkt_status.rssi_signal_pkt_in_dbm;
    ral_rx_pkt_status->freq_offset_hz         = radio_rx_pkt_status.freq_offset_hz;

    return status;
}

ral_status_t ral_lr20xx_get_flrc_rx_pkt_status( const void* context, ral_flrc_rx_pkt_status_t* rx_pkt_status )
{
    lr20xx_radio_flrc_pkt_status_t radio_status = { 0 };

    const ral_status_t status = ( ral_status_t ) lr20xx_radio_flrc_get_pkt_status( context, &radio_status );

    if( status == RAL_STATUS_OK )
    {
        rx_pkt_status->packet_length_bytes      = radio_status.packet_length_bytes;
        rx_pkt_status->syncword_index           = radio_status.syncword_index;
        rx_pkt_status->rssi_avg_in_dbm          = radio_status.rssi_avg_in_dbm;
        rx_pkt_status->rssi_avg_half_dbm_count  = radio_status.rssi_avg_half_dbm_count;
        rx_pkt_status->rssi_sync_in_dbm         = radio_status.rssi_sync_in_dbm;
        rx_pkt_status->rssi_sync_half_dbm_count = radio_status.rssi_sync_half_dbm_count;
        rx_pkt_status->freq_offset_hz           = radio_status.freq_offset_hz;
    }
    return status;
}

ral_status_t ral_lr20xx_convert_flrc_mod_params_from_ral( const ral_flrc_mod_params_t*    ral_mod_params,
                                                          lr20xx_radio_flrc_mod_params_t* radio_mod_params )
{
    lr20xx_radio_flrc_br_bw_t       br_bw;  //!< Bitrate & bandwidth
    lr20xx_radio_flrc_cr_t          cr;     //!< Coding rate
    lr20xx_radio_flrc_pulse_shape_t shape;  //!< Shaping

    switch( ral_mod_params->raw_bit_rate )
    {
    case RAL_FLRC_RAW_BIT_RATE_0_260_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_0_260_BW_0_307;
        break;
    case RAL_FLRC_RAW_BIT_RATE_0_325_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_0_325_BW_0_357;
        break;
    case RAL_FLRC_RAW_BIT_RATE_0_520_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_0_520_BW_0_571;
        break;
    case RAL_FLRC_RAW_BIT_RATE_0_650_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_0_650_BW_0_740;
        break;
    case RAL_FLRC_RAW_BIT_RATE_1_040_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_1_040_BW_1_333;
        break;
    case RAL_FLRC_RAW_BIT_RATE_1_300_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_1_300_BW_1_333;
        break;
    case RAL_FLRC_RAW_BIT_RATE_2_080_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_2_080_BW_2_222;
        break;
    case RAL_FLRC_RAW_BIT_RATE_2_600_MBPS:
        br_bw = LR20XX_RADIO_FLRC_BR_2_600_BW_2_666;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    switch( ral_mod_params->cr )
    {
    case RAL_FLRC_CR_1_2:
        cr = LR20XX_RADIO_FLRC_CR_1_2;
        break;
    case RAL_FLRC_CR_3_4:
        cr = LR20XX_RADIO_FLRC_CR_3_4;
        break;
    case RAL_FLRC_CR_1_1:
        cr = LR20XX_RADIO_FLRC_CR_NONE;
        break;
    case RAL_FLRC_CR_2_3:
        cr = LR20XX_RADIO_FLRC_CR_2_3;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    switch( ral_mod_params->pulse_shape )
    {
    case RAL_FLRC_PULSE_SHAPE_OFF:
        shape = LR20XX_RADIO_FLRC_PULSE_SHAPE_OFF;
        break;
    case RAL_FLRC_PULSE_SHAPE_BT_05:
        shape = LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_05;
        break;
    case RAL_FLRC_PULSE_SHAPE_BT_1:
        shape = LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_1;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    radio_mod_params->br_bw = br_bw;
    radio_mod_params->cr    = cr;
    radio_mod_params->shape = shape;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_convert_flrc_pkt_params_from_ral( const ral_flrc_pkt_params_t*    ral_pkt_params,
                                                          lr20xx_radio_flrc_pkt_params_t* radio_pkt_params )
{
    switch( ral_pkt_params->preamble_len )
    {
    case RAL_FLRC_PREAMBLE_LENGTH_4_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_04_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_8_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_08_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_12_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_12_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_16_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_16_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_20_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_20_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_24_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_24_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_28_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_28_BITS;
        break;
    case RAL_FLRC_PREAMBLE_LENGTH_32_BITS:
        radio_pkt_params->preamble_len = LR20XX_RADIO_FLRC_PREAMBLE_LEN_32_BITS;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    switch( ral_pkt_params->sync_word_len )
    {
    case RAL_FLRC_SYNCWORD_LENGTH_OFF:
        radio_pkt_params->sync_word_len = LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_OFF;
        break;
    case RAL_FLRC_SYNCWORD_LENGTH_2_BYTES:
        radio_pkt_params->sync_word_len = LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_2_BYTES;
        break;
    case RAL_FLRC_SYNCWORD_LENGTH_4_BYTES:
        radio_pkt_params->sync_word_len = LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_4_BYTES;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    switch( ral_pkt_params->tx_syncword )
    {
    case RAL_FLRC_TX_SYNCWORD_NONE:
        radio_pkt_params->tx_syncword = LR20XX_RADIO_FLRC_TX_SYNCWORD_NONE;
        break;
    case RAL_FLRC_TX_SYNCWORD_1:
        radio_pkt_params->tx_syncword = LR20XX_RADIO_FLRC_TX_SYNCWORD_1;
        break;
    case RAL_FLRC_TX_SYNCWORD_2:
        radio_pkt_params->tx_syncword = LR20XX_RADIO_FLRC_TX_SYNCWORD_2;
        break;
    case RAL_FLRC_TX_SYNCWORD_3:
        radio_pkt_params->tx_syncword = LR20XX_RADIO_FLRC_TX_SYNCWORD_3;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    switch( ral_pkt_params->match_sync_word )
    {
    case RAL_FLRC_RX_MATCH_SYNCWORD_OFF:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_OFF;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_1:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_2:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_2;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_1_OR_2:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1_OR_2;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_3:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_3;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_1_OR_3:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1_OR_3;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_2_OR_3:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_2_OR_3;
        break;
    case RAL_FLRC_RX_MATCH_SYNCWORD_1_OR_2_OR_3:
        radio_pkt_params->match_sync_word = LR20XX_RADIO_FLRC_RX_MATCH_SYNCWORD_1_OR_2_OR_3;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    radio_pkt_params->header_type =
        ral_pkt_params->pld_is_fix ? LR20XX_RADIO_FLRC_PKT_FIX_LEN : LR20XX_RADIO_FLRC_PKT_VAR_LEN;

    radio_pkt_params->pld_len_in_bytes = ral_pkt_params->pld_len_in_bytes;

    switch( ral_pkt_params->crc_type )
    {
    case RAL_FLRC_CRC_OFF:
        radio_pkt_params->crc_type = LR20XX_RADIO_FLRC_CRC_OFF;
        break;
    case RAL_FLRC_CRC_2_BYTES:
        radio_pkt_params->crc_type = LR20XX_RADIO_FLRC_CRC_2_BYTES;
        break;
    case RAL_FLRC_CRC_3_BYTES:
        radio_pkt_params->crc_type = LR20XX_RADIO_FLRC_CRC_3_BYTES;
        break;
    case RAL_FLRC_CRC_4_BYTES:
        radio_pkt_params->crc_type = LR20XX_RADIO_FLRC_CRC_4_BYTES;
        break;
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_load_pram( const void* context )
{
#if defined( LR2021 )
    return ( ral_status_t ) lr20xx_pram_load_pram_lr2021( context );
#elif defined( LR2012 ) || defined( LR2022 )
    return ( ral_status_t ) lr20xx_pram_load_pram_lr20x2( context );
#else
#warning The radio chip must be selected at compile time with macros LR2012, LR2021 or LR2022
    return RAL_STATUS_ERROR;
#endif
}

ral_status_t ral_lr20xx_check_pram_version( const void* context )
{
#if defined( LR2021 )
    return ( ral_status_t ) lr20xx_pram_check_pram_lr2021( context );
#elif defined( LR2012 ) || defined( LR2022 )
    return ( ral_status_t ) lr20xx_pram_check_pram_lr20x2( context );
#else
#warning The radio chip must be selected at compile time with macros LR2012, LR2021 or LR2022
    return RAL_STATUS_ERROR;
#endif
}

ral_status_t ral_lr20xx_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    return ( ral_status_t ) lr20xx_radio_common_get_rssi_inst( context, rssi_in_dbm, NULL );
}

uint32_t ral_lr20xx_get_lora_time_on_air_in_ms( const ral_lora_pkt_params_t* pkt_p, const ral_lora_mod_params_t* mod_p )
{
    lr20xx_radio_lora_mod_params_t radio_mod_params;
    lr20xx_radio_lora_pkt_params_t radio_pkt_params;

    ral_lr20xx_convert_lora_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_lr20xx_convert_lora_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return lr20xx_radio_lora_get_time_on_air_in_ms( &radio_pkt_params, &radio_mod_params );
}

uint32_t ral_lr20xx_get_gfsk_time_on_air_in_ms( const ral_gfsk_pkt_params_t* pkt_p, const ral_gfsk_mod_params_t* mod_p )
{
    lr20xx_radio_fsk_mod_params_t radio_mod_params;
    lr20xx_radio_fsk_pkt_params_t radio_pkt_params;

    ral_lr20xx_convert_gfsk_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_lr20xx_convert_gfsk_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return lr20xx_radio_fsk_get_time_on_air_in_ms( &radio_pkt_params, &radio_mod_params, pkt_p->sync_word_len_in_bits );
}

uint32_t ral_lr20xx_get_flrc_time_on_air_in_us( const ral_flrc_pkt_params_t* pkt_p, const ral_flrc_mod_params_t* mod_p )
{
    lr20xx_radio_flrc_mod_params_t radio_mod_params;
    lr20xx_radio_flrc_pkt_params_t radio_pkt_params;

    ral_lr20xx_convert_flrc_mod_params_from_ral( mod_p, &radio_mod_params );
    ral_lr20xx_convert_flrc_pkt_params_from_ral( pkt_p, &radio_pkt_params );

    return lr20xx_get_flrc_time_on_air_in_us( &radio_pkt_params, &radio_mod_params );
}

ral_status_t ral_lr20xx_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    uint8_t syncword[LR20XX_RADIO_FSK_SYNCWORD_LENGTH];

    if( sync_word_len > LR20XX_RADIO_FSK_SYNCWORD_LENGTH )
    {
        return RAL_STATUS_ERROR;
    }

    for( int i = 0; i < sync_word_len; i++ )
    {
        syncword[LR20XX_RADIO_FSK_SYNCWORD_LENGTH - i - 1] = sync_word[sync_word_len - i - 1];
    }

    for( int i = 0; i < LR20XX_RADIO_FSK_SYNCWORD_LENGTH - sync_word_len; i++ )
    {
        syncword[i] = 0x00;
    }

    return ( ral_status_t ) lr20xx_radio_fsk_set_syncword( context, syncword, sync_word_len * 8,
                                                           LR20XX_RADIO_FSK_SYNCWORD_MSBF );
}

ral_status_t ral_lr20xx_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    return ( ral_status_t ) lr20xx_radio_lora_set_syncword( context, sync_word );
}

ral_status_t ral_lr20xx_set_flrc_sync_word( const void* context, const uint8_t sync_word_id, const uint8_t* sync_word,
                                            const uint8_t sync_word_len )
{
    if( sync_word_len == LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_OFF )
    {
        return RAL_STATUS_OK;
    }

    if( ( sync_word_len != LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_2_BYTES ) &&
        ( sync_word_len != LR20XX_RADIO_FLRC_SYNCWORD_LENGTH_4_BYTES ) )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return ( ral_status_t ) lr20xx_radio_flrc_set_syncword( context, sync_word_id, sync_word );
}

ral_status_t ral_lr20xx_set_gfsk_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    return ( ral_status_t ) lr20xx_radio_fsk_set_crc_params( context, polynomial, seed );
}

ral_status_t ral_lr20xx_set_flrc_crc_params( const void* context, const uint32_t seed, const uint32_t polynomial )
{
    // If seed and polynomial are both 0, then keep the default CRC params as is
    if( ( seed == 0x00000000 ) && ( polynomial == 0x00000000 ) )
    {
        return RAL_STATUS_OK;
    }

    // FLRC CRC param are set in the same way as GFSK CRC params
    return ( ral_status_t ) lr20xx_radio_fsk_set_crc_params( context, polynomial, seed );
}

ral_status_t ral_lr20xx_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    return ( ral_status_t ) lr20xx_radio_fsk_set_whitening_params(
        context, LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX126X_LR11XX, seed );
}

ral_status_t ral_lr20xx_set_gfsk_whitening_seed_comp( const void* context, const ral_gfsk_dc_free_t dc_free,
                                                      const uint16_t seed )
{
    switch( dc_free )
    {
    case RAL_GFSK_DC_FREE_WHITENING:
        return ( ral_status_t ) lr20xx_radio_fsk_set_whitening_params(
            context, LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX126X_LR11XX, seed );
    case RAL_GFSK_DC_FREE_WHITENING_SX128X_COMP:
        return ( ral_status_t ) lr20xx_radio_fsk_set_whitening_params(
            context, LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX128X, seed );
    default:
        return RAL_STATUS_UNKNOWN_VALUE;
    }
}

ral_status_t ral_lr20xx_lr_fhss_init( const void* context, const ral_lr_fhss_params_t* lr_fhss_params )
{
    ( void ) lr_fhss_params;  // Unused parameter
    return ( ral_status_t ) lr20xx_radio_lr_fhss_init( context );
}

ral_status_t ral_lr20xx_lr_fhss_build_frame( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                             ral_lr_fhss_memory_state_t state, uint16_t hop_sequence_id,
                                             const uint8_t* payload, uint16_t payload_length )
{
    ( void ) state;  // Unused argument

    lr20xx_radio_lr_fhss_params_t lr20xx_params;
    const ral_status_t            ral_status =
        ral_lr20xx_convert_lr_fhss_params_from_ral( lr_fhss_params, hop_sequence_id, &lr20xx_params );
    if( ral_status != RAL_STATUS_OK )
    {
        return ral_status;
    }

    lr20xx_status_t status = lr20xx_radio_common_set_rf_freq( context, lr_fhss_params->center_frequency_in_hz );
    if( status != LR20XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    status = lr20xx_radio_lr_fhss_set_sync_word( context, lr_fhss_params->lr_fhss_params.sync_word );
    if( status != LR20XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    status = lr20xx_radio_lr_fhss_build_frame( context, &lr20xx_params, payload, payload_length );
    if( status != LR20XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    lr20xx_system_stat1_t stat1;
    status = lr20xx_system_get_status( context, &stat1, 0, 0 );
    if( status != LR20XX_STATUS_OK )
    {
        return ( ral_status_t ) status;
    }

    if( stat1.command_status != LR20XX_SYSTEM_CMD_STATUS_OK )
    {
        return RAL_STATUS_ERROR;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_lr_fhss_handle_hop( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                            ral_lr_fhss_memory_state_t state )
{
    ( void ) context;         // Unused arguments
    ( void ) state;           // Unused arguments
    ( void ) lr_fhss_params;  // Unused arguments
    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_lr_fhss_handle_tx_done( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                ral_lr_fhss_memory_state_t state )
{
    ( void ) context;         // Unused arguments
    ( void ) state;           // Unused arguments
    ( void ) lr_fhss_params;  // Unused arguments
    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    ral_status_t status = RAL_STATUS_ERROR;

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = ( ral_status_t ) lr20xx_system_get_random_number(
            context, LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_PLL | LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_ADC, numbers + i );
        if( status != RAL_STATUS_OK )
        {
            return status;
        }
    }

    return status;
}

ral_status_t ral_lr20xx_handle_rx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_handle_tx_done( const void* context )
{
    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_get_lora_rx_pkt_cr_crc( const void* context, ral_lora_cr_t* cr, bool* is_crc_present )
{
    ral_status_t                      status = RAL_STATUS_ERROR;
    lr20xx_radio_lora_packet_status_t radio_rx_pkt_status;

    status = ( ral_status_t ) lr20xx_radio_lora_get_packet_status( context, &radio_rx_pkt_status );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    *cr             = ( ral_lora_cr_t ) radio_rx_pkt_status.cr;
    *is_crc_present = ( radio_rx_pkt_status.crc == LR20XX_RADIO_LORA_CRC_DISABLED ) ? false : true;

    return status;
}

ral_status_t ral_lr20xx_get_tx_consumption_in_ua( const void* context, const int8_t output_pwr_in_dbm,
                                                  const uint32_t rf_freq_in_hz, uint32_t* pwr_consumption_in_ua )
{
    // 1. Get the LR20xx BSP configuration corresponding to the input parameters
    lr20xx_system_reg_mode_t                   radio_reg_mode;
    ral_lr20xx_bsp_tx_cfg_output_params_t      tx_cfg_output_params;
    const ral_lr20xx_bsp_tx_cfg_input_params_t tx_cfg_input_params = {
        .freq_in_hz               = rf_freq_in_hz,
        .system_output_pwr_in_dbm = output_pwr_in_dbm,
    };
    ral_lr20xx_bsp_get_tx_cfg( context, &tx_cfg_input_params, &tx_cfg_output_params );
    ral_lr20xx_bsp_get_reg_mode( context, &radio_reg_mode );

    // 2. Refer to the BSP to get the instantaneous power consumption corresponding to the LR1xx BSP configuration
    return ral_lr20xx_bsp_get_instantaneous_tx_power_consumption( context, &tx_cfg_output_params, radio_reg_mode,
                                                                  pwr_consumption_in_ua );
}

ral_status_t ral_lr20xx_get_gfsk_rx_consumption_in_ua( const void* context, const uint32_t br_in_bps,
                                                       const uint32_t bw_dsb_in_hz, const bool rx_boosted,
                                                       uint32_t* pwr_consumption_in_ua )
{
    // 1. Get the regulator configured
    lr20xx_system_reg_mode_t radio_reg_mode;
    ral_lr20xx_bsp_get_reg_mode( context, &radio_reg_mode );

    // 2. Refer to BSP to get the instantaneous GFSK Rx Power consumption
    return ral_lr20xx_bsp_get_instantaneous_gfsk_rx_power_consumption( context, radio_reg_mode, rx_boosted,
                                                                       pwr_consumption_in_ua );
}

ral_status_t ral_lr20xx_get_lora_rx_consumption_in_ua( const void* context, const ral_lora_bw_t bw,
                                                       const bool rx_boosted, uint32_t* pwr_consumption_in_ua )
{
    // 1. Get the regulator configured
    lr20xx_system_reg_mode_t radio_reg_mode;
    ral_lr20xx_bsp_get_reg_mode( context, &radio_reg_mode );

    // 2. Refer to BSP to get the instantaneous LoRa Rx Power consumption
    return ral_lr20xx_bsp_get_instantaneous_lora_rx_power_consumption( context, radio_reg_mode, bw, rx_boosted,
                                                                       pwr_consumption_in_ua );
}

ral_status_t ral_lr20xx_lr_fhss_get_time_on_air_in_ms( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                       uint16_t payload_length, uint32_t* time_on_air )
{
    lr20xx_radio_lr_fhss_params_t lr20xx_params;
    const uint16_t                unused_hop_sequence_id =
        0;  // The hop sequence id is not used here, but necessary for the structure conversion
    const ral_status_t ral_status =
        ral_lr20xx_convert_lr_fhss_params_from_ral( lr_fhss_params, unused_hop_sequence_id, &lr20xx_params );
    if( ral_status != RAL_STATUS_OK )
    {
        return ral_status;
    }

    *time_on_air = lr20xx_radio_lr_fhss_get_time_on_air_in_ms( &lr20xx_params, payload_length );

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_lr_fhss_get_bit_delay_in_us( const void* context, const ral_lr_fhss_params_t* params,
                                                     uint16_t payload_length, uint16_t* delay )
{
    // Not needed, the LR20xx can transmit the exact number of bits.
    *delay = 0;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_lr_fhss_get_hop_sequence_count( const void* context, const ral_lr_fhss_params_t* lr_fhss_params,
                                                        unsigned int* hop_sequence_count )
{
    lr20xx_radio_lr_fhss_params_t lr20xx_params;
    const uint16_t                unused_hop_sequence_id =
        0;  // The hop sequence id is not used here, but necessary for the structure conversion
    const ral_status_t ral_status =
        ral_lr20xx_convert_lr_fhss_params_from_ral( lr_fhss_params, unused_hop_sequence_id, &lr20xx_params );
    if( ral_status != RAL_STATUS_OK )
    {
        return ral_status;
    }

    *hop_sequence_count = lr20xx_radio_lr_fhss_get_hop_sequence_count( &lr20xx_params );

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                               ral_lora_cad_symbs_t nb_symbol, uint8_t* cad_det_peak )
{
    static const uint8_t det_peak_value_nb_symbol_vs_sf[3][RAL_LORA_SF12 - RAL_LORA_SF5 + 1] = {
        { 60, 60, 60, 64, 64, 66, 70, 74 },  // 1 symbols
        { 56, 56, 56, 58, 58, 60, 64, 68 },  // 2 symbols
        { 51, 51, 51, 54, 56, 60, 60, 64 }   // 4 symbols
    };

    if( ( sf < RAL_LORA_SF5 ) || ( sf > RAL_LORA_SF12 ) )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    if( nb_symbol > RAL_LORA_CAD_04_SYMB )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    uint8_t sf_index = sf - RAL_LORA_SF5;
    *cad_det_peak    = det_peak_value_nb_symbol_vs_sf[nb_symbol][sf_index];

    ral_lr20xx_bsp_get_lora_cad_det_peak( context, sf, nb_symbol, cad_det_peak );

    return RAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

ral_irq_t ral_lr20xx_convert_irq_flags_to_ral( lr20xx_system_irq_mask_t lr20xx_irq_status )
{
    ral_irq_t ral_irq = RAL_IRQ_NONE;

    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_TX_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_TX_DONE;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RX_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_DONE;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_PREAMBLE_DETECTED;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_TIMEOUT ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_TIMEOUT;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_HDR_OK;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_HDR_ERROR;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_CRC_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_CRC_ERROR;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_CAD_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_CAD_DONE;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_CAD_DETECTED ) != 0 )
    {
        ral_irq |= RAL_IRQ_CAD_OK;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP ) != 0 )
    {
        ral_irq |= RAL_IRQ_LR_FHSS_HOP;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_DISCARDED ) != 0 )
    {
        ral_irq |= RAL_IRQ_RTTOF_REQ_DISCARDED;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_RESPONSE_DONE ) != 0 )
    {
        ral_irq |= RAL_IRQ_RTTOF_RESP_DONE;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_EXCHANGE_VALID ) != 0 )
    {
        ral_irq |= RAL_IRQ_RTTOF_EXCH_VALID;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_TIMEOUT ) != 0 )
    {
        ral_irq |= RAL_IRQ_RTTOF_TIMEOUT;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_CMD_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_CMD_ERROR;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_ERROR;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_FIFO_RX ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_FIFO_LEVEL;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_FIFO_TX ) != 0 )
    {
        ral_irq |= RAL_IRQ_TX_FIFO_LEVEL;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RX_TIMESTAMP ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_TIMESTAMP;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_TX_TIMESTAMP ) != 0 )
    {
        ral_irq |= RAL_IRQ_TX_TIMESTAMP;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LEN_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_RX_LEN_ERROR;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_ADDR_ERROR ) != 0 )
    {
        ral_irq |= RAL_IRQ_ADDR_ERROR;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LOW_BATTERY ) != 0 )
    {
        ral_irq |= RAL_IRQ_LOW_BATTERY;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LORA_RX_HEADER_TIMESTAMP ) != 0 )
    {
        ral_irq |= RAL_IRQ_LORA_RX_HEADER_TIMESTAMP;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE ) != 0 )
    {
        ral_irq |= RAL_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD ) != 0 )
    {
        ral_irq |= RAL_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_PA_OVP_OCP ) != 0 )
    {
        ral_irq |= RAL_IRQ_PA_OVP_OCP;
    }
    if( ( lr20xx_irq_status & LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_VALID ) != 0 )
    {
        ral_irq |= RAL_IRQ_RTTOF_RESPONDER_REQUEST_VALID;
    }

    return ral_irq;
}

lr20xx_system_irq_mask_t ral_lr20xx_convert_irq_flags_from_ral( ral_irq_t ral_irq )
{
    lr20xx_system_irq_mask_t lr20xx_irq_status = LR20XX_SYSTEM_IRQ_NONE;

    if( ( ral_irq & RAL_IRQ_TX_DONE ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_TX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_DONE ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RX_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RX_PREAMBLE_DETECTED ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED;
    }
    if( ( ral_irq & RAL_IRQ_RX_TIMEOUT ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_TIMEOUT;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_OK ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID;
    }
    if( ( ral_irq & RAL_IRQ_RX_HDR_ERROR ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_RX_CRC_ERROR ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_CRC_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_CAD_DONE ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_CAD_DONE;
    }
    if( ( ral_irq & RAL_IRQ_CAD_OK ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_CAD_DETECTED;
    }
    if( ( ral_irq & RAL_IRQ_LR_FHSS_HOP ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP;
    }
    if( ( ral_irq & RAL_IRQ_RX_FIFO_LEVEL ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_FIFO_RX;
    }
    if( ( ral_irq & RAL_IRQ_TX_FIFO_LEVEL ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_FIFO_TX;
    }
    if( ( ral_irq & RAL_IRQ_RTTOF_REQ_DISCARDED ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_DISCARDED;
    }
    if( ( ral_irq & RAL_IRQ_RTTOF_RESP_DONE ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_RESPONSE_DONE;
    }
    if( ( ral_irq & RAL_IRQ_RTTOF_EXCH_VALID ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_EXCHANGE_VALID;
    }
    if( ( ral_irq & RAL_IRQ_RTTOF_TIMEOUT ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_TIMEOUT;
    }
    if( ( ral_irq & RAL_IRQ_RX_TIMESTAMP ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RX_TIMESTAMP;
    }
    if( ( ral_irq & RAL_IRQ_TX_TIMESTAMP ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_TX_TIMESTAMP;
    }
    if( ( ral_irq & RAL_IRQ_RX_LEN_ERROR ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LEN_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_ERROR ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_CMD_ERROR ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_CMD_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_ADDR_ERROR ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_ADDR_ERROR;
    }
    if( ( ral_irq & RAL_IRQ_LOW_BATTERY ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LOW_BATTERY;
    }
    if( ( ral_irq & RAL_IRQ_LORA_RX_HEADER_TIMESTAMP ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LORA_RX_HEADER_TIMESTAMP;
    }
    if( ( ral_irq & RAL_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE;
    }
    if( ( ral_irq & RAL_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD;
    }
    if( ( ral_irq & RAL_IRQ_PA_OVP_OCP ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_PA_OVP_OCP;
    }
    if( ( ral_irq & RAL_IRQ_RTTOF_RESPONDER_REQUEST_VALID ) != 0 )
    {
        lr20xx_irq_status |= LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_VALID;
    }

    return lr20xx_irq_status;
}

ral_radio_fifo_flag_t ral_lr20xx_convert_fifo_irq_flags_to_ral( lr20xx_radio_fifo_flag_t lr20xx_fifo_irq_status )
{
    ral_radio_fifo_flag_t ral_fifo_irq = RAL_RADIO_FIFO_FLAG_NONE;

    if( ( lr20xx_fifo_irq_status & LR20XX_RADIO_FIFO_FLAG_EMPTY ) != 0 )
    {
        ral_fifo_irq |= RAL_RADIO_FIFO_FLAG_EMPTY;
    }
    if( ( lr20xx_fifo_irq_status & LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW ) != 0 )
    {
        ral_fifo_irq |= RAL_RADIO_FIFO_FLAG_THRESHOLD_LOW;
    }
    if( ( lr20xx_fifo_irq_status & LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH ) != 0 )
    {
        ral_fifo_irq |= RAL_RADIO_FIFO_FLAG_THRESHOLD_HIGH;
    }
    if( ( lr20xx_fifo_irq_status & LR20XX_RADIO_FIFO_FLAG_FULL ) != 0 )
    {
        ral_fifo_irq |= RAL_RADIO_FIFO_FLAG_FULL;
    }
    if( ( lr20xx_fifo_irq_status & LR20XX_RADIO_FIFO_FLAG_OVERFLOW ) != 0 )
    {
        ral_fifo_irq |= RAL_RADIO_FIFO_FLAG_OVERFLOW;
    }
    if( ( lr20xx_fifo_irq_status & LR20XX_RADIO_FIFO_FLAG_UNDERFLOW ) != 0 )
    {
        ral_fifo_irq |= RAL_RADIO_FIFO_FLAG_UNDERFLOW;
    }
    return ral_fifo_irq;
}

lr20xx_radio_fifo_flag_t ral_lr20xx_convert_irq_fifo_flags_from_ral( ral_radio_fifo_flag_t ral_fifo_irq )
{
    lr20xx_radio_fifo_flag_t lr20xx_fifo_irq_status = LR20XX_RADIO_FIFO_FLAG_NONE;

    if( ( ral_fifo_irq & RAL_RADIO_FIFO_FLAG_EMPTY ) != 0 )
    {
        lr20xx_fifo_irq_status |= LR20XX_RADIO_FIFO_FLAG_EMPTY;
    }
    if( ( ral_fifo_irq & RAL_RADIO_FIFO_FLAG_THRESHOLD_LOW ) != 0 )
    {
        lr20xx_fifo_irq_status |= LR20XX_RADIO_FIFO_FLAG_THRESHOLD_LOW;
    }
    if( ( ral_fifo_irq & RAL_RADIO_FIFO_FLAG_THRESHOLD_HIGH ) != 0 )
    {
        lr20xx_fifo_irq_status |= LR20XX_RADIO_FIFO_FLAG_THRESHOLD_HIGH;
    }
    if( ( ral_fifo_irq & RAL_RADIO_FIFO_FLAG_FULL ) != 0 )
    {
        lr20xx_fifo_irq_status |= LR20XX_RADIO_FIFO_FLAG_FULL;
    }
    if( ( ral_fifo_irq & RAL_RADIO_FIFO_FLAG_OVERFLOW ) != 0 )
    {
        lr20xx_fifo_irq_status |= LR20XX_RADIO_FIFO_FLAG_OVERFLOW;
    }
    if( ( ral_fifo_irq & RAL_RADIO_FIFO_FLAG_UNDERFLOW ) != 0 )
    {
        lr20xx_fifo_irq_status |= LR20XX_RADIO_FIFO_FLAG_UNDERFLOW;
    }
    return lr20xx_fifo_irq_status;
}

ral_status_t ral_lr20xx_convert_gfsk_mod_params_from_ral( const ral_gfsk_mod_params_t*   ral_mod_params,
                                                          lr20xx_radio_fsk_mod_params_t* radio_mod_params )
{
    ral_status_t                 status = RAL_STATUS_ERROR;
    lr20xx_radio_fsk_common_bw_t bw_dsb_param;

    status = ( ral_status_t ) lr20xx_radio_fsk_get_rx_bandwidth( ral_mod_params->bw_dsb_in_hz, &bw_dsb_param );
    if( status != RAL_STATUS_OK )
    {
        return status;
    }

    radio_mod_params->bitrate_unit = LR20XX_RADIO_FSK_MOD_PARAMS_BR_IN_BPS;
    radio_mod_params->bitrate      = ral_mod_params->br_in_bps;
    radio_mod_params->fdev_in_hz   = ral_mod_params->fdev_in_hz;
    radio_mod_params->bw           = bw_dsb_param;

    switch( ral_mod_params->pulse_shape )
    {
    case RAL_GFSK_PULSE_SHAPE_OFF:
    {
        radio_mod_params->pulse_shape = LR20XX_RADIO_FSK_PULSE_SHAPE_DISABLED;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_03:
    {
        radio_mod_params->pulse_shape = LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_3;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_05:
    {
        radio_mod_params->pulse_shape = LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_5;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_07:
    {
        radio_mod_params->pulse_shape = LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_7;
        break;
    }
    case RAL_GFSK_PULSE_SHAPE_BT_1:
    {
        radio_mod_params->pulse_shape = LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_1_0;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return status;
}

ral_status_t ral_lr20xx_convert_gfsk_pkt_params_from_ral( const ral_gfsk_pkt_params_t*   ral_pkt_params,
                                                          lr20xx_radio_fsk_pkt_params_t* radio_pkt_params )
{
    radio_pkt_params->pbl_length_in_bit = ral_pkt_params->preamble_len_in_bits;

    switch( ral_pkt_params->preamble_detector )
    {
    case RAL_GFSK_PREAMBLE_DETECTOR_OFF:
    {
        radio_pkt_params->preamble_detector = LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_DISABLED;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_8BITS:
    {
        radio_pkt_params->preamble_detector = LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_8_BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS:
    {
        radio_pkt_params->preamble_detector = LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_16_BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_24BITS:
    {
        radio_pkt_params->preamble_detector = LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_24_BITS;
        break;
    }
    case RAL_GFSK_PREAMBLE_DETECTOR_MIN_32BITS:
    {
        radio_pkt_params->preamble_detector = LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_32_BITS;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    switch( ral_pkt_params->address_filtering )
    {
    case RAL_GFSK_ADDRESS_FILTERING_DISABLE:
    {
        radio_pkt_params->address_filtering = LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED;
        break;
    }
    case RAL_GFSK_ADDRESS_FILTERING_NODE_ADDRESS:
    {
        radio_pkt_params->address_filtering = LR20XX_RADIO_FSK_ADDRESS_FILTERING_NODE;
        break;
    }
    case RAL_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES:
    {
        radio_pkt_params->address_filtering = LR20XX_RADIO_FSK_ADDRESS_FILTERING_NODE_BROADCAST;
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
        radio_pkt_params->header_mode = LR20XX_RADIO_FSK_HEADER_IMPLICIT;
        break;
    }
    case RAL_GFSK_PKT_VAR_LEN:
    {
        radio_pkt_params->header_mode = LR20XX_RADIO_FSK_HEADER_8BITS;
        break;
    }
    case RAL_GFSK_PKT_VAR_LEN_SX128X_COMP:
    {
        radio_pkt_params->header_mode = LR20XX_RADIO_FSK_HEADER_SX128X_COMPATIBLE;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_pkt_params->payload_length = ral_pkt_params->pld_len_in_bytes;

    switch( ral_pkt_params->crc_type )
    {
    case RAL_GFSK_CRC_OFF:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_OFF;
        break;
    }
    case RAL_GFSK_CRC_1_BYTE:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_1_BYTE;
        break;
    }
    case RAL_GFSK_CRC_2_BYTES:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_2_BYTES;
        break;
    }
    case RAL_GFSK_CRC_1_BYTE_INV:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_1_BYTE_INVERTED;
        break;
    }
    case RAL_GFSK_CRC_2_BYTES_INV:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_2_BYTES_INVERTED;
        break;
    }
    case RAL_GFSK_CRC_3_BYTES:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_3_BYTES;
        break;
    }
    case RAL_GFSK_CRC_3_BYTES_INV:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_3_BYTES_INVERTED;
        break;
    }
    case RAL_GFSK_CRC_4_BYTES:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_4_BYTES;
        break;
    }
    case RAL_GFSK_CRC_4_BYTES_INV:
    {
        radio_pkt_params->crc = LR20XX_RADIO_FSK_CRC_4_BYTES_INVERTED;
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
        radio_pkt_params->whitening = LR20XX_RADIO_FSK_WHITENING_OFF;
        break;
    }
    case RAL_GFSK_DC_FREE_WHITENING:
    {
        radio_pkt_params->whitening = LR20XX_RADIO_FSK_WHITENING_ON;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_convert_lora_mod_params_from_ral( const ral_lora_mod_params_t*    ral_mod_params,
                                                          lr20xx_radio_lora_mod_params_t* radio_mod_params )
{
    radio_mod_params->sf = ( lr20xx_radio_lora_sf_t ) ral_mod_params->sf;

    switch( ral_mod_params->bw )
    {
    case RAL_LORA_BW_031_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_31;
        break;
    }
    case RAL_LORA_BW_041_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_41;
        break;
    }
    case RAL_LORA_BW_062_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_62;
        break;
    }
    case RAL_LORA_BW_083_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_83;
        break;
    }
    case RAL_LORA_BW_101_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_101;
        break;
    }
    case RAL_LORA_BW_125_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_125;
        break;
    }
    case RAL_LORA_BW_200_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_203;
        break;
    }
    case RAL_LORA_BW_250_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_250;
        break;
    }
    case RAL_LORA_BW_400_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_406;
        break;
    }
    case RAL_LORA_BW_500_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_500;
        break;
    }
    case RAL_LORA_BW_800_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_812;
        break;
    }
    case RAL_LORA_BW_1000_KHZ:
    {
        radio_mod_params->bw = LR20XX_RADIO_LORA_BW_1000;
        break;
    }
    case RAL_LORA_BW_007_KHZ:
    case RAL_LORA_BW_010_KHZ:
    case RAL_LORA_BW_015_KHZ:
    case RAL_LORA_BW_020_KHZ:
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_mod_params->cr = ( lr20xx_radio_lora_cr_t ) ral_mod_params->cr;

    radio_mod_params->ppm = ral_mod_params->ldro;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_convert_lora_pkt_params_from_ral( const ral_lora_pkt_params_t*    ral_pkt_params,
                                                          lr20xx_radio_lora_pkt_params_t* radio_pkt_params )
{
    radio_pkt_params->preamble_len_in_symb = ral_pkt_params->preamble_len_in_symb;

    switch( ral_pkt_params->header_type )
    {
    case( RAL_LORA_PKT_EXPLICIT ):
    {
        radio_pkt_params->pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT;
        break;
    }
    case( RAL_LORA_PKT_IMPLICIT ):
    {
        radio_pkt_params->pkt_mode = LR20XX_RADIO_LORA_PKT_IMPLICIT;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_pkt_params->pld_len_in_bytes = ral_pkt_params->pld_len_in_bytes;
    radio_pkt_params->crc =
        ( ral_pkt_params->crc_is_on == false ) ? LR20XX_RADIO_LORA_CRC_DISABLED : LR20XX_RADIO_LORA_CRC_ENABLED;
    radio_pkt_params->iq =
        ( ral_pkt_params->invert_iq_is_on == false ) ? LR20XX_RADIO_LORA_IQ_STANDARD : LR20XX_RADIO_LORA_IQ_INVERTED;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_convert_lora_cad_params_from_ral( const ral_lora_cad_params_t*    ral_lora_cad_params,
                                                          lr20xx_radio_lora_cad_params_t* radio_lora_cad_params )
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

    switch( ral_lora_cad_params->cad_exit_mode )
    {
    case RAL_LORA_CAD_ONLY:
    {
        radio_lora_cad_params->cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_STANDBYRC;
        break;
    }
    case RAL_LORA_CAD_RX:
    {
        radio_lora_cad_params->cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_RX;
        break;
    }
    case RAL_LORA_CAD_LBT:
    {
        radio_lora_cad_params->cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_TX;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_lora_cad_params->cad_timeout_in_pll_step =
        lr20xx_radio_common_convert_time_in_ms_to_rtc_step( ral_lora_cad_params->cad_timeout_in_ms );

    radio_lora_cad_params->pnr_delta = 0;

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_convert_lr_fhss_params_from_ral( const ral_lr_fhss_params_t*    ral_lr_fhss_params,
                                                         uint16_t                       hop_sequence_id,
                                                         lr20xx_radio_lr_fhss_params_t* radio_lr_fhss_params )
{
    switch( ral_lr_fhss_params->lr_fhss_params.modulation_type )
    {
    case LR_FHSS_V1_MODULATION_TYPE_GMSK_488:
    {
        radio_lr_fhss_params->modulation_type = LR20XX_RADIO_LR_FHSS_MODULATION_TYPE_GMSK_488;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    switch( ral_lr_fhss_params->lr_fhss_params.cr )
    {
    case LR_FHSS_V1_CR_5_6:
    {
        radio_lr_fhss_params->cr = LR20XX_RADIO_LR_FHSS_CR_5_6;
        break;
    }
    case LR_FHSS_V1_CR_2_3:
    {
        radio_lr_fhss_params->cr = LR20XX_RADIO_LR_FHSS_CR_2_3;
        break;
    }
    case LR_FHSS_V1_CR_1_2:
    {
        radio_lr_fhss_params->cr = LR20XX_RADIO_LR_FHSS_CR_1_2;
        break;
    }
    case LR_FHSS_V1_CR_1_3:
    {
        radio_lr_fhss_params->cr = LR20XX_RADIO_LR_FHSS_CR_1_3;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    switch( ral_lr_fhss_params->lr_fhss_params.grid )
    {
    case LR_FHSS_V1_GRID_25391_HZ:
    {
        radio_lr_fhss_params->grid = LR20XX_RADIO_LR_FHSS_GRID_25391_HZ;
        break;
    }
    case LR_FHSS_V1_GRID_3906_HZ:
    {
        radio_lr_fhss_params->grid = LR20XX_RADIO_LR_FHSS_GRID_3906_HZ;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    switch( ral_lr_fhss_params->lr_fhss_params.bw )
    {
    case LR_FHSS_V1_BW_39063_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_39063_HZ;
        break;
    }
    case LR_FHSS_V1_BW_85938_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_85938_HZ;
        break;
    }
    case LR_FHSS_V1_BW_136719_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_136719_HZ;
        break;
    }
    case LR_FHSS_V1_BW_183594_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_183594_HZ;
        break;
    }
    case LR_FHSS_V1_BW_335938_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_335938_HZ;
        break;
    }
    case LR_FHSS_V1_BW_386719_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_386719_HZ;
        break;
    }
    case LR_FHSS_V1_BW_722656_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_722656_HZ;
        break;
    }
    case LR_FHSS_V1_BW_773438_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_773438_HZ;
        break;
    }
    case LR_FHSS_V1_BW_1523438_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_1523438_HZ;
        break;
    }
    case LR_FHSS_V1_BW_1574219_HZ:
    {
        radio_lr_fhss_params->bw = LR20XX_RADIO_LR_FHSS_BW_1574219_HZ;
        break;
    }
    default:
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }
    }

    radio_lr_fhss_params->syncword_header_count = ral_lr_fhss_params->lr_fhss_params.header_count;
    radio_lr_fhss_params->hop_sequence_id       = hop_sequence_id;
    radio_lr_fhss_params->device_offset         = ral_lr_fhss_params->device_offset;
    radio_lr_fhss_params->hopping_mode          = ( ral_lr_fhss_params->lr_fhss_params.enable_hopping == true )
                                                      ? LR20XX_RADIO_LR_FHSS_HOPPING_ON
                                                      : LR20XX_RADIO_LR_FHSS_HOPPING_OFF;

    // If this point is reached, the conversion has been correctly executed: return success status
    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_rttof_set_parameters( const void* context, const uint8_t nb_symbols )
{
    lr20xx_rttof_params_t params = {
        .mode      = LR20XX_RTTOF_MODE_NORMAL,
        .spy_mode  = LR20XX_RTTOF_SPY_MODE_DISABLED,
        .nb_symbol = nb_symbols,
    };
    return ( ral_status_t ) lr20xx_rttof_set_params( context, &params );
}

ral_status_t ral_lr20xx_rttof_set_address( const void* context, const uint32_t address, const uint8_t check_length )
{
    return ( ral_status_t ) lr20xx_rttof_set_responder_address( context, address, check_length );
}

ral_status_t ral_lr20xx_rttof_set_request_address( const void* context, const uint32_t request_address )
{
    return ( ral_status_t ) lr20xx_rttof_set_initiator_address( context, request_address );
}

ral_status_t ral_lr20xx_rttof_set_rx_tx_delay_indicator( const void* context, const uint32_t delay_indicator )
{
    return ( ral_status_t ) lr20xx_rttof_set_tx_rx_delay( context, delay_indicator );
}

ral_status_t ral_lr20xx_rttof_get_raw_result( const void* context, ral_lora_bw_t rttof_bw, int32_t* raw_results,
                                              int32_t* meter_results, int8_t* rssi_result )
{
    // get raw results
    lr20xx_rttof_results_t results       = { 0 };
    lr20xx_status_t        lr20xx_status = lr20xx_rttof_get_results( context, &results );
    if( lr20xx_status != LR20XX_STATUS_OK )
    {
        return ( ral_status_t ) lr20xx_status;
    }

    // convert bandwith
    ral_lora_mod_params_t lora_mod_params           = { 0 };
    lora_mod_params.bw                              = rttof_bw;
    lr20xx_radio_lora_mod_params_t radio_mod_params = { 0 };
    ral_status_t ral_status = ral_lr20xx_convert_lora_mod_params_from_ral( &lora_mod_params, &radio_mod_params );
    if( ral_status != RAL_STATUS_OK )
    {
        return ral_status;
    }

    // fill output parameters
    *raw_results   = results.val;
    *rssi_result   = results.rssi;
    *meter_results = lr20xx_rttof_distance_raw_to_meter( radio_mod_params.bw, results.val );

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_check_lora_parameters( const void* context, const uint32_t rf_freq_in_hz,
                                               const ral_lora_pkt_params_t* pkt_params,
                                               const ral_lora_mod_params_t* mod_params )
{
    if( ( rf_freq_in_hz < RAL_LR20XX_LF_HZ_MIN ) || ( rf_freq_in_hz > RAL_LR20XX_LF_HZ_MAX ) )
    {
        if( SUPPORT_2G4 )
        {
            if( ( rf_freq_in_hz < RAL_LR20XX_HF_HZ_MIN ) || ( rf_freq_in_hz > RAL_LR20XX_HF_HZ_MAX ) )
            {
                return RAL_STATUS_UNKNOWN_VALUE;
            }
        }
        else
        {
            return RAL_STATUS_UNKNOWN_VALUE;
        }
    }

    bool bw_supported = false;
    for( uint8_t i = 0; i < RAL_LR20XX_SUPPORTED_BW_SIZE; i++ )
    {
        if( RAL_LR20XX_SUPPORTED_BW[i] == mod_params->bw )
        {
            bw_supported = true;
            break;
        }
    }
    if( !bw_supported )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    if( ( mod_params->sf < RAL_LORA_SF5 ) || ( mod_params->sf > RAL_LORA_SF12 ) )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    if( ( mod_params->cr < RAL_LORA_CR_4_5 ) || ( mod_params->cr > RAL_LORA_CR_LI_CONVOLUTIONAL_4_8 ) )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    if( pkt_params->header_type > RAL_LORA_PKT_IMPLICIT )
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_check_rttof_parameters( const void* context, const uint32_t rf_freq_in_hz,
                                                const ral_lora_pkt_params_t* pkt_params,
                                                const ral_lora_mod_params_t* mod_params )
{
    return ral_lr20xx_check_lora_parameters( context, rf_freq_in_hz, pkt_params, mod_params );
}
/* --- EOF ------------------------------------------------------------------ */
