/**
 * @file      ral_lr20xx_bsp.h
 *
 * @brief     Board Support Package for the Lr20xx-specific RAL.
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

#ifndef RAL_LR20XX_BSP_H
#define RAL_LR20XX_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "ral_defs.h"
#include "lr20xx_radio_common_types.h"
#include "lr20xx_system_types.h"

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

typedef struct ral_lr20xx_bsp_tx_cfg_input_params_s
{
    int8_t   system_output_pwr_in_dbm;
    uint32_t freq_in_hz;
} ral_lr20xx_bsp_tx_cfg_input_params_t;

typedef struct ral_lr20xx_bsp_tx_cfg_output_params_s
{
    lr20xx_radio_common_pa_cfg_t    pa_cfg;
    lr20xx_radio_common_ramp_time_t pa_ramp_time;
    int8_t                          chip_output_half_pwr_in_dbm_configured;
    int8_t                          chip_output_pwr_in_dbm_expected;
} ral_lr20xx_bsp_tx_cfg_output_params_t;

typedef struct ral_lr20xx_bsp_irq_assignment_s
{
    lr20xx_system_dio_t      dio;
    lr20xx_system_irq_mask_t irq_mask;
} ral_lr20xx_bsp_irq_assignment_t;

typedef struct ral_lr20xx_bsp_dio_rf_switch_cfg_s
{
    lr20xx_system_dio_t               dio;
    lr20xx_system_dio_rf_switch_cfg_t rf_switch;
} ral_lr20xx_bsp_dio_rf_switch_cfg_s;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Get the Tx-related configuration (power amplifier configuration, output power and ramp time) to be applied to the
 * chip
 *
 * @param [in] context Chip implementation context
 * @param [in] input_params Parameters used to compute the chip configuration
 * @param [out] output_params Parameters to be configured in the chip
 */
void ral_lr20xx_bsp_get_tx_cfg( const void* context, const ral_lr20xx_bsp_tx_cfg_input_params_t* input_params,
                                ral_lr20xx_bsp_tx_cfg_output_params_t* output_params );

/**
 * @brief Get the Rx-related configuration to set the Rx Path
 *
 * @param [in] context Chip implementation context
 * @param [in] freq_in_hz Rx frequency
 * @param [out] rx_path RxPath LowFreq or HighFreq
 * @param [out] boost_mode Rx boost mode
 */
void ral_lr20xx_bsp_get_rx_cfg( const void* context, const uint32_t freq_in_hz, lr20xx_radio_common_rx_path_t* rx_path,
                                lr20xx_radio_common_rx_path_boost_mode_t* boost_mode );

/**
 * @brief Get the Rx front-end calibration frequency list
 *
 * @param [in] context Chip implementation context
 * @param [in] front_end_calibration_structures Rx frequencies and Rx path, can contain up to 3 configurations
 */
void ral_lr20xx_bsp_get_front_end_calibration_cfg(
    const void* context, lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures[3] );

/**
 * Get the function to be configured on a DIO
 *
 * @param [in] context Chip implementation context
 * @param [in] dio The requested DIO
 * @param [out] function The DIO function to be configured
 */
void ral_lr20xx_bsp_get_dio_function( const void* context, lr20xx_system_dio_t dio,
    lr20xx_system_dio_func_t* function );

/**
 * Get the sleep drive to be configured on a DIO
 *
 * @param [in] context Chip implementation context
 * @param [in] dio The requested DIO
 * @param [out] function The DIO sleep drive configuration
 */
void ral_lr20xx_bsp_get_dio_sleep_drive( const void* context, lr20xx_system_dio_t dio,
    lr20xx_system_dio_drive_t* drive );

/**
 * Get the IRQ mask to be configured on a DIO
 *
 * @param [in] context Chip implementation context
 * @param [in] dio The requested DIO
 * @param [out] irq_mask The IRQ mask to apply on the DIO
 */
void ral_lr20xx_bsp_get_dio_irq_mask( const void* context, lr20xx_system_dio_t dio,
    lr20xx_system_irq_mask_t* irq_mask );

/**
 * Get the RF switch configuration to be used on a DIO
 *
 * @param [in] context Chip implementation context
 * @param [in] dio The requested DIO
 * @param [out] rf_switch_cfg Pointer to a rf_switch configuration structure
 */
void ral_lr20xx_bsp_get_dio_rf_switch_cfg( const void* context, lr20xx_system_dio_t dio,
    lr20xx_system_dio_rf_switch_cfg_t* rf_switch_cfg );

/**
 * Get the HF Clock output scaling to be configured
 *
 * @param [in] context Chip implementation context
 * @param [out] rf_switch_cfg Pointer to a rf_switch configuration structure
 */
void ral_lr20xx_bsp_get_dio_hf_clk_scaling_cfg( const void* context,
    lr20xx_system_hf_clk_scaling_t* hf_clk_scaling);

/**
 * @brief Get regulator mode
 *
 * @param [in] context Chip implementation context
 * @param [out] reg_mode Pointer to a regulator mode configuration
 */
void ral_lr20xx_bsp_get_reg_mode( const void* context, lr20xx_system_reg_mode_t* reg_mode );

/**
 * @brief Get low frequency clock configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] lfclk_cfg Pointer to a low frequency clock configuration variable
 */
void ral_bsp_lr20xx_get_lfclk_cfg( const void* context, lr20xx_system_lfclk_cfg_t* lfclk_cfg );

/**
 * @brief Get oscillator configuration (XTAL vs TCXO)
 *
 * @param [in] context Chip implementation context
 * @param [out] xosc_cfg Pointer to a ral_xosc_cfg_t variable
 * @param [out] tcxo_supply_voltage Pointer to a voltage variable
 * @param [out] uint32_t Pointer to a uint32_t variable
 */
void ral_lr20xx_bsp_get_xosc_cfg( const void* context, ral_xosc_cfg_t* xosc_cfg,
                                  lr20xx_system_tcxo_supply_voltage_t* tcxo_supply_voltage,
                                  uint32_t*                            startup_time_in_tick );

/**
 * @brief Get the Channel Activity Detection (CAD) DetPeak value
 *
 * @param [in] context Chip implementation context
 * @param [in] sf                       CAD LoRa spreading factor
 * @param [in] nb_symbol                CAD on number of symbols
 * @param [in, out] in_out_cad_det_peak  CAD DetPeak value proposed by the ral could be overwritten
 */
void ral_lr20xx_bsp_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_cad_symbs_t nb_symbol,
                                           uint8_t* in_out_cad_det_peak );

/**
 * @brief Get the instantaneous power consumption for the given Tx configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_cfg The Tx configuration
 * @param [in] radio_reg_mode The regulator configuration
 * @param [out] pwr_consumption_in_ua The corresponding instantaneous power consumption
 * @return ral_status_t
 */
ral_status_t ral_lr20xx_bsp_get_instantaneous_tx_power_consumption( const void* context,
                                                                    const ral_lr20xx_bsp_tx_cfg_output_params_t* tx_cfg,
                                                                    lr20xx_system_reg_mode_t radio_reg_mode,
                                                                    uint32_t*                pwr_consumption_in_ua );

/**
 * @brief Get the instantaneous power consumption for the given GFSK Rx configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] radio_reg_mode The regulator configuration
 * @param [in] rx_boosted The Rx boosted configuration
 * @param [out] pwr_consumption_in_ua The corresponding instantaneous power consumption
 * @return ral_status_t
 */
ral_status_t ral_lr20xx_bsp_get_instantaneous_gfsk_rx_power_consumption( const void*              context,
                                                                         lr20xx_system_reg_mode_t radio_reg_mode,
                                                                         bool                     rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua );

/**
 * @brief Get the instantaneous power consumption for the given LoRa Rx configuration
 *
 * @param [in] context Chip implementation context
 * @param radio_reg_mode The regulator configuration
 * @param bw The LoRa bandwidth
 * @param rx_boosted The Rx boosted configuration
 * @param pwr_consumption_in_ua The corresponding instantaneous power consumption
 * @return ral_status_t
 */
ral_status_t ral_lr20xx_bsp_get_instantaneous_lora_rx_power_consumption( const void*                    context,
                                                                         const lr20xx_system_reg_mode_t radio_reg_mode,
                                                                         const ral_lora_bw_t bw, bool rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua );
#ifdef __cplusplus
}
#endif

#endif  // RAL_LR20XX_BSP_H

/* --- EOF ------------------------------------------------------------------ */
