/**
 * @file      ral_llcc68_bsp.h
 *
 * @brief     Board Support Package for the LLCC68-specific RAL.
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

#ifndef RAL_LLCC68_BSP_H
#define RAL_LLCC68_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "ral_defs.h"
#include "llcc68.h"

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

typedef struct ral_llcc68_bsp_tx_cfg_input_params_s
{
    int8_t   system_output_pwr_in_dbm;
    uint32_t freq_in_hz;
} ral_llcc68_bsp_tx_cfg_input_params_t;

typedef struct ral_llcc68_bsp_tx_cfg_output_params_s
{
    llcc68_pa_cfg_params_t pa_cfg;
    llcc68_ramp_time_t     pa_ramp_time;
    int8_t                 chip_output_pwr_in_dbm_configured;
    int8_t                 chip_output_pwr_in_dbm_expected;
} ral_llcc68_bsp_tx_cfg_output_params_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Get the regulator mode configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] reg_mode
 */
void ral_llcc68_bsp_get_reg_mode( const void* context, llcc68_reg_mod_t* reg_mode );

/**
 * Get the internal RF switch configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] dio2_is_set_as_rf_switch
 */
void ral_llcc68_bsp_get_rf_switch_cfg( const void* context, bool* dio2_is_set_as_rf_switch );

/**
 * Get the Tx-related configuration (power amplifier configuration, output power and ramp time) to be applied to the
 * chip
 *
 * @param [in] context Chip implementation context
 * @param [in] input_params Parameters used to compute the chip configuration
 * @param [out] output_params Parameters to be configured in the chip
 */
void ral_llcc68_bsp_get_tx_cfg( const void* context, const ral_llcc68_bsp_tx_cfg_input_params_t* input_params,
                                ral_llcc68_bsp_tx_cfg_output_params_t* output_params );

/**
 * Get the XOSC configuration
 *
 * @remark If no TCXO is present, this function should set tcxo_is_radio_controlled to false, and return.
 *
 * @param [in] context Chip implementation context
 * @param [out] tcxo_is_radio_controlled Let the caller know if there is a radio-controlled TCXO
 * @param [out] supply_voltage TCXO supply voltage parameter
 * @param [out] startup_time_in_tick TCXO setup time in clock tick
 */
void ral_llcc68_bsp_get_xosc_cfg( const void* context, bool* tcxo_is_radio_controlled,
                                  llcc68_tcxo_ctrl_voltages_t* supply_voltage, uint32_t* startup_time_in_tick );

/**
 * Get the trimming capacitor values
 *
 * @param [in] context Chip implementation context
 * @param [out] trimming_cap_xta Value for the trimming capacitor connected to XTA pin
 * @param [out] trimming_cap_xtb Value for the trimming capacitor connected to XTB pin
 */
void ral_llcc68_bsp_get_trim_cap( const void* context, uint8_t* trimming_cap_xta, uint8_t* trimming_cap_xtb );

/**
 * Get the trimming capacitor values
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_boost_is_activated Let the caller know if the RX boosted mode is activated
 */
void ral_llcc68_bsp_get_rx_boost_cfg( const void* context, bool* rx_boost_is_activated );

/**
 * Get the OCP (Over Current Protection) value
 *
 * @param [in] context Chip implementation context
 * @param [out] ocp_in_step_of_2_5_ma OCP value given in steps of 2.5 mA
 */
void ral_llcc68_bsp_get_ocp_value( const void* context, uint8_t* ocp_in_step_of_2_5_ma );

/**
 * @brief Get the Channel Activity Detection (CAD) DetPeak value
 *
 * @param [in] context                  Chip implementation context
 * @param [in] sf                       CAD LoRa spreading factor
 * @param [in] bw                       CAD LoRa bandwidth
 * @param [in] nb_symbol                CAD on number of symbols
 * @param [in, out] in_out_cad_det_peak  CAD DetPeak value proposed by the ral could be overwritten
 */
void ral_llcc68_bsp_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_bw_t bw,
                                           ral_lora_cad_symbs_t nb_symbol, uint8_t* in_out_cad_det_peak );

/**
 * @brief Get the instantaneous power consumption for the given Tx configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_cfg_output_params_local The Tx configuration
 * @param [in] radio_reg_mode The regulator configuration
 * @param [out] pwr_consumption_in_ua The corresponding instantaneous power consumption
 * @return ral_status_t
 */
ral_status_t ral_llcc68_bsp_get_instantaneous_tx_power_consumption( const void* context,
    const ral_llcc68_bsp_tx_cfg_output_params_t* tx_cfg_output_params_local, llcc68_reg_mod_t radio_reg_mode,
    uint32_t* pwr_consumption_in_ua );

/**
 * @brief Get the instantaneous power consumption for the given GFSK Rx configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] radio_reg_mode The regulator configuration
 * @param [in] rx_boosted The Rx boosted configuration
 * @param [out] pwr_consumption_in_ua The corresponding instantaneous power consumption
 * @return ral_status_t
 */
ral_status_t ral_llcc68_bsp_get_instantaneous_gfsk_rx_power_consumption( const void* context,
    llcc68_reg_mod_t radio_reg_mode, bool rx_boosted, uint32_t* pwr_consumption_in_ua );

/**
 * @brief Get the instantaneous power consumption for the given LoRa Rx configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] radio_reg_mode The regulator configuration
 * @param [in] rx_boosted The Rx boosted configuration
 * @param [out] pwr_consumption_in_ua The corresponding instantaneous power consumption
 * @return ral_status_t
 */
ral_status_t ral_llcc68_bsp_get_instantaneous_lora_rx_power_consumption( const void* context,
    llcc68_reg_mod_t radio_reg_mode, bool rx_boosted, uint32_t* pwr_consumption_in_ua );
#ifdef __cplusplus
}
#endif

#endif  // RAL_LLCC68_BSP_H

/* --- EOF ------------------------------------------------------------------ */
