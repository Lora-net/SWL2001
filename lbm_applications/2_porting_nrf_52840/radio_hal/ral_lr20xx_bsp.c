/**
 * @file      ral_lr20xx_bsp.c
 *
 * @brief     Board Support Package for the LR20xx-specific Radio Abstraction Layer.
 *
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

#include <stdint.h>
#include "lr20xx_radio_common.h"
#include "lr20xx_system_types.h"
#include "lr20xx_pa_pwr_cfg.h"
#include "ral_lr20xx_bsp.h"
#include "radio_utilities.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR20XX_LF_MIN_OUTPUT_POWER -10
#define LR20XX_LF_MAX_OUTPUT_POWER 22

#define LR20XX_HF_MIN_OUTPUT_POWER -17
#define LR20XX_HF_MAX_OUTPUT_POWER 12

// TODO check value
#define LR20XX_GFSK_RX_CONSUMPTION_DCDC 5410
#define LR20XX_GFSK_RX_BOOSTED_CONSUMPTION_DCDC 6970

#define LR20XX_GFSK_RX_CONSUMPTION_LDO 9500
#define LR20XX_GFSK_RX_BOOSTED_CONSUMPTION_LDO 11730

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

typedef struct lr20xx_pa_pwr_cfg_s
{
    int8_t  half_power;
    uint8_t pa_duty_cycle;
    uint8_t pa_lf_slices;
} lr20xx_pa_pwr_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get the tx output param configuration given the type of power amplifier and expected output power
 *
 * @param [in] pa_type Power Amplifier type
 * @param [in] expected_output_pwr_in_dbm TX output power in dBm
 * @param [out] output_params The tx config output params
 */

void lr20xx_get_tx_cfg( lr20xx_radio_common_pa_selection_t pa_type, int8_t expected_output_pwr_in_dbm,
                        ral_lr20xx_bsp_tx_cfg_output_params_t* output_params );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
const lr20xx_pa_pwr_cfg_t pa_lf_cfg_table[LR20XX_LF_MAX_OUTPUT_POWER - LR20XX_LF_MIN_OUTPUT_POWER + 1] =
    LR20XX_PA_LF_CFG_TABLE;

const lr20xx_pa_pwr_cfg_t pa_hf_cfg_table[LR20XX_HF_MAX_OUTPUT_POWER - LR20XX_HF_MIN_OUTPUT_POWER + 1] =
    LR20XX_PA_HF_CFG_TABLE;

/**
 * @brief Power consumption table in Tx Low frequency mode
 * @remark measured at 3.3v and DCDC
 */
static const uint32_t ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_dcdc_lf_vreg[] = {
    10400,   // -10 dBm
    10100,   //  -9 dBm
    9000,    //  -8 dBm
    9000,    //  -7 dBm
    8400,    //  -6 dBm
    8700,    //  -5 dBm
    10100,   //  -4 dBm
    9200,    //  -3 dBm
    10600,   //  -2 dBm
    10000,   //  -1 dBm
    10000,   //   0 dBm
    12100,   //   1 dBm
    10900,   //   2 dBm
    12000,   //   3 dBm
    13100,   //   4 dBm
    13300,   //   5 dBm
    13400,   //   6 dBm
    14600,   //   7 dBm
    15300,   //   8 dBm
    16800,   //   9 dBm
    18100,   //  10 dBm
    20200,   //  11 dBm
    21900,   //  12 dBm
    25600,   //  13 dBm
    28300,   //  14 dBm
    32200,   //  15 dBm
    37600,   //  16 dBm
    46000,   //  17 dBm
    51800,   //  18 dBm
    61300,   //  19 dBm
    74100,   //  20 dBm
    89400,   //  21 dBm
    108200,  //  22 dBm
};

// TODO
/**
 * @brief Power consumption table in Tx Low frequency mode
 * @remark measured at 3.3v and LDO
 */
static const uint32_t ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_ldo_lf_vreg[] = {
    1000,  // -10 dBm // TODO
    1000,  //  -9 dBm // TODO
    1000,  //  -8 dBm // TODO
    1000,  //  -7 dBm // TODO
    1000,  //  -6 dBm // TODO
    1000,  //  -5 dBm // TODO
    1000,  //  -4 dBm // TODO
    1000,  //  -3 dBm // TODO
    1000,  //  -2 dBm // TODO
    1000,  //  -1 dBm // TODO
    1000,  //   0 dBm // TODO
    1000,  //   1 dBm // TODO
    1000,  //   2 dBm // TODO
    1000,  //   3 dBm // TODO
    1000,  //   4 dBm // TODO
    1000,  //   5 dBm // TODO
    1000,  //   6 dBm // TODO
    1000,  //   7 dBm // TODO
    1000,  //   8 dBm // TODO
    1000,  //   9 dBm // TODO
    1000,  //  10 dBm // TODO
    1000,  //  11 dBm // TODO
    1000,  //  12 dBm // TODO
    1000,  //  13 dBm // TODO
    1000,  //  14 dBm // TODO
    1000,  //  15 dBm // TODO
    1000,  //  16 dBm // TODO
    1000,  //  17 dBm // TODO
    1000,  //  18 dBm // TODO
    1000,  //  19 dBm // TODO
    1000,  //  20 dBm // TODO
    1000,  //  21 dBm // TODO
    1000,  //  22 dBm // TODO
};

/**
 * @brief Power consumption table in Tx High frequency mode
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_dcdc_hf_vreg[] = {
    8625,   // -17 dBm
    8783,   // -16 dBm
    8874,   // -15 dBm
    8998,   // -14 dBm
    9145,   // -13 dBm
    9289,   // -12 dBm
    9481,   // -11 dBm
    9646,   // -10 dBm
    9840,   //  -9 dBm
    10073,  //  -8 dBm
    10280,  //  -7 dBm
    10597,  //  -6 dBm
    10511,  //  -5 dBm
    11200,  //  -4 dBm
    11391,  //  -3 dBm
    11630,  //  -2 dBm
    11360,  //  -1 dBm
    11635,  //   0 dBm
    11986,  //   1 dBm
    12670,  //   2 dBm
    13292,  //   3 dBm
    13859,  //   4 dBm
    13685,  //   5 dBm
    14629,  //   6 dBm
    15388,  //   7 dBm
    15804,  //   8 dBm
    17270,  //   9 dBm
    18943,  //  10 dBm
    21041,  //  11 dBm
    22524,  //  12 dBm
};

/**
 * @brief Power consumption table in Rx Low frequency mode
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_lf_vreg[] = {
    5700,  // RAL_LORA_BW_125_KHZ
    5790,  // RAL_LORA_BW_250_KHZ
    6320,  // RAL_LORA_BW_400_KHZ
    6140,  // RAL_LORA_BW_500_KHZ
    6840,  // RAL_LORA_BW_800_KHZ
    6600,  // RAL_LORA_BW_1000_KHZ
};

/**
 * @brief Power consumption table in Rx High frequency mode
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_hf_vreg[] = {
    6400,  // RAL_LORA_BW_125_KHZ // TODO
    6510,  // RAL_LORA_BW_250_KHZ // TODO
    7040,  // RAL_LORA_BW_400_KHZ // TODO
    6860,  // RAL_LORA_BW_500_KHZ // TODO
    6880,  // RAL_LORA_BW_800_KHZ
    7350,  // RAL_LORA_BW_1000_KHZ //TODO
};

/**
 * @brief Power consumption table in Rx Low frequency mode + Boosted
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_lf_vreg_boosted[] = {
    6920,  // RAL_LORA_BW_125_KHZ
    6990,  // RAL_LORA_BW_250_KHZ
    7530,  // RAL_LORA_BW_400_KHZ
    7370,  // RAL_LORA_BW_500_KHZ
    8160,  // RAL_LORA_BW_800_KHZ
    7820,  // RAL_LORA_BW_1000_KHZ
};

/**
 * @brief Power consumption table in Rx High frequency mode + Boosted
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_hf_vreg_boosted[] = {
    6740,  // RAL_LORA_BW_125_KHZ
    6850,  // RAL_LORA_BW_250_KHZ
    7130,  // RAL_LORA_BW_400_KHZ
    7180,  // RAL_LORA_BW_500_KHZ
    8010,  // RAL_LORA_BW_800_KHZ
    7690,  // RAL_LORA_BW_1000_KHZ
};

/**
 * @brief Power consumption table in Rx Low frequency mode
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_lf_vreg[] = {
    9420,   // RAL_LORA_BW_125_KHZ
    9580,   // RAL_LORA_BW_250_KHZ
    10580,  // RAL_LORA_BW_400_KHZ
    10240,  // RAL_LORA_BW_500_KHZ
    11770,  // RAL_LORA_BW_800_KHZ
    11130,  // RAL_LORA_BW_1000_KHZ
};

/**
 * @brief Power consumption table in Rx High frequency mode
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_hf_vreg[] = {
    10760,  // RAL_LORA_BW_125_KHZ
    10890,  // RAL_LORA_BW_250_KHZ
    11870,  // RAL_LORA_BW_400_KHZ
    11640,  // RAL_LORA_BW_500_KHZ
    13130,  // RAL_LORA_BW_800_KHZ
    12500,  // RAL_LORA_BW_1000_KHZ
};

/**
 * @brief Power consumption table in Rx Low frequency mode + boossted
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_lf_vreg_boosted[] = {
    11670,  // RAL_LORA_BW_125_KHZ
    11710,  // RAL_LORA_BW_250_KHZ
    12880,  // RAL_LORA_BW_400_KHZ
    12560,  // RAL_LORA_BW_500_KHZ
    14090,  // RAL_LORA_BW_800_KHZ
    13500,  // RAL_LORA_BW_1000_KHZ
};

/**
 * @brief Power consumption table in Rx High frequency mode + boosted
 * @remark measured at 3.3v
 */
static const uint32_t ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_hf_vreg_boosted[] = {
    11410,  // RAL_LORA_BW_125_KHZ
    11570,  // RAL_LORA_BW_250_KHZ
    12590,  // RAL_LORA_BW_400_KHZ
    12280,  // RAL_LORA_BW_500_KHZ
    13720,  // RAL_LORA_BW_800_KHZ
    13180,  // RAL_LORA_BW_1000_KHZ
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
void ral_lr20xx_bsp_get_tx_cfg( const void* context, const ral_lr20xx_bsp_tx_cfg_input_params_t* input_params,
                                ral_lr20xx_bsp_tx_cfg_output_params_t* output_params )
{
    // get board tx power offset
    int8_t board_tx_pwr_offset_db = radio_utilities_get_tx_power_offset( );

    int16_t power = input_params->system_output_pwr_in_dbm + board_tx_pwr_offset_db;

    lr20xx_radio_common_pa_selection_t pa_type;

    // check frequency band first to choose Low Frequency of High Frequency Power Amplifier
    if( input_params->freq_in_hz >= 1600000000 )  // 1.6GHz
    {
        pa_type = LR20XX_RADIO_COMMON_PA_SEL_HF;
    }
    else
    {
        pa_type = LR20XX_RADIO_COMMON_PA_SEL_LF;
    }

    // call the configuration function
    lr20xx_get_tx_cfg( pa_type, power, output_params );
}

void ral_lr20xx_bsp_get_rx_cfg( const void* context, const uint32_t freq_in_hz, lr20xx_radio_common_rx_path_t* rx_path,
                                lr20xx_radio_common_rx_path_boost_mode_t* boost_mode )
{
    if( freq_in_hz >= 1600000000 )  // 1.6GHz
    {
        *rx_path = LR20XX_RADIO_COMMON_RX_PATH_HF;
    }
    else
    {
        *rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF;
    }

    *boost_mode = LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE;
}

void ral_lr20xx_bsp_get_front_end_calibration_cfg(
    const void* context, lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures[3] )
{
    lr20xx_radio_common_rx_path_t            rx_path    = LR20XX_RADIO_COMMON_RX_PATH_LF;
    lr20xx_radio_common_rx_path_boost_mode_t boost_mode = LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE;

    uint32_t freq_in_hz[3] = {
        470000000,   // Frequency 0 (range from 430MHz to 510MHz)
        897500000,   // Frequency 1 (range from 867MHz to 928MHz)
        2441000000,  // Frequency 2 (range from 2.403GHz to 2.479GHz)
    };

    for( uint8_t i = 0; i < 3; i++ )
    {
        ral_lr20xx_bsp_get_rx_cfg( context, freq_in_hz[i], &rx_path, &boost_mode );
        front_end_calibration_structures[i].rx_path            = rx_path;
        front_end_calibration_structures[i].frequency_in_hertz = freq_in_hz[i];
    };
}

void ral_lr20xx_bsp_get_dio_function( const void* context, lr20xx_system_dio_t dio, lr20xx_system_dio_func_t* function )
{
    switch( dio )
    {
    case LR20XX_SYSTEM_DIO_5:
        break;
    case LR20XX_SYSTEM_DIO_6:
        break;
    case LR20XX_SYSTEM_DIO_7:
        break;
#if defined( LEGACY_EVK_LR2021 )
    case LR20XX_SYSTEM_DIO_8:
        break;
    case LR20XX_SYSTEM_DIO_9:
        *function = LR20XX_SYSTEM_DIO_FUNC_IRQ;  // LEGACY EVK
        break;
#else
    case LR20XX_SYSTEM_DIO_8:
        *function = LR20XX_SYSTEM_DIO_FUNC_IRQ;  // WIO board
        break;
    case LR20XX_SYSTEM_DIO_9:
        break;
#endif
    case LR20XX_SYSTEM_DIO_10:
        break;
    case LR20XX_SYSTEM_DIO_11:
        break;
    }
}

void ral_lr20xx_bsp_get_dio_sleep_drive( const void* context, lr20xx_system_dio_t dio,
                                         lr20xx_system_dio_drive_t* drive )
{
    *drive = LR20XX_SYSTEM_DIO_DRIVE_NONE;
}

void ral_lr20xx_bsp_get_dio_irq_mask( const void* context, lr20xx_system_dio_t dio, lr20xx_system_irq_mask_t* irq_mask )
{
#if defined( LEGACY_EVK_LR2021 )
    if( dio == LR20XX_SYSTEM_DIO_9 )
    {
        *irq_mask = 0xFFFFFFFF & ~( LR20XX_SYSTEM_IRQ_FIFO_RX | LR20XX_SYSTEM_IRQ_FIFO_TX );
    }
#else
    if( dio == LR20XX_SYSTEM_DIO_8 )
    {
        *irq_mask = 0xFFFFFFFF & ~( LR20XX_SYSTEM_IRQ_FIFO_RX | LR20XX_SYSTEM_IRQ_FIFO_TX );
    }
#endif
}

void ral_lr20xx_bsp_get_dio_rf_switch_cfg( const void* context, lr20xx_system_dio_t dio,
                                           lr20xx_system_dio_rf_switch_cfg_t* rf_switch_cfg )
{
    switch( dio )
    {
    case LR20XX_SYSTEM_DIO_5:
        *rf_switch_cfg = LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF | LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF;
        break;
    case LR20XX_SYSTEM_DIO_6:
        *rf_switch_cfg = LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF | LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF |
                         LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF | LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF;
        break;
    default:
        break;
    }
}

void ral_lr20xx_bsp_get_reg_mode( const void* context, lr20xx_system_reg_mode_t* reg_mode )
{
    *reg_mode = LR20XX_SYSTEM_REG_MODE_DCDC;
}

void ral_lr20xx_bsp_get_dio_hf_clk_scaling_cfg( const void* context, lr20xx_system_hf_clk_scaling_t* hf_clk_scaling )
{
    // Unneeded
}

void ral_bsp_lr20xx_get_lfclk_cfg( const void* context, lr20xx_system_lfclk_cfg_t* lfclk_cfg )
{
    *lfclk_cfg = LR20XX_SYSTEM_LFCLK_RC;
}

void ral_lr20xx_bsp_get_xosc_cfg( const void* context, ral_xosc_cfg_t* xosc_cfg,
                                  lr20xx_system_tcxo_supply_voltage_t* supply_voltage, uint32_t* startup_time_in_tick )
{
    *xosc_cfg             = RAL_XOSC_CFG_XTAL;
    *supply_voltage       = LR20XX_SYSTEM_TCXO_CTRL_1_8V;
    *startup_time_in_tick = 0;  // lr20xx_radio_common_convert_time_in_ms_to_rtc_step(10);
}

void ral_lr20xx_bsp_get_lora_cad_det_peak( const void* context, ral_lora_sf_t sf, ral_lora_cad_symbs_t nb_symbol,
                                           uint8_t* in_out_cad_det_peak )
{
    // Function used to fine tune the cad detection peak, update if needed
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr20xx_get_tx_cfg( lr20xx_radio_common_pa_selection_t pa_type, int8_t expected_output_pwr_in_dbm,
                        ral_lr20xx_bsp_tx_cfg_output_params_t* output_params )
{
    int8_t power = expected_output_pwr_in_dbm;

    // Ramp time is the same for any config
    output_params->pa_ramp_time = LR20XX_RADIO_COMMON_RAMP_48_US;

    switch( pa_type )
    {
    case LR20XX_RADIO_COMMON_PA_SEL_LF:
    {
        // Check power boundaries for LP LF PA: The output power must be in range [ -17 , +15 ] dBm
        if( power < LR20XX_LF_MIN_OUTPUT_POWER )
        {
            power = LR20XX_LF_MIN_OUTPUT_POWER;
        }
        else if( power > LR20XX_LF_MAX_OUTPUT_POWER )
        {
            power = LR20XX_LF_MAX_OUTPUT_POWER;
        }
        output_params->pa_cfg.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_LF;
        output_params->pa_cfg.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        output_params->pa_cfg.pa_lf_slices     = pa_lf_cfg_table[power - LR20XX_LF_MIN_OUTPUT_POWER].pa_lf_slices;
        output_params->pa_cfg.pa_lf_duty_cycle = pa_lf_cfg_table[power - LR20XX_LF_MIN_OUTPUT_POWER].pa_duty_cycle;
        output_params->pa_cfg.pa_hf_duty_cycle = 16;

        output_params->chip_output_half_pwr_in_dbm_configured =
            pa_lf_cfg_table[power - LR20XX_LF_MIN_OUTPUT_POWER].half_power;
        output_params->chip_output_pwr_in_dbm_expected = power;
        break;
    }

    case LR20XX_RADIO_COMMON_PA_SEL_HF:
    {
        // Check power boundaries for HF PA: The output power must be in range [ -17 , +12 ] dBm
        if( power < LR20XX_HF_MIN_OUTPUT_POWER )
        {
            power = LR20XX_HF_MIN_OUTPUT_POWER;
        }
        else if( power > LR20XX_HF_MAX_OUTPUT_POWER )
        {
            power = LR20XX_HF_MAX_OUTPUT_POWER;
        }
        output_params->pa_cfg.pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF;
        output_params->pa_cfg.pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
        output_params->pa_cfg.pa_lf_slices     = pa_hf_cfg_table[power - LR20XX_HF_MIN_OUTPUT_POWER].pa_lf_slices;
        output_params->pa_cfg.pa_lf_duty_cycle = 6;
        output_params->pa_cfg.pa_hf_duty_cycle = pa_hf_cfg_table[power - LR20XX_HF_MIN_OUTPUT_POWER].pa_duty_cycle;

        output_params->chip_output_half_pwr_in_dbm_configured =
            pa_hf_cfg_table[power - LR20XX_HF_MIN_OUTPUT_POWER].half_power;
        output_params->chip_output_pwr_in_dbm_expected = power;
        break;
    }
    }
}

ral_status_t ral_lr20xx_bsp_get_instantaneous_tx_power_consumption( const void* context,
                                                                    const ral_lr20xx_bsp_tx_cfg_output_params_t* tx_cfg,
                                                                    lr20xx_system_reg_mode_t radio_reg_mode,
                                                                    uint32_t*                pwr_consumption_in_ua )
{
    if( tx_cfg->pa_cfg.pa_sel == LR20XX_RADIO_COMMON_PA_SEL_LF )
    {
        uint8_t index = 0;

        if( tx_cfg->chip_output_pwr_in_dbm_expected > LR20XX_LF_MAX_OUTPUT_POWER )
        {
            index = LR20XX_LF_MAX_OUTPUT_POWER - LR20XX_LF_MIN_OUTPUT_POWER;
        }
        else if( tx_cfg->chip_output_pwr_in_dbm_expected < LR20XX_LF_MIN_OUTPUT_POWER )
        {
            index = 0;
        }
        else
        {
            index = tx_cfg->chip_output_pwr_in_dbm_expected - LR20XX_LF_MIN_OUTPUT_POWER;
        }

        if( radio_reg_mode == LR20XX_SYSTEM_REG_MODE_DCDC )
        {
            *pwr_consumption_in_ua = ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_dcdc_lf_vreg[index];
        }
        else
        {
            *pwr_consumption_in_ua = ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_ldo_lf_vreg[index];
        }
    }
    else if( tx_cfg->pa_cfg.pa_sel == LR20XX_RADIO_COMMON_PA_SEL_HF )
    {
        uint8_t index = 0;

        if( tx_cfg->chip_output_pwr_in_dbm_expected > LR20XX_HF_MAX_OUTPUT_POWER )
        {
            index = LR20XX_HF_MAX_OUTPUT_POWER - LR20XX_HF_MIN_OUTPUT_POWER;
        }
        else if( tx_cfg->chip_output_pwr_in_dbm_expected < LR20XX_HF_MIN_OUTPUT_POWER )
        {
            index = 0;
        }
        else
        {
            index = tx_cfg->chip_output_pwr_in_dbm_expected - LR20XX_HF_MIN_OUTPUT_POWER;
        }

        if( radio_reg_mode == LR20XX_SYSTEM_REG_MODE_DCDC )
        {
            *pwr_consumption_in_ua = ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_dcdc_hf_vreg[index];
        }
        else
        {
            return RAL_STATUS_UNSUPPORTED_FEATURE;
        }
    }
    else
    {
        return RAL_STATUS_UNKNOWN_VALUE;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_bsp_get_instantaneous_gfsk_rx_power_consumption( const void*              context,
                                                                         lr20xx_system_reg_mode_t radio_reg_mode,
                                                                         bool                     rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua )
{
    if( radio_reg_mode == LR20XX_SYSTEM_REG_MODE_DCDC )
    {
        *pwr_consumption_in_ua =
            ( rx_boosted ) ? LR20XX_GFSK_RX_BOOSTED_CONSUMPTION_DCDC : LR20XX_GFSK_RX_CONSUMPTION_DCDC;
    }
    else
    {
        // TODO: find the good values
        *pwr_consumption_in_ua =
            ( rx_boosted ) ? LR20XX_GFSK_RX_BOOSTED_CONSUMPTION_LDO : LR20XX_GFSK_RX_CONSUMPTION_LDO;
    }

    return RAL_STATUS_OK;
}

ral_status_t ral_lr20xx_bsp_get_instantaneous_lora_rx_power_consumption( const void*              context,
                                                                         lr20xx_system_reg_mode_t radio_reg_mode,
                                                                         const ral_lora_bw_t bw, const bool rx_boosted,
                                                                         uint32_t* pwr_consumption_in_ua )
{
    lr20xx_radio_common_rx_path_t rfi   = LR20XX_RADIO_COMMON_RX_PATH_LF;
    uint8_t                       index = 0;

    if( bw <= RAL_LORA_BW_125_KHZ )
    {
        rfi   = LR20XX_RADIO_COMMON_RX_PATH_LF;
        index = 0;
    }
    else if( bw == RAL_LORA_BW_250_KHZ )
    {
        rfi   = LR20XX_RADIO_COMMON_RX_PATH_LF;
        index = 1;
    }
    else if( bw == RAL_LORA_BW_400_KHZ )
    {
        rfi   = LR20XX_RADIO_COMMON_RX_PATH_LF;
        index = 2;
    }
    else if( bw == RAL_LORA_BW_500_KHZ )
    {
        rfi   = LR20XX_RADIO_COMMON_RX_PATH_LF;
        index = 3;
    }
    else if( bw == RAL_LORA_BW_800_KHZ )
    {  // Select Rx HF path
        rfi   = LR20XX_RADIO_COMMON_RX_PATH_HF;
        index = 4;
    }
    else if( bw >= RAL_LORA_BW_1000_KHZ )
    {  // Select Rx HF path
        rfi   = LR20XX_RADIO_COMMON_RX_PATH_HF;
        index = 5;
    }

    if( radio_reg_mode == LR20XX_SYSTEM_REG_MODE_DCDC )
    {
        if( rfi == LR20XX_RADIO_COMMON_RX_PATH_LF )
        {
            *pwr_consumption_in_ua = ( rx_boosted )
                                         ? ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_lf_vreg[index]
                                         : ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_lf_vreg_boosted[index];
        }
        else  // HF path
        {
            *pwr_consumption_in_ua = ( rx_boosted )
                                         ? ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_hf_vreg[index]
                                         : ral_lr20xx_convert_rx_bw_to_ua_reg_mode_dcdc_hf_vreg_boosted[index];
        }
    }
    else  // LDO mode
    {
        if( rfi == LR20XX_RADIO_COMMON_RX_PATH_LF )
        {
            *pwr_consumption_in_ua = ( rx_boosted )
                                         ? ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_lf_vreg[index]
                                         : ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_lf_vreg_boosted[index];
        }
        else  // HF path
        {
            *pwr_consumption_in_ua = ( rx_boosted )
                                         ? ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_hf_vreg[index]
                                         : ral_lr20xx_convert_rx_bw_to_ua_reg_mode_ldo_hf_vreg_boosted[index];
        }
    }

    return RAL_STATUS_OK;
}
/* --- EOF ------------------------------------------------------------------ */
