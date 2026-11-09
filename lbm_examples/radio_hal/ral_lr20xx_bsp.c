/**
 * @file      ral_lr20xx_bsp.c
 *
 * @brief     Board Support Package for the LR20xx-specific Radio Abstraction Layer.
 *
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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
#include <stddef.h>
#include "lr20xx_radio_common.h"
#include "lr20xx_system_types.h"
#include "pa_tables_lr20xx_common.h"
#include "pa_tables_lr20xx_types.h"
#include "ral_lr20xx_bsp.h"
#include "radio_utilities.h"
#include "smtc_hal_mcu.h"

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
 * --- PRIVATE CONST -----------------------------------------------------------
 */

/* The following rssi calibration gain tables is formating as follow : gain_value [10 bits signed fixed 8.2] -
 * nois_figure [unsigned fixed 6.2]*/

const lr20xx_radio_common_rssi_calibration_gain_table_t table_lf_868_915 = {
    .g1         = { .gain_value = -32, .noise_figure = 219 },
    .g2         = { .gain_value = -8, .noise_figure = 204 },
    .g3         = { .gain_value = 16, .noise_figure = 194 },
    .g4         = { .gain_value = 36, .noise_figure = 175 },
    .g5         = { .gain_value = 56, .noise_figure = 156 },
    .g6         = { .gain_value = 73, .noise_figure = 137 },
    .g7         = { .gain_value = 90, .noise_figure = 122 },
    .g8         = { .gain_value = 112, .noise_figure = 95 },
    .g9         = { .gain_value = 130, .noise_figure = 62 },
    .g10        = { .gain_value = 152, .noise_figure = 41 },
    .g11        = { .gain_value = 173, .noise_figure = 26 },
    .g12_boost0 = { .gain_value = 190, .noise_figure = 16 },
    .g12_boost1 = { .gain_value = 196, .noise_figure = 13 },
    .g12_boost2 = { .gain_value = 200, .noise_figure = 13 },
    .g12_boost3 = { .gain_value = 202, .noise_figure = 13 },
    .g12_boost4 = { .gain_value = 203, .noise_figure = 12 },
    .g12_boost5 = { .gain_value = 205, .noise_figure = 12 },
    .g12_boost6 = { .gain_value = 207, .noise_figure = 12 },
    .g12_boost7 = { .gain_value = 208, .noise_figure = 12 },
    .g13_boost0 = { .gain_value = 214, .noise_figure = 12 },
    .g13_boost1 = { .gain_value = 220, .noise_figure = 11 },
    .g13_boost2 = { .gain_value = 223, .noise_figure = 11 },
    .g13_boost3 = { .gain_value = 226, .noise_figure = 10 },
    .g13_boost4 = { .gain_value = 227, .noise_figure = 10 },
    .g13_boost5 = { .gain_value = 228, .noise_figure = 11 },
    .g13_boost6 = { .gain_value = 230, .noise_figure = 10 },
    .g13_boost7 = { .gain_value = 232, .noise_figure = 9 }
};

/*const lr20xx_radio_common_rssi_calibration_gain_table_t table_lf_490 =
{
    .g1 = {.gain_value = -46, .noise_figure = 221},
    .g2 = {.gain_value = -22, .noise_figure = 205},
    .g3 = {.gain_value = 2, .noise_figure = 190},
    .g4 = {.gain_value = 24, .noise_figure = 169},
    .g5 = {.gain_value = 44, .noise_figure = 147},
    .g6 = {.gain_value = 63, .noise_figure = 130},
    .g7 = {.gain_value = 84, .noise_figure = 113},
    .g8 = {.gain_value = 104, .noise_figure = 87},
    .g9 = {.gain_value = 138, .noise_figure = 46},
    .g10 = {.gain_value = 158, .noise_figure = 30},
    .g11 = {.gain_value = 181, .noise_figure = 18},
    .g12_boost0 = {.gain_value = 200, .noise_figure = 13},
    .g12_boost1 = {.gain_value = 196, .noise_figure = 13},
    .g12_boost2 = {.gain_value = 208, .noise_figure = 13},
    .g12_boost3 = {.gain_value = 210, .noise_figure = 13},
    .g12_boost4 = {.gain_value = 211, .noise_figure = 13},
    .g12_boost5 = {.gain_value = 215, .noise_figure = 12},
    .g12_boost6 = {.gain_value = 217, .noise_figure = 12},
    .g12_boost7 = {.gain_value = 218, .noise_figure = 12},
    .g13_boost0 = {.gain_value = 224, .noise_figure = 12},
    .g13_boost1 = {.gain_value = 228, .noise_figure = 12},
    .g13_boost2 = {.gain_value = 231, .noise_figure = 12},
    .g13_boost3 = {.gain_value = 234, .noise_figure = 12},
    .g13_boost4 = {.gain_value = 237, .noise_figure = 11},
    .g13_boost5 = {.gain_value = 238, .noise_figure = 11},
    .g13_boost6 = {.gain_value = 240, .noise_figure = 10},
    .g13_boost7 = {.gain_value = 242, .noise_figure = 10}
};*/

const lr20xx_radio_common_rssi_calibration_gain_table_t table_hf_2G4 = {
    .g1         = { .gain_value = -58, .noise_figure = 240 },
    .g2         = { .gain_value = -34, .noise_figure = 221 },
    .g3         = { .gain_value = -10, .noise_figure = 208 },
    .g4         = { .gain_value = 7, .noise_figure = 190 },
    .g5         = { .gain_value = 25, .noise_figure = 171 },
    .g6         = { .gain_value = 40, .noise_figure = 153 },
    .g7         = { .gain_value = 53, .noise_figure = 137 },
    .g8         = { .gain_value = 77, .noise_figure = 113 },
    .g9         = { .gain_value = 102, .noise_figure = 89 },
    .g10        = { .gain_value = 126, .noise_figure = 65 },
    .g11        = { .gain_value = 146, .noise_figure = 47 },
    .g12_boost0 = { .gain_value = 163, .noise_figure = 34 },
    .g12_boost1 = { .gain_value = 166, .noise_figure = 32 },
    .g12_boost2 = { .gain_value = 168, .noise_figure = 31 },
    .g12_boost3 = { .gain_value = 170, .noise_figure = 30 },
    .g12_boost4 = { .gain_value = 171, .noise_figure = 30 },
    .g12_boost5 = { .gain_value = 172, .noise_figure = 29 },
    .g12_boost6 = { .gain_value = 176, .noise_figure = 26 },
    .g12_boost7 = { .gain_value = 175, .noise_figure = 27 },
    .g13_boost0 = { .gain_value = 187, .noise_figure = 29 },
    .g13_boost1 = { .gain_value = 190, .noise_figure = 29 },
    .g13_boost2 = { .gain_value = 193, .noise_figure = 27 },
    .g13_boost3 = { .gain_value = 194, .noise_figure = 26 },
    .g13_boost4 = { .gain_value = 195, .noise_figure = 26 },
    .g13_boost5 = { .gain_value = 196, .noise_figure = 25 },
    .g13_boost6 = { .gain_value = 198, .noise_figure = 24 },
    .g13_boost7 = { .gain_value = 199, .noise_figure = 24 }
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

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

void lr20xx_get_tx_cfg( lr20xx_radio_common_pa_selection_t pa_type, uint32_t rf_freq_in_hz,
                        int8_t expected_output_pwr_in_dbm, ral_lr20xx_bsp_tx_cfg_output_params_t* output_params );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Power consumption table in Tx Low frequency mode
 * @remark measured at 3.3v and DCDC
 */
static const uint32_t ral_lr20xx_convert_tx_dbm_to_ua_reg_mode_dcdc_lf_vreg[] = {
    6970,   // -10 dBm
    7110,   //  -9 dBm
    7270,   //  -8 dBm
    7000,   //  -7 dBm
    7200,   //  -6 dBm
    7390,   //  -5 dBm
    7920,   //  -4 dBm
    8660,   //  -3 dBm
    8520,   //  -2 dBm
    8830,   //  -1 dBm
    8500,   //   0 dBm
    8820,   //   1 dBm
    9300,   //   2 dBm
    9710,   //   3 dBm
    10200,  //   4 dBm
    12180,  //   5 dBm
    11630,  //   6 dBm
    12780,  //   7 dBm
    13580,  //   8 dBm
    14810,  //   9 dBm
    16500,  //  10 dBm
    18290,  //  11 dBm
    19810,  //  12 dBm
    22880,  //  13 dBm
    26490,  //  14 dBm
    30980,  //  15 dBm
    36170,  //  16 dBm
    43010,  //  17 dBm
    51400,  //  18 dBm
    60980,  //  19 dBm
    71710,  //  20 dBm
    86560,  //  21 dBm
    100480  //  22 dBm
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
    8320,   // -17 dBm
    8460,   // -16 dBm
    8600,   // -15 dBm
    8650,   // -14 dBm
    8830,   // -13 dBm
    9010,   // -12 dBm
    9190,   // -11 dBm
    9410,   // -10 dBm
    9460,   //  -9 dBm
    9520,   //  -8 dBm
    9950,   //  -7 dBm
    10160,  //  -6 dBm
    10450,  //  -5 dBm
    10290,  //  -4 dBm
    10750,  //  -3 dBm
    11520,  //  -2 dBm
    11550,  //  -1 dBm
    11400,  //   0 dBm
    11810,  //   1 dBm
    12430,  //   2 dBm
    12950,  //   3 dBm
    13550,  //   4 dBm
    14130,  //   5 dBm
    14850,  //   6 dBm
    15520,  //   7 dBm
    15810,  //   8 dBm
    16670,  //   9 dBm
    18280,  //  10 dBm
    20140,  //  11 dBm
    22270   //  12 dBm
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
    // (smtc-shields is the source of truth for the exact frequency/band mapping)
    if( input_params->freq_in_hz >= SMTC_SHIELD_LR20XX_LF_HF_FREQ_THRESHOLD )
    {
        pa_type = LR20XX_RADIO_COMMON_PA_SEL_HF;
    }
    else
    {
        pa_type = LR20XX_RADIO_COMMON_PA_SEL_LF;
    }

    // call the configuration function
    lr20xx_get_tx_cfg( pa_type, input_params->freq_in_hz, power, output_params );
}

void ral_lr20xx_bsp_get_rx_cfg( const void* context, const uint32_t freq_in_hz, lr20xx_radio_common_rx_path_t* rx_path,
                                lr20xx_radio_common_rx_path_boost_mode_t* boost_mode )
{
    if( freq_in_hz >= 1500000000 )  // 1.5GHz
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
        870000000,   // Frequency 0 (range from 850MHz to 890MHz) +/- 20MHz
        920000000,   // Frequency 1 (range from 900MHz to 940MHz) +/- 20MHz
        2441000000,  // Frequency 2 (range from 2.391GHz to 2.491GHz) +/- 50MHz
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
#if defined( LEGACY_EVK_LR2021 )
    case LR20XX_SYSTEM_DIO_7:
        break;
    case LR20XX_SYSTEM_DIO_8:
        break;
    case LR20XX_SYSTEM_DIO_9:
        *function = LR20XX_SYSTEM_DIO_FUNC_IRQ;  // RADIO IRQ
        break;
#else
    case LR20XX_SYSTEM_DIO_7:
        *function = LR20XX_SYSTEM_DIO_FUNC_IRQ;  // FIFO IRQs
        break;
    case LR20XX_SYSTEM_DIO_8:
        *function = LR20XX_SYSTEM_DIO_FUNC_IRQ;  // Radio classic IRQs
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
    ( void ) dio;
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
    if( dio == LR20XX_SYSTEM_DIO_8 )  // Main IRQ - All but FIFO RX/TX
    {
        *irq_mask = 0xFFFFFFFF & ~( LR20XX_SYSTEM_IRQ_FIFO_RX | LR20XX_SYSTEM_IRQ_FIFO_TX );
    }
    if( dio == LR20XX_SYSTEM_DIO_7 )  // FIFO IRQ - FIFO RX and TX
    {
        *irq_mask = ( LR20XX_SYSTEM_IRQ_FIFO_RX | LR20XX_SYSTEM_IRQ_FIFO_TX );
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

void lr20xx_get_tx_cfg( lr20xx_radio_common_pa_selection_t pa_type, uint32_t rf_freq_in_hz,
                        int8_t expected_output_pwr_in_dbm, ral_lr20xx_bsp_tx_cfg_output_params_t* output_params )
{
    int8_t power = expected_output_pwr_in_dbm;

    // Ramp time is the same for any config.
    output_params->pa_ramp_time = LR20XX_RADIO_COMMON_RAMP_48_US;

    // PA table and RSSI calibration data come from smtc-shields (source of truth for the
    // board-characterized values); smtc-shields picks the exact frequency sub-band internally.
    const smtc_shield_lr20xx_pa_pwr_cfg_t* cfg = NULL;

    switch( pa_type )
    {
    case LR20XX_RADIO_COMMON_PA_SEL_LF:
    {
        // Check power boundaries for LP LF PA: The output power must be in range [ -10 , +22 ] dBm
        if( power < LR20XX_LF_MIN_OUTPUT_POWER )
        {
            power = LR20XX_LF_MIN_OUTPUT_POWER;
        }
        else if( power > LR20XX_LF_MAX_OUTPUT_POWER )
        {
            power = LR20XX_LF_MAX_OUTPUT_POWER;
        }

        cfg = smtc_shield_lr20xx_common_get_pa_pwr_cfg( rf_freq_in_hz, power * 2 );
        if( cfg == NULL )
        {
            mcu_panic( "Unsupported LF frequency for PA config: %u Hz\n", rf_freq_in_hz );
        }
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

        cfg = smtc_shield_lr20xx_common_get_pa_pwr_cfg( rf_freq_in_hz, power * 2 );
        if( cfg == NULL )
        {
            mcu_panic( "Unsupported HF frequency for PA config: %u Hz\n", rf_freq_in_hz );
        }
        break;
    }
    }

    output_params->pa_cfg                                 = cfg->pa_config;
    output_params->chip_output_half_pwr_in_dbm_configured = cfg->power;
    output_params->chip_output_pwr_in_dbm_expected         = power;
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

void ral_lr20xx_bsp_get_rssi_calibration_table(
    const void* context, const lr20xx_radio_common_rssi_calibration_gain_table_t** rssi_calibration_table_lf,
    const lr20xx_radio_common_rssi_calibration_gain_table_t** rssi_calibration_table_hf )
{
    // Make sure pointers is not affected
    *rssi_calibration_table_lf = NULL;
    *rssi_calibration_table_hf = NULL;

    /*Getting High Frequency Calibration Table*/

    // Choosing HF 2.4GHz
    *rssi_calibration_table_hf = &table_hf_2G4;

    /*Getting Low Frequency Calibration Table*/

    // Choosing LF 868 MHz to 915 MHz
    *rssi_calibration_table_lf = &table_lf_868_915;

    // Choosing LF 490 MHz
    //*rssi_calibration_table_lf = &table_lf_490;
}
/* --- EOF ------------------------------------------------------------------ */
