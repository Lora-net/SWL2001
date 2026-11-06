/*!
 * @file      lr20xx_radio_common_types.h
 *
 * @brief     Radio common driver types for LR20XX
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

#ifndef LR20XX_RADIO_COMMON_TYPES_H
#define LR20XX_RADIO_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

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

/*!
 * @brief Rx path values
 */
typedef enum
{
    LR20XX_RADIO_COMMON_RX_PATH_LF = 0x00,  //!< Low-frequency Rx path
    LR20XX_RADIO_COMMON_RX_PATH_HF = 0x01,  //!< High-frequency Rx path
} lr20xx_radio_common_rx_path_t;

/**
 * @brief Raw front end calibration value
 *
 * MSBit is the Rx path selection:
 * - 0: LF path
 * - 1: HF path
 *
 * The remaining 15 LSbits are the frequency expressed as 4MHz steps.
 *
 * For instance:
 * - 0x80E1 means 900MHz on HF path
 * - 0x00E1 means 900MHz on LF path
 *
 */
typedef uint16_t lr20xx_radio_common_raw_front_end_calibration_value_t;

/**
 * @brief Helper structure for front end calibration value
 *
 * @see lr20xx_radio_common_raw_front_end_calibration_value_t
 */
typedef struct
{
    lr20xx_radio_common_rx_path_t rx_path;             //!< The RX path to calibrate
    uint32_t                      frequency_in_hertz;  //!< The frequency to calibrate
} lr20xx_radio_common_front_end_calibration_value_t;

/*!
 * @brief Rx path boost configuration values
 */
typedef enum
{
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE = 0x00,  //!< Boost deactivated
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_1    = 0x01,  //!< Boost mode 1
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_2    = 0x02,  //!< Boost mode 2
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_3    = 0x03,  //!< Boost mode 3
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4    = 0x04,  //!< Boost mode 4
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_5    = 0x05,  //!< Boost mode 5
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_6    = 0x06,  //!< Boost mode 6
    LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_7    = 0x07,  //!< Boost mode 7
} lr20xx_radio_common_rx_path_boost_mode_t;

/*!
 * @brief Power Amplifier Selection values
 */
typedef enum
{
    LR20XX_RADIO_COMMON_PA_SEL_LF = 0x00,  //!< Low-frequency Power Amplifier
    LR20XX_RADIO_COMMON_PA_SEL_HF = 0x01,  //!< High-frequency Power Amplifier
} lr20xx_radio_common_pa_selection_t;

/*!
 * @brief Power Amplifier Low-Frequency mode
 */
typedef enum lr20xx_radio_common_pa_lf_mode_e
{
    LR20XX_RADIO_COMMON_PA_LF_MODE_FSM         = 0x00,  //!< Full Single-ended Mode
    LR20XX_RADIO_COMMON_PA_LF_MODE_FDM         = 0x01,  //!< Full Differential Mode
    LR20XX_RADIO_COMMON_PA_LF_MODE_HSM_RFO_LF1 = 0x02,  //!< Half Single-ended Mode on RF output LF 1
    LR20XX_RADIO_COMMON_PA_LF_MODE_HSM_RFO_LF2 = 0x03,  //!< Half Single-ended Mode on RF output LF 2
} lr20xx_radio_common_pa_lf_mode_t;

/*!
 * @brief Ramping time for PA
 *
 * This parameter is the ramping time of the PA. A high value improves spectral quality.
 */
typedef enum
{
    LR20XX_RADIO_COMMON_RAMP_2_US   = 0x00,  //!< 2 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_4_US   = 0x01,  //!< 4 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_8_US   = 0x02,  //!< 8 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_16_US  = 0x03,  //!< 16 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_32_US  = 0x04,  //!< 32 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_48_US  = 0x05,  //!< 48 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_64_US  = 0x06,  //!< 64 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_80_US  = 0x07,  //!< 80 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_96_US  = 0x08,  //!< 96 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_112_US = 0x09,  //!< 112 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_128_US = 0x0A,  //!< 128 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_144_US = 0x0B,  //!< 144 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_160_US = 0x0C,  //!< 160 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_176_US = 0x0D,  //!< 176 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_192_US = 0x0E,  //!< 192 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_208_US = 0x0F,  //!< 208 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_240_US = 0x10,  //!< 240 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_272_US = 0x11,  //!< 272 us Ramp Time
    LR20XX_RADIO_COMMON_RAMP_304_US = 0x12,  //!< 304 us Ramp Time
} lr20xx_radio_common_ramp_time_t;

/*!
 * @brief Chip mode after leaving transmission or reception mode
 */
typedef enum lr20xx_radio_common_fallback_modes_e
{
    LR20XX_RADIO_FALLBACK_STDBY_RC   = 0x01,  //!< Standby RC (Default)
    LR20XX_RADIO_FALLBACK_STDBY_XOSC = 0x02,  //!< Standby XOSC
    LR20XX_RADIO_FALLBACK_FS         = 0x03   //!< FS
} lr20xx_radio_common_fallback_modes_t;

/*!
 * @brief Packet type values
 */
typedef enum
{
    LR20XX_RADIO_COMMON_PKT_TYPE_LORA = 0x00,          //!< LoRa packet engine (default)
    LR20XX_RADIO_COMMON_PKT_TYPE_FSK  = 0x02,          //!< FSK packet engine - configuration compatible with
                                                       //!< existing transceivers (SX127x / SX126x / SX128x / LR11xx)
    LR20XX_RADIO_COMMON_PKT_TYPE_BLUETOOTH_LE = 0x03,  //!< Bluetooth_LE packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_RTTOF        = 0x04,  //!< RTToF packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_FLRC         = 0x05,  //!< FLRC packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_BPSK         = 0x06,  //!< BPSK packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_LRFHSS       = 0x07,  //!< LR-FHSS packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_WM_BUS       = 0x08,  //!< Wireless M-Bus packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_WI_SUN       = 0x09,  //!< Wi-SUN packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_OOK          = 0x0A,  //!< OOK packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_Z_WAVE       = 0x0C,  //!< Z-Wave packet engine
    LR20XX_RADIO_COMMON_PKT_TYPE_OQPSK_15_4   = 0x0D,  //!< OQPSK 15.4 packet engine
} lr20xx_radio_common_pkt_type_t;

/*!
 * @brief RX Duty Cycle Modes
 */
typedef enum
{
    LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_RX  = 0x00,  //!< LoRa/GFSK: Uses Rx for listening to packets
    LR20XX_RADIO_COMMON_RX_DUTY_CYCLE_MODE_CAD = 0x01,  //!< LoRa only: Uses CAD to listen for over-the-air activity
} lr20xx_radio_common_rx_duty_cycle_mode_t;

/*!
 * @brief Timestamp configuration slot
 */
typedef enum
{
    LR20XX_RADIO_COMMON_TIMESTAMP_CFG_SLOT_0 = 0x00,  //!< Timestamp source configuration slot 0
    LR20XX_RADIO_COMMON_TIMESTAMP_CFG_SLOT_1 = 0x01,  //!< Timestamp source configuration slot 1
    LR20XX_RADIO_COMMON_TIMESTAMP_CFG_SLOT_2 = 0x02,  //!< Timestamp source configuration slot 2
} lr20xx_radio_common_timestamp_cfg_slot_t;

/*!
 * @brief Timestamp source
 */
typedef enum
{
    LR20XX_RADIO_COMMON_TIMESTAMP_SOURCE_NONE    = 0x00,  //!< Timestamp deactivated
    LR20XX_RADIO_COMMON_TIMESTAMP_SOURCE_TX_DONE = 0x01,  //!< Timestamp triggered on last payload bit/symbol sent
    LR20XX_RADIO_COMMON_TIMESTAMP_SOURCE_RX_DONE = 0x02,  //!< Timestamp triggered on last payload bit/symbol received
    LR20XX_RADIO_COMMON_TIMESTAMP_SOURCE_SYNC    = 0x03,  //!< Timestamp triggered on last syncword bit/symbol received
    LR20XX_RADIO_COMMON_TIMESTAMP_SOURCE_HEADER =
        0x04,  //!< Timestamp triggered on last header bit/symbol received (LoRa only)
} lr20xx_radio_common_timestamp_source_t;

/*!
 * @brief Tx test modes
 */
typedef enum
{
    LR20XX_RADIO_COMMON_TX_TEST_MODE_NORMAL =
        0x00,  //!< Equivalent to lr20xx_radio_common_set_tx_with_timeout_in_rtc_step with @p timeout_in_ms set to 0
    LR20XX_RADIO_COMMON_TX_TEST_MODE_INFINITE_PREAMBLE =
        0x01,  //!< Generate an infinite preamble (not available with LR-FHSS)
    LR20XX_RADIO_COMMON_TX_TEST_MODE_CONTINUOUS_WAVE = 0x02,  //!< Generate continuous wave (not available with LR-FHSS)
    LR20XX_RADIO_COMMON_TX_TEST_MODE_PRBS9 =
        0x03,  //!< Generate a PseudoRandom Binary Sequence (not available with LoRa nor LR-FHSS)
} lr20xx_radio_common_tx_test_mode_t;

/*!
 * @brief Structure to define RSSI calibration gain item
 */
typedef struct
{
    uint16_t gain_value;    //!< Gain value expressed on 10 bits fix point decimal format 8.2
    uint8_t  noise_figure;  //!< Noise figure value expressed on 8 bits unsigned fix point decimal format 6.2
} lr20xx_radio_common_rssi_calibration_gain_item_t;

/*!
 * @brief Structure to define RSSI calibration gain table for one RF path
 */
typedef struct
{
    lr20xx_radio_common_rssi_calibration_gain_item_t g1;
    lr20xx_radio_common_rssi_calibration_gain_item_t g2;
    lr20xx_radio_common_rssi_calibration_gain_item_t g3;
    lr20xx_radio_common_rssi_calibration_gain_item_t g4;
    lr20xx_radio_common_rssi_calibration_gain_item_t g5;
    lr20xx_radio_common_rssi_calibration_gain_item_t g6;
    lr20xx_radio_common_rssi_calibration_gain_item_t g7;
    lr20xx_radio_common_rssi_calibration_gain_item_t g8;
    lr20xx_radio_common_rssi_calibration_gain_item_t g9;
    lr20xx_radio_common_rssi_calibration_gain_item_t g10;
    lr20xx_radio_common_rssi_calibration_gain_item_t g11;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost0;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost1;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost2;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost3;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost4;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost5;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost6;
    lr20xx_radio_common_rssi_calibration_gain_item_t g12_boost7;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost0;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost1;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost2;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost3;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost4;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost5;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost6;
    lr20xx_radio_common_rssi_calibration_gain_item_t g13_boost7;
} lr20xx_radio_common_rssi_calibration_gain_table_t;

/*!
 * @brief Gain step
 */
typedef enum lr20xx_radio_common_gain_step_e
{
    LR20XX_RADIO_COMMON_GAIN_STEP_AUTO = 0x00,  //!< Enable Automatic gain control (AGC)
    LR20XX_RADIO_COMMON_GAIN_STEP_G1   = 0x01,  //!< Gain set to G1
    LR20XX_RADIO_COMMON_GAIN_STEP_G2   = 0x02,  //!< Gain set to G2
    LR20XX_RADIO_COMMON_GAIN_STEP_G3   = 0x03,  //!< Gain set to G3
    LR20XX_RADIO_COMMON_GAIN_STEP_G4   = 0x04,  //!< Gain set to G4
    LR20XX_RADIO_COMMON_GAIN_STEP_G5   = 0x05,  //!< Gain set to G5
    LR20XX_RADIO_COMMON_GAIN_STEP_G6   = 0x06,  //!< Gain set to G6
    LR20XX_RADIO_COMMON_GAIN_STEP_G7   = 0x07,  //!< Gain set to G7
    LR20XX_RADIO_COMMON_GAIN_STEP_G8   = 0x08,  //!< Gain set to G8
    LR20XX_RADIO_COMMON_GAIN_STEP_G9   = 0x09,  //!< Gain set to G9
    LR20XX_RADIO_COMMON_GAIN_STEP_G10  = 0x0A,  //!< Gain set to G10
    LR20XX_RADIO_COMMON_GAIN_STEP_G11  = 0x0B,  //!< Gain set to G11
    LR20XX_RADIO_COMMON_GAIN_STEP_G12  = 0x0C,  //!< Gain set to G12
    LR20XX_RADIO_COMMON_GAIN_STEP_G13  = 0x0D,  //!< Gain set to G13
} lr20xx_radio_common_gain_step_t;

/**
 * @brief Exit mode of LoRa Channel Activity Detection (CAD) operation
 */
typedef enum
{
    LR20XX_RADIO_COMMON_CAD_EXIT_MODE_FALLBACK = 0x00,  //!< The chip goes to the configured fallback mode after CAD
                                                        //!< operation, no matter what the CAD result is
    LR20XX_RADIO_COMMON_CAD_EXIT_MODE_TX =
        0x01,  //!< If the CAD operation does not detect an activity, the chip enters in TX mode
    LR20XX_RADIO_COMMON_CAD_EXIT_MODE_RX =
        0x02,  //!< If the CAD operation detects an activity, the chip enters in RX mode
} lr20xx_radio_common_cad_exit_mode_t;

/**
 * @brief Condition that triggers automatic Tx (or Rx) after Rx (or Tx) operation
 *
 */
typedef enum
{
    LR20XX_RADIO_COMMON_AUTO_TX_RX_OFF    = 0x00,  //!< Disable Auto Tx (or Rx) after Rx (or Tx) operation
    LR20XX_RADIO_COMMON_AUTO_TX_RX_ALWAYS = 0x01,  //!< Always trigger Tx (or Rx) operation after Rx (or Tx) operation
    LR20XX_RADIO_COMMON_AUTO_TX_RX_RX_DONE_ONLY =
        0x02,  //!< Trigger Tx operation only if Rx operation terminates with CRC Ok. Similar to @ref
               //!< LR20XX_RADIO_COMMON_AUTO_TX_RX_ALWAYS when used for automatic Rx after Tx situation
} lr20xx_radio_common_auto_tx_rx_conditions_t;

/**
 * @brief Configuration of automatic Tx/Rx feature
 */
typedef struct
{
    lr20xx_radio_common_auto_tx_rx_conditions_t
         condition;           //!< The condition for executing the automatic Tx (or Rx) operation after Rx (or Tx)
    bool disable_on_failure;  //!< Set to @p true to disable the automatic Tx (or Rx) on Timeout, or on invalid packet
                              //!< received with @p condition sets to @ref LR20XX_RADIO_COMMON_AUTO_TX_RX_RX_DONE_ONLY
    uint32_t delay_in_tick;  //!< The delay between the Rx (or Tx) termination, and the trig of the automatic Tx (or Rx)
                             //!< operation. Expressed in ticks of 32MHz clock.
    uint32_t tx_rx_timeout_in_rtc_step;  //!< The timeout to apply on the automatic Tx (or Rx) operation. Expressed
                                         //!< in 32.768kHz RTC step. Possible values are the same as documented for
                                         //!< lr20xx_radio_common_set_tx_with_timeout_in_rtc_step or
                                         //!< lr20xx_radio_common_set_rx_with_timeout_in_rtc_step
} lr20xx_radio_common_auto_tx_rx_configuration_t;

/*!
 * @brief Configuration of Power Amplifier
 */
typedef struct lr20xx_radio_common_pa_cfg_s
{
    lr20xx_radio_common_pa_selection_t pa_sel;    //!< Power Amplifier selection
    lr20xx_radio_common_pa_lf_mode_t pa_lf_mode;  //!< Power Amplifier Low-Frequency mode. If pa_sel is unused set to @p
                                                  //!< LR20XX_RADIO_COMMON_PA_LF_MODE_FSM
    uint8_t pa_lf_duty_cycle;  //!< Power Amplifier - Low-frequency duty cycle. If pa_sel is unused set to 6
    uint8_t pa_lf_slices;      //!< Power Amplifier - Low-frequency number of slices. If pa_sel is unused set to 7
    uint8_t pa_hf_duty_cycle;  //!< Power Amplifier - High-frequency duty cycle. If pa_sel is unused set to 16
} lr20xx_radio_common_pa_cfg_t;

/*!
 * @brief Clear Channel Assessment results
 */
typedef struct lr20xx_radio_common_cca_res_s
{
    int16_t min;  //!< CCA min - in dBm
    int16_t max;  //!< CCA max - in dBm
    int16_t avg;  //!< CCA average - in dBm
} lr20xx_radio_common_cca_res_t;

/*!
 * @brief CAD parameters
 */
typedef struct lr20xx_radio_common_cad_params_s
{
    uint32_t                            timeout;        //!< Timeout in 32MHz step
    uint8_t                             threshold;      //!< RSSI threshold in -dBm
    lr20xx_radio_common_cad_exit_mode_t exit_mode;      //!< Exit mode
    uint32_t                            tx_rx_timeout;  //!< Tx / Rx timeout in 32kHz step
} lr20xx_radio_common_cad_params_t;

/**
 * @brief Link Quality Indicator (LQI) value
 *
 * The LQI value is a positive value that is defined as the margin between detection sensitivity threshold and detection
 * peak that trigger the detection of a signal.
 *
 * Here it is given as an integer part (in dB), and a decimal part (in 1/4dB). The actual decimal value can then be
 * obtained with: \f$LQI_{dB} = lqi\_db + 0.25 \times lqi\_quarter\_db\_counter \f$.
 */
typedef struct lr20xx_radio_common_lqi_value_s
{
    uint8_t lqi_db;                  //!< The integer part of LQI (in dB)
    uint8_t lqi_quarter_db_counter;  //!< The decimal part of LQI (in 1/4dB)
} lr20xx_radio_common_lqi_value_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_COMMON_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
