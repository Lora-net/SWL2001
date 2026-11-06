/*!
 * @file      lr20xx_system_types.h
 *
 * @brief     System driver types for LR20XX
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

#ifndef LR20XX_SYSTEM_TYPES_H
#define LR20XX_SYSTEM_TYPES_H

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

/**
 * @brief Command status
 */
typedef enum lr20xx_system_command_status_e
{
    LR20XX_SYSTEM_CMD_STATUS_FAIL = 0x00,  //!< Last command could not be executed
    LR20XX_SYSTEM_CMD_STATUS_PERR = 0x01,  //!< Last command could not be processed (wrong opcode, wrong argument)
    LR20XX_SYSTEM_CMD_STATUS_OK   = 0x02,  //!< Last command processed successfullly
    LR20XX_SYSTEM_CMD_STATUS_DATA = 0x03,  //!< Last command was a successfully processed read, and data is currently
                                           //!< transmitted instead of IRQ status
} lr20xx_system_command_status_t;

/**
 * @brief Reset status
 */
typedef enum lr20xx_system_reset_status_e
{
    LR20XX_SYSTEM_RESET_STATUS_CLEARED          = 0x00,  //!< Reset status has been cleared
    LR20XX_SYSTEM_RESET_STATUS_PWR_ON_BROWN_OUT = 0x01,  //!< Reset triggered by power-on or brown-out
    LR20XX_SYSTEM_RESET_STATUS_RESET_PIN        = 0x02,  //!< Reset triggered by reset pin
    LR20XX_SYSTEM_RESET_STATUS_WATCHDOG         = 0x04,  //!< Reset triggered by internal watchdog
    LR20XX_SYSTEM_RESET_STATUS_NSS_WAKEUP       = 0x05,  //!< Reset triggered by leaving sleep mode with NSS toggling
    LR20XX_SYSTEM_RESET_STATUS_RTC_WAKEUP       = 0x06,  //!< Reset triggered by leaving sleep mode with RTC timeout
} lr20xx_system_reset_status_t;

/**
 * @brief Chip modes
 */
typedef enum
{
    LR20XX_SYSTEM_CHIP_MODE_SLEEP     = 0x00,
    LR20XX_SYSTEM_CHIP_MODE_STBY_RC   = 0x01,
    LR20XX_SYSTEM_CHIP_MODE_STBY_XOSC = 0x02,
    LR20XX_SYSTEM_CHIP_MODE_FS        = 0x03,
    LR20XX_SYSTEM_CHIP_MODE_RX        = 0x04,
    LR20XX_SYSTEM_CHIP_MODE_TX        = 0x05,
} lr20xx_system_chip_modes_t;

/**
 * @brief Status register 1 structure definition
 */
typedef struct lr20xx_system_stat1_s
{
    lr20xx_system_command_status_t command_status;       //!< Command status
    bool                           is_interrupt_active;  //!< Indication of a still active interrupt
} lr20xx_system_stat1_t;

/**
 * @brief Status register 2 structure definition
 */
typedef struct lr20xx_system_stat2_s
{
    lr20xx_system_reset_status_t reset_status;  //!< Source of the last reset
    lr20xx_system_chip_modes_t   chip_mode;     //!< Current chip mode
} lr20xx_system_stat2_t;

/**
 * @brief Version structure definition
 */
typedef struct lr20xx_system_version_s
{
    uint8_t major;  //!< Version major field
    uint8_t minor;  //!< Version minor field
} lr20xx_system_version_t;

/**
 * @brief Version structure definition
 */
typedef struct lr20xx_system_sleep_cfg_s
{
    bool is_clk_32k_enabled;        //!< 32kHz clock state when entering sleep mode
    bool is_ram_retention_enabled;  //!< RAM retention state when entering sleep mode
} lr20xx_system_sleep_cfg_t;

/**
 * @brief Error flags
 */
enum lr20xx_system_errors_e
{
    LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK = ( 1 << 0 ),  //!< HF XOSC did not start correctly
    LR20XX_SYSTEM_ERRORS_LF_XOSC_START_MASK = ( 1 << 1 ),  //!< LF XOSC did not start correctly
    LR20XX_SYSTEM_ERRORS_PLL_LOCK_MASK      = ( 1 << 2 ),  //!< PLL did not lock
    LR20XX_SYSTEM_ERRORS_LF_RC_CALIB_MASK   = ( 1 << 3 ),  //!< Calibration of the LF RC clock failed
    LR20XX_SYSTEM_ERRORS_HF_RC_CALIB_MASK   = ( 1 << 4 ),  //!< Calibration of the HF RC clock failed
    LR20XX_SYSTEM_ERRORS_PLL_CALIB_MASK     = ( 1 << 5 ),  //!< Calibration of the min / max RF frequencies failed
    LR20XX_SYSTEM_ERRORS_AAF_CALIB_MASK     = ( 1 << 6 ),  //!< Calibration of the anti-aliasing filter (AAF) failed
    LR20XX_SYSTEM_ERRORS_IMG_CALIB_MASK     = ( 1 << 7 ),  //!< Calibration of the image rejection failed
    LR20XX_SYSTEM_ERRORS_CHIP_BUSY_MASK =
        ( 1 << 8 ),  //!< DIO Tx or Rx trigger cannot be executed because chip was changing mode
    LR20XX_SYSTEM_ERRORS_RXFREQ_NO_FRONT_END_CALIB_MASK =
        ( 1 << 9 ),  //!< Front end calibration not available (image rejection, Poly-Phase filter, ADC offset) for the
                     //!< configured RF frequency
    LR20XX_SYSTEM_ERRORS_MEAS_UNIT_ADC_CALIB_MASK = ( 1 << 10 ),  //!< Error during calibration of the measure unit ADC
    LR20XX_SYSTEM_ERRORS_PA_OFFSET_CALIB_MASK =
        ( 1 << 11 ),                                    //!< Error during calibration of the Power Amplifier offset
    LR20XX_SYSTEM_ERRORS_PPF_CALIB_MASK = ( 1 << 12 ),  //!< Error during calibration of the Poly-Phase Filter (PPF)
    LR20XX_SYSTEM_ERRORS_SRC_CALIB_MASK =
        ( 1 << 13 ),  //!< Error during calibration of the Self Reception Cancellation (SRC)
    LR20XX_SYSTEM_ERRORS_SRC_SATURATION_CALIB_MASK =
        ( 1 << 14 ),  //!< RSSI saturation detected during SRC calibration. It may comes from an interferer.
    LR20XX_SYSTEM_ERRORS_SRC_TOLERANCE_CALIB_MASK = ( 1 << 15 ),  //!< SRC calibration values are out of tolerance
};

/**
 * @brief Interrupt flags
 */
enum lr20xx_system_irq_e
{
    LR20XX_SYSTEM_IRQ_NONE    = ( 0 << 0 ),           //!< No interrupt
    LR20XX_SYSTEM_IRQ_FIFO_RX = ( int ) ( 1u << 0 ),  //!< RX FIFO threshold reached
    LR20XX_SYSTEM_IRQ_FIFO_TX = ( int ) ( 1u << 1 ),  //!< TX FIFO threshold reached
    LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_VALID =
        ( int ) ( 1u << 2 ),                                    //!< Responder received a valid RTToF request
    LR20XX_SYSTEM_IRQ_TX_TIMESTAMP      = ( int ) ( 1u << 3 ),  //!< Last bit sent timestamp
    LR20XX_SYSTEM_IRQ_RX_TIMESTAMP      = ( int ) ( 1u << 4 ),  //!< Last bit received timestamp
    LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED = ( int ) ( 1u << 5 ),  //!< Preamble detected
    LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID =
        ( int ) ( 1u << 6 ),                               //!< Valid LoRa header received / Sync word received
    LR20XX_SYSTEM_IRQ_CAD_DETECTED = ( int ) ( 1u << 7 ),  //!< Activity detected during CAD operation
    LR20XX_SYSTEM_IRQ_LORA_RX_HEADER_TIMESTAMP =
        ( int ) ( 1u << 8 ),  //!< Last bit of LoRa header received in explicit mode,
                              //!< asserted after 8 symbols of payload in implicit mode
    LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR = ( int ) ( 1u << 9 ),   //!< Erroneous LoRa header received
    LR20XX_SYSTEM_IRQ_POWER_SUPPLY_LOW  = ( int ) ( 1u << 10 ),  //!< Power supply level dropped below the threshold
    LR20XX_SYSTEM_IRQ_PA_OVP_OCP = ( int ) ( 1u << 11 ),  //!< Power amplifier over-current protection has triggered
    LR20XX_SYSTEM_IRQ_ERROR =
        ( int ) ( 1u << 16 ),  //!< Error other than a command error occurred - call lr20xx_system_get_errors
    LR20XX_SYSTEM_IRQ_CMD_ERROR  = ( int ) ( 1u << 17 ),  //!< Host command fail/error occurred
    LR20XX_SYSTEM_IRQ_RX_DONE    = ( int ) ( 1u << 18 ),  //!< Packet received
    LR20XX_SYSTEM_IRQ_TX_DONE    = ( int ) ( 1u << 19 ),  //!< Packet sent
    LR20XX_SYSTEM_IRQ_CAD_DONE   = ( int ) ( 1u << 20 ),  //!< CAD operation done
    LR20XX_SYSTEM_IRQ_TIMEOUT    = ( int ) ( 1u << 21 ),  //!< Timeout occurred during Rx or Tx operation
    LR20XX_SYSTEM_IRQ_CRC_ERROR  = ( int ) ( 1u << 22 ),  //!< Packet received with wrong CRC
    LR20XX_SYSTEM_IRQ_LEN_ERROR  = ( int ) ( 1u << 23 ),  //!< Length of the received packet higher than expected
    LR20XX_SYSTEM_IRQ_ADDR_ERROR = ( int ) ( 1u << 24 ),  //!< Received packet discarded - no address match
    LR20XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP = ( int ) ( 1u << 25 ),  //!< LR-FHSS intra-packet hopping occurred
    LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE =
        ( int ) ( 1u << 26 ),  //!< A new LR-FHSS frequency table can be loaded
    LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD   = ( int ) ( 1u << 27 ),  //!< A new LR-FHSS payload can be loaded
    LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_RESPONSE_DONE = ( int ) ( 1u << 28 ),  //!< Responder sent an RTToF response
    LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_DISCARDED =
        ( int ) ( 1u << 29 ),  //!< Responder discarded the RTToF request - no address match
    LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_EXCHANGE_VALID =
        ( int ) ( 1u << 30 ),  //!< Initiator received a valid RTToF response from a responder
    LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_TIMEOUT =
        ( int ) ( 1u << 31 ),  //!< Initiator did not receive an RTToF response from a responder
    LR20XX_SYSTEM_IRQ_ALL_MASK =
        LR20XX_SYSTEM_IRQ_FIFO_RX | LR20XX_SYSTEM_IRQ_FIFO_TX | LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_VALID |
        LR20XX_SYSTEM_IRQ_TX_TIMESTAMP | LR20XX_SYSTEM_IRQ_RX_TIMESTAMP | LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED |
        LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID | LR20XX_SYSTEM_IRQ_CAD_DETECTED |
        LR20XX_SYSTEM_IRQ_LORA_RX_HEADER_TIMESTAMP | LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR |
        LR20XX_SYSTEM_IRQ_POWER_SUPPLY_LOW | LR20XX_SYSTEM_IRQ_ERROR | LR20XX_SYSTEM_IRQ_CMD_ERROR |
        LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_CAD_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT |
        LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR | LR20XX_SYSTEM_IRQ_ADDR_ERROR |
        LR20XX_SYSTEM_IRQ_LR_FHSS_INTRA_PKT_HOP | LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_FREQ_TABLE |
        LR20XX_SYSTEM_IRQ_LR_FHSS_RDY_FOR_NEW_PAYLOAD | LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_RESPONSE_DONE |
        LR20XX_SYSTEM_IRQ_RTTOF_RESPONDER_REQUEST_DISCARDED | LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_EXCHANGE_VALID |
        LR20XX_SYSTEM_IRQ_RTTOF_INITIATOR_TIMEOUT,
};

/**
 * @brief Interrupt type re-definition
 *
 * @see lr20xx_system_irq_e
 */
typedef uint32_t lr20xx_system_irq_mask_t;

/**
 * @brief Calibration mask type re-definition
 *
 * The values are from @ref lr20xx_system_calibration_e
 */
typedef uint8_t lr20xx_system_calibration_mask_t;

/**
 * @brief Error type re-definition
 *
 * @see lr20xx_system_errors_e
 */
typedef uint16_t lr20xx_system_errors_t;

/**
 * @brief Configurable DIO pins
 */
typedef enum lr20xx_system_dio_e
{
    LR20XX_SYSTEM_DIO_5  = 0x05,  //!< DIO5
    LR20XX_SYSTEM_DIO_6  = 0x06,  //!< DIO6
    LR20XX_SYSTEM_DIO_7  = 0x07,  //!< DIO7
    LR20XX_SYSTEM_DIO_8  = 0x08,  //!< DIO8
    LR20XX_SYSTEM_DIO_9  = 0x09,  //!< DIO9
    LR20XX_SYSTEM_DIO_10 = 0x0A,  //!< DIO10
    LR20XX_SYSTEM_DIO_11 = 0x0B,  //!< DIO11
} lr20xx_system_dio_t;

/**
 * @brief DIO pin functions
 */
typedef enum lr20xx_system_dio_func_e
{
    LR20XX_SYSTEM_DIO_FUNC_NONE       = 0x00,  //!< No DIO function
    LR20XX_SYSTEM_DIO_FUNC_IRQ        = 0x01,  //!< Interrupt request function
    LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH  = 0x02,  //!< RF switch control function
    LR20XX_SYSTEM_DIO_FUNC_GPIO_LOW   = 0x05,  //!< Set DIO state to low
    LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH  = 0x06,  //!< Set DIO state to high
    LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT = 0x07,  //!< High frequency clock output function
    LR20XX_SYSTEM_DIO_FUNC_LF_CLK_OUT = 0x08,  //!< Low frequency clock output function
    LR20XX_SYSTEM_DIO_FUNC_TX_TRIGGER = 0x09,  //!< TX trigger function
    LR20XX_SYSTEM_DIO_FUNC_RX_TRIGGER = 0x0A,  //!< RX trigger function
} lr20xx_system_dio_func_t;

/**
 * @brief DIO pin drive modes
 */
typedef enum lr20xx_system_dio_drive_e
{
    LR20XX_SYSTEM_DIO_DRIVE_NONE      = 0x00,  //!< No pull
    LR20XX_SYSTEM_DIO_DRIVE_PULL_DOWN = 0x01,  //!< Pull down
    LR20XX_SYSTEM_DIO_DRIVE_PULL_UP   = 0x02,  //!< Pull up
    LR20XX_SYSTEM_DIO_DRIVE_AUTO =
        0x03,  //!< If the DIO value in standby mode was ‘1’, it will be pulled-up, if it was ‘0’ it will be pulled-down
} lr20xx_system_dio_drive_t;

/**
 * @brief DIO RF switch states
 */
typedef enum lr20xx_system_dio_rf_switch_cfg_state_e
{
    LR20XX_SYSTEM_DIO_RF_SWITCH_CFG_STATE_LOW  = 0x00,
    LR20XX_SYSTEM_DIO_RF_SWITCH_CFG_STATE_HIGH = 0x01,
} lr20xx_system_dio_rf_switch_cfg_state_t;

/**
 * @brief RF switch configuration bitmask definition
 */
enum lr20xx_system_dio_rf_switch_cfg_e
{
    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_STANDBY = ( 1 << 0 ),
    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF   = ( 1 << 1 ),
    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF   = ( 1 << 2 ),
    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF   = ( 1 << 3 ),
    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF   = ( 1 << 4 ),
    LR20XX_SYSTEM_DIO_RF_SWITCH_ALL_MASK =
        LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_STANDBY | LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF |
        LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF | LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF |
        LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF,
};

/**
 * @brief RF switch configuration type
 *
 * @see lr20xx_system_dio_rf_switch_cfg_e
 */
typedef uint8_t lr20xx_system_dio_rf_switch_cfg_t;

/**
 * @brief Low-frequency clock modes
 */
typedef enum lr20xx_system_lfclk_cfg_e
{
    LR20XX_SYSTEM_LFCLK_RC   = 0x00,  //!< Use internal RC 32kHz (Default)
    LR20XX_SYSTEM_LFCLK_XTAL = 0x01,  //!< Use XTAL 32kHz
    LR20XX_SYSTEM_LFCLK_EXT  = 0x02,  //!< Use externally provided 32kHz signal on DIO11
} lr20xx_system_lfclk_cfg_t;

/**
 * @brief TCXO supply voltage values
 */
typedef enum
{
    LR20XX_SYSTEM_TCXO_CTRL_1_6V = 0x00,  //!< Supply voltage = 1.6v
    LR20XX_SYSTEM_TCXO_CTRL_1_7V = 0x01,  //!< Supply voltage = 1.7v
    LR20XX_SYSTEM_TCXO_CTRL_1_8V = 0x02,  //!< Supply voltage = 1.8v
    LR20XX_SYSTEM_TCXO_CTRL_2_2V = 0x03,  //!< Supply voltage = 2.2v
    LR20XX_SYSTEM_TCXO_CTRL_2_4V = 0x04,  //!< Supply voltage = 2.4v
    LR20XX_SYSTEM_TCXO_CTRL_2_7V = 0x05,  //!< Supply voltage = 2.7v
    LR20XX_SYSTEM_TCXO_CTRL_3_0V = 0x06,  //!< Supply voltage = 3.0v
    LR20XX_SYSTEM_TCXO_CTRL_3_3V = 0x07,  //!< Supply voltage = 3.3v
} lr20xx_system_tcxo_supply_voltage_t;

/**
 * @brief Regulator modes
 */
typedef enum lr20xx_system_reg_mode_e
{
    LR20XX_SYSTEM_REG_MODE_LDO  = 0x00,  //!< (Default) Only use the Low-Dropout Regulator
    LR20XX_SYSTEM_REG_MODE_DCDC = 0x02,  //!< Switch on the DC-to-DC regulator in applicable chip modes
} lr20xx_system_reg_mode_t;

/**
 * @brief Calibration flags
 */
enum lr20xx_system_calibration_e
{
    LR20XX_SYSTEM_CALIB_LF_RC_MASK  = ( 1 << 0 ),  //!< Low Frequency clock RC
    LR20XX_SYSTEM_CALIB_HF_RC_MASK  = ( 1 << 1 ),  //!< High Frequency clock RC
    LR20XX_SYSTEM_CALIB_PLL_MASK    = ( 1 << 2 ),  //!< PLL
    LR20XX_SYSTEM_CALIB_AAF_MASK    = ( 1 << 3 ),  //!< Anti aliasing filter bandwidth
    LR20XX_SYSTEM_CALIB_MU_MASK     = ( 1 << 5 ),  //!< Measure unit ADC gain
    LR20XX_SYSTEM_CALIB_PA_OFF_MASK = ( 1 << 6 ),  //!< Power amplifier offset
};

/**
 * @brief Value formats
 */
typedef enum lr20xx_system_value_format_e
{
    LR20XX_SYSTEM_VALUE_FORMAT_RAW  = 0x00,
    LR20XX_SYSTEM_VALUE_FORMAT_UNIT = 0x01,
} lr20xx_system_value_format_t;

/**
 * @brief Measurement resolution
 */
typedef enum lr20xx_system_meas_res_e
{
    LR20XX_SYSTEM_MEAS_RES_8_BITS  = 0x00,
    LR20XX_SYSTEM_MEAS_RES_9_BITS  = 0x01,
    LR20XX_SYSTEM_MEAS_RES_10_BITS = 0x02,
    LR20XX_SYSTEM_MEAS_RES_11_BITS = 0x03,
    LR20XX_SYSTEM_MEAS_RES_12_BITS = 0x04,
    LR20XX_SYSTEM_MEAS_RES_13_BITS = 0x05,
} lr20xx_system_meas_res_t;

/**
 * @brief Source of the measured temperature
 */
typedef enum lr20xx_system_temp_src_e
{
    LR20XX_SYSTEM_TEMP_SRC_VBE  = 0x00,
    LR20XX_SYSTEM_TEMP_SRC_XOSC = 0x01,
    LR20XX_SYSTEM_TEMP_SRC_NTC  = 0x02,
} lr20xx_system_temp_src_t;

/**
 * @brief Select the entropy source to enable for random number generator
 *
 * It is advised to enable both PLL and ADC entropy sources.
 * By default PLL and ADC are used as entropy sources.
 */
typedef enum
{
    LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_PLL = 0x01,  //!< PLL is used as entropy source. The chip automatically goes to
                                                     //!< FS mode when needed, and goes back to original mode afterward.
    LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_ADC = 0x02,  //!< ADC is used as entropy source. The chip automatically goes to
                                                     //!< FS mode when needed, and goes back to original mode afterward.
} lr20xx_system_random_entropy_source_t;

/**
 * @brief Bit mask of entropy source to enable.
 *
 * The values are from @ref lr20xx_system_random_entropy_source_t.
 */
typedef uint8_t lr20xx_system_random_entropy_source_bitmask_t;

/**
 * @brief Stand-by modes
 */
typedef enum lr20xx_system_standby_mode_e
{
    LR20XX_SYSTEM_STANDBY_MODE_RC   = 0x00,
    LR20XX_SYSTEM_STANDBY_MODE_XOSC = 0x01,
} lr20xx_system_standby_mode_t;

/**
 * @brief High Frequency Clock scaling values
 */
typedef enum
{
    LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ   = 0x00,  //!< Division by 1 - 32 MHz
    LR20XX_SYSTEM_HF_CLK_SCALING_16_MHZ   = 0x01,  //!< Division by 2 - 16 MHz
    LR20XX_SYSTEM_HF_CLK_SCALING_8_MHZ    = 0x02,  //!< Division by 4 - 8 MHz
    LR20XX_SYSTEM_HF_CLK_SCALING_4_MHZ    = 0x03,  //!< Division by 8 - 4 MHz
    LR20XX_SYSTEM_HF_CLK_SCALING_2_MHZ    = 0x04,  //!< Division by 16 - 2 MHz
    LR20XX_SYSTEM_HF_CLK_SCALING_1_MHZ    = 0x05,  //!< Division by 32 - 1 MHz
    LR20XX_SYSTEM_HF_CLK_SCALING_500_KHZ  = 0x06,  //!< Division by 64 - 500 kHz
    LR20XX_SYSTEM_HF_CLK_SCALING_250_KHZ  = 0x07,  //!< Division by 128 - 250 kHz
    LR20XX_SYSTEM_HF_CLK_SCALING_125_KHZ  = 0x08,  //!< Division by 256 - 125 kHz
    LR20XX_SYSTEM_HF_CLK_SCALING_62500_HZ = 0x09,  //!< Division by 512 - 62500 Hz
    LR20XX_SYSTEM_HF_CLK_SCALING_31250_HZ = 0x0A,  //!< Division by 1024 - 31250 Hz
    LR20XX_SYSTEM_HF_CLK_SCALING_15625_HZ = 0x0B,  //!< Division by 2048 - 15625 Hz
    LR20XX_SYSTEM_HF_CLK_SCALING_7813_HZ  = 0x0C,  //!< Division by 4096 - 7812.5 Hz
    LR20XX_SYSTEM_HF_CLK_SCALING_3906_HZ  = 0x0D,  //!< Division by 8192 - 3906.25 Hz
    LR20XX_SYSTEM_HF_CLK_SCALING_1953_HZ  = 0x0E,  //!< Division by 16384 - 1953.125 Hz
    LR20XX_SYSTEM_HF_CLK_SCALING_977_HZ   = 0x0F,  //!< Division by 32768 - 976.5625 Hz
} lr20xx_system_hf_clk_scaling_t;

/**
 * @brief Low battery detector trimming used to configure threshold triggering a battery low interrupt
 */
typedef enum lr20xx_system_lbd_trim_e
{
    LR20XX_SYSTEM_LBD_TRIM_1_60_V = 0x00,  //!< EoL threshold set to 1.60V
    LR20XX_SYSTEM_LBD_TRIM_1_67_V = 0x01,  //!< EoL threshold set to 1.67V
    LR20XX_SYSTEM_LBD_TRIM_1_74_V = 0x02,  //!< EoL threshold set to 1.74V
    LR20XX_SYSTEM_LBD_TRIM_1_80_V = 0x03,  //!< EoL threshold set to 1.80V
    LR20XX_SYSTEM_LBD_TRIM_1_88_V = 0x04,  //!< EoL threshold set to 1.88V - default value
    LR20XX_SYSTEM_LBD_TRIM_1_95_V = 0x05,  //!< EoL threshold set to 1.98V
    LR20XX_SYSTEM_LBD_TRIM_2_00_V = 0x06,  //!< EoL threshold set to 2.00V
    LR20XX_SYSTEM_LBD_TRIM_2_10_V = 0x07,  //!< EoL threshold set to 2.10V
} lr20xx_system_lbd_trim_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_SYSTEM_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
