/*!
 * @file      lr20xx_system.h
 *
 * @brief     System driver definition for LR20XX
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

#ifndef LR20XX_SYSTEM_H
#define LR20XX_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_system_types.h"
#include "lr20xx_status.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Reset the radio
 *
 * @param [in]  context Chip implementation context.
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_reset( const void* context );

/*!
 * @brief Wake the radio up from sleep mode.
 *
 * @param [in]  context Chip implementation context.
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_wakeup( const void* context );

/*!
 * @brief Return stat1, stat2, and irq_status
 *
 * @param [in] context Chip implementation context
 * @param [out] stat1      Pointer to a variable for holding stat1. Can be NULL.
 * @param [out] stat2      Pointer to a variable for holding stat2. Can be NULL.
 * @param [out] irq_status Pointer to a variable for holding irq_status. Can be NULL.
 *
 * @returns Operation status
 *
 * @remark To simplify system integration, this function does not actually execute the GetStatus command, which would
 * require bidirectional SPI communication. It obtains the stat1, stat2, and irq_status values by performing an ordinary
 * SPI read (which is required to send null/NOP bytes on the MOSI line). This is possible since the LR20XX returns these
 * values automatically whenever a read that does not directly follow a response-carrying command is performed.
 * Unlike with the GetStatus command, however, the reset status information is NOT cleared by this command. The function
 * @ref lr20xx_system_clear_reset_status_info may be used for this purpose when necessary.
 */
lr20xx_status_t lr20xx_system_get_status( const void* context, lr20xx_system_stat1_t* stat1,
                                          lr20xx_system_stat2_t* stat2, lr20xx_system_irq_mask_t* irq_status );

/*!
 * @brief Clear the reset status information stored in stat2
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_clear_reset_status_info( const void* context );

/*!
 * @brief Return the version of the system
 *
 * @param [in] context Chip implementation context
 * @param [out] version Pointer to the structure holding the system version
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_get_version( const void* context, lr20xx_system_version_t* version );

/*!
 * @brief Return the system errors
 *
 * Errors may be fixed following:
 * - calibration error can be fixed by attempting another RC calibration;
 * - XOsc related errors may be due to hardware problems, can be fixed by reset;
 * - PLL lock related errors can be due to not-locked PLL, or by attempting to use an out-of-band frequency, can be
 * fixed by executing a PLL calibration, or by using other frequencies.
 *
 * @param [in] context Chip implementation context
 * @param [out] errors Pointer to a value holding error flags
 *
 * @returns Operation status
 *
 * @see lr20xx_system_calibrate, lr20xx_radio_common_calibrate_front_end, lr20xx_system_clear_errors
 */
lr20xx_status_t lr20xx_system_get_errors( const void* context, lr20xx_system_errors_t* errors );

/*!
 * @brief Clear all error flags pending.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @see lr20xx_system_get_errors
 */
lr20xx_status_t lr20xx_system_clear_errors( const void* context );

/**
 * @brief Returns the number of available DIOs
 *
 * @remark Also is the valid range for lr20xx_system_dio_get_nth.
 *
 * @return uint8_t the number of valid DIOs.
 *
 * @see lr20xx_system_dio_get_nth
 */
uint8_t lr20xx_system_dio_get_count( void );

/**
 * @brief Returns the nth value from the enum lr20xx_system_dio_t
 *
 * @param [in] nth from 0 to lr20xx_system_dio_get_count() - 1
 * @param [out] dio Pointer to a value holding the corresponding DIO enum
 * @return true if nth is a valid number, false otherwise
 *
 * @see lr20xx_system_dio_get_count, lr20xx_system_dio_t
 */
bool lr20xx_system_dio_get_nth( uint8_t nth, lr20xx_system_dio_t* dio );

/*!
 * @brief Configure the function and the drive mode of a given DIO
 *
 * @remark @p drive is applied when entering sleep mode if @p func is not @ref LR20XX_SYSTEM_DIO_FUNC_NONE.
 * When leaving sleep mode without retention, @p drive is reset to @ref LR20XX_SYSTEM_DIO_DRIVE_NONE (except for @p dio
 * LR20XX_SYSTEM_DIO_5 and LR20XX_SYSTEM_DIO_6 where @p drive is reset to LR20XX_SYSTEM_DIO_DRIVE_PULL_UP).
 *
 * @remark The state of @p dio is reevaluated when sending this command
 *
 * @remark When @p func is set to either LR20XX_SYSTEM_DIO_FUNC_TX_TRIGGER or LR20XX_SYSTEM_DIO_FUNC_RX_TRIGGER, default
 * timeout set through @ref lr20xx_radio_common_set_default_rx_tx_timeout or @ref
 * lr20xx_radio_common_set_default_rx_tx_timeout_in_rtc_step is used when radio operation is triggered
 *
 * @remark On @ref LR20XX_SYSTEM_DIO_5, only @ref LR20XX_SYSTEM_DIO_DRIVE_PULL_UP for @p drive
 *
 * @remark Function @ref LR20XX_SYSTEM_DIO_FUNC_LF_CLK_OUT can only be used if @p dio is one of:
 *   - LR20XX_SYSTEM_DIO_7
 *   - LR20XX_SYSTEM_DIO_8
 *   - LR20XX_SYSTEM_DIO_9
 *   - LR20XX_SYSTEM_DIO_10
 *   - LR20XX_SYSTEM_DIO_11
 *
 * @ref LR20XX_SYSTEM_DIO_5 and @ref LR20XX_SYSTEM_DIO_6 must be configured explicitly to function @ref
 * LR20XX_SYSTEM_DIO_FUNC_NONE if they are connected to an external component that could toggle their state between a
 * cold start or a start without retention and the configuration of a function.
 *
 * @param [in] context Chip implementation context
 * @param [in] dio DIO pin
 * @param [in] func DIO pin function
 * @param [in] drive DIO pin drive
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_dio_function( const void* context, lr20xx_system_dio_t dio,
                                                lr20xx_system_dio_func_t func, lr20xx_system_dio_drive_t drive );

/*!
 * @brief Set the RF switch configurations for a given DIO
 *
 * @remark The state of @p dio is reevaluated when sending this command
 *
 * @param [in] context Chip implementation context
 * @param [in] dio DIO pin
 * @param [in] rf_switch_cfg Pointer to a structure that holds the RF switch configuration for @p dio
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_dio_rf_switch_cfg( const void* context, lr20xx_system_dio_t dio,
                                                     const lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg );

/*!
 * @brief Set the interrupt configurations for a given DIO
 *
 * It is not possible to set the same IRQ on multiple DIOs. Only the last mapping for each IRQ is take into account.
 *
 * @remark The state of \p dio is reevaluated when sending this command
 *
 * @param [in] context Chip implementation context
 * @param [in] dio DIO pin
 * @param [in] irq_cfg Interrupt mask for \p dio
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_dio_irq_cfg( const void* context, lr20xx_system_dio_t dio,
                                               const lr20xx_system_irq_mask_t irq_cfg );

/*!
 * @brief Clear requested bits in the internal pending interrupt register
 *
 * @param [in] context Chip implementation context
 * @param [in] irqs_to_clear Variable that holds the interrupts to be cleared
 *
 * @returns Operation status
 *
 * @see lr20xx_system_get_and_clear_irq_status
 */
lr20xx_status_t lr20xx_system_clear_irq_status( const void* context, const lr20xx_system_irq_mask_t irqs_to_clear );

/**
 * @brief This helper function clears any radio irq status flags that are set and returns the flags that were cleared.
 *
 * @param [in] context Chip implementation context.
 * @param [out] irq Pointer to a variable for holding the system interrupt status.
 *
 * @returns Operation status
 *
 * @see lr20xx_system_clear_irq_status
 */
lr20xx_status_t lr20xx_system_get_and_clear_irq_status( const void* context, lr20xx_system_irq_mask_t* irq );

/*!
 * @brief Configure the source of the Low Frequency Clock (LF_CLK)
 *
 * When switching LF CLK to XTAL (@ref LR20XX_SYSTEM_LFCLK_XTAL), the XTAL is started and the chip waits for it to
 * start. If @ref LR20XX_SYSTEM_DIO_10 or @ref LR20XX_SYSTEM_DIO_11 is already configured for another function, this
 * command fails.
 *
 * When switching LF CLK to external source (@ref LR20XX_SYSTEM_LFCLK_EXT), the external clock source must be already
 * running, and shall keep running afterwards.
 *
 * It is not possible to switch to/from @ref LR20XX_SYSTEM_LFCLK_XTAL from/to @ref LR20XX_SYSTEM_LFCLK_EXT. The LF CLK
 * must be configured to LR20XX_SYSTEM_LFCLK_RC in between.
 *
 * @param [in] context Chip implementation context
 * @param [in] lfclock_cfg Low frequency clock configuration
 *
 * @returns Operation status
 *
 * @see lr20xx_system_calibrate, lr20xx_radio_common_calibrate_front_end, lr20xx_system_set_dio_function
 */
lr20xx_status_t lr20xx_system_cfg_lfclk( const void* context, const lr20xx_system_lfclk_cfg_t lfclock_cfg );

/*!
 * @brief Configure the High Frequency clock scaling on the output
 *
 * This command sets the HF clock scaling on the DIO configured with functionality
 * LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT through lr20xx_system_set_dio_function
 *
 * @param [in] context Chip implementation context
 * @param [in] hf_clk_scaling High frequency output scaling
 *
 * @returns Operation status
 *
 * @see lr20xx_system_set_dio_function
 */
lr20xx_status_t lr20xx_system_cfg_clk_output( const void* context, lr20xx_system_hf_clk_scaling_t hf_clk_scaling );

/*!
 * @brief Enable the usage of a TCXO as HF clock and configure supply voltage & start delay
 *
 * \p start_delay_in_32mhz_step is the time the firmware waits before going into RF mode, expressed in number of 32MHz
 * clock ticks.
 * The timeout duration is given by: \f$ start\_delay\_in\_ns = start\_delay\_in\_32mhz\_step \times 31.25 \f$
 *
 * The TCXO mode can be disabled by setting \p start_delay_in_32mhz_step to 0.
 *
 * In the situation where the TCXO has not started within \p start_delay_in_32mhz_step then the error bit
 * LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK will be set. It can be checked with a call to \p lr20xx_system_get_errors.
 *
 * It must be noted that the TCXO start time duration can last twice the duration of \p start_delay_in_32mhz_step
 * lr20xx_system_calibrate if the internal 32MHz RC clock source is not calibrated. Refer to \p lr20xx_system_calibrate
 * for details.
 *
 * The maximum value for \p start_delay_in_32mhz_step is 0xFFFFFFFF.
 *
 * @param [in] context Chip implementation context
 * @param [in] supply_voltage Supply voltage value
 * @param [in] start_delay_in_32mhz_step Gating time before which the radio starts its RF operation
 *
 * @returns Operation status
 *
 * @see lr20xx_system_calibrate, lr20xx_radio_common_calibrate_front_end, lr20xx_system_get_errors
 */
lr20xx_status_t lr20xx_system_set_tcxo_mode( const void*                               context,
                                             const lr20xx_system_tcxo_supply_voltage_t supply_voltage,
                                             const uint32_t                            start_delay_in_32mhz_step );

/*!
 * @brief Configure the regulator mode to be used in specific modes
 *
 * \p reg_mode defines if the DC-DC converter is switched on in the following modes: STANDBY XOSC, FS, RX, TX.
 *
 * @param [in] context Chip implementation context
 * @param [in] reg_mode Regulator mode configuration
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_reg_mode( const void* context, const lr20xx_system_reg_mode_t reg_mode );

/*!
 * @brief Calibrate the requested blocks
 *
 * This function can be called in any mode of the chip.
 *
 * The chip will return to standby RC mode on exit. Potential calibration issues can be read out with
 * lr20xx_system_get_errors command.
 *
 * @param [in] context Chip implementation context
 * @param [in] blocks_to_calibrate Blocks to be calibrated - bitfield built with lr20xx_system_calibration_e
 *
 * @returns Operation status
 *
 * @see lr20xx_system_get_errors
 */
lr20xx_status_t lr20xx_system_calibrate( const void*                            context,
                                         const lr20xx_system_calibration_mask_t blocks_to_calibrate );

/*!
 * @brief Get the value of the power supply voltage
 *
 * If @p format is set to LR20XX_SYSTEM_VALUE_FORMAT_RAW, Vbat value (in [V]) is a function of Vana (typ. 1.35V) and can
 * be obtained using the following formula: \f$ Vbat_{V} = (\frac{vbat}{8192} \times 5 - 1) \times Vana \f$ where vbat
 * is a 13-bit long value
 *
 * If @p format is set to LR20XX_SYSTEM_VALUE_FORMAT_UNIT, the power supply voltage is given in [mV]
 *
 * @param [in] context Chip implementation context
 * @param [in] format Format of the returned value of @p vbat
 * @param [in] res Resolution of the measure of @p vbat
 * @param [out] vbat A pointer to the @p vbat value
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_get_vbat( const void* context, lr20xx_system_value_format_t format,
                                        lr20xx_system_meas_res_t res, uint16_t* vbat );

/*!
 * @brief Get the value of the internal junction temperature
 *
 * If @p format is set to LR20XX_SYSTEM_VALUE_FORMAT_RAW, the temperature (in [°C]) is a function of Vana (typ. 1.35V),
 * Vbe25 (Vbe voltage @ 25°C, typ. 0.7295V) and VbeSlope (typ. -1.7mV/°C) using the following formula:
 * \f$ Temperature_{°C} = (\frac{temp(12:0)}{8192} \times Vana - Vbe25) \times \frac{1000}{VbeSlope} + 25 \f$ where
 * temp{12:0} is the value corresponding to the 12 LSBs of the output argument of @ref lr20xx_system_get_temp
 *
 * If @p format is set to LR20XX_SYSTEM_VALUE_FORMAT_UNIT, the temperature is given in [°C] in 13.5sb format, the first
 * byte returned contains the integer part, the second the fractional part.
 *
 * @remark If a TCXO is used, make sure to configure it with @ref lr20xx_system_set_tcxo_mode before calling this
 * function
 *
 * @param [in] context Chip implementation context
 * @param [in] format Format of the returned value of @p temp
 * @param [in] res Resolution of the measure
 * @param [in] src Temperature source
 * @param [out] temp A pointer to the @p temp value
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_get_temp( const void* context, lr20xx_system_value_format_t format,
                                        lr20xx_system_meas_res_t res, lr20xx_system_temp_src_t src, uint16_t* temp );

/*!
 * @brief Read and return a 32-bit random number
 *
 * This random number generator is not suitable for cryptographic operations.
 * It can be called during any mode without perturbation on ongoing Rx or Tx operation.
 *
 * @remark Radio operating mode must be set into standby.
 *
 * @param [in] context Chip implementation context
 * @param [in] source Select source of entropy for random number generator
 * @param [out] random_number 32-bit random number
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_get_random_number( const void*                                   context,
                                                 lr20xx_system_random_entropy_source_bitmask_t source,
                                                 uint32_t*                                     random_number );

/*!
 * @brief Switch the transceiver into sleep mode with the request configuration
 *
 * @param [in] context Chip implementation context
 * @param [in] sleep_cfg Sleep configuration
 * @param [in] sleep_time Sleep time in LF clock steps
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_sleep_mode( const void* context, const lr20xx_system_sleep_cfg_t* sleep_cfg,
                                              const uint32_t sleep_time );

/*!
 * @brief Switch the transceiver into the requested stand-by mode
 *
 * @param [in] context Chip implementation context
 * @param [in] standby_mode Requested stand-by mode
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_standby_mode( const void* context, const lr20xx_system_standby_mode_t standby_mode );

/*!
 * @brief Switch the transceiver into the Frequency Synthesis (FS) mode
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_fs_mode( const void* context );

/*!
 * @brief Add a register to be saved in retention memory
 *
 * @remark This command is used when a register is not added by default to the retention memory. It gives the
 * possibility to store up to 32 additional registers when entering sleep mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] slot Index in the storage list. Allowed values [0:31]
 * @param [in] address Address of the register to be added to the list. Only the 3 LSBs are significant. Address must be
 * word-aligned
 *
 * @returns Operation status
 *
 * @see lr20xx_system_set_sleep_mode
 */
lr20xx_status_t lr20xx_system_add_register_to_retention_mem( const void* context, uint8_t slot, uint32_t address );

/*!
 * @brief Configure the low battery detector
 *
 * @param [in] context Chip implementation context
 * @param [in] is_enabled Low battery detector activation
 * @param [in] trim Trimming value defining the threshold used to trigger a low battery interrupt
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_set_lbd_cfg( const void* context, bool is_enabled, lr20xx_system_lbd_trim_t trim );

/*!
 * @brief Configure the internal trimming capacitor values and XTAL start time
 *
 * @remark The device is fitted with internal programmable capacitors connected independently to the pins XTA and XTB of
 * the device. Each capacitor can be controlled independently in steps of 0.47 pF added to the minimal value of 11.3pF
 * for XTA and 10.1pF for XTB.
 *
 * The maximal capacitor value corresponds to 47 LSB steps added to the corresponding minimal value, so it is 33.39pF
 * for XTA and 32.19pF for XTB.
 *
 * @param [in] context Chip implementation context
 * @param [in] xta Value for the trimming capacitor connected to XTA pin
 * @param [in] xtb Value for the trimming capacitor connected to XTB pin
 * @param [in] wait_time_us Additional wait time after XTAL readiness in microsecond
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_system_configure_xosc( const void* context, uint8_t xta, uint8_t xtb, uint8_t wait_time_us );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_SYSTEM_H

/* --- EOF ------------------------------------------------------------------ */
