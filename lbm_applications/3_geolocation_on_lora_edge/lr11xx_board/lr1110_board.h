#ifndef LR1110_BOARD_H
#define LR1110_BOARD_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Return the mask for TX LED
 *
 * The mask is to be used for LED control functions like smtc_board_led_set or smtc_board_led_pulse
 *
 * @return TX LED mask
 */
uint32_t smtc_board_get_led_tx_mask( void );

/*!
 * @brief Return the mask for RX LED
 *
 * The mask is to be used for LED control functions like smtc_board_led_set or smtc_board_led_pulse
 *
 * @return RX LED mask
 */
uint32_t smtc_board_get_led_rx_mask( void );

/*!
 * @brief Return the mask for ALL LED
 *
 * The mask is to be used for LED control functions like smtc_board_led_set or smtc_board_led_pulse
 *
 * @return ALL LED mask
 */
uint32_t smtc_board_get_led_all_mask( void );

/*!
 * @brief Turn on/off the requested LED(s)
 *
 * @param [in] led_mask Mask representing the list of the LEDs to turn on/off
 * @param [in] turn_on If true, the requested LEDs are turned on, else they are turned off
 */
void smtc_board_led_set( uint32_t led_mask, bool turn_on );

/*!
 * @brief Turn on/off the requested LED(s) for a given duration
 *
 * @param [in] led_mask Mask representing the list of the LEDs to turn on/off
 * @param [in] turn_on If true, the requested LEDs are turned on, else they are turned off
 * @param [in] duration_ms Duration of the pulse, in milliseconds
 */
void smtc_board_led_pulse( uint32_t led_mask, bool turn_on, uint32_t duration_ms );

#endif