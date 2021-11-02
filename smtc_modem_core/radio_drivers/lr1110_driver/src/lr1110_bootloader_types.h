/*!
 * @file      lr1110_bootloader_types.h
 *
 * @brief     Bootloader driver types for LR1110
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

#ifndef LR1110_BOOTLOADER_TYPES_H
#define LR1110_BOOTLOADER_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Length in byte of the LR1110 version blob
 */
#define LR1110_BL_VERSION_LENGTH ( 4 )

/*!
 * @brief Length in bytes of a PIN
 */
#define LR1110_BL_PIN_LENGTH ( 4 )

/*!
 * @brief Length in bytes of a chip EUI
 */
#define LR1110_BL_CHIP_EUI_LENGTH ( 8 )

/*!
 * @brief Length in bytes of a join EUI
 */
#define LR1110_BL_JOIN_EUI_LENGTH ( 8 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Fixed-length array to store a PIN
 */
typedef uint8_t lr1110_bootloader_pin_t[LR1110_BL_PIN_LENGTH];

/*!
 * @brief Fixed-length array to store a chipEUI
 */
typedef uint8_t lr1110_bootloader_chip_eui_t[LR1110_BL_CHIP_EUI_LENGTH];

/*!
 * @brief Fixed-length array to store a joinEUI
 */
typedef uint8_t lr1110_bootloader_join_eui_t[LR1110_BL_JOIN_EUI_LENGTH];

/*!
 * @brief Chip modes
 */
typedef enum lr1110_bootloader_chip_modes_e
{
    LR1110_BOOTLOADER_CHIP_MODE_SLEEP     = 0x00,
    LR1110_BOOTLOADER_CHIP_MODE_STBY_RC   = 0x01,
    LR1110_BOOTLOADER_CHIP_MODE_STBY_XOSC = 0x02,
    LR1110_BOOTLOADER_CHIP_MODE_FS        = 0x03,
    LR1110_BOOTLOADER_CHIP_MODE_RX        = 0x04,
    LR1110_BOOTLOADER_CHIP_MODE_TX        = 0x05,
    LR1110_BOOTLOADER_CHIP_MODE_LOC       = 0x06,
} lr1110_bootloader_chip_modes_t;

/*!
 * @brief Reset status
 */
typedef enum lr1110_bootloader_reset_status_e
{
    LR1110_BOOTLOADER_RESET_STATUS_CLEARED      = 0x00,
    LR1110_BOOTLOADER_RESET_STATUS_ANALOG       = 0x01,
    LR1110_BOOTLOADER_RESET_STATUS_EXTERNAL     = 0x02,
    LR1110_BOOTLOADER_RESET_STATUS_SYSTEM       = 0x03,
    LR1110_BOOTLOADER_RESET_STATUS_WATCHDOG     = 0x04,
    LR1110_BOOTLOADER_RESET_STATUS_IOCD_RESTART = 0x05,
    LR1110_BOOTLOADER_RESET_STATUS_RTC_RESTART  = 0x06,
} lr1110_bootloader_reset_status_t;

/*!
 * @brief Command status
 */
typedef enum lr1110_bootloader_command_status_e
{
    LR1110_BOOTLOADER_CMD_STATUS_FAIL = 0x00,
    LR1110_BOOTLOADER_CMD_STATUS_PERR = 0x01,
    LR1110_BOOTLOADER_CMD_STATUS_OK   = 0x02,
    LR1110_BOOTLOADER_CMD_STATUS_DATA = 0x03,
} lr1110_bootloader_command_status_t;

/*!
 * @brief Status register 1 structure definition
 */
typedef struct lr1110_bootloader_stat1_s
{
    lr1110_bootloader_command_status_t command_status;
    bool                               is_interrupt_active;
} lr1110_bootloader_stat1_t;

/*!
 * @brief Status register 2 structure definition
 */
typedef struct lr1110_bootloader_stat2_s
{
    lr1110_bootloader_reset_status_t reset_status;
    lr1110_bootloader_chip_modes_t   chip_mode;
    bool                             is_running_from_flash;
} lr1110_bootloader_stat2_t;

/*!
 * @brief Bootloader version structure definition
 */
typedef struct lr1110_bootloader_version_s
{
    uint8_t  hw;
    uint8_t  type;
    uint16_t fw;
} lr1110_bootloader_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR1110_BOOTLOADER_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
