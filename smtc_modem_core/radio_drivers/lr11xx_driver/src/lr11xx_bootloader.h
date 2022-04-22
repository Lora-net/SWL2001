/*!
 * @file      lr11xx_bootloader.h
 *
 * @brief     Bootloader driver definition for LR11XX
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

#ifndef LR11XX_BOOTLOADER_H
#define LR11XX_BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_bootloader_types.h"
#include "lr11xx_types.h"

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

typedef uint32_t lr11xx_bootloader_irq_mask_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Return the status registers and interrupt flags
 *
 * @remark To simplify system integration, this function does not actually execute the GetStatus command, which would
 * require bidirectional SPI communication. It obtains the stat1, stat2, and irq_status values by performing an ordinary
 * SPI read (which is required to send null/NOP bytes on the MOSI line). This is possible since the LR11XX returns these
 * values automatically whenever a read that does not directly follow a response-carrying command is performed. Unlike
 * with the GetStatus command, however, the reset status information is NOT cleared by this command. The function @ref
 * lr11xx_bootloader_clear_reset_status_info may be used for this purpose when necessary.
 *
 * @param [in] context Chip implementation context
 * @param [out] stat1 Content of status register 1
 * @param [out] stat2 Content of status register 2
 * @param [out] irq_status Interrupt flags
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_get_status( const void* context, lr11xx_bootloader_stat1_t* stat1,
                                              lr11xx_bootloader_stat2_t*    stat2,
                                              lr11xx_bootloader_irq_mask_t* irq_status );

/*!
 * @brief Clear the reset status information stored in stat2
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_clear_reset_status_info( const void* context );

/*!
 * @brief Return the version of the system (hardware and software)
 *
 * @param [in] context Chip implementation context
 * @param [out] version Pointer to the structure holding the system version
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_get_version( const void* context, lr11xx_bootloader_version_t* version );

/*!
 * @brief Erase the whole flash memory of the chip
 *
 * This function shall be called before any attempt to write a new firmware in flash memory
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_erase_flash( const void* context );

/*!
 * @brief Write encrypted data in program flash memory of the chip
 *
 * This function shall be used when updating the encrypted flash content of the LR11XX.
 * The encrypted flash payload to transfer shall be represented as an array of words (i.e. 4-byte values).
 *
 * @param [in] context Chip implementation context
 * @param [in] offset The offset from start register of flash
 * @param [in] buffer Buffer holding the encrypted content. Its size in words must be at least length
 * @param [in] length Number of words (i.e. 4 bytes) in the buffer to transfer
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_write_flash_encrypted( const void* context, const uint32_t offset,
                                                         const uint32_t* buffer, const uint8_t length );

/*!
 * @brief Write encrypted data in program flash memory of the chip
 *
 * This function shall be used when updating the encrypted flash content of the LR11XX.
 * The encrypted flash payload to transfer shall be represented as an array of words (ie 4-byte values).
 *
 * @param [in] context Chip implementation context
 * @param [in] offset The offset from start register of flash
 * @param [in] buffer Buffer holding the encrypted content. Its size in words must be at least length
 * @param [in] length Number of words (i.e. 4 bytes) in the buffer to transfer
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_write_flash_encrypted_full( const void* context, const uint32_t offset,
                                                              const uint32_t* buffer, const uint32_t length );

/*!
 * @brief Software reset of the chip.
 *
 * This method should be used to reboot the chip in a specified mode.
 * Rebooting in flash mode presumes that the content in flash memory is not corrupted (i.e. the integrity check
 * performed by the bootloader before executing the first instruction in flash is OK).
 *
 * @param [in] context Chip implementation context
 * @param [in] stay_in_bootloader Selector to stay in bootloader or execute flash code after reboot. If true, the
 * bootloader will not execute the flash code but activate SPI interface to allow firmware upgrade
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_reboot( const void* context, const bool stay_in_bootloader );

/*!
 * @brief Returns the 4-byte PIN which can be used to claim a device on cloud services.
 *
 * @param [in] context Chip implementation context
 * @param [out] pin Pointer to the array to be populated with the PIN
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_read_pin( const void* context, lr11xx_bootloader_pin_t pin );

/*!
 * @brief Read and return the Chip EUI
 *
 * @param [in] context Chip implementation context
 * @param [out] chip_eui The buffer to be filled with chip EUI of the LR11XX. It is up to the application to ensure
 * chip_eui is long enough to hold the chip EUI
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_read_chip_eui( const void* context, lr11xx_bootloader_chip_eui_t chip_eui );

/*!
 * @brief Read and return the Join EUI
 *
 * @param [in] context Chip implementation context
 * @param [out] join_eui The buffer to be filled with Join EUI of the LR11XX. It is up to the application to ensure
 * join_eui is long enough to hold the join EUI
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_read_join_eui( const void* context, lr11xx_bootloader_join_eui_t join_eui );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_BOOTLOADER_H

/* --- EOF ------------------------------------------------------------------ */
