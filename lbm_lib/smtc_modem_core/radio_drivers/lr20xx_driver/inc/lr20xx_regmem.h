/*!
 * @file      lr20xx_regmem.h
 *
 * @brief     Register/memory driver definition for LR20XX
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

#ifndef LR20XX_REGMEM_H
#define LR20XX_REGMEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_status.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief Maximum number of words that can be written to / read from a LR20XX chip with regmem32 commands
 */
#define LR20XX_REGMEM_MAX_WRITE_READ_WORDS 32

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
 * @brief Write up to 32 words into register memory space of the transceiver.
 *
 * A word is 32-bit long. The writing operations write contiguously in register memory, starting at the address
 * provided.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start writing operation (only the 3 bytes LSB are relevant)
 * @param [in] buffer The buffer of words to write into memory. Its size must be enough to contain length words.
 * @param [in] length Number of words to write into memory
 *
 * @returns Operation status
 *
 * @see lr20xx_regmem_read_regmem32
 */
lr20xx_status_t lr20xx_regmem_write_regmem32( const void* context, const uint32_t address, const uint32_t* buffer,
                                              const uint8_t length );

/*!
 * @brief Read-modify-write data at given register/memory address
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to be modified (only the 3 bytes LSB are relevant)
 * @param [in] mask The mask to be applied on read data
 * @param [in] data The data to be written
 *
 * @returns Operation status
 */
lr20xx_status_t lr20xx_regmem_write_regmem32_mask( const void* context, const uint32_t address, const uint32_t mask,
                                                   const uint32_t data );

/*!
 * @brief Read up to 32 words into register memory space of the transceiver.
 *
 * A word is 32-bit long. The reading operations read contiguously from register memory, starting at the address
 * provided.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start reading operation (only the 3 bytes LSB are relevant)
 * @param [in] length Number of words to read from memory
 * @param [out] buffer Pointer to a words array to be filled with content from memory. Its size must be enough to
 * contain at least length words.
 *
 * @returns Operation status
 *
 * @see lr20xx_regmem_write_regmem32
 */
lr20xx_status_t lr20xx_regmem_read_regmem32( const void* context, const uint32_t address, uint32_t* buffer,
                                             const uint8_t length );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_REGMEM_H

/* --- EOF ------------------------------------------------------------------ */
