/*!
 * @file      lr20xx_pram.h
 *
 * @brief     Patching commands for Lr20xx
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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

#ifndef LR20XX_PRAM_H
#define LR20XX_PRAM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_patch_types.h"
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

/**
 * @brief Transfer a Patch RAM (PRAM) into the chip
 *
 * This function is a helper function that execute several calls to @ref lr20xx_regmem_write_regmem32 in order to
 * transfer the complete PRAM.
 * After loading the PRAM with @ref lr20xx_pram_load_pram, it must be enabled by calling @ref lr20xx_pram_enable_pram.
 *
 * @note The helper functions @ref lr20xx_pram_load_pram_lr2021 and @ref lr20xx_pram_load_pram_lr20x2 handle the load
 * and enabling of the appropriate PRAM.
 *
 * @param context Chip implementation context
 * @param address The base address in the chip memory where the PRAM is to be written
 * @param buffer Pointer to the buffer being the PRAM data to transfer. It is up to the caller to ensure it contains at
 * least @p length elements
 * @param length Number of elements fro @p buffer array to write
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_enable_pram, lr20xx_regmem_write_regmem32, lr20xx_pram_get_version
 */
lr20xx_status_t lr20xx_pram_load_pram( const void* context, const uint32_t address, const uint32_t* buffer,
                                       const uint32_t length );

/**
 * @brief Enable Patch RAM (PRAM)
 *
 * This function must be called after @ref lr20xx_pram_load_pram to enable the PRAM usage.
 *
 * @param context Chip implementation context
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_load_pram, lr20xx_pram_get_version, lr20xx_pram_load_pram_lr2021, lr20xx_pram_load_pram_lr20x2
 */
lr20xx_status_t lr20xx_pram_enable_pram( const void* context );

/**
 * @brief Load and enable the LR2021-dedicated Patch RAM (PRAM) into the chip
 *
 * This function is a helper function that calls @ref lr20xx_pram_load_pram and @ref lr20xx_pram_enable_pram.
 *
 * @param context Chip implementation context
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_load_pram, lr20xx_pram_enable_pram, lr20xx_pram_load_pram_lr20x2
 */
lr20xx_status_t lr20xx_pram_load_pram_lr2021( const void* context );

/**
 * @brief Load and enable the LR20X2-dedicated Patch RAM (PRAM) into the chip
 *
 * This function is a helper function that calls @ref lr20xx_pram_load_pram and @ref lr20xx_pram_enable_pram.
 *
 * @param context Chip implementation context
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_load_pram, lr20xx_pram_enable_pram, lr20xx_pram_load_pram_lr2021
 */
lr20xx_status_t lr20xx_pram_load_pram_lr20x2( const void* context );

/**
 * @brief Get Patch RAM (PRAM) version information
 *
 * This function can be called after @ref lr20xx_pram_load_pram to check that PRAM has been correctly loaded.
 *
 * @param context Chip implementation context
 * @param pram_version The version information of the PRAM. Only valid if function returned with status @ref
 * LR20XX_STATUS_OK
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_load_pram, lr20xx_pram_enable_pram, lr20xx_pram_load_pram_lr2021, lr20xx_pram_load_pram_lr20x2
 */

lr20xx_status_t lr20xx_pram_get_version( const void* context, lr20xx_patch_version_t* pram_version );

/**
 * @brief Load and enable the LR20X2-dedicated Patch RAM (PRAM) into the chip
 *
 * This function is a helper function that calls @ref lr20xx_pram_load_pram and @ref lr20xx_pram_enable_pram.
 *
 * @param context Chip implementation context
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_load_pram, lr20xx_pram_enable_pram, lr20xx_pram_check_pram_lr20x2
 */
lr20xx_status_t lr20xx_pram_check_pram_lr2021( const void* context );

/**
 * @brief Check Patch RAM (PRAM) version integrate in chip
 *
 * This function can be called after @ref lr20xx_pram_load_pram to check that PRAM has been correctly loaded and
 * the version is the same compared to the software.
 *
 * @param context Chip implementation context
 *
 * @return lr20xx_status_t Operation status
 *
 * @see lr20xx_pram_load_pram, lr20xx_pram_check_pram_lr2021
 */
lr20xx_status_t lr20xx_pram_check_pram_lr20x2( const void* context );

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_PRAM_H

/* --- EOF ------------------------------------------------------------------ */
