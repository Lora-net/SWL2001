/*!
 * @file      lr20xx_patch_types.h
 *
 * @brief     Patch types driver definition for LR20XX
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

#ifndef LR20XX_PATCH_TYPES_H
#define LR20XX_PATCH_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

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
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief PRAM version information
 *
 * @ref lr20xx_patch_version_e::is_pram_loaded indicates if a PRAM is loaded.
 * If @ref lr20xx_patch_version_e::is_pram_loaded is true then lr20xx_patch_version_e::pram_type contains the type of
 * PRAM, and lr20xx_patch_version_e::pram_version contains the version of the loaded PRAM. If @ref
 * lr20xx_patch_version_e::is_pram_loaded is false, then value of fields lr20xx_patch_version_e::pram_type and
 * lr20xx_patch_version_e::pram_version are undefined, and should not be read.
 */
typedef struct lr20xx_patch_version_e
{
    bool    is_pram_loaded;  //!< True if the PRAM is loaded, false otherwise
    uint8_t pram_type;       //!< Type of the loaded PRAM, if lr20xx_patch_version_e::is_pram_loaded is true
    uint8_t pram_version;    //!< Version of the loaded PRAM, if lr20xx_patch_version_e::is_pram_loaded is true
} lr20xx_patch_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_PATCH_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
