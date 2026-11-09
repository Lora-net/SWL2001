/**
 * @file      geolocation_bsp.h
 *
 * @brief     Board Support Package for the Geolocation service.
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

#ifndef GEOLOCATION_BSP_H
#define GEOLOCATION_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr11xx_system_types.h"
#include "lr11xx_gnss_types.h"

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
 * @brief Board specific actions that need to be taken before starting a GNSS scan
 */
void geolocation_bsp_gnss_prescan_actions( void );

/**
 * @brief Board specific actions that need to be taken after the end of a GNSS scan
 */
void geolocation_bsp_gnss_postscan_actions( void );

/**
 * @brief Board specific actions that need to be taken before starting a WIFI scan
 */
void geolocation_bsp_wifi_prescan_actions( void );

/**
 * @brief Board specific actions that need to be taken after the end of a WIFI scan
 */
void geolocation_bsp_wifi_postscan_actions( void );

/**
 * @brief Get the lr11xx current Low Frequency clock configuration
 *
 * @return lr11xx_system_lfclk_cfg_t
 */
lr11xx_system_lfclk_cfg_t geolocation_bsp_get_lr11xx_lf_clock_cfg( void );

/**
 * @brief Get the lr11xx current regulator mode
 */
void geolocation_bsp_get_lr11xx_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode );

/**
 * @brief Get the lr11xx instantaneous power consumption for the given board.
 */
void geolocation_bsp_gnss_get_consumption(
    lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua );

#ifdef __cplusplus
}
#endif

#endif  // GEOLOCATION_BSP_H

/* --- EOF ------------------------------------------------------------------ */
