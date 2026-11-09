/**
 * @file      ralf.h
 *
 * @brief     Radio abstraction layer definition
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

#ifndef RALF_H__
#define RALF_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "ral.h"
#include "ralf_defs.h"
#include "ralf_drv.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * Direct-dispatch switch (see ral.h for rationale).
 * When RAL_DRIVER_DIRECT_DISPATCH is defined, RALF dispatchers call the
 * concrete ralf_<family>_* entry points directly and the ralf_drv_t table is
 * omitted from ralf_t.
 */
#if defined( RAL_DRIVER_DIRECT_DISPATCH )
#if defined( RAL_DRIVER_DIRECT_DISPATCH_LR11XX )
#define RALF_DRV_CALL( fn ) ralf_lr11xx_##fn
#elif defined( RAL_DRIVER_DIRECT_DISPATCH_LR20XX )
#define RALF_DRV_CALL( fn ) ralf_lr20xx_##fn
#elif defined( RAL_DRIVER_DIRECT_DISPATCH_SX126X )
#define RALF_DRV_CALL( fn ) ralf_sx126x_##fn
#elif defined( RAL_DRIVER_DIRECT_DISPATCH_SX127X )
#define RALF_DRV_CALL( fn ) ralf_sx127x_##fn
#elif defined( RAL_DRIVER_DIRECT_DISPATCH_SX128X )
#define RALF_DRV_CALL( fn ) ralf_sx128x_##fn
#else
#error "RAL_DRIVER_DIRECT_DISPATCH: no RALF implementation for the selected radio family"
#endif
#define RALF_INSTANTIATE_DRIVER( drv )
#else
#define RALF_DRV_CALL( fn ) radio->ralf_drv.fn
#define RALF_INSTANTIATE_DRIVER( drv ) .ralf_drv = drv,
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct ralf_s
{
    // ral must be the first element of ralf_t to enable ralf_from_ral()
    ral_t      ral;
#if !defined( RAL_DRIVER_DIRECT_DISPATCH )
    ralf_drv_t ralf_drv;
#endif
} ralf_t;

/*
 * Forward-declare the concrete ralf_<family>_setup_* entry points used by the
 * direct dispatch macros above. They are defined in ralf_<family>.c. We cannot
 * include ralf_<family>.h here because it itself includes ralf.h (it needs
 * ralf_t), which would create a circular include.
 */
#if defined( RAL_DRIVER_DIRECT_DISPATCH )
ral_status_t RALF_DRV_CALL( setup_gfsk )( const ralf_t* radio, const ralf_params_gfsk_t* params );
ral_status_t RALF_DRV_CALL( setup_lora )( const ralf_t* radio, const ralf_params_lora_t* params );
ral_status_t RALF_DRV_CALL( setup_flrc )( const ralf_t* radio, const ralf_params_flrc_t* params );
ral_status_t RALF_DRV_CALL( setup_lora_cad )( const ralf_t* radio, const ralf_params_lora_cad_t* params );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Setup radio to transmit and receive data using GFSK modem
 *
 * @remark When transmitting in GFSK mode the radio access may be blocking
 *
 * @param [in] radio Pointer to radio data
 * @param [in] params GFSK modem transmission parameters
 *
 * @returns status Operation status
 */
static inline ral_status_t ralf_setup_gfsk( const ralf_t* radio, const ralf_params_gfsk_t* params )
{
    return RALF_DRV_CALL( setup_gfsk )( radio, params );
}

/**
 * Setup radio to transmit and receive data using LoRa modem
 *
 * @param [in] radio Pointer to radio data
 * @param [in] params LoRa modem transmission parameters
 *
 * @returns status Operation status
 */
static inline ral_status_t ralf_setup_lora( const ralf_t* radio, const ralf_params_lora_t* params )
{
    return RALF_DRV_CALL( setup_lora )( radio, params );
}

/**
 * Setup radio to transmit and receive data using the FLRC modem
 *
 * @param [in] radio Pointer to radio data
 * @param [in] params transmission parameters
 *
 * @returns status Operation status
 */
static inline ral_status_t ralf_setup_flrc( const ralf_t* radio, const ralf_params_flrc_t* params )
{
    return RALF_DRV_CALL( setup_flrc )( radio, params );
}

/**
 * Setup radio to run CAD using LoRa modem
 *
 * @param [in] radio Pointer to radio data
 * @param [in] params LoRa modem CAD parameters
 *
 * @returns status Operation status
 */
static inline ral_status_t ralf_setup_lora_cad( const ralf_t* radio, const ralf_params_lora_cad_t* params )
{
    return RALF_DRV_CALL( setup_lora_cad )( radio, params );
}

/**
 * @brief Convert ral_t* to ralf_t*
 *
 * @remark Only use this if you are certain that the object pointed to is actually a ralf_t*.
 *
 * @param [in] radio Pointer to ral_t
 * @returns          Pointer to ralf_t
 */
static inline ralf_t* ralf_from_ral( ral_t* radio )
{
    return ( ralf_t* ) radio;
}

/**
 * @brief Convert ralf_t* to ral_t*
 *
 * @param [in] radio Pointer to ralf_t
 * @returns          Pointer to ral_t
 */
static inline ral_t* ral_from_ralf( ralf_t* radio )
{
    return &radio->ral;
}

/**
 * @brief Convert const ral_t* to const ralf_t*
 *
 * @remark Only use this if you are certain that the object pointed to is actually a ralf_t*.
 *
 * @param [in] radio Pointer to const ral_t
 * @returns          Pointer to const ralf_t
 */
static inline const ralf_t* ralf_from_const_ral( const ral_t* radio )
{
    return ( const ralf_t* ) radio;
}

/**
 * @brief Convert const ralf_t* to const ral_t*
 *
 * @param [in] radio Pointer to const ralf_t
 * @returns          Pointer to const ral_t
 */
static inline const ral_t* ral_from_const_ralf( const ralf_t* radio )
{
    return &radio->ral;
}

#ifdef __cplusplus
}
#endif

#endif  // RALF_H__

/* --- EOF ------------------------------------------------------------------ */
