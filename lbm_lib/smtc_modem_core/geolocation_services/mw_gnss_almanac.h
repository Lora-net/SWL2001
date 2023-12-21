/**
 * @file      mw_gnss_almanac.h
 *
 * @brief     GNSS almanac demodulation service
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

#ifndef MW_GNSS_ALMANAC_H
#define MW_GNSS_ALMANAC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_geolocation_api.h"

#include "lr1_stack_mac_layer.h"

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

////////////////////////////////////////////////////////
//////////// Init service object ///////////////////////
////////////////////////////////////////////////////////

/**
 * @brief Initialize GNSS almanac demodulation service
 */
void mw_gnss_almanac_services_init( uint8_t* service_id, uint8_t task_id,
                                    uint8_t ( **downlink_callback )( lr1_stack_mac_down_data_t* ),
                                    void ( **on_launch_callback )( void* ), void ( **on_update_callback )( void* ),
                                    void** context_callback );

/**
 * @brief Start the GNSS almanac demodulation service
 */
void mw_gnss_almanac_add_task( void );

/**
 * @brief Get data associated with the SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE event.
 *
 * @param [out] data    Pointer to status data of the almanac update
 *
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Given pointer to hold data is NULL
 * @retval SMTC_MODEM_RC_FAIL       No SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE event pending
 */
smtc_modem_return_code_t mw_gnss_almanac_get_event_almanac_update(
    smtc_modem_almanac_demodulation_event_data_almanac_update_t* data );

/**
 * @brief Set the GNSS constellations to be used for almanac demodulation
 *
 * @param [in] constellations Constellation(s) to be used for almanac demodulation
 *
 * By default it is configured for using both GPS and BEIDOU constellations
 *
 * @return Modem return code as defined in @ref smtc_modem_return_code_t
 * @retval SMTC_MODEM_RC_OK         Command executed without errors
 * @retval SMTC_MODEM_RC_INVALID    Unsupported constellation
 */
smtc_modem_return_code_t mw_gnss_almanac_set_constellations( smtc_modem_gnss_constellation_t constellations );

#ifdef __cplusplus
}
#endif

#endif  // MW_GNSS_ALMANAC_H

/* --- EOF ------------------------------------------------------------------ */
