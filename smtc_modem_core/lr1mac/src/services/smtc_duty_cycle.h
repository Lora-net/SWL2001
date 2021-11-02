/*!
 * \file      smtc_duty_cycle.h
 *
 * \brief     Duty Cycle implementation
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
#ifndef __SMTC_DUTY_CYCLE_H__
#define __SMTC_DUTY_CYCLE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
// clang-format off
#define SMTC_DTC_BANDS_MAX          ( 6 )                      // Number of ETSI band supported by this algo
#define SMTC_DTC_PERIOD_MS          ( 3600000UL )              // Number of miliseconds in one period (3600000 for period 1h)
#define SMTC_DTC_SECONDS_BY_UNIT    ( 120 )                    // Sum TOA by step of N seconds, MIN VALUE IS 60s to avoid division by 0
#define SMTC_DTC_TOA_BUFF_SIZE      ( ( SMTC_DTC_PERIOD_MS / 1000UL ) / SMTC_DTC_SECONDS_BY_UNIT )  // Buffer size to sum all TOA over one period

#if SMTC_DTC_SECONDS_BY_UNIT < 60
#warning "SMTC_DTC_SECONDS_BY_UNIT must be greater or equal to 60"
#undef SMTC_DTC_SECONDS_BY_UNIT
#define SMTC_DTC_SECONDS_BY_UNIT 60
#endif

//
// Represention of the current configuration
//
// index                       0     1     2                  28    29   0
// 16bits array to save TOA {[    ][    ][    ][    ...    ][    ][    ][    ]}
// RTC                       0s    120s  240s  360s         3360s 3480s 3600s
//

// clang-format on

typedef enum smtc_dtc_enablement_type_e
{
    SMTC_DTC_PARTIAL_DISABLED = 0,  // No sending constraint but uplink TOA are take in care
    SMTC_DTC_ENABLED          = 1,  // Sending constraint and uplink TOA are take in care
    SMTC_DTC_FULL_DISABLED    = 2,  // Sending constraint and uplink TOA are NOT take in care
} smtc_dtc_enablement_type_t;
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
typedef struct smtc_dtc_band_s
{
    uint32_t freq_min;
    uint32_t freq_max;
    uint16_t duty_cycle_regulation;  // 1000->0.1%, 100->1%, 10->10%
    uint32_t toa_timestamp_ms;       // last access to the array when adding the TOA or reset all TOA
    uint8_t  index_previous;
    uint16_t toa_sum_ms[SMTC_DTC_TOA_BUFF_SIZE];  // Store all TOA by step of SMTC_DTC_SECONDS_BY_UNIT
} smtc_dtc_band_t;

typedef struct smtc_dtc_s
{
    smtc_dtc_enablement_type_t enabled;
    uint8_t                    number_of_bands;
    smtc_dtc_band_t            bands[SMTC_DTC_BANDS_MAX];
} smtc_dtc_t;

/**
 * @brief the resolution is the number of bits per SMTC_DTC_SECONDS_BY_UNIT
 * @remark 16 represents the number of bits to save the TOA
 *          ceil(120s/65536) -> ceil(0.001831055) -> 2ms per bit
 */
static const uint32_t smtc_dtc_resolution_ms = ( ( ( SMTC_DTC_SECONDS_BY_UNIT * 1000 ) / ( 1 << ( 16 ) ) ) + 1 );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Duty cycle initialization
 *
 * @param dtc_obj Contains the duty cycle context
 */
void smtc_duty_cycle_init( smtc_dtc_t* dtc_obj );

/**
 * @brief Duty cycle configuration
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param number_of_bands           Number of bands in this region
 * @param band_idx                  Index bands to store configuration
 * @param duty_cycle_regulation     Duty cycle limitation on this band
 * @param freq_min                  Frequency min on this band
 * @param freq_max                  Frequency max on this band
 */
void smtc_duty_cycle_config( smtc_dtc_t* dtc_obj, uint8_t number_of_bands, uint8_t band_idx,
                             uint16_t duty_cycle_regulation, uint32_t freq_min, uint32_t freq_max );

/**
 * @brief Duty cycle enablement
 *
 * @remark When disabled the TOA is take into account, but no restriction if DTC full consumed
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param enable                    Enable: true, Disabled: false,
 * @return uint8_t                  return false if not initialized
 */
uint8_t smtc_duty_cycle_enable_set( smtc_dtc_t* dtc_obj, smtc_dtc_enablement_type_t enable );

/**
 * @brief Duty cycle enablement status
 *
 * @param dtc_obj
 * @return smtc_dtc_enablement_type_t
 */
smtc_dtc_enablement_type_t smtc_duty_cycle_enable_get( smtc_dtc_t* dtc_obj );

/**
 * @brief Sum Time On Air for a specified freq (inside a band)
 *
 * @remark smtc_duty_cycle_update() must be called before this function to have a right value
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param freq_hz                   Frequency used for the packet
 * @param toa_ms                    Packet Time On Air in milliseconds
 */
void smtc_duty_cycle_sum( smtc_dtc_t* dtc_obj, uint32_t freq_hz, uint32_t toa_ms );

/**
 * @brief  Update Time On Air
 *
 * @remark smtc_duty_cycle_update() must be called before check Duty Cycle available
 *
 * @param dtc_obj                   Contains the duty cycle context
 */
void smtc_duty_cycle_update( smtc_dtc_t* dtc_obj );

/**
 * @brief Check if Time On Air is not greater than TOA available for a frequency
 *
 * @remark  smtc_duty_cycle_update() must be called before this function to have a right value
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param freq_hz                   Frequency used for the packet
 * @param toa_ms                    Packet Time On Air in milliseconds
 * @return bool
 */
bool smtc_duty_cycle_is_toa_accepted( smtc_dtc_t* dtc_obj, uint32_t freq_hz, uint32_t toa_ms );

/**
 * @brief Get Time On Air available in a band
 *
 * @remark  smtc_duty_cycle_update() must be called before this function to have a right value
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param band                      Band id
 * @return int32_t                  return Time On Air available
 */
int32_t smtc_duty_cycle_band_get_available_toa_ms( smtc_dtc_t* dtc_obj, uint8_t band );

/**
 * @brief Check if a channel is Duty Cycle free
 *
 * @remark  smtc_duty_cycle_update() must be called before this function to have a right value
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param freq_hz                   Frequency that need a check
 * @return bool
 */
bool smtc_duty_cycle_is_channel_free( smtc_dtc_t* dtc_obj, uint32_t freq_hz );

/**
 * @brief Check if a band is Duty Cycle free
 *
 * @remark  smtc_duty_cycle_update() must be called before this function to have a right value
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param band                      Band that need a check
 * @return bool
 */
bool smtc_duty_cycle_is_band_free( smtc_dtc_t* dtc_obj, uint8_t band );

/**
 * @brief Get the next available slot with free duty cycle
 *
 * @remark  smtc_duty_cycle_update() must be called before this function to have a right value
 *
 * @param dtc_obj                   Contains the duty cycle context
 * @param number_of_tx_freq         number of tx freq in list
 * @param tx_freq_list              tx frequency list used by the app to check only duty cycle in these bands
 * @return int32_t                  milliseconds, if > 0: the next slot availble, else the available time
 */
int32_t smtc_duty_cycle_get_next_free_time_ms( smtc_dtc_t* dtc_obj, uint8_t number_of_tx_freq, uint32_t* tx_freq_list );
#ifdef __cplusplus
}
#endif

#endif  // __SMTC_DUTY_CYCLE_H__

/* --- EOF ------------------------------------------------------------------ */
