/**
 * @file      smtc_modem_hal.h
 *
 * @brief     Modem Hardware Abstraction Layer API description
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
#ifndef SMTC_MODEM_HAL_H
#define SMTC_MODEM_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_core_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/* ------------ Environment management ------------*/

/**
 * @brief Return the battery level
 *
 * @remark
 * Please implement according to used board
 * According to LoRaWan 1.0.4 spec:
 *  0: The end-device is connected to an external power source.
 *  1..254: Battery level, where 1 is the minimum and 254 is the maximum.
 *  255: The end-device was not able to measure the battery level.
 * @return uint8_t Battery level for lorawan stack
 */
uint8_t smtc_modem_hal_get_battery_level( void );

/* ------------ Fuota management ------------*/

/**
 * @brief Only use if fmp package is activated
 *
 * @return uint32_t hw version as defined in fmp Alliance package TS006-1.0.0
 */
uint32_t smtc_modem_hal_get_hw_version_for_fuota( void );

/**
 * @brief Only use if fmp package is activated
 *
 * @return uint32_t fw version as defined in fmp Alliance package TS006-1.0.0
 */
uint32_t smtc_modem_hal_get_fw_version_for_fuota( void );

/**
 * @brief Only use if fmp package is activated
 *
 * @return uint8_t fw status field as defined in fmp Alliance package TS006-1.0.0
 */
uint8_t smtc_modem_hal_get_fw_status_available_for_fuota( void );

/**
 * @brief Only use if fmp package is activated
 *
 * @param [in] fw_to_delete_version    fw_to_delete_version as described in TS006-1.0.0
 * @return uint8_t fw status field as defined in fmp Alliance package TS006-1.0.0
 */
uint8_t smtc_modem_hal_get_fw_delete_status_for_fuota( uint32_t fw_to_delete_version );

/**
 * @brief Only use if fmp package is activated
 *
 * @return uint32_t the fw version that will be running once this firmware upgrade image is installed (as defined in fmp
 * Alliance package TS006-1.0.0)
 */
uint32_t smtc_modem_hal_get_next_fw_version_for_fuota( void );

/* ------------ Needed for Device Management  ------------*/

/**
 * @brief Return temperature in celsius
 *
 * @return int8_t temperature in celsius
 */
int8_t smtc_modem_hal_get_temperature( void );

/**
 * @brief Return mcu voltage in mv
 *
 * @return uint8_t MCU voltage
 */
uint16_t smtc_modem_hal_get_voltage_mv( void );

/**
 * @brief Stores the crashlog
 *
 * @remark This function is used to store the Modem crashlog in a non volatile memory
 *
 * @param [in] crash_string        Crashlog string to be stored
 * @param [in] crash_string_length Crashlog string length
 */
void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length );

/**
 * @brief Restores the crashlog
 *
 * @remark This function is used to restore the Modem crashlog from a non volatile memory
 *
 * @param [out] crash_string        Crashlog string to be restored
 * @param [out] crash_string_length Crashlog string length
 */
void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length );

/**
 * @brief Stores the crashlog status
 *
 * @remark This function is used to store the Modem crashlog status in a non volatile memory. This status will
 * allow the Modem to handle crashlog send task if needed after a crash
 *
 * @param [in] available True if a crashlog is available, false otherwise
 */
void smtc_modem_hal_crashlog_set_status( bool available );

/**
 * @brief Get the previously stored crashlog status
 *
 * @remark This function is used to get the Modem crashlog status from a non volatile memory. This status will
 * allow the Modem to handle crashlog send task if needed after a crash
 *
 * @return bool True if a crashlog is available, false otherwise
 */
bool smtc_modem_hal_crashlog_get_status( void );

/* ------------ Needed for Store and Forward service  ------------*/

/**
 * @brief The number of reserved pages in flash for the Store and Forward service
 * @remark the number must be at least 3 pages
 *
 * @return uint16_t
 */
uint16_t smtc_modem_hal_store_and_forward_get_number_of_pages( void );

/**
 * @brief Gives the size of a flash page in bytes
 *
 * @return uint16_t
 */
uint16_t smtc_modem_hal_flash_get_page_size( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_MODEM_HAL_H

/* --- EOF ------------------------------------------------------------------ */
