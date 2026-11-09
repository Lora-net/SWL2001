/*!
 * \file      device_management_defs.h
 *
 * \brief     definition file for device management
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

#ifndef DEVICE_MANAGEMENT_DEFS_H__
#define DEVICE_MANAGEMENT_DEFS_H__

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
#if defined( LR1110_MODEM_E ) && defined( _MODEM_E_GNSS_ENABLE )
#define GNSS_DM_MSG 2  // Message for the device management
#endif                 // LR1110_MODEM_E && _MODEM_E_GNSS_ENABLE

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */
#define DM_PORT 199
#define DM_DOWNLINK_HEADER_LENGTH 3

/**
 * @brief Defined the type of reset that can be requested by downlink
 *
 * @enum dm_reset_code_t
 */

/**
 * @brief Definition of return codes for dm functions
 *
 * @enum dm_rc_t
 */
typedef enum dm_rc_e
{
    DM_ERROR,  //!< Error during dm function
    DM_OK,     //!< dm function executed without error
} dm_rc_t;

/**
 * @brief Status of DM downlink length
 *
 * @enum dm_cmd_length_valid_t
 */
typedef enum dm_cmd_length_valid_e
{
    DM_CMD_LENGTH_VALID,      //!< The length of the command is valid
    DM_CMD_LENGTH_NOT_VALID,  //!< The length of the command is not valid
    DM_CMD_NOT_VALID,         //!< The command is not valid
} dm_cmd_length_valid_t;

/**
 * @brief Definition of the DAS downlink request opcodes
 *
 * @enum dm_opcode_t
 */
typedef enum dm_opcode_e
{
    DM_RESET       = 0x00,  //!< reset modem or application MCU
    DM_FUOTA       = 0x01,  //!< FUOTA firmware update chunk
    DM_FILE_DONE   = 0x02,  //!< file upload complete
    DM_GET_INFO    = 0x03,  //!< get info fields
    DM_SET_CONF    = 0x04,  //!< set config field
    DM_REJOIN      = 0x05,  //!< rejoin network
    DM_MUTE        = 0x06,  //!< permanently disable/enable modem
    DM_SET_DM_INFO = 0x07,  //!< set list of default info fields
    DM_STREAM      = 0x08,  //!< set data stream parameters
    DM_ALC_SYNC    = 0x09,  //!< application layer clock sync data
    DM_ALM_UPDATE  = 0x0A,  //!< almanac update, short, long, full updates
    DM_ALM_DBG     = 0x0B,  //!< almanac debug
    DM_SOLV_UPDATE = 0x0C,  //!< assistance position, xtal update
    DM_ALM_FUPDATE = 0x0D,  //!< Force almanac update, short, long, full updates
    DM_CMD_MAX              //!< number of elements
} dm_opcode_t;

/**
 * @brief Type of DM, immediate or periodic reporting
 *
 * @enum dm_info_rate_t
 */
typedef enum dm_info_rate_e
{
    DM_INFO_PERIODIC = 0,  //!< Related to the Device Management periodic reporting
    DM_INFO_NOW            //!< Related to the Device Management immediately reporting
} dm_info_rate_t;

/**
 * @brief Periodic Status Reporting field opcode
 *
 * @enum dm_info_field_t
 */
typedef enum dm_info_field_e
{
    DM_INFO_STATUS    = 0x00,  //!< modem status
    DM_INFO_CHARGE    = 0x01,  //!< charge counter [mAh]
    DM_INFO_VOLTAGE   = 0x02,  //!< supply voltage [1/50 V]
    DM_INFO_TEMP      = 0x03,  //!< junction temperature [deg Celsius]
    DM_INFO_SIGNAL    = 0x04,  //!< strength of last downlink (RSSI [dBm]+64, SNR [0.25 dB])
    DM_INFO_UPTIME    = 0x05,  //!< duration since last reset [h]
    DM_INFO_RXTIME    = 0x06,  //!< duration since last downlink [h]
    DM_INFO_FIRMWARE  = 0x07,  //!< firmware CRC and fuota progress (completed/total chunks)
    DM_INFO_ADRMODE   = 0x08,  //!< ADR profile (0-3)
    DM_INFO_JOINEUI   = 0x09,  //!< JoinEUI
    DM_INFO_INTERVAL  = 0x0A,  //!< reporting interval [values 0-63, units s/m/h/d]
    DM_INFO_REGION    = 0x0B,  //!< regulatory region
    DM_INFO_RFU_0     = 0x0C,  //!< not defined
    DM_INFO_CRASHLOG  = 0x0D,  //!< crash log data
    DM_INFO_UPLOAD    = 0x0E,  //!< application file fragments
    DM_INFO_RSTCOUNT  = 0x0F,  //!< modem reset count
    DM_INFO_DEVEUI    = 0x10,  //!< DevEUI
    DM_INFO_RFU_1     = 0x11,  //!< not defined, old owner number
    DM_INFO_SESSION   = 0x12,  //!< session id / join nonce
    DM_INFO_CHIPEUI   = 0x13,  //!< ChipEUI
    DM_INFO_STREAM    = 0x14,  //!< data stream fragments
    DM_INFO_STREAMPAR = 0x15,  //!< data stream parameters
    DM_INFO_APPSTATUS = 0x16,  //!< application-specific status
    DM_INFO_ALCSYNC   = 0x17,  //!< application layer clock sync data
    DM_INFO_ALMSTATUS = 0x18,  //!< almanac status
    DM_INFO_DBGRSP    = 0x19,  //!< almanac dbg response
    DM_INFO_GNSSLOC   = 0x1A,  //!< GNSS scan NAV message
    DM_INFO_WIFILOC   = 0x1B,  //!< Wifi scan results message
    DM_INFO_MAX                //!< number of elements
} dm_info_field_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // DEVICE_MANAGEMENT_DEFS_H__

/* --- EOF ------------------------------------------------------------------ */
