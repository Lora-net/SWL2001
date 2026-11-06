/**
 * @file      sx126x_bpsk.h
 *
 * @brief     SX126x BPSK radio driver definition
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#ifndef SX126X_BPSK_H
#define SX126X_BPSK_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "sx126x_status.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * \brief Ramp-up delay for the power amplifier
 *
 * This parameter configures the delay to fine tune the ramp-up time of the power amplifier for BPSK operation.
 */
enum
{
    SX126X_SIGFOX_DBPSK_RAMP_UP_TIME_DEFAULT = 0x0000,  //!< No optimization
    SX126X_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS = 0x370F,  //!< Ramp-up optimization for 100bps
    SX126X_SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS = 0x092F,  //!< Ramp-up optimization for 600bps
};

/*!
 * \brief Ramp-down delay for the power amplifier
 *
 * This parameter configures the delay to fine tune the ramp-down time of the power amplifier for BPSK operation.
 */
enum
{
    SX126X_SIGFOX_DBPSK_RAMP_DOWN_TIME_DEFAULT = 0x0000,  //!< No optimization
    SX126X_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS = 0x1D70,  //!< Ramp-down optimization for 100bps
    SX126X_SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS = 0x04E1,  //!< Ramp-down optimization for 600bps
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief SX126X BPSK modulation shaping enumeration definition
 */
typedef enum
{
    SX126X_DBPSK_PULSE_SHAPE = 0x16,  //!< Double OSR / RRC / BT 0.7
} sx126x_bpsk_pulse_shape_t;

/**
 * @brief Modulation configuration for BPSK packet
 */
typedef struct sx126x_mod_params_bpsk_s
{
    uint32_t                  br_in_bps;    //!< BPSK bitrate [bit/s]
    sx126x_bpsk_pulse_shape_t pulse_shape;  //!< BPSK pulse shape
} sx126x_mod_params_bpsk_t;

/**
 * @brief SX126X BPSK packet parameters structure definition
 */
typedef struct sx126x_pkt_params_bpsk_s
{
    uint8_t  pld_len_in_bytes;  //!< Payload length [bytes]
    uint16_t ramp_up_delay;     //!< Delay to fine tune ramp-up time, if non-zero
    uint16_t ramp_down_delay;   //!< Delay to fine tune ramp-down time, if non-zero
    uint16_t pld_len_in_bits;   //!< If non-zero, used to ramp down PA before end of a payload with length that is not a
                                //!< multiple of 8
} sx126x_pkt_params_bpsk_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Set the modulation parameters for BPSK packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of BPSK modulation configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_bpsk_mod_params( const void* context, const sx126x_mod_params_bpsk_t* params );

/**
 * @brief Set the packet parameters for BPSK packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of BPSK packet configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_bpsk_pkt_params( const void* context, const sx126x_pkt_params_bpsk_t* params );

#ifdef __cplusplus
}
#endif

#endif  // SX126X_BPSK_H

/* --- EOF ------------------------------------------------------------------ */
