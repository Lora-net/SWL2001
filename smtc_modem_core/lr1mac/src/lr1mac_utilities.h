/*!
 * \file      lr1_mac_utilities.h
 *
 * \brief     LoRaWan utilities definition
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

#ifndef __LR1MAC_UTILITIES_H__
#define __LR1MAC_UTILITIES_H__

#ifdef __cplusplus
extern "C" {
#endif
/*
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>  // ceilf() and floorf()
#include "ral_defs.h"
#include "lr1mac_defs.h"
/*
 *-----------------------------------------------------------------------------------
 * --- PUBLIC MACROS ----------------------------------------------------------------
 */
#define SMTC_MODEM_HAL_TRACE_PRINTF_DEBUG( ... )  //@todo push it to the modem layer
/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )

/**
 * @brief Math Abs macro
 */
#define ABS( N ) ( ( N < 0 ) ? ( -N ) : ( N ) )
/**
 * @brief Math signed macro
 */

#define SIGN( N ) ( ( N < 0 ) ? ( -1 ) : ( 1 ) )

uint8_t SMTC_GET_BIT8( const uint8_t* array, uint8_t index );
void    SMTC_SET_BIT8( uint8_t* array, uint8_t index );
void    SMTC_CLR_BIT8( uint8_t* array, uint8_t index );
void    SMTC_PUT_BIT8( uint8_t* array, uint8_t index, uint8_t bit );
uint8_t SMTC_ARE_CLR_BYTE8( uint8_t* array, uint8_t length );

uint8_t SMTC_GET_BIT16( const uint16_t* array, uint8_t index );
void    SMTC_SET_BIT16( uint16_t* array, uint8_t index );
void    SMTC_CLR_BIT16( uint16_t* array, uint8_t index );
void    SMTC_PUT_BIT16( uint16_t* array, uint8_t index, uint8_t bit );
uint8_t SMTC_ARE_CLR_BYTE16( uint16_t* array, uint8_t length );
/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------------
 */

/*!
 * \brief Copy size elements of src array to dst array
 *
 * \remark STM32 Standard memcpy function only works on pointers that are aligned
 *
 * \param [OUT] dst   Destination array
 * \param [IN]  src   Source array
 * \param [IN]  size  Number of bytes to be copied
 */
void memcpy1( uint8_t* dst, const uint8_t* src, uint16_t size );

/*!
 * \brief Reverse copy size elements of src
 * array to dst array
 *
 * \remark STM32 Standard memcpy function only
 * works on pointers that are aligned
 *
 * \param [OUT] dst   Destination array
 * \param [IN]  src   Source array
 * \param [IN]  size  Number of bytes to be
 * copied
 */
void memcpy1_r( uint8_t* dst, const uint8_t* src, uint16_t size );

/*!
 * \brief Set size elements of dst array with
 * value
 *
 * \remark STM32 Standard memset function only
 * works on pointers that are aligned
 *
 * \param [OUT] dst   Destination array
 * \param [IN]  value Default value
 * \param [IN]  size  Number of bytes to be
 * copied
 */
void memset1( uint8_t* dst, uint8_t value, uint16_t size );

/*!
 * \brief Crc32 implementation for flash
 * corruption
 *
 */
uint32_t lr1mac_utilities_crc( uint8_t* buf, int len );

/*!
 * \brief Compute symbol time in µs
 *
 */
uint32_t lr1mac_utilities_get_symb_time_us( const uint16_t nb_symb, const ral_lora_sf_t sf, const ral_lora_bw_t bw );

/*!
 * \brief is valid Rx payload min size
 *
 */
status_lorawan_t lr1mac_rx_payload_min_size_check( uint8_t rx_payload_size );

/*!
 * \brief Extract MHDR
 *
 */
status_lorawan_t lr1mac_rx_mhdr_extract( uint8_t* rx_payload, uint8_t* rx_ftype, uint8_t* rx_major,
                                         uint8_t* tx_ack_bit );

/*!
 * \brief Extract FHDR
 *
 */
int lr1mac_rx_fhdr_extract( uint8_t* rx_payload, uint8_t rx_payload_size, uint8_t* rx_fopts_length,
                            uint16_t* fcnt_dwn_tmp, uint32_t dev_addr, uint8_t* rx_fport, uint8_t* rx_payload_empty,
                            uint8_t* rx_fctrl, uint8_t* rx_fopts );
/*!
 * \brief Extract MHDR
 *
 */
status_lorawan_t lr1mac_fcnt_dwn_accept( uint16_t fcnt_dwn_tmp, uint32_t* fcnt_lorawan );

/**
 * @brief if the mac command answer is bigger than the allowed payload size, the payload is cut
 *
 * @param nwk_ans
 * @param nwk_ans_size_in
 * @param max_allowed_size
 * @return uint8_t
 */
uint8_t lr1_stack_mac_cmd_ans_cut( uint8_t* nwk_ans, uint8_t nwk_ans_size_in, uint8_t max_allowed_size );

#ifdef __cplusplus
}
#endif

#endif  // __LR1MAC_UTILITIES_H__
