/*!
 * \file      smtc_modem_crypto.h
 *
 * \brief     Definition of crypto functions needed by modem.
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

#ifndef __SMTC_MODEM_CRYPTO_H__
#define __SMTC_MODEM_CRYPTO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_secure_element.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * LoRaWAN Join Nonce size in bytes
 */
#define LORAWAN_JOIN_NONCE_SIZE 3

/*!
 * LoRaWAN Net ID size in bytes
 */
#define LORAWAN_NET_ID_SIZE 3

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Crypto return codes
 *
 * @enum smtc_modem_crypto_return_code_t
 */
typedef enum smtc_modem_crypto_return_code_e
{
    SMTC_MODEM_CRYPTO_RC_SUCCESS = 0,             //!< No error occurred
    SMTC_MODEM_CRYPTO_RC_FAIL_MIC,                //!< MIC does not match
    SMTC_MODEM_CRYPTO_RC_FAIL_ADDRESS,            //!< Address does not match
    SMTC_MODEM_CRYPTO_RC_FAIL_JOIN_NONCE,         //!< JoinNonce was not greater than previous one.
    SMTC_MODEM_CRYPTO_RC_FAIL_RJCOUNT0_OVERFLOW,  //!< RJcount0 reached 2^16-1
    SMTC_MODEM_CRYPTO_RC_FAIL_FCNT_ID,            //!< FCNT_ID is not supported
    SMTC_MODEM_CRYPTO_RC_FAIL_FCNT_SMALLER,       //!<  FCntUp/Down check failed (new FCnt is smaller than previous one)
    SMTC_MODEM_CRYPTO_RC_FAIL_FCNT_DUPLICATED,    //!< FCntUp/Down check failed (duplicated)
    SMTC_MODEM_CRYPTO_RC_FAIL_PARAM,              //!< Not allowed parameter value
    SMTC_MODEM_CRYPTO_RC_ERROR_NPE,               //!< Null pointer exception
    SMTC_MODEM_CRYPTO_RC_ERROR_INVALID_KEY_ID,    //!< Invalid key identifier exception
    SMTC_MODEM_CRYPTO_RC_ERROR_INVALID_ADDR_ID,   //!< Invalid address identifier exception
    SMTC_MODEM_CRYPTO_RC_ERROR_INVALID_VERSION,   //!< Invalid LoRaWAN specification version
    SMTC_MODEM_CRYPTO_RC_ERROR_BUF_SIZE,          //!< Incompatible buffer size
    SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT,    //!< The secure element reports an error
    SMTC_MODEM_CRYPTO_RC_ERROR_PARSER,            //!< Error from parser reported
    SMTC_MODEM_CRYPTO_RC_ERROR_SERIALIZER,        //!< Error from serializer reported
    SMTC_MODEM_CRYPTO_RC_ERROR_RJCOUNT1_OVERFLOW,  //!< RJcount1 reached 2^16-1 which should never happen
    SMTC_MODEM_CRYPTO_RC_ERROR,                    //!< Undefined Error occurred
} smtc_modem_crypto_return_code_t;

/**
 * @brief Crypto address identifier
 *
 * @enum smtc_modem_crypto_addr_id_t
 */
typedef enum smtc_modem_crypto_addr_id_e
{
    SMTC_MODEM_CRYPTO_MULTICAST_0_ADDR = 0,  //!< Multicast Address 0
    SMTC_MODEM_CRYPTO_MULTICAST_1_ADDR = 1,  //!< Multicast Address 1
    SMTC_MODEM_CRYPTO_MULTICAST_2_ADDR = 2,  //!< Multicast Address 2
    SMTC_MODEM_CRYPTO_MULTICAST_3_ADDR = 3,  //!< Multicast Address 3
    SMTC_MODEM_CRYPTO_UNICAST_DEV_ADDR = 4,  //!< Unicast End-device address
} smtc_modem_crypto_addr_id_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Computes the payload encryption
 *
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] key_id Key identifier
 * @param [in] address Frame address
 * @param [in] dir Frame direction [0: uplink, 1: downlink]
 * @param [in] frame_counter Frame sequence counter
 * @param [in] enc_buffer Encrypted buffer
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_payload_encrypt( const uint8_t* buffer, uint16_t size,
                                                                   smtc_se_key_identifier_t key_id, uint32_t address,
                                                                   uint8_t dir, uint32_t frame_counter,
                                                                   uint8_t* enc_buffer );

/**
 * @brief Computes the payload decryption
 *
 * @param [in] enc_buffer Encrypted buffer
 * @param [in] size Encrypted buffer size
 * @param [in] key_id Key identifier
 * @param [in] address Frame address
 * @param [in] dir Frame direction [0: uplink, 1: downlink]
 * @param [in] frame_counter Frame sequence counter
 * @param [in] dec_buffer Decrypted buffer
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_payload_decrypt( const uint8_t* enc_buffer, uint16_t size,
                                                                   smtc_se_key_identifier_t key_id, uint32_t address,
                                                                   uint8_t dir, uint32_t frame_counter,
                                                                   uint8_t* dec_buffer );

/**
 * @brief Computes the LoRaWAN Join Request frame MIC field
 *
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] mic Computed MIC field
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_compute_join_mic( const uint8_t* buffer, uint16_t size,
                                                                    uint32_t* mic );

/**
 * @brief Process join-accept message. It decrypts the message and verifies the MIC
 *
 * @param [in] enc_buffer Encrypted buffer
 * @param [in] size Encrypted buffer size
 * @param [in] dec_buffer Decrypted buffer
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_process_join_accept( const uint8_t* enc_buffer, uint16_t size,
                                                                       uint8_t* dec_buffer );

/**
 * @brief Derives and store the sessions keys
 *
 * @param [in] join_nonce Sever nonce
 * @param [in] net_id Network Identifier
 * @param [in] dev_nonce Device nonce
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_derive_skeys( const uint8_t join_nonce[LORAWAN_JOIN_NONCE_SIZE],
                                                                const uint8_t net_id[LORAWAN_NET_ID_SIZE],
                                                                uint16_t      dev_nonce );

/**
 * @brief Verifies mic
 *
 * @param [in] buffer Data buffer to compute the integrity code
 * @param [in] size Data buffer size
 * @param [in] key_id Key identifier
 * @param [in] devaddr Device address
 * @param [in] dir Frame direction ( Uplink:0, Downlink:1 )
 * @param [in] fcnt Frame counter
 * @param [in] expected_mic Expected mic
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_verify_mic( const uint8_t* buffer, uint16_t size,
                                                              smtc_se_key_identifier_t key_id, uint32_t devaddr,
                                                              uint8_t dir, uint32_t fcnt, uint32_t expected_mic );

/**
 * @brief Compute and add mic to a buffer
 *
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] key_id Key identifier
 * @param [in] devaddr Device address
 * @param [in] dir Frame direction [0: uplink, 1: downlink]
 * @param [in] fcnt Frame counter
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_compute_and_add_mic( uint8_t* buffer, uint16_t size,
                                                                       smtc_se_key_identifier_t key_id,
                                                                       uint32_t devaddr, uint8_t dir, uint32_t fcnt );

/**
 * @brief Sets a key
 *
 * @param [in] key_id Key identifier
 * @param [in] key Key value (16 bytes), if its a multicast key it must be encrypted with McKEKey
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_set_key( smtc_se_key_identifier_t key_id, const uint8_t* key );

/**
 * @brief Derives a Multicast group key pair ( McAppSKey, McNwkSKey ) from McKey
 *
 * @param [in] addr_id Address identifier to select the multicast group
 * @param [in] mc_addr Multicast group address (4 bytes)
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_derive_multicast_session_keys( smtc_modem_crypto_addr_id_t addr_id,
                                                                                 uint32_t                    mc_addr );

/**
 * @brief Get the class B ping slot rand number that will be used to compute PingPeriod Offset
 *
 * @param [in] beacon_epoch_time The beacon epoch time
 * @param [in] dev_addr          Device Address
 * @param [out] rand             The output 16 bytes rand generated
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_get_class_b_rand( uint32_t beacon_epoch_time, uint32_t dev_addr,
                                                                    uint8_t rand[16] );

/**
 * @brief Encryption function for modem services (LFU and Stream)
 *
 * @param [in]  clear_buff Clear buffer
 * @param [in]  len        Buffer length
 * @param [in]  nonce      Nonce to be used
 * @param [out] enc_buff   Encrypted buffer
 * @return smtc_modem_crypto_return_code_t
 */
smtc_modem_crypto_return_code_t smtc_modem_crypto_service_encrypt( const uint8_t* clear_buff, uint16_t len,
                                                                   uint8_t nonce[14], uint8_t* enc_buff );

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_MODEM_CRYPTO_H__

/* --- EOF ------------------------------------------------------------------ */
