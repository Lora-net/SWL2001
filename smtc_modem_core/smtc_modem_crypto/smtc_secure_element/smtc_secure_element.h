/**
 * @file      smtc_secure_element.h
 *
 * @brief     Secure Element API
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SMTC_SECURE_ELEMENT_H
#define SMTC_SECURE_ELEMENT_H

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

/*!
 * Secure-element keys size in bytes
 */
#define SMTC_SE_KEY_SIZE 16

/*!
 * Secure-element EUI size in bytes
 */
#define SMTC_SE_EUI_SIZE 8

/*!
 * Secure-element pin size in bytes
 */
#define SMTC_SE_PIN_SIZE 4

/*!
 * Start value for multicast keys enumeration
 */
#define SMTC_SE_MULTICAST_KEYS 127

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Secure element return values.
 *
 * @enum smtc_se_return_code_t
 */

typedef enum smtc_secure_element_return_code_e
{
    SMTC_SE_RC_SUCCESS = 0,                         //!< No error occurred
    SMTC_SE_RC_FAIL_CMAC,                           //!< CMAC does not match
    SMTC_SE_RC_ERROR_NPE,                           //!<  Null pointer exception
    SMTC_SE_RC_ERROR_INVALID_KEY_ID,                //!< Invalid key identifier exception
    SMTC_SE_RC_ERROR_INVALID_LORAWAM_SPEC_VERSION,  //!< Invalid LoRaWAN specification version
    SMTC_SE_RC_ERROR_BUF_SIZE,                      //!< Incompatible buffer size
    SMTC_SE_RC_ERROR,                               //!< Undefined Error occurred
    SMTC_SE_RC_FAIL_ENCRYPT,                        //!< Failed to encrypt
} smtc_se_return_code_t;

/**
 * @brief join-request / rejoin type identifier
 *
 * @enum smtc_se_join_req_identifier_t
 */
typedef enum smtc_se_join_req_identifier_e
{
    SMTC_SE_REJOIN_REQ_0 = 0x00,  //!< Rejoin type 0
    SMTC_SE_REJOIN_REQ_1 = 0x01,  //!< Rejoin type 1
    SMTC_SE_REJOIN_REQ_2 = 0x02,  //!< Rejoin type 2
    SMTC_SE_JOIN_REQ     = 0xFF,  //!< Join-request
} smtc_se_join_req_identifier_t;

/*!
 * LoRaMac Key identifier
 *
 * @enum smtc_se_key_identifier_t
 */
typedef enum smtc_se_key_identifier_e
{
    SMTC_SE_APP_KEY = 0,                         //!< Application root key
    SMTC_SE_NWK_KEY,                             //!< Network root key
    SMTC_SE_J_S_INT_KEY,                         //!< Join session integrity key
    SMTC_SE_J_S_ENC_KEY,                         //!< Join session encryption key
    SMTC_SE_F_NWK_S_INT_KEY,                     //!< Forwarding Network session integrity key
    SMTC_SE_S_NWK_S_INT_KEY,                     //!< Serving Network session integrity key
    SMTC_SE_NWK_S_ENC_KEY,                       //!< Network session encryption key
    SMTC_SE_APP_S_KEY,                           //!< Application session key
    SMTC_SE_MC_ROOT_KEY,                         //!< Multicast root key
    SMTC_SE_MC_KE_KEY = SMTC_SE_MULTICAST_KEYS,  //!< Multicast key encryption key
    SMTC_SE_MC_KEY_0,                            //!< Multicast root key index 0
    SMTC_SE_MC_APP_S_KEY_0,                      //!< Multicast Application session key index 0
    SMTC_SE_MC_NWK_S_KEY_0,                      //!< Multicast Network session key index 0
    SMTC_SE_MC_KEY_1,                            //!< Multicast root key index 1
    SMTC_SE_MC_APP_S_KEY_1,                      //!< Multicast Application session key index 1
    SMTC_SE_MC_NWK_S_KEY_1,                      //!< Multicast Network session key index 1
    SMTC_SE_MC_KEY_2,                            //!< Multicast root key index 2
    SMTC_SE_MC_APP_S_KEY_2,                      //!< Multicast Application session key index 2
    SMTC_SE_MC_NWK_S_KEY_2,                      //!< Multicast Network session key index 2
    SMTC_SE_MC_KEY_3,                            //!< Multicast root key index 3
    SMTC_SE_MC_APP_S_KEY_3,                      //!< Multicast Application session key index 3
    SMTC_SE_MC_NWK_S_KEY_3,                      //!< Multicast Network session key index 3
    SMTC_SE_SLOT_RAND_ZERO_KEY,                  //!< Zero key for slot randomization in class B
    SMTC_SE_NO_KEY,                              //!< No Key
} smtc_se_key_identifier_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialization of Secure Element driver
 *
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_init( void );

/**
 * @brief Sets a key
 *
 * @param [in] key_id Key identifier
 * @param [in] key Key value
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_set_key( smtc_se_key_identifier_t key_id,
                                                   const uint8_t            key[SMTC_SE_KEY_SIZE] );

/**
 * @brief Computes a CMAC of a message using provided initial Bx block
 *
 * @param [in] mic_bx_buffer Buffer containing the initial Bx block
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] key_id Key identifier to determine the AES key to be used
 * @param [in] cmac Computed cmac
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_compute_aes_cmac( uint8_t* mic_bx_buffer, const uint8_t* buffer,
                                                            uint16_t size, smtc_se_key_identifier_t key_id,
                                                            uint32_t* cmac );

/**
 * @brief Verifies a CMAC (computes and compare with expected cmac)
 *
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] expected_cmac Expected cmac
 * @param [in] key_id Key identifier to determine the AES key to be used
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_verify_aes_cmac( uint8_t* buffer, uint16_t size, uint32_t expected_cmac,
                                                           smtc_se_key_identifier_t key_id );

/**
 * @brief Encrypt a buffer
 *
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size - this value shall be a multiple of 16
 * @param [in] key_id Key identifier to determine the AES key to be used
 * @param [in] enc_buffer Encrypted buffer
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_aes_encrypt( const uint8_t* buffer, uint16_t size,
                                                       smtc_se_key_identifier_t key_id, uint8_t* enc_buffer );

/**
 * @brief Derives and store a key
 *
 * @param [in] input Input data from which the key is derived ( 16 bytes )
 * @param [in] rootkey_id Key identifier of the root key to use to perform the derivation
 * @param [in] targetkey_id Key identifier of the key which will be derived
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_derive_and_store_key( uint8_t* input, smtc_se_key_identifier_t rootkey_id,
                                                                smtc_se_key_identifier_t targetkey_id );

/**
 * @brief Process join_accept message.
 *
 * @param [in] join_req_type LoRaMac join-request / rejoin type identifier
 * @param [in] joineui LoRaWAN Join server EUI
 * @param [in] dev_nonce Device nonce
 * @param [in] enc_join_accept Received encrypted join_accept message
 * @param [in] enc_join_accept_size Received encrypted join_accept message Size
 * @param [in] dec_join_accept Decrypted and validated join_accept message
 * @param [in] version_minor Detected LoRaWAN specification version minor field.
 *                          - 0 -> LoRaWAN 1.0.x
 *                          - 1 -> LoRaWAN 1.1.x
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_process_join_accept( smtc_se_join_req_identifier_t join_req_type,
                                                               uint8_t joineui[SMTC_SE_EUI_SIZE], uint16_t dev_nonce,
                                                               const uint8_t* enc_join_accept,
                                                               uint8_t enc_join_accept_size, uint8_t* dec_join_accept,
                                                               uint8_t* version_minor );

/**
 * @brief Sets the DevEUI
 *
 * @param [in] deveui LoRaWAN devEUI
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_set_deveui( const uint8_t deveui[SMTC_SE_EUI_SIZE] );

/**
 * @brief Gets the DevEUI
 *
 * @param [out] deveui The current DevEUI
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_get_deveui( uint8_t deveui[SMTC_SE_EUI_SIZE] );

/**
 * @brief Sets the JoinEUI
 *
 * @param [in] joineui LoRaWAN JoinEUI
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_set_joineui( const uint8_t joineui[SMTC_SE_EUI_SIZE] );

/**
 * @brief Gets the JoinEUI
 *
 * @param [out] joineui The current JoinEUI
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_get_joineui( uint8_t joineui[SMTC_SE_EUI_SIZE] );

/**
 * @brief Sets the pin
 *
 * @param [in] pin The pin code
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_set_pin( const uint8_t pin[SMTC_SE_PIN_SIZE] );

/**
 * @brief Gets the pin
 *
 * @param [out] pin The current pin code
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_get_pin( uint8_t pin[SMTC_SE_PIN_SIZE] );

/**
 * @brief Store the current secure element context into NVM
 *
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_store_context( void );

/**
 * @brief Restore the stored secure element context from NVM to RAM
 *
 * @return Secure element return code as defined in @ref smtc_se_return_code_t
 */
smtc_se_return_code_t smtc_secure_element_restore_context( void );

#ifdef __cplusplus
}
#endif

#endif  //  SMTC_SECURE_ELEMENT_H__
