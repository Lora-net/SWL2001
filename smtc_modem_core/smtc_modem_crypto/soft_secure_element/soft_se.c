/**
 * @file      soft_se.c
 *
 * @brief     Secure Element software implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_secure_element.h"

#include "aes.h"
#include "cmac.h"

#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#include <string.h>  //for memset, memcpy

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * Number of keys supported in soft secure element
 */
#define SOFT_SE_NUMBER_OF_KEYS 23

/*!
 * JoinAccept frame maximum size
 */
#define JOIN_ACCEPT_FRAME_MAX_SIZE 33

/*!
 * Lorawan MIC size
 */
#define LORWAN_MIC_FIELD_SIZE 4

/*!
 * Lorawan MHDR SIZE
 */
#define LORAMAC_MHDR_FIELD_SIZE 1

#define SOFT_SE_KEY_LIST                                                                                             \
    {                                                                                                                \
        {                                                                                                            \
            /*!                                                                                                      \
             * Application root key                                                                                  \
             * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_GEN_APP_KEY                                         \
             */                                                                                                      \
            .key_id    = SMTC_SE_APP_KEY,                                                                            \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Network root key                                                                                      \
             * WARNING: FOR 1.0.x DEVICES IT IS THE \ref LORAWAN_APP_KEY                                             \
             */                                                                                                      \
            .key_id    = SMTC_SE_NWK_KEY,                                                                            \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Join session integrity key (Dynamically updated)                                                      \
             * WARNING: NOT USED FOR 1.0.x DEVICES                                                                   \
             */                                                                                                      \
            .key_id    = SMTC_SE_J_S_INT_KEY,                                                                        \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Join session encryption key (Dynamically updated)                                                     \
             * WARNING: NOT USED FOR 1.0.x DEVICES                                                                   \
             */                                                                                                      \
            .key_id    = SMTC_SE_J_S_ENC_KEY,                                                                        \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Forwarding Network session integrity key                                                              \
             * WARNING: NWK_S_KEY FOR 1.0.x DEVICES                                                                  \
             */                                                                                                      \
            .key_id    = SMTC_SE_F_NWK_S_INT_KEY,                                                                    \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Serving Network session integrity key                                                                 \
             * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY                 \
             */                                                                                                      \
            .key_id    = SMTC_SE_S_NWK_S_INT_KEY,                                                                    \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Network session encryption key                                                                        \
             * WARNING: NOT USED FOR 1.0.x DEVICES. MUST BE THE SAME AS \ref LORAWAN_F_NWK_S_INT_KEY                 \
             */                                                                                                      \
            .key_id    = SMTC_SE_NWK_S_ENC_KEY,                                                                      \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Application session key                                                                               \
             */                                                                                                      \
            .key_id    = SMTC_SE_APP_S_KEY,                                                                          \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast root key (Dynamically updated)                                                              \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_ROOT_KEY,                                                                        \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast key encryption key (Dynamically updated)                                                    \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_KE_KEY,                                                                          \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #0 root key (Dynamically updated)                                                     \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_KEY_0,                                                                           \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #0 application session key (Dynamically updated)                                      \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_APP_S_KEY_0,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #0 network session key (Dynamically updated)                                          \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_NWK_S_KEY_0,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #1 root key (Dynamically updated)                                                     \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_KEY_1,                                                                           \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #1 application session key (Dynamically updated)                                      \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_APP_S_KEY_1,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #1 network session key (Dynamically updated)                                          \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_NWK_S_KEY_1,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #2 root key (Dynamically updated)                                                     \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_KEY_2,                                                                           \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #2 application session key (Dynamically updated)                                      \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_APP_S_KEY_2,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #2 network session key (Dynamically updated)                                          \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_NWK_S_KEY_2,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #3 root key (Dynamically updated)                                                     \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_KEY_3,                                                                           \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #3 application session key (Dynamically updated)                                      \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_APP_S_KEY_3,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * Multicast group #3 network session key (Dynamically updated)                                          \
             */                                                                                                      \
            .key_id    = SMTC_SE_MC_NWK_S_KEY_3,                                                                     \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
        {                                                                                                            \
            /*!                                                                                                      \
             * All zeros key. (ClassB usage)(constant)                                                               \
             */                                                                                                      \
            .key_id    = SMTC_SE_SLOT_RAND_ZERO_KEY,                                                                 \
            .key_value = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                           0x00 },                                                                                   \
        },                                                                                                           \
    },

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Key structure definition for the soft-se
 *
 * @struct soft_se_key_t
 */
typedef struct soft_se_key_s
{
    smtc_se_key_identifier_t key_id;                       //!< Key identifier
    uint8_t                  key_value[SMTC_SE_KEY_SIZE];  //!< Key value
} soft_se_key_t;

/**
 * @brief Structure for data needed by soft secure element
 *
 * @struct soft_se_data_t
 */
typedef struct soft_se_data_s
{
    uint8_t       deveui[SMTC_SE_EUI_SIZE];          //!< DevEUI storage
    uint8_t       joineui[SMTC_SE_EUI_SIZE];         //!< Join EUI storage
    uint8_t       pin[SMTC_SE_PIN_SIZE];             //!< pin storage
    soft_se_key_t key_list[SOFT_SE_NUMBER_OF_KEYS];  //!< The key list
} soft_se_data_t;

/**
 * @brief Struture for soft secure element context saving in NVM
 *
 * @struct soft_se_context_nvm_t
 */
typedef struct soft_se_context_nvm_s
{
    soft_se_data_t data;
    uint32_t       crc;
} soft_se_context_nvm_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static soft_se_data_t soft_se_data = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * Local functions
 */

/**
 * @brief Gets key item from key list.
 *
 * @param [in] key_id Key identifier
 * @param [in] key_item Key item reference
 * @return smtc_se_return_code_t
 */
static smtc_se_return_code_t get_key_by_id( smtc_se_key_identifier_t key_id, soft_se_key_t** key_item );

/**
 * @brief Computes a CMAC of a message using provided initial Bx block
 *
 * cmac = aes128_cmac(key_id, blocks[i].Buffer)
 *
 * @param [in] mic_bx_buffer Buffer containing the initial Bx block
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] key_id Key identifier to determine the AES key to be used
 * @param [out] cmac Computed cmac
 * @return smtc_se_return_code_t
 */
static smtc_se_return_code_t compute_cmac( uint8_t* mic_bx_buffer, const uint8_t* buffer, uint16_t size,
                                           smtc_se_key_identifier_t key_id, uint32_t* cmac );

/**
 * @brief CRC function for soft se context security
 *
 * @param [in] buf  Data buffer
 * @param [in] len Length of the data
 * @return uint32_t
 */
uint32_t soft_ce_crc( const uint8_t* buf, int len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_se_return_code_t smtc_secure_element_init( void )
{
    soft_se_data_t local_data = { .deveui   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                                  .joineui  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                                  .pin      = { 0x00, 0x00, 0x00, 0x00 },
                                  .key_list = SOFT_SE_KEY_LIST };
    // init soft secure element data euis and pin to 0 and key_list with empty lut
    memcpy( ( uint8_t* ) &soft_se_data, ( uint8_t* ) &local_data, sizeof( local_data ) );

    SMTC_MODEM_HAL_TRACE_INFO( "Use soft secure element for cryptographic functionalities\n" );

    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_set_key( smtc_se_key_identifier_t key_id,
                                                   const uint8_t            key[SMTC_SE_KEY_SIZE] )
{
    if( key == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    for( uint8_t i = 0; i < SOFT_SE_NUMBER_OF_KEYS; i++ )
    {
        if( soft_se_data.key_list[i].key_id == key_id )
        {
            if( ( key_id == SMTC_SE_MC_KEY_0 ) || ( key_id == SMTC_SE_MC_KEY_1 ) || ( key_id == SMTC_SE_MC_KEY_2 ) ||
                ( key_id == SMTC_SE_MC_KEY_3 ) )
            {  // Decrypt the key if its a Mckey
                smtc_se_return_code_t rc                = SMTC_SE_RC_ERROR;
                uint8_t               decrypted_key[16] = { 0 };

                rc = smtc_secure_element_aes_encrypt( key, 16, SMTC_SE_MC_KE_KEY, decrypted_key );

                memcpy( soft_se_data.key_list[i].key_value, decrypted_key, SMTC_SE_KEY_SIZE );
                return rc;
            }
            else
            {
                memcpy( &( soft_se_data.key_list[i].key_value ), key, SMTC_SE_KEY_SIZE );
                return SMTC_SE_RC_SUCCESS;
            }
        }
    }

    return SMTC_SE_RC_ERROR_INVALID_KEY_ID;
}

smtc_se_return_code_t smtc_secure_element_compute_aes_cmac( uint8_t* mic_bx_buffer, const uint8_t* buffer,
                                                            uint16_t size, smtc_se_key_identifier_t key_id,
                                                            uint32_t* cmac )
{
    if( key_id >= SMTC_SE_SLOT_RAND_ZERO_KEY )
    {
        return SMTC_SE_RC_ERROR_INVALID_KEY_ID;
    }

    return compute_cmac( mic_bx_buffer, buffer, size, key_id, cmac );
}

smtc_se_return_code_t smtc_secure_element_verify_aes_cmac( uint8_t* buffer, uint16_t size, uint32_t expected_cmac,
                                                           smtc_se_key_identifier_t key_id )
{
    if( buffer == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    smtc_se_return_code_t rc        = SMTC_SE_RC_ERROR;
    uint32_t              comp_cmac = 0;

    rc = compute_cmac( NULL, buffer, size, key_id, &comp_cmac );

    if( rc != SMTC_SE_RC_SUCCESS )
    {
        return rc;
    }

    if( expected_cmac != comp_cmac )
    {
        rc = SMTC_SE_RC_FAIL_CMAC;
    }

    return rc;
}

smtc_se_return_code_t smtc_secure_element_aes_encrypt( const uint8_t* buffer, uint16_t size,
                                                       smtc_se_key_identifier_t key_id, uint8_t* enc_buffer )
{
    if( buffer == NULL || enc_buffer == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    // Check if the size is divisible by 16,
    if( ( size % 16 ) != 0 )
    {
        return SMTC_SE_RC_ERROR_BUF_SIZE;
    }

    aes_context aes_ctx;
    memset( &aes_ctx, 0, sizeof( aes_context ) );

    soft_se_key_t*        key_item;
    smtc_se_return_code_t rc = get_key_by_id( key_id, &key_item );

    if( rc == SMTC_SE_RC_SUCCESS )
    {
        aes_set_key( key_item->key_value, 16, &aes_ctx );

        uint8_t block = 0;

        while( size != 0 )
        {
            aes_encrypt( &buffer[block], &enc_buffer[block], &aes_ctx );
            block = block + 16;
            size  = size - 16;
        }
    }
    return rc;
}

smtc_se_return_code_t smtc_secure_element_derive_and_store_key( uint8_t* input, smtc_se_key_identifier_t rootkey_id,
                                                                smtc_se_key_identifier_t targetkey_id )
{
    if( input == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    smtc_se_return_code_t rc      = SMTC_SE_RC_ERROR;
    uint8_t               key[16] = { 0 };

    // In case of SMTC_SE_MC_KE_KEY, only McRootKey can be used as root key
    if( targetkey_id == SMTC_SE_MC_KE_KEY )
    {
        if( rootkey_id != SMTC_SE_MC_ROOT_KEY )
        {
            return SMTC_SE_RC_ERROR_INVALID_KEY_ID;
        }
    }

    // Derive key
    rc = smtc_secure_element_aes_encrypt( input, 16, rootkey_id, key );
    if( rc != SMTC_SE_RC_SUCCESS )
    {
        return rc;
    }

    // Store key
    rc = smtc_secure_element_set_key( targetkey_id, key );
    if( rc != SMTC_SE_RC_SUCCESS )
    {
        return rc;
    }

    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_process_join_accept( smtc_se_join_req_identifier_t join_req_type,
                                                               uint8_t joineui[SMTC_SE_EUI_SIZE], uint16_t dev_nonce,
                                                               const uint8_t* enc_join_accept,
                                                               uint8_t enc_join_accept_size, uint8_t* dec_join_accept,
                                                               uint8_t* version_minor )
{
    if( ( enc_join_accept == NULL ) || ( dec_join_accept == NULL ) || ( version_minor == NULL ) )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    // Check that frame size isn't bigger than a JoinAccept with CFList size
    if( enc_join_accept_size > JOIN_ACCEPT_FRAME_MAX_SIZE )
    {
        return SMTC_SE_RC_ERROR_BUF_SIZE;
    }

    // Determine decryption key
    smtc_se_key_identifier_t enckey_id = SMTC_SE_NWK_KEY;

    if( join_req_type != SMTC_SE_JOIN_REQ )
    {
        enckey_id = SMTC_SE_J_S_ENC_KEY;
    }

    memcpy( dec_join_accept, enc_join_accept, enc_join_accept_size );

    // Decrypt JoinAccept, skip MHDR
    if( smtc_secure_element_aes_encrypt( enc_join_accept + LORAMAC_MHDR_FIELD_SIZE,
                                         enc_join_accept_size - LORAMAC_MHDR_FIELD_SIZE, enckey_id,
                                         dec_join_accept + LORAMAC_MHDR_FIELD_SIZE ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_SE_RC_FAIL_ENCRYPT;
    }

    *version_minor = ( ( dec_join_accept[11] & 0x80 ) == 0x80 ) ? 1 : 0;

    uint32_t mic = 0;

    mic = ( ( uint32_t ) dec_join_accept[enc_join_accept_size - LORWAN_MIC_FIELD_SIZE] << 0 );
    mic |= ( ( uint32_t ) dec_join_accept[enc_join_accept_size - LORWAN_MIC_FIELD_SIZE + 1] << 8 );
    mic |= ( ( uint32_t ) dec_join_accept[enc_join_accept_size - LORWAN_MIC_FIELD_SIZE + 2] << 16 );
    mic |= ( ( uint32_t ) dec_join_accept[enc_join_accept_size - LORWAN_MIC_FIELD_SIZE + 3] << 24 );

    //  - Header buffer to be used for MIC computation
    //        - LoRaWAN 1.0.x : micHeader = [MHDR(1)]
    //        - LoRaWAN 1.1.x : micHeader = [JoinReqType(1), JoinEUI(8), DevNonce(2), MHDR(1)]

    // Verify mic
    if( *version_minor == 0 )
    {
        // For LoRaWAN 1.0.x
        //   cmac = aes128_cmac(NwkKey, MHDR |  JoinNonce | NetID | DevAddr | DLSettings | RxDelay | CFList |
        //   CFListType)
        if( smtc_secure_element_verify_aes_cmac( dec_join_accept, ( enc_join_accept_size - LORWAN_MIC_FIELD_SIZE ), mic,
                                                 SMTC_SE_NWK_KEY ) != SMTC_SE_RC_SUCCESS )
        {
            return SMTC_SE_RC_FAIL_CMAC;
        }
    }
    else
    {
        return SMTC_SE_RC_ERROR_INVALID_LORAWAM_SPEC_VERSION;
    }

    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_set_deveui( const uint8_t deveui[SMTC_SE_EUI_SIZE] )
{
    if( deveui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( soft_se_data.deveui, deveui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_get_deveui( uint8_t deveui[SMTC_SE_EUI_SIZE] )
{
    if( deveui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( deveui, soft_se_data.deveui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_set_joineui( const uint8_t joineui[SMTC_SE_EUI_SIZE] )
{
    if( joineui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( soft_se_data.joineui, joineui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_get_joineui( uint8_t joineui[SMTC_SE_EUI_SIZE] )
{
    if( joineui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( joineui, soft_se_data.joineui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_set_pin( const uint8_t pin[SMTC_SE_PIN_SIZE] )
{
    if( pin == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    memcpy( soft_se_data.pin, pin, SMTC_SE_PIN_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_get_pin( uint8_t pin[SMTC_SE_PIN_SIZE] )
{
    if( pin == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( pin, soft_se_data.pin, SMTC_SE_PIN_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_store_context( void )
{
    soft_se_context_nvm_t ctx = {
        .data = soft_se_data,
    };
    ctx.crc = soft_ce_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );

    smtc_modem_hal_context_store( CONTEXT_SECURE_ELEMENT, ( uint8_t* ) &ctx, sizeof( ctx ) );
    smtc_secure_element_restore_context( );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_restore_context( void )
{
    soft_se_context_nvm_t ctx;
    smtc_modem_hal_context_restore( CONTEXT_SECURE_ELEMENT, ( uint8_t* ) &ctx, sizeof( ctx ) );
    if( soft_ce_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 ) == ctx.crc )
    {
        soft_se_data = ctx.data;
        return SMTC_SE_RC_SUCCESS;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Restore of Secure Element context fails => Return to init values\n" );
        soft_se_data_t local_data = { .deveui   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                                      .joineui  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                                      .pin      = { 0x00, 0x00, 0x00, 0x00 },
                                      .key_list = SOFT_SE_KEY_LIST };
        // init soft secure element data euis and pin to 0 and key_list with empty lut
        memcpy( ( uint8_t* ) &soft_se_data, ( uint8_t* ) &local_data, sizeof( local_data ) );
        return SMTC_SE_RC_ERROR;
    }
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static smtc_se_return_code_t get_key_by_id( smtc_se_key_identifier_t key_id, soft_se_key_t** key_item )
{
    for( uint8_t i = 0; i < SOFT_SE_NUMBER_OF_KEYS; i++ )
    {
        if( soft_se_data.key_list[i].key_id == key_id )
        {
            *key_item = &( soft_se_data.key_list[i] );
            return SMTC_SE_RC_SUCCESS;
        }
    }
    return SMTC_SE_RC_ERROR_INVALID_KEY_ID;
}

static smtc_se_return_code_t compute_cmac( uint8_t* mic_bx_buffer, const uint8_t* buffer, uint16_t size,
                                           smtc_se_key_identifier_t key_id, uint32_t* cmac )
{
    if( ( buffer == NULL ) || ( cmac == NULL ) )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    uint8_t      local_cmac[16];
    AES_CMAC_CTX aes_cmac_ctx[1];

    AES_CMAC_Init( aes_cmac_ctx );

    soft_se_key_t* key_item;

    smtc_se_return_code_t rc = get_key_by_id( key_id, &key_item );

    if( rc == SMTC_SE_RC_SUCCESS )
    {
        AES_CMAC_SetKey( aes_cmac_ctx, key_item->key_value );

        if( mic_bx_buffer != NULL )
        {
            AES_CMAC_Update( aes_cmac_ctx, mic_bx_buffer, 16 );
        }

        AES_CMAC_Update( aes_cmac_ctx, buffer, size );

        AES_CMAC_Final( local_cmac, aes_cmac_ctx );

        // Bring into the required format
        *cmac = ( uint32_t )( ( uint32_t ) local_cmac[3] << 24 | ( uint32_t ) local_cmac[2] << 16 |
                              ( uint32_t ) local_cmac[1] << 8 | ( uint32_t ) local_cmac[0] );
    }

    return rc;
}

uint32_t soft_ce_crc( const uint8_t* buf, int len )
{
    uint32_t crc = 0xFFFFFFFF;
    while( len-- > 0 )
    {
        crc = crc ^ *buf++;
        for( int i = 0; i < 8; i++ )
        {
            uint32_t mask = -( crc & 1 );
            crc           = ( crc >> 1 ) ^ ( 0xEDB88320 & mask );
        }
    }
    return ~crc;
}

/* --- EOF ------------------------------------------------------------------ */
