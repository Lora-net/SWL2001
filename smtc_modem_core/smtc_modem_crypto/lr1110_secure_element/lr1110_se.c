/**
 * @file      lr1110_se.c
 *
 * @brief     LR1110 Secure Element hardware implementation
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

#include "smtc_modem_api_lr1110_system.h"
#include "smtc_modem_api_lr1110_crypto_engine.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#include "modem_context.h"

#include <string.h>  //for memset

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define MIC_BLOCK_BX_SIZE 16

/**
 * Maximum size of the message that can be handled by the crypto operations
 */
#define CRYPTO_MAXMESSAGE_SIZE 256

/**
 * Maximum size of the buffer for crypto operations
 */
#define CRYPTO_BUFFER_SIZE CRYPTO_MAXMESSAGE_SIZE + MIC_BLOCK_BX_SIZE

/**
 * JoinAccept frame maximum size
 */
#define JOIN_ACCEPT_FRAME_MAX_SIZE 33

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Structure for data needed by lr1110 secure element
 *
 * @struct lr1110_se_data_t
 */
typedef struct lr1110_se_data_s
{
    uint8_t deveui[SMTC_SE_EUI_SIZE];   //!< DevEUI storage
    uint8_t joineui[SMTC_SE_EUI_SIZE];  //!< Join EUI storage
    uint8_t pin[SMTC_SE_PIN_SIZE];      //!< pin storage
} lr1110_se_data_t;

/**
 * @brief Struture for lr1110 secure element context saving in NVM
 *
 * @struct lr1110_se_context_nvm_t
 */
typedef struct lr1110_se_context_nvm_s
{
    lr1110_se_data_t data;
    uint32_t         crc;
} lr1110_se_context_nvm_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr1110_se_data_t lr1110_se_data;
static const void*      lr1110_ctx;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief  Converts key ids from Secure Element abstraction to LR1110
 *
 * @param [in] key_id
 * @return lr1110_crypto_keys_idx_t
 */
static lr1110_crypto_keys_idx_t convert_key_id_from_se_to_lr1110( smtc_se_key_identifier_t key_id );

/**
 * @brief CRC function for lr1110 se context security
 *
 * @param [in] buf  Data buffer
 * @param [in] len Length of the data
 * @return uint32_t
 */
uint32_t lr1110_se_crc( const uint8_t* buf, int len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_se_return_code_t smtc_secure_element_init( void )
{
    SMTC_MODEM_HAL_TRACE_INFO( "Use lr1110 crypto engine for cryptographic functionalities\n" );

    lr1110_crypto_status_t lr1110_crypto_status = LR1110_CRYPTO_STATUS_ERROR;

    // Initialize data structure to 0
    memset( &lr1110_se_data, 0, sizeof( lr1110_se_data_t ) );

    // get radio context
    lr1110_ctx = modem_context_get_modem_radio_ctx( );

    // Restore lr1110 crypto data from flash memory into RAM
    smtc_modem_lr1110_crypto_restore_from_flash( lr1110_ctx, &lr1110_crypto_status );

#if defined( USE_PRE_PROVISIONED_FEATURES )
    SMTC_MODEM_HAL_TRACE_WARNING( "Use lr1110 preprovisioned EUIs and keys\n" );

    // Read LR1110 pre-provisioned identity
    smtc_modem_lr1110_system_read_uid( lr1110_ctx, lr1110_se_data.deveui );
    smtc_modem_lr1110_system_read_join_eui( lr1110_ctx, lr1110_se_data.joineui );
    smtc_modem_lr1110_system_read_pin( lr1110_ctx, lr1110_se_data.pin );
#endif
    // Return codes are in line between secure element definition and lr1110 internal definition
    return ( smtc_se_return_code_t ) lr1110_crypto_status;
}

smtc_se_return_code_t smtc_secure_element_set_key( smtc_se_key_identifier_t key_id,
                                                   const uint8_t            key[SMTC_SE_KEY_SIZE] )
{
    if( key == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    smtc_se_return_code_t status = SMTC_SE_RC_ERROR;

    if( ( key_id == SMTC_SE_MC_KEY_0 ) || ( key_id == SMTC_SE_MC_KEY_1 ) || ( key_id == SMTC_SE_MC_KEY_2 ) ||
        ( key_id == SMTC_SE_MC_KEY_3 ) )
    {  // Decrypt the key if its a Mckey

        smtc_modem_lr1110_crypto_derive_key( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                             convert_key_id_from_se_to_lr1110( SMTC_SE_MC_KE_KEY ),
                                             convert_key_id_from_se_to_lr1110( key_id ), key );

        if( status == SMTC_SE_RC_SUCCESS )
        {
            smtc_modem_lr1110_crypto_store_to_flash( lr1110_ctx, ( lr1110_crypto_status_t* ) &status );
        }
    }
    else
    {
        smtc_modem_lr1110_crypto_set_key( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                          convert_key_id_from_se_to_lr1110( key_id ), key );
        if( status == SMTC_SE_RC_SUCCESS )
        {
            smtc_modem_lr1110_crypto_store_to_flash( lr1110_ctx, ( lr1110_crypto_status_t* ) &status );
        }
    }
    return status;
}

smtc_se_return_code_t smtc_secure_element_compute_aes_cmac( uint8_t* mic_bx_buffer, const uint8_t* buffer,
                                                            uint16_t size, smtc_se_key_identifier_t key_id,
                                                            uint32_t* cmac )
{
    smtc_se_return_code_t status = SMTC_SE_RC_ERROR;
    // uint8_t*              local_buffer = buffer;

    if( ( buffer == NULL ) || ( cmac == NULL ) )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    if( mic_bx_buffer != NULL )
    {
        uint8_t  mic_buff[CRYPTO_BUFFER_SIZE];
        uint16_t cur_size = size + MIC_BLOCK_BX_SIZE;

        memset( mic_buff, 0, CRYPTO_BUFFER_SIZE );

        memcpy( mic_buff, mic_bx_buffer, MIC_BLOCK_BX_SIZE );
        memcpy( ( mic_buff + MIC_BLOCK_BX_SIZE ), buffer, size );
        // local_buffer = mic_buff;
        smtc_modem_lr1110_crypto_compute_aes_cmac( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                                   convert_key_id_from_se_to_lr1110( key_id ), mic_buff, cur_size,
                                                   ( uint8_t* ) cmac );
    }
    else
    {
        smtc_modem_lr1110_crypto_compute_aes_cmac( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                                   convert_key_id_from_se_to_lr1110( key_id ), buffer, size,
                                                   ( uint8_t* ) cmac );
    }
    return status;
}

smtc_se_return_code_t smtc_secure_element_verify_aes_cmac( uint8_t* buffer, uint16_t size, uint32_t expected_cmac,
                                                           smtc_se_key_identifier_t key_id )
{
    smtc_se_return_code_t status = SMTC_SE_RC_ERROR;

    if( buffer == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    smtc_modem_lr1110_crypto_verify_aes_cmac( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                              convert_key_id_from_se_to_lr1110( key_id ), buffer, size,
                                              ( uint8_t* ) &expected_cmac );
    return status;
}

smtc_se_return_code_t smtc_secure_element_aes_encrypt( const uint8_t* buffer, uint16_t size,
                                                       smtc_se_key_identifier_t key_id, uint8_t* enc_buffer )
{
    smtc_se_return_code_t status = SMTC_SE_RC_ERROR;

    if( ( buffer == NULL ) || ( enc_buffer == NULL ) )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    smtc_modem_lr1110_crypto_aes_encrypt_01( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                             convert_key_id_from_se_to_lr1110( key_id ), buffer, size, enc_buffer );
    return status;
}

smtc_se_return_code_t smtc_secure_element_derive_and_store_key( uint8_t* input, smtc_se_key_identifier_t rootkey_id,
                                                                smtc_se_key_identifier_t targetkey_id )
{
    smtc_se_return_code_t status = SMTC_SE_RC_ERROR;

    if( input == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    smtc_modem_lr1110_crypto_derive_key( lr1110_ctx, ( lr1110_crypto_status_t* ) &status,
                                         convert_key_id_from_se_to_lr1110( rootkey_id ),
                                         convert_key_id_from_se_to_lr1110( targetkey_id ), input );

    smtc_modem_lr1110_crypto_store_to_flash( lr1110_ctx, ( lr1110_crypto_status_t* ) &status );
    return status;
}

smtc_se_return_code_t smtc_secure_element_process_join_accept( smtc_se_join_req_identifier_t join_req_type,
                                                               uint8_t* joineui, uint16_t dev_nonce,
                                                               const uint8_t* enc_join_accept,
                                                               uint8_t enc_join_accept_size, uint8_t* dec_join_accept,
                                                               uint8_t* version_minor )
{
    smtc_se_return_code_t status = SMTC_SE_RC_ERROR;

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

    //  - Header buffer to be used for MIC computation
    //        - LoRaWAN 1.0.x : micHeader = [MHDR(1)]
    //        - LoRaWAN 1.1.x : micHeader = [JoinReqType(1), JoinEUI(8), DevNonce(2), MHDR(1)]

    // Try first to process LoRaWAN 1.0.x JoinAccept
    uint8_t mic_header_10x[1] = { 0x20 };

    //   cmac = aes128_cmac(NwkKey, MHDR |  JoinNonce | NetID | DevAddr | DLSettings | RxDelay | CFList |
    //   CFListType)
    smtc_modem_lr1110_crypto_process_join_accept(
        lr1110_ctx, ( lr1110_crypto_status_t* ) &status, convert_key_id_from_se_to_lr1110( enckey_id ),
        convert_key_id_from_se_to_lr1110( SMTC_SE_NWK_KEY ), ( lr1110_crypto_lorawan_version_t ) 0, mic_header_10x,
        enc_join_accept + 1, enc_join_accept_size - 1, dec_join_accept + 1 );

    if( status == SMTC_SE_RC_SUCCESS )
    {
        *version_minor = ( ( dec_join_accept[11] & 0x80 ) == 0x80 ) ? 1 : 0;
        if( *version_minor == 0 )
        {
            // Network server is operating according to LoRaWAN 1.0.x
            return SMTC_SE_RC_SUCCESS;
        }
    }
    return status;
}

smtc_se_return_code_t smtc_secure_element_set_deveui( const uint8_t deveui[SMTC_SE_EUI_SIZE] )
{
    if( deveui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( lr1110_se_data.deveui, deveui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_get_deveui( uint8_t deveui[SMTC_SE_EUI_SIZE] )
{
    if( deveui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( deveui, lr1110_se_data.deveui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_set_joineui( const uint8_t joineui[SMTC_SE_EUI_SIZE] )
{
    if( joineui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( lr1110_se_data.joineui, joineui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_get_joineui( uint8_t joineui[SMTC_SE_EUI_SIZE] )
{
    if( joineui == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( joineui, lr1110_se_data.joineui, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_set_pin( const uint8_t pin[SMTC_SE_PIN_SIZE] )
{
    if( pin == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }
    memcpy( lr1110_se_data.pin, pin, SMTC_SE_PIN_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_get_pin( uint8_t pin[SMTC_SE_PIN_SIZE] )
{
    if( pin == NULL )
    {
        return SMTC_SE_RC_ERROR_NPE;
    }

    memcpy( pin, lr1110_se_data.pin, SMTC_SE_EUI_SIZE );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_store_context( void )
{
    lr1110_se_context_nvm_t ctx = {
        .data = lr1110_se_data,
    };
    ctx.crc = lr1110_se_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 );

    smtc_modem_hal_context_store( CONTEXT_SECURE_ELEMENT, ( uint8_t* ) &ctx, sizeof( ctx ) );
    smtc_secure_element_restore_context( );
    return SMTC_SE_RC_SUCCESS;
}

smtc_se_return_code_t smtc_secure_element_restore_context( void )
{
    lr1110_se_context_nvm_t ctx;
    smtc_modem_hal_context_restore( CONTEXT_SECURE_ELEMENT, ( uint8_t* ) &ctx, sizeof( ctx ) );
    if( lr1110_se_crc( ( uint8_t* ) &ctx, sizeof( ctx ) - 4 ) == ctx.crc )
    {
        lr1110_se_data = ctx.data;
        return SMTC_SE_RC_SUCCESS;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Restore of Secure Element context fails => Return to init values\n" );
        // Initialize data structure to 0
        memset( &lr1110_se_data, 0, sizeof( lr1110_se_data_t ) );
        return SMTC_SE_RC_ERROR;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr1110_crypto_keys_idx_t convert_key_id_from_se_to_lr1110( smtc_se_key_identifier_t key_id )
{
    lr1110_crypto_keys_idx_t id = LR1110_CRYPTO_KEYS_IDX_GP0;

    switch( key_id )
    {
    case SMTC_SE_APP_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_APP_KEY;
        break;
    case SMTC_SE_NWK_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_NWK_KEY;
        break;
    case SMTC_SE_J_S_INT_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_J_S_INT_KEY;
        break;
    case SMTC_SE_J_S_ENC_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_J_S_ENC_KEY;
        break;
    case SMTC_SE_F_NWK_S_INT_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_F_NWK_S_INT_KEY;
        break;
    case SMTC_SE_S_NWK_S_INT_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_S_NWK_S_INT_KEY;
        break;
    case SMTC_SE_NWK_S_ENC_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_NWK_S_ENC_KEY;
        break;
    case SMTC_SE_APP_S_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_APP_S_KEY;
        break;
    case SMTC_SE_MC_ROOT_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_GP_KE_KEY_5;
        break;
    case SMTC_SE_MC_KE_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_GP_KE_KEY_4;
        break;
    case SMTC_SE_MC_KEY_0:
        id = LR1110_CRYPTO_KEYS_IDX_GP_KE_KEY_0;
        break;
    case SMTC_SE_MC_APP_S_KEY_0:
        id = LR1110_CRYPTO_KEYS_IDX_MC_APP_S_KEY_0;
        break;
    case SMTC_SE_MC_NWK_S_KEY_0:
        id = LR1110_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_0;
        break;
    case SMTC_SE_MC_KEY_1:
        id = LR1110_CRYPTO_KEYS_IDX_GP_KE_KEY_1;
        break;
    case SMTC_SE_MC_APP_S_KEY_1:
        id = LR1110_CRYPTO_KEYS_IDX_MC_APP_S_KEY_1;
        break;
    case SMTC_SE_MC_NWK_S_KEY_1:
        id = LR1110_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_1;
        break;
    case SMTC_SE_MC_KEY_2:
        id = LR1110_CRYPTO_KEYS_IDX_GP_KE_KEY_2;
        break;
    case SMTC_SE_MC_APP_S_KEY_2:
        id = LR1110_CRYPTO_KEYS_IDX_MC_APP_S_KEY_2;
        break;
    case SMTC_SE_MC_NWK_S_KEY_2:
        id = LR1110_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_2;
        break;
    case SMTC_SE_MC_KEY_3:
        id = LR1110_CRYPTO_KEYS_IDX_GP_KE_KEY_3;
        break;
    case SMTC_SE_MC_APP_S_KEY_3:
        id = LR1110_CRYPTO_KEYS_IDX_MC_APP_S_KEY_3;
        break;
    case SMTC_SE_MC_NWK_S_KEY_3:
        id = LR1110_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_3;
        break;
    case SMTC_SE_SLOT_RAND_ZERO_KEY:
        id = LR1110_CRYPTO_KEYS_IDX_GP0;
        break;
    default:
        id = LR1110_CRYPTO_KEYS_IDX_GP1;
        break;
    }
    return id;
}

uint32_t lr1110_se_crc( const uint8_t* buf, int len )
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
