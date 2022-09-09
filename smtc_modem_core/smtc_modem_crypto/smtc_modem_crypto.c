/*!
 * \file      smtc_modem_crypto.c
 *
 * \brief     Implements crypto funtions needed by modem.
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
 *-----------------------------------------------------------------------------------
 * --- DEPENDENCIES -----------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdint.h>
#include "smtc_modem_crypto.h"
#include "smtc_modem_hal_dbg_trace.h"

#include <string.h>  //for memcpy
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LORAMAC_MIC_BLOCK_B0_SIZE 16

/*
 * Number multicast security contexts
 */
#define NUM_OF_MC_SEC_CTX 4

/*
 * Maximum size of the message that can be handled by the crypto operations
 */
#define CRYPTO_MAXMESSAGE_SIZE 256

/*
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define MIC_BLOCK_BX_SIZE 16

/*
 * Maximum size of the buffer for crypto operations
 */
#define CRYPTO_BUFFER_SIZE ( CRYPTO_MAXMESSAGE_SIZE + MIC_BLOCK_BX_SIZE )

/*
 * LoRaWAN version minor value
 */
#define LORAWAN_VERSION_1_0_X_MINOR_VALUE 0

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Key-Address item
 *
 * @struct smtc_crypto_key_addr_t
 */
typedef struct smtc_crypto_key_addr_s
{
    smtc_modem_crypto_addr_id_t addr_id;   //!< Address identifier
    smtc_se_key_identifier_t    app_skey;  //!< Application session key
    smtc_se_key_identifier_t    nwk_skey;  //!< Network session key
    smtc_se_key_identifier_t    root_key;  //!< Rootkey (Multicast only)
} smtc_crypto_key_addr_t;

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE VARIABLES -------------------------------------------------------------
 */

/*
 * Key-Address list
 */
static smtc_crypto_key_addr_t smtc_crypto_key_addr_list[NUM_OF_MC_SEC_CTX] = {
    { SMTC_MODEM_CRYPTO_MULTICAST_0_ADDR, SMTC_SE_MC_APP_S_KEY_0, SMTC_SE_MC_NWK_S_KEY_0, SMTC_SE_MC_KEY_0 },
    { SMTC_MODEM_CRYPTO_MULTICAST_1_ADDR, SMTC_SE_MC_APP_S_KEY_1, SMTC_SE_MC_NWK_S_KEY_1, SMTC_SE_MC_KEY_1 },
    { SMTC_MODEM_CRYPTO_MULTICAST_2_ADDR, SMTC_SE_MC_APP_S_KEY_2, SMTC_SE_MC_NWK_S_KEY_2, SMTC_SE_MC_KEY_2 },
    { SMTC_MODEM_CRYPTO_MULTICAST_3_ADDR, SMTC_SE_MC_APP_S_KEY_3, SMTC_SE_MC_NWK_S_KEY_3, SMTC_SE_MC_KEY_3 },
};

/*
 *-----------------------------------------------------------------------------------
 *--- PRIVATE FUNCTIONS DECLARATION -------------------------------------------------
 */
/**
 * @brief Computes the frame MIC field
 *
 * @param [in] buffer Data buffer
 * @param [in] size Data buffer size
 * @param [in] key_id Key identifier
 * @param [in] devaddr Device address
 * @param [in] dir Frame direction [0: uplink, 1: downlink]
 * @param [in] fcnt Frame counter
 * @param [in] mic Computed MIC field
 * @return smtc_modem_crypto_return_code_t
 */
static smtc_modem_crypto_return_code_t compute_mic( const uint8_t* buffer, uint16_t size,
                                                    smtc_se_key_identifier_t key_id, uint32_t devaddr, uint8_t dir,
                                                    uint32_t fcnt, uint32_t* mic );

/**
 * @brief Prepares B0 block for cmac computation.
 *
 * @param [in] msg_len Length of message
 * @param [in] dir Frame direction ( Uplink:0, Downlink:1 )
 * @param [in] devaddr Device address
 * @param [in] fcnt Frame counter
 * @param [in] b0 B0 block
 * @return smtc_modem_crypto_return_code_t
 */
static smtc_modem_crypto_return_code_t prepare_b0( uint16_t msg_len, uint8_t dir, uint32_t devaddr, uint32_t fcnt,
                                                   uint8_t* b0 );

/**
 * @brief Derives a session key as of LoRaWAN versions prior to 1.1.0
 *
 * @param [in] key_id Key Identifier for the key to be calculated
 * @param [in] join_nonce Sever nonce (3 Bytes)
 * @param [in] net_id Network Identifier (3 bytes)
 * @param [in] dev_nonce Device nonce
 * @return smtc_modem_crypto_return_code_t
 */
static smtc_modem_crypto_return_code_t derive_session_key_1_0_x( smtc_se_key_identifier_t key_id,
                                                                 const uint8_t* join_nonce, const uint8_t* net_id,
                                                                 uint16_t dev_nonce );

/**
 * @brief Derives the Multicast Root Key (McRootKey) from the AppKey.
 *
 * @param [in] lorawan_version_minor LoRaWAN specification minor version to be used.
 * @return smtc_modem_crypto_return_code_t
 */
static smtc_modem_crypto_return_code_t derive_multicast_root_key( uint8_t lorawan_version_minor );

/**
 * @brief Derives the Multicast Key Encryption Key (McKEKey) from the McRootKey
 *
 * @return smtc_modem_crypto_return_code_t
 */
static smtc_modem_crypto_return_code_t derive_multicast_key_encryption_key( void );

/**
 * @brief Get key addr item from address
 *
 * @param [in] addr_id Address identifier
 * @param [in] itemKey item reference
 * @return smtc_modem_crypto_return_code_t
 */
static smtc_modem_crypto_return_code_t get_key_addr_item( smtc_modem_crypto_addr_id_t addr_id,
                                                          smtc_crypto_key_addr_t**    item );

/*
 *-----------------------------------------------------------------------------------
 *--- PUBLIC FUNCTIONS DEFINITIONS --------------------------------------------------
 */

smtc_modem_crypto_return_code_t smtc_modem_crypto_payload_encrypt( const uint8_t* buffer, uint16_t size,
                                                                   smtc_se_key_identifier_t key_id, uint32_t address,
                                                                   uint8_t dir, uint32_t frame_counter,
                                                                   uint8_t* enc_buffer )
{
    if( ( buffer == 0 ) || ( enc_buffer == 0 ) )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_NPE;
    }

    uint16_t buffer_idx = 0;
    uint16_t ctr        = 1;
    uint8_t  sBlock[16] = { 0 };
    uint8_t  aBlock[16] = { 0 };
    int16_t  local_size = size;

    aBlock[0] = 0x01;

    aBlock[5] = dir;

    aBlock[6] = address & 0xFF;
    aBlock[7] = ( address >> 8 ) & 0xFF;
    aBlock[8] = ( address >> 16 ) & 0xFF;
    aBlock[9] = ( address >> 24 ) & 0xFF;

    aBlock[10] = frame_counter & 0xFF;
    aBlock[11] = ( frame_counter >> 8 ) & 0xFF;
    aBlock[12] = ( frame_counter >> 16 ) & 0xFF;
    aBlock[13] = ( frame_counter >> 24 ) & 0xFF;

    while( local_size > 0 )
    {
        aBlock[15] = ctr & 0xFF;

        ctr++;

        if( smtc_secure_element_aes_encrypt( aBlock, 16, key_id, sBlock ) != SMTC_SE_RC_SUCCESS )
        {
            return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
        }

        for( uint8_t i = 0; i < ( ( local_size > 16 ) ? 16 : local_size ); i++ )
        {
            enc_buffer[buffer_idx + i] = buffer[buffer_idx + i] ^ sBlock[i];
        }
        local_size -= 16;
        buffer_idx += 16;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_payload_decrypt( const uint8_t* enc_buffer, uint16_t size,
                                                                   smtc_se_key_identifier_t key_id, uint32_t address,
                                                                   uint8_t dir, uint32_t frame_counter,
                                                                   uint8_t* dec_buffer )
{
    return smtc_modem_crypto_payload_encrypt( enc_buffer, size, key_id, address, dir, frame_counter, dec_buffer );
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_compute_join_mic( const uint8_t* buffer, uint16_t size,
                                                                    uint32_t* mic )
{
    if( smtc_secure_element_compute_aes_cmac( NULL, buffer, size, SMTC_SE_NWK_KEY, mic ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

// decipher + mic check => replace old function join_decrypt and check join_mic
smtc_modem_crypto_return_code_t smtc_modem_crypto_process_join_accept( const uint8_t* enc_buffer, uint16_t size,
                                                                       uint8_t* dec_buffer )
{
    // Join EUI and nonce arguments of smtc_secure_element_process_join_accept function are only used for 1.1.x LoRaWAN
    // versions

    uint8_t lorawan_minor_value = 0;

    if( smtc_secure_element_process_join_accept( SMTC_SE_JOIN_REQ, NULL, 0, enc_buffer, size, dec_buffer,
                                                 &lorawan_minor_value ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }
    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_derive_skeys( const uint8_t join_nonce[LORAWAN_JOIN_NONCE_SIZE],
                                                                const uint8_t net_id[LORAWAN_NET_ID_SIZE],
                                                                uint16_t      dev_nonce )
{
    smtc_modem_crypto_return_code_t rc = SMTC_MODEM_CRYPTO_RC_ERROR;

    rc = derive_session_key_1_0_x( SMTC_SE_APP_S_KEY, join_nonce, net_id, dev_nonce );
    if( rc != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return rc;
    }

    rc = derive_session_key_1_0_x( SMTC_SE_NWK_S_ENC_KEY, join_nonce, net_id, dev_nonce );
    if( rc != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return rc;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_verify_mic( const uint8_t* buffer, uint16_t size,
                                                              smtc_se_key_identifier_t key_id, uint32_t devaddr,
                                                              uint8_t dir, uint32_t fcnt, uint32_t expected_mic )
{
    if( buffer == 0 )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_NPE;
    }
    if( size > CRYPTO_MAXMESSAGE_SIZE )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_BUF_SIZE;
    }

    uint8_t mic_buff[CRYPTO_BUFFER_SIZE] = { 0 };

    // Initialize the first Block
    prepare_b0( size, dir, devaddr, fcnt, mic_buff );

    // Copy the given data to the mic computation buffer
    memcpy( ( mic_buff + MIC_BLOCK_BX_SIZE ), buffer, size );

    smtc_se_return_code_t rc = SMTC_SE_RC_ERROR;
    rc = smtc_secure_element_verify_aes_cmac( mic_buff, ( size + MIC_BLOCK_BX_SIZE ), expected_mic, key_id );

    if( rc == SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_SUCCESS;
    }
    else if( rc == SMTC_SE_RC_FAIL_CMAC )
    {
        return SMTC_MODEM_CRYPTO_RC_FAIL_MIC;
    }
    else
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_compute_and_add_mic( uint8_t* buffer, uint16_t size,
                                                                       smtc_se_key_identifier_t key_id,
                                                                       uint32_t devaddr, uint8_t dir, uint32_t fcnt )
{
    smtc_modem_crypto_return_code_t rc;
    uint32_t                        computed_mic;
    rc = compute_mic( buffer, size, key_id, devaddr, dir, fcnt, &computed_mic );
    memcpy( &buffer[size], ( uint8_t* ) &computed_mic, 4 );
    return rc;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_set_key( smtc_se_key_identifier_t key_id, const uint8_t* key )
{
    if( smtc_secure_element_set_key( key_id, key ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    // Multicast lifetime key derivation
    if( key_id == SMTC_SE_APP_KEY )
    {
        // Derive lifetime keys
        if( derive_multicast_root_key( LORAWAN_VERSION_1_0_X_MINOR_VALUE ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
        }
        if( derive_multicast_key_encryption_key( ) != SMTC_MODEM_CRYPTO_RC_SUCCESS )
        {
            return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
        }
    }
    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_derive_multicast_session_keys( smtc_modem_crypto_addr_id_t addr_id,
                                                                                 uint32_t                    mc_addr )
{
    if( mc_addr == 0 )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_NPE;
    }
    if( addr_id > SMTC_MODEM_CRYPTO_MULTICAST_3_ADDR )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_INVALID_ADDR_ID;
    }

    smtc_modem_crypto_return_code_t rc = SMTC_MODEM_CRYPTO_RC_ERROR;

    // Determine current security context
    smtc_crypto_key_addr_t* cur_item;
    rc = get_key_addr_item( addr_id, &cur_item );
    if( rc != SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        return rc;
    }

    // McAppSKey = aes128_encrypt(McKey, 0x01 | McAddr | pad16)
    // McNwkSKey = aes128_encrypt(McKey, 0x02 | McAddr | pad16)

    uint8_t comp_base_app_s[16] = { 0 };
    uint8_t comp_base_nwk_s[16] = { 0 };

    comp_base_app_s[0] = 0x01;
    comp_base_app_s[1] = mc_addr & 0xFF;
    comp_base_app_s[2] = ( mc_addr >> 8 ) & 0xFF;
    comp_base_app_s[3] = ( mc_addr >> 16 ) & 0xFF;
    comp_base_app_s[4] = ( mc_addr >> 24 ) & 0xFF;

    comp_base_nwk_s[0] = 0x02;
    comp_base_nwk_s[1] = mc_addr & 0xFF;
    comp_base_nwk_s[2] = ( mc_addr >> 8 ) & 0xFF;
    comp_base_nwk_s[3] = ( mc_addr >> 16 ) & 0xFF;
    comp_base_nwk_s[4] = ( mc_addr >> 24 ) & 0xFF;

    if( smtc_secure_element_derive_and_store_key( comp_base_app_s, cur_item->root_key, cur_item->app_skey ) !=
        SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    if( smtc_secure_element_derive_and_store_key( comp_base_nwk_s, cur_item->root_key, cur_item->nwk_skey ) !=
        SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_get_class_b_rand( uint32_t beacon_epoch_time, uint32_t dev_addr,
                                                                    uint8_t rand[16] )
{
    uint8_t a_block[16] = { 0 };

    // First fill RAND_ZERO_KEY with 0 (use a_block as it is initially filled with 0)
    if( smtc_secure_element_set_key( SMTC_SE_SLOT_RAND_ZERO_KEY, a_block ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    // a block filling
    a_block[0] = ( beacon_epoch_time ) &0xFF;
    a_block[1] = ( beacon_epoch_time >> 8 ) & 0xFF;
    a_block[2] = ( beacon_epoch_time >> 16 ) & 0xFF;
    a_block[3] = ( beacon_epoch_time >> 24 ) & 0xFF;
    a_block[4] = ( dev_addr ) &0xFF;
    a_block[5] = ( dev_addr >> 8 ) & 0xFF;
    a_block[6] = ( dev_addr >> 16 ) & 0xFF;
    a_block[7] = ( dev_addr >> 24 ) & 0xFF;

    // Then compute an aes on the a_block with the RAND_ZERO_KEY
    if( smtc_secure_element_aes_encrypt( a_block, 16, SMTC_SE_SLOT_RAND_ZERO_KEY, rand ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

smtc_modem_crypto_return_code_t smtc_modem_crypto_service_encrypt( const uint8_t* clear_buff, uint16_t len,
                                                                   uint8_t nonce[14], uint8_t* enc_buff )
{
    if( ( clear_buff == 0 ) || ( enc_buff == 0 ) )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_NPE;
    }

    uint16_t index       = 0;
    uint16_t ctr         = 1;
    uint8_t  s_block[16] = { 0 };
    uint8_t  a_block[16] = { 0 };
    int16_t  local_size  = len;

    // first copy the 14 bytes of nonce into a_block first 14 bytes
    memcpy( a_block, nonce, 14 );

    while( local_size > 0 )
    {
        a_block[15] = ctr & 0xFF;
        a_block[14] = ( ctr >> 8 ) & 0xFF;

        ctr++;

        if( smtc_secure_element_aes_encrypt( a_block, 16, SMTC_SE_APP_S_KEY, s_block ) != SMTC_SE_RC_SUCCESS )
        {
            return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
        }

        for( uint8_t i = 0; i < ( ( local_size > 16 ) ? 16 : local_size ); i++ )
        {
            enc_buff[index + i] = clear_buff[index + i] ^ s_block[i];
        }
        local_size -= 16;
        index += 16;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

/*
 *-----------------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITIONS ------------------------------------------------
 */

static smtc_modem_crypto_return_code_t compute_mic( const uint8_t* buffer, uint16_t size,
                                                    smtc_se_key_identifier_t key_id, uint32_t devaddr, uint8_t dir,
                                                    uint32_t fcnt, uint32_t* mic )
{
    if( ( buffer == 0 ) || ( mic == 0 ) )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_NPE;
    }
    if( size > CRYPTO_MAXMESSAGE_SIZE )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_BUF_SIZE;
    }

    uint8_t mic_buff[MIC_BLOCK_BX_SIZE];

    // Initialize the first Block
    prepare_b0( size, dir, devaddr, fcnt, mic_buff );

    if( smtc_secure_element_compute_aes_cmac( mic_buff, buffer, size, key_id, mic ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }
    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

static smtc_modem_crypto_return_code_t prepare_b0( uint16_t msg_len, uint8_t dir, uint32_t devaddr, uint32_t fcnt,
                                                   uint8_t* b0 )
{
    if( b0 == 0 )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_NPE;
    }

    b0[0] = 0x49;
    b0[1] = 0x00;
    b0[2] = 0x00;
    b0[3] = 0x00;
    b0[4] = 0x00;

    b0[5] = dir;

    b0[6] = devaddr & 0xFF;
    b0[7] = ( devaddr >> 8 ) & 0xFF;
    b0[8] = ( devaddr >> 16 ) & 0xFF;
    b0[9] = ( devaddr >> 24 ) & 0xFF;

    b0[10] = fcnt & 0xFF;
    b0[11] = ( fcnt >> 8 ) & 0xFF;
    b0[12] = ( fcnt >> 16 ) & 0xFF;
    b0[13] = ( fcnt >> 24 ) & 0xFF;

    b0[14] = 0x00;

    b0[15] = msg_len & 0xFF;

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

static smtc_modem_crypto_return_code_t derive_session_key_1_0_x( smtc_se_key_identifier_t key_id,
                                                                 const uint8_t* join_nonce, const uint8_t* net_id,
                                                                 uint16_t dev_nonce )
{
    uint8_t comp_base[16] = { 0 };

    switch( key_id )
    {
    case SMTC_SE_F_NWK_S_INT_KEY:
    case SMTC_SE_S_NWK_S_INT_KEY:
    case SMTC_SE_NWK_S_ENC_KEY:
        comp_base[0] = 0x01;
        break;
    case SMTC_SE_APP_S_KEY:
        comp_base[0] = 0x02;
        break;
    default:
        return SMTC_MODEM_CRYPTO_RC_ERROR_INVALID_KEY_ID;
    }

    // join_nonce
    comp_base[1] = join_nonce[0];
    comp_base[2] = join_nonce[1];
    comp_base[3] = join_nonce[2];

    // net_id
    comp_base[4] = net_id[0];
    comp_base[5] = net_id[1];
    comp_base[6] = net_id[2];

    comp_base[7] = ( uint8_t )( ( dev_nonce >> 0 ) & 0xFF );
    comp_base[8] = ( uint8_t )( ( dev_nonce >> 8 ) & 0xFF );

    if( smtc_secure_element_derive_and_store_key( comp_base, SMTC_SE_NWK_KEY, key_id ) != SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }

    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

static smtc_modem_crypto_return_code_t derive_multicast_root_key( uint8_t lorawan_version_minor )
{
    uint8_t comp_base[16] = { 0 };

    if( lorawan_version_minor == 1 )
    {
        comp_base[0] = 0x20;
    }
    if( smtc_secure_element_derive_and_store_key( comp_base, SMTC_SE_APP_KEY, SMTC_SE_MC_ROOT_KEY ) !=
        SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }
    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

static smtc_modem_crypto_return_code_t derive_multicast_key_encryption_key( void )
{
    uint8_t comp_base[16] = { 0 };

    if( smtc_secure_element_derive_and_store_key( comp_base, SMTC_SE_MC_ROOT_KEY, SMTC_SE_MC_KE_KEY ) !=
        SMTC_SE_RC_SUCCESS )
    {
        return SMTC_MODEM_CRYPTO_RC_ERROR_SECURE_ELEMENT;
    }
    return SMTC_MODEM_CRYPTO_RC_SUCCESS;
}

static smtc_modem_crypto_return_code_t get_key_addr_item( smtc_modem_crypto_addr_id_t addr_id,
                                                          smtc_crypto_key_addr_t**    item )
{
    for( uint8_t i = 0; i < NUM_OF_MC_SEC_CTX; i++ )
    {
        if( smtc_crypto_key_addr_list[i].addr_id == addr_id )
        {
            *item = &( smtc_crypto_key_addr_list[i] );
            return SMTC_MODEM_CRYPTO_RC_SUCCESS;
        }
    }
    return SMTC_MODEM_CRYPTO_RC_ERROR_INVALID_ADDR_ID;
}

/* --- EOF ------------------------------------------------------------------ */
