/*!
 * @file      lr11xx_crypto_engine.c
 *
 * @brief     Cryptographic engine driver implementation for LR11XX
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

#include "lr11xx_crypto_engine.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_CRYPTO_SELECT_CMD_LENGTH ( 2 + 1 )
#define LR11XX_CRYPTO_SET_KEY_CMD_LENGTH ( 2 + 17 )
#define LR11XX_CRYPTO_DERIVE_KEY_CMD_LENGTH ( 2 + 18 )
#define LR11XX_CRYPTO_PROCESS_JOIN_ACCEPT_CMD_LENGTH ( 2 + 3 + 12 + 32 )
#define LR11XX_CRYPTO_COMPUTE_AES_CMAC_CMD_LENGTH ( 2 + 1 + 272 )
#define LR11XX_CRYPTO_VERIFY_AES_CMAC_CMD_LENGTH ( 2 + 1 + 4 + 256 )
#define LR11XX_CRYPTO_AES_ENCRYPT_CMD_LENGTH ( 2 + 1 + 256 )
#define LR11XX_CRYPTO_AES_DECRYPT_CMD_LENGTH ( 2 + 1 + 256 )
#define LR11XX_CRYPTO_STORE_TO_FLASH_CMD_LENGTH ( 2 )
#define LR11XX_CRYPTO_RESTORE_FROM_FLASH_CMD_LENGTH ( 2 )
#define LR11XX_CRYPTO_SET_PARAMETER_CMD_LENGTH ( 2 + 1 + 4 )
#define LR11XX_CRYPTO_GET_PARAMETER_CMD_LENGTH ( 2 + 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operating codes for crypto-related operations
 */
enum
{
    LR11XX_CRYPTO_SELECT_OC              = 0x0500,
    LR11XX_CRYPTO_SET_KEY_OC             = 0x0502,
    LR11XX_CRYPTO_DERIVE_KEY_OC          = 0x0503,
    LR11XX_CRYPTO_PROCESS_JOIN_ACCEPT_OC = 0x0504,
    LR11XX_CRYPTO_COMPUTE_AES_CMAC_OC    = 0x0505,
    LR11XX_CRYPTO_VERIFY_AES_CMAC_OC     = 0x0506,
    LR11XX_CRYPTO_ENCRYPT_AES_01_OC      = 0x0507,
    LR11XX_CRYPTO_ENCRYPT_AES_OC         = 0x0508,
    LR11XX_CRYPTO_DECRYPT_AES_OC         = 0x0509,
    LR11XX_CRYPTO_STORE_TO_FLASH_OC      = 0x050A,
    LR11XX_CRYPTO_RESTORE_FROM_FLASH_OC  = 0x050B,
    LR11XX_CRYPTO_SET_PARAMETER_OC       = 0x050D,
    LR11XX_CRYPTO_GET_PARAMETER_OC       = 0x050E,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Helper function that fill the cbuffer provided in first argument with the command opcode, the key id
 * and the data to encrypt/decrypt/compute aes cmac
 *
 * @param [out] cbuffer Buffer used to build the frame
 * @param [in] opcode Opcode to be added to the frame
 * @param [in] key_id Key ID to be added to the frame
 * @param [in] data Data to be added to the frame
 * @param [in] length Number of bytes from data to be added to the frame
 *
 * @warning The caller MUST ensure cbuffer is array is big enough to contain opcode, key_id, and data!
 */
static void lr11xx_crypto_fill_cbuffer_opcode_key_data( uint8_t* cbuffer, uint16_t opcode, uint8_t key_id,
                                                        const uint8_t* data, uint16_t length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_crypto_select( const void* context, const lr11xx_crypto_element_t element )
{
    uint8_t cbuffer[LR11XX_CRYPTO_SELECT_CMD_LENGTH] = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_SELECT_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_SELECT_OC >> 0 );

    cbuffer[2] = element;

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_CRYPTO_SELECT_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_crypto_set_key( const void* context, lr11xx_crypto_status_t* status, const uint8_t key_id,
                                       const lr11xx_crypto_key_t key )
{
    uint8_t cbuffer[LR11XX_CRYPTO_SET_KEY_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH]      = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_SET_KEY_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_SET_KEY_OC >> 0 );

    cbuffer[2] = key_id;

    for( uint8_t index = 0; index < sizeof( lr11xx_crypto_key_t ); index++ )
    {
        cbuffer[3 + index] = key[index];
    }

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_CRYPTO_SET_KEY_CMD_LENGTH, rbuffer, LR11XX_CRYPTO_STATUS_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_derive_key( const void* context, lr11xx_crypto_status_t* status, const uint8_t src_key_id,
                                          const uint8_t dest_key_id, const lr11xx_crypto_nonce_t nonce )
{
    uint8_t cbuffer[LR11XX_CRYPTO_DERIVE_KEY_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH]         = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_DERIVE_KEY_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_DERIVE_KEY_OC >> 0 );

    cbuffer[2] = src_key_id;
    cbuffer[3] = dest_key_id;

    for( uint8_t index = 0; index < LR11XX_CRYPTO_NONCE_LENGTH; index++ )
    {
        cbuffer[4 + index] = nonce[index];
    }

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_CRYPTO_DERIVE_KEY_CMD_LENGTH, rbuffer, LR11XX_CRYPTO_STATUS_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_process_join_accept( const void* context, lr11xx_crypto_status_t* status,
                                                   const uint8_t dec_key_id, const uint8_t ver_key_id,
                                                   const lr11xx_crypto_lorawan_version_t lorawan_version,
                                                   const uint8_t* header, const uint8_t* data_in, const uint8_t length,
                                                   uint8_t* data_out )
{
    uint8_t cbuffer[LR11XX_CRYPTO_PROCESS_JOIN_ACCEPT_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH + 32]             = { 0x00 };
    uint8_t header_length                                         = ( lorawan_version == 0 ) ? 1 : 12;

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_PROCESS_JOIN_ACCEPT_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_PROCESS_JOIN_ACCEPT_OC >> 0 );

    cbuffer[2] = dec_key_id;
    cbuffer[3] = ver_key_id;
    cbuffer[4] = ( uint8_t ) lorawan_version;

    for( uint8_t index = 0; index < header_length; index++ )
    {
        cbuffer[5 + index] = header[index];
    }

    for( uint8_t index = 0; index < length; index++ )
    {
        cbuffer[5 + header_length + index] = data_in[index];
    }

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, 2 + 3 + header_length + length, rbuffer, 1 + length );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];

        if( *status == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            for( uint8_t index = 0; index < length; index++ )
            {
                data_out[index] = rbuffer[1 + index];
            }
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_compute_aes_cmac( const void* context, lr11xx_crypto_status_t* status,
                                                const uint8_t key_id, const uint8_t* data, const uint16_t length,
                                                lr11xx_crypto_mic_t mic )
{
    uint8_t cbuffer[LR11XX_CRYPTO_COMPUTE_AES_CMAC_CMD_LENGTH]              = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_MIC_LENGTH] = { 0x00 };

    lr11xx_crypto_fill_cbuffer_opcode_key_data( cbuffer, LR11XX_CRYPTO_COMPUTE_AES_CMAC_OC, key_id, data, length );

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, 3 + length, rbuffer,
                                                            LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_MIC_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];

        if( *status == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            for( uint8_t index = 0; index < LR11XX_CRYPTO_MIC_LENGTH; index++ )
            {
                mic[index] = rbuffer[1 + index];
            }
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_verify_aes_cmac( const void* context, lr11xx_crypto_status_t* status,
                                               const uint8_t key_id, const uint8_t* data, const uint16_t length,
                                               const lr11xx_crypto_mic_t mic )
{
    uint8_t cbuffer[LR11XX_CRYPTO_VERIFY_AES_CMAC_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH]              = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_VERIFY_AES_CMAC_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_VERIFY_AES_CMAC_OC >> 0 );

    cbuffer[2] = key_id;

    for( uint8_t index = 0; index < LR11XX_CRYPTO_MIC_LENGTH; index++ )
    {
        cbuffer[3 + index] = mic[index];
    }

    for( uint16_t index = 0; index < length; index++ )
    {
        cbuffer[3 + LR11XX_CRYPTO_MIC_LENGTH + index] = data[index];
    }

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, 3 + LR11XX_CRYPTO_MIC_LENGTH + length,
                                                            rbuffer, LR11XX_CRYPTO_STATUS_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_aes_encrypt_01( const void* context, lr11xx_crypto_status_t* status, const uint8_t key_id,
                                              const uint8_t* data, const uint16_t length, uint8_t* result )
{
    uint8_t cbuffer[LR11XX_CRYPTO_AES_ENCRYPT_CMD_LENGTH]                        = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_DATA_MAX_LENGTH] = { 0x00 };

    lr11xx_crypto_fill_cbuffer_opcode_key_data( cbuffer, LR11XX_CRYPTO_ENCRYPT_AES_01_OC, key_id, data, length );

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, 3 + length, rbuffer, LR11XX_CRYPTO_STATUS_LENGTH + length );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];

        if( *status == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            for( uint16_t index = 0; index < length; index++ )
            {
                result[index] = rbuffer[1 + index];
            }
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_aes_encrypt( const void* context, lr11xx_crypto_status_t* status, const uint8_t key_id,
                                           const uint8_t* data, const uint16_t length, uint8_t* result )
{
    uint8_t cbuffer[LR11XX_CRYPTO_AES_ENCRYPT_CMD_LENGTH]                        = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_DATA_MAX_LENGTH] = { 0x00 };

    lr11xx_crypto_fill_cbuffer_opcode_key_data( cbuffer, LR11XX_CRYPTO_ENCRYPT_AES_OC, key_id, data, length );

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, 3 + length, rbuffer, LR11XX_CRYPTO_STATUS_LENGTH + length );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];

        if( *status == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            for( uint16_t index = 0; index < length; index++ )
            {
                result[index] = rbuffer[1 + index];
            }
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_aes_decrypt( const void* context, lr11xx_crypto_status_t* status, const uint8_t key_id,
                                           const uint8_t* data, const uint16_t length, uint8_t* result )
{
    uint8_t cbuffer[LR11XX_CRYPTO_AES_DECRYPT_CMD_LENGTH]                        = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_DATA_MAX_LENGTH] = { 0x00 };

    lr11xx_crypto_fill_cbuffer_opcode_key_data( cbuffer, LR11XX_CRYPTO_DECRYPT_AES_OC, key_id, data, length );

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, 3 + length, rbuffer, LR11XX_CRYPTO_STATUS_LENGTH + length );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];

        if( *status == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            for( uint16_t index = 0; index < length; index++ )
            {
                result[index] = rbuffer[1 + index];
            }
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_store_to_flash( const void* context, lr11xx_crypto_status_t* status )
{
    uint8_t cbuffer[LR11XX_CRYPTO_STORE_TO_FLASH_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH]             = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_STORE_TO_FLASH_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_STORE_TO_FLASH_OC >> 0 );

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_CRYPTO_STORE_TO_FLASH_CMD_LENGTH,
                                                            rbuffer, LR11XX_CRYPTO_STATUS_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_restore_from_flash( const void* context, lr11xx_crypto_status_t* status )
{
    uint8_t cbuffer[LR11XX_CRYPTO_RESTORE_FROM_FLASH_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH]                 = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_RESTORE_FROM_FLASH_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_RESTORE_FROM_FLASH_OC >> 0 );

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read(
        context, cbuffer, LR11XX_CRYPTO_RESTORE_FROM_FLASH_CMD_LENGTH, rbuffer, LR11XX_CRYPTO_STATUS_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_set_parameter( const void* context, lr11xx_crypto_status_t* status,
                                             const uint8_t param_id, const lr11xx_crypto_param_t parameter )
{
    uint8_t cbuffer[LR11XX_CRYPTO_SET_PARAMETER_CMD_LENGTH] = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH]            = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_SET_PARAMETER_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_SET_PARAMETER_OC >> 0 );

    cbuffer[2] = param_id;

    for( uint8_t index = 0; index < LR11XX_CRYPTO_PARAMETER_LENGTH; index++ )
    {
        cbuffer[3 + index] = parameter[index];
    }

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_CRYPTO_SET_PARAMETER_CMD_LENGTH,
                                                            rbuffer, LR11XX_CRYPTO_STATUS_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_crypto_get_parameter( const void* context, lr11xx_crypto_status_t* status,
                                             const uint8_t param_id, lr11xx_crypto_param_t parameter )
{
    uint8_t cbuffer[LR11XX_CRYPTO_GET_PARAMETER_CMD_LENGTH]                       = { 0x00 };
    uint8_t rbuffer[LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_PARAMETER_LENGTH] = { 0x00 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_CRYPTO_GET_PARAMETER_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_CRYPTO_GET_PARAMETER_OC >> 0 );

    cbuffer[2] = param_id;

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_CRYPTO_GET_PARAMETER_CMD_LENGTH, rbuffer,
                         LR11XX_CRYPTO_STATUS_LENGTH + LR11XX_CRYPTO_PARAMETER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *status = ( lr11xx_crypto_status_t ) rbuffer[0];

        if( *status == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            for( uint8_t index = 0; index < LR11XX_CRYPTO_PARAMETER_LENGTH; index++ )
            {
                parameter[index] = rbuffer[1 + index];
            }
        }
    }

    return ( lr11xx_status_t ) hal_status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr11xx_crypto_fill_cbuffer_opcode_key_data( uint8_t* cbuffer, uint16_t opcode, uint8_t key_id,
                                                        const uint8_t* data, uint16_t length )
{
    cbuffer[0] = ( uint8_t ) ( opcode >> 8 );
    cbuffer[1] = ( uint8_t ) ( opcode >> 0 );

    cbuffer[2] = key_id;

    for( uint16_t index = 0; index < length; index++ )
    {
        cbuffer[3 + index] = data[index];
    }
}

/* --- EOF ------------------------------------------------------------------ */
