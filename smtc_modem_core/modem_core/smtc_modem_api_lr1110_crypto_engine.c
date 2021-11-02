/**
 * @file      smtc_modem_api_lr1110_crypto_engine.c
 *
 * @brief     Cryptographic engine api implementation for modem on LR1110
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

#include "smtc_modem_api_lr1110_crypto_engine.h"
#include "lr1110_crypto_engine.h"
#include "lr1110_types.h"

#include "modem_context.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_status_t smtc_modem_lr1110_crypto_select( const void* context, const lr1110_crypto_element_t element )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_select( context, element );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_set_key( const void* context, lr1110_crypto_status_t* status,
                                                  const uint8_t key_id, const lr1110_crypto_key_t key )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_set_key( context, status, key_id, key );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_derive_key( const void* context, lr1110_crypto_status_t* status,
                                                     const uint8_t src_key_id, const uint8_t dest_key_id,
                                                     const lr1110_crypto_nonce_t nonce )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_derive_key( context, status, src_key_id, dest_key_id, nonce );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_process_join_accept( const void* context, lr1110_crypto_status_t* status,
                                                              const uint8_t dec_key_id, const uint8_t ver_key_id,
                                                              const lr1110_crypto_lorawan_version_t lorawan_version,
                                                              const uint8_t* header, const uint8_t* data_in,
                                                              const uint8_t length, uint8_t* data_out )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_process_join_accept( context, status, dec_key_id, ver_key_id, lorawan_version, header,
                                                       data_in, length, data_out );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_compute_aes_cmac( const void* context, lr1110_crypto_status_t* status,
                                                           const uint8_t key_id, const uint8_t* data,
                                                           const uint16_t length, lr1110_crypto_mic_t mic )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_compute_aes_cmac( context, status, key_id, data, length, mic );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_verify_aes_cmac( const void* context, lr1110_crypto_status_t* status,
                                                          const uint8_t key_id, const uint8_t* data,
                                                          const uint16_t length, const lr1110_crypto_mic_t mic )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_verify_aes_cmac( context, status, key_id, data, length, mic );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_aes_encrypt_01( const void* context, lr1110_crypto_status_t* status,
                                                         const uint8_t key_id, const uint8_t* data,
                                                         const uint16_t length, uint8_t* result )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_aes_encrypt_01( context, status, key_id, data, length, result );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_aes_encrypt( const void* context, lr1110_crypto_status_t* status,
                                                      const uint8_t key_id, const uint8_t* data, const uint16_t length,
                                                      uint8_t* result )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_aes_encrypt( context, status, key_id, data, length, result );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_aes_decrypt( const void* context, lr1110_crypto_status_t* status,
                                                      const uint8_t key_id, const uint8_t* data, const uint16_t length,
                                                      uint8_t* result )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_aes_decrypt( context, status, key_id, data, length, result );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_store_to_flash( const void* context, lr1110_crypto_status_t* status )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_store_to_flash( context, status );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_restore_from_flash( const void* context, lr1110_crypto_status_t* status )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_restore_from_flash( context, status );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_set_parameter( const void* context, lr1110_crypto_status_t* status,
                                                        const uint8_t param_id, const lr1110_crypto_param_t parameter )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_set_parameter( context, status, param_id, parameter );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

lr1110_status_t smtc_modem_lr1110_crypto_get_parameter( const void* context, lr1110_crypto_status_t* status,
                                                        const uint8_t param_id, lr1110_crypto_param_t parameter )
{
    lr1110_status_t lr1110_status;
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    lr1110_status = lr1110_crypto_get_parameter( context, status, param_id, parameter );
    modem_context_resume_radio_access( );
    return lr1110_status;
}

/* --- EOF ------------------------------------------------------------------ */
