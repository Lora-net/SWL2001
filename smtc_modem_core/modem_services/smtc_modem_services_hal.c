/**
 * @file      smtc_modem_services_hal.c
 *
 * @brief     Implementation of smtc_modem_services hal functions
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>   // memcpy, memset
// for variadic args
#include <stdio.h>
#include <stdarg.h>

#include "smtc_modem_services_hal.h"
#include "smtc_modem_crypto.h"
#include "smtc_modem_hal.h"
#include "modem_context.h"

#if defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
#include "lr11xx_gnss.h"
#endif  // LR11XX_TRANSCEIVER && ENABLE_MODEM_GNSS_FEATURE

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

void smtc_modem_services_aes_encrypt( const uint8_t* raw_buffer, uint16_t size, uint8_t aes_ctr_nonce[14],
                                      uint8_t* enc_buffer )
{
    // Modem crypto lib can be used here
    if( smtc_modem_crypto_service_encrypt( raw_buffer, size, aes_ctr_nonce, enc_buffer ) !=
        SMTC_MODEM_CRYPTO_RC_SUCCESS )
    {
        smtc_modem_hal_mcu_panic( "Encryption of lfu failed\n" );
    }
}

uint32_t smtc_modem_services_get_time_s( void )
{
    return smtc_modem_hal_get_compensated_time_in_s( );
}

#if defined( LR11XX_TRANSCEIVER ) && defined( ENABLE_MODEM_GNSS_FEATURE )
radio_return_code_t smtc_modem_services_lr11xx_gnss_get_context_status( const void* radio_ctx, uint8_t buff[9] )
{
    // Secure radio access
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    // read gnss context status
    lr11xx_status_t status = lr11xx_gnss_get_context_status( radio_ctx, buff );
    // Release radio access
    modem_context_resume_radio_access( );
    if( status != LR11XX_STATUS_OK )
    {
        return MODEM_SERVICES_RADIO_ERROR;
    }
    return MODEM_SERVICES_RADIO_OK;
}

radio_return_code_t smtc_modem_services_lr11xx_gnss_push_dmc_msg( const void* radio_ctx, uint8_t* buff,
                                                                  uint16_t buff_len )
{
    // Secure radio access
    modem_context_suspend_radio_access( RP_TASK_TYPE_NONE );
    // push gnss dmc message
    lr11xx_status_t status = lr11xx_gnss_push_dmc_msg( radio_ctx, buff, buff_len );
    // Release radio access
    modem_context_resume_radio_access( );

    if( status != LR11XX_STATUS_OK )
    {
        return MODEM_SERVICES_RADIO_ERROR;
    }
    return MODEM_SERVICES_RADIO_OK;
}
#endif  // LR11XX_TRANSCEIVER && ENABLE_MODEM_GNSS_FEATURE
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
