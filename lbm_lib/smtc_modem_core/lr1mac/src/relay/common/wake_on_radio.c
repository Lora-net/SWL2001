/*!
 * \file    wake_on_radio.c
 *
 * \brief   WOR and WOR ACK generation function
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#include <string.h>

#include "relay_def.h"
#include "smtc_modem_crypto.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "smtc_modem_hal.h"
#include "smtc_secure_element.h"
#include "wake_on_radio_def.h"
#include "wake_on_radio.h"

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

/**
 * @brief Decode/encode WOR payload
 *
 * @param[in]   buffer_in       Data to be encoded/decoded
 * @param[out]  buffer_out      Encoded/Decoded data
 * @param[in]   wor_ul          Infos required to encode/decode WOR
 * @param[in]   wor_rf          Infos required to encode/decode WOR
 * @param[in]   wor_s_enc_key   WOR Session Encryption Key
 */
static void wor_aes_wor_uplink_enc( const uint8_t* buffer_in, uint8_t* buffer_out, const wor_uplink_t* wor_ul,
                                    const wor_rf_infos_t* wor_rf, const uint8_t wor_s_enc_key[16] );

/**
 * @brief Decode/encode WOR ACK payload
 *
 * @param[in]   buffer_in       Data to be encoded/decoded
 * @param[out]  buffer_out      Encoded/Decoded data
 * @param[in]   crypto_info     Infos required to encode/decode WOR
 * @param[in]   wor_s_enc_key   WOR Session Encryption Key
 */
static void wor_aes_ack_uplink_enc( const uint8_t* buffer_in, uint8_t* buffer_out,
                                    const wor_ack_mic_info_t* crypto_info, const uint8_t wor_s_enc_key[16] );

/**
 * @brief AES encryption for WOR and WOR ACK
 *
 * @param[in]   buffer          Buffer to encode/decode
 * @param[in]   len             Lenght of buffer
 * @param[in]   wor_s_enc_key   WOR Session Encryption Key
 * @param[in]   a_block         a_block
 * @param[out]  enc_buffer      Encoded/Decoded buffer
 */
static void crypto_relay_encrypt( const uint8_t* buffer, uint16_t len, const uint8_t wor_s_enc_key[16],
                                  const uint8_t a_block[16], uint8_t* enc_buffer );

/**
 * @brief AES operation to compute MIC
 *
 * @param[in]   wor_s_int_key   WOR Session Integrity Key
 * @param[in]   mic_bx_buffer   MIC BX buffer
 * @param[in]   buffer          Buffer
 * @param[in]   size            Length of buffer
 * @return uint32_t             MIC value
 */
static uint32_t crypto_relay_compute_mic( const uint8_t* wor_s_int_key, const uint8_t mic_bx_buffer[16],
                                          const uint8_t* buffer, uint16_t size );
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION -------------------------------------------
 */
uint8_t wor_generate_wor( uint8_t* buffer, const wor_infos_t* wor )
{
    if( wor->wor_type == WOR_MSG_TYPE_JOIN_REQUEST )
    {
        const uint32_t freq_step = wor->join_request.freq_hz / 100;

        buffer[WOR_JOINREQ_HEADER]     = WOR_MSG_TYPE_JOIN_REQUEST & 0x0F;
        buffer[WOR_JOINREQ_DR_PL]      = wor->join_request.dr & 0x0F;
        buffer[WOR_JOINREQ_FREQ_7_0]   = ( uint8_t )( freq_step );
        buffer[WOR_JOINREQ_FREQ_15_8]  = ( uint8_t )( freq_step >> 8 );
        buffer[WOR_JOINREQ_FREQ_23_16] = ( uint8_t )( freq_step >> 16 );
        return WOR_JOINREQ_LENGTH;
    }

    if( wor->wor_type == WOR_MSG_TYPE_STANDARD_UPLINK )
    {
        const uint32_t freq_step = wor->uplink.freq_hz / 100;

        buffer[WOR_UPLINK_HEADER] = WOR_MSG_TYPE_STANDARD_UPLINK & 0x0F;

        buffer[WOR_UPLINK_DEVADDR_7_0]   = ( uint8_t )( wor->uplink.devaddr );
        buffer[WOR_UPLINK_DEVADDR_15_8]  = ( uint8_t )( wor->uplink.devaddr >> 8 );
        buffer[WOR_UPLINK_DEVADDR_23_16] = ( uint8_t )( wor->uplink.devaddr >> 16 );
        buffer[WOR_UPLINK_DEVADDR_31_24] = ( uint8_t )( wor->uplink.devaddr >> 24 );
        buffer[WOR_UPLINK_FCNT_7_0]      = ( uint8_t )( wor->uplink.fcnt );
        buffer[WOR_UPLINK_FCNT_15_8]     = ( uint8_t )( wor->uplink.fcnt >> 8 );

        const uint8_t buffer_to_enc[4] = {
            [0] = wor->uplink.dr & 0x0F,
            [1] = ( uint8_t )( freq_step ),
            [2] = ( uint8_t )( freq_step >> 8 ),
            [3] = ( uint8_t )( freq_step >> 16 ),
        };

        wor_aes_wor_uplink_enc( buffer_to_enc, buffer + WOR_UPLINK_PAYLOAD_ENC_1, &wor->uplink, &wor->rf_infos, NULL );
        const wor_mic_infos_t mic_info = { .dev_addr = wor->uplink.devaddr, .wfcnt = wor->uplink.fcnt };
        const uint32_t        mic      = wor_compute_mic_wor( &mic_info, buffer + WOR_UPLINK_PAYLOAD_ENC_1, NULL );

        memcpy( buffer + WOR_UPLINK_MIC_1, &mic, 4 );

        return WOR_UPLINK_LENGTH;
    }

    SMTC_MODEM_HAL_PANIC( "Unknow WOR type" );
    return 0;
}

uint32_t wor_compute_mic_wor( const wor_mic_infos_t* mic_info, const uint8_t* wor_enc, const uint8_t wor_s_int_key[16] )
{
    // CMAC = aes128_cmac(WorSIntKey, B0 | msg)
    // MIC = CMAC[0..3]
    //
    // msg = DevAddr | WorUplinkEnc | WFCnt
    //
    // Size (octets)   || 1    | 4       | 1          | 4       |  4      | 1    | 1
    // B0              || 0x49 | 4x 0x00 | 0x00 (Dir) | DevAddr | WFCnt32 | 0x00 | 0x0E len(WOR Frame)

    uint8_t buffer_b0[16];
    buffer_b0[0]  = 0x49;
    buffer_b0[1]  = 0x00;
    buffer_b0[2]  = 0x00;
    buffer_b0[3]  = 0x00;
    buffer_b0[4]  = 0x00;
    buffer_b0[5]  = 0x00;  // Direction
    buffer_b0[6]  = ( uint8_t )( mic_info->dev_addr );
    buffer_b0[7]  = ( uint8_t )( mic_info->dev_addr >> 8 );
    buffer_b0[8]  = ( uint8_t )( mic_info->dev_addr >> 16 );
    buffer_b0[9]  = ( uint8_t )( mic_info->dev_addr >> 24 );
    buffer_b0[10] = ( uint8_t )( mic_info->wfcnt );
    buffer_b0[11] = ( uint8_t )( mic_info->wfcnt >> 8 );
    buffer_b0[12] = ( uint8_t )( mic_info->wfcnt >> 16 );
    buffer_b0[13] = ( uint8_t )( mic_info->wfcnt >> 24 );
    buffer_b0[14] = 0x00;
    buffer_b0[15] = 0x0E;

    uint8_t tmp[10];

    tmp[0] = ( uint8_t )( mic_info->dev_addr );
    tmp[1] = ( uint8_t )( mic_info->dev_addr >> 8 );
    tmp[2] = ( uint8_t )( mic_info->dev_addr >> 16 );
    tmp[3] = ( uint8_t )( mic_info->dev_addr >> 24 );
    memcpy( tmp + 4, wor_enc, 4 );
    tmp[8] = ( uint8_t )( mic_info->wfcnt );
    tmp[9] = ( uint8_t )( mic_info->wfcnt >> 8 );

    return crypto_relay_compute_mic( wor_s_int_key, buffer_b0, tmp, sizeof( tmp ) );
}

void wor_decrypt_ack( const uint8_t* buffer, const wor_ack_mic_info_t* mic_infos, wor_ack_infos_t* ack,
                      const uint8_t wor_s_enc_key[16] )
{
    uint8_t tmp[3];
    wor_aes_ack_uplink_enc( buffer + WOR_ACK_PAYLOAD_ENC_1, tmp, mic_infos, wor_s_enc_key );
    const uint32_t state_sync = ( tmp[2] << 16 ) + ( tmp[1] << 8 ) + tmp[0];

    ack->t_offset     = WOR_ACK_UPLINK_GET_TOFFSET( state_sync );
    ack->period       = WOR_ACK_UPLINK_GET_CADP( state_sync );
    ack->relay_ppm    = WOR_ACK_UPLINK_GET_XTAL( state_sync );
    ack->dr_relay_gtw = WOR_ACK_UPLINK_GET_GTW_DR( state_sync );
    ack->cad_to_rx    = WOR_ACK_UPLINK_GET_CAD_RX( state_sync );
    ack->relay_fwd    = WOR_ACK_UPLINK_GET_FWD( state_sync );

#if( 0 )
    // Un-comment to display more information on the WOR ACK
    const char* cad_periodd_str[] = { "1 s", "500 ms", "250 ms", "100 ms", "50 ms", "20 ms" };
    const char* fwd_relay_str[]   = { "OK", "RETRY_30MIN", "RETRY_60MIN", "DISABLED" };
    const char* ppm_relay_str[]   = { "10 ppm", "20 ppm", "30 ppm", "40 ppm" };

    SMTC_MODEM_HAL_TRACE_PRINTF( "Decode WOR ACK\n" );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Offset : %d ms\n", ack->t_offset );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Period : %d (%s)\n", ack->period, cad_periodd_str[ack->period] );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - PPM : %d (%s)\n", ack->relay_ppm, ppm_relay_str[ack->relay_ppm] );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - DR GTW : %d\n", ack->dr_relay_gtw );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - Forward : %d (%s)\n", ack->relay_fwd, fwd_relay_str[ack->relay_fwd] );
    SMTC_MODEM_HAL_TRACE_PRINTF( " - CAD2RX : %d (%d symb)\n", ack->cad_to_rx, ( ack->cad_to_rx + 1 ) * 2 );
#endif
}

uint32_t wor_extract_mic_ack( const uint8_t* wor_ack_buffer )
{
    uint32_t mic;
    memcpy( &mic, wor_ack_buffer + WOR_ACK_MIC_1, 4 );
    return mic;
}

uint32_t wor_compute_mic_ack( const wor_ack_mic_info_t* mic_info, const uint8_t* ack_uplink_enc,
                              const uint8_t wor_s_int_key[16] )
{
    // CMAC = aes128_cmac(WorSIntKey, B0 | AckUplinkEnc | WOR | pad16)
    // MIC = CMAC[0..3]
    //
    // Size (octets)    || 1    | 4       | 1         | 4       | 4       | 1    | 1
    // B0               || 0x49 | 4x 0x00 | 0x01 (Dir)| DevAddr | WFCnt32 | 0x00 | 0x07 (len(ACK))
    //
    // Size (bits)      || 4   | 4        | 24        | 16    | 32
    // WOR              || RFU | Datarate | Frequency | WFCnt | DevAddr

    uint8_t buffer_b0[16];
    buffer_b0[0]  = 0x49;
    buffer_b0[1]  = 0x00;
    buffer_b0[2]  = 0x00;
    buffer_b0[3]  = 0x00;
    buffer_b0[4]  = 0x00;
    buffer_b0[5]  = 0x01;  // Direction
    buffer_b0[6]  = ( uint8_t )( mic_info->dev_addr );
    buffer_b0[7]  = ( uint8_t )( mic_info->dev_addr >> 8 );
    buffer_b0[8]  = ( uint8_t )( mic_info->dev_addr >> 16 );
    buffer_b0[9]  = ( uint8_t )( mic_info->dev_addr >> 24 );
    buffer_b0[10] = ( uint8_t )( mic_info->wfcnt );
    buffer_b0[11] = ( uint8_t )( mic_info->wfcnt >> 8 );
    buffer_b0[12] = ( uint8_t )( mic_info->wfcnt >> 16 );
    buffer_b0[13] = ( uint8_t )( mic_info->wfcnt >> 24 );
    buffer_b0[14] = 0x00;
    buffer_b0[15] = 0x07;

    uint8_t tmp[13];
    memcpy( tmp, ack_uplink_enc, 3 );

    const uint32_t freq_step = mic_info->frequency_hz / 100;

    tmp[3]  = mic_info->datarate & 0x0F;
    tmp[4]  = ( uint8_t )( freq_step );
    tmp[5]  = ( uint8_t )( freq_step >> 8 );
    tmp[6]  = ( uint8_t )( freq_step >> 16 );
    tmp[7]  = ( uint8_t )( mic_info->wfcnt );
    tmp[8]  = ( uint8_t )( mic_info->wfcnt >> 8 );
    tmp[9]  = ( uint8_t )( mic_info->dev_addr );
    tmp[10] = ( uint8_t )( mic_info->dev_addr >> 8 );
    tmp[11] = ( uint8_t )( mic_info->dev_addr >> 16 );
    tmp[12] = ( uint8_t )( mic_info->dev_addr >> 24 );

    return crypto_relay_compute_mic( wor_s_int_key, buffer_b0, tmp, sizeof( tmp ) );
}

uint16_t wor_convert_cad_period_in_ms( const wor_cad_periodicity_t period )
{
    const uint16_t tab_convert_cad_period[] = {
        [WOR_CAD_PERIOD_1S] = 1000,   [WOR_CAD_PERIOD_500MS] = 500, [WOR_CAD_PERIOD_250MS] = 250,
        [WOR_CAD_PERIOD_100MS] = 100, [WOR_CAD_PERIOD_50MS] = 50,   [WOR_CAD_PERIOD_20MS] = 20
    };
    if( period < WOR_CAD_PERIOD_RFU )
    {
        return tab_convert_cad_period[period];
    }
    SMTC_MODEM_HAL_TRACE_MSG( "Unknown cad period\n" );
    return 0;
}

uint8_t wor_convert_ppm( const wor_ack_ppm_error_t ppm )
{
    const uint8_t tab_convert_xtal_accuracy[] = { [WOR_ACK_PPM_ERROR_10PPM] = 10,
                                                  [WOR_ACK_PPM_ERROR_20PPM] = 20,
                                                  [WOR_ACK_PPM_ERROR_30PPM] = 30,
                                                  [WOR_ACK_PPM_ERROR_40PPM] = 40 };
    if( ppm <= WOR_ACK_PPM_ERROR_40PPM )
    {
        return tab_convert_xtal_accuracy[ppm];
    }
    SMTC_MODEM_HAL_TRACE_MSG( "Unknown ppm error\n" );
    return 0;
}

uint8_t wor_convert_cadtorx( const wor_ack_cad_to_rx_t cad_to_rx )
{
    const uint8_t tab_convert_cad_to_rx[] = { [WOR_ACK_CAD_TO_RX_2SYMB] = 2,
                                              [WOR_ACK_CAD_TO_RX_4SYMB] = 4,
                                              [WOR_ACK_CAD_TO_RX_6SYMB] = 6,
                                              [WOR_ACK_CAD_TO_RX_8SYMB] = 8 };
    if( cad_to_rx <= WOR_ACK_CAD_TO_RX_8SYMB )
    {
        return tab_convert_cad_to_rx[cad_to_rx];
    }
    SMTC_MODEM_HAL_TRACE_MSG( "Unknown cad to rx\n" );
    return 0;
}

void wor_derive_root_skey( uint32_t dev_addr )
{
    uint8_t block[16];
    memset( block, 0, sizeof( block ) );
    block[0] = 0x01;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_secure_element_derive_and_store_key( block, SMTC_SE_NWK_S_ENC_KEY,
                                                                               SMTC_RELAY_ROOT_WOR_S_KEY,
                                                                               RELAY_STACK_ID ) == SMTC_SE_RC_SUCCESS );

    memset( block, 0, sizeof( block ) );
    block[0] = 0x01;
    block[1] = ( uint8_t )( dev_addr );
    block[2] = ( uint8_t )( dev_addr >> 8 );
    block[3] = ( uint8_t )( dev_addr >> 16 );
    block[4] = ( uint8_t )( dev_addr >> 24 );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_secure_element_derive_and_store_key( block, SMTC_RELAY_ROOT_WOR_S_KEY,
                                                                               SMTC_RELAY_WOR_S_INT_KEY,
                                                                               RELAY_STACK_ID ) == SMTC_SE_RC_SUCCESS );

    memset( block, 0, sizeof( block ) );
    block[0] = 0x02;
    block[1] = ( uint8_t )( dev_addr );
    block[2] = ( uint8_t )( dev_addr >> 8 );
    block[3] = ( uint8_t )( dev_addr >> 16 );
    block[4] = ( uint8_t )( dev_addr >> 24 );
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_secure_element_derive_and_store_key( block, SMTC_RELAY_ROOT_WOR_S_KEY,
                                                                               SMTC_RELAY_WOR_S_ENC_KEY,
                                                                               RELAY_STACK_ID ) == SMTC_SE_RC_SUCCESS );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void wor_aes_wor_uplink_enc( const uint8_t* buffer_in, uint8_t* buffer_out, const wor_uplink_t* wor_ul,
                                    const wor_rf_infos_t* wor_rf, const uint8_t wor_s_enc_key[16] )
{
    uint8_t a_block[16];

    const uint32_t freq_step = wor_rf->freq_hz / 100;

    a_block[0]  = 0x01;
    a_block[1]  = 0x00;
    a_block[2]  = 0x00;
    a_block[3]  = 0x00;  // Uplink
    a_block[4]  = ( uint8_t )( wor_ul->devaddr );
    a_block[5]  = ( uint8_t )( wor_ul->devaddr >> 8 );
    a_block[6]  = ( uint8_t )( wor_ul->devaddr >> 16 );
    a_block[7]  = ( uint8_t )( wor_ul->devaddr >> 24 );
    a_block[8]  = ( uint8_t )( wor_ul->fcnt );
    a_block[9]  = ( uint8_t )( wor_ul->fcnt >> 8 );
    a_block[10] = ( uint8_t )( wor_ul->fcnt >> 16 );
    a_block[11] = ( uint8_t )( wor_ul->fcnt >> 24 );
    a_block[12] = ( uint8_t )( freq_step );
    a_block[13] = ( uint8_t )( freq_step >> 8 );
    a_block[14] = ( uint8_t )( freq_step >> 16 );
    a_block[15] = ( uint8_t )( wor_rf->dr & 0x0F );

    crypto_relay_encrypt( buffer_in, 4, wor_s_enc_key, a_block, buffer_out );
}

static void wor_aes_ack_uplink_enc( const uint8_t* buffer_in, uint8_t* buffer_out,
                                    const wor_ack_mic_info_t* crypto_info, const uint8_t wor_s_enc_key[16] )
{
    uint8_t a_block[16];

    const uint32_t freq_step = crypto_info->frequency_hz / 100;

    a_block[0]  = 0x01;
    a_block[1]  = 0x00;
    a_block[2]  = 0x00;
    a_block[3]  = 0x01;
    a_block[4]  = ( uint8_t )( crypto_info->dev_addr );
    a_block[5]  = ( uint8_t )( crypto_info->dev_addr >> 8 );
    a_block[6]  = ( uint8_t )( crypto_info->dev_addr >> 16 );
    a_block[7]  = ( uint8_t )( crypto_info->dev_addr >> 24 );
    a_block[8]  = ( uint8_t )( crypto_info->wfcnt );
    a_block[9]  = ( uint8_t )( crypto_info->wfcnt >> 8 );
    a_block[10] = ( uint8_t )( crypto_info->wfcnt >> 16 );
    a_block[11] = ( uint8_t )( crypto_info->wfcnt >> 24 );
    a_block[12] = ( uint8_t )( freq_step );
    a_block[13] = ( uint8_t )( freq_step >> 8 );
    a_block[14] = ( uint8_t )( freq_step >> 16 );
    a_block[15] = ( uint8_t )( crypto_info->datarate & 0x0F );

    crypto_relay_encrypt( buffer_in, 3, wor_s_enc_key, a_block, buffer_out );
}

static void crypto_relay_encrypt( const uint8_t* buffer, uint16_t len, const uint8_t wor_s_enc_key[16],
                                  const uint8_t a_block[16], uint8_t* enc_buffer )
{
    if( wor_s_enc_key != NULL )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_modem_crypto_set_key( SMTC_RELAY_WOR_S_ENC_KEY, wor_s_enc_key,
                                                                    RELAY_STACK_ID ) == SMTC_MODEM_CRYPTO_RC_SUCCESS );
    }

    uint16_t buffer_idx  = 0;
    uint8_t  s_block[16] = { 0 };
    int16_t  local_size  = len;  // Length in relay communication is always less than 255.
    while( local_size > 0 )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_secure_element_aes_encrypt( a_block, 16, SMTC_RELAY_WOR_S_ENC_KEY,
                                                                          s_block,
                                                                          RELAY_STACK_ID ) == SMTC_SE_RC_SUCCESS );

        for( uint8_t i = 0; i < ( ( local_size > 16 ) ? 16 : local_size ); i++ )
        {
            enc_buffer[buffer_idx + i] = buffer[buffer_idx + i] ^ s_block[i];
        }
        local_size -= 16;
        buffer_idx += 16;
    }
}

static uint32_t crypto_relay_compute_mic( const uint8_t wor_s_int_key[16], const uint8_t mic_bx_buffer[16],
                                          const uint8_t* buffer, uint16_t size )
{
    if( wor_s_int_key != NULL )
    {
        SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_modem_crypto_set_key( SMTC_RELAY_WOR_S_INT_KEY, wor_s_int_key,
                                                                    RELAY_STACK_ID ) == SMTC_MODEM_CRYPTO_RC_SUCCESS );
    }

    uint32_t mic32 = 0;

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_secure_element_compute_aes_cmac( mic_bx_buffer, buffer, size,
                                                                           SMTC_RELAY_WOR_S_INT_KEY, &mic32,
                                                                           RELAY_STACK_ID ) == SMTC_SE_RC_SUCCESS );

    return mic32;
}
