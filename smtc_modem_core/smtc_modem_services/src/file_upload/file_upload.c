/*!
 * \file      file_upload.c
 *
 * \brief     File upload implementation
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
#include "file_upload.h"

#include <stdbool.h>  // bool type
#include <stdint.h>   // C99 types
#include <string.h>   //memcpy
#include "modem_services_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

// ------------------------------------------------
// SHA-256

#undef ROR
#undef CH
#undef MAJ
#undef EP0
#undef EP1
#undef SIG0
#undef SIG1

#define ROR( a, b ) ( ( ( a ) >> ( b ) ) | ( ( a ) << ( 32 - ( b ) ) ) )

#define CH( x, y, z ) ( ( ( x ) & ( y ) ) ^ ( ~( x ) & ( z ) ) )
#define MAJ( x, y, z ) ( ( ( x ) & ( y ) ) ^ ( ( x ) & ( z ) ) ^ ( ( y ) & ( z ) ) )
#define EP0( x ) ( ROR( x, 2 ) ^ ROR( x, 13 ) ^ ROR( x, 22 ) )
#define EP1( x ) ( ROR( x, 6 ) ^ ROR( x, 11 ) ^ ROR( x, 25 ) )
#define SIG0( x ) ( ROR( x, 7 ) ^ ROR( x, 18 ) ^ ( ( x ) >> 3 ) )
#define SIG1( x ) ( ROR( x, 17 ) ^ ROR( x, 19 ) ^ ( ( x ) >> 10 ) )

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define ENDIAN_n2b32( x ) __builtin_bswap32( x )
#else
#define ENDIAN_n2b32( x ) ( x )
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// file upload header size
#define FILE_UPLOAD_HEADER_SIZE ( 12 )

// Length of filedone frame
#define FILE_UPLOAD_FILEDONE_FRAME_LENGTH ( 1 )

// File upload maximum size
#ifndef FILE_UPLOAD_MAX_SIZE
#define FILE_UPLOAD_MAX_SIZE ( ( 8 * 1024 ) - FILE_UPLOAD_HEADER_SIZE )
#endif

// number of words per chunk
#define CHUNK_NW ( 2 )

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
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t phash( uint32_t x );
static uint32_t checkbits( uint32_t cid, uint32_t cct, uint32_t i );
static void     function_xor( uint32_t* dst, uint32_t* src, int32_t nw );
static void     gen_chunk( file_upload_t* file_upload, uint32_t* dst, uint32_t* src, uint32_t cct, uint32_t cid );

/**
 * @brief Compute SHA256
 *
 * @param [in] hash Contains the computed hash
 * @param [in] msg  input buffer
 * @param [in] len  input buffer length
 */
static void sha256( uint32_t* hash, const uint8_t* msg, uint32_t len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

file_upload_return_code_t file_upload_init( file_upload_t* file_upload, uint32_t session_id, uint32_t file_len,
                                            uint16_t average_delay, uint8_t port, uint8_t encryption,
                                            uint8_t session_counter )
{
    if( file_len > FILE_UPLOAD_MAX_SIZE )
    {
        LOG_ERROR( "FileUpload is too large (%d > %d )\n", file_len, FILE_UPLOAD_MAX_SIZE );
        return FILE_UPLOAD_ERROR;
    }
    uint16_t sz_tmp = file_len + FILE_UPLOAD_HEADER_SIZE;
    uint32_t cct    = ( sz_tmp + ( ( 4 * CHUNK_NW ) - 1 ) ) / ( 4 * CHUNK_NW );

    file_upload->sid             = session_id & 0x3;
    file_upload->session_counter = session_counter;
    file_upload->encrypt_mode    = encryption;
    file_upload->average_delay   = average_delay;
    file_upload->port            = port;
    file_upload->file_len        = file_len;
    file_upload->cct             = cct;
    file_upload->cntx            = 0;
    file_upload->fntx            = 0;
    file_upload->header[0] =
        ( port ) + ( encryption << 8 ) + ( ( file_len & 0xFF ) << 16 ) + ( ( ( file_len & 0xFF00 ) >> 8 ) << 24 );

    return FILE_UPLOAD_OK;
}

void file_upload_attach_file_buffer( file_upload_t* file_upload, const uint8_t* file )
{
    file_upload->file_buf = ( uint32_t* ) file;
}

file_upload_return_code_t file_upload_prepare_upload( file_upload_t* file_upload )
{
    uint32_t hash[8];
    sha256( hash, ( unsigned char* ) file_upload->file_buf, file_upload->file_len );
    file_upload->header[1] = hash[0];
    file_upload->header[2] = hash[1];

    if( file_upload->encrypt_mode == FILE_UPLOAD_ENCRYPTED )
    {
        // encrypt using AppSKey with "upload" category and file size and hash as diversification data
        uint8_t nonce[14] = { 0 };

        nonce[0] = 0x01;

        nonce[5]  = FILE_UPLOAD_DIRECTION;
        nonce[6]  = file_upload->file_len & 0xFF;
        nonce[7]  = ( file_upload->file_len >> 8 ) & 0xFF;
        nonce[8]  = ( file_upload->file_len >> 16 ) & 0xFF;
        nonce[9]  = ( file_upload->file_len >> 24 ) & 0xFF;
        nonce[10] = hash[0] & 0xFF;
        nonce[11] = ( hash[0] >> 8 ) & 0xFF;
        nonce[12] = ( hash[0] >> 16 ) & 0xFF;
        nonce[13] = ( hash[0] >> 24 ) & 0xFF;
        smtc_modem_services_aes_encrypt( ( uint8_t* ) file_upload->file_buf, file_upload->file_len, nonce,
                                         ( uint8_t* ) file_upload->file_buf );

        // compute hash over encrypted data
        sha256( hash, ( unsigned char* ) file_upload->file_buf, file_upload->file_len );

        // hash over plain data (first byte)
        file_upload->header[2] = file_upload->header[1];
        // hash over encrypted data (first byte)
        file_upload->header[1] = hash[0];
    }
    return FILE_UPLOAD_OK;
}

int32_t file_upload_get_fragment( file_upload_t* file_upload, uint8_t* buf, int32_t len, uint32_t fcnt )
{
    if( ( len - 3 ) < ( CHUNK_NW * 4 ) )
    {
        return 0;
    }
    len = len - 3;
    // discriminator (16bit little endian): 2bit session id, 4bit session
    // counter, 10bit chunk count-1
    uint32_t d = ( ( file_upload->sid & 0x03 ) << 14 ) | ( ( file_upload->session_counter & 0x0F ) << 10 ) |
                 ( ( file_upload->cct - 1 ) & 0x03FF );
    int32_t n     = 0;
    buf[n++]      = FILE_UPLOAD_TOKEN;
    buf[n++]      = d;
    buf[n++]      = d >> 8;
    uint32_t  cid = phash( fcnt );
    uint32_t* src = &file_upload->file_buf[0];

    while( len >= ( CHUNK_NW * 4 ) )
    {
        uint32_t tmp[CHUNK_NW];
        gen_chunk( file_upload, tmp, src, file_upload->cct, cid++ );
        memcpy( buf + n, tmp, CHUNK_NW * 4 );
        n += ( CHUNK_NW * 4 );
        len -= ( CHUNK_NW * 4 );
    }
    if( n > 0 )
    {
        file_upload->cntx += ( n - 3 ) / ( CHUNK_NW * 4 );  // update number of chunks sent
        if( file_upload->fntx < 255 )
        {
            file_upload->fntx += 1;  // update number of frames sent
        }
    }
    return n;
}

bool file_upload_is_data_remaining( file_upload_t* file_upload )
{
    // limit number of chunks sent to twice the chunk count but send minimum three frames
    return ( ( file_upload->fntx < 3 ) || ( file_upload->cntx < ( 2 * file_upload->cct ) ) );
}

file_upload_return_code_t file_upload_process_file_done_frame( file_upload_t* file_upload, const uint8_t* payload,
                                                               uint8_t len )
{
    if( len != FILE_UPLOAD_FILEDONE_FRAME_LENGTH )
    {
        return FILE_UPLOAD_ERROR;
    }

    // TODO: check if session id is also present in downlink and compare it to current

    // check if sctr value in filedone message corresponds to current sctr
    if( ( payload[0] & 0xf ) == file_upload->session_counter )
    {
        return FILE_UPLOAD_OK;
    }
    else
    {
        return FILE_UPLOAD_ERROR;
    }
}

uint32_t file_upload_get_average_delay_in_s( file_upload_t* file_upload )
{
    return file_upload->average_delay;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void gen_chunk( file_upload_t* file_upload, uint32_t* dst, uint32_t* src, uint32_t cct, uint32_t cid )
{
    memset( dst, 0, CHUNK_NW * 4 );
    uint32_t bits = 0;  // initialized to make compiler happy
    for( uint32_t i = 0; i < cct; i++ )
    {
        if( ( i & 31 ) == 0 )
        {
            bits = checkbits( cid, cct, i >> 5 );
        }
        if( bits == 0 )
        {
            continue;
        }
        if( bits & 1 )
        {
            if( i == 0 )
            {
                uint32_t tmp[2];
                tmp[0] = file_upload->header[0];
                tmp[1] = file_upload->header[1];
                function_xor( dst, tmp, CHUNK_NW );
            }
            else if( i == 1 )
            {
                uint32_t tmp[2];
                tmp[0] = file_upload->header[2];
                tmp[1] = *( src );
                function_xor( dst, tmp, CHUNK_NW );
            }
            else
            {
                function_xor( dst, src + ( CHUNK_NW * i ) - 3, CHUNK_NW );
            }
        }
        bits >>= 1;
    }
}

// 32bit pseudo hash
static uint32_t phash( uint32_t x )
{
    x = ( ( x >> 16 ) ^ x ) * 0x45d9f3b;
    x = ( ( x >> 16 ) ^ x ) * 0x45d9f3b;
    x = ( ( x >> 16 ) ^ x );
    return x;
}

static uint32_t checkbits( uint32_t cid, uint32_t cct, uint32_t i )
{
    uint32_t ncw = ( cct + 31 ) >> 5;  // number of checkwords per chunk
    return phash( cid * ncw + i );
}

static void function_xor( uint32_t* dst, uint32_t* src, int32_t nw )
{
    while( nw-- > 0 )
    {
        *dst++ ^= *src++;
    }
}

static void sha256_do( uint32_t* state, const uint8_t* block )
{
    static const uint32_t K[64] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
                                    0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
                                    0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
                                    0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
                                    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
                                    0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
                                    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
                                    0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                                    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
                                    0xc67178f2 };

    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, w[64];

    for( i = 0, j = 0; i < 16; i++, j += 4 )
    {
        w[i] = ( block[j] << 24 ) | ( block[j + 1] << 16 ) | ( block[j + 2] << 8 ) | ( block[j + 3] );
    }
    for( ; i < 64; i++ )
    {
        w[i] = SIG1( w[i - 2] ) + w[i - 7] + SIG0( w[i - 15] ) + w[i - 16];
    }

    a = state[0];
    b = state[1];
    c = state[2];
    d = state[3];
    e = state[4];
    f = state[5];
    g = state[6];
    h = state[7];

    for( i = 0; i < 64; i++ )
    {
        t1 = h + EP1( e ) + CH( e, f, g ) + K[i] + w[i];
        t2 = EP0( a ) + MAJ( a, b, c );
        h  = g;
        g  = f;
        f  = e;
        e  = d + t1;
        d  = c;
        c  = b;
        b  = a;
        a  = t1 + t2;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}

static void sha256( uint32_t* hash, const uint8_t* msg, uint32_t len )
{
    uint32_t state[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };

    uint32_t bitlen = len << 3;
    while( 1 )
    {
        if( len < 64 )
        {
            union
            {
                uint8_t  bytes[64];
                uint32_t words[16];
            } tmp;
            memset( tmp.words, 0, sizeof( tmp ) );
            memcpy( tmp.bytes, msg, len );
            tmp.bytes[len] = 0x80;
            if( len < 56 )
            {
            last:
                tmp.words[15] = ENDIAN_n2b32( bitlen );
                sha256_do( state, tmp.bytes );
                int i;
                for( i = 0; i < 8; i++ )
                {
                    hash[i] = ENDIAN_n2b32( state[i] );
                }
                break;
            }
            else
            {
                sha256_do( state, tmp.bytes );
                memset( tmp.words, 0, sizeof( tmp ) );
                goto last;
            }
        }
        else
        {
            sha256_do( state, msg );
            msg += 64;
            len -= 64;
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
