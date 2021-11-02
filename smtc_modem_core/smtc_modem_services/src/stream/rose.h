/*!
 * \file      rose.h
 *
 * \brief     RELIABLE OCTET STREAM ENCODING (ROSE) public interface
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

#ifndef __ROSE_H__
#define __ROSE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* DO NOT TOUCH WL */
#define ROSE_DEFAULT_WL 512       // default window length
#define ROSE_DEFAULT_MINFREE 512  // free buffer
#define ROSE_DEFAULT_RR 110       // default redundancy rate (110%)
#define ROSE_DEFAULT_PCTXINTV 8   // include protocol context in every N+1st frame

#ifndef ROSE_FIFO_SIZE
#if defined( CFG_simul )
#define ROSE_FIFO_SIZE 10240  // bigger - for performance analysis
#else
#define ROSE_FIFO_SIZE                         \
    ( ROSE_DEFAULT_WL + ROSE_DEFAULT_MINFREE + \
      ROSE_DEFAULT_WL / 8 )  // big enough for wl=512 minfree=512 + rvec for wl=512
#endif
#endif

/*!
 *  \brief ROSE Status codes
 */
typedef enum
{
    ROSE_OK     = 0,
    ROSE_IDLE   = 1,
    ROSE_PENDTX = 2,

    ROSE_OVERRUN     = -1,        // buffer overrun, some data got dropped
    ROSE_NOMEM       = -2,        // FIFO too small for app data size + window size
    ROSE_LFRAME_SIZE = -3,        // Lora frame is too small
    ROSE_BAD_UNITSZ  = -4,        // Illegal unit size
    ROSE_BAD_DATALEN = -5,        // Data length is not a multiple of unitsz or not a legal record length
    ROSE_NOTFORME    = -6,        // DN frame is not for ROSE engine (maybe forward to app)
    ROSE_BUSY        = -7,        //
    ROSE_ERROR       = INT8_MIN,  // Generic error
} rose_rc_e;

/*!
 *  \brief ROSE flags
 */
typedef enum
{                             // flags:
    ROSE_DROP_OVR    = 0x01,  // alternate strategy to handle overruns
    ROSE_PEND_SINFO  = 0x02,  // server asked to send info mesage
    ROSE_PEND_WLACK  = 0x04,  // waiting for server to ACK WL update
    ROSE_CIPHER_REC  = 0x08,  // encipher records
    ROSE_FIRST_DATA  = 0x10,  // very first data sent
    ROSE_FILLREDC    = 0x20,  // if frame has space do fill with extra redundancy
    ROSE_LOW_LATENCY = 0x40,  // operate in low latency mode
} rose_flag_e;

/*!
 *  \brief ROSE structure
 */
typedef struct rose_s
{
    uint8_t  flags;
    uint8_t  pctxintv;  // include protocol context every Nth frame
    uint8_t  framecnt;  // frame counter to include protocol context
    uint8_t  rr;        // current redundancy rate
    uint32_t soff;      // stream offset label (of unsent position)
    int      redcnt;    // how many redundancy octets have been sent over redundancy pool
    uint16_t wl;        // window length
    uint16_t unsent;    // start of unsent systematic data
    uint16_t fill;      // start of free buffer space
    uint8_t  unitsz;
    uint8_t  fifo[ROSE_FIFO_SIZE];
} rose_t;

// minfree in bytes
int ROSE_init( rose_t* ROSE, uint16_t windowLen, uint16_t minfree, uint8_t redundancyRate, uint8_t unitsz );
int ROSE_enable_encryption( rose_t* ROSE );

int  ROSE_addData( const uint8_t* data, uint16_t n );  // n in bytes, but n%unitsz ==0
int  ROSE_getData( rose_t* ROSE, uint32_t fcntup, uint8_t* frmpayload, uint8_t* transferSize );
void ROSE_cipher( rose_t* ROSE, uint32_t soff, uint8_t* data, uint8_t len );
int  ROSE_processDnFrame( rose_t* ROSE, const uint8_t* frmpayload, uint8_t flen );
int  ROSE_addRecord( rose_t* ROSE, const uint8_t* data, uint16_t nbytes );

uint8_t  ROSE_encWL( uint16_t wl );        // encode window length into a byte (wl rounded up to next std length)
uint16_t ROSE_decWL( uint8_t wlcode );     // decode byte into window length
uint16_t ROSE_getPending( rose_t* ROSE );  // number of bytes awaiting transmit
uint16_t ROSE_getFree( rose_t* ROSE );     // get free buffer space in bytes
int      ROSE_getStatus( rose_t* ROSE );
uint32_t ROSE_getSoff( rose_t* ROSE );  // stream offset of 1st byte of next addData
int      ROSE_rvec_len( rose_t* ROSE );

#endif  // __ROSE_H__
