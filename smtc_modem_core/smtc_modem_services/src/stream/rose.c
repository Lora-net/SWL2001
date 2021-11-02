/*!
 * \file      rose.c
 *
 * \brief     RELIABLE OCTET STREAM ENCODING (ROSE) implementation
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

#include "rose.h"

#include <string.h>

#include "modem_services_common.h"
#include "rose_defs.h"

//#include "lmic_defines.h"
//#include "board.h"
//#include "secure-element.h"
//#include "lr1mac_utilities.h"
//#include "lorawan_api.h"
//#include "smtc_crypto.h"

// TODO JLG
// Extract this in utilities
/* From TrackMac lmic.c */
/**
 * @brief Write LSB First (2 bytes)
 *
 * @param[out]  buf
 * @param[in]   v
 */
STATIC void os_wlsbf2( uint8_t* buf, uint16_t v )
{
    TEST_ASSERT_NOT_NULL( buf );
    buf[0] = v;
    buf[1] = v >> 8;
}

/**
 * @brief Write LSB First (4 bytes)
 *
 * @param[out]  buf
 * @param[in]   v
 */
STATIC void os_wlsbf4( uint8_t* buf, uint32_t v )
{
    TEST_ASSERT_NOT_NULL( buf );
    buf[0] = v;
    buf[1] = v >> 8;
    buf[2] = v >> 16;
    buf[3] = v >> 24;
}
//
//  rvec = random bit vector over wl (size: MAX(16, (wl+7)/8)
//         also serves as temp buffer for stream encryption
//  redundancy = redundancy octets draw from this area
//              initially zero, gradually filled by send operations
//              can also contain unsent data if FIFO is overloaded
//  pending_send = data to be sent yet
//  free = free FIFO buffer space
//
//
//   <--------------------------------ROSE_FIFO_SIZE---------->
//   <------wl----->                                  <--wl8-->
//   +--------------+------------------+-------------+--------+
//   |  redundancy  |.  pending_send   |.     free   |  rvec  |
//   +--------------+------------------+-------------+--------+
//                   ^                  ^
//                   |                  |
//          octet with label soff      fill
//
//
//
//  SDATA message:
//            0  -    6      7
//   byte 0  ____SYSC____|PCTXFLAG
//        1  ........SOFFL........
//        2  ........SOFFL........
//        3   systematic octets
//      ...
//   SYSC+3   redundancy octets
//      ...
//

STATIC void clearFifo( rose_t* ROSE, int off, int len )
{
    int sz = ROSE->unitsz;
    memset( &ROSE->fifo[off * sz], 0, len * sz );
}

STATIC void shiftFifo( rose_t* ROSE, int dest, int src, int len )
{
    int sz = ROSE->unitsz;
    memmove( &ROSE->fifo[dest * sz], &ROSE->fifo[src * sz], len * sz );
}

STATIC void drainFifo( rose_t* ROSE, uint8_t* dest, int src, int len )
{
    int sz = ROSE->unitsz;
    memcpy( dest, &ROSE->fifo[src * sz], len * sz );
}

STATIC_INLINE void xorUnit( rose_t* ROSE, uint8_t* dest, int destidx, const uint8_t* src, int srcidx )
{
    int sz = ROSE->unitsz;
    dest += destidx * sz;
    src += srcidx * sz;
    for( int i = 0; i < sz; i++ )
    {
        dest[i] ^= src[i];
    }
}

int ROSE_rvec_len( rose_t* ROSE )
{
    return MAX( 16, ( ROSE->wl + 7 ) / 8 );
}

// Pointer rvec buffer
STATIC_INLINE uint8_t* get_rvec( rose_t* ROSE )
{
    int rveclen = ROSE_rvec_len( ROSE );
    return &ROSE->fifo[ROSE_FIFO_SIZE - rveclen];
}

// Window length encoding parameters
STATIC const uint16_t WLENCP[] = { 16, 4, 272, 8, 784, 16, 0, 0 };  // max 1808

// Encode window length into a byte
uint8_t ROSE_encWL( uint16_t wl )
{
    for( int i = 0; i < 6; i += 2 )
    {
        uint16_t b = WLENCP[i];
        uint16_t k = WLENCP[i + 1];
        int      r = MAX( 0, wl - b + k - 1 ) / k;
        if( r <= 0x3F )
            return ( i << 5 ) + r;
    }
    return 0xBF;
}

// Decode window length into a byte length
uint16_t ROSE_decWL( uint8_t wlcode )
{
    if( wlcode > 0xBF )
        wlcode = 0xBF;
    int      i  = ( wlcode >> 5 ) & 6;
    uint16_t wl = WLENCP[i] + WLENCP[i + 1] * ( wlcode & 0x3F );
    // Restrict the growth of WL -- we don't have much RAM to play with
    if( wl > ROSE_DEFAULT_WL )
    {
        wl = ROSE_DEFAULT_WL;
    }
    return wl;
}

// Pseudo random number generator prbs23
// https://en.wikipedia.org/wiki/Pseudorandom_binary_sequence
//
STATIC_INLINE uint32_t prbs23( uint32_t x )
{
    uint32_t b0 = x & 1;
    uint32_t b1 = ( x & 0x20 ) >> 5;
    return ( x >> 1 ) + ( ( b0 ^ b1 ) << 22 );
}

// How many redundancy octets we should sent
STATIC int targetRedCnt( rose_t* ROSE )
{
    return ( ( ( uint32_t ) ROSE->wl ) * ROSE->rr + 99 ) / 100;
}

// Dilute redundancy account because n fresh octets entered
// redundancy area.
STATIC int32_t diluteRedCnt( rose_t* ROSE, uint16_t n )
{
    return ROSE->redcnt * ( ROSE->wl - n ) / ROSE->wl;
}

// Write an XOR combination of fragments into buffer pfrag
// Selection of fragment is controlled by AppCnt
//
STATIC void buildRedundancyOctets( rose_t* ROSE, uint32_t fcntup, uint8_t* redbuf, uint8_t n_units )
{
    // ASSERT(n_units <= ROSE->wl);
    uint32_t wl   = ROSE->wl;
    uint32_t wlx  = wl + ( ( ( wl - 1 ) & wl ) == 0 );  // fixup if wl=2^i => wlx = wl+1
    uint8_t* rvec = get_rvec( ROSE );                   // holds pseudo random bit vector
    uint8_t* redp = ROSE->fifo;                         // redundancy pool
    memset( redbuf, 0, n_units * ROSE->unitsz );

    for( int i = 0; i < n_units; i++ )
    {
        uint32_t nbCoeff = 0;
        uint32_t x       = 1 + ( 1001 * ( fcntup ^ ( i << 8 ) ) );
        memset( rvec, 0, &ROSE->fifo[ROSE_FIFO_SIZE] - rvec );
        while( nbCoeff < wl / 2 )
        {  // 50% 1-bits
            uint32_t r = 1 << 16;
            while( r >= wl )
            {  // only relevant for m=1
                x = prbs23( x );
                r = x % wlx;
            }
            int ri = r >> 3, rb = 1 << ( r & 7 );
            if( ( rvec[ri] & rb ) == 0 )
            {
                nbCoeff += 1;
                rvec[ri] |= rb;
                xorUnit( ROSE, redbuf, i, redp, r );
            }
        }
    }
}

STATIC void ROSE_payload_encrypt( const uint8_t* buffer, uint16_t size, uint8_t dir, uint32_t sequenceCounter,
                                  uint8_t* encBuffer )
{
    uint8_t nonce[14] = { 0 };
    // See Milestone 17 specification for nonce description
    nonce[0]  = 0x01;
    nonce[5]  = dir;
    nonce[10] = ( sequenceCounter ) &0xFF;
    nonce[11] = ( sequenceCounter >> 8 ) & 0xFF;
    nonce[12] = ( sequenceCounter >> 16 ) & 0xFF;
    nonce[13] = ( sequenceCounter >> 24 ) & 0xFF;

    smtc_modem_services_aes_encrypt( buffer, size, nonce, encBuffer );
}

void ROSE_cipher( rose_t* ROSE, uint32_t soff, uint8_t* data, uint8_t len )
{
    uint8_t* rvec = get_rvec( ROSE );  // use random bit vector as temp buffer (min size 16)
    uint8_t  off  = ( intptr_t ) data & 15;
    data -= off;
    len += off;
    while( off < len )
    {
        if( off == 0 && off + 16 <= len )
        {
            ROSE_payload_encrypt( data,            // buffer
                                  16,              // size
                                  ROSE_CRYPT_DIR,  // dir = cat
                                  soff >> 4,       // sequenceCounter
                                  data );          // encBuffer
        }
        else
        {
            int n = MIN( len - off, 16 - ( off & 15 ) );
            memcpy( rvec + off, data + off, n );
            ROSE_payload_encrypt( rvec,            // buffer
                                  16,              // size
                                  ROSE_CRYPT_DIR,  // dir = cat
                                  soff >> 4,       // sequenceCounter
                                  rvec );          // encBuffer
            memcpy( data + off, rvec + off, n );
        }
        off = ( off + 15 ) & ~15;
    }
}

int ROSE_init( rose_t* ROSE, uint16_t windowLen, uint16_t minfree, uint8_t redundancyRate, uint8_t unitsz )
{
    memset( ROSE, 0, sizeof( rose_t ) );

    if( ( unitsz != 1 && unitsz != 2 && unitsz != 4 && unitsz != 8 ) || ROSE_FIFO_SIZE % unitsz != 0 )
    {
        LOG_ERROR( "ROSE_BAD_UNITSZ\n" );
        return ROSE_BAD_UNITSZ;
    }
    uint16_t wl  = ROSE_decWL( ROSE_encWL( windowLen ) );
    ROSE->wl     = wl;
    ROSE->unitsz = unitsz;
    if( &ROSE->fifo[wl * unitsz + minfree] > get_rvec( ROSE ) )
    {
        LOG_ERROR( "ROSE_NOMEM\n" );
        return ROSE_NOMEM;
    }
    ROSE->pctxintv = ROSE_DEFAULT_PCTXINTV;
    ROSE->rr       = redundancyRate;
    // Do not initialize with targetRedCnt(wl) - although initially
    // we would not have to sent redundancy data for well known 0x00 bytes.
    // Doing so means, the first frame contains only systematic data and if that
    // is lost it creates a big whole which makes recovery harder. This useless
    // redundancy data for 0x00 bytes lasts only for a few initial frames.
    ROSE->redcnt = targetRedCnt( ROSE );
    // LOG_INFO( "INIT: ROSE->wl %d\tROSE.redcnt %d\n", ROSE->wl, ROSE->redcnt
    // );
    ROSE->fill   = wl;
    ROSE->unsent = wl;
    ROSE->soff   = 0;
    ROSE->flags  = ROSE_FIRST_DATA;
    return ROSE_OK;
}

int ROSE_enable_encryption( rose_t* ROSE )
{
    rose_rc_e rc = ROSE_ERROR;
    // Do not allow encrytion mode change in case of on going stream
    if( ROSE->flags != ROSE_FIRST_DATA )
    {
        rc = ROSE_BUSY;
    }
    else
    {
        ROSE->flags |= ROSE_CIPHER_REC;
        rc = ROSE_OK;
    }
    return rc;
}

uint32_t ROSE_getSoff( rose_t* ROSE )
{
    return ROSE->soff + ROSE->fill - ROSE->unsent;
}

uint16_t ROSE_getFree( rose_t* ROSE )
{
    return get_rvec( ROSE ) - &ROSE->fifo[ROSE->fill * ROSE->unitsz];
}

uint16_t ROSE_getPending( rose_t* ROSE )
{
    return ( ROSE->fill - ROSE->unsent ) * ROSE->unitsz;
}

int ROSE_getStatus( rose_t* ROSE )
{
    /* low latency mode will send an extra frame without systematic data but
       redundancy data to achive target redundancy rate on all current
       systematic data. This reduce latency on the expense of sending extra
       frames. */
    if( ROSE->fill > ROSE->unsent  // still unsent data
        || ( ( ROSE->flags & ROSE_LOW_LATENCY ) &&
             targetRedCnt( ROSE ) > ROSE->redcnt )  // still not reached redudancy level
        // Server asked for SINFO message or WLACK from server is pending
        || ( ROSE->flags & ( ROSE_PEND_SINFO | ROSE_PEND_WLACK ) ) != 0 )
        return ROSE_PENDTX;
    return ROSE_IDLE;
}

int ROSE_getData( rose_t* ROSE, uint32_t fcntup, uint8_t* frame, uint8_t* pTransferSize )
{
    if( ( ROSE->flags & ( ROSE_PEND_SINFO | ROSE_PEND_WLACK ) ) != 0 )
    {
        if( pTransferSize[0] < SINFO_LEN )
        {
            // FRMPayload too small to fit anything meaningfull
        toosmall:
            pTransferSize[0] = 0;
            return ROSE_LFRAME_SIZE;
        }
        uint8_t usz          = ROSE->unitsz == 1 ? 0 : ROSE->unitsz == 2 ? 1 : ROSE->unitsz == 4 ? 2 : 3;
        frame[SINFO_HDR_OFF] = SINFO_HDR_VALUE;
        frame[SINFO_FLAGS_OFF] =
            ( ( ROSE->flags & ROSE_PEND_WLACK ) ? SINFO_FLAGS_RQAWL : 0 ) | ( usz << SINFO_FLAGS_USZ_SHIFT );
        frame[SINFO_WL_OFF]       = ROSE_encWL( ROSE->wl );
        frame[SINFO_RR_OFF]       = ROSE->rr;
        frame[SINFO_PCTXINTV_OFF] = ROSE->pctxintv;
        os_wlsbf4( &frame[SINFO_SOFFL_OFF], ROSE->soff );
        pTransferSize[0] = SINFO_LEN;
        ROSE->flags &= ~ROSE_PEND_SINFO;
        return ROSE_OK;
    }
    uint32_t soff  = ROSE->soff;
    uint8_t  pctx  = ( ROSE->framecnt ? 0 : SDATA_PCTX_LEN );  // include context information
    int      avail = ( pTransferSize[0] - SDATA_HDR_LEN - pctx ) / ROSE->unitsz;
    int      sysc, redc;
    if( avail <= 0 )
    {
        goto toosmall;
    }
    //
    // 'unsent' normally sits at 'wl' unless we had a buffer overrun
    // (i.e. we just increased WL and we have a lot of unsent data)
    // In this case 'unsent' can fall below 'wl'. Before we use the
    // redundancy pool 'unsent' must be at 'wl' again.
    // Thus, drop or sent unsent octets below 'wl' with this frame if
    // we want to include redundancy octets.
    //
    if( ROSE->unsent < ROSE->wl )
    {
        // We had a buffer overrun and unsent data spilled into redundancy area
        if( ROSE->fill < ROSE->wl || !( ROSE->flags & ROSE_DROP_OVR ) )
        {
            // No space for redundant octets - we're keeping unsent in
            // redundancy area making sure it get's sent at least once as
            // systematic data
            sysc = ROSE->fill - ROSE->unsent;
            // ASSERT(sysc >= 0);
            if( sysc >= 0 )
            {
                if( sysc == 0 )
                {
                    // Can happen after WL resized bigger - wait for more data
                    // to fill up redundancy area
                    pTransferSize[0] = 0;
                    return ROSE_OK;
                }
                sysc = MIN( MIN( sysc, avail ), MAX_SYSC );
                drainFifo( ROSE, &frame[SDATA_HDR_LEN], ROSE->unsent, sysc );
                ROSE->unsent += sysc;
                ROSE->soff += sysc;
                redc = 0;
                goto addHdr;
            }
            else
            {
                return ROSE_OVERRUN;
            }
        }
        // Give up on sending overrun as systematic data - it still can
        // be recovered through redundancy data. This essentially
        // means we value older data less than newer.
        ROSE->soff   = soff += ROSE->wl - ROSE->unsent;
        ROSE->unsent = ROSE->wl;
    }
    // ASSERT(ROSE->unsent == ROSE->wl);
    int redc_target = targetRedCnt( ROSE );
    int max_sysc    = MIN( ROSE->fill - ROSE->unsent, MAX_SYSC );
    int max_redc    = MIN( ROSE->wl, MAX( 0, redc_target - ROSE->redcnt ) );
    if( max_redc > avail )
    {
        redc = avail;
        sysc = 0;
    }
    else
    {
        redc = max_redc;
        sysc = MIN( max_sysc, avail - redc );
        if( sysc + redc < avail && ( ROSE->flags & ( ROSE_FILLREDC | ROSE_FIRST_DATA ) ) == ROSE_FILLREDC )
        {
            // If frame has space fill up with redundancy octets
            redc = MIN( ROSE->wl, avail - sysc );
        }
    }
    LOG_INFO( "getData %d redc %d sysc %d redc_target %d redcnt %d\n", fcntup, redc, sysc, redc_target, ROSE->redcnt );
    if( sysc )
    {
        // Copy systematic octets into frame
        drainFifo( ROSE, &frame[SDATA_HDR_LEN], ROSE->unsent, sysc );
        ROSE->unsent += sysc;
        ROSE->soff += sysc;
    }
    if( redc )
    {
        ROSE->redcnt = MIN( redc_target, ROSE->redcnt + redc );
        LOG_INFO( "GET: ROSE->redcnt %d redc %d\n", ROSE->redcnt, redc );
        buildRedundancyOctets( ROSE, fcntup, &frame[SDATA_HDR_LEN + sysc * ROSE->unitsz], redc );
    }
    ROSE->flags &= ~ROSE_FIRST_DATA;
addHdr:
    if( ROSE->unsent > ROSE->wl )
    {
        int shift = ROSE->unsent - ROSE->wl;
        ROSE->fill -= shift;
        ROSE->unsent -= shift;
        ROSE->redcnt = diluteRedCnt( ROSE, shift );
        LOG_INFO( "DILUTE: ROSE->redcnt %d shift %d\n", ROSE->redcnt, shift );
        shiftFifo( ROSE, 0, shift, ROSE->fill );
        clearFifo( ROSE, ROSE->fill, shift );
    }
    if( sysc + redc == 0 )
    {
        pTransferSize[0] = 0;
    }
    else
    {
        pTransferSize[0]     = SDATA_HDR_LEN + ( sysc + redc ) * ROSE->unitsz;
        frame[SDATA_HDR_OFF] = sysc;
        os_wlsbf2( &frame[SDATA_SOFFL_OFF], ( uint16_t ) soff );
        if( pctx )
        {
            // Append protocol context
            frame[SDATA_HDR_OFF] |= SDATA_PCTX_FLAG;
            int off = pTransferSize[0];
            pTransferSize[0] += SDATA_PCTX_LEN;
            frame[off] = ROSE_encWL( ROSE->wl );
            os_wlsbf2( &frame[off + 1], ( uint16_t )( soff >> 16 ) );
        }
        ROSE->framecnt = ( ROSE->framecnt + 1 ) % ( ROSE->pctxintv + 1 );
    }
    return ROSE_OK;
}

int ROSE_processDnFrame( rose_t* ROSE, const uint8_t* frmpayload, uint8_t flen )
{
    if( ( frmpayload[0] & SCMD_FLAGS_SCMD ) != SCMD_FLAGS_SCMD || flen != SCMD_LEN )
    {
        return ROSE_NOTFORME;
    }
    uint8_t flags = frmpayload[SCMD_FLAGS_OFF];
    if( flags & SCMD_FLAGS_SINFO )
    {
        ROSE->flags |= ROSE_PEND_SINFO;
    }
    if( flags & SCMD_FLAGS_UPDRR )
    {
        // When we update the redundancy rate, we don't touch the internal
        // redcnt value. The algorithm will generate frames with redundancy data
        // as needed to go to the desired target redundancy. This may cause a
        // large amount of frames with NULL data when increasing the redundancy
        // rate, or a large number of frames without any redundancy data at all
        // if we decrease the rate.
        ROSE->rr = frmpayload[SCMD_RR_OFF];
    }
    if( flags & SCMD_FLAGS_UPDPCI )
    {
        ROSE->pctxintv = frmpayload[SCMD_PCTXINTV_OFF];
    }
    if( flags & SCMD_FLAGS_UPDWL )
    {
        // Current state
        //   <--------------------------------ROSE_FIFO_SIZE---------->
        //   <------WL----->                                  <--WL/8->
        //   +--------------+------------------+-------------+--------+
        //   |  redundancy  |.  pending_send   |.     free   |  rvec  |
        //   +--------------+------------------+-------------+--------+
        //                   ^                  ^
        //                   |                  |
        //                 unsent             fill
        //
        //
        // Case 1: we want to reduce WL.
        //   This is always possible, as we don't risk to overwrite pending
        //   data. We just need to shift the pending data accordingly, and
        //   update unsent and fill
        //   <--------------------------------ROSE_FIFO_SIZE---------->
        //   <---WL---->                                      <--WL/8->
        //   +----------+------------------+-----------------+--------+
        //   |  redund  |.  pending_send   |.     free       |  rvec  |
        //   +----------+------------------+-----------------+--------+
        //              ^                  ^
        //              |                  |
        //            unsent             fill
        //
        //
        // Case 2: we want to increase WL.
        //   In this case we need to ensure that the free space is big enough
        //   to accomodate the additional redundancy + rvec space before
        //   shifting
        //
        //   <--------------------------------ROSE_FIFO_SIZE----------->
        //   <---------WL-------->                           <---WL/8-->
        //   +-------------------+------------------+-------+----------+
        //   |  redundancy       |.  pending_send   |. free |    rvec  |
        //   +-------------------+------------------+-------+----------+
        //                       ^                  ^
        //                       |                  |
        //                     unsent             fill
        //
        int      wl      = ROSE_decWL( frmpayload[SCMD_WL_OFF] );
        int      rveclen = MAX( 16, ( wl + 7 ) / 8 );
        uint8_t* rvec    = &ROSE->fifo[ROSE_FIFO_SIZE - rveclen];
        if( &ROSE->fifo[ROSE->fill * ROSE->unitsz] > rvec )
        {
            // Ignore change request if bigger WL would lead
            // to overwriting FIFO contents thru increased rvec space.
            LOG_WARN( "Ignoring WL increase to avoid overwriting pending data\n" );
        }
        else
        {
            // Alright we can shift
            int shift = ROSE->wl - wl;
            if( shift > 0 )
            {
                // Here we decrease WL, we can always do it.
                ROSE->fill -= shift;
                ROSE->unsent -= shift;
                shiftFifo( ROSE, 0, shift, ROSE->fill );
                clearFifo( ROSE, ROSE->fill, shift );
                // Update redcnt
                // this is very important to ensure that we continue sending
                // redundancy data when WL is reduced
                ROSE->redcnt = diluteRedCnt( ROSE, shift );
                rvec         = get_rvec( ROSE );  // old - bigger rvec
                memset( rvec, 0, &ROSE->fifo[ROSE_FIFO_SIZE] - rvec );
            }
            // We don't shift pending data when we increase WL, because that
            // will be taken care of in ROSE_getData. Pending data has overrun
            // in the redundancy buffer, it will be sent in priority.
            ROSE->wl = wl;
            ROSE->flags |= ROSE_PEND_WLACK;
            LOG_INFO( "NEW WL: ROSE->wl %d\n", ROSE->wl );
        }
    }
    if( ( flags & SCMD_FLAGS_ACKWL ) != 0 && ROSE->wl == ROSE_decWL( frmpayload[SCMD_WL_OFF] ) )
    {
        ROSE->flags &= ~ROSE_PEND_WLACK;
    }
    return ROSE_OK;
}

int ROSE_addRecord( rose_t* ROSE, const uint8_t* data, uint16_t nbytes )
{
    if( nbytes == 0 || nbytes >= 0xFF )
        return ROSE_BAD_DATALEN;
    uint16_t n       = ( 2 + nbytes + ROSE->unitsz - 1 ) / ROSE->unitsz;
    uint16_t freeEnd = ( get_rvec( ROSE ) - &ROSE->fifo[0] ) / ROSE->unitsz;
    uint16_t free    = freeEnd - ROSE->fill;
    if( n > free )
        return ROSE_OVERRUN;
    uint8_t* p = &ROSE->fifo[ROSE->fill * ROSE->unitsz];
    memcpy( p + 1, data, nbytes );

    if( ( ROSE->flags & ROSE_CIPHER_REC ) != 0 )
    {
        ROSE_payload_encrypt( data,            // buffer
                              nbytes,          // size
                              ROSE_CRYPT_DIR,  // dir = cat
                              ROSE->soff,      // sequenceCounter
                              p + 1 );         // encBuffer
    }

    uint8_t j = n * ROSE->unitsz;
    do
    {
        p[--j] = REC_TAG;  // termination + padding
    } while( j > nbytes + 1 );
    int rj = j;  // pos of last REC_TAG
    while( --j >= 1 )
    {
        if( p[j] == REC_TAG )
        {
            p[j] = REC_TAG + ( rj - j );
            rj   = j;
        }
    }
    p[0] = REC_TAG + ( rj - j );
    ROSE->fill += n;
    return ROSE_OK;
}
