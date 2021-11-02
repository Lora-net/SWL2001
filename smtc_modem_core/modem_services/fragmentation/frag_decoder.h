/*!
 * \file      FragDecoder.h
 *
 * \brief     Implements the LoRa-Alliance fragmentation decoder
 *            Specification:
 * https://lora-alliance.org/sites/default/files/2018-09/fragmented_data_block_transport_v1.0.0.pdf
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
#ifndef __FRAG_DECODER_H__
#define __FRAG_DECODER_H__

#include <stdint.h>
#include <errno.h>

/*
 * The following parameters have an impact on the memory footprint.
 * The major contributors are the parity matrix and missing fragment index.
 *
 * Heap size >=   FRAG_MAX_FRAME_LOSS * (FRAG_MAX_FRAME_LOSS + 1) / 2 / 8
 *              + 2 * FRAG_MAX_FRAME_LOSS
 *              + FRAG_MAX_NB / 8
 *
 * Stack size >= FRAG_MAX_NB
 */

/*!
 * Maximum number of uncoded fragment that can be handled.
 *
 * \remark This parameter has an impact on the heap memory footprint.
 *         It defines the size of the bitarray of missing fragments.
 */
#define FRAG_MAX_NB 150

/*!
 * Maximum fragment size that can be handled.
 *
 * \remark This parameter has a slight impact on the stack memory footprint,
 *         as a buffer of this size is necessary for the decoding computation.
 */
#define FRAG_MAX_SIZE 200

/*!
 * Maximum number of uncoded fragments that can be lost.
 * This determines the resistance of the algorithm wrt. packet loss
 *
 * \remark This parameter has an impact on the heap memory footprint.
 *         It defines the size of the parity matrix and of the missing fragment index array.
 */
#define FRAG_MAX_FRAME_LOSS 64

/*!
 * \brief This return code indicates the state of the session
 */
typedef enum
{
    FRAG_SESSION_OK = 0,
    FRAG_SESSION_ONGOING,
    FRAG_SESSION_ERROR,
    FRAG_SESSION_BADSIZE,
    FRAG_SESSION_ABORT,
    FRAG_SESSION_MEM_ERROR,
} FragDecoderSessionStatus_t;

/*!
 * \brief This stores the current session statistics
 */
typedef struct sFragDecoderStatus
{
    uint16_t FragNbRx;
    uint16_t FragNbLost;
    uint16_t FragNbLastRx;
    uint8_t  MatrixError;
} FragDecoderStatus_t;

/*!
 * \brief The callbacks are used to read/write fragments in the final data block memory.
 *        This allows the client code to use Flash memory as the destination.
 */
typedef struct sFragDecoderCallbacks
{
    /*!
     * Writes `data` buffer of `size` starting at address `addr`
     *
     * \param [IN] addr Address start index to write to.
     * \param [IN] data Data buffer to be written.
     * \param [IN] size Size of data buffer to be written.
     *
     * \retval status Write operation status [0: Success, -1 Fail]
     */
    int8_t ( *FragDecoderWrite )( uint32_t addr, uint8_t* data, uint32_t size );
    /*!
     * Reads `data` buffer of `size` starting at address `addr`
     *
     * \param [IN] addr Address start index to read from.
     * \param [OUT] data Data buffer to be read.
     * \param [IN] size Size of data buffer to be read.
     *
     * \retval status Read operation status [0: Success, -1 Fail]
     */
    int8_t ( *FragDecoderRead )( uint32_t addr, uint8_t* data, uint32_t size );
} FragDecoderCallbacks_t;

/*!
 * \brief Initializes the fragmentation decoder
 *
 * \param [IN] fragNb     Number of expected fragments (without redundancy packets)
 * \param [IN] fragSize   Size of a fragment
 * \param [IN] callbacks  Pointer to the Write/Read functions.
 */
int32_t FragDecoderInit( uint16_t fragNb, uint8_t fragSize, FragDecoderCallbacks_t* callbacks );
/*!
 * \brief Gets the maximum file size that can be received
 *
 * \retval size FileSize
 */
uint32_t FragDecoderGetMaxFileSize( void );

/*!
 * \brief Gets the current file size that is configured in the decoding session
 *
 * \retval size FileSize
 */
uint32_t FragDecoderFileSize( void );

/*!
 * \brief Function to decode and reconstruct the binary file
 *        Called for each receive frame
 *
 * \param [IN]  fragCounter Fragment counter [1..(FragDecoder.FragNb + FragDecoder.Redundancy)]
 * \param [IN]  rawData     Pointer to the fragment to be processed (length = FragDecoder.FragSize)
 * \param [OUT] nbLost      Number of non-coded packets lost
 *
 * \retval status          Process status.
 */
FragDecoderSessionStatus_t FragDecoderProcess( uint16_t fragCounter, uint8_t* rawData );

/*!
 * \brief Gets the current fragmentation status
 *
 * \retval status Fragmentation decoder status
 */
FragDecoderStatus_t FragDecoderGetStatus( void );

#if defined( TEST )
// This is only accessible during unit testing.

/*!
 * \brief Gets the parity matrix row for fragment N (M non-coded fragments)
 *
 * \param [IN] n            Current fragCounter
 * \param [IN] m            Number of non-coded fragments
 * \param [OUT] matrixRow   Destination array
 */
void FragGetParityMatrixRow( int32_t n, int32_t m, uint8_t* matrixRow );

/*!
 * \brief Gets the binary triangular-sup matrix row from M2B
 *
 * \param [OUT] bitArray   Destination array
 * \param [IN] rowIndex    Index of the requested row
 * \param [IN] bitsInRow   Size of the matrix
 */
void FragExtractLineFromBinaryMatrix( uint8_t* bitArray, uint16_t rowIndex, uint16_t bitsInRow );

/*!
 * \brief Store the binary triangular-sup matrix row to M2B
 *
 * \param [OUT] bitArray   Source array
 * \param [IN] rowIndex    Index of the requested row
 * \param [IN] bitsInRow   Size of the matrix
 */
void FragPushLineToBinaryMatrix( uint8_t* bitArray, uint16_t rowIndex, uint16_t bitsInRow );

/*!
 * \brief Gets the bit stored in a binary array
 *
 * \param [IN] index        Index of the requested bit
 * \param [IN] matrixRow    Source array
 */
uint8_t GetParity( uint16_t index, uint8_t* matrixRow );

/*!
 * \brief Sets the bit stored in a binary array
 *
 * \param [IN] index        Index of the requested bit
 * \param [IN] matrixRow    Destination array
 */
void SetParity( uint16_t index, uint8_t* matrixRow, uint8_t parity );
#endif

#endif  // __FRAG_DECODER_H__
