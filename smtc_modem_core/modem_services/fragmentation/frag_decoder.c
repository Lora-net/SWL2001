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

#include <stddef.h>
#include <stdbool.h>
#include "lr1mac_utilities.h"
#include "frag_decoder.h"
#include "smtc_modem_hal.h"
#include "nvmcu_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#if defined( TEST )
#define STATIC
#else
#define STATIC static
#endif

/*
 *=============================================================================
 * Fragmentation decoder algorithm utilities
 *=============================================================================
 */

/*
 * L = FRAG_MAX_SIZE
 * M = FRAG_MAX_NB
 * R = FRAG_MAX_FRAME_LOSS
 *
 *
 * Global
 *  MatrixM2B [R][R/8]          little parity matrix
 *  FragNbMissingIndex [M]      Fragment i is the Nth missing
 *  S[R/8]
 *
 * Local
 *  matrixRow [M/8]             Ci in the paper
 *  matrixDataTemp [L]          Coded fragment, Si in the paper
 *  dataTempVector [R/8]        Line of MatrixM2B
 *  dataTempVector2 [R/8]       Line of MatrixM2B
 *
 */

#if defined( UNIT_TEST_DBG )
#define PARITY_LINE_PRINT( name, array, start, cols )         \
    {                                                         \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s\t", name );          \
        for( size_t _j = start; _j < ( start + cols ); _j++ ) \
        {                                                     \
            uint8_t elem = GetParity( _j, ( array ) );        \
            if( elem == 0 )                                   \
                SMTC_MODEM_HAL_TRACE_PRINTF( ". " );          \
            else                                              \
                SMTC_MODEM_HAL_TRACE_PRINTF( "x " );          \
        }                                                     \
        SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );                 \
    }

#define PARITY_ARRAY_PRINT( name, array, rows, cols )             \
    {                                                             \
        SMTC_MODEM_HAL_TRACE_PRINTF( "\n%s\n", name );            \
        uint8_t tmp[16];                                          \
        for( size_t _i = 0; _i < ( rows ); _i++ )                 \
        {                                                         \
            FragExtractLineFromBinaryMatrix( tmp, _i, ( cols ) ); \
            PARITY_LINE_PRINT( "", tmp, 0, ( cols ) );            \
        }                                                         \
        SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );                     \
    }

#define MISSING_PRINT_LINE( array, cols )                          \
    {                                                              \
        SMTC_MODEM_HAL_TRACE_PRINTF( "Missing\t" );                \
        for( size_t _j = 0; _j < ( cols ); _j++ )                  \
        {                                                          \
            if( array[_j] )                                        \
                SMTC_MODEM_HAL_TRACE_PRINTF( "%02d ", array[_j] ); \
            else                                                   \
                SMTC_MODEM_HAL_TRACE_PRINTF( " . ", array[_j] );   \
        }                                                          \
        SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );                      \
    }

#define DATA_PRINT_FRAG( name, array, cols )                     \
    {                                                            \
        SMTC_MODEM_HAL_TRACE_PRINTF( "%s\t", name );             \
        for( size_t _j = 0; _j < ( cols ); _j++ )                \
        {                                                        \
            SMTC_MODEM_HAL_TRACE_PRINTF( "0x%02x ", array[_j] ); \
        }                                                        \
        SMTC_MODEM_HAL_TRACE_PRINTF( " \n" );                    \
    }
#else
#define PARITY_LINE_PRINT( ... )
#define PARITY_ARRAY_PRINT( ... )
#define MISSING_PRINT_LINE( ... )
#define DATA_PRINT_FRAG( ... )
#endif  // TEST

// This computes the number of bytes needed to store N bits.
#define BITARRAY_BYTES( N ) ( ( ( N ) + 7 ) >> 3 )

typedef struct
{
    FragDecoderCallbacks_t* Callbacks;
    uint16_t                FragNb;
    uint8_t                 FragSize;

    uint32_t M2BLine;

    /*
     * This is the "little" parity matrix, which is used to compute the linear combinations
     * between the uncoded fragments and the redundant ones.
     *
     * This stores a triangular superior matrix. The "PushLine" and "ExtractLine" functions
     * manage the compression and bit layout.
     * We will store at most L*(L+1)/2 bits, with L the maximum number of missing fragments
     * that we can tolerate.
     *
     * NbElems = FRAG_MAX_FRAME_LOSS * (FRAG_MAX_FRAME_LOSS + 1) / 2
     *
     * // (N+7)/8 is the correct floor function for 8 bits/byte
     * NbBytes = (NbElems + 7) >> 3
     *
     */
#define M2B_NB_ELEM ( FRAG_MAX_FRAME_LOSS * ( FRAG_MAX_FRAME_LOSS + 1 ) >> 1 )
#define M2B_STORAGE_SIZE ( BITARRAY_BYTES( M2B_NB_ELEM ) )
    uint8_t MatrixM2B[M2B_STORAGE_SIZE];

    /*
     * BitArray containing if fragment {I} is missing or not.
     * This is used to quickly check if a fragment is missing or not.
     * We could trade memory consumption with computation time by
     * iterating through FragMissingIndex every time we want to check
     * if a fragment is missing or not. The gain is small though (32 bytes for 256 fragments)
     */
#define MISSING_STORAGE_SIZE ( BITARRAY_BYTES( FRAG_MAX_NB ) )
    uint8_t FragMissing[MISSING_STORAGE_SIZE];

    /*
     * Array containing Status.FragNbLost elements.
     * When we discover that a fragment is missing, we store its fragCounter (0-indexed)
     * at the Nth position in this array.
     * I.e. if fragments #4 and #7 are missing (1-indexed), the content is [3, 6]
     * Type is a uint16_t because we might have more than 255 uncoded fragments.
     * We could also remove this if we do an exhaustive search through FragMissing, by keeping count
     * of the real index and the number of missing bits set to 1.
     */
    uint16_t FragMissingIndex[FRAG_MAX_FRAME_LOSS];

    uint8_t S[BITARRAY_BYTES( FRAG_MAX_FRAME_LOSS )];

    FragDecoderStatus_t Status;
} FragDecoder_t;

/*!
 * \brief Sets a row from source into file destination
 *
 * \param [IN] src  Source buffer pointer
 * \param [IN] row  Destination index of the row to be copied
 * \param [IN] size Source number of bytes to be copied
 */
static void SetRow( uint8_t* src, uint16_t row, uint16_t size );
/*!
 * \brief Gets a row from source and stores it into file destination
 *
 * \param [IN] src  Source buffer pointer
 * \param [IN] row  Source index of the row to be copied
 * \param [IN] size Source number of bytes to be copied
 */
static void GetRow( uint8_t* src, uint16_t row, uint16_t size );

/*!
 * \brief Gets the parity value from a given row of the parity matrix
 *
 * \param [IN] index      The index of the row to be computed
 * \param [IN] matrixRow  Pointer to the parity matrix (parity bit array)
 *
 * \retval parity         Parity value at the given index
 */
STATIC uint8_t GetParity( uint16_t index, uint8_t* matrixRow );

/*!
 * \brief Sets the parity value on the given row of the parity matrix
 *
 * \param [IN]     index     The index of the row to be computed
 * \param [IN/OUT] matrixRow Pointer to the parity matrix.
 * \param [IN]     parity    The parity value to be set in the parity matrix
 */
STATIC void SetParity( uint16_t index, uint8_t* matrixRow, uint8_t parity );

/*!
 * \brief Check if the provided value is a power of 2
 *
 * \param [IN] x  Value to be tested
 *
 * \retval status Return true if frame is a power of two
 */
static bool IsPowerOfTwo( uint32_t x );

/*!
 * \brief XOrs two data lines
 *
 * \param [IN]  line1  1st Data line to be XORed
 * \param [IN]  line2  2nd Data line to be XORed
 * \param [IN]  size   Number of elements in line1
 *
 * \param [OUT] result XOR( line1, line2 ) result stored in line1
 */
static void XorDataLine( uint8_t* line1, uint8_t* line2, int32_t size );

/*!
 * \brief XORs two parity lines
 *
 * \param [IN]  line1  1st Parity line to be XORed
 * \param [IN]  line2  2nd Parity line to be XORed
 * \param [IN]  size   Number of elements in line1
 *
 * \param [OUT] result XOR( line1, line2 ) result stored in line1
 */
static void XorParityLine( uint8_t* line1, uint8_t* line2, int32_t size );

/*!
 * \brief Generates a pseudo random number : PRBS23
 *
 * \param [IN] value The input of the PRBS23 generator
 *
 * \retval nextValue Returns the next pseudo random number
 */
static int32_t FragPrbs23( int32_t value );

/*!
 * \brief Gets and fills the parity matrix
 *
 * \param [IN]  n         Fragment N
 * \param [IN]  m         Fragment number
 * \param [OUT] matrixRow Parity matrix
 */
STATIC void FragGetParityMatrixRow( int32_t n, int32_t m, uint8_t* matrixRow );

/*!
 * \brief Finds the index of the first one in a bit array
 *
 * \param [IN] bitArray Pointer to the bit array
 * \param [IN] size     Bit array size
 * \retval index        The index of the first 1 in the bit array
 */
static uint16_t BitArrayFindFirstOne( uint8_t* bitArray, uint16_t size );

/*!
 * \brief Checks if the provided bit array only contains zeros
 *
 * \param [IN] bitArray Pointer to the bit array
 * \param [IN] size     Bit array size
 * \retval isAllZeros   [0: Contains ones, 1: Contains all zeros]
 */
static uint8_t BitArrayIsAllZeros( uint8_t* bitArray, uint16_t size );

/*!
 * \brief Finds & marks missing fragments
 *
 * \param [IN]  counter Current fragment counter
 * \param [OUT] FragDecoder.FragMissing[] array is updated in place
 * \param [OUT] FragDecoder.FragNbMissingIndex[] array is updated in place
 */
static void FragFindMissingFrags( uint16_t counter );

/*!
 * \brief Finds the index (frag counter) of the x th missing frag
 *
 * \param [IN] x   x th missing frag
 *
 * \retval counter The counter value associated to the x th missing frag
 */
static uint16_t FragFindMissingIndex( uint16_t x );

/*!
 * \brief Find the index of missing fragment x in the small table
 *
 * \param [IN] fragCounter      Number of the missing fragment (1-indexed)
 *
 * \retval index    The index of the missing fragment in the small matrix. (0-indexed)
 *                  If the index is not found, FRAG_MAX_FRAME_LOSS is returned
 *                  to indicate an error, and the caller should check this condition.
 */
static uint16_t FragFindMissing( uint16_t fragCounter );

/*!
 * \brief Extacts a row from the M2B binary matrix and expands it to a bitArray
 *
 * \param [IN] bitArray  Pointer to the bit array
 * \param [IN] rowIndex  Matrix row index           Max FRAG_MAX_FRAME_LOSS
 * \param [IN] bitsInRow Number of bits in one row. Max FRAG_MAX_FRAME_LOSS
 */
STATIC void FragExtractLineFromBinaryMatrix( uint8_t* bitArray, uint16_t rowIndex, uint16_t bitsInRow );

/*!
 * \brief Collapses and Pushs a row of a bit array to the M2B matrix
 *
 * \param [IN] bitArray  Pointer to the bit array
 * \param [IN] rowIndex  Matrix row index           Max FRAG_MAX_FRAME_LOSS
 * \param [IN] bitsInRow Number of bits in one row. Max FRAG_MAX_FRAME_LOSS
 */
STATIC void FragPushLineToBinaryMatrix( uint8_t* bitArray, uint16_t rowIndex, uint16_t bitsInRow );

/*
 *=============================================================================
 * Fragmentation decoder algorithm
 *=============================================================================
 */

static FragDecoder_t FragDecoder;

int32_t FragDecoderInit( uint16_t fragNb, uint8_t fragSize, FragDecoderCallbacks_t* callbacks )
{
    if( !callbacks || !callbacks->FragDecoderWrite || !callbacks->FragDecoderRead )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FRAG No callback defined!\n" );
        return FRAG_SESSION_ERROR;
    }

    if( fragNb > FRAG_MAX_NB || fragSize > FRAG_MAX_SIZE )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FRAG fragNb > %d || fragSize > %d\n", FRAG_MAX_NB, FRAG_MAX_SIZE );
        return FRAG_SESSION_BADSIZE;
    }

    FragDecoder.Callbacks           = callbacks;
    FragDecoder.FragNb              = fragNb;    // FragNb = FRAG_MAX_SIZE
    FragDecoder.FragSize            = fragSize;  // number of byte on a row
    FragDecoder.Status.FragNbRx     = 0;
    FragDecoder.Status.FragNbLastRx = 0;
    FragDecoder.Status.FragNbLost   = 0;
    FragDecoder.M2BLine             = 0;

    // Initialize missing fragments index array
    for( uint16_t i = 0; i < FRAG_MAX_FRAME_LOSS; i++ )
    {
        FragDecoder.FragMissingIndex[i] = 0;
    }
    for( size_t i = 0; i < MISSING_STORAGE_SIZE; i++ )
    {
        FragDecoder.FragMissing[i] = 0;
    }

    // Initialize parity matrix
    for( uint32_t i = 0; i < BITARRAY_BYTES( FRAG_MAX_FRAME_LOSS ); i++ )
    {
        FragDecoder.S[i] = 0;
    }

    for( uint32_t i = 0; i < M2B_STORAGE_SIZE; i++ )
    {
        FragDecoder.MatrixM2B[i] = 0xFF;
    }

    SMTC_MODEM_HAL_TRACE_INFO( "Missing %3d bytes\n", MISSING_STORAGE_SIZE );
    SMTC_MODEM_HAL_TRACE_INFO( "MIndex  %3d bytes\n", FRAG_MAX_FRAME_LOSS );
    SMTC_MODEM_HAL_TRACE_INFO( "M2B     %3d bytes\n", M2B_STORAGE_SIZE );

    // Initialize final uncoded data buffer ( FRAG_MAX_NB * FRAG_MAX_SIZE )
    // erase Delta update storage pages
    for( uint16_t page = 109; page < 124; page++ )
    {
        if( FlashErasePage( page, 0 ) != 1 )
        {
            SMTC_MODEM_HAL_TRACE_INFO( "Erase error page %d:\n", page );
            return FRAG_SESSION_MEM_ERROR;
        }
    }
    FragDecoder.Status.FragNbLost   = 0;
    FragDecoder.Status.FragNbLastRx = 0;

    SMTC_MODEM_HAL_TRACE_INFO( "FragDecoderInit %d %d\n", FragDecoder.FragNb, FragDecoder.FragSize );
    return FRAG_SESSION_OK;
}

uint32_t FragDecoderGetMaxFileSize( void )
{
    return FRAG_MAX_NB * FRAG_MAX_SIZE;
}

FragDecoderSessionStatus_t FragDecoderProcess( uint16_t fragCounter, uint8_t* rawData )
{
    uint16_t firstOneInRow = 0;
    int32_t  first         = 0;
    int32_t  noInfo        = 0;

    uint8_t matrixRow[( FRAG_MAX_NB >> 3 ) + 1];
    uint8_t matrixDataTemp[FRAG_MAX_SIZE];
    uint8_t dataTempVector[( FRAG_MAX_FRAME_LOSS >> 3 ) + 1];
    uint8_t dataTempVector2[( FRAG_MAX_FRAME_LOSS >> 3 ) + 1];

    memset1( matrixRow, 0, ( FRAG_MAX_NB >> 3 ) + 1 );
    memset1( matrixDataTemp, 0, FRAG_MAX_SIZE );
    memset1( dataTempVector, 0, ( FRAG_MAX_FRAME_LOSS >> 3 ) + 1 );
    memset1( dataTempVector2, 0, ( FRAG_MAX_FRAME_LOSS >> 3 ) + 1 );

    SMTC_MODEM_HAL_TRACE_INFO( "FragProcess cnt %d nb_frag %d frag_size %d\n", fragCounter, FragDecoder.FragNb,
                               FragDecoder.FragSize );

    if( rawData == NULL )
    {
        return FRAG_SESSION_ERROR;
    }

    if( fragCounter < 1 )
    {
        return FRAG_SESSION_ERROR;
    }

    // This stores the number of really received fragments. Not used in the algorithm,
    // only in debug messages.
    FragDecoder.Status.FragNbRx += 1;

    if( fragCounter <= FragDecoder.Status.FragNbLastRx )
    {
        return FRAG_SESSION_ONGOING;  // Drop frame out of order
    }

    // The M (FragNb) first packets aren't encoded or in other words they are
    // encoded with the unitary matrix
    if( fragCounter <= FragDecoder.FragNb )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "Frame %d not encoded - directly store it and keep going\n", fragCounter );

        // The M first frame are not encoded store them
        SetRow( rawData, fragCounter - 1, FragDecoder.FragSize );

        SetParity( fragCounter - 1, FragDecoder.FragMissing, 0 );

        // Update the FragDecoder.FragMissing with the lost frames since the last Rx
        FragFindMissingFrags( fragCounter );

        if( fragCounter == FragDecoder.FragNb && FragDecoder.Status.FragNbLost == 0 )
        {
            // the case : all the M(FragNb) first rows have been transmitted with no error
            SMTC_MODEM_HAL_TRACE_INFO( "[OK] All uncoded fragments have been received - no need to continue\n" );
            return FRAG_SESSION_OK;
        }

        return FRAG_SESSION_ONGOING;
    }

    // In case of the end of true data is missing
    FragFindMissingFrags( fragCounter );

    // It will be impossible to reconstruct the original data
    if( FragDecoder.Status.FragNbLost > FRAG_MAX_FRAME_LOSS )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Lost too many fragments\n" );
        FragDecoder.Status.MatrixError = 1;
        return FRAG_SESSION_ABORT;
    }

    // At this point we receive encoded frames and the number of lost frames is well known
    FragGetParityMatrixRow( fragCounter, FragDecoder.FragNb, matrixRow );
    SMTC_MODEM_HAL_TRACE_INFO( "Get parity matrix row %d\n", fragCounter );
    PARITY_LINE_PRINT( "matrixRow", matrixRow, 0, FragDecoder.FragNb );

    SMTC_MODEM_HAL_TRACE_INFO( "Checking if this fragments brings interesting information\n" );
    DATA_PRINT_FRAG( "Raw", rawData, FragDecoder.FragSize );
    for( int32_t i = 0; i < FragDecoder.FragNb; i++ )
    {
        if( GetParity( i, matrixRow ) == 1 )  // This fragment brings potentially new data for missing frag i
        {
            if( GetParity( i, FragDecoder.FragMissing ) == 0 )  // Nope, already received
            {
                SetParity( i, matrixRow, 0 );
                GetRow( matrixDataTemp, i, FragDecoder.FragSize );
                XorDataLine( rawData, matrixDataTemp, FragDecoder.FragSize );
                SMTC_MODEM_HAL_TRACE_INFO( "Fragment %d already received\n", i + 1 );
                DATA_PRINT_FRAG( "XOR", rawData, FragDecoder.FragSize );
            }
            else  // New unknown data, store it somewhere
            {
                // Fill the "little" boolean matrix m2b
                // - Fragment {fragCounter} can give information on fragment {i}.
                // - Fragment {i} is the {n}th missing, we need to find n
                // Warning, we need to give the real fragCounter (1-indexed)
                uint16_t nth = FragFindMissing( i + 1 );
                if( nth >= FRAG_MAX_FRAME_LOSS )
                {
                    // We didn't find it, maybe we have too many frames lost?
                    // We panic, because this should really not happen
                    // and means we have a deeper source of errors.
                    smtc_modem_hal_mcu_panic( "Could not find missing fragment %d in FragMissingIndex\n", i + 1 );
                }

                // - We store that this fragment {fragCounter} can retrieve data for the {n}th in dataTempVector
                SMTC_MODEM_HAL_TRACE_INFO(
                    "Fragment %d could bring new data for fragment %d (missing #%d) (total %d)\n", fragCounter, i + 1,
                    nth, FragDecoder.Status.FragNbLost );
                SMTC_MODEM_HAL_TRACE_INFO( "SetParity for missing %d\n", nth );

                SetParity( nth, dataTempVector, 1 );
                if( first == 0 )
                {
                    // Used to tell that we received at least one useful redundant fragment
                    first = 1;
                }
            }
        }
    }
    PARITY_LINE_PRINT( "matrixRow", matrixRow, 0, FragDecoder.FragNb );

    PARITY_LINE_PRINT( "dataTempVector", dataTempVector, 0, FRAG_MAX_FRAME_LOSS );
    firstOneInRow = BitArrayFindFirstOne( dataTempVector, FragDecoder.Status.FragNbLost );

    SMTC_MODEM_HAL_TRACE_INFO( "first %d firstOneInRow %d\n", first, firstOneInRow + 1 );

    if( first > 0 )
    {
        int32_t li;
        int32_t lj;

        // Manage a new line in MatrixM2B
        PARITY_LINE_PRINT( "S", FragDecoder.S, 0, FRAG_MAX_FRAME_LOSS );
        while( GetParity( firstOneInRow, FragDecoder.S ) == 1 )
        {
            // Row already diagonalized exist & ( FragDecoder.MatrixM2B[firstOneInRow][0] )
            FragExtractLineFromBinaryMatrix( dataTempVector2, firstOneInRow, FragDecoder.Status.FragNbLost );
            PARITY_LINE_PRINT( "dataTempVector2", dataTempVector2, 0, FragDecoder.Status.FragNbLost );
            XorParityLine( dataTempVector, dataTempVector2, FragDecoder.Status.FragNbLost );

            // Have to store it in the mi th position of the missing frag
            li = FragFindMissingIndex( firstOneInRow );
            GetRow( matrixDataTemp, li, FragDecoder.FragSize );
            XorDataLine( rawData, matrixDataTemp, FragDecoder.FragSize );
            DATA_PRINT_FRAG( "XOR2", rawData, FragDecoder.FragSize );
            if( BitArrayIsAllZeros( dataTempVector, FragDecoder.Status.FragNbLost ) )
            {
                noInfo = 1;
                break;
            }
            firstOneInRow = BitArrayFindFirstOne( dataTempVector, FragDecoder.Status.FragNbLost );
        }

        if( noInfo == 0 )
        {
            // Store the raw data into the final file, to retrieve later
            FragPushLineToBinaryMatrix( dataTempVector, firstOneInRow, FragDecoder.Status.FragNbLost );
            li = FragFindMissingIndex( firstOneInRow );
            SMTC_MODEM_HAL_TRACE_INFO( "SetRow %d (fragment %d)\n", li, li + 1 );
            SetRow( rawData, li, FragDecoder.FragSize );
            SetParity( firstOneInRow, FragDecoder.S, 1 );
            FragDecoder.M2BLine++;
            DATA_PRINT_FRAG( "SAVE", rawData, FragDecoder.FragSize );
        }

        if( FragDecoder.M2BLine == FragDecoder.Status.FragNbLost )
        {
            // Then last step diagonalized
            // Step 5 from the paper
            if( FragDecoder.Status.FragNbLost > 1 )
            {
                int32_t i, j;

                for( i = ( FragDecoder.Status.FragNbLost - 2 ); i >= 0; i-- )
                {
                    li = FragFindMissingIndex( i );
                    GetRow( matrixDataTemp, li, FragDecoder.FragSize );
                    for( j = ( FragDecoder.Status.FragNbLost - 1 ); j > i; j-- )
                    {
                        FragExtractLineFromBinaryMatrix( dataTempVector2, i, FragDecoder.Status.FragNbLost );
                        FragExtractLineFromBinaryMatrix( dataTempVector, j, FragDecoder.Status.FragNbLost );
                        if( GetParity( j, dataTempVector2 ) == 1 )
                        {
                            XorParityLine( dataTempVector2, dataTempVector, FragDecoder.Status.FragNbLost );

                            lj = FragFindMissingIndex( j );

                            GetRow( rawData, lj, FragDecoder.FragSize );
                            XorDataLine( matrixDataTemp, rawData, FragDecoder.FragSize );
                        }
                    }
                    SetRow( matrixDataTemp, li, FragDecoder.FragSize );
                }
            }

            SMTC_MODEM_HAL_TRACE_INFO( "Session reconstructed, FragNbLost %d\n", FragDecoder.Status.FragNbLost );
            return FRAG_SESSION_OK;
        }
    }

    SMTC_MODEM_HAL_TRACE_INFO( "No new information -- return FRAG_SESSION_ONGOING\n" );
    return FRAG_SESSION_ONGOING;
}

FragDecoderStatus_t FragDecoderGetStatus( void )
{
    return FragDecoder.Status;
}

uint32_t FragDecoderFileSize( void )
{
    uint32_t size = FragDecoder.FragNb * FragDecoder.FragSize;
    SMTC_MODEM_HAL_TRACE_INFO( "FileSize NB %d Size %d total %d\n", FragDecoder.FragNb, FragDecoder.FragSize, size );
    return size;
}

/*
 *=============================================================================
 * Fragmentation decoder algorithm utilities
 *=============================================================================
 */

static void SetRow( uint8_t* src, uint16_t row, uint16_t size )
{
    if( ( FragDecoder.Callbacks != NULL ) && ( FragDecoder.Callbacks->FragDecoderWrite != NULL ) )
    {
        FragDecoder.Callbacks->FragDecoderWrite( row * size, src, size );
    }
}

static void GetRow( uint8_t* dst, uint16_t row, uint16_t size )
{
    if( ( FragDecoder.Callbacks != NULL ) && ( FragDecoder.Callbacks->FragDecoderRead != NULL ) )
    {
        FragDecoder.Callbacks->FragDecoderRead( row * size, dst, size );
    }
}

STATIC uint8_t GetParity( uint16_t index, uint8_t* matrixRow )
{
    uint8_t parity;
    parity = matrixRow[index >> 3];
    parity = ( parity >> ( 7 - ( index % 8 ) ) ) & 0x01;
    return parity;
}

STATIC void SetParity( uint16_t index, uint8_t* matrixRow, uint8_t parity )
{
    uint8_t mask          = 0xFF - ( 1 << ( 7 - ( index % 8 ) ) );
    parity                = parity << ( 7 - ( index % 8 ) );
    matrixRow[index >> 3] = ( matrixRow[index >> 3] & mask ) + parity;
}

static bool IsPowerOfTwo( uint32_t x )
{
    uint8_t sumBit = 0;

    for( uint8_t i = 0; i < 32; i++ )
    {
        sumBit += ( x & ( 1 << i ) ) >> i;
    }
    if( sumBit == 1 )
    {
        return true;
    }
    return false;
}

static void XorDataLine( uint8_t* line1, uint8_t* line2, int32_t size )
{
    for( int32_t i = 0; i < size; i++ )
    {
        line1[i] = line1[i] ^ line2[i];
    }
}

static void XorParityLine( uint8_t* line1, uint8_t* line2, int32_t size )
{
    for( int32_t i = 0; i < size; i++ )
    {
        SetParity( i, line1, ( GetParity( i, line1 ) ^ GetParity( i, line2 ) ) );
    }
}

static int32_t FragPrbs23( int32_t value )
{
    int32_t b0 = value & 0x01;
    int32_t b1 = ( value & 0x20 ) >> 5;
    return ( value >> 1 ) + ( ( b0 ^ b1 ) << 22 );
    ;
}

STATIC void FragGetParityMatrixRow( int32_t n, int32_t m, uint8_t* matrixRow )
{
    int32_t mTemp;
    int32_t x;
    int32_t nbCoeff = 0;
    int32_t r;

    if( IsPowerOfTwo( m ) != false )
    {
        mTemp = 1;
    }
    else
    {
        mTemp = 0;
    }

    x = 1 + ( 1001 * n );
    for( uint32_t i = 0; i < ( ( m >> 3 ) + 1 ); i++ )
    {
        matrixRow[i] = 0;
    }
    while( nbCoeff < ( m >> 1 ) )
    {
        r = 1 << 16;
        while( r >= m )
        {
            x = FragPrbs23( x );
            r = x % ( m + mTemp );
        }
        if( GetParity( r, matrixRow ) == 0 )
        {
            SetParity( r, matrixRow, 1 );
            nbCoeff += 1;
        }
    }
}

static uint16_t BitArrayFindFirstOne( uint8_t* bitArray, uint16_t size )
{
    for( uint16_t i = 0; i < size; i++ )
    {
        if( GetParity( i, bitArray ) == 1 )
        {
            return i;
        }
    }
    return 0;
}

static uint8_t BitArrayIsAllZeros( uint8_t* bitArray, uint16_t size )
{
    for( uint16_t i = 0; i < size; i++ )
    {
        if( GetParity( i, bitArray ) == 1 )
        {
            return 0;
        }
    }
    return 1;
}

/*!
 * \brief Finds & marks missing fragments
 *
 * \param [IN]  counter Current fragment counter
 * \param [OUT] FragDecoder.FragNbMissingIndex[] array is updated in place
 */
static void FragFindMissingFrags( uint16_t counter )
{
    int32_t i;
    for( i = FragDecoder.Status.FragNbLastRx; i < ( counter - 1 ); i++ )
    {
        if( i < FragDecoder.FragNb )
        {
            SMTC_MODEM_HAL_TRACE_INFO( "Fragment %d is missing, store it at index %d\n", i + 1, i );
            SetParity( i, FragDecoder.FragMissing, 1 );
            // Nth missing fragment is number i+1 (we keep the 0-indexed value)
            FragDecoder.FragMissingIndex[FragDecoder.Status.FragNbLost] = i;
            FragDecoder.Status.FragNbLost++;
        }
    }
    if( i < FragDecoder.FragNb )
    {
        FragDecoder.Status.FragNbLastRx = counter;
    }
    else
    {
        FragDecoder.Status.FragNbLastRx = FragDecoder.FragNb + 1;
    }
    SMTC_MODEM_HAL_TRACE_INFO( "RECEIVED    : %5d / %5d Fragments\r\n", FragDecoder.Status.FragNbRx,
                               FragDecoder.FragNb );
    SMTC_MODEM_HAL_TRACE_INFO( "              %5d / %5d Bytes\r\n", FragDecoder.Status.FragNbRx * FragDecoder.FragSize,
                               FragDecoder.FragNb * FragDecoder.FragSize );
    SMTC_MODEM_HAL_TRACE_INFO( "LOST        :       %7d Fragments\r\n\r\n", FragDecoder.Status.FragNbLost );
}

/*!
 * \brief Finds the index (frag counter) of the x th missing frag
 *
 * \param [IN] x   x th missing frag. Max FRAG_MAX_FRAME_LOSS
 *
 * \retval counter The counter value associated to the x th missing frag
 */
static uint16_t FragFindMissingIndex( uint16_t x )
{
    SMTC_MODEM_HAL_TRACE_INFO( "FragFindMissingIndex x %d -> %d\n", x, FragDecoder.FragMissingIndex[x] );
    return FragDecoder.FragMissingIndex[x];
}

/*!
 * \brief Find the index of missing fragment x in the small table
 *
 * \param [IN] fragCounter      Number of the missing fragment (1-indexed)
 *
 * \retval index    The index of the missing fragment in the small matrix. (0-indexed)
 *                  If the index is not found, FRAG_MAX_FRAME_LOSS is returned
 *                  to indicate an error, and the caller should check this condition.
 */
static uint16_t FragFindMissing( uint16_t fragCounter )
{
    for( uint16_t i = 0; i < FragDecoder.Status.FragNbLost; i++ )
    {
        if( FragDecoder.FragMissingIndex[i] == fragCounter - 1 )
        {
            return i;
        }
    }
    return FRAG_MAX_FRAME_LOSS;
}

/*!
 * \brief Extacts a row from the binary matrix and expands it to a bitArray
 * Only extracts the triangular sup part of the matrix. So all bits left
 * of the triangle are 0.
 *
 * \param [OUT] bitArray  Pointer to the bit array
 * \param [IN] rowIndex  Matrix row index           Max FRAG_MAX_FRAME_LOSS
 * \param [IN] bitsInRow Number of bits in one row. Max FRAG_MAX_FRAME_LOSS
 */
STATIC void FragExtractLineFromBinaryMatrix( uint8_t* bitArray, uint16_t rowIndex, uint16_t bitsInRow )
{
    uint32_t findByte      = 0;
    uint32_t findBitInByte = 0;

    if( rowIndex > 0 )
    {
        findByte      = ( rowIndex * bitsInRow - ( ( rowIndex * ( rowIndex - 1 ) ) >> 1 ) ) >> 3;
        findBitInByte = ( rowIndex * bitsInRow - ( ( rowIndex * ( rowIndex - 1 ) ) >> 1 ) ) % 8;
    }
    if( rowIndex > 0 )
    {
        for( uint16_t i = 0; i < rowIndex; i++ )
        {
            SetParity( i, bitArray, 0 );
        }
    }
    for( uint16_t i = rowIndex; i < bitsInRow; i++ )
    {
        SetParity( i, bitArray, ( FragDecoder.MatrixM2B[findByte] >> ( 7 - findBitInByte ) ) & 0x01 );

        findBitInByte++;
        if( findBitInByte == 8 )
        {
            findBitInByte = 0;
            findByte++;
        }
    }
}

/*!
 * \brief Collapses and Pushs a row of a bit array to the matrix
 * Only store the triangular sup part of the matrix. All bits left of the triangle are ignored
 *
 * \param [IN] bitArray  Pointer to the bit array
 * \param [IN] rowIndex  Matrix row index.          Max FRAG_MAX_FRAME_LOSS
 * \param [IN] bitsInRow Number of bits in one row. Max FRAG_MAX_FRAME_LOSS
 */
STATIC void FragPushLineToBinaryMatrix( uint8_t* bitArray, uint16_t rowIndex, uint16_t bitsInRow )
{
    uint32_t findByte      = 0;
    uint32_t findBitInByte = 0;

    if( rowIndex > 0 )
    {
        findByte      = ( rowIndex * bitsInRow - ( ( rowIndex * ( rowIndex - 1 ) ) >> 1 ) ) >> 3;
        findBitInByte = ( rowIndex * bitsInRow - ( ( rowIndex * ( rowIndex - 1 ) ) >> 1 ) ) % 8;
    }
    SMTC_MODEM_HAL_TRACE_PRINTF( "PushLine row %d nb_bits %d | findByte %d bitInByte %d\n", rowIndex, bitsInRow,
                                 findByte, findBitInByte );

    for( uint16_t i = rowIndex; i < bitsInRow; i++ )
    {
        if( GetParity( i, bitArray ) == 0 )
        {
            FragDecoder.MatrixM2B[findByte] =
                FragDecoder.MatrixM2B[findByte] & ( 0xFF - ( 1 << ( 7 - findBitInByte ) ) );
        }
        findBitInByte++;
        if( findBitInByte == 8 )
        {
            findBitInByte = 0;
            findByte++;
        }
    }
    PARITY_ARRAY_PRINT( "M2B", FragDecoder.MatrixM2B, bitsInRow, bitsInRow );
}
