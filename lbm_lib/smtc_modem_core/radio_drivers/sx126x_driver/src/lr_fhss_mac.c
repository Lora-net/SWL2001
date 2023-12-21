/**
 * @file      lr_fhss_mac.c
 *
 * @brief     Radio-independent LR-FHSS driver implementation
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
#include "lr_fhss_mac.h"
#include <string.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifdef TEST
#define STATIC
#else
#define STATIC static
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR_FHSS_MAX_TMP_BUF_BYTES ( 608 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/** @brief Channel count as function of bandwidth index, from Table 9 specification v18 */
STATIC const uint16_t lr_fhss_channel_count[] = { 80, 176, 280, 376, 688, 792, 1480, 1584, 3120, 3224 };

/** @brief Generating polynomial as function of polynomial index, n_grid in { 10, 22, 28, 30, 35, 47 } */
STATIC const uint8_t lr_fhss_lfsr_poly1[] = { 33, 45, 48, 51, 54, 57 };

/** @brief Generating polynomial as function of polynomial index, n_grid in { 86, 99 } */
STATIC const uint8_t lr_fhss_lfsr_poly2[] = { 65, 68, 71, 72 };

/** @brief Generating polynomial as function of polynomial index, n_grid in { 185, 198 } */
STATIC const uint8_t lr_fhss_lfsr_poly3[] = { 142, 149 };

/** @brief used for 1/3 rate viterbi encoding */
STATIC const uint8_t lr_fhss_viterbi_1_3_table[64][2] = {
    { 0, 7 }, { 3, 4 }, { 7, 0 }, { 4, 3 }, { 6, 1 }, { 5, 2 }, { 1, 6 }, { 2, 5 }, { 1, 6 }, { 2, 5 }, { 6, 1 },
    { 5, 2 }, { 7, 0 }, { 4, 3 }, { 0, 7 }, { 3, 4 }, { 4, 3 }, { 7, 0 }, { 3, 4 }, { 0, 7 }, { 2, 5 }, { 1, 6 },
    { 5, 2 }, { 6, 1 }, { 5, 2 }, { 6, 1 }, { 2, 5 }, { 1, 6 }, { 3, 4 }, { 0, 7 }, { 4, 3 }, { 7, 0 }, { 7, 0 },
    { 4, 3 }, { 0, 7 }, { 3, 4 }, { 1, 6 }, { 2, 5 }, { 6, 1 }, { 5, 2 }, { 6, 1 }, { 5, 2 }, { 1, 6 }, { 2, 5 },
    { 0, 7 }, { 3, 4 }, { 7, 0 }, { 4, 3 }, { 3, 4 }, { 0, 7 }, { 4, 3 }, { 7, 0 }, { 5, 2 }, { 6, 1 }, { 2, 5 },
    { 1, 6 }, { 2, 5 }, { 1, 6 }, { 5, 2 }, { 6, 1 }, { 4, 3 }, { 7, 0 }, { 3, 4 }, { 0, 7 }
};

/** @brief used for 1/2 rate viterbi encoding */
STATIC const uint8_t lr_fhss_viterbi_1_2_table[16][2] = { { 0, 3 }, { 1, 2 }, { 2, 1 }, { 3, 0 }, { 2, 1 }, { 3, 0 },
                                                          { 0, 3 }, { 1, 2 }, { 3, 0 }, { 2, 1 }, { 1, 2 }, { 0, 3 },
                                                          { 1, 2 }, { 0, 3 }, { 3, 0 }, { 2, 1 } };

/** @brief used header interleaving */
STATIC const uint8_t lr_fhss_header_interleaver_minus_one[80] = {
    0,  18, 36, 54, 72, 4,  22, 40,  //
    58, 76, 8,  26, 44, 62, 12, 30,  //
    48, 66, 16, 34, 52, 70, 1,  19,  //
    37, 55, 73, 5,  23, 41, 59, 77,  //
    9,  27, 45, 63, 13, 31, 49, 67,  //
    17, 35, 53, 71, 2,  20, 38, 56,  //
    74, 6,  24, 42, 60, 78, 10, 28,  //
    46, 64, 14, 32, 50, 68, 3,  21,  //
    39, 57, 75, 7,  25, 43, 61, 79,  //
    11, 29, 47, 65, 15, 33, 51, 69   //
};

/** @brief lookup table for lr_fhss_header_crc8 */
const uint8_t lr_fhss_header_crc8_lut[256] = {
    0,   47,  94,  113, 188, 147, 226, 205, 87,  120, 9,   38,  235, 196, 181, 154,  //
    174, 129, 240, 223, 18,  61,  76,  99,  249, 214, 167, 136, 69,  106, 27,  52,   //
    115, 92,  45,  2,   207, 224, 145, 190, 36,  11,  122, 85,  152, 183, 198, 233,  //
    221, 242, 131, 172, 97,  78,  63,  16,  138, 165, 212, 251, 54,  25,  104, 71,   //
    230, 201, 184, 151, 90,  117, 4,   43,  177, 158, 239, 192, 13,  34,  83,  124,  //
    72,  103, 22,  57,  244, 219, 170, 133, 31,  48,  65,  110, 163, 140, 253, 210,  //
    149, 186, 203, 228, 41,  6,   119, 88,  194, 237, 156, 179, 126, 81,  32,  15,   //
    59,  20,  101, 74,  135, 168, 217, 246, 108, 67,  50,  29,  208, 255, 142, 161,  //
    227, 204, 189, 146, 95,  112, 1,   46,  180, 155, 234, 197, 8,   39,  86,  121,  //
    77,  98,  19,  60,  241, 222, 175, 128, 26,  53,  68,  107, 166, 137, 248, 215,  //
    144, 191, 206, 225, 44,  3,   114, 93,  199, 232, 153, 182, 123, 84,  37,  10,   //
    62,  17,  96,  79,  130, 173, 220, 243, 105, 70,  55,  24,  213, 250, 139, 164,  //
    5,   42,  91,  116, 185, 150, 231, 200, 82,  125, 12,  35,  238, 193, 176, 159,  //
    171, 132, 245, 218, 23,  56,  73,  102, 252, 211, 162, 141, 64,  111, 30,  49,   //
    118, 89,  40,  7,   202, 229, 148, 187, 33,  14,  127, 80,  157, 178, 195, 236,  //
    216, 247, 134, 169, 100, 75,  58,  21,  143, 160, 209, 254, 51,  28,  109, 66    //
};

/** @brief lookup table for lr_fhss_payload_crc16 */
const uint16_t lr_fhss_payload_crc16_lut[256] = {
    0,     30043, 60086, 40941, 41015, 54636, 19073, 16346, 13621, 16494, 57219, 43736, 38146, 57433, 32692, 2799,   //
    27242, 7985,  32988, 62855, 51805, 48902, 8427,  21936, 24415, 10756, 46569, 49330, 65384, 35379, 5598,  24709,  //
    54484, 41359, 15970, 19257, 29923, 440,   40533, 60174, 57825, 38074, 2903,  32268, 16854, 13453, 43872, 56891,  //
    48830, 52197, 21512, 8531,  7817,  27602, 62527, 33124, 35723, 65232, 24893, 5222,  11196, 24295, 49418, 46161,  //
    56563, 43432, 13893, 17182, 31940, 2463,  38514, 58153, 59846, 40093, 880,   30251, 18929, 15530, 41799, 54812,  //
    46745, 50114, 23599, 10612, 5806,  25589, 64536, 35139, 33708, 63223, 26906, 7233,  9115,  22208, 51501, 48246,  //
    2087,  32124, 58001, 38858, 43024, 56651, 17062, 14333, 15634, 18505, 55204, 41727, 40229, 59518, 30611, 712,    //
    25165, 5910,  35067, 64928, 49786, 46881, 10444, 23959, 22392, 8739,  48590, 51349, 63311, 33300, 7673,  26786,  //
    52413, 47590, 9739,  21328, 27786, 6609,  34364, 62311, 63880, 36051, 4926,  26213, 22975, 11492, 45833, 50770,  //
    42711, 54156, 19553, 14650, 1760,  29627, 60502, 39181, 37858, 59065, 31060, 3087,  13269, 18062, 55651, 44088,  //
    6249,  27954, 62175, 34692, 47198, 52485, 21224, 10163, 11612, 22535, 51178, 45745, 36203, 63536, 26589, 4742,   //
    29187, 1880,  39093, 60910, 53812, 42863, 14466, 19929, 18230, 12909, 44416, 55515, 59137, 37466, 3511,  30956,  //
    4174,  25877, 64248, 36771, 45177, 50466, 23247, 12180, 9595,  20512, 53197, 47766, 34124, 61463, 28666, 6817,   //
    31268, 3967,  37010, 58825, 55827, 44872, 12453, 17918, 20241, 14922, 42407, 53500, 61222, 39549, 1424,  28875,  //
    50330, 45505, 11820, 23415, 25773, 4598,  36379, 64320, 61871, 34036, 6937,  28226, 20888, 9411,  47918, 52853,  //
    44784, 56235, 17478, 12573, 3783,  31644, 58481, 37162, 39877, 61086, 29043, 1064,  15346, 20137, 53572, 42015
};

/**
 * @brief integral square root, rounded up
 *
 * @param  [in] x argument
 *
 * @returns Square root of argument, rounded up to next integer
 *
 * @remark This function is only appropriate to use for reasonably small arguments
 */
STATIC uint16_t sqrt_uint16( uint16_t x );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/**
 * @brief Compute 16-bit payload CRC
 *
 * @param  [in] data_in        Pointer to input buffer
 * @param  [in] data_in_bytecount Input buffer length, in bytes
 *
 * @returns 16-bit CRC
 */
STATIC uint16_t lr_fhss_payload_crc16( const uint8_t* data_in, uint16_t data_in_bytecount );

/**
 * @brief Compute 8-bit header CRC
 *
 * @param  [in] data_in        Pointer to input buffer
 * @param  [in] data_in_bytecount Input buffer length, in bytes
 *
 * @returns 8-bit CRC
 */
STATIC uint8_t lr_fhss_header_crc8( const uint8_t* data_in, uint16_t data_in_bytecount );

/**
 * @brief Whiten the payload
 *
 * @param  [in] data_in           Pointer to input buffer
 * @param  [in] data_in_bytecount Input buffer length, in bytes
 * @param [out] data_out          Pointer to output buffer, of same length as input buffer
 */
STATIC void lr_fhss_payload_whitening( const uint8_t* data_in, uint16_t data_in_bytecount, uint8_t* data_out );

/**
 * @brief Extract specific bit from array of bytes
 *
 * @param  [in] data_in    Array of bytes
 * @param  [in] bit_number Index of bit in array
 *
 * @returns Value of the bit
 */
STATIC uint8_t lr_fhss_extract_bit_in_byte_vector( const uint8_t* data_in, uint32_t bit_number );

/**
 * @brief Set specific bit in array of bytes
 *
 * @param  [in] data_in   Array of bytes
 * @param  [in] bit_number Index of bit in array
 * @param  [in] bit_value  Value to be set
 */
STATIC void lr_fhss_set_bit_in_byte_vector( uint8_t* vector, uint32_t bit_number, uint8_t bit_value );

/**
 * @brief Compute 1/2 rate Viterbi encoding
 *
 * @param [in,out] encod_state      Pointer to encoded state
 * @param     [in] data_in          Pointer to input buffer
 * @param     [in] data_in_bitcount Length of input buffer, in bits
 * @param     [in] data_out         Pointer to output buffer
 *
 * @returns Length of output buffer, in bits
 */
STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_2_base( uint8_t* encod_state, const uint8_t* data_in,
                                                             uint16_t data_in_bitcount, uint8_t* data_out );

/**
 * @brief Compute 1/3 rate Viterbi encoding
 *
 * @param [in,out] encod_state      Pointer to encoded state
 * @param     [in] data_in          Pointer to input buffer
 * @param     [in] data_in_bitcount Length of input buffer, in bits
 * @param    [out] data_out         Pointer to output buffer
 *
 * @returns Length of output buffer, in bits
 */
STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_3_base( uint8_t* encod_state, const uint8_t* data_in,
                                                             uint16_t data_in_bitcount, uint8_t* data_out );

/**
 * @brief Convolute using lr_fhss_convolution_encode_viterbi_1_2_base with optional tail-biting
 *
 * @param  [in] data_in          Pointer to input buffer
 * @param  [in] data_in_bitcount Length of input buffer, in bits
 * @param  [in] tail_biting      Set to true to activate tail-biting
 * @param [out] data_out         Pointer to output buffer
 *
 * @remark If tail-biting is activated, this function calls lr_fhss_convolution_encode_viterbi_1_2_base twice
 *
 * @returns Length of output buffer, in bits
 */
STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_2( const uint8_t* data_in, uint16_t data_in_bitcount,
                                                        bool tail_biting, uint8_t* data_out );

/**
 * @brief Convolute using lr_fhss_convolution_encode_viterbi_1_3_base
 *
 * @param  [in] data_in          Pointer to input buffer
 * @param  [in] data_in_bitcount Length of input buffer, in bits
 * @param [out] data_out         Pointer to output buffer
 *
 * @returns Length of output buffer, in bits
 */
STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_3( const uint8_t* data_in, uint16_t data_in_bitcount,
                                                        uint8_t* data_out );

/**
 * @brief Computes payload interleaving
 *
 * @param  [in] data_in          Pointer to input buffer
 * @param  [in] data_in_bitcount Length of input buffer, in bits
 * @param [out] data_out         Pointer to output buffer
 * @param  [in] output_offset    Output offset indicating where data must be placed, in bits, relative to data_out bit 0
 *
 * @returns Length of output buffer, in bits
 */
STATIC uint16_t lr_fhss_payload_interleaving( const uint8_t* data_in, uint16_t data_in_bitcount, uint8_t* data_out,
                                              uint32_t output_offset );

/**
 * @brief Create the raw LR-FHSS header
 *
 * @param  [in] params          Parameter structure
 * @param  [in] hop_sequence_id The hop sequence ID that will be used to obtain hop-related data
 * @param  [in] payload_length  Length of application payload, in bytes
 * @param [out] data_out        Pointer to output buffer
 */
STATIC void lr_fhss_raw_header( const lr_fhss_v1_params_t* params, uint16_t hop_sequence_id, uint16_t payload_length,
                                uint8_t* data_out );

/**
 * @brief Store sync word index inside provided header
 *
 * @param  [in] sync_word_index The sync word index to store
 * @param [out] data_out        Pointer to output buffer
 */
STATIC void lr_fhss_store_header_sync_word_index( uint8_t sync_word_index, uint8_t* data_out );

/**
 * @brief Get the bit count and block count for a LR-FHSS frame
 *
 * @param  [in] params         Parameter structure
 * @param  [in] payload_length Length of physical payload, in bytes
 * @param [out] nb_hops_out    Number of LR-FHSS hops
 *
 * @returns Length of physical payload, in bits
 */
STATIC uint16_t lr_fhss_get_bit_and_hop_count( const lr_fhss_v1_params_t* params, uint16_t payload_length,
                                               uint8_t* nb_hops_out );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

unsigned int lr_fhss_get_hop_sequence_count( const lr_fhss_v1_params_t* params )
{
    if( ( params->grid == LR_FHSS_V1_GRID_25391_HZ ) ||
        ( ( params->grid == LR_FHSS_V1_GRID_3906_HZ ) && ( params->bw < LR_FHSS_V1_BW_335938_HZ ) ) )
    {
        return 384;
    }
    return 512;
}

void lr_fhss_process_parameters( const lr_fhss_v1_params_t* params, uint16_t payload_length, lr_fhss_digest_t* digest )
{
    digest->nb_bits = lr_fhss_get_bit_and_hop_count( params, payload_length, &digest->nb_hops );

    digest->nb_bytes = ( digest->nb_bits + 8 - 1 ) / 8;
    if( params->enable_hopping == false )
    {
        digest->nb_hops = 1;
    }
}

lr_fhss_status_t lr_fhss_get_hop_params( const lr_fhss_v1_params_t* params, lr_fhss_hop_params_t* hop_params,
                                         uint16_t* initial_state, uint16_t hop_sequence_id )
{
    uint32_t channel_count = lr_fhss_channel_count[params->bw];

    if( params->grid == LR_FHSS_V1_GRID_3906_HZ )
    {
        hop_params->n_grid = channel_count / 8;
    }
    else
    {
        hop_params->n_grid = channel_count / 52;
    }

    switch( hop_params->n_grid )
    {
    case 10:
    case 22:
    case 28:
    case 30:
    case 35:
    case 47:
    {
        *initial_state          = 6;
        hop_params->polynomial  = lr_fhss_lfsr_poly1[hop_sequence_id >> 6];
        hop_params->xoring_seed = hop_sequence_id & 0x3F;
        if( hop_sequence_id >= 384 )
        {
            return LR_FHSS_STATUS_ERROR;
        }
        break;
    }
    case 60:
    case 62:
    {
        *initial_state          = 56;
        hop_params->polynomial  = lr_fhss_lfsr_poly1[hop_sequence_id >> 6];
        hop_params->xoring_seed = hop_sequence_id & 0x3F;
        if( hop_sequence_id >= 384 )
        {
            return LR_FHSS_STATUS_ERROR;
        }
        break;
    }
    case 86:
    case 99:
    {
        *initial_state          = 6;
        hop_params->polynomial  = lr_fhss_lfsr_poly2[hop_sequence_id >> 7];
        hop_params->xoring_seed = hop_sequence_id & 0x7F;
        break;
    }
    case 185:
    case 198:
    {
        *initial_state          = 6;
        hop_params->polynomial  = lr_fhss_lfsr_poly3[hop_sequence_id >> 8];
        hop_params->xoring_seed = hop_sequence_id & 0xFF;
        break;
    }
    case 390:
    case 403:
    {
        *initial_state          = 6;
        hop_params->polynomial  = 264;
        hop_params->xoring_seed = hop_sequence_id;
        break;
    }
    default:
        return LR_FHSS_STATUS_ERROR;
    }

    hop_params->hop_sequence_id = hop_sequence_id;

    return LR_FHSS_STATUS_OK;
}

uint16_t lr_fhss_get_next_state( uint16_t* lfsr_state, const lr_fhss_hop_params_t* hop_params )
{
    uint16_t hop;

    do
    {
        uint16_t lsb = *lfsr_state & 1;
        *lfsr_state >>= 1;
        if( lsb )
        {
            *lfsr_state ^= hop_params->polynomial;
        }
        hop = hop_params->xoring_seed;
        if( hop != *lfsr_state )
        {
            hop ^= *lfsr_state;
        }
    } while( hop > hop_params->n_grid );

    return hop - 1;
}

int16_t lr_fhss_get_next_freq_in_grid( uint16_t* lfsr_state, const lr_fhss_hop_params_t* hop_params,
                                       const lr_fhss_v1_params_t* params )
{
    uint16_t n_i;

    if( params->enable_hopping )
    {
        n_i = lr_fhss_get_next_state( lfsr_state, hop_params );
    }
    else
    {
        n_i = hop_params->hop_sequence_id % hop_params->n_grid;
    }

    if( n_i < ( hop_params->n_grid >> 1 ) )
    {
        return n_i;
    }
    else
    {
        return n_i - hop_params->n_grid;
    }
}

/**************************** Build LR-FHSS Frame ***********************************************************
 *    Core of the LR-FHSS frame generator                                                                   *
 *                                                                                                           *
 * In |---------|  |-----|  |-----------------|  |-------|  |------------|  |----------------------|  Out    *
 **---|Whitening|--|CRC16|--|Outer Code + CRC8|--|Viterbi|--|Interleaving|--|Sync+header+crc Header|------   *
 *    |---------|  |-----|  |-----------------|--|-------|  |------------|  |----------------------|         *
 *                                                                                                           *
 ************************************************************************************************************/
uint16_t lr_fhss_build_frame( const lr_fhss_v1_params_t* params, uint16_t hop_sequence_id, const uint8_t* data_in,
                              uint16_t data_in_bytecount, uint8_t* data_out )
{
    uint8_t data_out_tmp[LR_FHSS_MAX_TMP_BUF_BYTES] = { 0 };

    lr_fhss_payload_whitening( data_in, data_in_bytecount, data_out );
    uint16_t payload_crc = lr_fhss_payload_crc16( data_out, data_in_bytecount );

    data_out[data_in_bytecount]     = ( payload_crc >> 8 ) & 0xFF;
    data_out[data_in_bytecount + 1] = payload_crc & 0xFF;
    data_out[data_in_bytecount + 2] = 0;

    // the 1/3 encoded bytes can go up to LR_FHSS_MAX_TMP_BUF_BYTES temporarly, before puncturing it
    uint16_t nb_bits =
        lr_fhss_convolution_encode_viterbi_1_3( data_out, 8 * ( data_in_bytecount + 2 ) + 6, data_out_tmp );

    // Avoid putting random stack data into payload
    memset( data_out, 0, LR_FHSS_MAX_PHY_PAYLOAD_BYTES );

    if( params->cr != LR_FHSS_V1_CR_1_3 )
    {
        // this assumes first matrix values are always the same, which is the case
        uint32_t matrix_index = 0;
        uint8_t  matrix[15]   = { 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0 };
        uint8_t  matrix_len   = 0;
        switch( params->cr )
        {
        case LR_FHSS_V1_CR_5_6:
            matrix_len = 15;
            break;
        case LR_FHSS_V1_CR_2_3:
            matrix_len = 6;
            break;
        case LR_FHSS_V1_CR_1_2:
            matrix_len = 3;
            break;
        default:
            // LR_FHSS_V1_CR_1_3 is excluded from this code block
            break;
        }

        uint32_t j = 0;
        for( uint32_t i = 0; i < nb_bits; i++ )
        {
            if( matrix[matrix_index] )
            {
                lr_fhss_set_bit_in_byte_vector( data_out, j++, lr_fhss_extract_bit_in_byte_vector( data_out_tmp, i ) );
            }
            if( ++matrix_index == matrix_len )
            {
                matrix_index = 0;
            }
        }
        nb_bits = j;

        memcpy( data_out_tmp, data_out, ( nb_bits + 7 ) / 8 );
    }

    // Interleave directly to data_out
    nb_bits =
        lr_fhss_payload_interleaving( data_out_tmp, nb_bits, data_out, LR_FHSS_HEADER_BITS * params->header_count );

    // Build the header
    uint8_t raw_header[LR_FHSS_HALF_HDR_BYTES];
    lr_fhss_raw_header( params, hop_sequence_id, data_in_bytecount, raw_header );

    uint16_t header_offset = 0;
    for( uint32_t i = 0; i < params->header_count; i++ )
    {
        // Insert appropriate index into header
        lr_fhss_store_header_sync_word_index( params->header_count - i - 1, raw_header );
        raw_header[4] = lr_fhss_header_crc8( raw_header, 4 );

        // Convolutional encode
        uint8_t coded_header[LR_FHSS_HDR_BYTES] = { 0 };
        lr_fhss_convolution_encode_viterbi_1_2( raw_header, LR_FHSS_HALF_HDR_BITS, 1, coded_header );

        // Header guard bits
        lr_fhss_set_bit_in_byte_vector( data_out, header_offset + 0, 0 );
        lr_fhss_set_bit_in_byte_vector( data_out, header_offset + 1, 0 );

        // Interleave the header directly to the physical payload buffer
        for( uint32_t j = 0; j < LR_FHSS_HALF_HDR_BITS; j++ )
        {
            lr_fhss_set_bit_in_byte_vector(
                data_out, header_offset + 2 + j,
                lr_fhss_extract_bit_in_byte_vector( coded_header, lr_fhss_header_interleaver_minus_one[j] ) );
        }
        for( uint32_t j = 0; j < LR_FHSS_HALF_HDR_BITS; j++ )
        {
            lr_fhss_set_bit_in_byte_vector(
                data_out, header_offset + 2 + LR_FHSS_HALF_HDR_BITS + LR_FHSS_SYNC_WORD_BITS + j,
                lr_fhss_extract_bit_in_byte_vector( coded_header,
                                                    lr_fhss_header_interleaver_minus_one[LR_FHSS_HALF_HDR_BITS + j] ) );
        }

        // Copy the sync word to the physical payload buffer
        for( uint32_t j = 0; j < LR_FHSS_SYNC_WORD_BITS; j++ )
        {
            lr_fhss_set_bit_in_byte_vector( data_out, header_offset + 2 + LR_FHSS_HALF_HDR_BITS + j,
                                            lr_fhss_extract_bit_in_byte_vector( params->sync_word, j ) );
        }

        header_offset += LR_FHSS_HEADER_BITS;
    }

    return ( header_offset + nb_bits + 7 ) / 8;
}

uint32_t lr_fhss_get_time_on_air_in_ms( const lr_fhss_v1_params_t* params, uint16_t payload_length )
{
    // Multiply by 1000 / 488.28125, or equivalently 256/125, rounding up
    return ( ( lr_fhss_get_time_on_air_numerator( params, payload_length ) << 8 ) + 124 ) / 125;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

STATIC uint16_t lr_fhss_payload_crc16( const uint8_t* data_in, uint16_t data_in_bytecount )
{
    uint16_t crc16 = 65535;
    uint8_t  pos   = 0;
    for( uint16_t k = 0; k < data_in_bytecount; k++ )
    {
        pos   = ( ( crc16 >> 8 ) ^ data_in[k] );
        crc16 = ( crc16 << 8 ) ^ lr_fhss_payload_crc16_lut[pos];
    }
    return crc16;
}

STATIC uint8_t lr_fhss_header_crc8( const uint8_t* data_in, uint16_t data_in_bytecount )
{
    uint8_t crc8 = 255;
    for( uint16_t k = 0; k < data_in_bytecount; k++ )
    {
        uint8_t pos = ( crc8 ^ data_in[k] );
        crc8        = lr_fhss_header_crc8_lut[pos];
    }

    return crc8;
}

STATIC void lr_fhss_payload_whitening( const uint8_t* data_in, uint16_t data_in_bytecount, uint8_t* data_out )
{
    uint8_t lfsr = 0xFF;

    for( uint16_t index = 0; index < data_in_bytecount; index++ )
    {
        uint8_t u       = data_in[index] ^ lfsr;
        data_out[index] = ( ( u & 0x0F ) << 4 ) | ( ( u & 0xF0 ) >> 4 );
        lfsr =
            ( uint8_t ) ( ( lfsr << 1 ) |
                          ( ( ( lfsr & 0x80 ) >> 7 ) ^
                            ( ( ( lfsr & 0x20 ) >> 5 ) ^ ( ( ( lfsr & 0x10 ) >> 4 ) ^ ( ( lfsr & 0x8 ) >> 3 ) ) ) ) );
    }
}

STATIC uint8_t lr_fhss_extract_bit_in_byte_vector( const uint8_t* data_in, uint32_t bit_number )
{
    uint32_t index   = bit_number >> 3;
    uint8_t  bit_pos = 7 - ( bit_number % 8 );

    if( data_in[index] & ( 1 << bit_pos ) )
    {
        return 1;
    }
    return 0;
}

STATIC void lr_fhss_set_bit_in_byte_vector( uint8_t* vector, uint32_t bit_number, uint8_t bit_value )
{
    uint32_t index   = bit_number >> 3;
    uint8_t  bit_pos = 7 - ( bit_number % 8 );

    vector[index] = ( vector[index] & ( 0xff - ( 1 << bit_pos ) ) ) | ( bit_value << bit_pos );
}

STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_2_base( uint8_t* encod_state, const uint8_t* data_in,
                                                             uint16_t data_in_bitcount, uint8_t* data_out )
{
    uint8_t  g1g0;
    uint8_t  cur_bit;
    uint16_t ind_bit;
    uint16_t data_out_bitcount = 0;
    uint16_t bin_out_16        = 0;

    for( ind_bit = 0; ind_bit < data_in_bitcount; ind_bit++ )
    {
        cur_bit      = lr_fhss_extract_bit_in_byte_vector( data_in, ind_bit );
        g1g0         = lr_fhss_viterbi_1_2_table[*encod_state][cur_bit];
        *encod_state = ( *encod_state * 2 + cur_bit ) % 16;
        bin_out_16 |= ( g1g0 << ( ( 7 - ( ind_bit % 8 ) ) << 1 ) );
        if( ind_bit % 8 == 7 )
        {
            *data_out++ = ( uint8_t ) ( bin_out_16 >> 8 );
            *data_out++ = ( uint8_t ) bin_out_16;
            bin_out_16  = 0;
        }
        data_out_bitcount += 2;
    }
    if( ind_bit % 8 )
    {
        *data_out++ = ( uint8_t ) ( bin_out_16 >> 8 );
        *data_out++ = ( uint8_t ) bin_out_16;
    }

    return data_out_bitcount;
}

STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_3_base( uint8_t* encod_state, const uint8_t* data_in,
                                                             uint16_t data_in_bitcount, uint8_t* data_out )
{
    uint8_t  g1g0;
    uint8_t  cur_bit;
    uint16_t ind_bit;
    uint16_t data_out_bitcount = 0;
    uint32_t bin_out_32        = 0;

    for( ind_bit = 0; ind_bit < data_in_bitcount; ind_bit++ )
    {
        cur_bit      = lr_fhss_extract_bit_in_byte_vector( data_in, ind_bit );
        g1g0         = lr_fhss_viterbi_1_3_table[*encod_state][cur_bit];
        *encod_state = ( *encod_state * 2 + cur_bit ) % 64;
        bin_out_32 |= ( g1g0 << ( ( 7 - ( ind_bit % 8 ) ) * 3 ) );
        if( ind_bit % 8 == 7 )
        {
            *data_out++ = ( uint8_t ) ( bin_out_32 >> 16 );
            *data_out++ = ( uint8_t ) ( bin_out_32 >> 8 );
            *data_out++ = ( uint8_t ) bin_out_32;
            bin_out_32  = 0;
        }
        data_out_bitcount += 3;
    }
    if( ind_bit % 8 )
    {
        *data_out++ = ( uint8_t ) ( bin_out_32 >> 16 );
        *data_out++ = ( uint8_t ) ( bin_out_32 >> 8 );
        *data_out++ = ( uint8_t ) bin_out_32;
    }

    return data_out_bitcount;
}

STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_2( const uint8_t* data_in, uint16_t data_in_bitcount,
                                                        bool tail_biting, uint8_t* data_out )
{
    uint8_t  encode_state = 0;
    uint16_t data_out_bitcount;

    data_out_bitcount =
        lr_fhss_convolution_encode_viterbi_1_2_base( &encode_state, data_in, data_in_bitcount, data_out );
    if( tail_biting )
    {
        data_out_bitcount =
            lr_fhss_convolution_encode_viterbi_1_2_base( &encode_state, data_in, data_in_bitcount, data_out );
    }
    return data_out_bitcount;
}

STATIC uint16_t lr_fhss_convolution_encode_viterbi_1_3( const uint8_t* data_in, uint16_t data_in_bitcount,
                                                        uint8_t* data_out )
{
    uint8_t encode_state = 0;
    return lr_fhss_convolution_encode_viterbi_1_3_base( &encode_state, data_in, data_in_bitcount, data_out );
}

STATIC uint16_t sqrt_uint16( uint16_t x )
{
    uint16_t y = 0;

    while( y * y < x )
    {
        y += 1;
    }

    return y;
}

STATIC uint16_t lr_fhss_payload_interleaving( const uint8_t* data_in, uint16_t data_in_bitcount, uint8_t* data_out,
                                              uint32_t output_offset )
{
    uint16_t       step   = sqrt_uint16( data_in_bitcount );
    const uint16_t step_v = step >> 1;
    step                  = step << 1;

    uint16_t pos           = 0;
    uint16_t st_idx        = 0;
    uint16_t st_idx_init   = 0;
    int16_t  bits_left     = data_in_bitcount;
    uint16_t out_row_index = output_offset;

    while( bits_left > 0 )
    {
        int16_t in_row_width = bits_left;
        if( in_row_width > LR_FHSS_FRAG_BITS )
        {
            in_row_width = LR_FHSS_FRAG_BITS;
        }

        lr_fhss_set_bit_in_byte_vector( data_out, 0 + out_row_index, 0 );  // guard bits
        lr_fhss_set_bit_in_byte_vector( data_out, 1 + out_row_index, 0 );  // guard bits
        for( int32_t j = 0; j < in_row_width; j++ )
        {
            lr_fhss_set_bit_in_byte_vector( data_out, j + 2 + out_row_index,
                                            lr_fhss_extract_bit_in_byte_vector( data_in, pos ) );  // guard bit

            pos += step;
            if( pos >= data_in_bitcount )
            {
                st_idx += step_v;
                if( st_idx >= step )
                {
                    st_idx_init++;
                    st_idx = st_idx_init;
                }
                pos = st_idx;
            }
        }

        bits_left -= LR_FHSS_FRAG_BITS;
        out_row_index += 2 + in_row_width;
    }

    return out_row_index - output_offset;
}

STATIC void lr_fhss_raw_header( const lr_fhss_v1_params_t* params, uint16_t hop_sequence_id, uint16_t payload_length,
                                uint8_t* data_out )
{
    data_out[0] = payload_length;
    data_out[1] = ( params->modulation_type << 5 ) + ( params->cr << 3 ) + ( params->grid << 2 ) +
                  ( params->enable_hopping ? 2 : 0 ) + ( params->bw >> 3 );
    data_out[2] = ( ( params->bw & 0x07 ) << 5 ) + ( hop_sequence_id >> 4 );
    data_out[3] = ( ( hop_sequence_id & 0x000F ) << 4 );
}

STATIC void lr_fhss_store_header_sync_word_index( uint8_t sync_word_index, uint8_t* data_out )
{
    data_out[3] = ( data_out[3] & ~0x0C ) | ( sync_word_index << 2 );
}

STATIC uint16_t lr_fhss_get_bit_and_hop_count( const lr_fhss_v1_params_t* params, uint16_t payload_length,
                                               uint8_t* nb_hops_out )
{
    // check length : payload + 16bit crc, encoded, padded to 48bits, adding 2 guard bit / 48bits
    uint16_t length_bits = ( payload_length + 2 ) * 8 + 6;
    switch( params->cr )
    {
    case LR_FHSS_V1_CR_5_6:
        length_bits = ( ( length_bits * 6 ) + 4 ) / 5;
        break;

    case LR_FHSS_V1_CR_2_3:
        length_bits = length_bits * 3 / 2;
        break;

    case LR_FHSS_V1_CR_1_2:
        length_bits = length_bits * 2;
        break;

    case LR_FHSS_V1_CR_1_3:
        length_bits = length_bits * 3;
        break;
    }

    *nb_hops_out = ( length_bits + 47 ) / 48 + params->header_count;

    // calculate total number of payload bits, after breaking into blocks
    uint16_t payload_bits    = length_bits / LR_FHSS_FRAG_BITS * LR_FHSS_BLOCK_BITS;
    uint16_t last_block_bits = length_bits % LR_FHSS_FRAG_BITS;
    if( last_block_bits > 0 )
    {
        // add the 2 guard bits for the last block + the actual remaining payload bits
        payload_bits += last_block_bits + 2;
    }

    return ( LR_FHSS_HEADER_BITS * params->header_count ) + payload_bits;
}

/* --- EOF ------------------------------------------------------------------ */
