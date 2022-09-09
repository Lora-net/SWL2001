/*!
 * \file      fragmented_data_block.c
 *
 * \brief     LoRaWAN Fragmented Data Block Transport protocol
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

#include "gpio.h"
#include "radio_ctrl.h"
#include "frag_decoder.h"
#include "smtc_modem_hal.h"
#include "aes.h"
#include "cmac.h"
#include "nvmcu_hal.h"
#include "modem_utilities.h"  // for crc fw
#include "patch_upd.h"
#include "pool_mem.h"
#include "smtc_modem_hal_dbg_trace.h"
#include "fragmented_data_block.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

/*
 * Allow access to internal functions in unit tests
 */
#if defined( TEST )
#define STATIC
#else
#define STATIC static
#endif

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Get a particular bit value from a byte
 *
 * \param [IN] b Any byte from which we want a bit value
 * \param [IN] p Position of the bit in the byte [0..7]
 * \param [IN] n Number of bits we want to get
 * \retval The value corresponding the requested bits
 */
#define TAKE_N_BITS_FROM( b, p, n ) ( ( ( b ) >> ( p ) ) & ( ( 1 << ( n ) ) - 1 ) )

// Return byte b from value v
#define BYTE( v, b ) ( ( ( v ) >> ( 8 * ( b ) ) ) & 0xFF )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct s_frag_session_status_req
{
    uint8_t frag_index;
    uint8_t participants;
} s_frag_session_status_req_t;

struct s_frag_session
{
    uint8_t frag_index;
    uint8_t mc_group_bit_mask;
};

struct s_control
{
    uint8_t ack_reception;
    uint8_t frag_algo;
    uint8_t block_ack_delay;
};

typedef enum target_type
{
    TARGET_MODEM = 0,
    TARGET_HOST  = 0x01000000
} e_target_type_t;

typedef enum descriptor_error
{
    DESCRIPTOR_ERROR,
    DESCRIPTOR_OK
} e_descriptor_error_t;

typedef struct s_frag_session_setup_req
{
    struct s_frag_session frag_session;
    uint16_t              nb_frag;
    uint8_t               frag_size;
    struct s_control      control;
    uint8_t               padding;
    uint32_t              descriptor;
    uint16_t              session_cnt;
    uint32_t              mic;
} s_frag_session_setup_req_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
static const char* frag_cmd_str[FRAG_CMD_MAX] = {
    [FRAG_CMD_PACKAGE_VERSION]          = "PACKAGE_VERSION",
    [FRAG_CMD_FRAG_SESSION_STATUS]      = "FRAG_SESSION_STATUS",
    [FRAG_CMD_FRAG_SESSION_SETUP]       = "FRAG_SESSION_SETUP",
    [FRAG_CMD_FRAG_SESSION_DELETE]      = "FRAG_SESSION_DELETE",
    [FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED] = "FRAG_DATA_BLOCK_RECEIVED",
    [FRAG_CMD_FRAG_DATA_FRAGMENT]       = "FRAG_DATA_FRAGMENT",
};
#endif

// TODO Change this to put it in flash
// Memory to hold the full reconstructed data block
// static uint8_t frag_data_buffer[FRAG_DATA_BLOCK_SIZE_MAX];
#define FRAG_MIC_BLOCK_SIZE 16
#define FRAG_MIC_BUFFER_SIZE 128
#define FRAG_MIC_B0_HEADER 0x49

static uint8_t        frag_tx_payload[FRAG_UPLINK_LENGTH_MAX];
static e_file_error_t check_received_patch( void );
struct
{
    // Uplink buffer
    uint8_t frag_tx_payload_index;
    uint8_t frag_max_length_up_payload;

    // Pending requests
    uint8_t                     frag_req_status[FRAG_UPLINK_REQ_MAX];
    uint8_t                     frag_req_status_num;
    s_frag_session_status_req_t frag_session_status_req;
    s_frag_session_setup_req_t  frag_session_setup_req;
    uint8_t                     frag_session_delete_req;

    // Current fragmentation status
    bool     is_frag_session_exist;
    bool     is_data_block_reconstructed;
    bool     is_data_block_mic_success;
    bool     is_data_block_sign_success;
    bool     is_data_block_crc_fw_success;
    bool     is_defrag_memory_exceeded;
    bool     is_session_aborted;     // lost too many fragments for decoder to recover
    bool     is_ack_reception_done;  // The server has confirmed the ack_reception
    uint16_t nb_frag_uncoded_received;
    uint16_t nb_frag_coded_received;
    uint16_t nb_frag_ignored;
    int32_t  session_cnt_prev;  // TODO: to be stored in flash ??
} frag_context;

#define frag_tx_payload_index frag_context.frag_tx_payload_index
#define frag_max_length_up_payload frag_context.frag_max_length_up_payload

#define frag_req_status frag_context.frag_req_status
#define frag_req_status_num frag_context.frag_req_status_num
#define frag_session_status_req frag_context.frag_session_status_req
#define frag_session_setup_req frag_context.frag_session_setup_req
#define frag_session_delete_req frag_context.frag_session_delete_req

#define is_frag_session_exist frag_context.is_frag_session_exist
#define is_data_block_reconstructed frag_context.is_data_block_reconstructed
#define is_data_block_mic_success frag_context.is_data_block_mic_success
#define is_data_block_sign_success frag_context.is_data_block_sign_success
#define is_data_block_crc_fw_success frag_context.is_data_block_crc_fw_success
#define is_defrag_memory_exceeded frag_context.is_defrag_memory_exceeded
#define is_session_aborted frag_context.is_session_aborted
#define is_ack_reception_done frag_context.is_ack_reception_done
#define nb_frag_uncoded_received frag_context.nb_frag_uncoded_received
#define nb_frag_coded_received frag_context.nb_frag_coded_received
#define nb_frag_ignored frag_context.nb_frag_ignored
#define session_cnt_prev frag_context.session_cnt_prev

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * \brief initialise the session variables
 *
 *
 * \param [IN] void
 */
void frag_context_reset( void )
{
    is_data_block_reconstructed  = false;
    is_data_block_mic_success    = false;
    is_data_block_sign_success   = false;
    is_data_block_crc_fw_success = false;
    is_defrag_memory_exceeded    = false;
    is_ack_reception_done        = false;
    nb_frag_uncoded_received     = 0;
    nb_frag_coded_received       = 0;
}

/*!
 * \brief Write fragment to memory, callback for frag_decoder
 *
 * Writes `data` buffer of `size` starting at address `addr`
 *
 * \param [IN] addr Address start index to write to.
 * \param [IN] data Data buffer to be written.
 * \param [IN] size Size of data buffer to be written.
 *
 * \retval status Write operation status [0: Success, -1 Fail]
 */
int8_t frag_decoder_write_fl( uint32_t addr, uint8_t* data, uint32_t size )
{
    uint32_t target_address = FLASH_DELTA_UPDATE + addr;

    if( data == NULL )
    {
        return -1;
    }
    if( target_address < FLASH_DELTA_UPDATE )
    {
        DEBUG_PRINT( DBG_FATAL, "Flash write error address out of limit:%x\n", target_address );
        return -1;
    }

    // Check that it fits
    if( target_address + size >= FLASH_DELTA_UPDATE + FRAG_DATA_BLOCK_SIZE_MAX )
    {
        DEBUG_PRINT( DBG_FATAL, "Flash write error address out of limit:%x\n", target_address + size );
        return -1;
    }

    // write
    DEBUG_PRINT( DBG_NOTE, "Flash write addr:%x,data:%x,size:%d\n", target_address, *data, size );

    target_address       = ( target_address - FLASH_BASE );
    uint8_t   page_start = target_address >> 11;
    uint32_t* copy_page  = POOL_MEM.StartOfPoolMem;
    DEBUG_PRINT( DBG_NOTE, "page is %d\n", page_start );
    memcpy( ( uint8_t* ) copy_page, ( uint8_t* ) ( ( page_start << 11 ) + FLASH_BASE ), 4096 );
    DEBUG_PRINT( DBG_NOTE, "page is %d\n", page_start );
    memcpy( ( uint8_t* ) copy_page + ( target_address - ( page_start << 11 ) ), data, size );
    FlashErasePage( page_start, MainFlash );
    FlashErasePage( page_start + 1, MainFlash );
    if( FlashWrite( ( page_start << 11 ) >> 2, copy_page, 1024, 0, MainFlash ) != 1 )
    {
        DEBUG_PRINT( DBG_FATAL, "Flash write error:%x\n", target_address );
        return -1;
    }
    return 0;
}

/*!
 * \brief Read fragment from memory, callback for frag_decoder
 *
 * Reads `data` buffer of `size` starting at address `addr`
 *
 * \param [IN] addr Address start index to read from.
 * \param [IN] data Data buffer to be read.
 * \param [IN] size Size of data buffer to be read.
 *
 * \retval status Read operation status [0: Success, -1 Fail]
 */
int8_t frag_decoder_read_fl( uint32_t addr, uint8_t* data, uint32_t size )
{
    void* rc;
    // uint32_t  flashStartAddr;

    uint32_t target_address = FLASH_DELTA_UPDATE + addr;
    // flashStartAddr = target_address;

    if( data == NULL )
    {
        return -1;
    }
    if( target_address < FLASH_DELTA_UPDATE )
    {
        DEBUG_PRINT( DBG_FATAL, "Flash read error address out of limit:%x\n", target_address );
        return -1;
    }

    // Check that it fits
    if( target_address + size >= FLASH_DELTA_UPDATE + FRAG_DATA_BLOCK_SIZE_MAX )
    {
        DEBUG_PRINT( DBG_FATAL, "Flash read error address out of limit:%x\n", target_address + size );
        return -1;
    }
    // DEBUG_PRINT(DBG_FATAL,"Flash read addr:%x,size:%d\n",target_address,size);
    rc = memcpy( data, ( uint8_t* ) target_address, size );
    if( rc != NULL )
    {
        DEBUG_PRINT( DBG_FATAL, "Flash read [0]:%x,[1]:%x,[2]:%x,[3]:%x\n", data[0], data[1], data[2], data[3] );
        return 0;  // Success
    }

    // data = (uint8_t*)target_address;
    DEBUG_PRINT( DBG_FATAL, "Flash read [0]:%x,[1]:%x,[2]:%x,[3]:%x\n", data[0], data[1], data[2], data[3] );
    return -1;
}

// Callback structure passed to frag_decoder
static FragDecoderCallbacks_t frag_decoder_callbacks = {
    .FragDecoderWrite = frag_decoder_write_fl,
    .FragDecoderRead  = frag_decoder_read_fl,
};

void frag_session_print( void )
{
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_INFO( "------ Current Frag Session context -------\n" );
    SMTC_MODEM_HAL_TRACE_INFO( "  session exist: %d\n", is_frag_session_exist );
    SMTC_MODEM_HAL_TRACE_INFO( "  reconstructed: %d\n", is_data_block_reconstructed );
    SMTC_MODEM_HAL_TRACE_INFO( "  MIC success: %d\n", is_data_block_mic_success );
    SMTC_MODEM_HAL_TRACE_INFO( "  SIGN success: %d\n", is_data_block_sign_success );
    SMTC_MODEM_HAL_TRACE_INFO( "  CRC_FW success: %d\n", is_data_block_crc_fw_success );
    SMTC_MODEM_HAL_TRACE_INFO( "  Memory Exceeded: %d\n", is_defrag_memory_exceeded );
    SMTC_MODEM_HAL_TRACE_INFO( "  nb frag received: %u (uncoded:%u, coded:%u, ignored:%u)\n",
                               nb_frag_uncoded_received + nb_frag_coded_received, nb_frag_uncoded_received,
                               nb_frag_coded_received, nb_frag_ignored );
    SMTC_MODEM_HAL_TRACE_INFO( "  previous session cnt: %d\n", session_cnt_prev );
    if( is_frag_session_exist == true )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "------ Current Frag Session Setup request ------\n" );
        SMTC_MODEM_HAL_TRACE_INFO( "  frag_session.frag_index: %u\n", frag_session_setup_req.frag_session.frag_index );
        SMTC_MODEM_HAL_TRACE_INFO( "  frag_session.mc_group_bit_mask: %u\n",
                                   frag_session_setup_req.frag_session.mc_group_bit_mask );
        SMTC_MODEM_HAL_TRACE_INFO( "  nb_frag: %u\n", frag_session_setup_req.nb_frag );
        SMTC_MODEM_HAL_TRACE_INFO( "  frag_size: %u\n", frag_session_setup_req.frag_size );
        SMTC_MODEM_HAL_TRACE_INFO( "  control.ack_reception: %u\n", frag_session_setup_req.control.ack_reception );
        SMTC_MODEM_HAL_TRACE_INFO( "  control.frag_algo: %u\n", frag_session_setup_req.control.frag_algo );
        SMTC_MODEM_HAL_TRACE_INFO( "  control.block_ack_delay: %u\n", frag_session_setup_req.control.block_ack_delay );
        SMTC_MODEM_HAL_TRACE_INFO( "  padding: %u\n", frag_session_setup_req.padding );
        SMTC_MODEM_HAL_TRACE_INFO( "  descriptor: 0x%x\n", frag_session_setup_req.descriptor );
        SMTC_MODEM_HAL_TRACE_INFO( "  session_cnt: %u\n", frag_session_setup_req.session_cnt );
        SMTC_MODEM_HAL_TRACE_INFO( "  mic: 0x%x\n", frag_session_setup_req.mic );
        SMTC_MODEM_HAL_TRACE_INFO( "-----------------------------------\n" );
    }
#endif
}

static inline bool is_frag_tx_buffer_not_full( uint8_t cmd_size )
{
    return ( ( frag_tx_payload_index + cmd_size ) <= MIN( FRAG_UPLINK_LENGTH_MAX, frag_max_length_up_payload )
                 ? true
                 : false );
}

static void frag_decode_session_status_req( uint8_t* buffer )
{
    frag_session_status_req.frag_index   = TAKE_N_BITS_FROM( buffer[0], 1, 2 );
    frag_session_status_req.participants = TAKE_N_BITS_FROM( buffer[0], 0, 1 );
}

static void frag_decode_session_setup_req( uint8_t* buffer )
{
    // byte 0
    frag_session_setup_req.frag_session.frag_index        = TAKE_N_BITS_FROM( buffer[0], 4, 2 );
    frag_session_setup_req.frag_session.mc_group_bit_mask = TAKE_N_BITS_FROM( buffer[0], 0, 4 );

    // bytes 1, 2
    frag_session_setup_req.nb_frag = ( buffer[2] << 8 ) | buffer[1];

    // byte 3
    frag_session_setup_req.frag_size = buffer[3];

    // byte 4
    frag_session_setup_req.control.ack_reception   = TAKE_N_BITS_FROM( buffer[4], 6, 1 );
    frag_session_setup_req.control.frag_algo       = TAKE_N_BITS_FROM( buffer[4], 3, 3 );
    frag_session_setup_req.control.block_ack_delay = TAKE_N_BITS_FROM( buffer[4], 0, 3 );

    // byte 5
    frag_session_setup_req.padding = buffer[5];

    // bytes 6, 7, 8, 9
    frag_session_setup_req.descriptor =
        ( buffer[9] << 24 ) | ( buffer[8] << 16 ) | ( buffer[7] << 8 ) | ( buffer[6] << 0 );

    // bytes 10, 11
    frag_session_setup_req.session_cnt = ( buffer[11] << 8 ) | buffer[10];

    // bytes 12, 13, 14, 15
    frag_session_setup_req.mic =
        ( buffer[15] << 24 ) | ( buffer[14] << 16 ) | ( buffer[13] << 8 ) | ( buffer[12] << 0 );
}

static void frag_decode_session_delete_req( uint8_t* buffer )
{
    // byte 0
    frag_session_delete_req = buffer[0] & 0x03;
}

static void frag_decode_data_block_received_ans( uint8_t* buffer )
{
    if( TAKE_N_BITS_FROM( buffer[0], 0, 2 ) != 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragReceivedDataBlockAns: invalid FragIndex\n" );
        return;
    }
    else
    {
        SMTC_MODEM_HAL_TRACE_INFO( "FragReceivedDataBlockAns: AckReception has been received by the server\n" );
        is_ack_reception_done = true;
    }
}

#define FRAG_MIC_BLOCK_SIZE 16
#define FRAG_MIC_BUFFER_SIZE 128
#define FRAG_MIC_B0_HEADER 0x49

STATIC e_frag_error_t frag_compute_datablock_key( uint8_t* key )
{
    memset( key, 0, 16 );
    return FRAG_OK;
}

/*
 * Compute the MIC for the FragmentedDataBlock session.
 * We suppose that:
 *  - all fields are little-endian
 *  - devaddr is 0x00000000
 *  - key is {0}[16]
 */
STATIC e_frag_error_t frag_compute_mic( s_frag_session_setup_req_t session, uint32_t* mic )
{
    AES_CMAC_CTX aes_cmac_ctx;
    uint8_t      buffer[FRAG_MIC_BUFFER_SIZE];
    uint8_t      micB0[FRAG_MIC_BLOCK_SIZE] = { 0 };
    uint8_t      cmac[FRAG_MIC_BLOCK_SIZE]  = { 0 };
    uint8_t      data_block_int_key[16];

    /*
     * Use a static NULL address because the DAS does not know the unicast device address.
     * We should use lr1mac_core_devaddr_get() to comply with the LoRa Alliance paper.
     */
    uint32_t devaddr = 0;

    uint32_t file_size = ( session.nb_frag * session.frag_size ) - session.padding;
    uint8_t  padding   = file_size % FRAG_MIC_BLOCK_SIZE;

    /* Get the Data Block Integrity Key */
    if( frag_compute_datablock_key( data_block_int_key ) != FRAG_OK )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "frag_compute_mic: failed to get Data Block Integrity key\n" );
        return FRAG_ERROR;
    }

    SMTC_MODEM_HAL_TRACE_INFO( "MIC cnt %x index %x desc %x devaddr %x size %x\n", session.session_cnt,
                               session.frag_session.frag_index, session.descriptor, devaddr, file_size );

    micB0[0] = FRAG_MIC_B0_HEADER;

    /* LoRaWAN uses a little endian representation */
    micB0[1] = BYTE( session.session_cnt, 0 );
    micB0[2] = BYTE( session.session_cnt, 1 );

    micB0[3] = session.frag_session.frag_index;

    micB0[4] = BYTE( session.descriptor, 0 );
    micB0[5] = BYTE( session.descriptor, 1 );
    micB0[6] = BYTE( session.descriptor, 2 );
    micB0[7] = BYTE( session.descriptor, 3 );

    micB0[8]  = BYTE( devaddr, 0 );
    micB0[9]  = BYTE( devaddr, 1 );
    micB0[10] = BYTE( devaddr, 2 );
    micB0[11] = BYTE( devaddr, 3 );

    micB0[12] = BYTE( file_size, 0 );
    micB0[13] = BYTE( file_size, 1 );
    micB0[14] = BYTE( file_size, 2 );
    micB0[15] = BYTE( file_size, 3 );

    AES_CMAC_Init( &aes_cmac_ctx );
    AES_CMAC_SetKey( &aes_cmac_ctx, data_block_int_key );
    AES_CMAC_Update( &aes_cmac_ctx, micB0, FRAG_MIC_BLOCK_SIZE );

    // The file might be in flash, so we have to copy it to a temporary buffer
    // to do the CMAC computation, which requires the buffer to be in RAM

    for( uint32_t addr = 0; addr < file_size; addr += FRAG_MIC_BUFFER_SIZE )
    {
        uint32_t len = MIN( FRAG_MIC_BUFFER_SIZE, file_size - addr );
        frag_decoder_read_fl( addr, buffer, len );
        for( int i = 0; i < len; i++ )
            DEBUG_PRINT( DBG_FATAL, "%x,", buffer[i] );
        DEBUG_PRINT( DBG_FATAL, " \n" );
        AES_CMAC_Update( &aes_cmac_ctx, buffer, len );
    }
    memset( buffer, 0, padding );

    AES_CMAC_Update( &aes_cmac_ctx, buffer, padding );
    AES_CMAC_Final( cmac, &aes_cmac_ctx );

    *mic = cmac[3] << 24 | cmac[2] << 16 | cmac[1] << 8 | cmac[0];
    return FRAG_OK;
}

STATIC int8_t frag_process_data_fragment( uint8_t* buffer, uint8_t buffer_len )
{
    uint8_t                    frag_index;  // session
    uint16_t                   frag_n;      // index of the coded fragment
    FragDecoderSessionStatus_t rc;          // FragDecoder return code

    if( is_frag_session_exist == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DataFragment: no frag session on-going\n" );
        return FRAG_INVALID;
    }

    if( ( buffer_len - 2 ) != frag_session_setup_req.frag_size )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DataFragment: wrong fragment size (exp:%u, got:%u)\n",
                                    frag_session_setup_req.frag_size, buffer_len - 2 );
        return FRAG_BADSIZE;
    }

    // Parse Index&N
    frag_index = TAKE_N_BITS_FROM( buffer[1], 6, 2 );
    frag_n     = ( TAKE_N_BITS_FROM( buffer[1], 0, 6 ) << 8 ) | buffer[0];

    if( frag_index != 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DataFragment: FragIndex unsupported\n" );
        return FRAG_INVALID;
    }

    if( frag_n < 1 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "DataFragment: N must be >= 1 \n" );
        return FRAG_INVALID;
    }

    if( frag_n <= frag_session_setup_req.nb_frag )
    {
        nb_frag_uncoded_received += 1;
    }
    if( frag_n > frag_session_setup_req.nb_frag )
    {
        nb_frag_coded_received += 1;
    }
    if( is_data_block_reconstructed )
    {
        // We don't need to process more fragments
        nb_frag_ignored += 1;

        frag_session_print( );
        return FRAG_OK;
    }
    if( is_session_aborted )
    {
        // We don't need to process more fragments
        nb_frag_ignored += 1;
        frag_session_print( );
        return FRAG_OK;
    }

    // The decoder rejects 'old' fragments, if their frag_n precede the latest received.
    rc = FragDecoderProcess( frag_n, &buffer[2] );
    SMTC_MODEM_HAL_TRACE_PRINTF( "Fragment %d FragDecoderProcess rc %d\n", frag_n, rc );
    if( rc == FRAG_SESSION_OK )
    {
        SMTC_MODEM_HAL_TRACE_INFO( "SUCCESS: FragDecoder reconstructed the full data, no need for more fragments\n" );
        // This means the decoder received enough fragments and reconstructed the data.
        // rc contains the number of missing frames
        is_data_block_reconstructed = true;
        // BlockReceived message should be sent automatically, if needed, now that reconstructed is True

        uint32_t mic;
        frag_compute_mic( frag_session_setup_req, &mic );
        is_data_block_mic_success = ( frag_session_setup_req.mic == mic );
        SMTC_MODEM_HAL_TRACE_INFO( "MIC setup %x | computed %x [%s]\n", frag_session_setup_req.mic, mic,
                                   is_data_block_mic_success ? " OK " : "FAIL" );
    }
    else
    {
        // This means that the decoder generated an error, or is ongoing wihout
        // any means to reconstruct the original data yet
        switch( rc )
        {
        case FRAG_SESSION_ONGOING:
            // All is well, decoder is waiting for additional fragments
            SMTC_MODEM_HAL_TRACE_INFO( "FRAG: Waiting for more fragments\n" );
            break;
        case FRAG_SESSION_ABORT:
            // Lost too many fragments
            SMTC_MODEM_HAL_TRACE_ERROR( "FRAG: Lost too many fragments to be able to reconstruct\n" );
            is_session_aborted = true;
            break;
        case FRAG_SESSION_ERROR:
            // Invalid frag number
            SMTC_MODEM_HAL_TRACE_WARNING( "FRAG: Invalid packet number\n" );
            break;
        default:
            break;
        }
    }

    frag_session_print( );

    return FRAG_OK;
}

e_descriptor_error_t frag_session_parse_descriptor( uint32_t descriptor )
{
    // this function check if the target is modem and check the validity of the current fw
    // return 1 if check is valid
    if( ( descriptor & 0x01000000 ) == TARGET_HOST )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( " ERROR parsing descriptor : Bad target \n" );
        return DESCRIPTOR_ERROR;
    }
    uint32_t crctmp         = compute_crc_fw( ) & 0x00FFFFFF;
    uint32_t crc_descriptor = descriptor & 0x00FFFFFF;

    if( crctmp != crc_descriptor )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( " ERROR parsing descriptor : Bad crc fw  crc fw = %x crc descriptor = %x \n",
                                    crctmp, crc_descriptor );
        return DESCRIPTOR_ERROR;
    }
    return DESCRIPTOR_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void frag_init( void )
{
    // Initialize requests holding variables
    frag_req_status_num = 0;
    memset( frag_req_status, 0, sizeof frag_req_status );
    memset( &frag_session_status_req, 0, sizeof( struct s_frag_session_status_req ) );
    memset( &frag_session_setup_req, 0, sizeof( struct s_frag_session_setup_req ) );
    frag_session_delete_req = 0;

    // Initialize uplink buffer context
    frag_tx_payload_index      = 0;
    frag_max_length_up_payload = FRAG_UPLINK_LENGTH_MAX;

    // Initialize fragmentation session context variables
    is_frag_session_exist        = false;
    is_data_block_reconstructed  = false;
    is_data_block_mic_success    = false;
    is_data_block_sign_success   = false;
    is_data_block_crc_fw_success = false;
    is_defrag_memory_exceeded    = false;
    is_ack_reception_done        = false;
    is_session_aborted           = false;
    nb_frag_uncoded_received     = 0;
    nb_frag_coded_received       = 0;
    nb_frag_ignored              = 0;
    session_cnt_prev             = -1;
}

// Returns the number of commands handled, or FRAG_CMD_ERROR
int8_t frag_parser( uint8_t* frag_buffer, uint8_t frag_buffer_len )
{
    uint8_t frag_rx_buffer_index = 0;
    // bao add
    uint8_t nb_cmd                          = 0;
    uint8_t nb_cmd_package_version          = 0;  // FRAG_CMD_PACKAGE_VERSION = 0;
    uint8_t nb_cmd_frag_session_status      = 0;  // FRAG_CMD_FRAG_SESSION_STATUS = 0;
    uint8_t nb_cmd_frag_session_setup       = 0;  // FRAG_CMD_FRAG_SESSION_SETUP = 0;
    uint8_t nb_cmd_frag_session_delete      = 0;  // FRAG_CMD_FRAG_SESSION_DELETE = 0;
    uint8_t nb_cmd_frag_data_block_received = 0;  // FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED = 0;
    // -----------
    // Reset answer buffer index
    frag_tx_payload_index = 0;
    frag_req_status_num   = 0;

    if( frag_buffer == NULL )
    {
        return ( int8_t ) FRAG_CMD_ERROR;
    }

    if( frag_buffer_len < FRAG_DOWNLINK_HEADER_LENGTH )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "Fragmentation Downlink must contain at least 1 byte\n" );
        return ( int8_t ) FRAG_CMD_ERROR;
    }

    while( ( frag_buffer_len > frag_rx_buffer_index ) && ( nb_cmd < 2 ) )
    {
#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
        if( frag_buffer[frag_rx_buffer_index] < FRAG_CMD_MAX )
            SMTC_MODEM_HAL_TRACE_INFO( "FRAG_CMD_%s (0x%x)\n", frag_cmd_str[frag_buffer[frag_rx_buffer_index]],
                                       frag_buffer[frag_rx_buffer_index] );
#endif

        switch( frag_buffer[frag_rx_buffer_index] )
        {
        case FRAG_CMD_PACKAGE_VERSION:  // REQ
            if( nb_cmd_package_version == 0 )
            {
                if( ( frag_rx_buffer_index + FRAG_PACKAGE_VERSION_REQ_SIZE ) <= frag_buffer_len )
                {
                    frag_req_status[frag_req_status_num++] = FRAG_CMD_PACKAGE_VERSION;
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_ERROR( "Fragmented Data Block payload bad size (FRAG_CMD_PACKAGE_VERSION)\n" );
                }
                frag_rx_buffer_index += FRAG_PACKAGE_VERSION_REQ_SIZE;
                nb_cmd++;
                nb_cmd_package_version++;
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: double CMD PACKET VERSION, aborting\n" );
                return ( int8_t ) FRAG_CMD_ERROR;
            }
            // if( ( frag_rx_buffer_index + FRAG_PACKAGE_VERSION_REQ_SIZE ) <= frag_buffer_len )
            // {
            //     frag_req_status[frag_req_status_num++] = FRAG_CMD_PACKAGE_VERSION;
            // }
            // else
            // {
            //     SMTC_MODEM_HAL_TRACE_ERROR( "Fragmented Data Block payload bad size (FRAG_CMD_PACKAGE_VERSION)\n" );
            // }
            // frag_rx_buffer_index += FRAG_PACKAGE_VERSION_REQ_SIZE;
            break;

        case FRAG_CMD_FRAG_SESSION_STATUS:  // REQ
            if( nb_cmd_frag_session_status == 0 )
            {
                if( ( frag_rx_buffer_index + FRAG_SESSION_STATUS_REQ_SIZE ) <= frag_buffer_len )
                {
                    frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_SESSION_STATUS;
                    frag_decode_session_status_req( &frag_buffer[frag_rx_buffer_index + 1] );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_ERROR(
                        "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_SESSION_STATUS)\n" );
                }
                frag_rx_buffer_index += FRAG_SESSION_STATUS_REQ_SIZE;
                nb_cmd++;
                nb_cmd_frag_session_status++;
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: double CMD FRAG SESSION STATUS, aborting\n" );
                return ( int8_t ) FRAG_CMD_ERROR;
            }
            // if( ( frag_rx_buffer_index + FRAG_SESSION_STATUS_REQ_SIZE ) <= frag_buffer_len )
            // {
            //     frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_SESSION_STATUS;
            //     frag_decode_session_status_req( &frag_buffer[frag_rx_buffer_index + 1] );
            // }
            // else
            // {
            //     SMTC_MODEM_HAL_TRACE_ERROR( "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_SESSION_STATUS)\n"
            //     );
            // }
            // frag_rx_buffer_index += FRAG_SESSION_STATUS_REQ_SIZE;
            break;

        case FRAG_CMD_FRAG_SESSION_SETUP:  // REQ

            if( nb_cmd_frag_session_setup == 0 )
            {
                if( ( frag_rx_buffer_index + FRAG_SESSION_SETUP_REQ_SIZE ) <= frag_buffer_len )
                {
                    frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_SESSION_SETUP;
                    frag_decode_session_setup_req( &frag_buffer[frag_rx_buffer_index + 1] );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_ERROR(
                        "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_SESSION_SETUP)\n" );
                }
                frag_rx_buffer_index += FRAG_SESSION_SETUP_REQ_SIZE;
                nb_cmd++;
                nb_cmd_frag_session_setup++;
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: double CMD FRAG SESSION SETUP, aborting\n" );
                return ( int8_t ) FRAG_CMD_ERROR;
            }

            // if( ( frag_rx_buffer_index + FRAG_SESSION_SETUP_REQ_SIZE ) <= frag_buffer_len )
            // {
            //     frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_SESSION_SETUP;
            //     frag_decode_session_setup_req( &frag_buffer[frag_rx_buffer_index + 1] );
            // }
            // else
            // {
            //     SMTC_MODEM_HAL_TRACE_ERROR( "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_SESSION_SETUP)\n"
            //     );
            // }
            // frag_rx_buffer_index += FRAG_SESSION_SETUP_REQ_SIZE;
            break;

        case FRAG_CMD_FRAG_SESSION_DELETE:  // REQ

            if( nb_cmd_frag_session_delete == 0 )
            {
                if( ( frag_rx_buffer_index + FRAG_SESSION_DELETE_REQ_SIZE ) <= frag_buffer_len )
                {
                    frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_SESSION_DELETE;
                    frag_decode_session_delete_req( &frag_buffer[frag_rx_buffer_index + 1] );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_ERROR(
                        "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_SESSION_DELETE)\n" );
                }
                frag_rx_buffer_index += FRAG_SESSION_DELETE_REQ_SIZE;
                nb_cmd++;
                nb_cmd_frag_session_delete++;
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: double CMD FRAG SESSION DELETE, aborting\n" );
                return ( int8_t ) FRAG_CMD_ERROR;
            }

            // if( ( frag_rx_buffer_index + FRAG_SESSION_DELETE_REQ_SIZE ) <= frag_buffer_len )
            // {
            //     frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_SESSION_DELETE;
            //     frag_decode_session_delete_req( &frag_buffer[frag_rx_buffer_index + 1] );
            // }
            // else
            // {
            //     SMTC_MODEM_HAL_TRACE_ERROR( "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_SESSION_DELETE)\n"
            //     );
            // }
            // frag_rx_buffer_index += FRAG_SESSION_DELETE_REQ_SIZE;
            break;

        case FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED:  // ANS

            if( nb_cmd_frag_data_block_received == 0 )
            {
                if( ( frag_rx_buffer_index + FRAG_DATA_BLOCK_RECEIVED_ANS_SIZE ) <= frag_buffer_len )
                {
                    frag_decode_data_block_received_ans( &frag_buffer[frag_rx_buffer_index + 1] );
                }
                else
                {
                    SMTC_MODEM_HAL_TRACE_ERROR(
                        "Fragmented Data Block payload bad size (FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED)\n" );
                }
                frag_rx_buffer_index += FRAG_DATA_BLOCK_RECEIVED_ANS_SIZE;
                nb_cmd++;
                nb_cmd_frag_data_block_received++;
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: double CMD FRAG DATA BLOCK RECEIVED, aborting\n" );
                return ( int8_t ) FRAG_CMD_ERROR;
            }

            // if( ( frag_rx_buffer_index + FRAG_DATA_BLOCK_RECEIVED_ANS_SIZE ) <= frag_buffer_len )
            // {
            //     frag_decode_data_block_received_ans( &frag_buffer[frag_rx_buffer_index + 1] );
            // }
            // else
            // {
            //     SMTC_MODEM_HAL_TRACE_ERROR( "Fragmented Data Block payload bad size
            //     (FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED)\n"
            //     );
            // }
            // frag_rx_buffer_index += FRAG_DATA_BLOCK_RECEIVED_ANS_SIZE;
            break;

        case FRAG_CMD_FRAG_DATA_FRAGMENT:  // CANNOT BE APPENDED WITH OTHER REQUEST MESSAGES

            if( ( frag_rx_buffer_index == 0 ) && ( frag_buffer_len == frag_session_setup_req.frag_size + 3 ) )
            {
                frag_process_data_fragment( &frag_buffer[frag_rx_buffer_index + 1], frag_buffer_len - 1 );
                if( ( is_data_block_reconstructed == true ) &&
                    ( frag_session_setup_req.control.ack_reception == 0x01 ) && ( is_ack_reception_done == false ) )
                {
                    SMTC_MODEM_HAL_TRACE_WARNING( "Preparing BLOCK_RECEIVED\n" );
                    // at this point we will verify the received file , ancm
                    if( check_received_patch( ) > 0 )
                    {
                        SMTC_MODEM_HAL_TRACE_WARNING( "file is valid!!\n" );
                        frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED;
                    }
                    else
                        SMTC_MODEM_HAL_TRACE_WARNING( "file is not valid!!\n" );
                    // create the uplink with data block received for das aknownledge
                    frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED;
                }
                else
                {
                    frag_req_status_num = 0;
                }
                frag_rx_buffer_index += frag_buffer_len;
                nb_cmd++;
                break;
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_ERROR(
                    "ERROR: wrong data frag command size or mix with other cmd, size = ((0x%x)), aborting\n",
                    frag_buffer_len );
                return ( int8_t ) FRAG_CMD_ERROR;
            }

            // frag_process_data_fragment( &frag_buffer[frag_rx_buffer_index + 1], frag_buffer_len - 1 );
            // if( ( is_data_block_reconstructed == true ) && ( frag_session_setup_req.control.ack_reception == 0x01 )
            // &&
            //     ( is_ack_reception_done == false ) )
            // {
            //     SMTC_MODEM_HAL_TRACE_WARNING( "Preparing BLOCK_RECEIVED\n" );
            //     if( is_data_block_mic_success == true )
            //     {
            //         // at this point we will verify the received file , ancm
            //         if( check_received_patch( ) > 0 )
            //         {
            //             // code never reach because jump to bootloader 2
            //             SMTC_MODEM_HAL_TRACE_WARNING( "file is valid!!\n" );
            //         }
            //         else
            //         {
            //             SMTC_MODEM_HAL_TRACE_WARNING( "file is not valid!!\n" );
            //         }
            //     }
            //     // create the uplink with data block received for das aknownledge
            //     frag_req_status[frag_req_status_num++] = FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED;
            // }
            // else
            // {
            //     frag_req_status_num = 0;
            // }
            // frag_rx_buffer_index += frag_buffer_len;
            // break;

        default:
            SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: wrong frag command (0x%x), aborting\n",
                                        frag_buffer[frag_rx_buffer_index] );
            return ( int8_t ) FRAG_CMD_ERROR;
        }
    }

    // if the parsed index != equal to the frag buffer, the packet length is wrong
    if( frag_rx_buffer_index != frag_buffer_len )
    {
        frag_req_status_num = 0;
        SMTC_MODEM_HAL_TRACE_ERROR(
            "ERROR: the downlink length is not correct, parse fails, frag_rx_buffer_index= ((0x%x)), frag_buffer_len=  "
            "((0x%x)), nb_cmd= ((0x%x)), aborting\n",
            frag_rx_buffer_index, frag_buffer_len, nb_cmd );
        return ( int8_t ) FRAG_CMD_ERROR;
    }

#if MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON
    SMTC_MODEM_HAL_TRACE_ARRAY( "FRAG_REQ_STATUS", frag_req_status, frag_req_status_num );
#endif

    return ( int8_t ) frag_req_status_num;
}

void frag_set_max_length_up_payload( uint8_t max_payload )
{
    frag_max_length_up_payload = max_payload;
}

void frag_construct_uplink_payload( void )
{
    int i;

    for( i = 0; i < frag_req_status_num; i++ )
    {
        switch( frag_req_status[i] )
        {
        case FRAG_CMD_PACKAGE_VERSION:
            frag_construct_package_version_answer( );
            break;
        case FRAG_CMD_FRAG_SESSION_STATUS:
            frag_construct_frag_session_status_answer( );
            break;
        case FRAG_CMD_FRAG_SESSION_SETUP:
            frag_construct_frag_session_setup_answer( );
            break;
        case FRAG_CMD_FRAG_SESSION_DELETE:
            frag_construct_frag_session_delete_answer( );
            break;
        case FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED:
            frag_construct_data_block_received_request( );
            break;
        default:
            SMTC_MODEM_HAL_TRACE_ERROR( "ERROR: wrong command in freq_req_status[], SHOULD NOT HAPPEN !!\n" );
            break;
        }
    }

    // All commands should be answered now. If some could not due to max payload length exceeded
    // it is truncated, and the server will need to reiterate the request which has not been answered.
    frag_req_status_num = 0;
}

void frag_construct_package_version_answer( void )
{
    SMTC_MODEM_HAL_TRACE_INFO( "=> running %s\n", __FUNCTION__ );

    if( is_frag_tx_buffer_not_full( FRAG_PACKAGE_VERSION_ANS_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "frag_tx_payload buffer is full, skip this request\n" );
        return;
    }

    frag_tx_payload[frag_tx_payload_index++] = FRAG_CMD_PACKAGE_VERSION;
    frag_tx_payload[frag_tx_payload_index++] = FRAG_PACKAGE_IDENTIFIER;
    frag_tx_payload[frag_tx_payload_index++] = FRAG_PACKAGE_VERSION;
}

void frag_construct_frag_session_status_answer( void )
{
    uint8_t  status         = 0x00;
    uint16_t received_index = 0x00;
    uint8_t  missing_frag   = 0x00;

    SMTC_MODEM_HAL_TRACE_INFO( "=> running %s\n", __FUNCTION__ );

    if( is_frag_tx_buffer_not_full( FRAG_SESSION_STATUS_ANS_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionStatus: frag_tx_payload buffer is full, skip this request\n" );
        return;
    }

    if( ( frag_session_status_req.frag_index > 0 ) || ( is_frag_session_exist == false ) )
    {
        // Session does not exist
        status = ( 1 << FRAG_SESSION_STATUS_NO_SESSION );
    }
    else  // Session exists
    {
        // Do we have to answer this request ?
        if( ( frag_session_status_req.participants == 0 ) &&
            ( nb_frag_uncoded_received == frag_session_setup_req.nb_frag ) )
        {
            // The request targets only the receivers still missing fragments
            SMTC_MODEM_HAL_TRACE_ERROR( "Ignoring FragSessionStatusReq, all uncoded fragments have been received\n" );
            return;
        }

        // Set status field
        if( ( is_data_block_reconstructed == true ) && ( is_data_block_mic_success == false ) )
        {
            status |= ( 1 << FRAG_SESSION_STATUS_MIC_ERROR );
        }
        if( ( is_data_block_reconstructed == true ) && ( is_data_block_sign_success == false ) )
        {
            status |= ( 1 << FRAG_SESSION_STATUS_SIGN_ERROR );
        }
        if( ( is_data_block_reconstructed == true ) && ( is_data_block_crc_fw_success == false ) )
        {
            status |= ( 1 << FRAG_SESSION_STATUS_CRC_FW_ERROR );
        }
        if( is_defrag_memory_exceeded == true )
        {
            status |= ( 1 << FRAG_SESSION_STATUS_MEMORY_ERROR );
        }

        // Set Received&index field
        received_index =
            ( ( nb_frag_uncoded_received + nb_frag_coded_received ) & 0x3FFF );  // Only FragIndex 0 is supported

        // Set MissingFrag field
        if( is_data_block_reconstructed == false )
        {
            missing_frag = ( ( frag_session_setup_req.nb_frag - nb_frag_uncoded_received ) > 255 )
                               ? 255
                               : ( uint8_t )( frag_session_setup_req.nb_frag - nb_frag_uncoded_received );
        }
        else
        {
            missing_frag = 0;
        }
    }

    frag_tx_payload[frag_tx_payload_index++] = FRAG_CMD_FRAG_SESSION_STATUS;
    frag_tx_payload[frag_tx_payload_index++] = status;

    if( ( status & ( 1 << FRAG_SESSION_STATUS_NO_SESSION ) ) == 0 )
    {
        // A session exists, adding Received&Index and MissingFrag status information
        frag_tx_payload[frag_tx_payload_index++] = ( uint8_t )( ( received_index >> 0 ) & 0xFF );
        frag_tx_payload[frag_tx_payload_index++] = ( uint8_t )( ( received_index >> 8 ) & 0xFF );
        frag_tx_payload[frag_tx_payload_index++] = missing_frag;
    }
}

void frag_construct_frag_session_setup_answer( void )
{
    int32_t rc;  // FragDecoder return code
    uint8_t frag_session_setup_ans = 0x0;

    uint32_t tmp;

    SMTC_MODEM_HAL_TRACE_INFO( "=> running %s\n", __FUNCTION__ );

    if( is_frag_tx_buffer_not_full( FRAG_SESSION_SETUP_ANS_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: frag_tx_payload buffer is full, skip this request\n" );
        return;
    }

    // Set the FragIndex bits for the answer
    frag_session_setup_ans = ( frag_session_setup_req.frag_session.frag_index << 6 ) & 0xC0;

    //
    // Section 1: check if the session setup request is valid (do not update context in this section)
    //

    // Check if FragIndex for next session is valid (MUST be 0)
    if( frag_session_setup_req.frag_session.frag_index != 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: FragIndex unsupported\n" );
        frag_session_setup_ans |= ( 1 << FRAG_SESSION_SETUP_INDEX_UNSUPPORTED );
    }
    // frag_session_setup_req.frag_session.mc_group_bit_mask is ignored, only unicast supported

    // Check NbFrag range (2^14 - 1)
    if( frag_session_setup_req.nb_frag > 16383 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: NbFrag invalid\n" );
        frag_session_setup_ans |= ( 1 << FRAG_SESSION_SETUP_NO_MEMORY );
    }

    // Check if there is enough memory to store all the fragments (data_block_size = NbFrag * FragSize - Padding)
    tmp = frag_session_setup_req.nb_frag * frag_session_setup_req.frag_size;
    if( tmp - frag_session_setup_req.padding > FRAG_DATA_BLOCK_SIZE_MAX )
    // if( frag_session_setup_req.nb_frag * frag_session_setup_req.frag_size - frag_session_setup_req.padding >
    //     FRAG_DATA_BLOCK_SIZE_MAX )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: Not enough memory\n" );
        frag_session_setup_ans |= ( 1 << FRAG_SESSION_SETUP_NO_MEMORY );
    }

    if( frag_session_setup_req.control.frag_algo != 0 )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: FragAlgo unsupported\n" );
        frag_session_setup_ans |= ( 1 << FRAG_SESSION_SETUP_ALGO_UNSUPPORTED );
    }
    // Parse the descriptor
    e_descriptor_error_t descriptor_status = frag_session_parse_descriptor( frag_session_setup_req.descriptor );
    if( descriptor_status == DESCRIPTOR_ERROR )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: descriptor unsupported\n" );
        frag_session_setup_ans |= ( 1 << FRAG_SESSION_SETUP_WRONG_DESCRIPTOR );
    }

    // Check frag session counter
    if( session_cnt_prev >= ( int32_t )( frag_session_setup_req.session_cnt ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionSetup: SessionCnt replay\n" );
        frag_session_setup_ans |= ( 1 << FRAG_SESSION_SETUP_SESSIONCNT_REPLAY );
    }

    // MIC: nothing to do with it for now

    //
    // Section 2: If the session is valid, set the frag context
    //

    // If session setup request is valid
    if( TAKE_N_BITS_FROM( frag_session_setup_ans, 0, 5 ) == 0 )  // no error bit set
    {
        if( is_frag_session_exist == true )
        {
            // If there is already an existing session, abort the current one, reset context
            SMTC_MODEM_HAL_TRACE_ERROR( "TODO: Current fragmentation session SHALL BE reset\n" );
            frag_context_reset( );
            // ... ?
        }
        else
        {
            is_frag_session_exist = true;
        }

        // Set the frag session context variables
        // BlockAckDelay is ignored, not used for Unicast
        // Store the new Session Cnt
        session_cnt_prev = frag_session_setup_req.session_cnt;

        // Initialize underlying frag_decoder

        rc = FragDecoderInit( frag_session_setup_req.nb_frag, frag_session_setup_req.frag_size,
                              &frag_decoder_callbacks );
        switch( rc )
        {
        case FRAG_SESSION_ERROR:
            frag_session_setup_ans |= ( 1 << 1 );  // Not enough memory (bit 1)
            break;
        case FRAG_SESSION_BADSIZE:
            frag_session_setup_ans |= ( 1 << 1 );  // Not enough memory (bit 1)
            break;
        }
    }

    frag_session_print( );

    frag_tx_payload[frag_tx_payload_index++] = FRAG_CMD_FRAG_SESSION_SETUP;
    frag_tx_payload[frag_tx_payload_index++] = frag_session_setup_ans;
}

void frag_construct_frag_session_delete_answer( void )
{
    uint8_t frag_session_delete_ans = 0x0;

    SMTC_MODEM_HAL_TRACE_INFO( "=> running %s\n", __FUNCTION__ );

    if( is_frag_tx_buffer_not_full( FRAG_SESSION_DELETE_ANS_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionDelete: frag_tx_payload buffer is full, skip this request\n" );
        return;
    }

    // Set FragIndex
    frag_session_delete_ans |= frag_session_delete_req;

    // Set error if any
    if( ( frag_session_delete_req != 0 ) || ( is_frag_session_exist == false ) )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "FragSessionDelete: Session does not exist\n" );
        frag_session_delete_ans |= ( 1 << FRAG_SESSION_DELETE_NO_SESSION );
    }
    else
    {
        // Actually delete the session
        is_frag_session_exist = false;

        // Reset session context
        frag_context_reset( );
    }

    frag_session_print( );

    frag_tx_payload[frag_tx_payload_index++] = FRAG_CMD_FRAG_SESSION_DELETE;
    frag_tx_payload[frag_tx_payload_index++] = frag_session_delete_ans;
}

void frag_construct_data_block_received_request( void )
{
    uint8_t status = 0x00;  // Only FragIndex == 0 supported

    SMTC_MODEM_HAL_TRACE_INFO( "=> running %s\n", __FUNCTION__ );

    if( is_frag_tx_buffer_not_full( FRAG_DATA_BLOCK_RECEIVED_REQ_SIZE ) == false )
    {
        SMTC_MODEM_HAL_TRACE_ERROR( "ReceivedDataBlock: frag_tx_payload buffer is full, skip this request\n" );
        return;
    }

    if( is_data_block_mic_success == false )
    {
        status |= ( 1 << FRAG_RECEIVED_DATA_BLOCK_MIC_ERROR );
    }
    if( is_data_block_sign_success == false )
    {
        status |= ( 1 << FRAG_RECEIVED_DATA_BLOC_SIGN_ERROR );
    }
    if( is_data_block_crc_fw_success == false )
    {
        status |= ( 1 << FRAG_RECEIVED_DATA_BLOC_CRC_FW_ERROR );
    }

    frag_tx_payload[frag_tx_payload_index++] = FRAG_CMD_FRAG_DATA_BLOCK_RECEIVED;
    frag_tx_payload[frag_tx_payload_index++] = status;
}

e_frag_error_t frag_get_tx_buffer( uint8_t* tx_buffer_out, uint8_t* tx_buffer_length_out )
{
    if( tx_buffer_length_out == NULL )
    {
        return FRAG_INVALID;
    }
    if( tx_buffer_out == NULL )
    {
        *tx_buffer_length_out = 0;
        return FRAG_INVALID;
    }
    if( *tx_buffer_length_out < frag_tx_payload_index )
    {
        *tx_buffer_length_out = 0;
        return FRAG_BADSIZE;
    }

    memcpy( tx_buffer_out, frag_tx_payload, frag_tx_payload_index );
    *tx_buffer_length_out      = frag_tx_payload_index;
    frag_tx_payload_index      = 0;                       // Reset tx index after data reached
    frag_max_length_up_payload = FRAG_UPLINK_LENGTH_MAX;  // Reinit the max length authorized
    return FRAG_OK;
}

bool frag_uplink_pending( void )
{
    return frag_tx_payload_index > 0;
}

static e_file_error_t check_received_patch( void )
{
    uint8_t                 sign_ok      = 0;
    DELTA_PARTITION_HEADER* delta_header = ( DELTA_PARTITION_HEADER* ) UPDT_FIRM_HEADER;
    uint32_t                magicword    = delta_header->upd_fw_crc;
    // uint32_t delta_size = delta_header->upd_size;
    if( magicword == 0x35CA139A )
    {
        sign_ok = VerifySignature( );
        DEBUG_PRINT( DBG_INFO, "VerifySignature %d\n", sign_ok );
    }
    else
    {
        sign_ok = CheckAesHash( );
    }
    is_data_block_sign_success = ( sign_ok == 1 ) ? true : false;
    if( compute_crc_fw( ) == delta_header->ref_crc_descriptor )
    {
        is_data_block_crc_fw_success = true;
        DEBUG_PRINT( DBG_INFO, "verified crc_fw ok\n" );
    }
    else
    {
        is_data_block_crc_fw_success = false;
        DEBUG_PRINT( DBG_INFO, "verified crc_fw fail\n" );
    }
    DEBUG_PRINT( DBG_INFO, "verified = %d\n", sign_ok );

    if( ( sign_ok == 1 ) && ( is_data_block_crc_fw_success == true ) )
    {
        DEBUG_PRINT( DBG_INFO, "Format EE\n" );
        xEEformat( );

        DEBUG_PRINT( DBG_INFO, "Reset to BL2\n" );
        // perform a reboot to bootloader2
        // Before reboot, need to go to standbyRC mode
        SetStandbyRc( );

        P_PMU->scratch1.reg32 &= ~( ( uint32_t )( STAY_IN_BOOT_MSK | STAY_IN_BOOT2_MSK ) );
        P_PMU->scratch1.reg32 |= STAY_IN_BOOT2_MSK;
        // To be sure everything is correctly rebooted, use system reset for bootloader reset
        // But first reset GPIO for correct busy and GPIO restart.
        InitGPIO( );

        // CER-561 avoid eol poping in bootloader, where pmu irq handler is not implemented
        // smtc_modem_hal_reset_mcu( );
        memset( ( uint8_t* ) 0x00800000, 0, 16384 );  // Clear the Retention RAM

        P_PMU->interruptMask.reg32   = 0;
        P_CCU->rstCtrl.fields.sysRst = 1;
        return FILE_OK;
    }
    else
    {
        DEBUG_PRINT( DBG_INFO, "No valid Update found!!\n" );
        smtc_modem_hal_mcu_panic( );
        return FILE_INVALID;
    }
}

uint16_t frag_get_nb_frag_received( void )
{
    return nb_frag_uncoded_received + nb_frag_coded_received;
}
uint16_t frag_get_nb_frag( void )
{
    return frag_session_setup_req.nb_frag;
}
uint16_t frag_get_session_counter( void )
{
    return ( ( session_cnt_prev == -1 ) ? 0xFFFF : ( uint16_t )( session_cnt_prev & 0xFFFF ) );
}

// uint32_t frag_get_data_buffer( uint8_t** data )
// {
//     if( data == NULL )
//     {
//         return 0;
//     }
//     *data = frag_data_buffer;
//     return frag_session_setup_req.frag_size * frag_session_setup_req.nb_frag;
// }

/* --- EOF ------------------------------------------------------------------ */
