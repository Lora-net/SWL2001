/**
 * @file      patch_upd.h
 *
 * @brief     Patch update api
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

#ifndef PATCH_UPD
#define PATCH_UPD

#include <stdint.h>
#include "..\cervin_trx_fw\src\applications\bootloader\src\bootloader.h"
#include "..\cervin_trx_fw\src\applications\bootloader\src\ecc.h"

#define ROM_FLASH_ECC_SECP160R1_IDX 0
#define ROM_ECC_VERIFY_IDX 1
// 200f
// #define ROM_0F_10_FLASH_WRITE_ADDR         0x00000930
// #define ROM_0F_10_FLASH_PAGE_ERASE_ADDR    0x000007d4
#define ROM_0F_FLASH_ECC_SECP160R1_ADDR 0x00003b4c
#define ROM_0F_ECC_VERIFY_ADDR 0x00003b54

// 2010
#define ROM_10_FLASH_ECC_SECP160R1_ADDR 0x00003c1c
#define ROM_10_ECC_VERIFY_ADDR 0x00003c24

#define FLASH_BASE ( uint32_t ) 0x80000  // start of main app
#define FLASH_PAGE_SIZE ( ( uint32_t ) 0x800 )
// -- eeprom simulate memory map --
#define XTERNAL_EE_SIGNATURE "[SEMTECH_SX126L]"
#define EEPROM_SIM_BASE ( uint32_t ) 0xBE000
#define EEPROM_NEW_DELTA ( uint32_t ) 0xBE010
#define _OK 0
#define _ERR 1

#define PACKED __attribute__( ( packed ) )
#define DELTA_WORD_SIZE 0x00007800  // 30Kb
#define UPDT_FIRM_HEADER ( uint32_t ) 0xB6800

typedef struct delta_partition_header
{
    uint32_t upd_crc;   // the crc of the actual update binary
    uint32_t upd_size;  // the size of actual update binary
    uint32_t upd_fw_crc;
    uint32_t upd_fw_size;
    uint32_t ref_crc_descriptor;
    uint8_t  type;
    uint8_t  rfu1;
    uint8_t  rfu2;
    uint8_t  rfu3;
} PACKED DELTA_PARTITION_HEADER;

typedef uECC_Curve RomuECC_secp160r1( void );
typedef int        RomuECC_verify( const uint8_t* public_key, const uint8_t* message_hash, unsigned hash_size,
                                   const uint8_t* signature, uECC_Curve curve );

extern uint8_t CheckAesHash( );
extern uint8_t VerifySignature( );
extern int8_t  xEEcheck( void );
extern int8_t  xEEformat( void );
extern int8_t  EE_setDeltaUpdateInfo( uint32_t value );

#endif
