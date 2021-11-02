/**
 * @file      patch_upd.c
 *
 * @brief     Patch update sources
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>
#include "patch_upd.h"
#include "core.h"
#include "crypto.h"
#include "nvmcu_hal.h"
// #include "debug.h"

static const uint8_t PubKey[40] = { 0x7b, 0x5e, 0x8f, 0x4f, 0x5b, 0xdc, 0xc2, 0xc8, 0x7a, 0x5d, 0x85, 0xce, 0xc7, 0x91,
                                    0x5d, 0xa9, 0x9d, 0xe6, 0x7b, 0xf2, 0x6b, 0x1e, 0x50, 0x3b, 0x0f, 0x2e, 0x07, 0x9f,
                                    0xb2, 0xd8, 0x53, 0x3e, 0xc0, 0x7c, 0x79, 0xa2, 0x82, 0x7e, 0x59, 0x13 };

static const uint32_t RomBootFuncAddr_200f[] = { ROM_0F_FLASH_ECC_SECP160R1_ADDR, ROM_0F_ECC_VERIFY_ADDR };

static const uint32_t RomBootFuncAddr_2010[] = { ROM_10_FLASH_ECC_SECP160R1_ADDR, ROM_10_ECC_VERIFY_ADDR };

static uint32_t GetPage( uint32_t Addr )
{
    uint32_t page = 0;

    page = ( Addr - FLASH_BASE ) / FLASH_PAGE_SIZE;
    return page;
}
int8_t xEEformat( void )
{
    uint8_t  values[4];
    uint8_t  sign[16];
    uint32_t target_address = EEPROM_SIM_BASE;
    uint32_t page_addr      = EEPROM_SIM_BASE;

    uint32_t page_idx = GetPage( page_addr );
    if( FlashErasePage( page_idx, MainFlash ) != 1 )
    {
        DEBUG_PRINT( DBG_INFO, "Erase page %d error\n", page_addr );
        return _ERR;
    }

    memcpy( sign, XTERNAL_EE_SIGNATURE, 16 );
    // for(i=0; i<16; i++){
    // 	DEBUG_PRINT(DBG_INFO, "sign_ee[%d]:%x\n",i,sign[i]);
    // }

    target_address = ( target_address - FLASH_BASE );
    if( FlashWrite( target_address >> 2, ( _Unaligned uint32_t* ) &sign[0], 16 / 4, 0, MainFlash ) != 1 )
    {
        DEBUG_PRINT( DBG_FATAL, "EEflash temp write error:%x\n", target_address );
        return _ERR;
    }

    values[0]      = 1;
    values[1]      = 0;
    values[2]      = 0;
    values[3]      = 0;
    target_address = EEPROM_NEW_DELTA;
    target_address = ( target_address - FLASH_BASE );
    if( FlashWrite( target_address >> 2, ( _Unaligned uint32_t* ) &values[0], 4 / 4, 0, MainFlash ) != 1 )
    {
        DEBUG_PRINT( DBG_FATAL, "EE_flash temp write error:%x\n", target_address );
        return _ERR;
    }
    return _OK;
}

int8_t xEEcheck( void )
{
    uint8_t sign[16];
    memset( sign, 0, 16 );
    uint8_t    i;
    _Unaligned uint32_t* memAdrPtr;
    uint32_t             dataLen;
    uint8_t*             mem8AdrPtr;
    _Unaligned uint16_t* mem16AdrPtr;

    memAdrPtr   = ( uint32_t* ) EEPROM_SIM_BASE;
    dataLen     = 16;  // size of eeprom sign
    mem16AdrPtr = ( uint16_t* ) memAdrPtr;
    for( i = 0; i < ( dataLen / 2 ); i++ )
    {
        *( uint16_t* ) &sign[i * 2] = *mem16AdrPtr++;
    }
    if( ( i * 2 ) < dataLen )
    {
        mem8AdrPtr  = ( uint8_t* ) mem16AdrPtr;
        sign[i * 2] = *mem8AdrPtr;
    }
    if( memcmp( XTERNAL_EE_SIGNATURE, sign, 16 ) != 0 )
    {
        return _ERR;
    }
    DEBUG_PRINT( DBG_INFO, "SIM EEPROM is OK\n" );  // delta test
    return _OK;
}

int8_t EE_setDeltaUpdateInfo( uint32_t value )
{
    uint8_t    info[4];
    uint16_t   i;
    uint32_t   target_address;
    uint8_t    temp[20];
    _Unaligned uint32_t* memAdrPtr;
    uint32_t             dataLen;
    uint8_t*             mem8AdrPtr;
    _Unaligned uint16_t* mem16AdrPtr;

    memAdrPtr   = ( uint32_t* ) EEPROM_SIM_BASE;
    dataLen     = 20;
    mem16AdrPtr = ( uint16_t* ) memAdrPtr;
    for( i = 0; i < ( dataLen / 2 ); i++ )
    {
        *( uint16_t* ) &temp[i * 2] = *mem16AdrPtr++;
    }
    if( ( i * 2 ) < dataLen )
    {
        mem8AdrPtr  = ( uint8_t* ) mem16AdrPtr;
        temp[i * 2] = *mem8AdrPtr;
    }
    info[0] = ( uint8_t ) value;
    info[1] = ( uint8_t )( ( value & 0x0000FF00 ) >> 8 );
    info[2] = ( uint8_t )( ( value & 0x00FF0000 ) >> 16 );
    info[3] = ( uint8_t )( ( value & 0xFF000000 ) >> 24 );
    for( i = 0; i < 4; i++ )
    {
        DEBUG_PRINT( DBG_INFO, "setInfo[%d]:%x\n", i, info[i] );
        temp[i + 16] = info[i];
    }
    target_address    = EEPROM_SIM_BASE;
    uint32_t page_idx = GetPage( target_address );

    DEBUG_PRINT( DBG_INFO, "Erase page %d \n", page_idx );  // delta test
    if( FlashErasePage( page_idx, MainFlash ) != 1 )
    {
        DEBUG_PRINT( DBG_FATAL, "Erase page %d error\n", page_idx );
        return _ERR;
    }
    DEBUG_PRINT( DBG_INFO, "EE_flash temp write:%x\n", target_address );  // delta test
    target_address = ( target_address - FLASH_BASE );
    if( FlashWrite( target_address >> 2, ( _Unaligned uint32_t* ) &temp[0], 2048 / 4, 0, MainFlash ) != 1 )
    {
        DEBUG_PRINT( DBG_FATAL, "EE_flash temp write error:%x\n", target_address );
        return _ERR;
    }

    return 0;
}

uint32_t CalcFwAes128CheckSum( )
{
    // uint32_t* flash;
    uint32_t flashStartAddr;
    uint32_t len;
    uint32_t mic;

    DEBUG_PRINT( DBG_INFO, "Calc CMAC...\n" );

    EnableCrypto( );

    if( CryptoWriteKey32( CRYPTO_AppSKey_IDX, ( uint32_t[] ){ 0, 0, 0, 0 } ) != 0 )
    {
        DEBUG_PRINT( DBG_ERROR, "CryptoWriteKey failed\n" );
        DisableCrypto( );
        return 0;
    }

    flashStartAddr                       = ( uint32_t ) UPDT_FIRM_HEADER;
    DELTA_PARTITION_HEADER* delta_header = ( DELTA_PARTITION_HEADER* ) UPDT_FIRM_HEADER;
    len                                  = delta_header->upd_crc;
    len                                  = len >> 2;
    // DEBUG_PRINT(DBG_INFO, "data: %x\n",*flash++ );
    // flash = (uint32_t*)flashStartAddr; // for test
    // len = *flash;
    // DEBUG_PRINT(DBG_INFO, "flash:%x,Len: 0x%x\n", *flash++,len);
    // DEBUG_PRINT(DBG_INFO, "Len: 0x%x\n", len);
    if( len > DELTA_WORD_SIZE )
    {
        DEBUG_PRINT( DBG_ERROR, "Reading length of flash failed\n" );
        DisableCrypto( );
        return 0;
    }
    if( CryptoMic32( ( uint32_t* ) flashStartAddr, len / 4, CRYPTO_AppSKey_IDX, 1, &mic, 0 ) != 0 )
    {
        DEBUG_PRINT( DBG_ERROR, "CryptoComputeMic failed\n" );
        DisableCrypto( );
        return 0;
    }

    DisableCrypto( );
    return len;
}

uint8_t CheckAesHash( )
{
    uint8_t verified = 1;

    uint32_t* flash;
    uint32_t  flashStartAddr;
    uint32_t  len;

    DEBUG_PRINT( DBG_INFO, "Check Aes hash\n" );  // for delta test
    flashStartAddr = ( uint32_t ) UPDT_FIRM_HEADER;
    len            = CalcFwAes128CheckSum( );
    if( len == 0 )
    {
        return 0;
    }
    flash = ( uint32_t* ) flashStartAddr + len;
    // check all 128bits in register aes data_out
    if( ( P_CRYPTO->dataBufferOut0.reg32 != *flash++ ) || ( P_CRYPTO->dataBufferOut1.reg32 != *flash++ ) ||
        ( P_CRYPTO->dataBufferOut2.reg32 != *flash++ ) || ( P_CRYPTO->dataBufferOut3.reg32 != *flash ) )
    {
        DEBUG_PRINT( DBG_ERROR, "CryptoComputeMic on flash failed\n" );
        verified = 0;
    }
    return verified;
}

uint8_t VerifySignature( )
{
    // signature algorithm:
    // 1) Calculate e = HASH ( m ), where HASH is a cryptographic hash function (AES-128 in our case with key=000000)
    // 2) calculate the signature of the hash e with the private key with ECDSA
    // 3) store signature in the flash after the fw

    // verify agorithm :
    // 1) Calculate e = HASH ( m ), where HASH is a cryptographic hash function (AES-128 in our case with key=000000)
    // 2) Check the stored signature with the public key

    //    not whole flash: only between 0 and stop addr in flash +  stop addr  (just after irq vectors in flash)
    // 2) encrypted hash is stored just after stop addr  (aes hash )
    //    encrypt the flash hash with  ecc asymetric algo the hash (paded with zeros if needed) with the public key
    //    stored in the rom.
    // 3) encrypted hash from the flash  == with the calculated one

    // DEBUG_PRINT(DBG_INFO, "verify sign\n");
    uint32_t* flash;
    uint32_t  flashStartAddr;
    uint32_t  len;
    uint32_t* versionAddr;
    // int ver_res;
    DEBUG_PRINT( DBG_INFO, "Check Signature\n" );  // for delta test
    flashStartAddr = ( uint32_t ) UPDT_FIRM_HEADER;
    len            = CalcFwAes128CheckSum( );
    if( len == 0 )
    {
        return 0;
    }

    uint8_t    hash[16];
    _Unaligned uint32_t* ptr = ( uint32_t* ) &hash[0];
    *ptr++                   = Swap32( P_CRYPTO->dataBufferOut0.reg32 );
    *ptr++                   = Swap32( P_CRYPTO->dataBufferOut1.reg32 );
    *ptr++                   = Swap32( P_CRYPTO->dataBufferOut2.reg32 );
    *ptr                     = Swap32( P_CRYPTO->dataBufferOut3.reg32 );

    flash       = ( uint32_t* ) flashStartAddr + len;
    versionAddr = ( uint32_t* ) 0x00007FCC;
    RomuECC_secp160r1* f;
    RomuECC_verify*    verify_f;

    if( Swap32( *versionAddr ) == 0x2010ffff )
    {
        DEBUG_PRINT( DBG_INFO, "2010 version:%x\n", Swap32( *versionAddr ) );
        f        = ( RomuECC_secp160r1* ) RomBootFuncAddr_2010[ROM_FLASH_ECC_SECP160R1_IDX];
        verify_f = ( RomuECC_verify* ) RomBootFuncAddr_2010[ROM_ECC_VERIFY_IDX];
    }
    else
    {
        DEBUG_PRINT( DBG_INFO, "200f version:%x\n", Swap32( *versionAddr ) );
        f        = ( RomuECC_secp160r1* ) RomBootFuncAddr_200f[ROM_FLASH_ECC_SECP160R1_IDX];
        verify_f = ( RomuECC_verify* ) RomBootFuncAddr_200f[ROM_ECC_VERIFY_IDX];
    }
    //#ifdef USE_secp160r1
    uECC_Curve res;
    // RomuECC_secp160r1* f = (RomuECC_secp160r1*)0x00003c1c;
    res = f( );
    // RomuECC_verify* verify_f =(RomuECC_verify*)0x00003c24;
    return verify_f( PubKey, hash, 16, ( uint8_t* ) flash, res );
    // #endif

    // return 0;
}
