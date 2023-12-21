/*!
 * \file      smtc_hal_flash.h
 *
 * \brief     FLASH Hardware Abstraction Layer definition
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
#ifndef __SMTC_HAL_FLASH_H__
#define __SMTC_HAL_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define FLASH_PAGE_NUMBER NRF_FICR->CODESIZE /* 256 pages available on nRF52840 */
#define FLASH_PAGE_PER_BANK NRF_FICR->CODESIZE

#define FLASH_USER_START_ADDR ADDR_FLASH_PAGE_0 /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR \
    FLASH_USER_START_ADDR + ( FLASH_PAGE_NUMBER * ADDR_FLASH_PAGE_SIZE ) - 1 /* End @ of user Flash area */

#define ADDR_FLASH_PAGE_SIZE ( nrfx_nvmc_flash_page_size_get( ) ) /* Size of Page = 4 Kbytes (0x1000)*/

#define FLASH_BYTE_EMPTY_CONTENT ( ( uint8_t ) 0xFF )
#define FLASH_PAGE_EMPTY_CONTENT ( ( uint64_t ) 0xFFFFFFFFFFFFFFFF )

#define FLASH_PAGE_ADDR( page ) FLASH_USER_START_ADDR + ( ( page ) *ADDR_FLASH_PAGE_SIZE )

/* Base address of the Flash s */
#define ADDR_FLASH_PAGE_0 ( ( uint32_t ) 0x00000000 )   /* Base @ of Page 0, 4 Kbytes */
#define ADDR_FLASH_PAGE_1 ( ( uint32_t ) 0x00001000 )   /* Base @ of Page 1, 4 Kbytes */
#define ADDR_FLASH_PAGE_2 ( ( uint32_t ) 0x00002000 )   /* Base @ of Page 2, 4 Kbytes */
#define ADDR_FLASH_PAGE_3 ( ( uint32_t ) 0x00003000 )   /* Base @ of Page 3, 4 Kbytes */
#define ADDR_FLASH_PAGE_4 ( ( uint32_t ) 0x00004000 )   /* Base @ of Page 4, 4 Kbytes */
#define ADDR_FLASH_PAGE_5 ( ( uint32_t ) 0x00005000 )   /* Base @ of Page 5, 4 Kbytes */
#define ADDR_FLASH_PAGE_6 ( ( uint32_t ) 0x00006000 )   /* Base @ of Page 6, 4 Kbytes */
#define ADDR_FLASH_PAGE_7 ( ( uint32_t ) 0x00007000 )   /* Base @ of Page 7, 4 Kbytes */
#define ADDR_FLASH_PAGE_8 ( ( uint32_t ) 0x00008000 )   /* Base @ of Page 8, 4 Kbytes */
#define ADDR_FLASH_PAGE_9 ( ( uint32_t ) 0x00009000 )   /* Base @ of Page 9, 4 Kbytes */
#define ADDR_FLASH_PAGE_10 ( ( uint32_t ) 0x0000A000 )  /* Base @ of Page 10, 4 Kbytes */
#define ADDR_FLASH_PAGE_11 ( ( uint32_t ) 0x0000B000 )  /* Base @ of Page 11, 4 Kbytes */
#define ADDR_FLASH_PAGE_12 ( ( uint32_t ) 0x0000C000 )  /* Base @ of Page 12, 4 Kbytes */
#define ADDR_FLASH_PAGE_13 ( ( uint32_t ) 0x0000D000 )  /* Base @ of Page 13, 4 Kbytes */
#define ADDR_FLASH_PAGE_14 ( ( uint32_t ) 0x0000E000 )  /* Base @ of Page 14, 4 Kbytes */
#define ADDR_FLASH_PAGE_15 ( ( uint32_t ) 0x0000F000 )  /* Base @ of Page 15, 4 Kbytes */
#define ADDR_FLASH_PAGE_16 ( ( uint32_t ) 0x00010000 )  /* Base @ of Page 16, 4 Kbytes */
#define ADDR_FLASH_PAGE_17 ( ( uint32_t ) 0x00011000 )  /* Base @ of Page 17, 4 Kbytes */
#define ADDR_FLASH_PAGE_18 ( ( uint32_t ) 0x00012000 )  /* Base @ of Page 18, 4 Kbytes */
#define ADDR_FLASH_PAGE_19 ( ( uint32_t ) 0x00013000 )  /* Base @ of Page 19, 4 Kbytes */
#define ADDR_FLASH_PAGE_20 ( ( uint32_t ) 0x00014000 )  /* Base @ of Page 20, 4 Kbytes */
#define ADDR_FLASH_PAGE_21 ( ( uint32_t ) 0x00015000 )  /* Base @ of Page 21, 4 Kbytes */
#define ADDR_FLASH_PAGE_22 ( ( uint32_t ) 0x00016000 )  /* Base @ of Page 22, 4 Kbytes */
#define ADDR_FLASH_PAGE_23 ( ( uint32_t ) 0x00017000 )  /* Base @ of Page 23, 4 Kbytes */
#define ADDR_FLASH_PAGE_24 ( ( uint32_t ) 0x00018000 )  /* Base @ of Page 24, 4 Kbytes */
#define ADDR_FLASH_PAGE_25 ( ( uint32_t ) 0x00019000 )  /* Base @ of Page 25, 4 Kbytes */
#define ADDR_FLASH_PAGE_26 ( ( uint32_t ) 0x0001A000 )  /* Base @ of Page 26, 4 Kbytes */
#define ADDR_FLASH_PAGE_27 ( ( uint32_t ) 0x0001B000 )  /* Base @ of Page 27, 4 Kbytes */
#define ADDR_FLASH_PAGE_28 ( ( uint32_t ) 0x0001C000 )  /* Base @ of Page 28, 4 Kbytes */
#define ADDR_FLASH_PAGE_29 ( ( uint32_t ) 0x0001D000 )  /* Base @ of Page 29, 4 Kbytes */
#define ADDR_FLASH_PAGE_30 ( ( uint32_t ) 0x0001E000 )  /* Base @ of Page 30, 4 Kbytes */
#define ADDR_FLASH_PAGE_31 ( ( uint32_t ) 0x0001F000 )  /* Base @ of Page 31, 4 Kbytes */
#define ADDR_FLASH_PAGE_32 ( ( uint32_t ) 0x00020000 )  /* Base @ of Page 32, 4 Kbytes */
#define ADDR_FLASH_PAGE_33 ( ( uint32_t ) 0x00021000 )  /* Base @ of Page 33, 4 Kbytes */
#define ADDR_FLASH_PAGE_34 ( ( uint32_t ) 0x00022000 )  /* Base @ of Page 34, 4 Kbytes */
#define ADDR_FLASH_PAGE_35 ( ( uint32_t ) 0x00023000 )  /* Base @ of Page 35, 4 Kbytes */
#define ADDR_FLASH_PAGE_36 ( ( uint32_t ) 0x00024000 )  /* Base @ of Page 36, 4 Kbytes */
#define ADDR_FLASH_PAGE_37 ( ( uint32_t ) 0x00025000 )  /* Base @ of Page 37, 4 Kbytes */
#define ADDR_FLASH_PAGE_38 ( ( uint32_t ) 0x00026000 )  /* Base @ of Page 38, 4 Kbytes */
#define ADDR_FLASH_PAGE_39 ( ( uint32_t ) 0x00027000 )  /* Base @ of Page 39, 4 Kbytes */
#define ADDR_FLASH_PAGE_40 ( ( uint32_t ) 0x00028000 )  /* Base @ of Page 40, 4 Kbytes */
#define ADDR_FLASH_PAGE_41 ( ( uint32_t ) 0x00029000 )  /* Base @ of Page 41, 4 Kbytes */
#define ADDR_FLASH_PAGE_42 ( ( uint32_t ) 0x0002A000 )  /* Base @ of Page 42, 4 Kbytes */
#define ADDR_FLASH_PAGE_43 ( ( uint32_t ) 0x0002B000 )  /* Base @ of Page 43, 4 Kbytes */
#define ADDR_FLASH_PAGE_44 ( ( uint32_t ) 0x0002C000 )  /* Base @ of Page 44, 4 Kbytes */
#define ADDR_FLASH_PAGE_45 ( ( uint32_t ) 0x0002D000 )  /* Base @ of Page 45, 4 Kbytes */
#define ADDR_FLASH_PAGE_46 ( ( uint32_t ) 0x0002E000 )  /* Base @ of Page 46, 4 Kbytes */
#define ADDR_FLASH_PAGE_47 ( ( uint32_t ) 0x0002F000 )  /* Base @ of Page 47, 4 Kbytes */
#define ADDR_FLASH_PAGE_48 ( ( uint32_t ) 0x00030000 )  /* Base @ of Page 48, 4 Kbytes */
#define ADDR_FLASH_PAGE_49 ( ( uint32_t ) 0x00031000 )  /* Base @ of Page 49, 4 Kbytes */
#define ADDR_FLASH_PAGE_50 ( ( uint32_t ) 0x00032000 )  /* Base @ of Page 50, 4 Kbytes */
#define ADDR_FLASH_PAGE_51 ( ( uint32_t ) 0x00033000 )  /* Base @ of Page 51, 4 Kbytes */
#define ADDR_FLASH_PAGE_52 ( ( uint32_t ) 0x00034000 )  /* Base @ of Page 52, 4 Kbytes */
#define ADDR_FLASH_PAGE_53 ( ( uint32_t ) 0x00035000 )  /* Base @ of Page 53, 4 Kbytes */
#define ADDR_FLASH_PAGE_54 ( ( uint32_t ) 0x00036000 )  /* Base @ of Page 54, 4 Kbytes */
#define ADDR_FLASH_PAGE_55 ( ( uint32_t ) 0x00037000 )  /* Base @ of Page 55, 4 Kbytes */
#define ADDR_FLASH_PAGE_56 ( ( uint32_t ) 0x00038000 )  /* Base @ of Page 56, 4 Kbytes */
#define ADDR_FLASH_PAGE_57 ( ( uint32_t ) 0x00039000 )  /* Base @ of Page 57, 4 Kbytes */
#define ADDR_FLASH_PAGE_58 ( ( uint32_t ) 0x0003A000 )  /* Base @ of Page 58, 4 Kbytes */
#define ADDR_FLASH_PAGE_59 ( ( uint32_t ) 0x0003B000 )  /* Base @ of Page 59, 4 Kbytes */
#define ADDR_FLASH_PAGE_60 ( ( uint32_t ) 0x0003C000 )  /* Base @ of Page 60, 4 Kbytes */
#define ADDR_FLASH_PAGE_61 ( ( uint32_t ) 0x0003D000 )  /* Base @ of Page 61, 4 Kbytes */
#define ADDR_FLASH_PAGE_62 ( ( uint32_t ) 0x0003E000 )  /* Base @ of Page 62, 4 Kbytes */
#define ADDR_FLASH_PAGE_63 ( ( uint32_t ) 0x0003F000 )  /* Base @ of Page 63, 4 Kbytes */
#define ADDR_FLASH_PAGE_64 ( ( uint32_t ) 0x00040000 )  /* Base @ of Page 64, 4 Kbytes */
#define ADDR_FLASH_PAGE_65 ( ( uint32_t ) 0x00041000 )  /* Base @ of Page 65, 4 Kbytes */
#define ADDR_FLASH_PAGE_66 ( ( uint32_t ) 0x00042000 )  /* Base @ of Page 66, 4 Kbytes */
#define ADDR_FLASH_PAGE_67 ( ( uint32_t ) 0x00043000 )  /* Base @ of Page 67, 4 Kbytes */
#define ADDR_FLASH_PAGE_68 ( ( uint32_t ) 0x00044000 )  /* Base @ of Page 68, 4 Kbytes */
#define ADDR_FLASH_PAGE_69 ( ( uint32_t ) 0x00045000 )  /* Base @ of Page 69, 4 Kbytes */
#define ADDR_FLASH_PAGE_70 ( ( uint32_t ) 0x00046000 )  /* Base @ of Page 70, 4 Kbytes */
#define ADDR_FLASH_PAGE_71 ( ( uint32_t ) 0x00047000 )  /* Base @ of Page 71, 4 Kbytes */
#define ADDR_FLASH_PAGE_72 ( ( uint32_t ) 0x00048000 )  /* Base @ of Page 72, 4 Kbytes */
#define ADDR_FLASH_PAGE_73 ( ( uint32_t ) 0x00049000 )  /* Base @ of Page 73, 4 Kbytes */
#define ADDR_FLASH_PAGE_74 ( ( uint32_t ) 0x0004A000 )  /* Base @ of Page 74, 4 Kbytes */
#define ADDR_FLASH_PAGE_75 ( ( uint32_t ) 0x0004B000 )  /* Base @ of Page 75, 4 Kbytes */
#define ADDR_FLASH_PAGE_76 ( ( uint32_t ) 0x0004C000 )  /* Base @ of Page 76, 4 Kbytes */
#define ADDR_FLASH_PAGE_77 ( ( uint32_t ) 0x0004D000 )  /* Base @ of Page 77, 4 Kbytes */
#define ADDR_FLASH_PAGE_78 ( ( uint32_t ) 0x0004E000 )  /* Base @ of Page 78, 4 Kbytes */
#define ADDR_FLASH_PAGE_79 ( ( uint32_t ) 0x0004F000 )  /* Base @ of Page 79, 4 Kbytes */
#define ADDR_FLASH_PAGE_80 ( ( uint32_t ) 0x00050000 )  /* Base @ of Page 80, 4 Kbytes */
#define ADDR_FLASH_PAGE_81 ( ( uint32_t ) 0x00051000 )  /* Base @ of Page 81, 4 Kbytes */
#define ADDR_FLASH_PAGE_82 ( ( uint32_t ) 0x00052000 )  /* Base @ of Page 82, 4 Kbytes */
#define ADDR_FLASH_PAGE_83 ( ( uint32_t ) 0x00053000 )  /* Base @ of Page 83, 4 Kbytes */
#define ADDR_FLASH_PAGE_84 ( ( uint32_t ) 0x00054000 )  /* Base @ of Page 84, 4 Kbytes */
#define ADDR_FLASH_PAGE_85 ( ( uint32_t ) 0x00055000 )  /* Base @ of Page 85, 4 Kbytes */
#define ADDR_FLASH_PAGE_86 ( ( uint32_t ) 0x00056000 )  /* Base @ of Page 86, 4 Kbytes */
#define ADDR_FLASH_PAGE_87 ( ( uint32_t ) 0x00057000 )  /* Base @ of Page 87, 4 Kbytes */
#define ADDR_FLASH_PAGE_88 ( ( uint32_t ) 0x00058000 )  /* Base @ of Page 88, 4 Kbytes */
#define ADDR_FLASH_PAGE_89 ( ( uint32_t ) 0x00059000 )  /* Base @ of Page 89, 4 Kbytes */
#define ADDR_FLASH_PAGE_90 ( ( uint32_t ) 0x0005A000 )  /* Base @ of Page 90, 4 Kbytes */
#define ADDR_FLASH_PAGE_91 ( ( uint32_t ) 0x0005B000 )  /* Base @ of Page 91, 4 Kbytes */
#define ADDR_FLASH_PAGE_92 ( ( uint32_t ) 0x0005C000 )  /* Base @ of Page 92, 4 Kbytes */
#define ADDR_FLASH_PAGE_93 ( ( uint32_t ) 0x0005D000 )  /* Base @ of Page 93, 4 Kbytes */
#define ADDR_FLASH_PAGE_94 ( ( uint32_t ) 0x0005E000 )  /* Base @ of Page 94, 4 Kbytes */
#define ADDR_FLASH_PAGE_95 ( ( uint32_t ) 0x0005F000 )  /* Base @ of Page 95, 4 Kbytes */
#define ADDR_FLASH_PAGE_96 ( ( uint32_t ) 0x00060000 )  /* Base @ of Page 96, 4 Kbytes */
#define ADDR_FLASH_PAGE_97 ( ( uint32_t ) 0x00061000 )  /* Base @ of Page 97, 4 Kbytes */
#define ADDR_FLASH_PAGE_98 ( ( uint32_t ) 0x00062000 )  /* Base @ of Page 98, 4 Kbytes */
#define ADDR_FLASH_PAGE_99 ( ( uint32_t ) 0x00063000 )  /* Base @ of Page 99, 4 Kbytes */
#define ADDR_FLASH_PAGE_100 ( ( uint32_t ) 0x00064000 ) /* Base @ of Page 100, 4 Kbytes */
#define ADDR_FLASH_PAGE_101 ( ( uint32_t ) 0x00065000 ) /* Base @ of Page 101, 4 Kbytes */
#define ADDR_FLASH_PAGE_102 ( ( uint32_t ) 0x00066000 ) /* Base @ of Page 102, 4 Kbytes */
#define ADDR_FLASH_PAGE_103 ( ( uint32_t ) 0x00067000 ) /* Base @ of Page 103, 4 Kbytes */
#define ADDR_FLASH_PAGE_104 ( ( uint32_t ) 0x00068000 ) /* Base @ of Page 104, 4 Kbytes */
#define ADDR_FLASH_PAGE_105 ( ( uint32_t ) 0x00069000 ) /* Base @ of Page 105, 4 Kbytes */
#define ADDR_FLASH_PAGE_106 ( ( uint32_t ) 0x0006A000 ) /* Base @ of Page 106, 4 Kbytes */
#define ADDR_FLASH_PAGE_107 ( ( uint32_t ) 0x0006B000 ) /* Base @ of Page 107, 4 Kbytes */
#define ADDR_FLASH_PAGE_108 ( ( uint32_t ) 0x0006C000 ) /* Base @ of Page 108, 4 Kbytes */
#define ADDR_FLASH_PAGE_109 ( ( uint32_t ) 0x0006D000 ) /* Base @ of Page 109, 4 Kbytes */
#define ADDR_FLASH_PAGE_110 ( ( uint32_t ) 0x0006E000 ) /* Base @ of Page 110, 4 Kbytes */
#define ADDR_FLASH_PAGE_111 ( ( uint32_t ) 0x0006F000 ) /* Base @ of Page 111, 4 Kbytes */
#define ADDR_FLASH_PAGE_112 ( ( uint32_t ) 0x00070000 ) /* Base @ of Page 112, 4 Kbytes */
#define ADDR_FLASH_PAGE_113 ( ( uint32_t ) 0x00071000 ) /* Base @ of Page 113, 4 Kbytes */
#define ADDR_FLASH_PAGE_114 ( ( uint32_t ) 0x00072000 ) /* Base @ of Page 114, 4 Kbytes */
#define ADDR_FLASH_PAGE_115 ( ( uint32_t ) 0x00073000 ) /* Base @ of Page 115, 4 Kbytes */
#define ADDR_FLASH_PAGE_116 ( ( uint32_t ) 0x00074000 ) /* Base @ of Page 116, 4 Kbytes */
#define ADDR_FLASH_PAGE_117 ( ( uint32_t ) 0x00075000 ) /* Base @ of Page 117, 4 Kbytes */
#define ADDR_FLASH_PAGE_118 ( ( uint32_t ) 0x00076000 ) /* Base @ of Page 118, 4 Kbytes */
#define ADDR_FLASH_PAGE_119 ( ( uint32_t ) 0x00077000 ) /* Base @ of Page 119, 4 Kbytes */
#define ADDR_FLASH_PAGE_120 ( ( uint32_t ) 0x00078000 ) /* Base @ of Page 120, 4 Kbytes */
#define ADDR_FLASH_PAGE_121 ( ( uint32_t ) 0x00079000 ) /* Base @ of Page 121, 4 Kbytes */
#define ADDR_FLASH_PAGE_122 ( ( uint32_t ) 0x0007A000 ) /* Base @ of Page 122, 4 Kbytes */
#define ADDR_FLASH_PAGE_123 ( ( uint32_t ) 0x0007B000 ) /* Base @ of Page 123, 4 Kbytes */
#define ADDR_FLASH_PAGE_124 ( ( uint32_t ) 0x0007C000 ) /* Base @ of Page 124, 4 Kbytes */
#define ADDR_FLASH_PAGE_125 ( ( uint32_t ) 0x0007D000 ) /* Base @ of Page 125, 4 Kbytes */
#define ADDR_FLASH_PAGE_126 ( ( uint32_t ) 0x0007E000 ) /* Base @ of Page 126, 4 Kbytes */
#define ADDR_FLASH_PAGE_127 ( ( uint32_t ) 0x0007F000 ) /* Base @ of Page 127, 4 Kbytes */
#define ADDR_FLASH_PAGE_128 ( ( uint32_t ) 0x00080000 ) /* Base @ of Page 128, 4 Kbytes */
#define ADDR_FLASH_PAGE_129 ( ( uint32_t ) 0x00081000 ) /* Base @ of Page 129, 4 Kbytes */
#define ADDR_FLASH_PAGE_130 ( ( uint32_t ) 0x00082000 ) /* Base @ of Page 130, 4 Kbytes */
#define ADDR_FLASH_PAGE_131 ( ( uint32_t ) 0x00083000 ) /* Base @ of Page 131, 4 Kbytes */
#define ADDR_FLASH_PAGE_132 ( ( uint32_t ) 0x00084000 ) /* Base @ of Page 132, 4 Kbytes */
#define ADDR_FLASH_PAGE_133 ( ( uint32_t ) 0x00085000 ) /* Base @ of Page 133, 4 Kbytes */
#define ADDR_FLASH_PAGE_134 ( ( uint32_t ) 0x00086000 ) /* Base @ of Page 134, 4 Kbytes */
#define ADDR_FLASH_PAGE_135 ( ( uint32_t ) 0x00087000 ) /* Base @ of Page 135, 4 Kbytes */
#define ADDR_FLASH_PAGE_136 ( ( uint32_t ) 0x00088000 ) /* Base @ of Page 136, 4 Kbytes */
#define ADDR_FLASH_PAGE_137 ( ( uint32_t ) 0x00089000 ) /* Base @ of Page 137, 4 Kbytes */
#define ADDR_FLASH_PAGE_138 ( ( uint32_t ) 0x0008A000 ) /* Base @ of Page 138, 4 Kbytes */
#define ADDR_FLASH_PAGE_139 ( ( uint32_t ) 0x0008B000 ) /* Base @ of Page 139, 4 Kbytes */
#define ADDR_FLASH_PAGE_140 ( ( uint32_t ) 0x0008C000 ) /* Base @ of Page 140, 4 Kbytes */
#define ADDR_FLASH_PAGE_141 ( ( uint32_t ) 0x0008D000 ) /* Base @ of Page 141, 4 Kbytes */
#define ADDR_FLASH_PAGE_142 ( ( uint32_t ) 0x0008E000 ) /* Base @ of Page 142, 4 Kbytes */
#define ADDR_FLASH_PAGE_143 ( ( uint32_t ) 0x0008F000 ) /* Base @ of Page 143, 4 Kbytes */
#define ADDR_FLASH_PAGE_144 ( ( uint32_t ) 0x00090000 ) /* Base @ of Page 144, 4 Kbytes */
#define ADDR_FLASH_PAGE_145 ( ( uint32_t ) 0x00091000 ) /* Base @ of Page 145, 4 Kbytes */
#define ADDR_FLASH_PAGE_146 ( ( uint32_t ) 0x00092000 ) /* Base @ of Page 146, 4 Kbytes */
#define ADDR_FLASH_PAGE_147 ( ( uint32_t ) 0x00093000 ) /* Base @ of Page 147, 4 Kbytes */
#define ADDR_FLASH_PAGE_148 ( ( uint32_t ) 0x00094000 ) /* Base @ of Page 148, 4 Kbytes */
#define ADDR_FLASH_PAGE_149 ( ( uint32_t ) 0x00095000 ) /* Base @ of Page 149, 4 Kbytes */
#define ADDR_FLASH_PAGE_150 ( ( uint32_t ) 0x00096000 ) /* Base @ of Page 150, 4 Kbytes */
#define ADDR_FLASH_PAGE_151 ( ( uint32_t ) 0x00097000 ) /* Base @ of Page 151, 4 Kbytes */
#define ADDR_FLASH_PAGE_152 ( ( uint32_t ) 0x00098000 ) /* Base @ of Page 152, 4 Kbytes */
#define ADDR_FLASH_PAGE_153 ( ( uint32_t ) 0x00099000 ) /* Base @ of Page 153, 4 Kbytes */
#define ADDR_FLASH_PAGE_154 ( ( uint32_t ) 0x0009A000 ) /* Base @ of Page 154, 4 Kbytes */
#define ADDR_FLASH_PAGE_155 ( ( uint32_t ) 0x0009B000 ) /* Base @ of Page 155, 4 Kbytes */
#define ADDR_FLASH_PAGE_156 ( ( uint32_t ) 0x0009C000 ) /* Base @ of Page 156, 4 Kbytes */
#define ADDR_FLASH_PAGE_157 ( ( uint32_t ) 0x0009D000 ) /* Base @ of Page 157, 4 Kbytes */
#define ADDR_FLASH_PAGE_158 ( ( uint32_t ) 0x0009E000 ) /* Base @ of Page 158, 4 Kbytes */
#define ADDR_FLASH_PAGE_159 ( ( uint32_t ) 0x0009F000 ) /* Base @ of Page 159, 4 Kbytes */
#define ADDR_FLASH_PAGE_160 ( ( uint32_t ) 0x000A0000 ) /* Base @ of Page 160, 4 Kbytes */
#define ADDR_FLASH_PAGE_161 ( ( uint32_t ) 0x000A1000 ) /* Base @ of Page 161, 4 Kbytes */
#define ADDR_FLASH_PAGE_162 ( ( uint32_t ) 0x000A2000 ) /* Base @ of Page 162, 4 Kbytes */
#define ADDR_FLASH_PAGE_163 ( ( uint32_t ) 0x000A3000 ) /* Base @ of Page 163, 4 Kbytes */
#define ADDR_FLASH_PAGE_164 ( ( uint32_t ) 0x000A4000 ) /* Base @ of Page 164, 4 Kbytes */
#define ADDR_FLASH_PAGE_165 ( ( uint32_t ) 0x000A5000 ) /* Base @ of Page 165, 4 Kbytes */
#define ADDR_FLASH_PAGE_166 ( ( uint32_t ) 0x000A6000 ) /* Base @ of Page 166, 4 Kbytes */
#define ADDR_FLASH_PAGE_167 ( ( uint32_t ) 0x000A7000 ) /* Base @ of Page 167, 4 Kbytes */
#define ADDR_FLASH_PAGE_168 ( ( uint32_t ) 0x000A8000 ) /* Base @ of Page 168, 4 Kbytes */
#define ADDR_FLASH_PAGE_169 ( ( uint32_t ) 0x000A9000 ) /* Base @ of Page 169, 4 Kbytes */
#define ADDR_FLASH_PAGE_170 ( ( uint32_t ) 0x000AA000 ) /* Base @ of Page 170, 4 Kbytes */
#define ADDR_FLASH_PAGE_171 ( ( uint32_t ) 0x000AB000 ) /* Base @ of Page 171, 4 Kbytes */
#define ADDR_FLASH_PAGE_172 ( ( uint32_t ) 0x000AC000 ) /* Base @ of Page 172, 4 Kbytes */
#define ADDR_FLASH_PAGE_173 ( ( uint32_t ) 0x000AD000 ) /* Base @ of Page 173, 4 Kbytes */
#define ADDR_FLASH_PAGE_174 ( ( uint32_t ) 0x000AE000 ) /* Base @ of Page 174, 4 Kbytes */
#define ADDR_FLASH_PAGE_175 ( ( uint32_t ) 0x000AF000 ) /* Base @ of Page 175, 4 Kbytes */
#define ADDR_FLASH_PAGE_176 ( ( uint32_t ) 0x000B0000 ) /* Base @ of Page 176, 4 Kbytes */
#define ADDR_FLASH_PAGE_177 ( ( uint32_t ) 0x000B1000 ) /* Base @ of Page 177, 4 Kbytes */
#define ADDR_FLASH_PAGE_178 ( ( uint32_t ) 0x000B2000 ) /* Base @ of Page 178, 4 Kbytes */
#define ADDR_FLASH_PAGE_179 ( ( uint32_t ) 0x000B3000 ) /* Base @ of Page 179, 4 Kbytes */
#define ADDR_FLASH_PAGE_180 ( ( uint32_t ) 0x000B4000 ) /* Base @ of Page 180, 4 Kbytes */
#define ADDR_FLASH_PAGE_181 ( ( uint32_t ) 0x000B5000 ) /* Base @ of Page 181, 4 Kbytes */
#define ADDR_FLASH_PAGE_182 ( ( uint32_t ) 0x000B6000 ) /* Base @ of Page 182, 4 Kbytes */
#define ADDR_FLASH_PAGE_183 ( ( uint32_t ) 0x000B7000 ) /* Base @ of Page 183, 4 Kbytes */
#define ADDR_FLASH_PAGE_184 ( ( uint32_t ) 0x000B8000 ) /* Base @ of Page 184, 4 Kbytes */
#define ADDR_FLASH_PAGE_185 ( ( uint32_t ) 0x000B9000 ) /* Base @ of Page 185, 4 Kbytes */
#define ADDR_FLASH_PAGE_186 ( ( uint32_t ) 0x000BA000 ) /* Base @ of Page 186, 4 Kbytes */
#define ADDR_FLASH_PAGE_187 ( ( uint32_t ) 0x000BB000 ) /* Base @ of Page 187, 4 Kbytes */
#define ADDR_FLASH_PAGE_188 ( ( uint32_t ) 0x000BC000 ) /* Base @ of Page 188, 4 Kbytes */
#define ADDR_FLASH_PAGE_189 ( ( uint32_t ) 0x000BD000 ) /* Base @ of Page 189, 4 Kbytes */
#define ADDR_FLASH_PAGE_190 ( ( uint32_t ) 0x000BE000 ) /* Base @ of Page 190, 4 Kbytes */
#define ADDR_FLASH_PAGE_191 ( ( uint32_t ) 0x000BF000 ) /* Base @ of Page 191, 4 Kbytes */
#define ADDR_FLASH_PAGE_192 ( ( uint32_t ) 0x000C0000 ) /* Base @ of Page 192, 4 Kbytes */
#define ADDR_FLASH_PAGE_193 ( ( uint32_t ) 0x000C1000 ) /* Base @ of Page 193, 4 Kbytes */
#define ADDR_FLASH_PAGE_194 ( ( uint32_t ) 0x000C2000 ) /* Base @ of Page 194, 4 Kbytes */
#define ADDR_FLASH_PAGE_195 ( ( uint32_t ) 0x000C3000 ) /* Base @ of Page 195, 4 Kbytes */
#define ADDR_FLASH_PAGE_196 ( ( uint32_t ) 0x000C4000 ) /* Base @ of Page 196, 4 Kbytes */
#define ADDR_FLASH_PAGE_197 ( ( uint32_t ) 0x000C5000 ) /* Base @ of Page 197, 4 Kbytes */
#define ADDR_FLASH_PAGE_198 ( ( uint32_t ) 0x000C6000 ) /* Base @ of Page 198, 4 Kbytes */
#define ADDR_FLASH_PAGE_199 ( ( uint32_t ) 0x000C7000 ) /* Base @ of Page 199, 4 Kbytes */
#define ADDR_FLASH_PAGE_200 ( ( uint32_t ) 0x000C8000 ) /* Base @ of Page 200, 4 Kbytes */
#define ADDR_FLASH_PAGE_201 ( ( uint32_t ) 0x000C9000 ) /* Base @ of Page 201, 4 Kbytes */
#define ADDR_FLASH_PAGE_202 ( ( uint32_t ) 0x000CA000 ) /* Base @ of Page 202, 4 Kbytes */
#define ADDR_FLASH_PAGE_203 ( ( uint32_t ) 0x000CB000 ) /* Base @ of Page 203, 4 Kbytes */
#define ADDR_FLASH_PAGE_204 ( ( uint32_t ) 0x000CC000 ) /* Base @ of Page 204, 4 Kbytes */
#define ADDR_FLASH_PAGE_205 ( ( uint32_t ) 0x000CD000 ) /* Base @ of Page 205, 4 Kbytes */
#define ADDR_FLASH_PAGE_206 ( ( uint32_t ) 0x000CE000 ) /* Base @ of Page 206, 4 Kbytes */
#define ADDR_FLASH_PAGE_207 ( ( uint32_t ) 0x000CF000 ) /* Base @ of Page 207, 4 Kbytes */
#define ADDR_FLASH_PAGE_208 ( ( uint32_t ) 0x000D0000 ) /* Base @ of Page 208, 4 Kbytes */
#define ADDR_FLASH_PAGE_209 ( ( uint32_t ) 0x000D1000 ) /* Base @ of Page 209, 4 Kbytes */
#define ADDR_FLASH_PAGE_210 ( ( uint32_t ) 0x000D2000 ) /* Base @ of Page 210, 4 Kbytes */
#define ADDR_FLASH_PAGE_211 ( ( uint32_t ) 0x000D3000 ) /* Base @ of Page 211, 4 Kbytes */
#define ADDR_FLASH_PAGE_212 ( ( uint32_t ) 0x000D4000 ) /* Base @ of Page 212, 4 Kbytes */
#define ADDR_FLASH_PAGE_213 ( ( uint32_t ) 0x000D5000 ) /* Base @ of Page 213, 4 Kbytes */
#define ADDR_FLASH_PAGE_214 ( ( uint32_t ) 0x000D6000 ) /* Base @ of Page 214, 4 Kbytes */
#define ADDR_FLASH_PAGE_215 ( ( uint32_t ) 0x000D7000 ) /* Base @ of Page 215, 4 Kbytes */
#define ADDR_FLASH_PAGE_216 ( ( uint32_t ) 0x000D8000 ) /* Base @ of Page 216, 4 Kbytes */
#define ADDR_FLASH_PAGE_217 ( ( uint32_t ) 0x000D9000 ) /* Base @ of Page 217, 4 Kbytes */
#define ADDR_FLASH_PAGE_218 ( ( uint32_t ) 0x000DA000 ) /* Base @ of Page 218, 4 Kbytes */
#define ADDR_FLASH_PAGE_219 ( ( uint32_t ) 0x000DB000 ) /* Base @ of Page 219, 4 Kbytes */
#define ADDR_FLASH_PAGE_220 ( ( uint32_t ) 0x000DC000 ) /* Base @ of Page 220, 4 Kbytes */
#define ADDR_FLASH_PAGE_221 ( ( uint32_t ) 0x000DD000 ) /* Base @ of Page 221, 4 Kbytes */
#define ADDR_FLASH_PAGE_222 ( ( uint32_t ) 0x000DE000 ) /* Base @ of Page 222, 4 Kbytes */
#define ADDR_FLASH_PAGE_223 ( ( uint32_t ) 0x000DF000 ) /* Base @ of Page 223, 4 Kbytes */
#define ADDR_FLASH_PAGE_224 ( ( uint32_t ) 0x000E0000 ) /* Base @ of Page 224, 4 Kbytes */
#define ADDR_FLASH_PAGE_225 ( ( uint32_t ) 0x000E1000 ) /* Base @ of Page 225, 4 Kbytes */
#define ADDR_FLASH_PAGE_226 ( ( uint32_t ) 0x000E2000 ) /* Base @ of Page 226, 4 Kbytes */
#define ADDR_FLASH_PAGE_227 ( ( uint32_t ) 0x000E3000 ) /* Base @ of Page 227, 4 Kbytes */
#define ADDR_FLASH_PAGE_228 ( ( uint32_t ) 0x000E4000 ) /* Base @ of Page 228, 4 Kbytes */
#define ADDR_FLASH_PAGE_229 ( ( uint32_t ) 0x000E5000 ) /* Base @ of Page 229, 4 Kbytes */
#define ADDR_FLASH_PAGE_230 ( ( uint32_t ) 0x000E6000 ) /* Base @ of Page 230, 4 Kbytes */
#define ADDR_FLASH_PAGE_231 ( ( uint32_t ) 0x000E7000 ) /* Base @ of Page 231, 4 Kbytes */
#define ADDR_FLASH_PAGE_232 ( ( uint32_t ) 0x000E8000 ) /* Base @ of Page 232, 4 Kbytes */
#define ADDR_FLASH_PAGE_233 ( ( uint32_t ) 0x000E9000 ) /* Base @ of Page 233, 4 Kbytes */
#define ADDR_FLASH_PAGE_234 ( ( uint32_t ) 0x000EA000 ) /* Base @ of Page 234, 4 Kbytes */
#define ADDR_FLASH_PAGE_235 ( ( uint32_t ) 0x000EB000 ) /* Base @ of Page 235, 4 Kbytes */
#define ADDR_FLASH_PAGE_236 ( ( uint32_t ) 0x000EC000 ) /* Base @ of Page 236, 4 Kbytes */
#define ADDR_FLASH_PAGE_237 ( ( uint32_t ) 0x000ED000 ) /* Base @ of Page 237, 4 Kbytes */
#define ADDR_FLASH_PAGE_238 ( ( uint32_t ) 0x000EE000 ) /* Base @ of Page 238, 4 Kbytes */
#define ADDR_FLASH_PAGE_239 ( ( uint32_t ) 0x000EF000 ) /* Base @ of Page 239, 4 Kbytes */
#define ADDR_FLASH_PAGE_240 ( ( uint32_t ) 0x000F0000 ) /* Base @ of Page 240, 4 Kbytes */
#define ADDR_FLASH_PAGE_241 ( ( uint32_t ) 0x000F1000 ) /* Base @ of Page 241, 4 Kbytes */
#define ADDR_FLASH_PAGE_242 ( ( uint32_t ) 0x000F2000 ) /* Base @ of Page 242, 4 Kbytes */
#define ADDR_FLASH_PAGE_243 ( ( uint32_t ) 0x000F3000 ) /* Base @ of Page 243, 4 Kbytes */
#define ADDR_FLASH_PAGE_244 ( ( uint32_t ) 0x000F4000 ) /* Base @ of Page 244, 4 Kbytes */
#define ADDR_FLASH_PAGE_245 ( ( uint32_t ) 0x000F5000 ) /* Base @ of Page 245, 4 Kbytes */
#define ADDR_FLASH_PAGE_246 ( ( uint32_t ) 0x000F6000 ) /* Base @ of Page 246, 4 Kbytes */
#define ADDR_FLASH_PAGE_247 ( ( uint32_t ) 0x000F7000 ) /* Base @ of Page 247, 4 Kbytes */
#define ADDR_FLASH_PAGE_248 ( ( uint32_t ) 0x000F8000 ) /* Base @ of Page 248, 4 Kbytes */
#define ADDR_FLASH_PAGE_249 ( ( uint32_t ) 0x000F9000 ) /* Base @ of Page 249, 4 Kbytes */
#define ADDR_FLASH_PAGE_250 ( ( uint32_t ) 0x000FA000 ) /* Base @ of Page 250, 4 Kbytes */
#define ADDR_FLASH_PAGE_251 ( ( uint32_t ) 0x000FB000 ) /* Base @ of Page 251, 4 Kbytes */
#define ADDR_FLASH_PAGE_252 ( ( uint32_t ) 0x000FC000 ) /* Base @ of Page 252, 4 Kbytes */
#define ADDR_FLASH_PAGE_253 ( ( uint32_t ) 0x000FD000 ) /* Base @ of Page 253, 4 Kbytes */
#define ADDR_FLASH_PAGE_254 ( ( uint32_t ) 0x000FE000 ) /* Base @ of Page 254, 4 Kbytes */
#define ADDR_FLASH_PAGE_255 ( ( uint32_t ) 0x000FF000 ) /* Base @ of Page 255, 4 Kbytes */

#define FLASH_OPERATION_MAX_RETRY 2

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief return the size of one page in flash
 *
 * @return uint16_t
 */
uint16_t hal_flash_get_page_size( );

/*!
 * \brief Erase a given nb page to the FLASH at the specified address.
 *
 * \param[IN] addr FLASH address to start the erase
 * \param[IN] nb_page the number of page to erase.
 * \retval status [SUCCESS, FAIL]
 */
uint8_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page );

/*!
 * \brief Writes the given buffer to the FLASH at the specified address.
 *
 * \param[IN] addr FLASH address to write to
 * \param[IN] buffer Pointer to the buffer to be written.
 * \param[IN] size Size of the buffer to be written.
 * \retval status [Real_size_written, FAIL]
 */
uint32_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size );

/*!
 * \brief Reads the FLASH at the specified address to the given buffer.
 *
 * \param[IN] addr FLASH address to read from
 * \param[OUT] buffer Pointer to the buffer to be written with read data.
 * \param[IN] size Size of the buffer to be read.
 * \retval status [SUCCESS, FAIL]
 */
void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size );

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
/**
 * @brief Reads a flash page, modify it, erase page and then write it
 *
 * @param [in] addr FLASH address
 * @param [in] buffer Pointer to the buffer to be written.
 * @param [in] size Size of the buffer to be written.
 */
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size );
#endif  // USE_FLASH_READ_MODIFY_WRITE or MULTISTACK

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_FLASH_H__

/* --- EOF ------------------------------------------------------------------ */
