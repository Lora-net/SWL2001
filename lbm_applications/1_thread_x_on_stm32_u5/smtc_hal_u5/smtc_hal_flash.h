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
#define FLASH_PAGE_NUMBER 256 /* 512 pages available on U575 */
#define FLASH_PAGE_PER_BANK 128

#define FLASH_USER_START_ADDR ADDR_FLASH_PAGE_0 /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR \
    FLASH_USER_START_ADDR + ( FLASH_PAGE_NUMBER * ADDR_FLASH_PAGE_SIZE ) - 1 /* End @ of user Flash area */

#define ADDR_FLASH_PAGE_SIZE ( ( uint32_t ) 0x00002000 ) /* Size of Page = 8 Kbytes */

#define FLASH_BYTE_EMPTY_CONTENT ( ( uint8_t ) 0xFF )
#define FLASH_PAGE_EMPTY_CONTENT ( ( uint64_t ) 0xFFFFFFFFFFFFFFFF )

#define FLASH_PAGE_ADDR( page ) FLASH_USER_START_ADDR + ( ( page ) * ADDR_FLASH_PAGE_SIZE )

/* Base address of the Flash s */
#define ADDR_FLASH_PAGE_0 ( ( uint32_t ) 0x08000000 )   /* Base @ of Page 0, 8 Kbytes */
#define ADDR_FLASH_PAGE_1 ( ( uint32_t ) 0x08002000 )   /* Base @ of Page 1, 8 Kbytes */
#define ADDR_FLASH_PAGE_2 ( ( uint32_t ) 0x08004000 )   /* Base @ of Page 2, 8 Kbytes */
#define ADDR_FLASH_PAGE_3 ( ( uint32_t ) 0x08006000 )   /* Base @ of Page 3, 8 Kbytes */
#define ADDR_FLASH_PAGE_4 ( ( uint32_t ) 0x08008000 )   /* Base @ of Page 4, 8 Kbytes */
#define ADDR_FLASH_PAGE_5 ( ( uint32_t ) 0x0800a000 )   /* Base @ of Page 5, 8 Kbytes */
#define ADDR_FLASH_PAGE_6 ( ( uint32_t ) 0x0800c000 )   /* Base @ of Page 6, 8 Kbytes */
#define ADDR_FLASH_PAGE_7 ( ( uint32_t ) 0x0800e000 )   /* Base @ of Page 7, 8 Kbytes */
#define ADDR_FLASH_PAGE_8 ( ( uint32_t ) 0x08010000 )   /* Base @ of Page 8, 8 Kbytes */
#define ADDR_FLASH_PAGE_9 ( ( uint32_t ) 0x08012000 )   /* Base @ of Page 9, 8 Kbytes */
#define ADDR_FLASH_PAGE_10 ( ( uint32_t ) 0x08014000 )  /* Base @ of Page 10, 8 Kbytes */
#define ADDR_FLASH_PAGE_11 ( ( uint32_t ) 0x08016000 )  /* Base @ of Page 11, 8 Kbytes */
#define ADDR_FLASH_PAGE_12 ( ( uint32_t ) 0x08018000 )  /* Base @ of Page 12, 8 Kbytes */
#define ADDR_FLASH_PAGE_13 ( ( uint32_t ) 0x0801a000 )  /* Base @ of Page 13, 8 Kbytes */
#define ADDR_FLASH_PAGE_14 ( ( uint32_t ) 0x0801c000 )  /* Base @ of Page 14, 8 Kbytes */
#define ADDR_FLASH_PAGE_15 ( ( uint32_t ) 0x0801e000 )  /* Base @ of Page 15, 8 Kbytes */
#define ADDR_FLASH_PAGE_16 ( ( uint32_t ) 0x08020000 )  /* Base @ of Page 16, 8 Kbytes */
#define ADDR_FLASH_PAGE_17 ( ( uint32_t ) 0x08022000 )  /* Base @ of Page 17, 8 Kbytes */
#define ADDR_FLASH_PAGE_18 ( ( uint32_t ) 0x08024000 )  /* Base @ of Page 18, 8 Kbytes */
#define ADDR_FLASH_PAGE_19 ( ( uint32_t ) 0x08026000 )  /* Base @ of Page 19, 8 Kbytes */
#define ADDR_FLASH_PAGE_20 ( ( uint32_t ) 0x08028000 )  /* Base @ of Page 20, 8 Kbytes */
#define ADDR_FLASH_PAGE_21 ( ( uint32_t ) 0x0802a000 )  /* Base @ of Page 21, 8 Kbytes */
#define ADDR_FLASH_PAGE_22 ( ( uint32_t ) 0x0802c000 )  /* Base @ of Page 22, 8 Kbytes */
#define ADDR_FLASH_PAGE_23 ( ( uint32_t ) 0x0802e000 )  /* Base @ of Page 23, 8 Kbytes */
#define ADDR_FLASH_PAGE_24 ( ( uint32_t ) 0x08030000 )  /* Base @ of Page 24, 8 Kbytes */
#define ADDR_FLASH_PAGE_25 ( ( uint32_t ) 0x08032000 )  /* Base @ of Page 25, 8 Kbytes */
#define ADDR_FLASH_PAGE_26 ( ( uint32_t ) 0x08034000 )  /* Base @ of Page 26, 8 Kbytes */
#define ADDR_FLASH_PAGE_27 ( ( uint32_t ) 0x08036000 )  /* Base @ of Page 27, 8 Kbytes */
#define ADDR_FLASH_PAGE_28 ( ( uint32_t ) 0x08038000 )  /* Base @ of Page 28, 8 Kbytes */
#define ADDR_FLASH_PAGE_29 ( ( uint32_t ) 0x0803a000 )  /* Base @ of Page 29, 8 Kbytes */
#define ADDR_FLASH_PAGE_30 ( ( uint32_t ) 0x0803c000 )  /* Base @ of Page 30, 8 Kbytes */
#define ADDR_FLASH_PAGE_31 ( ( uint32_t ) 0x0803e000 )  /* Base @ of Page 31, 8 Kbytes */
#define ADDR_FLASH_PAGE_32 ( ( uint32_t ) 0x08040000 )  /* Base @ of Page 32, 8 Kbytes */
#define ADDR_FLASH_PAGE_33 ( ( uint32_t ) 0x08042000 )  /* Base @ of Page 33, 8 Kbytes */
#define ADDR_FLASH_PAGE_34 ( ( uint32_t ) 0x08044000 )  /* Base @ of Page 34, 8 Kbytes */
#define ADDR_FLASH_PAGE_35 ( ( uint32_t ) 0x08046000 )  /* Base @ of Page 35, 8 Kbytes */
#define ADDR_FLASH_PAGE_36 ( ( uint32_t ) 0x08048000 )  /* Base @ of Page 36, 8 Kbytes */
#define ADDR_FLASH_PAGE_37 ( ( uint32_t ) 0x0804a000 )  /* Base @ of Page 37, 8 Kbytes */
#define ADDR_FLASH_PAGE_38 ( ( uint32_t ) 0x0804c000 )  /* Base @ of Page 38, 8 Kbytes */
#define ADDR_FLASH_PAGE_39 ( ( uint32_t ) 0x0804e000 )  /* Base @ of Page 39, 8 Kbytes */
#define ADDR_FLASH_PAGE_40 ( ( uint32_t ) 0x08050000 )  /* Base @ of Page 40, 8 Kbytes */
#define ADDR_FLASH_PAGE_41 ( ( uint32_t ) 0x08052000 )  /* Base @ of Page 41, 8 Kbytes */
#define ADDR_FLASH_PAGE_42 ( ( uint32_t ) 0x08054000 )  /* Base @ of Page 42, 8 Kbytes */
#define ADDR_FLASH_PAGE_43 ( ( uint32_t ) 0x08056000 )  /* Base @ of Page 43, 8 Kbytes */
#define ADDR_FLASH_PAGE_44 ( ( uint32_t ) 0x08058000 )  /* Base @ of Page 44, 8 Kbytes */
#define ADDR_FLASH_PAGE_45 ( ( uint32_t ) 0x0805a000 )  /* Base @ of Page 45, 8 Kbytes */
#define ADDR_FLASH_PAGE_46 ( ( uint32_t ) 0x0805c000 )  /* Base @ of Page 46, 8 Kbytes */
#define ADDR_FLASH_PAGE_47 ( ( uint32_t ) 0x0805e000 )  /* Base @ of Page 47, 8 Kbytes */
#define ADDR_FLASH_PAGE_48 ( ( uint32_t ) 0x08060000 )  /* Base @ of Page 48, 8 Kbytes */
#define ADDR_FLASH_PAGE_49 ( ( uint32_t ) 0x08062000 )  /* Base @ of Page 49, 8 Kbytes */
#define ADDR_FLASH_PAGE_50 ( ( uint32_t ) 0x08064000 )  /* Base @ of Page 50, 8 Kbytes */
#define ADDR_FLASH_PAGE_51 ( ( uint32_t ) 0x08066000 )  /* Base @ of Page 51, 8 Kbytes */
#define ADDR_FLASH_PAGE_52 ( ( uint32_t ) 0x08068000 )  /* Base @ of Page 52, 8 Kbytes */
#define ADDR_FLASH_PAGE_53 ( ( uint32_t ) 0x0806a000 )  /* Base @ of Page 53, 8 Kbytes */
#define ADDR_FLASH_PAGE_54 ( ( uint32_t ) 0x0806c000 )  /* Base @ of Page 54, 8 Kbytes */
#define ADDR_FLASH_PAGE_55 ( ( uint32_t ) 0x0806e000 )  /* Base @ of Page 55, 8 Kbytes */
#define ADDR_FLASH_PAGE_56 ( ( uint32_t ) 0x08070000 )  /* Base @ of Page 56, 8 Kbytes */
#define ADDR_FLASH_PAGE_57 ( ( uint32_t ) 0x08072000 )  /* Base @ of Page 57, 8 Kbytes */
#define ADDR_FLASH_PAGE_58 ( ( uint32_t ) 0x08074000 )  /* Base @ of Page 58, 8 Kbytes */
#define ADDR_FLASH_PAGE_59 ( ( uint32_t ) 0x08076000 )  /* Base @ of Page 59, 8 Kbytes */
#define ADDR_FLASH_PAGE_60 ( ( uint32_t ) 0x08078000 )  /* Base @ of Page 60, 8 Kbytes */
#define ADDR_FLASH_PAGE_61 ( ( uint32_t ) 0x0807a000 )  /* Base @ of Page 61, 8 Kbytes */
#define ADDR_FLASH_PAGE_62 ( ( uint32_t ) 0x0807c000 )  /* Base @ of Page 62, 8 Kbytes */
#define ADDR_FLASH_PAGE_63 ( ( uint32_t ) 0x0807e000 )  /* Base @ of Page 63, 8 Kbytes */
#define ADDR_FLASH_PAGE_64 ( ( uint32_t ) 0x08080000 )  /* Base @ of Page 64, 8 Kbytes */
#define ADDR_FLASH_PAGE_65 ( ( uint32_t ) 0x08082000 )  /* Base @ of Page 65, 8 Kbytes */
#define ADDR_FLASH_PAGE_66 ( ( uint32_t ) 0x08084000 )  /* Base @ of Page 66, 8 Kbytes */
#define ADDR_FLASH_PAGE_67 ( ( uint32_t ) 0x08086000 )  /* Base @ of Page 67, 8 Kbytes */
#define ADDR_FLASH_PAGE_68 ( ( uint32_t ) 0x08088000 )  /* Base @ of Page 68, 8 Kbytes */
#define ADDR_FLASH_PAGE_69 ( ( uint32_t ) 0x0808a000 )  /* Base @ of Page 69, 8 Kbytes */
#define ADDR_FLASH_PAGE_70 ( ( uint32_t ) 0x0808c000 )  /* Base @ of Page 70, 8 Kbytes */
#define ADDR_FLASH_PAGE_71 ( ( uint32_t ) 0x0808e000 )  /* Base @ of Page 71, 8 Kbytes */
#define ADDR_FLASH_PAGE_72 ( ( uint32_t ) 0x08090000 )  /* Base @ of Page 72, 8 Kbytes */
#define ADDR_FLASH_PAGE_73 ( ( uint32_t ) 0x08092000 )  /* Base @ of Page 73, 8 Kbytes */
#define ADDR_FLASH_PAGE_74 ( ( uint32_t ) 0x08094000 )  /* Base @ of Page 74, 8 Kbytes */
#define ADDR_FLASH_PAGE_75 ( ( uint32_t ) 0x08096000 )  /* Base @ of Page 75, 8 Kbytes */
#define ADDR_FLASH_PAGE_76 ( ( uint32_t ) 0x08098000 )  /* Base @ of Page 76, 8 Kbytes */
#define ADDR_FLASH_PAGE_77 ( ( uint32_t ) 0x0809a000 )  /* Base @ of Page 77, 8 Kbytes */
#define ADDR_FLASH_PAGE_78 ( ( uint32_t ) 0x0809c000 )  /* Base @ of Page 78, 8 Kbytes */
#define ADDR_FLASH_PAGE_79 ( ( uint32_t ) 0x0809e000 )  /* Base @ of Page 79, 8 Kbytes */
#define ADDR_FLASH_PAGE_80 ( ( uint32_t ) 0x080a0000 )  /* Base @ of Page 80, 8 Kbytes */
#define ADDR_FLASH_PAGE_81 ( ( uint32_t ) 0x080a2000 )  /* Base @ of Page 81, 8 Kbytes */
#define ADDR_FLASH_PAGE_82 ( ( uint32_t ) 0x080a4000 )  /* Base @ of Page 82, 8 Kbytes */
#define ADDR_FLASH_PAGE_83 ( ( uint32_t ) 0x080a6000 )  /* Base @ of Page 83, 8 Kbytes */
#define ADDR_FLASH_PAGE_84 ( ( uint32_t ) 0x080a8000 )  /* Base @ of Page 84, 8 Kbytes */
#define ADDR_FLASH_PAGE_85 ( ( uint32_t ) 0x080aa000 )  /* Base @ of Page 85, 8 Kbytes */
#define ADDR_FLASH_PAGE_86 ( ( uint32_t ) 0x080ac000 )  /* Base @ of Page 86, 8 Kbytes */
#define ADDR_FLASH_PAGE_87 ( ( uint32_t ) 0x080ae000 )  /* Base @ of Page 87, 8 Kbytes */
#define ADDR_FLASH_PAGE_88 ( ( uint32_t ) 0x080b0000 )  /* Base @ of Page 88, 8 Kbytes */
#define ADDR_FLASH_PAGE_89 ( ( uint32_t ) 0x080b2000 )  /* Base @ of Page 89, 8 Kbytes */
#define ADDR_FLASH_PAGE_90 ( ( uint32_t ) 0x080b4000 )  /* Base @ of Page 90, 8 Kbytes */
#define ADDR_FLASH_PAGE_91 ( ( uint32_t ) 0x080b6000 )  /* Base @ of Page 91, 8 Kbytes */
#define ADDR_FLASH_PAGE_92 ( ( uint32_t ) 0x080b8000 )  /* Base @ of Page 92, 8 Kbytes */
#define ADDR_FLASH_PAGE_93 ( ( uint32_t ) 0x080ba000 )  /* Base @ of Page 93, 8 Kbytes */
#define ADDR_FLASH_PAGE_94 ( ( uint32_t ) 0x080bc000 )  /* Base @ of Page 94, 8 Kbytes */
#define ADDR_FLASH_PAGE_95 ( ( uint32_t ) 0x080be000 )  /* Base @ of Page 95, 8 Kbytes */
#define ADDR_FLASH_PAGE_96 ( ( uint32_t ) 0x080c0000 )  /* Base @ of Page 96, 8 Kbytes */
#define ADDR_FLASH_PAGE_97 ( ( uint32_t ) 0x080c2000 )  /* Base @ of Page 97, 8 Kbytes */
#define ADDR_FLASH_PAGE_98 ( ( uint32_t ) 0x080c4000 )  /* Base @ of Page 98, 8 Kbytes */
#define ADDR_FLASH_PAGE_99 ( ( uint32_t ) 0x080c6000 )  /* Base @ of Page 99, 8 Kbytes */
#define ADDR_FLASH_PAGE_100 ( ( uint32_t ) 0x080c8000 ) /* Base @ of Page 100, 8 Kbytes */
#define ADDR_FLASH_PAGE_101 ( ( uint32_t ) 0x080ca000 ) /* Base @ of Page 101, 8 Kbytes */
#define ADDR_FLASH_PAGE_102 ( ( uint32_t ) 0x080cc000 ) /* Base @ of Page 102, 8 Kbytes */
#define ADDR_FLASH_PAGE_103 ( ( uint32_t ) 0x080ce000 ) /* Base @ of Page 103, 8 Kbytes */
#define ADDR_FLASH_PAGE_104 ( ( uint32_t ) 0x080d0000 ) /* Base @ of Page 104, 8 Kbytes */
#define ADDR_FLASH_PAGE_105 ( ( uint32_t ) 0x080d2000 ) /* Base @ of Page 105, 8 Kbytes */
#define ADDR_FLASH_PAGE_106 ( ( uint32_t ) 0x080d4000 ) /* Base @ of Page 106, 8 Kbytes */
#define ADDR_FLASH_PAGE_107 ( ( uint32_t ) 0x080d6000 ) /* Base @ of Page 107, 8 Kbytes */
#define ADDR_FLASH_PAGE_108 ( ( uint32_t ) 0x080d8000 ) /* Base @ of Page 108, 8 Kbytes */
#define ADDR_FLASH_PAGE_109 ( ( uint32_t ) 0x080da000 ) /* Base @ of Page 109, 8 Kbytes */
#define ADDR_FLASH_PAGE_110 ( ( uint32_t ) 0x080dc000 ) /* Base @ of Page 110, 8 Kbytes */
#define ADDR_FLASH_PAGE_111 ( ( uint32_t ) 0x080de000 ) /* Base @ of Page 111, 8 Kbytes */
#define ADDR_FLASH_PAGE_112 ( ( uint32_t ) 0x080e0000 ) /* Base @ of Page 112, 8 Kbytes */
#define ADDR_FLASH_PAGE_113 ( ( uint32_t ) 0x080e2000 ) /* Base @ of Page 113, 8 Kbytes */
#define ADDR_FLASH_PAGE_114 ( ( uint32_t ) 0x080e4000 ) /* Base @ of Page 114, 8 Kbytes */
#define ADDR_FLASH_PAGE_115 ( ( uint32_t ) 0x080e6000 ) /* Base @ of Page 115, 8 Kbytes */
#define ADDR_FLASH_PAGE_116 ( ( uint32_t ) 0x080e8000 ) /* Base @ of Page 116, 8 Kbytes */
#define ADDR_FLASH_PAGE_117 ( ( uint32_t ) 0x080ea000 ) /* Base @ of Page 117, 8 Kbytes */
#define ADDR_FLASH_PAGE_118 ( ( uint32_t ) 0x080ec000 ) /* Base @ of Page 118, 8 Kbytes */
#define ADDR_FLASH_PAGE_119 ( ( uint32_t ) 0x080ee000 ) /* Base @ of Page 119, 8 Kbytes */
#define ADDR_FLASH_PAGE_120 ( ( uint32_t ) 0x080f0000 ) /* Base @ of Page 120, 8 Kbytes */
#define ADDR_FLASH_PAGE_121 ( ( uint32_t ) 0x080f2000 ) /* Base @ of Page 121, 8 Kbytes */
#define ADDR_FLASH_PAGE_122 ( ( uint32_t ) 0x080f4000 ) /* Base @ of Page 122, 8 Kbytes */
#define ADDR_FLASH_PAGE_123 ( ( uint32_t ) 0x080f6000 ) /* Base @ of Page 123, 8 Kbytes */
#define ADDR_FLASH_PAGE_124 ( ( uint32_t ) 0x080f8000 ) /* Base @ of Page 124, 8 Kbytes */
#define ADDR_FLASH_PAGE_125 ( ( uint32_t ) 0x080fa000 ) /* Base @ of Page 125, 8 Kbytes */
#define ADDR_FLASH_PAGE_126 ( ( uint32_t ) 0x080fc000 ) /* Base @ of Page 126, 8 Kbytes */
#define ADDR_FLASH_PAGE_127 ( ( uint32_t ) 0x080fe000 ) /* Base @ of Page 127, 8 Kbytes */
#define ADDR_FLASH_PAGE_128 ( ( uint32_t ) 0x08100000 ) /* Base @ of Page 128, 8 Kbytes */
#define ADDR_FLASH_PAGE_129 ( ( uint32_t ) 0x08102000 ) /* Base @ of Page 129, 8 Kbytes */
#define ADDR_FLASH_PAGE_130 ( ( uint32_t ) 0x08104000 ) /* Base @ of Page 130, 8 Kbytes */
#define ADDR_FLASH_PAGE_131 ( ( uint32_t ) 0x08106000 ) /* Base @ of Page 131, 8 Kbytes */
#define ADDR_FLASH_PAGE_132 ( ( uint32_t ) 0x08108000 ) /* Base @ of Page 132, 8 Kbytes */
#define ADDR_FLASH_PAGE_133 ( ( uint32_t ) 0x0810a000 ) /* Base @ of Page 133, 8 Kbytes */
#define ADDR_FLASH_PAGE_134 ( ( uint32_t ) 0x0810c000 ) /* Base @ of Page 134, 8 Kbytes */
#define ADDR_FLASH_PAGE_135 ( ( uint32_t ) 0x0810e000 ) /* Base @ of Page 135, 8 Kbytes */
#define ADDR_FLASH_PAGE_136 ( ( uint32_t ) 0x08110000 ) /* Base @ of Page 136, 8 Kbytes */
#define ADDR_FLASH_PAGE_137 ( ( uint32_t ) 0x08112000 ) /* Base @ of Page 137, 8 Kbytes */
#define ADDR_FLASH_PAGE_138 ( ( uint32_t ) 0x08114000 ) /* Base @ of Page 138, 8 Kbytes */
#define ADDR_FLASH_PAGE_139 ( ( uint32_t ) 0x08116000 ) /* Base @ of Page 139, 8 Kbytes */
#define ADDR_FLASH_PAGE_140 ( ( uint32_t ) 0x08118000 ) /* Base @ of Page 140, 8 Kbytes */
#define ADDR_FLASH_PAGE_141 ( ( uint32_t ) 0x0811a000 ) /* Base @ of Page 141, 8 Kbytes */
#define ADDR_FLASH_PAGE_142 ( ( uint32_t ) 0x0811c000 ) /* Base @ of Page 142, 8 Kbytes */
#define ADDR_FLASH_PAGE_143 ( ( uint32_t ) 0x0811e000 ) /* Base @ of Page 143, 8 Kbytes */
#define ADDR_FLASH_PAGE_144 ( ( uint32_t ) 0x08120000 ) /* Base @ of Page 144, 8 Kbytes */
#define ADDR_FLASH_PAGE_145 ( ( uint32_t ) 0x08122000 ) /* Base @ of Page 145, 8 Kbytes */
#define ADDR_FLASH_PAGE_146 ( ( uint32_t ) 0x08124000 ) /* Base @ of Page 146, 8 Kbytes */
#define ADDR_FLASH_PAGE_147 ( ( uint32_t ) 0x08126000 ) /* Base @ of Page 147, 8 Kbytes */
#define ADDR_FLASH_PAGE_148 ( ( uint32_t ) 0x08128000 ) /* Base @ of Page 148, 8 Kbytes */
#define ADDR_FLASH_PAGE_149 ( ( uint32_t ) 0x0812a000 ) /* Base @ of Page 149, 8 Kbytes */
#define ADDR_FLASH_PAGE_150 ( ( uint32_t ) 0x0812c000 ) /* Base @ of Page 150, 8 Kbytes */
#define ADDR_FLASH_PAGE_151 ( ( uint32_t ) 0x0812e000 ) /* Base @ of Page 151, 8 Kbytes */
#define ADDR_FLASH_PAGE_152 ( ( uint32_t ) 0x08130000 ) /* Base @ of Page 152, 8 Kbytes */
#define ADDR_FLASH_PAGE_153 ( ( uint32_t ) 0x08132000 ) /* Base @ of Page 153, 8 Kbytes */
#define ADDR_FLASH_PAGE_154 ( ( uint32_t ) 0x08134000 ) /* Base @ of Page 154, 8 Kbytes */
#define ADDR_FLASH_PAGE_155 ( ( uint32_t ) 0x08136000 ) /* Base @ of Page 155, 8 Kbytes */
#define ADDR_FLASH_PAGE_156 ( ( uint32_t ) 0x08138000 ) /* Base @ of Page 156, 8 Kbytes */
#define ADDR_FLASH_PAGE_157 ( ( uint32_t ) 0x0813a000 ) /* Base @ of Page 157, 8 Kbytes */
#define ADDR_FLASH_PAGE_158 ( ( uint32_t ) 0x0813c000 ) /* Base @ of Page 158, 8 Kbytes */
#define ADDR_FLASH_PAGE_159 ( ( uint32_t ) 0x0813e000 ) /* Base @ of Page 159, 8 Kbytes */
#define ADDR_FLASH_PAGE_160 ( ( uint32_t ) 0x08140000 ) /* Base @ of Page 160, 8 Kbytes */
#define ADDR_FLASH_PAGE_161 ( ( uint32_t ) 0x08142000 ) /* Base @ of Page 161, 8 Kbytes */
#define ADDR_FLASH_PAGE_162 ( ( uint32_t ) 0x08144000 ) /* Base @ of Page 162, 8 Kbytes */
#define ADDR_FLASH_PAGE_163 ( ( uint32_t ) 0x08146000 ) /* Base @ of Page 163, 8 Kbytes */
#define ADDR_FLASH_PAGE_164 ( ( uint32_t ) 0x08148000 ) /* Base @ of Page 164, 8 Kbytes */
#define ADDR_FLASH_PAGE_165 ( ( uint32_t ) 0x0814a000 ) /* Base @ of Page 165, 8 Kbytes */
#define ADDR_FLASH_PAGE_166 ( ( uint32_t ) 0x0814c000 ) /* Base @ of Page 166, 8 Kbytes */
#define ADDR_FLASH_PAGE_167 ( ( uint32_t ) 0x0814e000 ) /* Base @ of Page 167, 8 Kbytes */
#define ADDR_FLASH_PAGE_168 ( ( uint32_t ) 0x08150000 ) /* Base @ of Page 168, 8 Kbytes */
#define ADDR_FLASH_PAGE_169 ( ( uint32_t ) 0x08152000 ) /* Base @ of Page 169, 8 Kbytes */
#define ADDR_FLASH_PAGE_170 ( ( uint32_t ) 0x08154000 ) /* Base @ of Page 170, 8 Kbytes */
#define ADDR_FLASH_PAGE_171 ( ( uint32_t ) 0x08156000 ) /* Base @ of Page 171, 8 Kbytes */
#define ADDR_FLASH_PAGE_172 ( ( uint32_t ) 0x08158000 ) /* Base @ of Page 172, 8 Kbytes */
#define ADDR_FLASH_PAGE_173 ( ( uint32_t ) 0x0815a000 ) /* Base @ of Page 173, 8 Kbytes */
#define ADDR_FLASH_PAGE_174 ( ( uint32_t ) 0x0815c000 ) /* Base @ of Page 174, 8 Kbytes */
#define ADDR_FLASH_PAGE_175 ( ( uint32_t ) 0x0815e000 ) /* Base @ of Page 175, 8 Kbytes */
#define ADDR_FLASH_PAGE_176 ( ( uint32_t ) 0x08160000 ) /* Base @ of Page 176, 8 Kbytes */
#define ADDR_FLASH_PAGE_177 ( ( uint32_t ) 0x08162000 ) /* Base @ of Page 177, 8 Kbytes */
#define ADDR_FLASH_PAGE_178 ( ( uint32_t ) 0x08164000 ) /* Base @ of Page 178, 8 Kbytes */
#define ADDR_FLASH_PAGE_179 ( ( uint32_t ) 0x08166000 ) /* Base @ of Page 179, 8 Kbytes */
#define ADDR_FLASH_PAGE_180 ( ( uint32_t ) 0x08168000 ) /* Base @ of Page 180, 8 Kbytes */
#define ADDR_FLASH_PAGE_181 ( ( uint32_t ) 0x0816a000 ) /* Base @ of Page 181, 8 Kbytes */
#define ADDR_FLASH_PAGE_182 ( ( uint32_t ) 0x0816c000 ) /* Base @ of Page 182, 8 Kbytes */
#define ADDR_FLASH_PAGE_183 ( ( uint32_t ) 0x0816e000 ) /* Base @ of Page 183, 8 Kbytes */
#define ADDR_FLASH_PAGE_184 ( ( uint32_t ) 0x08170000 ) /* Base @ of Page 184, 8 Kbytes */
#define ADDR_FLASH_PAGE_185 ( ( uint32_t ) 0x08172000 ) /* Base @ of Page 185, 8 Kbytes */
#define ADDR_FLASH_PAGE_186 ( ( uint32_t ) 0x08174000 ) /* Base @ of Page 186, 8 Kbytes */
#define ADDR_FLASH_PAGE_187 ( ( uint32_t ) 0x08176000 ) /* Base @ of Page 187, 8 Kbytes */
#define ADDR_FLASH_PAGE_188 ( ( uint32_t ) 0x08178000 ) /* Base @ of Page 188, 8 Kbytes */
#define ADDR_FLASH_PAGE_189 ( ( uint32_t ) 0x0817a000 ) /* Base @ of Page 189, 8 Kbytes */
#define ADDR_FLASH_PAGE_190 ( ( uint32_t ) 0x0817c000 ) /* Base @ of Page 190, 8 Kbytes */
#define ADDR_FLASH_PAGE_191 ( ( uint32_t ) 0x0817e000 ) /* Base @ of Page 191, 8 Kbytes */
#define ADDR_FLASH_PAGE_192 ( ( uint32_t ) 0x08180000 ) /* Base @ of Page 192, 8 Kbytes */
#define ADDR_FLASH_PAGE_193 ( ( uint32_t ) 0x08182000 ) /* Base @ of Page 193, 8 Kbytes */
#define ADDR_FLASH_PAGE_194 ( ( uint32_t ) 0x08184000 ) /* Base @ of Page 194, 8 Kbytes */
#define ADDR_FLASH_PAGE_195 ( ( uint32_t ) 0x08186000 ) /* Base @ of Page 195, 8 Kbytes */
#define ADDR_FLASH_PAGE_196 ( ( uint32_t ) 0x08188000 ) /* Base @ of Page 196, 8 Kbytes */
#define ADDR_FLASH_PAGE_197 ( ( uint32_t ) 0x0818a000 ) /* Base @ of Page 197, 8 Kbytes */
#define ADDR_FLASH_PAGE_198 ( ( uint32_t ) 0x0818c000 ) /* Base @ of Page 198, 8 Kbytes */
#define ADDR_FLASH_PAGE_199 ( ( uint32_t ) 0x0818e000 ) /* Base @ of Page 199, 8 Kbytes */
#define ADDR_FLASH_PAGE_200 ( ( uint32_t ) 0x08190000 ) /* Base @ of Page 200, 8 Kbytes */
#define ADDR_FLASH_PAGE_201 ( ( uint32_t ) 0x08192000 ) /* Base @ of Page 201, 8 Kbytes */
#define ADDR_FLASH_PAGE_202 ( ( uint32_t ) 0x08194000 ) /* Base @ of Page 202, 8 Kbytes */
#define ADDR_FLASH_PAGE_203 ( ( uint32_t ) 0x08196000 ) /* Base @ of Page 203, 8 Kbytes */
#define ADDR_FLASH_PAGE_204 ( ( uint32_t ) 0x08198000 ) /* Base @ of Page 204, 8 Kbytes */
#define ADDR_FLASH_PAGE_205 ( ( uint32_t ) 0x0819a000 ) /* Base @ of Page 205, 8 Kbytes */
#define ADDR_FLASH_PAGE_206 ( ( uint32_t ) 0x0819c000 ) /* Base @ of Page 206, 8 Kbytes */
#define ADDR_FLASH_PAGE_207 ( ( uint32_t ) 0x0819e000 ) /* Base @ of Page 207, 8 Kbytes */
#define ADDR_FLASH_PAGE_208 ( ( uint32_t ) 0x081a0000 ) /* Base @ of Page 208, 8 Kbytes */
#define ADDR_FLASH_PAGE_209 ( ( uint32_t ) 0x081a2000 ) /* Base @ of Page 209, 8 Kbytes */
#define ADDR_FLASH_PAGE_210 ( ( uint32_t ) 0x081a4000 ) /* Base @ of Page 210, 8 Kbytes */
#define ADDR_FLASH_PAGE_211 ( ( uint32_t ) 0x081a6000 ) /* Base @ of Page 211, 8 Kbytes */
#define ADDR_FLASH_PAGE_212 ( ( uint32_t ) 0x081a8000 ) /* Base @ of Page 212, 8 Kbytes */
#define ADDR_FLASH_PAGE_213 ( ( uint32_t ) 0x081aa000 ) /* Base @ of Page 213, 8 Kbytes */
#define ADDR_FLASH_PAGE_214 ( ( uint32_t ) 0x081ac000 ) /* Base @ of Page 214, 8 Kbytes */
#define ADDR_FLASH_PAGE_215 ( ( uint32_t ) 0x081ae000 ) /* Base @ of Page 215, 8 Kbytes */
#define ADDR_FLASH_PAGE_216 ( ( uint32_t ) 0x081b0000 ) /* Base @ of Page 216, 8 Kbytes */
#define ADDR_FLASH_PAGE_217 ( ( uint32_t ) 0x081b2000 ) /* Base @ of Page 217, 8 Kbytes */
#define ADDR_FLASH_PAGE_218 ( ( uint32_t ) 0x081b4000 ) /* Base @ of Page 218, 8 Kbytes */
#define ADDR_FLASH_PAGE_219 ( ( uint32_t ) 0x081b6000 ) /* Base @ of Page 219, 8 Kbytes */
#define ADDR_FLASH_PAGE_220 ( ( uint32_t ) 0x081b8000 ) /* Base @ of Page 220, 8 Kbytes */
#define ADDR_FLASH_PAGE_221 ( ( uint32_t ) 0x081ba000 ) /* Base @ of Page 221, 8 Kbytes */
#define ADDR_FLASH_PAGE_222 ( ( uint32_t ) 0x081bc000 ) /* Base @ of Page 222, 8 Kbytes */
#define ADDR_FLASH_PAGE_223 ( ( uint32_t ) 0x081be000 ) /* Base @ of Page 223, 8 Kbytes */
#define ADDR_FLASH_PAGE_224 ( ( uint32_t ) 0x081c0000 ) /* Base @ of Page 224, 8 Kbytes */
#define ADDR_FLASH_PAGE_225 ( ( uint32_t ) 0x081c2000 ) /* Base @ of Page 225, 8 Kbytes */
#define ADDR_FLASH_PAGE_226 ( ( uint32_t ) 0x081c4000 ) /* Base @ of Page 226, 8 Kbytes */
#define ADDR_FLASH_PAGE_227 ( ( uint32_t ) 0x081c6000 ) /* Base @ of Page 227, 8 Kbytes */
#define ADDR_FLASH_PAGE_228 ( ( uint32_t ) 0x081c8000 ) /* Base @ of Page 228, 8 Kbytes */
#define ADDR_FLASH_PAGE_229 ( ( uint32_t ) 0x081ca000 ) /* Base @ of Page 229, 8 Kbytes */
#define ADDR_FLASH_PAGE_230 ( ( uint32_t ) 0x081cc000 ) /* Base @ of Page 230, 8 Kbytes */
#define ADDR_FLASH_PAGE_231 ( ( uint32_t ) 0x081ce000 ) /* Base @ of Page 231, 8 Kbytes */
#define ADDR_FLASH_PAGE_232 ( ( uint32_t ) 0x081d0000 ) /* Base @ of Page 232, 8 Kbytes */
#define ADDR_FLASH_PAGE_233 ( ( uint32_t ) 0x081d2000 ) /* Base @ of Page 233, 8 Kbytes */
#define ADDR_FLASH_PAGE_234 ( ( uint32_t ) 0x081d4000 ) /* Base @ of Page 234, 8 Kbytes */
#define ADDR_FLASH_PAGE_235 ( ( uint32_t ) 0x081d6000 ) /* Base @ of Page 235, 8 Kbytes */
#define ADDR_FLASH_PAGE_236 ( ( uint32_t ) 0x081d8000 ) /* Base @ of Page 236, 8 Kbytes */
#define ADDR_FLASH_PAGE_237 ( ( uint32_t ) 0x081da000 ) /* Base @ of Page 237, 8 Kbytes */
#define ADDR_FLASH_PAGE_238 ( ( uint32_t ) 0x081dc000 ) /* Base @ of Page 238, 8 Kbytes */
#define ADDR_FLASH_PAGE_239 ( ( uint32_t ) 0x081de000 ) /* Base @ of Page 239, 8 Kbytes */
#define ADDR_FLASH_PAGE_240 ( ( uint32_t ) 0x081e0000 ) /* Base @ of Page 240, 8 Kbytes */
#define ADDR_FLASH_PAGE_241 ( ( uint32_t ) 0x081e2000 ) /* Base @ of Page 241, 8 Kbytes */
#define ADDR_FLASH_PAGE_242 ( ( uint32_t ) 0x081e4000 ) /* Base @ of Page 242, 8 Kbytes */
#define ADDR_FLASH_PAGE_243 ( ( uint32_t ) 0x081e6000 ) /* Base @ of Page 243, 8 Kbytes */
#define ADDR_FLASH_PAGE_244 ( ( uint32_t ) 0x081e8000 ) /* Base @ of Page 244, 8 Kbytes */
#define ADDR_FLASH_PAGE_245 ( ( uint32_t ) 0x081ea000 ) /* Base @ of Page 245, 8 Kbytes */
#define ADDR_FLASH_PAGE_246 ( ( uint32_t ) 0x081ec000 ) /* Base @ of Page 246, 8 Kbytes */
#define ADDR_FLASH_PAGE_247 ( ( uint32_t ) 0x081ee000 ) /* Base @ of Page 247, 8 Kbytes */
#define ADDR_FLASH_PAGE_248 ( ( uint32_t ) 0x081f0000 ) /* Base @ of Page 248, 8 Kbytes */
#define ADDR_FLASH_PAGE_249 ( ( uint32_t ) 0x081f2000 ) /* Base @ of Page 249, 8 Kbytes */
#define ADDR_FLASH_PAGE_250 ( ( uint32_t ) 0x081f4000 ) /* Base @ of Page 250, 8 Kbytes */
#define ADDR_FLASH_PAGE_251 ( ( uint32_t ) 0x081f6000 ) /* Base @ of Page 251, 8 Kbytes */
#define ADDR_FLASH_PAGE_252 ( ( uint32_t ) 0x081f8000 ) /* Base @ of Page 252, 8 Kbytes */
#define ADDR_FLASH_PAGE_253 ( ( uint32_t ) 0x081fa000 ) /* Base @ of Page 253, 8 Kbytes */
#define ADDR_FLASH_PAGE_254 ( ( uint32_t ) 0x081fc000 ) /* Base @ of Page 254, 8 Kbytes */
#define ADDR_FLASH_PAGE_255 ( ( uint32_t ) 0x081fe000 ) /* Base @ of Page 255, 8 Kbytes */

#define FLASH_OPERATION_MAX_RETRY 4

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
