/*!
 * @file      lr20xx_radio_fsk_common_types.h
 *
 * @brief     Radio FSK common driver types for LR20XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

#ifndef LR20XX_RADIO_FSK_COMMON_TYPES_H
#define LR20XX_RADIO_FSK_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Reception Double Side Bandwidth (DSB) value for FSK modulation
 */
typedef enum
{
    LR20XX_RADIO_FSK_COMMON_RX_BW_3_500_HZ     = 0xE7,  //!< RX Bandwidth 3.5 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_4_200_HZ     = 0xA7,  //!< RX Bandwidth 4.2 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_4_300_HZ     = 0xDF,  //!< RX Bandwidth 4.3 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_4_500_HZ     = 0x67,  //!< RX Bandwidth 4.5 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_4_800_HZ     = 0x27,  //!< RX Bandwidth 4.8 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_5_200_HZ     = 0x9F,  //!< RX Bandwidth 5.2 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_5_600_HZ     = 0x5F,  //!< RX Bandwidth 5.6 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_5_800_HZ     = 0xD7,  //!< RX Bandwidth 5.8 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_6_000_HZ     = 0x1F,  //!< RX Bandwidth 6 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_6_900_HZ     = 0xE6,  //!< RX Bandwidth 6.9 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_7_400_HZ     = 0x57,  //!< RX Bandwidth 7.4 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_8_000_HZ     = 0x17,  //!< RX Bandwidth 8 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_8_300_HZ     = 0xA6,  //!< RX Bandwidth 8.3 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_8_700_HZ     = 0xDE,  //!< RX Bandwidth 8.7 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_8_900_HZ     = 0x66,  //!< RX Bandwidth 8.9 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_9_600_HZ     = 0x26,  //!< RX Bandwidth 9.6 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_10_000_HZ    = 0x9E,  //!< RX Bandwidth 10 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_11_000_HZ    = 0x5E,  //!< RX Bandwidth 11 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_12_000_HZ    = 0x1E,  //!< RX Bandwidth 12 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_13_000_HZ    = 0xE5,  //!< RX Bandwidth 13 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_14_000_HZ    = 0x56,  //!< RX Bandwidth 14 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_16_000_HZ    = 0xA5,  //!< RX Bandwidth 16 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_17_000_HZ    = 0x65,  //!< RX Bandwidth 17 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_19_000_HZ    = 0x25,  //!< RX Bandwidth 19 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_20_000_HZ    = 0x9D,  //!< RX Bandwidth 20 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_22_000_HZ    = 0x5D,  //!< RX Bandwidth 22 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_23_000_HZ    = 0xD5,  //!< RX Bandwidth 23 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_24_000_HZ    = 0x1D,  //!< RX Bandwidth 24 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_27_000_HZ    = 0xE4,  //!< RX Bandwidth 27 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_29_000_HZ    = 0x55,  //!< RX Bandwidth 29 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_32_000_HZ    = 0x15,  //!< RX Bandwidth 32 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_33_000_HZ    = 0xA4,  //!< RX Bandwidth 33 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_34_000_HZ    = 0xDC,  //!< RX Bandwidth 34 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_35_000_HZ    = 0x64,  //!< RX Bandwidth 35 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_38_000_HZ    = 0x24,  //!< RX Bandwidth 38 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_41_000_HZ    = 0x9C,  //!< RX Bandwidth 41 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_44_000_HZ    = 0x5C,  //!< RX Bandwidth 44 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_46_000_HZ    = 0xD4,  //!< RX Bandwidth 46 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_48_000_HZ    = 0x1C,  //!< RX Bandwidth 48 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_55_000_HZ    = 0xE3,  //!< RX Bandwidth 55 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_59_000_HZ    = 0x54,  //!< RX Bandwidth 59 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_64_000_HZ    = 0x14,  //!< RX Bandwidth 64 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_66_000_HZ    = 0xA3,  //!< RX Bandwidth 66 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_69_000_HZ    = 0xDB,  //!< RX Bandwidth 69 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_71_000_HZ    = 0x63,  //!< RX Bandwidth 71 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_76_000_HZ    = 0x23,  //!< RX Bandwidth 76 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_83_000_HZ    = 0x9B,  //!< RX Bandwidth 83 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_89_000_HZ    = 0x5B,  //!< RX Bandwidth 89 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_92_000_HZ    = 0xD3,  //!< RX Bandwidth 92 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_96_000_HZ    = 0x1B,  //!< RX Bandwidth 96 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_111_000_HZ   = 0xE2,  //!< RX Bandwidth 111 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_119_000_HZ   = 0x53,  //!< RX Bandwidth 119 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_128_000_HZ   = 0x13,  //!< RX Bandwidth 128 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_133_000_HZ   = 0xA2,  //!< RX Bandwidth 133 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_138_000_HZ   = 0xDA,  //!< RX Bandwidth 138 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_142_000_HZ   = 0x62,  //!< RX Bandwidth 142 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_153_000_HZ   = 0x22,  //!< RX Bandwidth 153 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_166_000_HZ   = 0x9A,  //!< RX Bandwidth 166 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_178_000_HZ   = 0x5A,  //!< RX Bandwidth 178 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_185_000_HZ   = 0xD2,  //!< RX Bandwidth 185 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_192_000_HZ   = 0x1A,  //!< RX Bandwidth 192 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_222_000_HZ   = 0xE1,  //!< RX Bandwidth 222 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_238_000_HZ   = 0x52,  //!< RX Bandwidth 238 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_256_000_HZ   = 0x12,  //!< RX Bandwidth 256 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_266_000_HZ   = 0xA1,  //!< RX Bandwidth 266 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_277_000_HZ   = 0xD9,  //!< RX Bandwidth 277 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_285_000_HZ   = 0x61,  //!< RX Bandwidth 285 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_307_000_HZ   = 0x21,  //!< RX Bandwidth 307 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_333_000_HZ   = 0x99,  //!< RX Bandwidth 333 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_357_000_HZ   = 0x59,  //!< RX Bandwidth 357 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_370_000_HZ   = 0xD1,  //!< RX Bandwidth 370 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_384_000_HZ   = 0x19,  //!< RX Bandwidth 384 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_444_000_HZ   = 0xE0,  //!< RX Bandwidth 444 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_476_000_HZ   = 0x51,  //!< RX Bandwidth 476 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_512_000_HZ   = 0x11,  //!< RX Bandwidth 512 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_533_000_HZ   = 0xA0,  //!< RX Bandwidth 533 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_555_000_HZ   = 0xD8,  //!< RX Bandwidth 555 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_571_000_HZ   = 0x60,  //!< RX Bandwidth 571 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_615_000_HZ   = 0x20,  //!< RX Bandwidth 615 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_666_000_HZ   = 0x98,  //!< RX Bandwidth 666 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_714_000_HZ   = 0x58,  //!< RX Bandwidth 714 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_740_000_HZ   = 0xD0,  //!< RX Bandwidth 740 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_769_000_HZ   = 0x18,  //!< RX Bandwidth 769 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_888_000_HZ   = 0x90,  //!< RX Bandwidth 888 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_1_111_000_HZ = 0xC8,  //!< RX Bandwidth 1111 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_1_333_000_HZ = 0x88,  //!< RX Bandwidth 1333 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_2_222_000_HZ = 0xC0,  //!< RX Bandwidth 2222 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_2_666_000_HZ = 0x80,  //!< RX Bandwidth 2666 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_2_857_000_HZ = 0x40,  //!< RX Bandwidth 2857 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_3_076_000_HZ = 0x00,  //!< RX Bandwidth 3076 kHz
    LR20XX_RADIO_FSK_COMMON_RX_BW_AUTO =
        0xFF,  //!< RX Bandwidth automatic choice - limited to Wi-SUN, Wireless M-BUS, Wi-SUN, and Z-Wave
} lr20xx_radio_fsk_common_bw_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR20XX_RADIO_FSK_COMMON_TYPES_H

/* --- EOF ------------------------------------------------------------------ */
