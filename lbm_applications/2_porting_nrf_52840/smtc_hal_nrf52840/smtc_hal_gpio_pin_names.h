/*!
 * \file      smtc_hal_gpio_pin_names.h
 *
 * \brief     Defines NucleoL476 platform pin names
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

#ifndef __SMTC_HAL_GPIO_PIN_NAMES_H__
#define __SMTC_HAL_GPIO_PIN_NAMES_H__

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
typedef enum gpio_pin_names_e
{
    P0_0  = 0,   // <0=> 0 (P0.0)
    P0_1  = 1,   // <1=> 1 (P0.1)
    P0_2  = 2,   // <2=> 2 (P0.2)
    P0_3  = 3,   // <3=> 3 (P0.3)
    P0_4  = 4,   // <4=> 4 (P0.4)
    P0_5  = 5,   // <5=> 5 (P0.5)
    P0_6  = 6,   // <6=> 6 (P0.6)
    P0_7  = 7,   // <7=> 7 (P0.7)
    P0_8  = 8,   // <8=> 8 (P0.8)
    P0_9  = 9,   // <9=> 9 (P0.9)
    P0_10 = 10,  // <10=> 10 (P0.10)
    P0_11 = 11,  // <11=> 11 (P0.11)
    P0_12 = 12,  // <12=> 12 (P0.12)
    P0_13 = 13,  // <13=> 13 (P0.13)
    P0_14 = 14,  // <14=> 14 (P0.14)
    P0_15 = 15,  // <15=> 15 (P0.15)
    P0_16 = 16,  // <16=> 16 (P0.16)
    P0_17 = 17,  // <17=> 17 (P0.17)
    P0_18 = 18,  // <18=> 18 (P0.18)
    P0_19 = 19,  // <19=> 19 (P0.19)
    P0_20 = 20,  // <20=> 20 (P0.20)
    P0_21 = 21,  // <21=> 21 (P0.21)
    P0_22 = 22,  // <22=> 22 (P0.22)
    P0_23 = 23,  // <23=> 23 (P0.23)
    P0_24 = 24,  // <24=> 24 (P0.24)
    P0_25 = 25,  // <25=> 25 (P0.25)
    P0_26 = 26,  // <26=> 26 (P0.26)
    P0_27 = 27,  // <27=> 27 (P0.27)
    P0_28 = 28,  // <28=> 28 (P0.28)
    P0_29 = 29,  // <29=> 29 (P0.29)
    P0_30 = 30,  // <30=> 30 (P0.30)
    P0_31 = 31,  // <31=> 31 (P0.31)
    P1_0  = 32,  // <32=> 32 (P1.0)
    P1_1  = 33,  // <33=> 33 (P1.1)
    P1_2  = 34,  // <34=> 34 (P1.2)
    P1_3  = 35,  // <35=> 35 (P1.3)
    P1_4  = 36,  // <36=> 36 (P1.4)
    P1_5  = 37,  // <37=> 37 (P1.5)
    P1_6  = 38,  // <38=> 38 (P1.6)
    P1_7  = 39,  // <39=> 39 (P1.7)
    P1_8  = 40,  // <40=> 40 (P1.8)
    P1_9  = 41,  // <41=> 41 (P1.9)
    P1_10 = 42,  // <42=> 42 (P1.10)
    P1_11 = 43,  // <43=> 43 (P1.11)
    P1_12 = 44,  // <44=> 44 (P1.12)
    P1_13 = 45,  // <45=> 45 (P1.13)
    P1_14 = 46,  // <46=> 46 (P1.14)
    P1_15 = 47,  // <47=> 47 (P1.15)
    Px_NC = -1,  // Not connected
} hal_gpio_pin_names_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_GPIO_PIN_NAMES_H__

/* --- EOF ------------------------------------------------------------------ */
