/*!
 * \file      wifi_region_finder.h
 *
 * \brief     Helper module detect region from WiFi access point information (Country code, SSID, ...)
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

#ifndef WIFI_REGION_FINDER_H
#define WIFI_REGION_FINDER_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include <stdbool.h>

#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct
{
    int     score;             // Actual score
    uint8_t top_score_ratio;   // In percent: score * 100 / total , [0-100]
    int8_t  top_score_margin;  // In percent: (score - second_best) * 100 / score , [0–100] or -1 if not applicabl
} wrf_region_score_entry_t;

typedef struct
{
    wrf_region_score_entry_t eu868;
    wrf_region_score_entry_t us915;
    wrf_region_score_entry_t cn470;
    wrf_region_score_entry_t au915;
    wrf_region_score_entry_t kr920;
    wrf_region_score_entry_t in865;
    wrf_region_score_entry_t ru864;
    wrf_region_score_entry_t as923;
    /* TODO: to be completed */
} wrf_region_score_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize internal structures of the region finder module.
 *
 * Resets the list of seen access points to ensure that each AP is counted only once
 * during a sequence of scans.
 */
void wrf_init( void );

/**
 * @brief Process a Wi-Fi access point scan result and update the regional scores.
 *
 * This function ensures each AP (based on MAC) contributes to scoring only once per
 * information type (SSID and Country Code). It updates the score accordingly.
 *
 * @param[in]  mac    6-byte MAC address of the access point
 * @param[in]  ssid   Null-terminated SSID string (may be empty)
 * @param[in]  cc     2-character country code string (may be empty)
 * @param[out] score  Pointer to the region score structure to be updated
 */
void wrf_process_ap( const uint8_t* mac, const char* ssid, const char* cc, wrf_region_score_t* score );

/**
 * @brief Print current region scores to the debug trace output.
 *
 * Useful for diagnostics and understanding which region is gaining confidence.
 *
 * @param[in] score Pointer to the score structure to be printed
 */
void wrf_print_region_scores( const wrf_region_score_t* score );

/**
 * @brief Determine the most likely region from scores, only if score >= threshold.
 *
 * @param[in]  score     Pointer to region score structure.
 * @param[in]  threshold Minimum score required to consider a region valid (must be > 0).
 * @param[out] region    Pointer to the selected region if successful.
 * @param[out] confidence  Pointer to an 8-bit integer that receives the confidence value (0–100).
 *                         This is the average of top_score_ratio and top_score_margin for the selected region.
 *                         Value is only valid if function returns 0.
 * @return 0 on success, -1 if ambiguous, below threshold, or invalid input.
 */
int wrf_get_highest_score_region( const wrf_region_score_t* score, int threshold, smtc_modem_region_t* region,
                                  uint8_t* confidence );

#endif  // WIFI_REGION_FINDER_H
