/*!
 * \file      wifi_region_finder.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "wifi_region_finder.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#ifdef WIFI_RF_UNIT_TEST
#define WIFI_RF_PRINTF( ... ) printf( __VA_ARGS__ )
#define WIFI_RF_TRACE_ERROR( ... ) fprintf( stderr, __VA_ARGS__ )
#else
#include "smtc_hal_dbg_trace.h"
#define WIFI_RF_PRINTF( ... ) SMTC_HAL_TRACE_PRINTF( __VA_ARGS__ )
#define WIFI_RF_TRACE_ERROR( ... ) SMTC_HAL_TRACE_ERROR( __VA_ARGS__ )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
const char* eu868_cc[] = {
    "FR",  // France
    "DE",  // Germany
    "IT",  // Italy
    "ES",  // Spain
    "PT",  // Portugal
    "NL",  // Netherlands
    "BE",  // Belgium
    "LU",  // Luxembourg
    "IE",  // Ireland
    "GB",  // United Kingdom
    "SE",  // Sweden
    "FI",  // Finland
    "NO",  // Norway
    "DK",  // Denmark
    "PL",  // Poland
    "CZ",  // Czech Republic
    "SK",  // Slovakia
    "AT",  // Austria
    "HU",  // Hungary
    "SI",  // Slovenia
    "HR",  // Croatia
    "GR",  // Greece
    "CH",  // Switzerland
    "EE",  // Estonia
    "LV",  // Latvia
    "LT",  // Lithuania
    "IS"   // Iceland
};

const char* us915_cc[] = {
    "US",  // United States
    "CA",  // Canada
    "MX"   // Mexico
};

const char* cn470_cc[] = {
    "CN",  // China
};

const char* au915_cc[] = {
    "AU",  // Australia
    "NZ",  // New Zealand
    "BR",  // Brazil
};

const char* kr920_cc[] = {
    "KR"  // South Korea
};

const char* in865_cc[] = {
    "IN"  // India
};

const char* ru864_cc[] = {
    "RU"  // Russia
};

const char* as923_cc[] = {
    "JP",  // Japan
    "SG",  // Singapore
    "MY",  // Malaysia
    "TW",  // Taiwan
    "HK",  // Hong Kong
};

const char* eu868_ssid_patterns[] = { "livebox", "freebox", "freewifi", "sfr", "bbox" };

const char* us915_ssid_patterns[] = { "xfinity", "att", "spectrum", "fios", "verizon", "bell", "rogers", "tenus" };

const char* cn470_ssid_patterns[] = { "chinanet", "huawei", "zte" };

const char* au915_ssid_patterns[] = { "telstra", "optus" };

const char* kr920_ssid_patterns[] = { "skt", "uplus" };

const char* in865_ssid_patterns[] = { "jio", "airtel" };

const char* ru864_ssid_patterns[] = { "rostelecom" };

const char* as923_ssid_patterns[] = { "ntt", "softbank" };

#define MAX_UNIQUE_APS 128
#define MAC_ADDR_LEN 6

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    uint8_t mac[MAC_ADDR_LEN];
    bool    ssid_scored;
    bool    cc_scored;
} wrf_ap_tracking_entry_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static wrf_ap_tracking_entry_t seen_aps[MAX_UNIQUE_APS];
static int                     seen_ap_count = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void                     to_lowercase( const char* src, char* dst, size_t len );
static wrf_ap_tracking_entry_t* get_or_add_ap_entry( const uint8_t* mac );

static bool is_eu868_country( const char* cc );
static bool is_us915_country( const char* cc );
static bool is_cn470_country( const char* cc );

static void score_country_code( const char* cc, wrf_region_score_t* score );
static void score_ssid( const char* ssid, wrf_region_score_t* score );

/*
 * -----------------------------------------------------------------------------
 * --- NON STATIC PRIVATE FUNCTIONS DECLARATION (for unit testing usage ) ------
 */
void wrf_update_score_metrics( wrf_region_score_t* score );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void wrf_init( void )
{
    seen_ap_count = 0;
    memset( seen_aps, 0, sizeof( seen_aps ) );
}

void wrf_process_ap( const uint8_t* mac, const char* ssid, const char* cc, wrf_region_score_t* score )
{
    wrf_ap_tracking_entry_t* ap = get_or_add_ap_entry( mac );
    if( ap == NULL )
        return;

    if( ssid && ssid[0] != '\0' && !ap->ssid_scored )
    {
        score_ssid( ssid, score );
        ap->ssid_scored = true;
    }
    if( cc && cc[0] != '\0' && !ap->cc_scored )
    {
        score_country_code( cc, score );
        ap->cc_scored = true;
    }

    wrf_update_score_metrics( score );
}

void wrf_print_region_scores( const wrf_region_score_t* score )
{
    WIFI_RF_PRINTF( "\n=== Region Scores (value, ratio%%, margin%%) ===\n" );

#define PRINT_ENTRY( label, entry )                                                                                   \
    if( entry.top_score_margin >= 0 )                                                                                 \
    {                                                                                                                 \
        WIFI_RF_PRINTF( label " : %2d, %3d%%, %3d%%\n", entry.score, entry.top_score_ratio, entry.top_score_margin ); \
    }                                                                                                                 \
    else                                                                                                              \
    {                                                                                                                 \
        WIFI_RF_PRINTF( label " : %2d, %3d%%,  N/A\n", entry.score, entry.top_score_ratio );                          \
    }

    PRINT_ENTRY( "EU868", score->eu868 );
    PRINT_ENTRY( "US915", score->us915 );
    PRINT_ENTRY( "CN470", score->cn470 );
    PRINT_ENTRY( "AU915", score->au915 );
    PRINT_ENTRY( "KR920", score->kr920 );
    PRINT_ENTRY( "IN865", score->in865 );
    PRINT_ENTRY( "RU864", score->ru864 );
    PRINT_ENTRY( "AS923", score->as923 );

#undef PRINT_ENTRY

    WIFI_RF_PRINTF( "===========================================\n" );
}

int wrf_get_highest_score_region( const wrf_region_score_t* score, int threshold, smtc_modem_region_t* region,
                                  uint8_t* confidence )
{
    if( score == NULL || region == NULL || confidence == NULL || threshold <= 0 )
    {
        return -1;
    }

    int                 max_score = 0;
    smtc_modem_region_t selected  = 0;
    uint8_t             ratio     = 0;
    int8_t              margin    = -1;

    if( score->eu868.score > max_score )
    {
        max_score = score->eu868.score;
        ratio     = score->eu868.top_score_ratio;
        margin    = score->eu868.top_score_margin;
        selected  = SMTC_MODEM_REGION_EU_868;
    }
    if( score->us915.score > max_score )
    {
        max_score = score->us915.score;
        ratio     = score->us915.top_score_ratio;
        margin    = score->us915.top_score_margin;
        selected  = SMTC_MODEM_REGION_US_915;
    }
    if( score->cn470.score > max_score )
    {
        max_score = score->cn470.score;
        ratio     = score->cn470.top_score_ratio;
        margin    = score->cn470.top_score_margin;
        selected  = SMTC_MODEM_REGION_CN_470;
    }
    if( score->au915.score > max_score )
    {
        max_score = score->au915.score;
        ratio     = score->au915.top_score_ratio;
        margin    = score->au915.top_score_margin;
        selected  = SMTC_MODEM_REGION_AU_915;
    }
    if( score->kr920.score > max_score )
    {
        max_score = score->kr920.score;
        ratio     = score->kr920.top_score_ratio;
        margin    = score->kr920.top_score_margin;
        selected  = SMTC_MODEM_REGION_KR_920;
    }
    if( score->in865.score > max_score )
    {
        max_score = score->in865.score;
        ratio     = score->in865.top_score_ratio;
        margin    = score->in865.top_score_margin;
        selected  = SMTC_MODEM_REGION_IN_865;
    }
    if( score->ru864.score > max_score )
    {
        max_score = score->ru864.score;
        ratio     = score->ru864.top_score_ratio;
        margin    = score->ru864.top_score_margin;
        selected  = SMTC_MODEM_REGION_RU_864;
    }
    if( score->as923.score > max_score )
    {
        max_score = score->as923.score;
        ratio     = score->as923.top_score_ratio;
        margin    = score->as923.top_score_margin;
        selected  = SMTC_MODEM_REGION_AS_923_GRP1; /* TODO: handle all groups */
    }

    if( ( max_score >= threshold ) && ( selected != 0 ) )
    {
        // Check for ambiguity: are there multiple regions with max_score?
        int num_max_regions = 0;

        if( score->eu868.score == max_score )
            num_max_regions++;
        if( score->us915.score == max_score )
            num_max_regions++;
        if( score->cn470.score == max_score )
            num_max_regions++;
        if( score->au915.score == max_score )
            num_max_regions++;
        if( score->kr920.score == max_score )
            num_max_regions++;
        if( score->in865.score == max_score )
            num_max_regions++;
        if( score->ru864.score == max_score )
            num_max_regions++;
        if( score->as923.score == max_score )
            num_max_regions++;

        if( num_max_regions == 1 )
        {
            *region     = selected;
            *confidence = ( margin >= 0 ) ? ( ratio + margin ) / 2 : ratio;
            return 0;  // Success
        }
    }

    return -1;  // Ambiguous or below threshold
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void to_lowercase( const char* src, char* dst, size_t len )
{
    for( size_t i = 0; i < len - 1 && src[i] != '\0'; i++ )
    {
        dst[i] = tolower( ( unsigned char ) src[i] );
    }
    dst[len - 1] = '\0';
}

static wrf_ap_tracking_entry_t* get_or_add_ap_entry( const uint8_t* mac )
{
    for( int i = 0; i < seen_ap_count; i++ )
    {
        if( memcmp( seen_aps[i].mac, mac, MAC_ADDR_LEN ) == 0 )
        {
            return &seen_aps[i];
        }
    }
    if( seen_ap_count < MAX_UNIQUE_APS )
    {
        memcpy( seen_aps[seen_ap_count].mac, mac, MAC_ADDR_LEN );
        seen_aps[seen_ap_count].ssid_scored = false;
        seen_aps[seen_ap_count].cc_scored   = false;
        return &seen_aps[seen_ap_count++];
    }
    WIFI_RF_TRACE_ERROR( "ERROR: AP list is full\n" );
    return NULL;
}

static bool is_country_in_list( const char* cc, const char** list, size_t size )
{
    for( size_t i = 0; i < size; ++i )
    {
        if( strncmp( cc, list[i], 2 ) == 0 )
        {
            return true;
        }
    }
    return false;
}

bool is_eu868_country( const char* cc )
{
    return is_country_in_list( cc, eu868_cc, sizeof( eu868_cc ) / sizeof( eu868_cc[0] ) );
}

bool is_us915_country( const char* cc )
{
    return is_country_in_list( cc, us915_cc, sizeof( us915_cc ) / sizeof( us915_cc[0] ) );
}

bool is_cn470_country( const char* cc )
{
    return is_country_in_list( cc, cn470_cc, sizeof( cn470_cc ) / sizeof( cn470_cc[0] ) );
}

bool is_au915_country( const char* cc )
{
    return is_country_in_list( cc, au915_cc, sizeof( au915_cc ) / sizeof( au915_cc[0] ) );
}

bool is_kr920_country( const char* cc )
{
    return is_country_in_list( cc, kr920_cc, sizeof( kr920_cc ) / sizeof( kr920_cc[0] ) );
}

bool is_in865_country( const char* cc )
{
    return is_country_in_list( cc, in865_cc, sizeof( in865_cc ) / sizeof( in865_cc[0] ) );
}

bool is_ru864_country( const char* cc )
{
    return is_country_in_list( cc, ru864_cc, sizeof( ru864_cc ) / sizeof( ru864_cc[0] ) );
}

bool is_as923_country( const char* cc )
{
    return is_country_in_list( cc, as923_cc, sizeof( as923_cc ) / sizeof( as923_cc[0] ) );
}

static void score_country_code( const char* cc, wrf_region_score_t* score )
{
    if( is_eu868_country( cc ) )
    {
        score->eu868.score += 1;
    }
    else if( is_us915_country( cc ) )
    {
        score->us915.score += 1;
    }
    else if( is_cn470_country( cc ) )
    {
        score->cn470.score += 1;
    }
    else if( is_au915_country( cc ) )
    {
        score->au915.score += 1;
    }
    else if( is_kr920_country( cc ) )
    {
        score->kr920.score += 1;
    }
    else if( is_in865_country( cc ) )
    {
        score->in865.score += 1;
    }
    else if( is_ru864_country( cc ) )
    {
        score->ru864.score += 1;
    }
    else if( is_as923_country( cc ) )
    {
        score->as923.score += 1;
    }
    else
    {
        WIFI_RF_TRACE_ERROR( "ERROR: Unknown country code: %s\n", cc );
    }
}

static bool ssid_matches_any( const char* ssid_lower, const char** patterns, size_t pattern_count )
{
    for( size_t i = 0; i < pattern_count; ++i )
    {
        if( strstr( ssid_lower, patterns[i] ) )
        {
            return true;
        }
    }
    return false;
}

bool is_match_with_eu868_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, eu868_ssid_patterns,
                             sizeof( eu868_ssid_patterns ) / sizeof( eu868_ssid_patterns[0] ) );
}

bool is_match_with_us915_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, us915_ssid_patterns,
                             sizeof( us915_ssid_patterns ) / sizeof( us915_ssid_patterns[0] ) );
}

bool is_match_with_cn470_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, cn470_ssid_patterns,
                             sizeof( cn470_ssid_patterns ) / sizeof( cn470_ssid_patterns[0] ) );
}

bool is_match_with_au915_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, au915_ssid_patterns,
                             sizeof( au915_ssid_patterns ) / sizeof( au915_ssid_patterns[0] ) );
}

bool is_match_with_kr920_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, kr920_ssid_patterns,
                             sizeof( kr920_ssid_patterns ) / sizeof( kr920_ssid_patterns[0] ) );
}

bool is_match_with_in865_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, in865_ssid_patterns,
                             sizeof( in865_ssid_patterns ) / sizeof( in865_ssid_patterns[0] ) );
}

bool is_match_with_ru864_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, ru864_ssid_patterns,
                             sizeof( ru864_ssid_patterns ) / sizeof( ru864_ssid_patterns[0] ) );
}

bool is_match_with_as923_ssid( const char* ssid_lower )
{
    return ssid_matches_any( ssid_lower, as923_ssid_patterns,
                             sizeof( as923_ssid_patterns ) / sizeof( as923_ssid_patterns[0] ) );
}

static void score_ssid( const char* ssid, wrf_region_score_t* score )
{
    char ssid_lower[33];
    to_lowercase( ssid, ssid_lower, sizeof( ssid_lower ) );

    if( is_match_with_eu868_ssid( ssid_lower ) )
    {
        score->eu868.score += 1;
    }
    else if( is_match_with_us915_ssid( ssid_lower ) )
    {
        score->us915.score += 1;
    }
    else if( is_match_with_cn470_ssid( ssid_lower ) )
    {
        score->cn470.score += 1;
    }
    else if( is_match_with_au915_ssid( ssid_lower ) )
    {
        score->au915.score += 1;
    }
    else if( is_match_with_kr920_ssid( ssid_lower ) )
    {
        score->kr920.score += 1;
    }
    else if( is_match_with_in865_ssid( ssid_lower ) )
    {
        score->in865.score += 1;
    }
    else if( is_match_with_ru864_ssid( ssid_lower ) )
    {
        score->ru864.score += 1;
    }
    else if( is_match_with_as923_ssid( ssid_lower ) )
    {
        score->as923.score += 1;
    }
}

/**
 * @brief Update the score table with confidence metrics (ratio and margin).
 *
 * This function analyzes the region scores and computes:
 *  - top_score_ratio: the proportion of each score relative to the total (0–100%)
 *  - top_score_margin: how much the top region leads over the second best (in %), or -1 if not the top score
 *
 * It should be called after all APs have been processed to keep the metrics consistent.
 *
 * @param[in,out] score  Pointer to the region score structure to update.
 */
void wrf_update_score_metrics( wrf_region_score_t* score )
{
    // Sum all region scores to get the total number of matched APs
    int total = score->eu868.score + score->us915.score + score->cn470.score + score->au915.score + score->kr920.score +
                score->in865.score + score->ru864.score + score->as923.score;

    // If no scoring data exists, there's nothing to calculate — exit early
    if( total == 0 )
    {
        return;
    }

    int max    = 0;  // highest score
    int second = 0;  // second-highest score

    // Array of all scores for easy iteration
    const int scores[] = { score->eu868.score, score->us915.score, score->cn470.score, score->au915.score,
                           score->kr920.score, score->in865.score, score->ru864.score, score->as923.score };

    // Determine the highest and second-highest scores
    for( int i = 0; i < 8; i++ )
    {
        if( scores[i] > max )
        {
            second = max;
            max    = scores[i];
        }
        else if( scores[i] > second )
        {
            second = scores[i];
        }
    }

    // Macro to calculate ratio and margin for each region
    // - ratio: percentage of the total (rounded)
    // - margin: only computed for the top-scoring region, others get -1
#define CALC_METRICS( SCORE_ENTRY )                                                                \
    SCORE_ENTRY.top_score_ratio = ( uint8_t ) ( ( SCORE_ENTRY.score * 100 + total / 2 ) / total ); \
    if( SCORE_ENTRY.score == max && max > 0 )                                                      \
    {                                                                                              \
        SCORE_ENTRY.top_score_margin = ( int8_t ) ( ( ( max - second ) * 100 + max / 2 ) / max );  \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
        SCORE_ENTRY.top_score_margin = -1;                                                         \
    }

    // Apply the macro to all region entries
    CALC_METRICS( score->eu868 );
    CALC_METRICS( score->us915 );
    CALC_METRICS( score->cn470 );
    CALC_METRICS( score->au915 );
    CALC_METRICS( score->kr920 );
    CALC_METRICS( score->in865 );
    CALC_METRICS( score->ru864 );
    CALC_METRICS( score->as923 );

#undef CALC_METRICS
}
