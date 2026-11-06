#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "wifi_region_finder.h"

extern void wrf_update_score_metrics( wrf_region_score_t* score );

static int test_counter = 1;
static int passed_count = 0;
static int failed_count = 0;

void print_result( const char* test_name, int passed )
{
    printf( "[%02d] %s : %s\n", test_counter++, test_name, passed ? "PASSED" : "** FAILED **" );
    if( passed )
        passed_count++;
    else
        failed_count++;
}

bool wrf_scores_all_zero( const wrf_region_score_t* score )
{
    return ( score->eu868.score == 0 && score->us915.score == 0 && score->cn470.score == 0 && score->au915.score == 0 &&
             score->kr920.score == 0 && score->in865.score == 0 && score->ru864.score == 0 && score->as923.score == 0 );
}

// --- TEST FUNCTIONS ----------------------------------------------------------

/**
 * Verifies that wrf_init() resets the internal state.
 */
void test_init_clears_state( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 };
    wrf_process_ap( mac, "Livebox_1234", "FR", &score );

    wrf_region_score_t new_score = { 0 };
    wrf_init( );
    wrf_process_ap( mac, "", "", &new_score );

    print_result( "test_init_clears_state", new_score.eu868.score == 0 );
}

/**
 * Verifies that an AP with both a known SSID and country code scores 2 points.
 */
void test_cc_and_ssid_scoring( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
    wrf_process_ap( mac, "Livebox", "FR", &score );
    print_result( "test_cc_and_ssid_scoring", score.eu868.score == 2 );
}

/**
 * Verifies that the same AP cannot contribute to the score more than once.
 */
void test_no_double_scoring( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
    wrf_process_ap( mac, "Freebox", "FR", &score );
    wrf_process_ap( mac, "Freebox", "FR", &score );
    print_result( "test_no_double_scoring", score.eu868.score == 2 );
}

/**
 * Verifies that only country code scoring works independently.
 */
void test_only_cc( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x07 };
    wrf_process_ap( mac, "", "FR", &score );
    print_result( "test_only_cc", score.eu868.score == 1 );
}

/**
 * Verifies that only SSID scoring works independently.
 */
void test_only_ssid( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x08 };
    wrf_process_ap( mac, "bbox", "", &score );
    print_result( "test_only_ssid", score.eu868.score == 1 );
}

/**
 * Verifies that unknown SSID and country code do not affect any score.
 */
void test_unknown_cc_and_ssid( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0x10, 0x20, 0x30, 0x40, 0x50, 0x60 };
    wrf_process_ap( mac, "some_wifi", "ZZ", &score );
    print_result( "test_unknown_cc_and_ssid", wrf_scores_all_zero( &score ) );
}

/**
 * Verifies that the most likely region is returned if above threshold.
 */
void test_region_detection_success( )
{
    wrf_init( );
    wrf_region_score_t score = { 0 };
    score.eu868.score        = 3;
    score.us915.score        = 4;
    wrf_update_score_metrics( &score );

    smtc_modem_region_t region;
    uint8_t             confidence = 0;
    int                 rc         = wrf_get_highest_score_region( &score, 3, &region, &confidence );

    int expected_confidence = ( score.us915.top_score_ratio + score.us915.top_score_margin ) / 2;

    print_result( "test_region_detection_success",
                  rc == 0 && region == SMTC_MODEM_REGION_US_915 && confidence == expected_confidence );
}

/**
 * Verifies that ambiguity (tie) leads to detection failure.
 */
void test_region_detection_ambiguous( )
{
    wrf_init( );
    wrf_region_score_t score = { 0 };
    score.eu868.score        = 3;
    score.us915.score        = 3;
    wrf_update_score_metrics( &score );

    smtc_modem_region_t region;
    uint8_t             confidence = 0;
    int                 rc         = wrf_get_highest_score_region( &score, 2, &region, &confidence );
    print_result( "test_region_detection_ambiguous", rc == -1 );
}

/**
 * Verifies that score below threshold results in no detection.
 */
void test_region_detection_below_threshold( )
{
    wrf_init( );
    wrf_region_score_t score = { 0 };
    score.eu868.score        = 1;
    wrf_update_score_metrics( &score );

    smtc_modem_region_t region;
    uint8_t             confidence = 0;
    int                 rc         = wrf_get_highest_score_region( &score, 2, &region, &confidence );
    print_result( "test_region_detection_below_threshold", rc == -1 );
}

/**
 * Verifies that all-zero score results in no region detection.
 */
void test_region_detection_failure( )
{
    wrf_init( );
    wrf_region_score_t score = { 0 };
    wrf_update_score_metrics( &score );

    smtc_modem_region_t region;
    uint8_t             confidence = 0;
    int                 rc         = wrf_get_highest_score_region( &score, 1, &region, &confidence );
    print_result( "test_region_detection_failure", rc == -1 );
}

/**
 * Verifies that an invalid threshold (0 or less) returns error.
 */
void test_region_detection_invalid_threshold( )
{
    wrf_init( );
    wrf_region_score_t score = { 0 };
    score.eu868.score        = 3;
    wrf_update_score_metrics( &score );

    smtc_modem_region_t region;
    uint8_t             confidence = 0;
    int                 rc         = wrf_get_highest_score_region( &score, 0, &region, &confidence );
    print_result( "test_region_detection_invalid_threshold", rc == -1 );
}

/**
 * Verifies that no more than MAX_UNIQUE_APS are stored/scored.
 */
void test_max_unique_aps_limit( )
{
    wrf_init( );
    wrf_region_score_t score = { 0 };
    for( int i = 0; i < 128; i++ )
    {
        uint8_t mac[6] = { 0x00, 0x11, 0x22, ( uint8_t ) ( i >> 8 ), ( uint8_t ) ( i & 0xFF ), 0x00 };
        wrf_process_ap( mac, "freebox", "FR", &score );
    }
    int     before          = score.eu868.score;
    uint8_t mac_overflow[6] = { 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
    wrf_process_ap( mac_overflow, "freebox", "FR", &score );
    print_result( "test_max_unique_aps_limit", score.eu868.score == before );
}

/**
 * Verifies that a SSID with embedded null still matches the region.
 */
void test_ssid_with_embedded_null( )
{
    wrf_init( );
    wrf_region_score_t score    = { 0 };
    uint8_t            mac[6]   = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
    char               ssid[32] = { 'f', 'r', 'e', 'e', 'b', 'o', 'x', '\0', 'X', 'Y' };
    wrf_process_ap( mac, ssid, "FR", &score );
    print_result( "test_ssid_with_embedded_null", score.eu868.score == 2 );
}

/**
 * Verifies that symbols in SSID do not block detection.
 */
void test_ssid_with_symbols( )
{
    wrf_init( );
    wrf_region_score_t score  = { 0 };
    uint8_t            mac[6] = { 0xBA, 0xAD, 0xF0, 0x0D, 0x00, 0x02 };
    wrf_process_ap( mac, "FreeBox@Home!", "FR", &score );
    print_result( "test_ssid_with_symbols", score.eu868.score == 2 );
}

void test_confidence_vectors( )
{
    wrf_region_score_t score;

    // --- Scenario 1: Strong EU868 dominance
    wrf_init( );
    memset( &score, 0, sizeof( score ) );
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 0, 1 }, "Freebox", "FR", &score );  // +2 EU868
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 0, 2 }, "Livebox", "FR", &score );  // +2 EU868
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 0, 3 }, "bbox", "FR", &score );     // +2 EU868
    printf( "\n--- Scenario 1: Strong EU868 dominance ---\n" );
    wrf_print_region_scores( &score );

    // --- Scenario 2: Close between EU868 and US915
    wrf_init( );
    memset( &score, 0, sizeof( score ) );
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 1, 1 }, "Freebox", "FR", &score );      // +2 EU868
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 1, 2 }, "xfinitywifi", "US", &score );  // +2 US915
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 1, 3 }, "ATTWiFi", "US", &score );      // +2 US915
    printf( "\n--- Scenario 2: EU868 vs US915 ---\n" );
    wrf_print_region_scores( &score );

    // --- Scenario 3: Multiple similar scores
    wrf_init( );
    memset( &score, 0, sizeof( score ) );
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 2, 1 }, "ChinaNet", "CN", &score );  // +2 CN470
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 2, 2 }, "TPG", "AU", &score );       // +2 AU915
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 2, 3 }, "JioFiber", "IN", &score );  // +2 IN865
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 2, 4 }, "KT_WLAN", "KR", &score );   // +2 KR920
    printf( "\n--- Scenario 3: Diverse regions ---\n" );
    wrf_print_region_scores( &score );

    // --- Scenario 4: Low/no match
    wrf_init( );
    memset( &score, 0, sizeof( score ) );
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 3, 1 }, "UnknownSSID", "XX", &score );  // 0
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 3, 2 }, "Unknown2", "ZZ", &score );     // 0
    printf( "\n--- Scenario 4: No matches ---\n" );
    wrf_print_region_scores( &score );

    // --- Scenario 5: Weak single leader
    wrf_init( );
    memset( &score, 0, sizeof( score ) );
    wrf_process_ap( ( uint8_t[] ) { 0, 0, 0, 0, 4, 1 }, "Freebox", "", &score );  // +1 EU868 (SSID only)
    printf( "\n--- Scenario 5: Weak leader ---\n" );
    wrf_print_region_scores( &score );
}

// --- MAIN ---------------------------------------------------------------------

int main( )
{
    test_init_clears_state( );
    test_cc_and_ssid_scoring( );
    test_no_double_scoring( );
    test_only_cc( );
    test_only_ssid( );
    test_unknown_cc_and_ssid( );
    test_region_detection_success( );
    test_region_detection_ambiguous( );
    test_region_detection_below_threshold( );
    test_region_detection_failure( );
    test_region_detection_invalid_threshold( );
    test_max_unique_aps_limit( );
    test_ssid_with_embedded_null( );
    test_ssid_with_symbols( );

    test_confidence_vectors( );

    printf( "\n---- TEST SUMMARY ----\n" );
    printf( "Tests passed : %d\n", passed_count );
    printf( "Tests failed : %d\n", failed_count );
    printf( "-----------------------\n" );

    return failed_count == 0 ? 0 : 1;
}
