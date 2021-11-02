TEST_VALUE( LLCC68_STATUS_OK, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK, LLCC68_LORA_BW_250,
            LLCC68_LORA_SF10, LLCC68_LORA_CR_4_6, 0x00, 0x00, 0x04 )
TEST_VALUE( LLCC68_STATUS_UNSUPPORTED_FEATURE, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK,
            LLCC68_LORA_BW_125, LLCC68_LORA_SF10, LLCC68_LORA_CR_4_6, 0x00, 0x00, 0x04 )
TEST_VALUE( LLCC68_STATUS_UNSUPPORTED_FEATURE, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK,
            LLCC68_LORA_BW_125, LLCC68_LORA_SF11, LLCC68_LORA_CR_4_6, 0x00, 0x00, 0x04 )
TEST_VALUE( LLCC68_STATUS_UNSUPPORTED_FEATURE, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK, LLCC68_HAL_STATUS_OK,
            LLCC68_LORA_BW_250, LLCC68_LORA_SF11, LLCC68_LORA_CR_4_6, 0x00, 0x00, 0x04 )
void test_llcc68_set_lora_mod_params( llcc68_status_t status_expected, llcc68_hal_status_t hal_status_1,
                                      llcc68_hal_status_t hal_status_2, llcc68_hal_status_t hal_status_3,
                                      llcc68_lora_bw_t bw, llcc68_lora_sf_t sf, llcc68_lora_cr_t cr, uint8_t ldro,
                                      uint8_t reg_init_value, uint8_t reg_expected_value )
{
    uint8_t cbuffer_expected_1_1[] = { 0x8B, sf, bw, cr, ldro };
    uint8_t cbuffer_expected_1_2[] = { 0x1D, 0x08, 0x89, 0x00 };
    uint8_t rbuffer_expected_1_2[] = { 0x00 };
    uint8_t cbuffer_expected_1_3[] = { 0x0D, 0x08, 0x89 };
    uint8_t cdata_expected_1_3[]   = { reg_expected_value };

    uint8_t response_1_2[] = { reg_init_value };

    llcc68_mod_params_lora_t mod_params;

    mod_params.bw   = bw;
    mod_params.cr   = cr;
    mod_params.ldro = ldro;
    mod_params.sf   = sf;

    if( !( ( ( mod_params.bw == LLCC68_LORA_BW_250 ) && ( mod_params.sf == LLCC68_LORA_SF11 ) ) ||
           ( ( mod_params.bw == LLCC68_LORA_BW_125 ) &&
             ( ( mod_params.sf == LLCC68_LORA_SF11 ) || ( mod_params.sf == LLCC68_LORA_SF10 ) ) ) ) )
    {
        llcc68_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 5, 5, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 == LLCC68_STATUS_OK )
        {
            llcc68_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 4, 4, rbuffer_expected_1_2, 1, 1,
                                                      hal_status_2 );
            llcc68_hal_read_ReturnArrayThruPtr_data( response_1_2, 1 );
            if( hal_status_2 == LLCC68_STATUS_OK )
            {
                llcc68_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_3, 3, 3, cdata_expected_1_3, 1,
                                                           1, hal_status_3 );
            }
        }
    }

    status = llcc68_set_lora_mod_params( radio, &mod_params );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}
