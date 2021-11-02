
llcc68_status_t llcc68_set_lora_mod_params( const void* context, const llcc68_mod_params_lora_t* params )
{
    llcc68_status_t status = LLCC68_STATUS_ERROR;

    if( ( ( params->bw == LLCC68_LORA_BW_250 ) && ( params->sf == LLCC68_LORA_SF11 ) ) ||
        ( ( params->bw == LLCC68_LORA_BW_125 ) &&
          ( ( params->sf == LLCC68_LORA_SF11 ) || ( params->sf == LLCC68_LORA_SF10 ) ) ) )
    {
        status = LLCC68_STATUS_UNSUPPORTED_FEATURE;
    }
    else
    {
        const uint8_t buf[LLCC68_SIZE_SET_MODULATION_PARAMS_LORA] = {
            LLCC68_SET_MODULATION_PARAMS, ( uint8_t )( params->sf ), ( uint8_t )( params->bw ),
            ( uint8_t )( params->cr ),    params->ldro & 0x01,
        };

        status = llcc68_hal_write( context, buf, LLCC68_SIZE_SET_MODULATION_PARAMS_LORA, 0, 0 );

        if( status == LLCC68_STATUS_OK )
        {
            // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_LLCC68_V1.0 §15.1
            status = llcc68_tx_modulation_workaround( context, LLCC68_PKT_TYPE_LORA, params->bw );
            // WORKAROUND END
        }
    }

    return status;
}
