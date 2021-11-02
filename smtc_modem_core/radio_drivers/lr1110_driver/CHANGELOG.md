# LR1110 driver changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v6.0.0] 2021-09-24

### Added

* [GNSS] `LR1110_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS` entry `lr1110_gnss_scan_mode_t`


### Changed

* [GNSS] `lr1110_gnss_set_scan_mode()` function does not take out parameter anymore and is now based on lr1110_hal_write()
* [GNSS] `lr1110_gnss_get_detected_satellites()` function returns also the doppler per satellite


### Removed

* [GNSS] `lr1110_gnss_scan_continuous()` function and `LR1110_GNSS_DOUBLE_SCAN_MODE` entry in lr1110_gnss_scan_mode_t

## [v5.0.1] 2021-07-19

### Added

* [bootloader] `lr1110_bootloader_clear_reset_status_info()` function
* [crypto] Functions now return a status
* [GNSS] `lr1110_gnss_get_result_destination()` function
* [GNSS] `lr1110_gnss_almanac_update()` function - replaces `lr1110_gnss_almanac_full_update()` and `lr1110_gnss_one_satellite_almanac_update()` functions
* [GNSS] `lr1110_gnss_compute_almanac_age` function
* [HAL] `lr1110_hal_direct_read()` function - replaces `lr1110_hal_write_read()` function and no longer requires bidirectional SPI
* [HAL] `LR1110_NOP` constant
* [system] `lr1110_system_clear_reset_status_info()` function
* [system] `lr1110_system_drive_dio_in_sleep_mode()` function
* [Wi-Fi] `LR1110_WIFI_SCAN_MODE_UNTIL_SSID` entry in `lr1110_wifi_mode_t`
* [Wi-Fi] `lr1110_wifi_is_well_formed_utf8_byte_sequence()` function

### Changed

* [LICENSE] Revised BSD License changed to the Clear BSD License
* [GNSS] `LR1110_GNSS_ERROR_UPDATE_TIME_DIFFERENCE_OVER_1_MONTH` is renamed `LR1110_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED` in `lr1110_gnss_error_code_t`
* [GNSS] `lr1110_gnss_parse_context_status_buffer()` returns a `lr1110_status_t` value
* [system] `lr1110_system_get_status()` function implementation uses `lr1110_hal_direct_read()` function and no longer requires bidirectional SPI
* [system] `lr1110_bootloader_get_status()` function implementation uses `lr1110_hal_direct_read()` function and no longer requires bidirectional SPI
* [system] `lr1110_system_get_irq_status()` function - has a faster implementation based on the `lr1110_system_get_status()` function

### Fixed

* [GNSS] Global almanac CRC endianness `lr1110_gnss_parse_context_status_buffer()`
* [GNSS] `lr1110_gnss_parse_context_status_buffer()` takes into account the message header
* [GNSS] Typo in `LR1110_GNSS_ALMANAC_REAC_RBUFFER_LENGTH` - now `LR1110_GNSS_ALMANAC_READ_RBUFFER_LENGTH`

### Removed

* [GNSS] `lr1110_gnss_get_almanac_crc()` - Almanac CRC can be read thanks to `lr1110_gnss_get_context_status()`
* [GNSS] `lr1110_gnss_almanac_full_update()` and `lr1110_gnss_one_satellite_almanac_update()` functions - merged in `lr1110_gnss_almanac_update()` function
* [HAL] `lr1110_hal_write_read()` function - replaced by `lr1110_hal_direct_read()` function
* [regmem] `lr1110_regmem_write_auxreg32()`and `lr1110_regmem_read_auxreg32()` functions
* [Wi-Fi] `lr1110_wifi_cfg_hardware_debarker()` function

## [v4.0.0] 2021-04-06

### Added

* [bootloader] `lr1110_bootloader_get_status` function
* [bootloader] `lr1110_bootloader_irq_mask_t` type definition
* [bootloader] `lr1110_bootloader_chip_modes_t` type definition
* [bootloader] `lr1110_bootloader_reset_status_t` type definition
* [bootloader] `lr1110_bootloader_command_status_t` type definition
* [bootloader] `lr1110_bootloader_stat1_t` type definition
* [bootloader] `lr1110_bootloader_stat2_t` type definition
* [timings] Add the functions to compute the radio timings
* [system] Add function `lr1110_system_enable_spi_crc` to enable or disable the CRC on SPI communication
* [HAL] Add the CRC calculation for SPI

### Fixed

* [GNSS] Fix typo: `lr1110_gnss_almanac_single_satellite_update_bytestram_t` to `lr1110_gnss_almanac_single_satellite_update_bytestream_t`
* [GNSS] Fix size of context status

### Changed

* [Wi-Fi] Added field `current_channel` to `lr1110_wifi_extended_full_result_t`

### Removed

* [bootloader] `lr1110_bootloader_get_hash` function
* [bootloader] `lr1110_bootloader_write_flash` function
* [bootloader] `lr1110_bootloader_write_flash_full` function
* [bootloader] `lr1110_bootloader_erase_page` function

## [v3.0.0] 2020-10-12

### Added

* [bootloader] `lr1110_bootloader_read_chip_eui` function
* [bootloader] `lr1110_bootloader_read_join_eui` function
* [bootloader] `LR1110_BL_CHIP_EUI_LENGTH` constant
* [bootloader] `LR1110_BL_JOIN_EUI_LENGTH` constant
* [bootloader] `lr1110_bootloader_chip_eui_t` type definition
* [bootloader] `lr1110_bootloader_join_eui_t` type definition
* [GNSS] `lr1110_gnss_get_context_status` function
* [GNSS] `lr1110_gnss_parse_context_status_buffer` function
* [GNSS] `LR1110_GNSS_CONTEXT_STATUS_LENGTH` constant
* [GNSS] `lr1110_gnss_error_code_t` type definition
* [GNSS] `lr1110_gnss_freq_search_space_t` type definition
* [GNSS] `lr1110_gnss_context_status_bytestream_t` type definition
* [GNSS] `lr1110_gnss_context_status_t` type definition
* [radio] `lr1110_radio_set_rx_with_timeout_in_rtc_step` and `lr1110_radio_set_tx_with_timeout_in_rtc_step` functions
* [radio] `lr1110_radio_set_rx_duty_cycle_with_timings_in_rtc_step` function
* [radio] `lr1110_radio_convert_time_in_ms_to_rtc_step` function
* [system] `lr1110_system_wakeup` function
* [system] `lr1110_system_read_pin_custom_eui` function
* [system] `reset_status` field to `lr1110_system_stat2_t`
* [Wi-Fi] `lr1110_wifi_scan_time_limit` function
* [Wi-Fi] `lr1110_wifi_search_country_code_time_limit` function
* [Wi-Fi] `lr1110_wifi_read_extended_full_results` function
* [Wi-Fi] `LR1110_WIFI_CHANNEL_*_POS` and `LR1110_WIFI_CHANNEL_*_MASK` constants
* [Wi-Fi] `LR1110_WIFI_SCAN_MODE_FULL_BEACON` entry in `lr1110_wifi_mode_t`
* [Wi-Fi] `lr1110_wifi_extended_full_result_t` type definition
* [Wi-Fi] `LR1110_WIFI_RESULT_FORMAT_EXTENDED_FULL` entry in `lr1110_wifi_result_format_t`

### Changed

* [crypto] `LR1110_CRYPTO_COMPUTE_AES_CMAC_CMD_LENGTH` is now equal to ( 2 + 1 + 272 )
* [GNSS] `LR1110_GNSS_FULL_ALMANAC_UPDATE_PACKET_LENGTH` is renamed `LR1110_GNSS_FULL_ALMANAC_UPDATE_PKT_LENGTH`
* [GNSS] `lr1110_gnss_scan_autonomous` takes also `effort_mode` as input parameter
* [radio] Implementation of time-on-air computation for LoRa modulation
* [radio] `lr1110_radio_set_lora_sync_word` takes a sync word as parameter instead of a network type
* [radio] `lr1110_radio_set_rx` and `lr1110_radio_set_tx` take a timeout parameter in millisecond instead of RTC step
* [radio] `lr1110_radio_set_rx_duty_cycle` takes a timeout parameter in millisecond instead of RTC step
* [radio] `LR1110_RADIO_PACKET_NONE` is renamed `LR1110_RADIO_PKT_NONE`
* [radio] `LR1110_RADIO_PA_REG_SUPPLY_DCDC` is renamed `LR1110_RADIO_PA_REG_SUPPLY_VREG`
* [radio] `lr1110_radio_pa_regulator_supply_t` is renamed `lr1110_radio_pa_reg_supply_t`
* [radio] `*_packet_*` is renamed `*_pkt_*` in `lr1110_radio_pkt_status_lora_t`
* [radio] `nb_packet_falsesync` is renamed `nb_pkt_falsesync` in `lr1110_radio_stats_lora_t`
* [Wi-Fi] `lr1110_extract_channel_info` is renamed `lr1110_wifi_parse_channel_info`
* [Wi-Fi] `lr1110_extract_channel_from_info_byte` is renamed `lr1110_wifi_extract_channel_from_info_byte`
* [Wi-Fi] `lr1110_extract_frame_type_info` is renamed `lr1110_wifi_parse_frame_type_info`
* [Wi-Fi] `lr1110_extract_data_rate_info` is renamed `lr1110_wifi_parse_data_rate_info`
* [Wi-Fi] `lr1110_wifi_n_results_max_per_chunk` is renamed `lr1110_wifi_get_nb_results_max_per_chunk`
* [Wi-Fi] `lr1110_extract_signal_type_from_data_rate_info` is renamed `lr1110_wifi_extract_signal_type_from_data_rate_info`
* [Wi-Fi] `LR1110_WIFI_ORIGIN_PACKET` is renamed `LR1110_WIFI_ORIGIN_UNKNWON`
* [Wi-Fi] `LR1110_WIFI_SCAN_MODE_BEACON_AND_PACKET` is renamed `LR1110_WIFI_SCAN_MODE_BEACON_AND_PKT`

### Fixed

* [all] Harmonized doxygen markups
* [all] Harmonized license header
* [all] Removed extraneous underscore in constants used for multiple inclusion protection
* [GNSS] Inversion of `LR1110_GNSS_BIT_CHANGE_MASK` and `LR1110_GNSS_IRQ_PSEUDO_RANGE_MASK` definitions
* [radio] Power amplifier ramp time values in lr1110_radio_ramp_time_t
* [system] `chip_mode` read from `stat2` value

## [v2.0.1] 2020-05-04

### Fixed

* [version] Enable c linkage driver version related functions

## [v2.0.0] 2020-04-27

### Added

* [all] All functions declared in .h files now return a status.
* [common] `lr1110_status_t` type definition
* [bootloader] `lr1110_bootloader_fill_cbuffer_opcode_offset_flash` static function
* [bootloader] `lr1110_bootloader_fill_cdata_flash` static function
* [bootloader] `lr1110_bootloader_fill_cbuffer_cdata_flash` static function
* [regmem] `lr1110_regmem_fill_cbuffer_opcode_address` static function
* [regmem] `lr1110_regmem_fill_cbuffer_opcode_address_length` static function
* [regmem] `lr1110_regmem_fill_cdata` static function
* [regmem] `lr1110_regmem_fill_cbuffer_cdata_opcode_address_data` static function
* [regmem] `lr1110_regmem_fill_out_buffer_from_raw_buffer` static function
* [system] `LR1110_SYSTEM_VERSION_LENGTH` constant
* [system] `lr1110_system_cal_mask_t` type definition
* [system] `lr1110_system_irq_mask_t` type definition
* [system] `lr1110_system_get_and_clear_irq_status` function
* [system] `lr1110_system_get_irq_status` function
* [crypto] `lr1110_crypto_fill_cbuffer_opcode_key_data` static function
* [GNSS] `lr1110_gnss_uint8_to_uint32` static function

### Changed

* [bootloader] `lr1110_bootloader_version_t` has now 3 fields: hardware, type, firmware
* [bootloader] `lr1110_bootloader_get_version` fills the updated `lr1110_bootloader_version_t` structure
* [system] `LR1110_SYSTEM_IRQ_NONE_MASK` is renamed `LR1110_SYSTEM_IRQ_NONE`
* [system] `LR1110_SYSTEM_IRQ_TXDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_TX_DONE`
* [system] `LR1110_SYSTEM_IRQ_RXDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_RX_DONE`
* [system] `LR1110_SYSTEM_IRQ_PREAMBLEDETECTED_MASK` is renamed `LR1110_SYSTEM_IRQ_PREAMBLE_DETECTED`
* [system] `LR1110_SYSTEM_IRQ_SYNCWORD_HEADERVALID_MASK` is renamed `LR1110_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID`
* [system] `LR1110_SYSTEM_IRQ_HEADERERR_MASK` is renamed `LR1110_SYSTEM_IRQ_HEADER_ERROR`
* [system] `LR1110_SYSTEM_IRQ_CRCERR_MASK` is renamed `LR1110_SYSTEM_IRQ_CRC_ERROR`
* [system] `LR1110_SYSTEM_IRQ_CADDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_CAD_DONE`
* [system] `LR1110_SYSTEM_IRQ_CADDETECTED_MASK` is renamed `LR1110_SYSTEM_IRQ_CAD_DETECTED`
* [system] `LR1110_SYSTEM_IRQ_TIMEOUT_MASK` is renamed `LR1110_SYSTEM_IRQ_TIMEOUT`
* [system] `LR1110_SYSTEM_IRQ_GNSSSCANDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_GNSS_SCAN_DONE`
* [system] `LR1110_SYSTEM_IRQ_WIFISCANDONE_MASK` is renamed `LR1110_SYSTEM_IRQ_WIFI_SCAN_DONE`
* [system] `LR1110_SYSTEM_IRQ_EOL_MASK` is renamed `LR1110_SYSTEM_IRQ_EOL`
* [system] `LR1110_SYSTEM_IRQ_CMDERR_MASK` is renamed `LR1110_SYSTEM_IRQ_CMD_ERROR`
* [system] `LR1110_SYSTEM_IRQ_ERR_MASK` is renamed `LR1110_SYSTEM_IRQ_ERROR`
* [system] `LR1110_SYSTEM_IRQ_FSK_LENGTH_ERROR_MASK` is renamed `LR1110_SYSTEM_IRQ_FSK_LEN_ERROR`
* [system] `LR1110_SYSTEM_IRQ_FSK_ADDRESS_ERROR_MASK` is renamed `LR1110_SYSTEM_IRQ_FSK_ADDR_ERROR`
* [system] `LR1110_SYSTEM_CALIBRATE_*_MASK` are renamed `LR1110_SYSTEM_CALIB_*_MASK`
* [system] `lr1110_system_chip_mode_t` is renamed `lr1110_system_chip_modes_t`
* [system] In `lr1110_system_chip_modes_t`, `LR1110_SYSTEM_CHIP_MODE_RC` is renamed `LR1110_SYSTEM_CHIP_MODE_STBY_RC`
* [system] In `lr1110_system_chip_modes_t`, `LR1110_SYSTEM_CHIP_MODE_XOSC` is renamed `LR1110_SYSTEM_CHIP_MODE_STBY_XOSC`
* [system] `lr1110_system_lfclk_config_t` is renamed `lr1110_system_lfclk_cfg_t`
* [system] `lr1110_regmodes_t` is renamed `lr1110_system_reg_mode_t`
* [system] `LR1110_SYSTEM_REGMODE_NO_DCDC` is renamed `LR1110_SYSTEM_REG_MODE_LDO`
* [system] `LR1110_SYSTEM_REGMODE_DCDC_CONVERTER` is renamed `LR1110_SYSTEM_REG_MODE_DCDC`
* [system] `lr1110_system_rfswitch_config_t` is renamed `lr1110_system_rfswitch_cfg_t`
* [system] `lr1110_system_standby_config_t` is renamed `lr1110_system_standby_cfg_t`
* [system] `LR1110_SYSTEM_STDBY_CONFIG_*` are renamed `LR1110_SYSTEM_STANDBY_CFG_*`
* [system] `LR1110_SYSTEM_TCXO_SUPPLY_VOLTAGE_*V` are renamed `LR1110_SYSTEM_TCXO_CTRL_*V`
* [system] `lr1110_system_sleep_config_t` is renamed `lr1110_system_sleep_cfg_t`
* [system] `lr1110_system_set_regmode` is renamed `lr1110_system_set_reg_mode`
* [system] `lr1110_system_config_lfclk` is renamed `lr1110_system_cfg_lfclk`
* [system] `lr1110_system_clear_irq` is renamed `lr1110_system_clear_irq_status`
* [crypto] `lr1110_crypto_derive_and_store_key` is renamed `lr1110_crypto_derive_key`
* [crypto] `LR1110_CRYPTO_DERIVE_AND_STORE_KEY_OC` is renamed `LR1110_CRYPTO_DERIVE_KEY_OC`
* [crypto] `LR1110_CRYPTO_DERIVE_AND_STORE_KEY_CMD_LENGTH` is renamed `LR1110_CRYPTO_DERIVE_KEY_CMD_LENGTH`
* [radio] `lr1110_radio_rx_tx_fallback_mode_t` is renamed `lr1110_radio_fallback_modes_t`
* [radio] `LR1110_RADIO_RX_TX_FALLBACK_MODE_*` are renamed `LR1110_RADIO_FALLBACK_*`
* [radio] `LR1110_RADIO_RAMP_TIME_*` are renamed `LR1110_RADIO_RAMP_*`
* [radio] `LR1110_RADIO_LORA_BW*` are renamed `LR1110_RADIO_LORA_BW_*`
* [radio] `LR1110_RADIO_LORA_CRXY_LI` are renamed `LR1110_RADIO_LORA_CR_LI_X_Y`
* [radio] `LR1110_RADIO_LORA_CRXY` are renamed `LR1110_RADIO_LORA_CR_X_Y`
* [radio] `LR1110_RADIO_MODE_STANDBY*` are renamed `LR1110_RADIO_MODE_STANDBY_*`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTE` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTE`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTES` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTES`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTE_INV` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTE_INV`
* [radio] `LR1110_RADIO_GFSK_CRC_XBYTES_INV` are renamed `LR1110_RADIO_GFSK_CRC_X_BYTES_INV`
* [radio] `LR1110_RADIO_GFSK_DCFREE_*` are renamed `LR1110_RADIO_GFSK_DC_FREE_*`
* [radio] `lr1110_radio_gfsk_header_type_t` is renamed `lr1110_radio_gfsk_pkt_len_modes_t`
* [radio] `LR1110_RADIO_GFSK_HEADER_TYPE_IMPLICIT` is renamed `LR1110_RADIO_GFSK_PKT_FIX_LEN`
* [radio] `LR1110_RADIO_GFSK_HEADER_TYPE_EXPLICIT` is renamed `LR1110_RADIO_GFSK_PKT_VAR_LEN`
* [radio] `lr1110_radio_gfsk_preamble_detect_length_t` is renamed `lr1110_radio_gfsk_preamble_detector_t`
* [radio] `LR1110_RADIO_GFSK_PREAMBLE_DETECTOR_LENGTH_*` are renamed `LR1110_RADIO_GFSK_PREAMBLE_DETECTOR_*`
* [radio] `lr1110_radio_lora_header_type_t` is renamed `lr1110_radio_lora_pkt_len_modes_t`
* [radio] `LR1110_RADIO_LORA_HEADER_*` are renamed `LR1110_RADIO_LORA_PKT_*`
* [radio] `lr1110_radio_packet_types_t` is renamed `lr1110_radio_pkt_type_t`
* [radio] `LR1110_RADIO_PACKET_*` are renamed `LR1110_RADIO_PKT_TYPE_*`
* [radio] `lr1110_radio_gfsk_rx_bw_t` is renamed `lr1110_radio_gfsk_bw_t`
* [radio] `LR1110_RADIO_GFSK_RX_BW_*` are renamed `LR1110_RADIO_GFSK_BW_*`
* [radio] `lr1110_radio_pulse_shape_t` is renamed `lr1110_radio_gfsk_pulse_shape_t`
* [radio] `LR1110_RADIO_PULSESHAPE_*` are renamed `LR1110_RADIO_GFSK_PULSE_SHAPE_*`
* [radio] In `lr1110_radio_cad_params_t`, `symbol_num` is renamed `cad_symb_nb`
* [radio] In `lr1110_radio_cad_params_t`, `det_peak` is renamed `cad_detect_peak`
* [radio] In `lr1110_radio_cad_params_t`, `det_min` is renamed `cad_detect_min`
* [radio] In `lr1110_radio_cad_params_t`, `exit_mode` is renamed `cad_exit_mode`
* [radio] In `lr1110_radio_cad_params_t`, `timeout` is renamed `cad_timeout`
* [radio] `lr1110_radio_packet_status_gfsk_t` is renamed `lr1110_radio_pkt_status_gfsk_t`
* [radio] In `lr1110_radio_pkt_status_gfsk_t`, `rx_length_in_bytes` is renamed `rx_len_in_bytes`
* [radio] `lr1110_radio_packet_status_lora_t` is renamed `lr1110_radio_pkt_status_lora_t`
* [radio] `lr1110_radio_rxbuffer_status_t` is renamed `lr1110_radio_rx_buffer_status_t`
* [radio] In `lr1110_radio_rx_buffer_status_t`, `rx_payload_length` is renamed `pld_len_in_bytes`
* [radio] In `lr1110_radio_rx_buffer_status_t`, `rx_start_buffer_pointer` is renamed `buffer_start_pointer`
* [radio] In `lr1110_radio_stats_gfsk_t`, `nb_packet_*` are renamed `nb_pkt_*`
* [radio] In `lr1110_radio_stats_lora_t`, `nb_packet_received` is renamed `nb_pkt_received`
* [radio] In `lr1110_radio_stats_lora_t`, `nb_packet_error_crc` is renamed `nb_pkt_error_crc`
* [radio] In `lr1110_radio_stats_lora_t`, `nb_packet_error_header` is renamed `nb_pkt_header_error`
* [radio] `lr1110_radio_modulation_param_*_t` are renamed `lr1110_radio_mod_params_*_t`
* [radio] In `lr1110_radio_mod_params_gfsk_t`, `bitrate` is renamed `br_in_bps`
* [radio] In `lr1110_radio_mod_params_gfsk_t`, `bandwidth` is renamed `bw_dsb_param`
* [radio] In `lr1110_radio_mod_params_gfsk_t`, `fdev` is renamed `fdev_in_hz`
* [radio] In `lr1110_radio_mod_params_lora_t`, `spreading_factor` is renamed `sf`
* [radio] In `lr1110_radio_mod_params_lora_t`, `coding_rate` is renamed `cr`
* [radio] In `lr1110_radio_mod_params_lora_t`, `ppm_offset` is renamed `ldro`
* [radio] `lr1110_radio_packet_param_*_t` are renamed `lr1110_radio_pkt_params_*_t`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `preamble_length_tx_in_bit` is renamed `preamble_len_in_bits`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `preamble_detect` is renamed `preamble_detector`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `sync_word_length_in_bit` is renamed `sync_word_len_in_bits`
* [radio] In `lr1110_radio_pkt_params_gfsk_t`, `payload_length_in_byte` is renamed `pld_len_in_bytes`
* [radio] In `lr1110_radio_pkt_params_lora_t`, `preamble_length_in_symb` is renamed `preamble_len_in_symb`
* [radio] In `lr1110_radio_pkt_params_lora_t`, `payload_length_in_byte` is renamed `pld_len_in_bytes`
* [radio] `lr1110_radio_pa_config_t` are renamed `lr1110_radio_pa_cfg_t`
* [Wi-Fi] `lr1110_wifi_configure_hardware_debarker` is renamed `lr1110_wifi_cfg_hardware_debarker`

### Fixed

* [system] Enable c linkage for system-related functions

### Removed

* [system] `LR1110_SYSTEM_IRQ_FHSS_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_INTERPACKET1_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_INTERPACKET2_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGREQVLD_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGREQDISC_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGRESPDONE_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGEXCHVLD_MASK` constant
* [system] `LR1110_SYSTEM_IRQ_RNGTIMEOUT_MASK` constant

## [v1.0.0] 2020-03-18

### Added

* [all] Initial version
