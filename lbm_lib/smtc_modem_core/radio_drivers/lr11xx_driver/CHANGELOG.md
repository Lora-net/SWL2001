# LR11xx driver changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v2.4.1] 2023-11-16

### Changed

- [GNSS] `lr11xx_gnss_compute_power_consumption` function uses floating point for internal calculus and returns nAh and nWh instead of uAh and uWh
- [Wi-Fi] `lr11xx_wifi_get_consumption_nah` function uses floating point for internal calculus.

### Removed

- [GNSS] `lr11xx_gnss_get_consumption` function
- [GNSS] `lr11xx_gnss_timings_t` type

## [v2.4.0] 2023-10-13

### Added

- [GNSS] `lr11xx_gnss_read_almanac_per_satellites` function
- [GNSS] `lr11xx_gnss_scan` function
- [GNSS] `lr11xx_gnss_fetch_time` function
- [GNSS] `lr11xx_gnss_read_time` function
- [GNSS] `lr11xx_gnss_reset_time` function
- [GNSS] `lr11xx_gnss_reset_position` function
- [GNSS] `lr11xx_gnss_read_week_number_rollover` function
- [GNSS] `lr11xx_gnss_read_demod_status` function
- [GNSS] `lr11xx_gnss_read_cumulative_timing` function
- [GNSS] `lr11xx_gnss_compute_power_consumption` function
- [GNSS] `lr11xx_gnss_set_time` function
- [GNSS] `lr11xx_gnss_config_delay_reset_assistance_position` function
- [GNSS] `lr11xx_gnss_read_doppler_solver_result` function
- [GNSS] `lr11xx_gnss_read_delay_reset_assistance_position` function
- [GNSS] `lr11xx_gnss_almanac_update_from_sat` function
- [GNSS] `lr11xx_gnss_read_keep_sync_status` function
- [GNSS] `lr11xx_gnss_read_almanac_status` function
- [GNSS] `lr11xx_gnss_config_almanac_update_period` function
- [GNSS] `lr11xx_gnss_read_keep_sync_status` function
- [GNSS] `lr11xx_gnss_read_almanac_status` function
- [GNSS] `lr11xx_gnss_config_almanac_update_period` function
- [GNSS] `lr11xx_gnss_read_almanac_update_period` function
- [GNSS] `lr11xx_gnss_get_sv_sync` function
- [GNSS] `lr11xx_gnss_set_gps_bit_mask_sat_activated` function
- [GNSS] `lr11xx_gnss_set_beidou_bit_mask_sat_activated` function
- [GNSS] `lr11xx_gnss_read_last_scan_mode_launched` function
- [HAL] `lr11xx_hal_abort_blocking_cmd` function
- [system] `lr11xx_system_abort_blocking_cmd` function

### Changed

- Arguments `length` and `offset` renamed respectively to `length_in_word` and `offset_in_byte` in functions `lr11xx_bootloader_write_flash_encrypted` and `lr11xx_bootloader_write_flash_encrypted_full`
- [RTToF] Rename *ranging* as *RTToF* (Round-Trip Time of Flight)
- [Bluetooth®-Low-Energy-Beaconing-Compatibility] Rename *BLE TX* as *Bluetooth® Low Energy Beaconing Compatibility*
- [Wi-Fi] Rename *lr11xx_wifi_get_consumption* function as *lr11xx_wifi_get_consumption_nah* and returns the power consumption in nAh

### Removed

- [radio] `lr11xx_gnss_apply_mixer_cfg_workaround` function
- [RTToF] `lr11xx_ranging_get_recommended_rx_tx_delay_indicator` as the recommanded delay indicator is hardware dependant
- [RTToF] `lr11xx_ranging_set_recommended_rx_tx_delay_indicator` as the recommanded delay indicator is hardware dependant
- [GNSS] `lr11xx_gnss_get_timings` function
- [GNSS] `lr11xx_gnss_scan_autonomous` function
- [GNSS] `lr11xx_gnss_scan_assisted` function
- [GNSS] `lr11xx_gnss_apply_mixer_cfg_workaround` function

## [v2.3.0] 2023-05-10

### Added

* [radio] `lr11xx_radio_set_lora_sync_timeout_with_mantissa_exponent()` function
* [radio] `LR11XX_RADIO_PKT_TYPE_BPSK` packet type, with `lr11xx_radio_set_bpsk_pkt_params` and `lr11xx_radio_set_bpsk_mod_params` functions
* [GNSS] `lr11xx_gnss_apply_mixer_cfg_workaround()` function
* [GNSS] Add function `lr11xx_gnss_read_gnss_rssi_test` to read RSSI on GNSS path
* [GNSS] Add function `lr11xx_gnss_read_freq_search_space`
* [GNSS] Add function `lr11xx_gnss_set_freq_search_space`
* [GNSS] Add function `lr11xx_gnss_get_nb_visible_satellites`
* [GNSS] Add function `lr11xx_gnss_get_visible_satellites`
* [GNSS] add structure `lr11xx_gnss_visible_satellite_t` to support `lr11xx_gnss_get_visible_satellites` function
* [lr-fhss] Add function `lr11xx_lr_fhss_get_bit_delay_in_us` to compute the delay between the last LR-FHSS bit sent and the TX done interrupt
* [radio] Add function `lr11xx_radio_set_lr_fhss_sync_word` to configure the LR-FHSS syncword
* [radio] Add function `lr11xx_radio_set_lr_fhss_mod_params` to configure the LR-FHSS modulation parameters
* [radio] Add helper function `lr11xx_radio_convert_nb_symb_to_mant_exp` to convert a number of symbols into mantissa/exponent representation
* [crypto] Add functions  `lr11xx_crypto_check_encrypted_firmware_image` and `lr11xx_crypto_get_check_encrypted_firmware_image_result`; and helper function `lr11xx_crypto_check_encrypted_firmware_image_full` to check suitability of encrypted firmware image prior to actual flashing
* [ranging] Add `lr11xx_ranging_*` functions as part of new ranging component `lr11xx_ranging`

### Changed

* [GNSS] Call to `lr11xx_gnss_apply_mixer_cfg_workaround()` in `lr11xx_gnss_scan_autonomous()` and `lr11xx_gnss_scan_assisted()` (can be removed by defining `LR11XX_DISABLE_MIXER_CFG_WORKAROUND`)
* [system] Field `type` of `lr11xx_system_version_t` is now an enumerated type named `lr11xx_system_version_type_t`
* [system] Add IRQ raised on LoRa symbol received over-the-air
* [radio] `lr11xx_radio_set_lora_sync_timeout` takes argument `nb_symbol` as `uint16_t`
* [regmem] `lr11xx_regmem_write_regmem32` and `lr11xx_regmem_read_regmem32` test the number of words to write/read before actually requesting the chip, and raise an error status if it is higher than 64

## [v2.1.1] 2022-04-06

### Fixed

* [radio] Order of the parameters in `lr11xx_radio_set_rssi_calibration()` function

## [v2.1.0] 2022-03-28

### Added

* [radio] `lr11xx_radio_apply_high_acp_workaround()` function
* [radio] `lr11xx_radio_set_rssi_calibration()` function
* [radio] `LR11XX_RADIO_LORA_BW_200`, `LR11XX_RADIO_LORA_BW_400` and `LR11XX_RADIO_LORA_BW_800` entries to `lr11xx_radio_lora_bw_t`
* [radio] `LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP` entry in `lr11xx_radio_gfsk_pkt_len_modes_t` to support compatibility with SX128x
* [radio] `LR11XX_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP` entry in `lr11xx_radio_gfsk_dc_free_t` to support compatibility with SX128x
* [GNSS] `lr11xx_gnss_get_consumption` function
* [Wi-Fi] `lr11xx_wifi_get_consumption` function
* [Wi-Fi] `lr11xx_wifi_are_scan_mode_result_format_compatible` function

### Changed

* [radio] Call to `lr11xx_radio_apply_high_acp_workaround()` in `lr11xx_radio_set_tx_with_timeout_in_rtc_step()`, `lr11xx_radio_set_rx_with_timeout_in_rtc_step()`, `lr11xx_radio_set_cad()` and `lr11xx_radio_set_tx_infinite_preamble()` (can be removed by defining `LR11XX_DISABLE_HIGH_ACP_WORKAROUND`)
* [Wi-Fi] Define type `lr11xx_wifi_country_code_str_t` for Wi-Fi country code data and update `lr11xx_wifi_extended_full_result_t` and `lr11xx_wifi_country_code_t` to use it.

## [v1.0.0] Unreleased

### Added

* [all] Initial version - based on LR1110 driver v7.0.0
