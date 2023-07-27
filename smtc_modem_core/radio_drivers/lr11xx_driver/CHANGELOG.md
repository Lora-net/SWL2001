# LR11xx driver changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
