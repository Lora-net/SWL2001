# LR11xx driver changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
* [GNSS] `lr1110_gnss_get_consumption` function
* [Wi-Fi] `lr1110_wifi_get_consumption` function
* [Wi-Fi] `lr11xx_wifi_are_scan_mode_result_format_compatible` function

### Changed

* [radio] Call to `lr11xx_radio_apply_high_acp_workaround()` in `lr11xx_radio_set_tx_with_timeout_in_rtc_step()`, `lr11xx_radio_set_rx_with_timeout_in_rtc_step()`, `lr11xx_radio_set_cad()` and `lr11xx_radio_set_tx_infinite_preamble()` (can be removed by defining `LR11XX_DISABLE_HIGH_ACP_WORKAROUND`)
* [Wi-Fi] Define type `lr11xx_wifi_country_code_str_t` for Wi-Fi country code data and update `lr11xx_wifi_extended_full_result_t` and `lr11xx_wifi_country_code_t` to use it.

## [v1.0.0] Unreleased

### Added

* [all] Initial version - based on LR1110 driver v7.0.0
