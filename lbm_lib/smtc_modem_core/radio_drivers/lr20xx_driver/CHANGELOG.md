# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.3.0] - UNRELEASED

### Added

- [FLRC] `LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_07` as possible pulse shape value
- [Z-Wave] Document recommended value for timeout per channel for scan parameters
- [system] Add system error masks `LR20XX_SYSTEM_ERRORS_SRC_SATURATION_CALIB_MASK` and `LR20XX_SYSTEM_ERRORS_SRC_TOLERANCE_CALIB_MASK`

### Changed

- [FSK] Add `long_preamble_enabled` to `lr20xx_radio_fsk_pkt_params_t`

### Fixed

- [LoRa] Documentation about CAD params
- [regmem] The maximal number of words to read/write with `lr20xx_regmem_read_regmem32`/`lr20xx_regmem_write_regmem32` is 32

### Removed

- [workarounds] `lr20xx_workarounds_configure_iq_calibration_default`
- [workarounds] `lr20xx_workarounds_apply_calibration_unit_adc`
- [workarounds] `lr20xx_workarounds_z_wave_set_detection_length_for_lr_datarate`
- [workarounds] `lr20xx_workarounds_set_lf_ocp_threshold_900mhz`
- [workarounds] `lr20xx_workarounds_fsk_long_preamble_rx_support`
- [workarounds] `lr20xx_workaround_wi_sun_enable_fdev_tracking`
- [workarounds] `lr20xx_workaround_load_pram`
- [patch] Remove patch related functions
- [common] Remove flags to keep pram in retention while sleep in `lr20xx_radio_common_set_rx_duty_cycle` as there is no more pram
- [system] Remove flag to keep pram in retention in `lr20xx_system_set_sleep_mode`
- [FSK] Remove constraint for order of operation for FSK modulation/packet parameter configurations

## [v1.2.1] - 2025-07-07

### Added

- [wi-sun] Add `lr20xx_radio_wi_sun_set_packet_length`
- [common] Add configuration to maintain PRAM retention in `lr20xx_radio_common_set_rx_duty_cycle`

### Fixed

- [system] Typo for `LR20XX_SYSTEM_HF_CLK_SCALING_977_HZ`
- [system] Add error bit `LR20XX_SYSTEM_ERRORS_LF_XOSC_START_MASK`
- [system] Fix documentation of temperature computation for `lr20xx_system_get_temp`
- [FSK] Add configuration of payload length unit for `lr20xx_radio_fsk_set_packet_params`

### Removed

- [system] Remove an unused reset source value

## [v1.2.0] - 2025-06-17

### Added

- Initial version
