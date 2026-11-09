# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v2.1.0] - 2026-10-16

### Added

- [common] `lr20xx_radio_common_set_cca_with_gain`
- [LoRa] Intra packet hopping IRQ `LR20XX_SYSTEM_IRQ_LORA_TX_RX_HOP`
- [LoRa] `lr20xx_radio_lora_configure_rx_bandwidth_extension`
- [FLRC] `freq_offset_hz` field to `lr20xx_radio_flrc_pkt_status_t`
- [FLRC] Pulse shape BT 0.3
- [LR-FHSS] `lr20xx_radio_lr_fhss_read_hopping_table` and `lr20xx_radio_lr_fhss_write_hopping_table`
- [Z-Wave] `lr20xx_radio_z_wave_execute_scan_operation`
- [workarounds] Add `lr20xx_workarounds_flrc_configure_double_rate_sampling`, `lr20xx_workarounds_flrc_configure_double_rate_sampling_store_retention_mem`
- [workarounds] `lr20xx_workarounds_lora_disable_rx_length_check` and `lr20xx_workarounds_lora_disable_rx_length_check_store_retention_mem`
- [workarounds] `lr20xx_workarounds_flrc_disable_if_correlators`, `lr20xx_workarounds_flrc_disable_if_correlators_store_retention_mem`
- [BPSK] Add pulse shape values:
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RC_0_3`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RC_0_5`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RC_0_7`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RC_1_0`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_1_0`
- [system] `lr20xx_system_configure_xosc_wait_time`
- [workaround] Add `lr20xx_workaround_clear_post_temperature_read`
- [pram] Add `lr20xx_pram_check_pram_lr2021` and `lr20xx_pram_check_pram_lr20x2`

### Fixed

- [FSK] `lr20xx_radio_fsk_get_time_on_air_numerator` to handle correctly `LR20XX_RADIO_FSK_HEADER_16BITS`
- [system] Document expected version returned for LR2012
- [Bluetooth_LE] Remove unused workaround header include
- [common] Fix `lr20xx_radio_common_get_cca_result` cbuffer length in call to `lr20xx_hal_read`
- [BPSK] Fix pulse shape values:
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_3`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_5`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_7`
- [LoRa] Fix upper value of `lr20xx_radio_lora_hopping_cfg_t::hop_period` field
- [system] `lr20xx_system_get_temp` automatically calls workaround `lr20xx_workaround_clear_post_temperature_read` to avoid incorrect value read with `lr20xx_system_get_vbat`

### Changed

- [common] CCA result resolution configurable in `lr20xx_radio_common_get_cca_result` call
- [rttof] Extended RTToF result `lr20xx_rttof_results_extended_t` has two raw ranging results and one RSSI result
- [system] Rename `LR20XX_SYSTEM_SET_LBD_CFG_OC` and `LR20XX_SYSTEM_SET_LBD_CFG_CMD_LENGTH`
- [pram] Update PRAMs to 0316/0326

### Removed

- [Z-Wave] `lr20xx_radio_z_wave_set_scan`
- [LoRa] `LR20XX_RADIO_LORA_NO_CR`

## [v2.0.2] - 2026-04-07

### Added

- Add patch RAM related functions:
  - `lr20xx_patch_load_pram`
  - `lr20xx_patch_get_version`
  - `lr20xx_patch_enable_pram`
  - `lr20xx_pram_load_pram_lr2021`
  - `lr20xx_pram_load_pram_lr20x2`

### Changed

- [Z-Wave] `lr20xx_radio_z_wave_get_pkt_status` returns the channel of last received packet
- [OQPSK 15.4] `lr20xx_radio_oqpsk_15_4_params_t` has a new parameter to control length check bypass on received packet

### Fixed

- Fix `lr20xx_radio_lora_get_rx_statistics` and `lr20xx_radio_lora_rx_statistics_t` to read `n_header_valid`
- Fix `lr20xx_radio_fsk_get_rx_bandwidth` that was missing bandwidth `LR20XX_RADIO_FSK_COMMON_RX_BW_119_000_HZ`

### Removed

- [Bluetooth_LE] `lr20xx_workarounds_bluetooth_le_phy_coded_syncwords`
- [Bluetooth_LE] `lr20xx_workarounds_bluetooth_le_phy_coded_frequency_drift` and `lr20xx_workarounds_bluetooth_le_phy_coded_frequency_drift_store_retention_mem`
- [Bluetooth_LE] `lr20xx_workarounds_bluetooth_le_2mbps_preamble_length`
- [Bluetooth_LE] `lr20xx_workarounds_bluetooth_le_phy_coded_improve_blocking` and `lr20xx_workarounds_bluetooth_le_phy_coded_improve_blocking_store_retention_mem`
- [RTToF] `lr20xx_workarounds_rttof_truncate_pll_freq_step`
- [RTToF] `lr20xx_workarounds_rttof_rssi_computation`
- [RTToF] `lr20xx_workarounds_rttof_extended_stuck_second_request_enable`, `lr20xx_workarounds_rttof_extended_stuck_second_request_disable`, and `lr20xx_workarounds_rttof_extended_stuck_second_request_store_retention_mem`
- `lr20xx_workarounds_dcdc_reset`, `lr20xx_workarounds_dcdc_configure`, and `lr20xx_workarounds_dcdc_store_retention_mem`

## [v1.5.1] - 2026-02-09

### Added

- [LoRa] `freq_offset_hz` field to `lr20xx_radio_lora_packet_status_t`
- [FiFo] Add functions for 1024 byte Tx/Rx FiFos:
  - `lr20xx_radio_fifo_configure_1024_byte_tx_fifo`
  - `lr20xx_radio_fifo_1024_byte_tx_fifo_store_retention_mem`
  - `lr20xx_radio_fifo_configure_1024_byte_rx_fifo`
  - `lr20xx_radio_fifo_1024_byte_rx_fifo_store_retention_mem`
- [workarounds] Functions related to 1024-byte Tx/Rx FiFos low thresholds above 256
  - `lr20xx_workarounds_1024_byte_fifo_cfg_irq`
  - `lr20xx_workarounds_1024_byte_fifo_cfg_irq_store_retention_mem`
- [FLRC] `crc_ok` field to `lr20xx_radio_flrc_rx_stats_t`

## [v1.4.0] - 2026-01-26

### Added

- [workarounds] `lr20xx_workarounds_bluetooth_le_phy_coded_improve_blocking` and `lr20xx_workarounds_bluetooth_le_phy_coded_improve_blocking_store_retention_mem`

### Changed

- [Bluetooth_LE] `lr20xx_radio_bluetooth_le_set_modulation_pkt_params` calls `lr20xx_workarounds_bluetooth_le_phy_coded_improve_blocking`
- [regmem] Change implementation of `lr20xx_regmem_write_regmem32` and `lr20xx_regmem_read_regmem32` to extract the buffer length check in a function

### Fixed

- Remove erroneous mention in readme concerning application of workarounds

## [v1.3.4] - 2025-11-25

### Added

- [LoRa] Helper function `lr20xx_radio_convert_nb_symb_to_mant_exp`
- [FSK] Add value `LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_2_0` in `lr20xx_radio_fsk_pulse_shape_t`
- [bpsk] Add pulse shape values:
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_2_0`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_4`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_3`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_5`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_7`

### Removed

- [LoRa] Remove unsupported bandwidths 7, 10, 15, 20 kHz.
- [FLRC] Remove `LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_07`
- Fix comments for derivative parts

### Changed

- [LoRa] `lr20xx_radio_lora_configure_timeout_by_number_of_symbols` takes parameter as `uint16_t` and propagate to `lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols` when appropriate
- [FSK] `lr20xx_radio_fsk_mod_params_t` has field `br` replaced by `bitrate` and `bitrate_unit` to clarify the bitrate unit
- [LR-FHSS] `lr20xx_radio_lr_fhss_build_frame` takes configuration from structure `lr20xx_radio_lr_fhss_params_t`
- [LR-FHSS] Structure `lr20xx_radio_lr_fhss_params_t` and corresponding types does not depend on LR-FHSS V1 base types
- [LR-FHSS] Hopping modes includes test modes `LR20XX_RADIO_LR_FHSS_HOPPING_TEST_PAYLOAD` and `LR20XX_RADIO_LR_FHSS_HOPPING_TEST_PA`

### Fixed

- [LoRa] Typo for `lr20xx_radio_lora_hopping_ctrl_t`
- [OOK] Fix available pulse shape values in `lr20xx_radio_ook_pulse_shape_t` which becomes an enumeration, remove `lr20xx_radio_ook_pulse_shape_filter_t` and `lr20xx_radio_ook_pulse_shape_bt_t`

## [v1.3.1] - 2025-10-15

### Added

- Initial version
