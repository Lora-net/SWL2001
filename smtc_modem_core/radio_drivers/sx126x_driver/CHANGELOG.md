# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.0] - 2022-05-18

### Added

- `sx126x_set_gfsk_pkt_address()` function - configure both GFSK node and brodcast filtering addresses
- `sx126x_handle_rx_done()` function - perform all requested actions when the chip leaves the Rx mode

## [2.0.1] - 2021-11-23

### Added

- Support of LR-FHSS (see `sx126x_lr_fhss.c` and `sx126x_lr_fhss.h`)
- `sx126x_get_lora_params_from_header()` function - extracts the LoRa coding rate and CRC configuration from the packet header
- `sx126x_add_registers_to_retention_list()` function - allows to add up to 4 registers to the retention list
- `sx126x_init_retention_list()` - add registers used by workarounds in the driver to the retention list
- `sx126x_cal_img_in_mhz()` - takes frequency interval bounds in MHz for calibration

### Changed

- Revised BSD License changed to the Clear BSD License
- `sx126x_set_lora_symb_nb_timeout` now rounds up to nearest possible number of symbol
- `SX126X_REG_IRQ_POLARITY` is renamed `SX126X_REG_IQ_POLARITY`
- `sx126x_cal_img()` function - takes frequency interval bounds in raw steps

## [1.0.0] - 2020-09-24

### General

- Initial version
