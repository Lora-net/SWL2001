# LoRa Basics Modem Library changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v3.2.4] 2022-08-23

### Added

* AS923 group 4 Regional Parameters
* WW2G4 Regional Parameters for LoRaWAN protocol emulation
* Support of sx128x radio

### Changed

* [lr11xx_driver] Update to version `v2.1.1`
* [sx126x_driver] Update to version `v2.1.0`
* [makefile] remove ARM Cortex option from makefile to make LoRa Basics Modem completely agnostic from the MCU. Makefile shall be called with a new MCU_FLAGS option containing all core options
* [makefile] Align built target directory with crypto compilation options
* [utility/example] Update PA configuration process in `ral_lr11xx_bsp_get_tx_cfg` function.
* [utility/example] Update `stm32l476rgtx_flash.ld` files to fix stack start and stop address
* [utility/example] Remove `ral_lr11xx_bsp_get_rssi_calibration_table` workaround as the lr11xx driver was fixed
* [utility/example] Fix `hal_rtc_get_time_ms` so that it returns a full range value
* Clock Sync Service with ALC Sync source can generate events:
  * SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC
* Clock Sync Service with DeviceTimeReq source can generate events:
  * SMTC_MODEM_EVENT_TIME_VALID_BUT_NOT_SYNC
  * SMTC_MODEM_EVENT_TIME_NOT_VALID

### Fixed

* [LFU] LoRa Basics Modem now rejects properly files with a size between 8181 and 8192 bytes
* [LFU] Fix issue regarding encryption of files with size higher than 4080 bytes
* [RP] Fix issue on radio interruption timestamp
* [LBT] On lr11xx targets, correct outdated LBT pre-hook issue
* [LBT] Remove log print when uplinking on fsk to avoid adding delay on scheduled tasks
* [LBT] Moved log print after enqueued the sniffing task in Radio Planer to avoid to add a delays
* [ADR] In case a MAC command `link_adr_req` with a new channel mask is received, it is now accepted if the custom datarate profile is enabled and configured with the highest datarate of the corresponding region
* [LFU/Stream] In case of reception of rejoin request from DAS, reset LFU and stream services properly
*[ClockSyncService/MAC] Fixed an issue where the Clock Synchronization Service was not reloaded when DeviceTimeAns was not received
*[DeviceTimeReq/MAC] Fixed an issue where the GPS epoch time became invalid if DeviceTimeAns not received

## [v3.1.7] 2022-04-22

### Added

* AS923 (3 sub regions included), IN865, KR920, RU864, AU915 Regional Parameters
* Class B support
* Class B Multicast support (up to 4 sessions)
* Class C Multicast support (up to 4 sessions)
* LR-FHSS Support (enabled with compilation option: `RP_VERSION=RP2_103`)
* Support of SX1261 and SX1262 radios
* Added commands:
  * New connectivity check function: smtc_modem_lorawan_get_lost_connection_counter
  * Makefile: add Regional Parameters option to choose to compile the code for RP2_101 or RP2_103
  * [smtc_modem_hal]:
    * `smtc_modem_hal_assert(expr)` macro
    * `smtc_modem_hal_assert_fail()` function
    * `smtc_modem_hal_get_time_in_100us()` function
    * `smtc_modem_hal_get_radio_irq_timestamp_in_100us()` function
  * In `SMTC_MODEM_EVENT_DOWNDATA` event status: added new class B reception windows, fpending bit status, reception frequency and datarate
  * Middleware API for geolocation
* Add basic example to provide an easy start point on Nucleo L476 board

### Changed

* `smtc_modem_set_crystal_error` renamed to `smtc_modem_set_crystal_error_ppm` and now takes real ppm (previously was ppt)
* `smtc_modem_get_stack_state`: Added a new stack state `SMTC_MODEM_STACK_STATE_TX_WAIT` when stack is between retransmissions
* `smtc_modem_time_trigger_sync_request` function does not take `sync_service` parameter anymore, now it will use the current enabled time synchronization service
* [smtc_modem_hal]:
  * `smtc_modem_hal_irq_is_radio_irq_pending()` function has been replaced with `smtc_modem_hal_radio_irq_clear_pending()`. Now modem only asks to clear radio pending irq
* LR1110 driver was renamed to LR11xx driver and now also supports LR1120 radio
* Updated to latest version of SX126x and LR11xx driver
* An `ALMANAC_UPDATE` event is generated if "Almanac force update" is received.
* File upload size can be now up to 8k
* Remove -2dB default tx power offset (now it is 0) and manage EIRP to ERP conversion in LoRaWAN stack
* `smtc_modem_connection_timeout_get_thresholds`: Default internal value of `nb_of_uplinks_before_network_controlled` is now 0 (before was 255). Result: the mobile to static automatic switching service is now deactivated by default.

### Fixed

* Corrected `Fcnt_down` msb management
* `smtc_modem_derive_keys` now takes user defined EUIs into account
* AU915: when dwell time was on, the returned max payload sizes were incorrect. This has been corrected
* Corrected bug in `smtc_modem_reset_charge`
* Internal join nonce value is now initialized to FFFFFF to avoid dropping the first join accept message

## [v2.1.0] 2021-11-03

Initial release
