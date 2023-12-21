# LoRa Basics Modem Library changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.4.0] 2023-12-21

This version is based on feature branch v4.3.0 of the LoRa Basics Modem.
Detailed Modem API changelog can be found [here](lbm_lib/smtc_modem_api/CHANGELOG.md)

!!! Important !!! Due to bugfixes in WOR protocol, this version is not compatible with version 4.2.0 release in July 2023. Please update both the relay RX and the relay TX.

### Added
* [relay] Add Relay TX event 
* [relay] Add Relay TX modem API 

### Fixed
* [relay] Issue [#15](https://github.com/Lora-net/SWL2001/issues/15) LBM no longer ignore MAC command sent in response to forwarding request (FPort 226)
* [relay] Issue [#16](https://github.com/Lora-net/SWL2001/issues/16) Relay forward uplink after 50ms instead of 2s
* [relay] Issue [#17](https://github.com/Lora-net/SWL2001/issues/17) Add CID in notify request 
* [relay] Issue [#28](https://github.com/Lora-net/SWL2001/issues/28) Fix backoff field parse in EndDeviceConfReq
* [relay] Fix join request forwarding rules management for relay rx
* [relay] Fix sync status for relay tx
* [relay] Fix backoff mecanism for relay tx
* [relay] Manage dual value of default channel for relay rx
* [relay] Use real device crystal error for relay rx
* [relay] WOR : Fix typo in b0 buffer for MIC computation 
* [relay] WOR : Use frequency step of 100Hz instead of real frequency in MIC
* [relay] WOR ACK : Fix AckUplinkEnc endianess
* [relay] WOR ACK : Fix LoRa polarity
* [relay] WOR ACK : Use frequency and datarate of WOR ACK in MIC
* [relay] WOR ACK : Use frequency step of 100Hz instead of real frequency in MIC

### Changed
* [ALCSync] ALC sync request is now send after 5s instead of 2s


## [v4.3.0] 2023-12-13

This version is based on feature branch v4.1.0 of the LoRa Basics Modem.

Detailed Modem API changelog can be found [here](lbm_lib/smtc_modem_api/CHANGELOG.md)
Detailed Modem HAL changelog can be found [here](lbm_lib/smtc_modem_hal/CHANGELOG.md)

### Added

* [radio] Re-add support of lr1121 radio
* [service] Add Geolocation services and associated makefile option
* [service] Add a Store and Forward service and associated makefile option
* [service] Re-added the following optional LBM services:
  * Device Management (DM) service and associated makefile option (related with LoRaCloud)
  * Large File Upload (LFU) service and associated makefile option (related with LoRaCloud)
  * Stream service and associated makefile option (related with LoRaCloud)
  * Almanac Update service  and associated makefile option (related with LoRaCloud)
* [general] Add a new folder `lbm_applications` that contains 3 specific implementation references on sereval MCU
* [general] Real Time OS compatibility (add hal function `smtc_modem_hal_user_lbm_irq`)
* [lr11xx-crypto] Re-add suspend/resume guard for lr11xx crypto access
* [makefile] Add `DEBUG_OPT` option to choose optimization in caseof DEBUG

### Fixed

* Issue [#12](https://github.com/Lora-net/SWL2001/issues/12): Correct ral_xxx_lr_fhss_get_hop_sequence_count prototype to return a ral_status_t and not the count
* Issue [#13](https://github.com/Lora-net/SWL2001/issues/13): Correct Firmware Management Protocol behavior issue
* Issue [#14](https://github.com/Lora-net/SWL2001/issues/14): Correct LoRaWAN packages parser missing default case in parser (Remote Multicast Setup V1 and V2, Firmware Management Protocol and Fragmented Data Block Transport V1 and V2 )  
* Issue [#19](https://github.com/Lora-net/SWL2001/issues/19): check all LoRaWAN packages answer overflows
* Issue [#20](https://github.com/Lora-net/SWL2001/issues/20): Index of the answer for McGroupStatusReq command is not fixed anymore
* Issue [#22](https://github.com/Lora-net/SWL2001/issues/22): In Remote Multicast Setup Package V1 and V2, check `lorawan_api_is_frequency_valid` and `lorawan_api_is_datarate_valid` return status according to `status_lorawan_t` enum and not boolean

* [LoRaWAN-context] In case there is no valid stack context in nvm, initialize Join Nonce to -1 so that it can accept first join accept with join nonce equal to 0
* [LoRaWAN] Add api to configure Gen App Key (used for Remote Multicast Setup package) and do not use by default the App key
*[lorawan-package] Fix several issues encountered while executing the LoRa Alliance certification on FUOTA V2 related packages
* [modem-test] Fix issue that lead to modem panic in case a cw test is launched for too long
* [Duty-cycle] Fix issue where the duty-cycle was not fully disable when a region without duty-cycle constraint was initialized

### Changed

* [general] Change repository organization to ease readability.  
            Put all LoRa Basics Modem libraray related code in `lbm_lib` folder.  
            Rename `utilities` folder into `lbm_examples` for generic Lora Basics Modem examples
* [lorawan-package] Change `lorawan_packages` folder organization to use a dedicated folder for each packages instead of fuota_v1 and fuota_V2

* [lr11xx-driver] Update lr11xx radio driver to v2.4.0 version ([CHANGELOG.md](lbm_lib/smtc_modem_core/radio_drivers/lr11xx_driver/CHANGELOG.md))
* [sx126x-driver] Update sx126x radio driver to v3.2.1 version ([CHANGELOG.md](lbm_lib/smtc_modem_core/radio_drivers/sx126x_driver/CHANGELOG.md))
* [sx128x-driver] Update sx128x radio driver to v1.0.0 version ([CHANGELOG.md](lbm_lib/smtc_modem_core/radio_drivers/sx128x_driver/CHANGELOG.md))
* [makefile] Set all LBM features options to `no` by default
* [certification-service] Update certification service to support FUOTA certification

## [v4.2.0] 2023-07-28
This version is based on feature branch v4.1.0 of the LoRa Basics Modem.
### Added
* [lorawan-stack] Add Relay RX feature that follows LoRaWAN® Specification TS011-1.0.0
* [lorawan-stack] Add Relay TX feature that follows LoRaWAN® Specification TS011-1.0.0

## [v4.1.0] 2023-07-27

This version is based on feature branch v4.0.1 of the LoRa Basics Modem.

Detailed Modem API changelog can be found [here](smtc_modem_api/CHANGELOG.md)
Detailed Modem HAL changelog can be found [here](smtc_modem_hal/CHANGELOG.md)

### Added

* [lorawan-stack] Add CSMA feature support, corresponding api functions and hw modem commands
* [lorawan-package] Add support of Firmware Management Protocol package that follows LoRaWAN® Specification TS006-1.0.0
* [lorawan-package] Add support of Multi-Package Access Protocol package that follows LoRaWAN® Specification TS007-1.0.0
* [lorawan-package] Add support of Remote Multicast Setup package that follows LoRaWAN® Specification TS005-2.0.0
* [lorawan-package] Add support of Application Layer Clock Synchronization that follows LoRaWAN® Specification TS003-2.0.0
* [lorawan-package] Add support of Fragmented Data Block Transport that follows LoRaWAN® Specification TS004-2.0.0
* [radio] Add support for sx1272 and sx1276 radios for experimental use only
* [lbm makefile] Add RADIO in options.mk to choose which LoRaWAN regions should be built.
* [lbm makefile] Add missing options in `make help` output
* [utilities] Add an makefile option in `app_options.mk` `LBM_BUILD_OPTIONS` to allow updating any LBM lib build options
* [ral_lr11xx_bsp] Add lr11xx new bsp functions `ral_lr11xx_bsp_get_lora_cad_det_peak` and `ral_lr11xx_bsp_get_rx_boost_cfg`
* [ral_sx126x_bsp] Add sx126x new bsp functions `ral_sx126x_bsp_get_lora_cad_det_peak`, `ral_sx126x_bsp_get_rx_boost_cfg` and `ral_sx126x_bsp_get_trim_cap`
* [ral_sx128x_bsp] Add sx128x new bsp function `ral_sx128x_bsp_get_lora_cad_det_peak`

### Fixed

* [AU915/US915] Fix LoRaWAN Link Adr Request channel mask control case 5 missing impact of 500MHz bank
* [class b multicast] Avoid opening multicast ping slots when the session is stopped
* [LRFHSS] Correct tx done timestamp with known bit padding delay to avoid issue with following rx windows (principally seen on US and AU regions)
* [porting tool] Correct porting tool test test_get_time_in_ms to avoid mis-alignment in radio symbol timeout
* [exti example] In `modem_event_callback`, use stack_id value given by `smtc_modem_get_event` instead of fixed defined one
* [ALCSync] Fix ALCsync periodic request timing issue

### Changed

* [lr11xx-driver] Update lr11xx radio driver to v2.3.0 version
* [sx126x-driver] Update sx126x radio driver to v2.2.0 version
* [radio-ral] Update ral to get cad specific features
* [Return codes] `smtc_modem_get_pin` and `smtc_modem_derive_keys` now return `SMTC_MODEM_RC_BUSY` in case modem is joining or joined (instead of SMTC_MODEM_RC_FAIL)

## [v4.0.1] 2023-03-16

### Fixed

* [utilities] fix incorrect location of `smtc_modem_is_irq_flag_pending()` check in exti example (shall be done before entering sleep)

## [v4.0.0] 2023-03-10

### General

This version is forked of the master branch v3.2.4 and proposes a major update of the LoRa Basics Modem.
The purpose is to have a complete and simple LoRaWAN stack implementation, including additional LoRaWAN packages (Application Layer Clock Synchronization, Fragmented Data Block Transport, Remote Multicast Setup).
Detailed Modem API changelog can be found [here](smtc_modem_api/CHANGELOG.md)
Detailed Modem HAL changelog can be found [here](smtc_modem_hal/CHANGELOG.md)

### Added

* [makefile]: add `options.mk` file that gathers all lbm build options

### Changed

* [behavior]: Under modem interruption (radio or timer), only the timestamp of the interruption is done (no radio or external peripheral access is done under irq). As a consequence the `smtc_modem_run_engine()` shall be called as soon as possible after any modem interruption (radio or timer)
* [makefile]: default Regional Parameters option is now RP2_103 (previous was RP2_101)
* [utilities]:
  * add `app_options.mk` file that gathers all application build options
  * [exti-example]:
    * update example to send periodical uplinks
    * in case compilation is done using `LR11XX_WITH_CREDENTIALS`, internal credentials won't be overridden
  * add hardware modem example (folder [hw_modem](utilities/user_app/hw_modem) )
  * add porting tool example
  * update smtc_hal_l4 code
  * update radio_hal code
  * update smtc_modem_hal.c code to be compliant with latest version of lbm

### Fixed

* Correct size error in `smtc_secure_element_get_pin`
* [makefile] Remove ARM-specific flag from compilation flag
* [makefile] Correct `MCU_FLAGS` issue

## [v3.3.0] 2023-05-31

### Added

* [general] Support of LR1121 radio (target: lr1121)
* [makefile] Provide a way to change any LBM define values in make command (use `EXTRAFLAGS` )
* [utilities] Add a porting on NUCLEO-L073 board using LL drivers for minimal flash usage
* [utilities] Add a porting tool in main examples to help during mcu porting

### Changed

* [lr11xx_driver] Update to version `v2.3.0`
* [sx126x_driver] Update to version `v2.2.0`
* [makefile] Default Regional Parameters option is now RP2-1.0.3 LoRaWAN® Regional Parameters (previous was RP2-1.0.1)
* [utilities] Add response code assert in exti example
* [utilities] Remove temperature from exti example and replace it with 32b counter
* [utilities] Remove unused implementations and calls of uart4 related functions in smtc_hal_l4
* [modem] Add randomness before any modem task that need to uplink something
* [alarm] Clamp alarm timer to 864000s ie 10 days
* [stack] Set minimal default reception window size to 16ms instead of 6 ms to avoid ping slots issue in FSK (can be changed be overriding `MIN_RX_WINDOW_DURATION_MS` define)

### Fixed

* Issue [#5](https://github.com/Lora-net/SWL2001/issues/5): Correct typo on MCU_FLAGS in makefile
* Issue [#6](https://github.com/Lora-net/SWL2001/issues/6): Example does not override EUI and Keys in case code is built with `CRYPTO=LR11XX_WITH_CREDENTIALS`
* Issue [#7](https://github.com/Lora-net/SWL2001/issues/7): Remove ARM-specific flag from common.mk
* Issue [#9](https://github.com/Lora-net/SWL2001/issues/9): Fix size error in smtc_secure_element_get_pin()
* [LBT] Fix lbt issue when tcxo startup delay is greater than default `RP_MARGIN_DELAY` value (8ms)
* [LRFHSS] Correct tx done timestamp with known bit padding delay to avoid issue with following rx windows (principally seen on US and AU regions)
* [AU915/US915] Fix LoRaWAN Link Adr Request issue (channel mask control case 5 missing impact on 500kHz bank)
* [utilities] In exti example, fix evaluation kit blue button missing pin in irq configuration

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
* [ClockSyncService/MAC] Fixed an issue where the Clock Synchronization Service was not reloaded when DeviceTimeAns was not received
* [DeviceTimeReq/MAC] Fixed an issue where the GPS epoch time became invalid if DeviceTimeAns not received

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
