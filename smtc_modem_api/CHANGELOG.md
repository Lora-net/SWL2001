# Lora Basics Modem API changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.1.0] 2023-07-07

### Added

* Event `SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT` and associated status structure used by LoRaWAN Firmware Management Package
* CSMA functions:
  * `smtc_modem_csma_set_state` to enable/disable the feature
  * `smtc_modem_csma_get_state` to get the current status of the csma feature
  * `smtc_modem_csma_set_parameters` to set optional parameters for csma
  * `smtc_modem_csma_get_parameters` to get the current parameters used by the csma feature
* smtc_modem_utilities: `smtc_modem_get_radio_context` to get previously set radio context (mandatory for sx127x support as the driver implements and uses an internal context)

## [v4.0.0] 2023-03-10

This version propose a major update of the LoRa Basics Modem.
The purpose is to have a complete and simple LoRaWAN stack implementation, including additional LoRaWAN packages (Application Layer Clock Synchronization, Fragmented Data Block Transport, Remote Multicast Setup).

### Added

* smtc_modem_utilities:
  * `smtc_modem_is_irq_flag_pending` function to allow checking if some modem flags are pending
  * `smtc_modem_set_radio_context` function to set optional radio context if needed
* `SMTC_MODEM_RC_NO_EVENT` return code in case `smtc_modem_get_event()` is called and there is no pending event
* `smtc_modem_get_downlink_data` to get received data
* `smtc_modem_get_lorawan_link_check_data` to get link check data
* `smtc_modem_debug_connect_with_abp` for debug purpose
* Fuota related events type
  * `SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE`
  * `SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C`
  * `SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B`
  * `SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C`
  * `SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B`
* Fuota related events status in `smtc_modem_event_t`:
  * `fuota_status`
  * `new_multicast_class_c`
  * `new_multicast_class_b`

### Changed

* lr11xx specific crypto features are now in main api file (`smtc_basic_modem_lr11xx_api_extension.h` removed)
* `smtc_modem_return_code_t` Modem return codes are now generic enum (no values specified)
* Events:
  * Events are now generic enum (no values specified): `smtc_modem_event_type_t`
  * Event structure `smtc_modem_event_t` does not contain downlink data anymore. Use `smtc_modem_get_downlink_data` to get received data
  * Event structure `smtc_modem_event_t` does not contain link_check data anymore. Use `smtc_modem_get_lorawan_link_check_data` to get link check data
  * In event structure: `class_b_ping_slot_info`, `lorawan_mac_time` and `link_check` share the same status type `smtc_modem_event_mac_request_status_t`
* Time services is now split in 2 standalone features:
  * LoRaWAN ALCSync package:
    * `smtc_modem_start_alcsync_service` to start the service
    * `smtc_modem_stop_alcsync_service` to stop the service
    * `smtc_modem_get_alcsync_time` to get the time from the service
    * `smtc_modem_trigger_alcsync_request` to trig a manual time request
    * New event `SMTC_MODEM_EVENT_ALCSYNC_TIME` for the service
  * LoRaWAN Device Time Request:
    * Request shall be done using `smtc_modem_trig_lorawan_mac_request`
    * `smtc_modem_get_lorawan_mac_time` to get the time from stack
    * New event `SMTC_MODEM_EVENT_LORAWAN_MAC_TIME`
* `smtc_modem_test_duty_cycle_app_activate` rename into `smtc_modem_debug_set_duty_cycle_state`
* `smtc_modem_lorawan_request_link_check` is now handled in `smtc_modem_trig_lorawan_mac_request`
* `smtc_modem_lorawan_class_b_request_ping_slot_info` is now handled in `smtc_modem_trig_lorawan_mac_request`

### Removed

* [**GENERAL**] Support of LoRaCloud services: Large File Upload (LFU), Stream, Device Management, Almanac update (All features, functions and events removed)
* All user radio direct access features, functions and events
* `smtc_modem_get_network_type` function
* `smtc_modem_suspend_radio_communications` function
* `smtc_modem_get_stack_state` function
* `smtc_modem_get_network_frame_pending_status` function
* Connection timeout features:
  * `smtc_modem_connection_timeout_set_thresholds` function
  * `smtc_modem_connection_timeout_get_current_values` function
  * Event `SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED`
* Event `SMTC_MODEM_EVENT_NEW_LINK_ADR`
* Return core `SMTC_MODEM_RC_MODEM_E_FRAME_ERROR`

## [v3.2.4] 2022-08-23

No changes

## [v3.1.7] 2022-04-13

### Added

* [class_b] `smtc_modem_lorawan_class_b_request_ping_slot_info()` function
* [class_b] `smtc_modem_class_b_set_ping_slot_periodicity()` function
* [class_b] `smtc_modem_class_b_get_ping_slot_periodicity()` function
* [multicast] `smtc_modem_multicast_class_b_start_session()` function
* [multicast] `smtc_modem_multicast_class_b_get_session_status()` function
* [multicast] `smtc_modem_multicast_class_b_stop_all_sessions()` function
* [LoRaWAN] `smtc_modem_lorawan_get_lost_connection_counter()` function

### Changed

* [multicast] `smtc_modem_multicast_start_session()` function is renamed `smtc_modem_multicast_class_c_start_session()`
* [multicast] `smtc_modem_multicast_get_session_status()` function is renamed `smtc_modem_multicast_class_c_get_session_status()`
* [multicast] `smtc_modem_multicast_stop_session()` function is renamed `smtc_modem_multicast_class_c_stop_session()`
* [multicast] `smtc_modem_multicast_stop_all_sessions()` function is renamed `smtc_modem_multicast_class_c_stop_all_sessions()`
* [time_sync] `smtc_modem_time_trigger_sync_request` function does not take `sync_service` parameter anymore and will use the current enabled time synchronization service

### Fixed

### Removed

## [v2.1.0] 2021-09-24

### Added

* [all] Initial version
