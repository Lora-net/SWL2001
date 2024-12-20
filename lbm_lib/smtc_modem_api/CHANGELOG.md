# Lora Basics Modem API changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.8.0] 2024-12-20

### Added

* Add relay rx event
  * `SMTC_MODEM_EVENT_RELAY_RX_RUNNING` : Relay Rx enabled by network server
* Add test mode event
  * `SMTC_MODEM_EVENT_TEST_MODE`: to be inform of the test mode status
* Add duty-cycle event
  * `SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE`: when the regional duty-cycle constraint is reached or released
* For FUOTA with R11XX Crypto Engine
  * `smtc_modem_get_data_block_int_key()` Get Fragmented DataBlockIntKey
  * `smtc_modem_derive_and_set_data_block_int_key()` Derive and set DataBlockIntKey

### Changed

* `smtc_modem_api.h`
  * function `smtc_modem_csma_set_parameters()` reverse some parameters to be consistent
  * function `smtc_modem_get_rp_stats_to_array()` update output parameter
* `smtc_modem_test_api.h` globally reworked to be more generic and use smtc_ral value

## [v4.5.0] 2024-05-06

### Added

* All events and API defined in the v4.4.0

## [v4.4.0] 2023-12-21

### Added

* Add `smtc_modem_relay_tx_disable` function to disable the relay tx feature (disable the WOR frame)
* Add `smtc_modem_relay_tx_enable` function to enable the relay tx feature (enable the WOR frame)
* Add `smtc_modem_relay_tx_get_activation_mode` function to get the current relay tx mode activation
* Add `smtc_modem_relay_tx_get_sync_status` function to get the current synchronization status with a relay rx
* Add `smtc_modem_relay_tx_is_enable` function to get if the relay tx feature is currently enable
* Add relay tx event
  * `SMTC_MODEM_EVENT_RELAY_TX_DYNAMIC` : Relay TX dynamic mode has enable or disable the WOR protocol
  * `SMTC_MODEM_EVENT_RELAY_TX_MODE` : Relay TX activation has been updated via a MAC command
  * `SMTC_MODEM_EVENT_RELAY_TX_SYNC` : Relay TX synchronization has changed

## [v4.3.0] 2023-12-15

### Added

* `smtc_modem_set_appkey` function to set the LoRaWAN v1.0.x Gen Application Key
* Re-add `smtc_modem_get_status` to get the modem current status (see also `smtc_modem_status_mask_t` definition)
* Add `smtc_modem_get_certification_mode` to get the current certification mode status
* Add `smtc_modem_adr_get_profile` to get current Datarate Profile of the modem
* Add or re-add functions related to LoRaCloud:
  * Almanac update service:
    * function `smtc_modem_almanac_start` (new)
    * function `smtc_modem_almanac_stop` (new)
  * Stream service:
    * Re-add `smtc_modem_stream_init`
    * Re-add `smtc_modem_stream_add_data`
    * Re-add `smtc_modem_stream_status`
    * Re-add related event `SMTC_MODEM_EVENT_STREAM_DONE`
  * Large File Update service:
    * Re-add `smtc_modem_file_upload_init`
    * Re-add `smtc_modem_file_upload_start`
    * Re-add `smtc_modem_file_upload_reset`
    * Re-add related event `SMTC_MODEM_EVENT_UPLOAD_DONE`
  * Device Management service:
    * `smtc_modem_dm_enable` (new)
    * `smtc_modem_dm_get_fport`
    * `smtc_modem_dm_set_fport`
    * `smtc_modem_dm_get_info_interval`
    * `smtc_modem_dm_set_info_interval`
    * `smtc_modem_dm_get_periodic_info_fields` (previously known as `smtc_modem_dm_get_info_fields`)
    * `smtc_modem_dm_set_periodic_info_fields` (previously known as `smtc_modem_dm_set_info_fields`)
    * `smtc_modem_dm_request_immediate_info_field` (previously known as `smtc_modem_dm_request_single_uplink`)
    * `smtc_modem_dm_set_user_data`
    * `smtc_modem_dm_get_user_data`
    * `smtc_modem_dm_handle_alcsync` (new)
    * Re-add related events `SMTC_MODEM_EVENT_DM_SET_CONF` and `SMTC_MODEM_EVENT_MUTE`
* Add Store and Forward service:
  * `smtc_modem_store_and_forward_set_state`
  * `smtc_modem_store_and_forward_get_state`
  * `smtc_modem_store_and_forward_flash_add_data`
  * `smtc_modem_store_and_forward_flash_clear_data`
  * `smtc_modem_store_and_forward_flash_get_number_of_free_slot`
* Add new API include file `smtc_modem_geolocation_api.h` got geolocation services.
* Add geolocation services (GNSS, Wi-Fi, Almanac demodulation):
  * `smtc_modem_gnss_scan`
  * `smtc_modem_gnss_scan_cancel`
  * `smtc_modem_gnss_get_event_data_scan_done`
  * `smtc_modem_gnss_get_event_data_terminated`
  * `smtc_modem_gnss_set_constellations`
  * `smtc_modem_gnss_set_port`
  * `smtc_modem_gnss_scan_aggregate`
  * `smtc_modem_gnss_send_mode`
  * `smtc_modem_almanac_demodulation_start`
  * `smtc_modem_almanac_demodulation_set_constellations`
  * `smtc_modem_almanac_demodulation_get_event_data_almanac_update`
  * `smtc_modem_wifi_scan`
  * `smtc_modem_wifi_scan_cancel`
  * `smtc_modem_wifi_get_event_data_scan_done`
  * `smtc_modem_wifi_get_event_data_terminated`
  * `smtc_modem_wifi_set_port`
  * `smtc_modem_wifi_send_mode`
  * `smtc_modem_wifi_set_payload_format`
* Add geolocation events `SMTC_MODEM_EVENT_GNSS_SCAN_DONE`, `SMTC_MODEM_EVENT_GNSS_TERMINATED`, `SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE`, `SMTC_MODEM_EVENT_WIFI_SCAN_DONE` and `SMTC_MODEM_EVENT_WIFI_TERMINATED`

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
