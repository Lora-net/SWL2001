# Lora Basics Modem API changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v3.0.0] Unreleased

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
