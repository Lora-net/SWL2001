# Lora Basics Modem HAL changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.1.0] 2023-07-07

### Added

* Functions needed by Firmware Management Package (TS006-1.0.0)
  * `smtc_modem_hal_get_hw_version_for_fuota`
  * `smtc_modem_hal_get_fw_version_for_fuota`
  * `smtc_modem_hal_get_fw_status_available_for_fuota`
  * `smtc_modem_hal_get_fw_delete_status_for_fuota`
  * `smtc_modem_hal_get_next_fw_version_for_fuota`

* `smtc_modem_hal_set_ant_switch`: to management radio antenna switch if needed (mandatory for sx127x radios)

## [v4.0.0] 2023-03-10

### Added

* `SMTC_MODEM_HAL_PANIC` macro for any internal modem panic situation
* `SMTC_MODEM_HAL_PANIC_ON_FAILURE` macro for any internal modem panic on expression check
* [context] New context type `CONTEXT_FUOTA` for fuota in `modem_context_type_t`

### Changed

* `smtc_modem_hal_assert_fail` has been replaced by `smtc_modem_hal_on_panic` function
* [context]
  * In `modem_context_type_t` `CONTEXT_LR1MAC` rename in `CONTEXT_LORAWAN_STACK`
  * add `offset` argument for `smtc_modem_hal_context_restore`
  * add `offset` argument for `smtc_modem_hal_context_store`

### Removed

* `smtc_modem_hal_get_temperature` function
* `smtc_modem_hal_get_voltage` function
* `smtc_modem_hal_get_compensated_time_in_s` function
* `smtc_modem_hal_get_time_compensation_in_s` function
* `smtc_modem_hal_store_crashlog` function
* `smtc_modem_hal_restore_crashlog` function
* `smtc_modem_hal_set_crashlog_status` function
* `smtc_modem_hal_get_crashlog_status` function
* `smtc_modem_hal_get_random_nb` function
* `smtc_modem_hal_get_signed_random_nb_in_range` function
* `smtc_modem_hal_mcu_panic` macro
* `smtc_modem_hal_lr1mac_panic` macro
* `smtc_modem_hal_get_radio_irq_timestamp_in_100us` function
* [context] `CONTEXT_DEVNONCE` is not used anymore in `modem_context_type_t` (handled in `CONTEXT_LORAWAN_STACK` )

## [v3.2.4]

### Added

* [assert] `smtc_modem_hal_assert(expr)` macro
* [assert] `smtc_modem_hal_assert_fail()` function
* [time] `smtc_modem_hal_get_time_in_100us()` function
* [radio_irq] `smtc_modem_hal_get_radio_irq_timestamp_in_100us()` function

### Changed

* [radio_irq] `smtc_modem_hal_irq_is_radio_irq_pending()` function has been replaced with `smtc_modem_hal_radio_irq_clear_pending()`. Now modem only asks to clear radio pending irq

### Fixed

### Removed

## [v2.1.0] 2021-09-24

### Added

* [all] Initial version
