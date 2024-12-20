# Lora Basics Modem HAL changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v4.8.0] 2024-12-20

### Changed

* [radio_irq] `smtc_modem_hal_radio_irq_clear_pending()` function has been removed

## [v4.3.0] 2023-12-15

### Added

* Add `CONTEXT_KEY_MODEM` to handle specific key context in case lr11xx hardware crypto element is used and that will prevent multiple re-write of the key
* Add `CONTEXT_STORE_AND_FORWARD` to handle specific store and forward nvm access
* Add `smtc_modem_hal_context_flash_pages_erase` function prototype to erase a chosen flash page (only used for store and forward service)
* Add store and forward related functions, `smtc_modem_hal_store_and_forward_get_number_of_pages` and `smtc_modem_hal_flash_get_page_size`
* Re-add crashlog management functions `smtc_modem_hal_crashlog_store`, `smtc_modem_hal_crashlog_restore`, `smtc_modem_hal_crashlog_set_status`, `smtc_modem_hal_crashlog_get_status`
* Re-add Device Management related functions `smtc_modem_hal_get_temperature` and `smtc_modem_hal_get_voltage_mv`
* Add `smtc_modem_hal_user_lbm_irq` function for Real Time OS compatibility

### Changed

* `smtc_modem_hal_on_panic` prototype changed to add variadic arguments
* `SMTC_MODEM_HAL_PANIC` macro implementation does not handle the error trace print directly and relay on `smtc_modem_hal_on_panic` to handle it
* `SMTC_MODEM_HAL_PANIC_ON_FAILURE` macro implementation forwards now the failed expression string to `smtc_modem_hal_on_panic` function
* `CONTEXT_MODEM` is not used anymore to store key crc in case lr11xx crypto element is used

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
