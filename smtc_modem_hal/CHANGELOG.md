# Lora Basics Modem HAL changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v3.0.0] Unreleased

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
