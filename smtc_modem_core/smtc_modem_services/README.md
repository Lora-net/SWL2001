# Modem Services
This module provides basic modem services to use with the Semtech Cloud.

## Overview
Service | Description
--------|-------------
alc_sync | Application Layer Clock Synchronization
file_upload | File Upload
stream | Reliable Data Stream
almanac_update | Almanac Update Service

![Modem services diagram](doc/services.png)

### Contents
- `headers/` directory contains the services public API.
- `src/` directory contains the services
- `tests/` contains the unit tests

## Requirements
### Radio
The services require a few utilities and radio API defined in file `smtc_modem_services_hal.h`

API | Description
---|---
smtc_modem_services_aes_encrypt | Encrypt a payload for LoRaWAN
smtc_modem_services_get_time_s | Return current time (in seconds)
smtc_modem_services_get_dm_upload_sctr | _DEPRECATED_
smtc_modem_services_set_dm_upload_sctr | _DEPRECATED_
smtc_modem_services_lr1110_gnss_get_context_status | call the lr1110 corresponding function that get the gnss context status
smtc_modem_services_lr1110_gnss_push_dmc_msg |  call the lr1110 corresponding function that push the DAS gnss message to the lr1110

### Logging
Modem Services require a variadic macro `LOG_PRINT( level, ... )` to be defined in `smtc_modem_services_config.h` by the user (see _Porting_).
This enables the logging system.

When the `LOG_PRINT` macro is available, the modem services log messages on 4 different levels: `LOG_ERROR`, `LOG_WARN`, `LOG_INFO` and `LOG_DEBUG`.

These 4 levels are enabled by default with the following definitions:
```C
#define LOG_ERROR( ... )	LOG_PRINT( "ERROR", __VA_ARGS__ )
#define LOG_WARN( ... )		LOG_PRINT( "WARNING", __VA_ARGS__ )
#define LOG_INFO( ... )		LOG_PRINT( "INFO", __VA_ARGS__ )
#define LOG_DEBUG( ... )	LOG_PRINT( "DEBUG", __VA_ARGS__ )
```

If the level macros are already defined by the user, the modem services use them instead. 

It is possible to disable a level by declaring an empty macro, as follows:
```C
#define LOG_DEBUG( ...)    // Empty to disable
```

## Porting
1. Include `modem_services` to your project
2. Copy the file `smtc_modem_services_config.template.h` as `smtc_modem_services_config.h` in your project and edit as necessary.
    It should contain the definition of logging macro `LOG_PRINT(level, ...)` to enable logging in the modem services.
3. Implement the API required by `smtc_modem_services_hal.h`
4. Add the source files to you build system. The `module.mk` file exports the following variables for `make`:
    - `MODULE_C_SOURCES` lists all C files
    - `MODULE_C_INCLUDES` lists header files locations

 
## Unit Tests
Unit tests run on a host computer and require a native GCC and test runner.
### Requirements
- GCC or similar:  
     `apt install gcc`
- [Ceedling](http://www.throwtheswitch.org/) test runner: 
    ```bash
    apt install ruby
    gem install ceedling
    ```
- Gcovr:  
    `apt install gcovr`

### Running
Unit tests live in the `tests` directory. Invoke Ceedling from this directory to run the tests.

|Command | Description
|---|---
|`ceedling test:all` | run all tests
|`ceedling clobber` | Clean up all generated files
|`ceedling gcov:all utils:gcov` | Run tests with code coverage and generate report
