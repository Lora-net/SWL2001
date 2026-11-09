# SX126X driver

This package proposes an implementation in C of the driver for **SX126X** radio component.
Please see the [changelog](CHANGELOG.md) for more information.

## Structure

The driver is defined as follows:

- sx126x.c: implementation of the driver functions
- sx126x.h: declarations of the driver functions
- sx126x_regs.h: definitions of all useful registers (address and fields)
- sx126x_hal.h: declarations of the HAL functions (to be implemented by the user - see below)
- lr_fhss_mac.c: Transceiver-independent LR-FHSS implementation
- sx126x_lr_fhss.c: Transceiver-dependent LR-FHSS implementation
- lr_fhss_mac.h: Transceiver-independent LR-FHSS declarations
- sx126x_lr_fhss.h: Transceiver-dependent LR-FHSS declarations
- lr_fhss_v1_base_types.h: LR-FHSS type interface
- sx126x_bpsk.c: implementation of BPSK driver functions
- sx126x_bpsk.h: declaration of BPSK driver functions

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions the user shall implement to write platform-dependant calls to the host. The list of functions is the following:

- sx126x_hal_reset
- sx126x_hal_wakeup
- sx126x_hal_write
- sx126x_hal_read

## Cmake usage

This driver exposes a cmake configuration allowing to integrate the driver in a cmake ready application.

### Integration

If the driver code resides in a directory of the application using it, it can be integrated by adding the subdirectory to the configuration as follows:

```cmake
add_subdirectory(sx126x_driver) # where sx126x_driver is the name of the folder containing the driver code
```

Alternatively, if the driver code is available through a code archive, it can be included directly by

```cmake
include(FetchContent)
FetchContent_Declare(
    sx126x_driver
    URL "path_to_archive" # Where path_to_archive is to be replaced by the path to the archive driver
)
FetchContent_MakeAvailable(sx126x_driver)
```

### Configuration

By default, LR-FHSS and BPSK are not included in the build of the driver. It can be added by adding the following line in the before including the driver code in the cmake suite:

```cmake
set(SX126X_ENABLE_BPSK ON CACHE BOOL "") # To enable BPSK
set(SX126X_ENABLE_LR_FHSS ON CACHE BOOL "") # To enable LR-FHSS
```
