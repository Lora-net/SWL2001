# SX128X driver

This package proposes an implementation in C of the driver for **SX128X** radio component.
Please see the [changelog](CHANGELOG.md) for more information.

## Structure

The driver is defined as follows:

- sx128x.c: implementation of the driver functions
- sx128x.h: declarations of the driver functions
- sx128x_regs.h: definitions of all useful registers (address and fields)
- sx128x_hal.h: declarations of the HAL functions (to be implemented by the user - see below)

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions the user shall implement to write platform-dependant calls to the host. The list of functions is the following:

- sx128x_hal_reset
- sx128x_hal_wakeup
- sx128x_hal_write
- sx128x_hal_read
