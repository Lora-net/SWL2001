# SX126X driver

This package proposes an implementation in C of the driver for **SX126X** radio component.

## Structure

The driver is defined as follows:

- sx126x.c: implementation of the driver functions
- sx126x.h: declarations of the driver functions
- sx126x_regs.h: definitions of all useful registers (address and fields)
- sx126x_hal.h: declarations of the HAL functions (to be implemented by the user - see below)

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions the user shall implement to write platform-dependant calls to the host. The list of functions is the following:

- sx126x_hal_reset
- sx126x_hal_wakeup
- sx126x_hal_write
- sx126x_hal_read
