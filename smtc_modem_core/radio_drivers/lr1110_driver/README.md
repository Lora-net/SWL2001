# LR1110_driver project

This package proposes an implementation in C of the driver for **LR1110** radio component.

## Components

The driver is split in several components:

- Bootloader
- Register / memory access
- System configuration
- Radio
- Wi-Fi Passive Scanning
- GNSS Scan Scanning
- Crypto engine

### Bootloader

This component is used to update the firmware.

### Register / memory access

This component is used to read / write data from registers or internal memory.

### System configuration

This component is used to interact with system-wide parameters like clock sources, integrated RF switches, etc.

### Radio

This component is used to send / receive data through the different modems (LoRa and GFSK) or perform a LoRa CAD (Channel Activity Detection). Parameters like power amplifier selection, output power and fallback modes are also accessible through this component.

### Wi-Fi Passive Scanning

This component is used to configure and initiate the passive scanning of the Wi-Fi signals that can be shared to request a geolocation.

### GNSS Scanning

This component is used to configure and initiate the acquisition of GNSS signals that can be shared to request a geolocation.

### Crypto engine

This component is used to set and derive keys in the internal keychain and perform cryptographic operations with the integrated hardware accelerator.

## Structure

Each component is based on different files:

- lr1110_component.c: implementation of the functions related to component
- lr1110_component.h: declarations of the functions related to component
- lr1110_component_types.h: type definitions related to components

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions that the user shall implement to write platform-dependant calls to the host. The list of functions is the following:

- lr1110_hal_reset()
- lr1110_hal_wakeup()
- lr1110_hal_write()
- lr1110_hal_read()
- lr1110_hal_direct_read()
