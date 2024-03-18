# Implementation reference 2: Porting on Nordic NRF52840 Board

This folder contains implementations of LoRa Basics Modem on some MCU boards and some application examples.

## Hardware

### MCU Board

- PCA10056 board ( with NRF52840 MCU)

### Radio Boards

Semtech chosen radio board:

- lr1110 (EVK board)
- lr1120 (EVK board)
- lr1121 (EVK board)
- sx1261 (SX1261MB2xAS)
- sx1262 (SX1262MB2xAS)
- sx1268 (SX1262MB2xAS)

## Getting Started

Please use `make help` to see all build options.

Compilation options can be provided within command line or directly in the makefile option file that can be found in [app_options.mk](app_makefiles/app_options.mk)

### Main Examples

#### Periodical Uplink

This simple example joins LoRaWAN network and then send uplinks periodically or when Nucleo blue button is pushed

LoRaWAN credentials shall be provided in [example_options.h](main_examples/example_options.h)

Build command example for lr1110 radio

```bash
make lr1110 MODEM_APP=PERIODICAL_UPLINK
```

#### Hardware Modem

This example proposes an implementation of a all integrated modem that can be addressed using UART RX/TX and 3 gpios for commands handling (command and busy) and events notification (event)
All functions included in lora basics modem api can be called using commands.

Build command example for lr1110 radio

```bash
make lr1110 MODEM_APP=HW_MODEM
```

#### Porting tool

This tool provides a automatic suite of tests that will help user ensures that lora basics modem mcu and radio HAL functions are implemented in a good way (SPI, radio_irq, time, timer, random, radio config, sleep and low power).

Build command example for lr1110 radio

```bash
make lr1110 MODEM_APP=PORTING_TESTS
```

#### LCTT Certification

This example provides an application that can be used to run the LCTT certification tool.  
Pushing Blue button will allow the modem to go in certification mode (ie it will accept communication on port 224)  
LoRaWAN credentials and region shall be provided in [example_options.h](main_examples/example_options.h)  

Build command example for lr1110 radio

```bash
make lr1110 MODEM_APP=LCTT_CERTIF
```

### MCU Porting

 All MCU specific code can be found under following folders:

- [mcu_drivers](mcu_drivers): contains NRF SDK code
- [smtc_hal_nrf52840](smtc_hal_nrf52840): contains the helper functions for NRF52840 that use ST HAL lib and that will be needed by modem hal (ex: RTC, GPIO, SPI, UART, WDOG...).

## Radio Porting folder

Radio HAL and BSP implementation are done in [radio_hal](radio_hal) folder

## Modem Porting folder

Modem HAL implementation is done in [smtc_modem_hal.c](smtc_modem_hal/smtc_modem_hal.c).
Any smtc_modem_hal function will be mapped with corresponding mcu porting function.

## Fuota support

Once the Fuota services are enabled (refer to the main readme for instructions on how to activate Fuota), the periodical example is sufficient to launch a Fuota campaign. However, you will need to enable the flag `ALLOW_FUOTA` to "yes" in the `app_options.mk` file.
