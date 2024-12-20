# LoRa Basics Modem examples

This folder contains implementations of LoRa Basics Modem on some MCU boards and some application examples.

## Hardware

### MCU Board

- Nucleo L476RG board ( with STM32L476RGT6 MCU)
- Nucleo L073RZ board ( with STM32L073RZ MCU)

### Radio Boards

Semtech chosen radio board:

- lr1110 (EVK board)
- lr1120 (EVK board)
- lr1121 (EVK board)
- sx1261 (SX1261MB2xAS)
- sx1262 (SX1262MB2xAS)

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

### Relay

#### Relay Tx

This example provides an application where the Relay Tx feature is enabled.

This example uses the Periodical Uplink main example.

The end-device is configured in **ENABLE** mode (refer to TS011-1.0.1 table 40). After the reset, the end-device will enable the Relay Tx feature and add a WOR frame before every LoRaWAN uplink. The relay feature will be automatically disable if the end-device receive a downlink on Rx1 or Rx2.

In this example, the CSMA is compiled and enabled by default.

LoRaWAN credentials shall be provided in [example_options.h](main_examples/example_options.h)

Build command example for lr1110 radio

```bash

make full_lr1110 MODEM_APP=PERIODICAL_UPLINK ALLOW_RELAY_TX=yes
```

Or to be tested with the LCTT

```bash

make full_lr1110 MODEM_APP=LCTT_CERTIF ALLOW_RELAY_TX=yes
```

#### Relay Rx

This example provides an application where the relay Rx feature is enabled.

This example uses the Periodical Uplink main example.

In this example, the CSMA is compiled and enabled by default.

LoRaWAN credentials shall be provided in [example_options.h](main_examples/example_options.h)

Build command example for lr1110 radio

```bash

make full_lr1110 MODEM_APP=PERIODICAL_UPLINK ALLOW_RELAY_RX=yes
```

Or to be tested with the LCTT

```bash

make full_lr1110 MODEM_APP=LCTT_CERTIF ALLOW_RELAY_RX=yes
```

### MCU Porting

 All MCU specific code can be found under following folders:

- [mcu_drivers](mcu_drivers): contains ST Microelectronics HAL and ARM CMSIS lib
- [smtc_hal_l4](smtc_hal_l4): contains the helper functions for STM32L4 that use ST HAL lib and that will be needed by modem hal (ex: RTC, GPIO, SPI, UART, WDOG, LPTIM...).
- [smtc_hal_l0](smtc_hal_l0_LL): contains the helper functions for STM32L0 that use ST HAL lib and that will be needed by modem hal (ex: RTC, GPIO, SPI, UART, WDOG, LPTIM...).

## Radio Porting folder

Radio HAL and BSP implementation are done in [radio_hal](radio_hal) folder

## Modem Porting folder

Modem HAL implementation is done in [smtc_modem_hal.c](smtc_modem_hal/smtc_modem_hal.c).
Any smtc_modem_hal function will be mapped with corresponding mcu porting function.

## Fuota support

Once the Fuota services are enabled (refer to the main readme for instructions on how to activate Fuota), the periodical example is sufficient to launch a Fuota campaign. However, you will need to enable the flag `ALLOW_FUOTA` to "yes" in the `app_options.mk` file.
