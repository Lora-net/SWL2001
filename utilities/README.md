# LoRa Basics Modem utilities

This folder contains a implementation of LoRa Basics Modem on a specific MCU board and some application examples.

## Hardware

### MCU Board

Nucleo L476RG board ( with stm32l476 MCU)

### Radio Boards

Semtech chosen radio board:

- lr1110 (EVK board)
- lr1120 (EVK board)
- sx1261 (SX1261MB2xAS)
- sx1262 (SX1262MB2xAS)
- sx1268 (SX1262MB2xAS)
- sx128x (EVK board)
- sx1272 (SX1272MB2DAS)
- sx1276 (SX1276MB1MAS)

## Getting Started

Please use `make help` to see all build options.

Compilation options can be provided within command line or directly in the makefile option file that can be found in [app_options.mk](utilities/app_makefiles/app_options.mk)

### Main Examples

#### Example EXTI

This simple example joins LoRaWAN network and then send uplinks periodically or when Nucleo blue button is pushed

LoRaWAN credentials shall be provided in [example_options.h](utilities/user_app/main_examples/example_options.h)

Build command example for lr1110 radio

```bash
make lr1110 MODEM_APP=EXAMPLE_EXTI
```

#### Example Hardware Modem

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
make lr1110 MODEM_APP=EXAMPLE_PORTING_TESTS
```

### MCU Porting

 All MCU specific code can be found under 2 folders:

- [mcu_drivers](ser_app/mcu_drivers): contains ST Microelectronics HAL and ARM CMSIS lib
- [smtc_hal_l4](user_app/smtc_hal_l4): contains the helper functions that use ST HAL lib and that will be needed by modem hal (ex: RTC, GPIO, SPI, UART, WDOG, LPTIM...).

## Radio Porting folder

Radio HAL and BSP implementation are done in [radio_hal](utilities/user_app/radio_hal) folder

## Modem Porting folder

Modem HAL implementation is done in [smtc_modem_hal.c](utilities/user_app/smtc_modem_hal/smtc_modem_hal.c).
Any smtc_modem_hal function will be mapped with corresponding mcu porting function.

## Fuota support

Once the Fuota services are enabled (refer to the main readme for instructions on how to activate Fuota), the exti example is sufficient to launch a Fuota campaign. However, you will need to enable the flag `ALLOW_FUOTA` to "yes" in the `app_options.mk` file.
