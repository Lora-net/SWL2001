# LoRa Basics Modem

## LoRaWAN parameters

### LoRaWAN version

The LoRaWAN version that is currently implemented in LoRa Basics Modem is v1.0.4.

### LoRaWAN region

LoRa Basics Modem supports the following LoRaWAN regions:

* EU868
* US915
* CN470_RP_1_0

### LoRaWAN class

LoRa Basics Modem supports the following LoRaWAN classes:

* Class A
* Class C

## LoRa Basics Modem services

LoRa Basics Modem supports the following services:

* Large files upload
* ROSE Streaming
* Application-Layer Clock synchronization
* Almanac Update

## LoRa Basics Modem API

The Application Programming Interface of LoRa Basics Modem is defined in `smtc_modem_api/smtc_modem_api.h` header file.

## LoRa Basics Modem engine

LoRa Basics Modem has to be initialized first by calling `smtc_modem_init()`. Then, calling periodically `smtc_modem_run_engine()` is required to make the state machine move forward.

These functions can be found in `smtc_modem_api/smtc_modem_utilities.h`

## LoRa Basics Modem HAL

The Hardware Abstraction Layer of LoRa Basics Modem is defined in the `smtc_modem_hal/smtc_modem_hal.h` header file. Porting LoRa Basics Modem to a new architecture requires one to implement the functions described by the prototypes in it.

## Transceiver

LoRa Basics Modem supports the following transceivers:

* LR1110 with firmware 0x0307.

## Disclaimer

This software has been extensively tested when targeting LR1110 for the EU868, US915, and CN470_RP_1_0 LoRaWAN regions. For all other combinations of features this software shall be considered an Engineering Sample.

All customers wanting to leverage LoRa Basics Modem for 2.4GHz running with SX1280 transceiver must still refer to the [release v1.0.1](https://github.com/lorabasics/lorabasicsmodem/releases/tag/v1.0.1) for which Semtech provides technical customer support.

### Disclaimer for Engineering Samples

Information relating to this product and the application or design described herein is believed to be reliable, however such information is provided as a guide only and Semtech assumes no liability for any errors related to the product, documentation, or for the application or design described herein. Semtech reserves the right to make changes to the product or this document at any time without notice.
