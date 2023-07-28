# LoRa Basic Modem

**LoRa Basic Modem** proposes an implementation of the [TS001-LoRaWAN L2 1.0.4](https://resources.lora-alliance.org/technical-specifications/ts001-1-0-4-lorawan-l2-1-0-4-specification) and [Regional Parameters RP2-1.0.3](https://resources.lora-alliance.org/technical-specifications/rp2-1-0-3-lorawan-regional-parameters) specifications.

The proposed stack implementation:

- Supports Class A operation
- Supports Class B operation (with up to 4 multicast sessions)
- Supports Class C operation (with up to 4 multicast sessions)
- Supports Relay operation as relayed end-device or relay itself (refer to [Relay](#relay) )
- Supports following regions:

  - AS_923 (AS923-1, AS923-2, AS923-3, AS923-4)
  - AU_915
  - CN_470
  - CN_470_RP_1_0
  - EU_868
  - IN_865
  - KR_920
  - RU_864
  - US_915

  **Note**: In addition the proposed implementation also provides a 2.4 GHz global ISM band (WW2G4) region support.

- Supports Semtech radios:
  - LR1110 with firmware 0x0308.
  - LR1120 with firmware 0x0102
  - SX1261
  - SX1262
  - SX1280
  - SX1281
  - SX1272 (experimental)
  - SX1276 (experimental)

- Provides the support for software emulated and hardware secure-elements.
  Such secure-elements provide all required cryptographic operations necessary to operate the LoRaWAN protocol.
- Provides an MCU abstraction layer to ease the porting process.
  The following lists the required MCU hardware resources:
  - 1 HW timer
  - 1 SPI interface
  - 40KB of FLASH memory minimum allowing to run an end-device in Class A mode for a single region and each additional region adds 1.5KB.
  - 6KB of RAM memory minimum.
  - 36B of non-volatile memory to store LoRaWAN activation session context.
    Up to 512B of non-volatile memory if software emulated secure-element is used.
  - 1 Watchdog (Optional)
- Supports OTAA activation by default
- ABP activation is implemented only for debugging purposes.
  **Note:** No stack context is saved in non-volatile memory. Therefore, it is strongly discouraged to use this mode for any other purpose than debugging.
- Bare Metal or RTOS based projects porting options.

## Examples

Under `utilities` folder, one will find a few examples on how to use the LoRa Basics Modem stack.

- Hardware Modem (Implements a hardware modem controlled by a serial interface)
- Exti (joins the network and then sends uplinks each time the button is pushed)
- Porting tests (Allows to verify if the project porting process is correct)

The examples are targeted for the NucleoL476 kit featuring an STM32L476 micro-controller.
For further details please refer to `utilities` directory [README](utilities/README.md) file.

To build the Exti example targeting the LR1110 Semtech radio the following should be executed on the command line:

```bash
cd utilities
make full_lr1110 MODEM_APP=EXAMPLE_EXTI
```

## SX1272/SX1276 (experimental - read disclaimer note at the bottom)

This version of Lora Basics Modem provides only an experimental support of sx1272 and sx1276 radios.  
The driver provided is not yet fully aligned with the required architecture of LBM V4 because it performs radio access in an interrupt context (this is no longer done on other radios).  
It also requires the use of an additional low-power timer to handle some reception timeout, especially for gfsk reception.  
In provided example on STM32L4 mcu (under utilities folder), the LPTIM2 is used with the disadvantage of preventing the use of STOP2 low power mode.  

## Build process

The build process using makefile allows flexibility on the choice of different compile time options.
To know all the available make process options run:

```bash
make help
```

The build process options can be provided either using the command line or by editing [options.mk](makefiles/options.mk)

## User application

The stack must be first initialized by calling the `smtc_modem_init()` API.  
Once initialized the `smtc_modem_run_engine()` API has to be periodically called to make the stack internal state machine move forward.  
The `smtc_modem_run_engine()` API returned value is an indication of when the next task will be handled.  
Each time a stack related interruption happens, the `smtc_modem_run_engine()` API shall be called to process it and take the required actions.  
Special care must take place before entering a low power wait mode. The application developer must ensure that no stack pending actions exist by calling `smtc_modem_is_irq_flag_pending()` API.  

These API definitions can be found in [smtc_modem_utilities.h](smtc_modem_api/smtc_modem_utilities.h)

## Modem API

The Application Programming Interface is defined in [smtc_modem_api.h](smtc_modem_api/smtc_modem_api.h) file.

## Modem HAL

The Hardware Abstraction Layer is defined in [smtc_modem_hal.h](smtc_modem_hal/smtc_modem_hal.h) file.

Porting LoRa Basics Modem to a new architecture/platform requires to implement the functions defined under [smtc_modem_hal.h](smtc_modem_hal/smtc_modem_hal.h) file.

## Radio BSP (Board Specific Package) and HAL

Semtech radio drivers and Radio Abstraction Layer (RAL) architecture requires the user to implement the radio HAL and BSP functions.

Radio BSP functions are defined for each transceiver and can be found in [smtc_modem_core/smtc_ral/src](smtc_modem_core/smtc_ral/src) folder. (Files named `ral_xxx_bsp.h`).

Radio HAL functions are defined for each transceiver in [smtc_modem_core/radio_drivers](smtc_modem_core/radio_drivers) folder.

## Provided Features

### LoRaWAN Certification Protocol handling

The LoRaWAN Certification package is used when  going through the LoRaWAN certification process.

1. To use it, you have to set the LoRaWAN keys:

```c
smtc_modem_return_code_t smtc_modem_set_deveui( uint8_t stack_id, const uint8_t deveui[SMTC_MODEM_EUI_LENGTH] );
smtc_modem_return_code_t smtc_modem_set_nwkkey( uint8_t stack_id, const uint8_t nwkkey[SMTC_MODEM_KEY_LENGTH] );
smtc_modem_return_code_t smtc_modem_set_joineui( uint8_t stack_id, const uint8_t joineui[SMTC_MODEM_EUI_LENGTH] );
```

2. Connect to the network:

```c
smtc_modem_return_code_t smtc_modem_join_network( uint8_t stack_id );
```

3. Launch the certification service:  This command save in nvm a flag to allow the certification to reset the mcu when required and continue the certification process at boot.

```c
smtc_modem_return_code_t smtc_modem_set_certification_mode( uint8_t stack_id, bool enable );
```

### Application Layer Clock Synchronization (ALCSync)

This application may be used when a Network Server does not support the DeviveTimeReq commands.  
When started the application will trig a time synchronization to the application server every `periodicity_s` configured by the ALCSync server package.  
The default value is 36h.  
Each time a valid time sync answer is received by the modem, it will generate an event of type `SMTC_MODEM_EVENT_ALCSYNC_TIME`

#### Makefile option

To support this feature in the modem the file `options.mk` must contains the LBM_ALC_SYNC flag to yes: `LBM_ALC_SYNC ?= yes`

#### C Example

- Start the service:

```c
smtc_modem_return_code_t smtc_modem_start_alcsync_service( uint8_t stack_id );
```

- Stop the service:

```c
smtc_modem_return_code_t smtc_modem_stop_alcsync_service( uint8_t stack_id );
```

- Get GPS epoch time, number of seconds elapsed since GPS epoch (00:00:00, Sunday 6th of January 1980).

```c
smtc_modem_return_code_t smtc_modem_get_alcsync_time( uint8_t stack_id, uint32_t* gps_time_s );
```

- Trigger a single uplink requesting time on Application Layer Clock Synchronization (ALCSync) service, the service must be start first

```c
smtc_modem_return_code_t smtc_modem_trigger_alcsync_request( uint8_t stack_id );
```

### Class B Support in LoRa Basic Modem

LoRa Basic Modem does not support Class B by default. To use this feature, it must be activated during project compilation.

#### Activating Class B Support

To activate Class B support, add the Class B service to the compilation by setting the flag `LBM_CLASS_B` to `yes` in the `options.mk` file. This will enable the Class B functionality in the LoRa Basic Modem.

#### Starting Class B

Class B starts after a successful join, so it is recommended to start Class B after receiving the `"SMTC_MODEM_EVENT_JOINED"` event. To start Class B, call the following function available in the LoRa Basic Modem API: `smtc_modem_set_class` with the parameter `SMTC_MODEM_CLASS_B`.

LoRa Basic Modem autonomously handles all the prerequisites inherent in Class B in LoRaWAN (ping slot info request, time search, acquisition of the first beacon, sending an uplink to signal the network about the activation of Class B). Once Class B is active, the user is informed via the `"SMTC_MODEM_EVENT_CLASS_B_STATUS"` event with the state `"SMTC_MODEM_EVENT_CLASS_B_READY"`.

To change the ping slot periodicity (default value is 7, which means a ping slot every 128 seconds), call the function `smtc_modem_class_b_set_ping_slot_periodicity` before calling `smtc_modem_set_class`.

#### Special Cases

- If your network server does not support time synchronization via a network command, you must activate the clk sync package to set the GPS time before starting Class B (refer to the clk sync package chapter in this readme).
- In case you are no longer connected to the LoRaWAN network, Class B will automatically stop after 2 hours without receiving a beacon. The user will then receive the  `"SMTC_MODEM_EVENT_CLASS_B_STATUS"` event with the state `"SMTC_MODEM_EVENT_CLASS_B_NOT_READY"`.

## FUOTA (experimental - read disclaimer note at the bottom)

The FUOTA service available in LoRa Basic Modem corresponds to the firmware update service described in the standard defined by the LoRa Alliance. Among the following 5 Applicative Packages described by the LoRaWan standard:

- <https://resources.lora-alliance.org/technical-specifications/lorawan-remote-multicast-setup-specification-v1-0-0>
- <https://resources.lora-alliance.org/technical-specifications/lorawan-fragmented-data-block-transport-specification-v1-0-0>
- <https://resources.lora-alliance.org/technical-specifications/lorawan-application-layer-clock-synchronization-specification-v1-0-0>
- <https://resources.lora-alliance.org/technical-specifications/ts006-1-0-0-firmware-management-protocol>
- <https://resources.lora-alliance.org/technical-specifications/ts007-1-0-0-multi-package-access>

The proposed implementation supports both Class B or Class C modes and supports up to 4 simultaneous multicast sessions.

### FUOTA Activation

FUOTA is not supported in the default mode of LoRa Basic Modem, this functionality must be activated during the project compilation.

To activate FUOTA: add the FUOTA service to the compilation by setting the `LBM_FUOTA` flag to "yes" , `LBM_FUOTA_VERSION` flag to "1" in the `options.mk` file.

You will also need to provide the following fields at compilation:

- `FUOTA_MAXIMUM_NB_OF_FRAGMENTS`
- `FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS`
- `FUOTA_MAXIMUM_FRAG_REDUNDANCY`

Class B, Class C, multicast, and the previously mentioned packages are automatically built by activating this compilation flag.

#### Prerequisites before starting a FUOTA session

The user must implement the following two functions for the FUOTA service to be functional:

- `smtc_modem_hal_context_store` for the `CONTEXT_FUOTA` parameter
- `smtc_modem_hal_context_restore` function

These two functions allow the received firmware update file to be stored in non-volatile memory during the FUOTA session.

#### Starting a Class C FUOTA session

There is nothing specific to do to start a multicast session in class C, it will be done transparently depending on the requests of the fuota server.

In the particular case where your Network server does not support the network time cmd, you will also need to activate the clk sync service (refer to the specific chapter of this service).

#### End of Class C FUOTA session

The implemented service only takes care of the transfer/reconstruction of the file and its storage in non-volatile memory. Everything related to the implementation of the new firmware itself (reboot, dual bank, etc.) is not implemented in this service and is left to the implementation of the end user.

Once the session is completed, the user receives an event: `SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE` with the status "true" if the transfer is completed without error.

#### Remark

The fragmentation session for FUOTA may be terminated, but not the Class C multicast session. The latter remains active until the session timeout. Of course, the user can end the session themselves when they receive the SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE event. They can also leave the multicast session active until the session timeout. At this point, they will be notified by the event `"SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C,"` and they will receive an event `"SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C"` at the start of the session, with the associated group_id present in the status of this event.

#### Special case

when all Class C multicast sessions are completed, the stack remains in Class C Unicast. It is therefore up to the user to switch back to Class A if they wish (on the event `"SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C"`).

#### To start a Class B multicast FUOTA session

 the mechanism is very similar to that described for Class C, with the exception that Class B must be launched beforehand. Please refer to the relevant chapter for information on starting Class B. Otherwise, the general principles are the same as for Class C. In normal operation, the user should therefore receive the following events successively:

- `SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B`
- `SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE`
- `SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B`

#### Fuota package V2

This library also offers the implementation of LoRaWAN Alliance's V2 Fuota packages. This implementation is provided on an experimental basis and will primarily serve as a testing framework for NS (Network Server) implementations of the Fuota version 2 packages. This implementation is currently undergoing validation against the certification test developed by the alliance. For all these reasons, we emphasize that this version of Fuota V2 should not be used on a production device.

To activate FUOTA: add the FUOTA service to the compilation by setting the `LBM_FUOTA` flag to "yes" , `LBM_FUOTA_VERSION` flag to "2" in the `options.mk` file.
ou will also need to provide the following fields at compilation:

- `FUOTA_MAXIMUM_NB_OF_FRAGMENTS`
- `FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS`
- `FUOTA_MAXIMUM_FRAG_REDUNDANCY`

Class B, Class C, multicast, and the previously mentioned packages are automatically built by activating this compilation flag.

## CSMA for LoRaWAN (experimental - read disclaimer note at the bottom)

The modem provides an experimental implementation of CSMA feature to avoid LoRa packet collision.

The compilation option `LBM_CSMA=yes` must set in Makefile to be build.

The compilation option `USE_CSMA_BY_DEFAULT=yes` enable the CSMA by default at modem startup.

The implementation will not allow to enable the CSMA when the LBT is mandatory by the current region.

1. To use it, you can enable or disable the CSMA feature with :

```c
smtc_modem_return_code_t smtc_modem_csma_set_state( uint8_t stack_id, bool enable );
```

2. Configuration

The CSMA could be configured to be optimized to your use cases

```c
smtc_modem_return_code_t smtc_modem_csma_set_parameters( uint8_t stack_id, uint8_t nb_bo_max, bool bo_enabled, uint8_t max_ch_change );
```

**Note**: This feature is only supported on lr11xx and sx126x radios, activation of this feature on other targets can lead to unwanted behavior or modem panic

## Relay (experimental - read disclaimer note at the bottom)
**LoRa Basic Modem** proposes an implementation of the [LoRaWAN® Relay Specification TS011-1.0.0](https://resources.lora-alliance.org/technical-specifications/ts011-1-0-0-relay) 

This implementation provides the code for the relayed end-device (refer as Relay TX) and for the relay itself (refer as Relay RX). 
### Building the Relay RX

To build the Relay RX, you need to add the option **RELAY_RX_ENABLE=yes** in the compile command line. By example:
```bash
make full_lr1110 RELAY_RX_ENABLE=yes
```
Adding this option to the command line will compile and include all the required file for the relay to be functional. 
This option will require an additional 2.5 kbytes of RAM and 10 kbytes of FLASH.

On a hardware note, it is strongly recommended to use a TCXO with a Relay RX to avoid RF mismatch between the end-device and the relay itself.

### Application for the Relay RX

The Relay RX doesn't require a specific application, it will start once it received the appropriate MAC command. 
It is important to note that the Relay RX will wake up very frequently and require a quick access to the radio (through the smtc_modem_run_engine function). Every action that could delay the call of this function may result in aborted scan. 

### Configuration of Relay TX

To add the relay feature to an end-device, you need to add the option **RELAY_TX_ENABLE=yes** in the compile command line. By example:
```bash
make full_lr1110 RELAY_TX_ENABLE=yes
```
Adding this option to the command line will compile and include all the required file for the end-device to be functional. 
This option will require an additional 500 bytes of RAM and 5.5 kbytes of FLASH.

### Application for the Relay TX

By default, the end-device is configured in **END-DEVICE CONTROLED** mode (refer to TS011-1.0.0 table 40). 
In this mode, we have chosen to enable the relay mode right from the start. 
The end-device will automatically disable the relay mode if it receives a downlink on RX1 or RX2. 
The end-device will automatically enable the relay mode if it doesn't receive a downlink after 8 uplinks.
The previous statements are true if the RelayModeActivation is either **DYNAMIC** or **END-DEVICE CONTROLED**. 

### Known limitation for the Relay
 - This implementation is only compatible with embedded software cryptographic operations. 
 - Only one component of the relay (RX or TX) could be compiled at the same time.
 - Default channel only support the index 0. 
 - SX128x and SX127x are not supported 
 - Relay TX and Relay RX are not yet compatible with LBT and CSMA.
 - Relay TX is only working with 1 LoRaWAN stack (stack ID 0)



## LoRa Basic Modem known limitations

- [test-mode-cw] Using cw test mode will end up with a radio planner failsafe and a reset after 2 minutes
- [charge] Values returned by smtc_modem_get_charge() for regions CN470 and CN470_RP1 are not accurate
- [charge] Values returned by smtc_modem_get_charge() for the LR-FHSS based datarates are not accurate
- [charge] Values returned by smtc_modem_get_charge() for sx127x radios are not accurate
- [sx127x] On sx127x radios test mode CW does not generate any continuous wave signal

## Disclaimer

**EXAMPLE ONLY**
This code is being released by Semtech as a feature branch including experimental features.  
This code has not been subjected to the same validation procedure applied to Semtech’s master branch releases.  
This code is released under the Clear BSD license LICENSE.txt.  
Consistent with the terms of the Clear BSD license, it is not warrantied in anyway.  
