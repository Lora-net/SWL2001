# LoRa Basic Modem Porting Guide

## MCU Requirements

LoRa Basics Modem contains a example code for the STMicroelectronics STM32L476 Nucleo board, however, it can be easily ported to other MCUs. The following MCU features are required:  

- 32-bit native operation (No specific CPU core needed)
- Memory : Minimum 40KB of FLASH memory allowing an end-device to run in Class A mode for a single region; each additional region adds 1.5KB.
- Little-endian
- Software MCU reset
- A timer with 100μs resolution or better (Timer accuracy compensation is possible by widening the LoRaWAN reception windows)
- A random number generator (can be implemented in software)
- Non-volatile storage for modem state storage
- An SPI controller with MISO, MOSI, SCK, NSS
- GPIO lines for the transceiver IRQ, BUSY, and RESET lines
- A dedicated (non-shared) GPIO MCU interrupt for the transceiver IRQ line is recommended  

Note that reliable Class A LoRaWAN communication can be obtained without any major time constraints on the MCU oscillator, or the oscillator used to clock the devices that implement the time-related LoRa Basics Modem HAL functions.  
To compensate for time-related oscillator frequency errors, calling the smtc_modem_set_crystal_error_ppm() modem API function with an appropriate value is sufficient.  
This results in a widening of the LoRaWAN reception window and increased power consumption.  
In the case of Class B, however, it is desirable to be able to remain synchronized with the beacon over relatively long time intervals, even if beacons are sometimes not received due to poor RF conditions.  
In this case, it is recommended to use an accurate crystal oscillator to clock the devices used to implement the time-related LoRa Basics Modem HAL functions.

## Transceiver Requirements

The transceivers supported by LoRa Basics Modem and their associated version if applicable can be found in LoRa Basics Modem [README.md](lbm_lib/README.md) file.  
In certain situations, such as the use of GNSS reception with the LR11xx, a transceiver TCXO is required. When using GNSS advanced scan on the LR11xx, the TCXO must have a relatively fast settling time, and the 32.768 kHz crystal oscillator must have 20ppm accuracy at 25 degrees.

## System Design Considerations

There are numerous requirements and options to consider when developing a device that implements LoRaWAN.  

For the LoRa Basics Modem, it is important to consider the transceiver and timer interrupt behavior and configuration. The LoRa Basics Modem is designed to use a specific transceiver DIO line as the radio interrupt source. For SX126x, this is the DIO1 line, and for LR11xx, this is the DIO9 line.
Two principal interrupt sources interact with the LoRa Basics Modem: a timer interrupt, and a radio interrupt.  
The system interrupt priorities must be configured in such a way that the timer and radio interrupts do not nest or interrupt each another.

Therefore, when designing hardware that will run LoRa Basics Modem, it is recommended that the MCU GPIO lines selected for the transceiver's DIO interrupt request line do not share an MCU interrupt flag with other timing-critical hardware. If MCU interrupt flags are shared, it may not always be possible to react immediately to interrupts originating from these other devices.  

No radio operations over the transceiver SPI bus are performed during timer and radio interrupt service routines.

## Radio Driver HAL Implementation

The LoRa Basics Modem depends on Semtech's radio driver, which, in turn, requires a radio driver HAL implementation.  
A brief description of the necessary steps for this implementation follows.  

The HAL implementation must provide platform-specific read, write, reset, and wake-up implementations.

- Radio driver API functions call the HAL implementation to perform the actual reset, wake, and communication operations needed by the driver.
- For the `LR11xx`, these functions are documented in [lr11xx_hal.h](smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_hal.h).
- For the `SX126x`, these functions are documented in [sx126x_hal.h](smtc_modem_core/radio_drivers/sx126x_driver/src/sx126x_hal.h).
- For the `SX128x`, these functions are documented in [sx128x_hal.h](smtc_modem_core/radio_drivers/sx128x_driver/src/sx128x_hal.h).
- For the `SX127x`, these functions are documented in [sx127x_hal.h](smtc_modem_core/radio_drivers/sx127x_driver/src/sx127x_hal.h).

All radio driver API functions take a 'const void* context' argument:

- This argument is opaque to both the radio driver and LoRa Basics Modem.
- It may be used by the HAL implementer to differentiate between different transceivers, which makes it easy to communicate with several radios inside the same application.
- Driver API functions do not use the context argument but pass it directly to the HAL implementation.

The LoRa Basics Modem imposes a specific requirement on the radio driver HAL implementation:

- If a radio driver API function is called while the transceiver is in sleep mode, the HAL implementation must properly wake the transceiver and wait until it is ready before initiating any SPI communication.
- This typically requires that the HAL keeps track of whether the radio is awake or asleep, potentially by monitoring any commands sent to the transceiver to detect the SetSleep command.
- For a concrete LR11xx example, see the file: [lr11xx_hal.c](../lbm_examples/radio_hal/lr11xx_hal.c).
- For a concrete SX126x example, see the file: [sx126x_hal.c](../lbm_examples/radio_hal/sx126x_hal.c).
- For a concrete SX128x example, see the file: [sx128x_hal.c](../lbm_examples/radio_hal/sx128x_hal.c).
- For a concrete SX127x example, see the file: [sx127x_hal.c](../lbm_examples/radio_hal/sx127x_hal.c).

When compiling the radio driver HAL implementation, it is necessary to add the radio driver source directory to the include path.  
For example, for LR11xx:

- `lbm_lib/smtc_modem_core/radio_drivers/lr11xx_driver/src`

## RAL BSP Implementation

When porting the LoRa Basics Modem to a new radio + MCU implementation, a Radio Abstraction Layer (RAL) board support package (BSP) implementation is necessary.  
A brief description of the necessary steps follows.  

The RAL provides radio-independent API functions that are similar to those provided by each radio driver.  
The RAL, and a complementary layer called the RALF, are described in the following header functions:

- [ral.h](smtc_modem_core/smtc_ral/src/ral.h)
- [ralf.h](smtc_modem_core/smtc_ralf/src/ralf.h)

The RAL requires the implementer to define a few BSP API functions for the selected transceiver, by providing platform or radio-specific information to the RAL.

- For the `LR11xx`, these functions are described in [ral_lr11xx_bsp.h](smtc_modem_core/smtc_ral/src/ral_lr11xx_bsp.h).
- For the `SX126x`, these functions are described in [ral_sx126x_bsp.h](smtc_modem_core/smtc_ral/src/ral_sx126x_bsp.h).
- For the `SX128x`, these functions are described in [ral_sx128x_bsp.h](smtc_modem_core/smtc_ral/src/ral_sx128x_bsp.h).
- For the `SX127x`, these functions are described in [ral_sx127x_bsp.h](smtc_modem_core/smtc_ral/src/ral_sx127x_bsp.h).

- An LR11xx sample implementation is in the file [ral_lr11xx_bsp.c](../lbm_examples/radio_hal/ral_lr11xx_bsp.c).
- An SX126x sample implementation is in the file [ral_sx126x_bsp.c](../lbm_examples/radio_hal/ral_sx126x_bsp.c).
- An SX128x sample implementation is in the file [ral_sx128x_bsp.c](../lbm_examples/radio_hal/ral_sx128x_bsp.c).
- An SX127x sample implementation is in the file [ral_sx127x_bsp.c](../lbm_examples/radio_hal/ral_sx127x_bsp.c).

The role of the 'const void* context' variable is described in previous section. It is typically used to store radio-specific information, but depending on the radio driver BSP implementation, it may be NULL if a single transceiver is used.  
When compiling the RAL BSP implementation, it is necessary to add the radio driver source directory and the RAL source directory to the include path. For example, for LR11xx:

- `lbm_lib/smtc_modem_core/radio_drivers/lr11xx_driver/src`
- `lbm_lib/smtc_modem_core/smtc_ral/src`

## LoRa Basics Modem HAL Implementation

Porting LoRa Basics Modem to a new MCU architecture requires implementing the modem Hardware Abstraction Layer (HAL) API commands described by the prototypes in the header file [smtc_modem_hal.h](smtc_modem_hal/smtc_modem_hal.h).  
Among other things, these API implementations define how timing information is provided to the LoRa Basics Modem, how random numbers are generated, and how data is stored in non-volatile memory.  
If a TCXO is used on the radio part, its startup timing behavior should be specified in the RAL BSP implementation, and the documentation of the smtc_modem_hal_start_radio_tcxo(), smtc_modem_hal_stop_radio_tcxo(), and smtc_modem_hal_get_radio_tcxo_startup_delay_ms() functions, should be consulted.  
Logging macros are defined in [smtc_modem_hal_dbg_trace.h](smtc_modem_core/logging/smtc_modem_hal_dbg_trace.h). In case the OS has a macro-based logging implementation, the HAL can provide its own header, by removing the `logging` directory from the include list.  

An example of implementation on STM32L476 and STM32L073 can be found in [smtc_modem_hal.c](../lbm_examples/smtc_modem_hal/smtc_modem_hal.c)

The following sections provide the list and more details on the different modem HAL APIs.  

### Reset related functions

#### `void smtc_modem_hal_reset_mcu( void )`

**Brief**: Reset the MCU.

LoRa Basics Modem may need to reset the MCU during LoRaWAN certification process or in case Device Management service is used and cloud is asking for a device reset.

Recommendation is also to reset the mcu in case of modem_panic (in `smtc_modem_hal_on_panic()`)

### Watchdog related functions

#### `void smtc_modem_hal_reload_wdog( void )`

**Brief**: Reload the watchdog timer.

If the HAL implementation configures a watchdog timer, then this function should be implemented to reload the watchdog timer. Currently, the only code in LoRa Basics Modem that calls this HAL API command is the test code in smtc_modem_test.c.

### Time management related functions

#### `uint32_t smtc_modem_hal_get_time_in_s( void )`

**Brief**:
Provide the time since startup, in seconds.
LoRa Basics Modem uses this command to help perform various LoRaWAN® activities that do not have significant time accuracy requirements, such as NbTrans retransmissions.  
**Return**:
The current system uptime in seconds.

#### `uint32_t smtc_modem_hal_get_time_in_ms( void )`

**Brief**:
Provide the time since startup, in milliseconds.  
**Return**:
The system uptime, in milliseconds. The value returned by this function must monotonically increase all the way to 0xFFFFFFFF and then overflow to 0x00000000.

#### `uint32_t smtc_modem_hal_get_time_in_100us( void )`

**Brief**:
Provide the time since startup, in 100μs units.
This command is used for Class B ping slot openings.  
**Return**:
The system uptime, in tenths of milliseconds. The value returned by this function must monotonically increase all the way to 0xFFFFFFFF, and then overflow to 0x00000000.

### Timer management related functions

#### `void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context )`

**Brief**:
Start a timer that will expire at the requested time.
Upon expiration, the provided callback is called with context as its sole argument.
The current design of the LoRa Basics Modem has only been tested in the case where the provided callback is executed in an interrupt context, with interrupts disabled.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|milliseconds|Number of milliseconds before callback execution|
|[in]|callback|Callback to execute|
|[in]|context|Argument that is passed to callback|

#### `void smtc_modem_hal_stop_timer( void )`

**Brief**:
Stop the timer that may have been started with `smtc_modem_hal_start_timer()`

### IRQ management related functions

#### `void smtc_modem_hal_disable_modem_irq( void )`

**Brief**:
Disable the two interrupt sources that execute the LoRa Basics Modem code: the timer, and the transceiver DIO interrupt source.  
Please also refer to System Design Considerations.

#### `void smtc_modem_hal_disable_modem_irq( void )`

**Brief**::
Enable the two interrupt sources that execute the LoRa Basics Modem code: the timer, and the transceiver DIO interrupt source.  
Please also refer to System Design Considerations.

### Context saving management related functions

#### `void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size )`

**Brief**:
Restore to RAM a data structure of type `ctx_type` that has previously been stored in non-volatile memory by calling `smtc_modem_hal_context_store()`.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|ctx_type|Type of modem context to be restored|
|[out]|buffer|Buffer where context must be restored|
|[in]|size|Number of bytes of context to restore|

#### `void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size )`

**Brief**:
Store a data structure of type `ctx_type` from RAM to non-volatile memory.

**Details of each type of context**:

|Context type|Size in bytes|Purpose|
|---    |---    |---    |
|CONTEXT_MODEM|16|To save general info of modem, eg reset|
|CONTEXT_KEY_MODEM|20|To save crc of keys in case lr11xx crypto engine is used|
|CONTEXT_LORAWAN_STACK|32|To save stack devnonce, joinonce, region, certification status|
|CONTEXT_FUOTA|variable|To save the fragmented data received|
|CONTEXT_SECURE_ELEMENT|480 or 24|To save all secure element context, needed only for certification purpose|
|CONTEXT_STORE_AND_FORWARD|variable|To save data for store and forward|

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|ctx_type|Type of modem context to be saved|
|[in]|buffer|Buffer which must be saved|
|[in]|size|Number of bytes of context to save|

#### `void smtc_modem_hal_context_flash_pages_erase( const modem_context_type_t ctx_type, uint32_t offset, uint8_t nb_page )`

**Brief**:
Erase a chosen number of flash pages of a context.  
This function is only used for Store and Forward service with `ctx_type` parameter set to `CONTEXT_STORE_AND_FORWARD`

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|ctx_type|Type of modem context that need to be erased|
|[in]|offset|Memory offset after ctx_type address|
|[in]|nb_page|Number of pages that|

### Panic management related functions

#### `void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... )`

**Brief**:
Action to be taken in case on modem panic.  
In case Device Management is used, it is recommended to perform the crashlog storage and status update in this function

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|func|The name of the function where the panic occurs|
|[in]|line|The line number where the panic occurs|
|[in]|fmt|String Format|
|[in]|...|String Arguments|

### Random management related functions

#### `uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 )`

**Brief**:
Return a uniformly-distributed unsigned random integer from the closed interval [val_1, ..., val_2] or [val_2, ..., val_1].  
**Return**:
The random integer.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|val_1|first range unsigned value|
|[in]|val_2|second range unsigned value|

### Radio environment management related functions

#### `void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context )`

**Brief**::
Store the callback and context argument that must be executed when a radio event occurs.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|callback|Callback that is executed upon radio interrupt service request|
|[in]|context|Argument that is provided to callback|

#### `void smtc_modem_hal_start_radio_tcxo( void )`

**Brief**:
If the TCXO is not controlled by the transceiver, powers up the TCXO.  
If no TCXO is used, or if the TCXO has been configured in the RAL BSP to start up automatically, then implement an empty command. If the TCXO is not controlled by the transceiver, then this function must power up the TCXO, and then busy wait until the TCXO is running with the proper accuracy.

#### `void smtc_modem_hal_stop_radio_tcxo( void )`

**Brief**:
If the TCXO is not controlled by the transceiver, stop the TCXO.  
If no TCXO is used, or if the TCXO has been configured in the RAL BSP to start up automatically, implement an empty command.

#### `uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void )`

**Brief**:
Return the time, in milliseconds, that the TCXO needs to start up with the required accuracy.
This does not implement a delay but is used to perform certain calculations in the LoRa Basics Modem so that this time will be taken into consideration when opening the Rx window.  
If the TCXO is configured by the RAL BSP to start up automatically, then the value used here should be the same as the startup delay used in the RAL BSP.  
**Return**:
The needed TCXO startup time, in milliseconds. Return 0 if no TCXO is used.

#### `void smtc_modem_hal_set_ant_switch( bool is_tx_on )`

**Brief**:
Set antenna switch for Tx operation or not.  
If no antenna switch is used then implement an empty command.

### Environment management related functions

#### `uint8_t smtc_modem_hal_get_battery_level( void )`

**Brief**:
Indicate the current battery state.  
According to LoRaWan 1.0.4 spec:

- 0: The end-device is connected to an external power source.
- 1..254: Battery level, where 1 is the minimum and 254 is the maximum.
- 255: The end-device was not able to measure the battery level.  
**Return**
A value between 0 (for 0%) and 254 (for 100%) or 255.

#### `int8_t smtc_modem_hal_get_board_delay_ms( void )`

**Brief**:
Return the amount of time that passes between the moment the MCU calls `ral_set_tx()` or `ral_set_rx()`, and the moment the radio transceiver enters RX or TX state.  
This varies depending on the MCU clock speed and SPI bus speed.  
**Return**:
The board delay, in milliseconds.

### Trace management related functions

The HAL can provide its own trace macros by:

- removing the `smtc_modem_core/logging` from the include list
- writing its own `smtc_modem_hal_dbg_trace.h` header, based on the content of the existing[smtc_modem_hal_dbg_trace.h](smtc_modem_core/logging/smtc_modem_hal_dbg_trace.h).

#### `void smtc_modem_hal_print_trace( const char* fmt, ... )`

**Brief**:
Output a printf-style variable-length argument list to the logging subsystem.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|fmt|String Format|
|[in]|...|Variadic arguments |

### Fuota management related functions (optional)

#### `uint32_t smtc_modem_hal_get_hw_version_for_fuota( void )`

**brief**:
Return the hardware version as defined in FMP Specification TS006-1.0.0.  
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.  
**Return**:
The hardware version

#### `uint32_t smtc_modem_hal_get_fw_version_for_fuota( void )`

**brief**:
Return the firmware version as defined in FMP Specification TS006-1.0.0.  
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.  
**Return**:
The firmware version

#### `uint8_t smtc_modem_hal_get_fw_status_available_for_fuota( void )`

**brief**:
Return the firmware status as defined in FMP Specification TS006-1.0.0.  
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.  
**Return**:
The firmware status field

#### `uint8_t smtc_modem_hal_get_fw_delete_status_for_fuota( uint32_t fw_to_delete_version )`

**brief**:
Delete a chosen firmware version as defined in FMP Specification TS006-1.0.0.  
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.  
**Return**:
The firmware status field

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|fw_to_delete_version|Version of firmware to delete|

#### `uint32_t smtc_modem_hal_get_next_fw_version_for_fuota( void )`

**brief**:
Return the firmware version that will be running once the firmware update imagine is installed as defined in FMP Specification TS006-1.0.0.

No need to implement this function if FMP package is not build in `LBM_FUOTA` option.  
**Return**:
The new firmware version.

### Device Management related functions (optional)

#### `int8_t smtc_modem_hal_get_temperature( void )`

**Brief**:
Indicate the current system temperature.  
**Return**:
The temperature, in degrees Celsius.

#### `uint16_t smtc_modem_hal_get_voltage_mv( void )`

**Brief**:
Indicates the current battery voltage.  
**Return**:
The battery voltage in mV.

#### `void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length )`

**Brief**:
Store the modem crash log to non-volatile memory.  
On most MCUs, RAM is preserved upon reset, so it may be possible to use RAM for this purpose.  
This function is not called by LoRa Basics Modem directly but the recommendation is to call it in `smtc_modem_hal_on_panic()`implementation.

In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|crash_string|Crashlog string to be stored|
|[in]|crash_string_length|Crashlog string length|

#### `void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length )`

**Brief**:
Retrieve the modem crash log from non-volatile memory.  
On most MCUs, RAM is preserved upon reset, so it may be possible to use RAM for this purpose.
In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[out]|crash_string|Crashlog string to be restored|
|[out]|crash_string_length|Crashlog string length|

#### `void smtc_modem_hal_crashlog_set_status( bool available )`

**Brief**:
Store the modem crash log status to non-volatile memory. True indicates that a crash log has been stored and is available for retrieval.  
On most MCUs, RAM is preserved upon reset, so it may be possible to use RAM for this purpose.
This function is not called by LoRa Basics Modem directly but the recommendation is to call it in `smtc_modem_hal_on_panic()`implementation.

In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

**Parameters**:  

|       |       |       |
|---    |---    |---    |
|[in]|available|True if a crashlog is available, false otherwise|

#### `bool smtc_modem_hal_crashlog_get_status( void )`

**Brief**:
Get the modem crash log status from non-volatile memory.  
**Return**:
The crash log status, as previously written using `smtc_modem_hal_set_crashlog_status()`.
In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

### Store and Forward related functions (optional)

#### `uint16_t smtc_modem_hal_store_and_forward_get_number_of_pages( void )`

**Brief**:
Return the number of reserved pages in flash for the Store and Forward service.  
At least 3 pages are needed for the service.  
**Return**:
The number of reserved pages

#### `uint16_t smtc_modem_hal_flash_get_page_size( void )`

**Brief**:
Gives the size of a flash page in bytes  
**Return**:
The size of a flash page.  

### RTOS compatibility related functions

#### `void smtc_modem_hal_user_lbm_irq( void )`

**Brief**:
This function is called by the LBM stack on each LBM interruption (radio interrupt or low-power timer interrupt). It could be convenient in the case of an RTOS implementation to notify the thread that manages the LBM stack
