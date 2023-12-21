# Reference Implementation 1: LBM on ThreadX on STM32U5 Nucleo board

<!-- ABOUT THE PROJECT -->
## About This Application

**This application**  proposes an implementation of the LoRa Basic Modem running on the ThreadX operating system. The application runs specifically on the STM32U575 microcontroller and is compatible with both SX126x and LR1110 Semtech radios.

### Built With

This application is using the ThreadX library delivered by Azure. For more details, please refer to
<https://github.com/azure-rtos/threadx>

<!-- GETTING STARTED -->
## Getting Started

This is an example of how run this application with a SX1261 radio.

### Prerequisites

The ARM GCC tool chain must be setup under your development environment

### Installation

1. Set LoRaWAN credentials. Below an example of how to modify credentials in the 'example_options.h' file :

```
#define USER_LORAWAN_DEVICE_EUI
    {0xAC, 0xAF, 0xEF, 0xAB, 0x00, 0x00, 0x00, 0x00}

#define USER_LORAWAN_JOIN_EUI
    {0xAC, 0xAF, 0xEF, 0xAB, 0x00, 0x00, 0x00, 0x00}

#define USER_LORAWAN_APP_KEY
    {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
```

2. Compile

   ```
   make full_sx1261
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

This application example is based on LoRaWAN v1.0.4 specification (Class A, Class B and Class C supported).
The FUOTA packages V1.0.0 as defined by the LoRa Alliance are compiled by default.

This application demonstrates 3 threads running at the same time :

```
// LBM thread to manage LBM stack
void thread_lbm( ULONG thread_input )

// First Application thread example , send LoRa uplink on port 101 periodically (every 30 seconds)
void thread_lorawan_tx_periodic( ULONG thread_input )

//Second Application thread example, send LoRa uplink on port 106 every 2 minutes
or suspend/resume modem communication when user pushes the nucleo blue button.
void thread_app( ULONG thread_input )
```

### <u>Note :</u>

To switch to FUOTA V2 packages, please modify the compilation option under 'app_options.mk' file :

```
ALLOW_FUOTA ?= yes
FUOTA_VERSION ?= 2
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Refer to LBM license policies
