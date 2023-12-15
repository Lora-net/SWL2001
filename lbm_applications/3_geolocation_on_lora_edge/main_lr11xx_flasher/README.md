# LR11xx firmware flasher tool

## Description

The flasher tool does the following:
* enter DFU mode
* write the firmware to the LR11xx flash memory.
* reset the LR11xx, get the firmware version and print it.

It expects a LR11xx transceiver firmware with a command `LR11XX_SYSTEM_GET_VERSION_OC (0x0101)` available.

## Building the flasher

The main_lr11xx_flasher tool takes as input a header file named `lr11xx_fw.h` which should contain the firmware to be programmed in the LR11xx flash memory.

This header file can just be copied from the distributed header files in the following repository:

https://github.com/Lora-net/radio_firmware_images


Once the `lr11xx_fw.h` has been put in the `main_lr11xx_flasher` directory, the flasher tool can be compiled with:

```console
make full_lr1110 MODEM_APP=EXAMPLE_LR11XX_FLASHER
```