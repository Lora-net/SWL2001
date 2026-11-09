# SMTC_core_HAL

This repo contains the prototypes of all the core functions used by Semtech samples and libraries.
These are the prototypes of all the user-dependant functions that are needed for the libraries. These functions make the link between the generic software and the user-dependant hardware. It is the responsability of the user to implement the definitions of these functions in a source file and to properly link this library.

## Compilation

This repo uses CMake, and, when included via `add_subdirectory()`, exposes an INTERFACE library called `smtc_core_hal_interface`. An INTERFACE library does not create artefacts on the disk and is merely a set of properties that are transferred to the library consumer. 
In that case, linking to `smtc_core_hal_interface` automatically reports the `target_include_directories()` to the caller.

## Functionalities

### Logging
This library exposes macros used for logging, defined in [smtc_modem_hal_dbg_trace](./smtc_core_hal_trace.h). Please look at this file for a more detailed explanation on how to use them.

## Sidenote - functions names
All the functions in this project start with `smtc_modem_hal_`. This is due to the fact that, historically, these functions were used by the Semtech LoRa Basics Modem (LBM) library. When transitionning to  Semtech RAC (Radio Access Control), the LBM architecture was kept to maximize the retrocompatibility.
In a future version, the functions prefix _might_ be updated to `smtc_core_hal_` in a harmonization effort.