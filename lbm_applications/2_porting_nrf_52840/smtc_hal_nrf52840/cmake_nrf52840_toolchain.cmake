# SPDX-License-Identifier: BSD-3-Clause-Clear

# This CMake toolchain file describes how to build for STM32L476xx

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CROSSCOMPILING 1)


set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_SIZE         arm-none-eabi-size)


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


set(SMTC_HAL_DIR ${CMAKE_CURRENT_LIST_DIR})
set(MCU_DRIVERS_DIR ${CMAKE_CURRENT_LIST_DIR}/../mcu_drivers)
set(BOARD_LDSCRIPT ${CMAKE_CURRENT_LIST_DIR}/generic_gcc_nrf52.ld)

set(C_FLAGS_COMMON "\
-mcpu=cortex-m4 -mthumb -mabi=aapcs -fno-builtin \
-mfloat-abi=hard -mfpu=fpv4-sp-d16 -mhard-float \
-fno-unroll-loops -ffast-math -ftree-vectorize -fomit-frame-pointer \
-fdata-sections -ffunction-sections -falign-functions=16 -fno-strict-aliasing \
-DNRF52840_XXAA \
"
)

set(CMAKE_C_FLAGS_INIT   "${C_FLAGS_COMMON}")
set(CMAKE_ASM_FLAGS_INIT "${C_FLAGS_COMMON} -x assembler-with-cpp")

set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nano.specs --specs=nosys.specs -T${BOARD_LDSCRIPT} -L${SMTC_HAL_DIR} -lnosys -lstdc++ -lsupc++ -lc -Wl,--gc-sections -Wl,--print-memory-usage")
