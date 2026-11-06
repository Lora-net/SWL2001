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
set(BOARD_LDSCRIPT ${MCU_DRIVERS_DIR}/core/STM32U5xx/STM32U575ZITXQ_FLASH.ld)

set(C_FLAGS_COMMON "\
-mcpu=cortex-m33 -mthumb -mabi=aapcs -fno-builtin \
-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
-fno-unroll-loops -ffast-math -ftree-vectorize -fomit-frame-pointer \
-fdata-sections -ffunction-sections -falign-functions=4 \
-DSTM32U575xx -DUSE_HAL_DRIVER \
"
)

set(CMAKE_C_FLAGS_INIT   "${C_FLAGS_COMMON}")
set(CMAKE_ASM_FLAGS_INIT "${C_FLAGS_COMMON} -x assembler-with-cpp")

set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nano.specs --specs=nosys.specs -T${BOARD_LDSCRIPT} -lnosys -lstdc++ -lsupc++ -lc -Wl,--gc-sections -Wl,--print-memory-usage")
