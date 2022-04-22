#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------

# Default MCU compilation flags
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

# If MCU is defined in caller Makefile, use it instead
MCU ?= $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

