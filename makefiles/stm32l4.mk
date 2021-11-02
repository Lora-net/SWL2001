#-----------------------------------------------------------------------------
# Compilation flags
#-----------------------------------------------------------------------------
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

