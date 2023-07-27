#-----------------------------------------------------------------------------
# Global configuration options
#-----------------------------------------------------------------------------

# Tranceiver
RADIO ?= nc

# MCU - Must be provided by user
MCU_FLAGS =? nc

# Prefix for all build directories
BUILD_ROOT = build

# Prefix for all binaries names
TARGET_ROOT = basic_modem

# External flags if needed
EXTRAFLAGS ?=

#-----------------------------------------------------------------------------
# Optimization and Debug
#-----------------------------------------------------------------------------
OPT ?= -Os

DEBUG ?= no

#-----------------------------------------------------------------------------
# Makefile Configuration options
#-----------------------------------------------------------------------------

# Use multithreaded build (make -j)
MULTITHREAD ?= yes

# Print each object file size
SIZE ?= no

# Verbosity
VERBOSE ?= no

#-----------------------------------------------------------------------------
# Internal LBM features management
#-----------------------------------------------------------------------------

# Trace prints
MODEM_TRACE ?= yes
MODEM_DEEP_TRACE ?= no

# LoRaWAN regions: ALL to build all available regions, otherwise regions can be added with a comma separator (ex EU_868,US_915)
# If radio target is sx128x WW_2G4 is forced 
REGION ?= ALL

# Crypto management only for lr11xx targets (SOFT, LR11XX, LR11XX_WITH_CREDENTIALS )
CRYPTO ?= SOFT

# class b feature
LBM_CLASS_B ?= yes

# class c feature
LBM_CLASS_C ?= yes

# Multicast feature (at leastclass B or class C shall be activated)
LBM_MULTICAST ?= yes

# ALCSYNC feature (these options are only taken in count if LBM_FUOTA is disabled)
LBM_ALC_SYNC ?= yes
# ALCSYNC Package version: 1 for v1.0.0 and 2 for v2.0.0
LBM_ALC_SYNC_VERSION ?= 1

# Fuota feature (this will automatically enable class b, class c, multicast and ALCSync)
LBM_FUOTA ?= no
LBM_FUOTA_VERSION ?= 1
FUOTA_MAXIMUM_NB_OF_FRAGMENTS ?= nc
FUOTA_MAXIMUM_SIZE_OF_FRAGMENTS ?= nc
FUOTA_MAXIMUM_FRAG_REDUNDANCY ?= nc
# In case FUOTA is allowed, allow the use of Firmware Management Package
LBM_FUOTA_ENABLE_FMP ?= no
# In case FUOTA is allowed, llow the use of Multi-Package Access Package
LBM_FUOTA_ENABLE_MPA ?= no

# CSMA Feature ( only usable for lr11xx and sx126x targets)
LBM_CSMA ?= yes
USE_CSMA_BY_DEFAULT ?= no

# Multistack
NB_OF_STACK ?= 1