#-----------------------------------------------------------------------------
# Global configuration options
#-----------------------------------------------------------------------------

# Lora Basics Modem path
LORA_BASICS_MODEM := ../lbm_lib/.

# Prefix for all build directories
APPBUILD_ROOT = build

# Prefix for all binaries names
APPTARGET_ROOT = app

# Target board (NUCLEO_L476 or NUCLEO_L073)
BOARD ?= NUCLEO_L476

# Target radio
TARGET_RADIO ?= nc

# Application (PERIODICAL_UPLINK, HW_MODEM, PORTING_TESTS or LCTT_CERTIF)
# Default: PERIODICAL_UPLINK
MODEM_APP ?= nc

# Application region for periodical uplink and lctt certif example (values can be found in smtc_modem_api.h)
# Default in code: SMTC_MODEM_REGION_EU_868 (SMTC_MODEM_REGION_WW2G4 for sx128x)
MODEM_APP_REGION ?= nc

# Allow fuota (take more RAM, due to read_modify_write feature) and force lbm build with fuota
ALLOW_FUOTA ?= no
FUOTA_VERSION ?= 1

# USE LBM Store and forward (take more RAM on STM32L4, due to read_modify_write feature)
ALLOW_STORE_AND_FORWARD ?= no

#TRACE
LBM_TRACE ?= yes
APP_TRACE ?= yes

# LR11xx option to use crc
USE_LR11XX_CRC_SPI ?= no

# Allow relay 
ALLOW_RELAY_RX ?= no
ALLOW_RELAY_TX ?= no

#-----------------------------------------------------------------------------
# LBM options management
#-----------------------------------------------------------------------------

# CRYPTO Management
CRYPTO ?= SOFT

# Multistack 
# Note: if more than one stack is used, more ram will be used for nvm context saving due to read_modify_write feature
LBM_NB_OF_STACK ?= 1

# Add any lbm build options (ex: LBM_BUILD_OPTIONS ?= LBM_CLASS_B=yes REGION=ALL)
LBM_BUILD_OPTIONS ?= LBM_CSMA=yes USE_CSMA_BY_DEFAULT=yes

#-----------------------------------------------------------------------------
# Optimization
#-----------------------------------------------------------------------------

# Application build optimization
APP_OPT = -Os

# Lora Basics Modem build optimization
LBM_OPT = -Os

#-----------------------------------------------------------------------------
# Debug
#-----------------------------------------------------------------------------

# Compile library and application with debug symbols
APP_DEBUG ?= no

# Debug optimization (will overwrite APP_OPT and LBM_OPT values in case DEBUG is set)
DEBUG_APP_OPT ?= -O0
DEBUG_LBM_OPT ?= -O0

#-----------------------------------------------------------------------------
# Makefile Configuration options
#-----------------------------------------------------------------------------

# Use multithreaded build (make -j)
MULTITHREAD ?= yes

# Print each object file size
SIZE ?= no

# Save memory usage to log file
LOG_MEM ?= yes

# Verbosity
VERBOSE ?= no