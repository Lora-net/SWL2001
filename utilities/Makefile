##############################################################################
# Main makefile for basic_modem
##############################################################################
LORA_BASICS_MODEM := ../.

-include $(LORA_BASICS_MODEM)/makefiles/printing.mk

#-----------------------------------------------------------------------------
# Global configuration options
#-----------------------------------------------------------------------------
# Prefix for all build directories
APPBUILD_ROOT = build

# Prefix for all binaries names
APPTARGET_ROOT = app

# Target board (NUCLEO_L476 or NUCLEO_L073)
BOARD ?= NUCLEO_L476
export BOARD

# CRYPTO Management
CRYPTO ?= SOFT

# Compile for debugging
DEBUG ?= no

# Use multithreaded build (make -j)
MULTITHREAD ?= yes

# Print each object file size
SIZE ?= no

# Save memory usage to log file
LOG_MEM ?= yes

# Flash board present on DRIVE
DRIVE ?= nc

# Tranceiver
RADIO ?= nc
USE_LR11XX_CRC_SPI ?= no

# Application
MODEM_APP ?= nc

#TRACE
MODEM_TRACE ?= yes
APP_TRACE ?= yes

# GNSS
USE_GNSS ?= yes

#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	sx128x	lr1110	lr1120	lr1121	sx1261	sx1262	sx1268")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------------------- Clean -------------------------------------")
	$(call echo_help, " * make clean_<TARGET>             : clean basic_modem app and lib for a given target")
	$(call echo_help, " * make clean_all                  : clean all")
	$(call echo_help, " * make clean_app                  : clean basic_modem app")
	$(call echo_help, "")
	$(call echo_help_b, "----------------------------- Compilation ----------------------------------")
	$(call echo_help, " * make <TARGET>                   : build basic_modem app and lib on a given target")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------------- All inclusive ---------------------------------")
	$(call echo_help, " * make full_<TARGET>              : clean and build basic_modem on a given target (also flash if DRIVE letter is specified)")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------- Optional build parameters ---------------------------")
	$(call echo_help, " * MODEM_APP=xxx                   : choose which modem application to build:(default is EXAMPLE_EXTI)")
	$(call echo_help, " *                                  - EXAMPLE_EXTI")
	$(call echo_help, " *                                  - EXAMPLE_PORTING_TESTS")
	$(call echo_help, " * BOARD=xxx                       : choose which mcu board will be used:(default is NUCLEO_L476)")
	$(call echo_help, " *                                  - NUCLEO_L476")
	$(call echo_help, " *                                  - NUCLEO_L073")
	$(call echo_help, " * REGION=xxx                      : choose which region should be compiled (default: all)")
	$(call echo_help, " *                                  - AS_923")
	$(call echo_help, " *                                  - AU_915")
	$(call echo_help, " *                                  - CN_470")
	$(call echo_help, " *                                  - CN_470_RP_1_0")
	$(call echo_help, " *                                  - EU_868")
	$(call echo_help, " *                                  - IN_865")
	$(call echo_help, " *                                  - KR_920")
	$(call echo_help, " *                                  - RU_864")
	$(call echo_help, " *                                  - US_915")
	$(call echo_help, " *                                  - WW_2G4 (to be used only for lr1120 and sx128x targets)")
	$(call echo_help, " * RP_VERSION=xxx                  : choose wich regional paramerter version should be compiled (default: RP2_103) ")
	$(call echo_help, " *                                  - RP2_101")
	$(call echo_help, " *                                  - RP2_103 (LR-FHSS support)")
	$(call echo_help, " * CRYPTO=xxx                      : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                  - SOFT")
	$(call echo_help, " *                                  - LR11XX (only for lr1110 and lr1120 targets)")
	$(call echo_help, " *                                  - LR11XX_WITH_CREDENTIALS (only for lr1110 and lr1120 targets)")
	$(call echo_help, " * MODEM_TRACE=yes/no              : choose to enable or disable modem trace print (default: trace is ON)")
	$(call echo_help, " * APP_TRACE=yes/no                : choose to enable or disable application trace print (default: trace is ON)")
	$(call echo_help, " * USE_LR11XX_CRC_SPI=yes          : only for lr1110 and lr1120 targets: use crc over spi")
	$(call echo_help_b, "-------------------- Optional makefile parameters --------------------------")
	$(call echo_help, " * DRIVE=xxx                       : choose drive letter for flash.  Example: DRIVE=g (Used under WSL; need to map the stm32l476 board to disk g:\ first)")
	$(call echo_help, " * MULTITHREAD=no                  : Disable multithreaded build")
	$(call echo_help, " * VERBOSE=yes                     : Increase build verbosity")
	$(call echo_help, " * SIZE=yes                        : Display size for all objects")
	$(call echo_help, " * DEBUG=yes                       : Compile library and application with debug symbols")



#-----------------------------------------------------------------------------
# Makefile include selection
#-----------------------------------------------------------------------------
ifeq ($(RADIO),lr1110)
-include makefiles/lr11xx.mk
endif

ifeq ($(RADIO),lr1120)
-include makefiles/lr11xx.mk
endif

ifeq ($(RADIO),lr1121)
-include makefiles/lr11xx.mk
endif

ifeq ($(RADIO),sx1261)
-include makefiles/sx126x.mk
endif

ifeq ($(RADIO),sx1262)
-include makefiles/sx126x.mk
endif

ifeq ($(RADIO),sx1268)
-include makefiles/sx126x.mk
endif

ifeq ($(RADIO),sx128x)
-include makefiles/sx128x.mk
endif

#-----------------------------------------------------------------------------
-include makefiles/common.mk

.PHONY: clean_all all help
.PHONY: FORCE
FORCE:

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_all: clean_app
	$(MAKE) -C $(LORA_BASICS_MODEM) clean_all $(MTHREAD_FLAG)

clean_lr1110:
	$(MAKE) clean_modem RADIO=lr1110
	$(MAKE) clean_target RADIO=lr1110

clean_lr1120:
	$(MAKE) clean_modem RADIO=lr1120
	$(MAKE) clean_target RADIO=lr1120

clean_lr1121:
	$(MAKE) clean_modem RADIO=lr1121
	$(MAKE) clean_target RADIO=lr1121
	
clean_sx1261:
	$(MAKE) clean_modem RADIO=sx1261
	$(MAKE) clean_target RADIO=sx1261

clean_sx1262:
	$(MAKE) clean_modem RADIO=sx1262
	$(MAKE) clean_target RADIO=sx1262

clean_sx1268:
	$(MAKE) clean_modem RADIO=sx1268
	$(MAKE) clean_target RADIO=sx1268

clean_sx128x:
	$(MAKE) clean_modem RADIO=sx128x
	$(MAKE) clean_target RADIO=sx128x

clean_app:
	-rm -rf $(APPBUILD_ROOT)*


#-----------------------------------------------------------------------------
# Application Compilation
#-----------------------------------------------------------------------------
#-- LR1110 -------------------------------------------------------------------
lr1110:
	$(MAKE) example RADIO=lr1110 $(MTHREAD_FLAG)

full_lr1110:
	$(MAKE) clean_lr1110
	$(MAKE) lr1110 $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=lr1110
endif

#-- LR1120 -------------------------------------------------------------------
lr1120:
	$(MAKE) example RADIO=lr1120 $(MTHREAD_FLAG)

full_lr1120:
	$(MAKE) clean_lr1120
	$(MAKE) lr1120 $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=lr1120
endif

#-- LR1121 -------------------------------------------------------------------
lr1121:
	$(MAKE) example RADIO=lr1121 $(MTHREAD_FLAG)

full_lr1121:
	$(MAKE) clean_lr1121
	$(MAKE) lr1121 $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=lr1121
endif

#-- SX1261 -------------------------------------------------------------------
sx1261:
	$(MAKE) example RADIO=sx1261 $(MTHREAD_FLAG)

full_sx1261:
	$(MAKE) clean_sx1261
	$(MAKE) sx1261 $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=sx1261
endif

#-- SX1262 -------------------------------------------------------------------
sx1262:
	$(MAKE) example RADIO=sx1262 $(MTHREAD_FLAG)

full_sx1262:
	$(MAKE) clean_sx1262
	$(MAKE) sx1262 $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=sx1262
endif

#-- SX1268 -------------------------------------------------------------------
sx1268:
	$(MAKE) example RADIO=sx1268 $(MTHREAD_FLAG)

full_sx1268:
	$(MAKE) clean_sx1268
	$(MAKE) sx1268 $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=sx1268
endif

#-- SX128x -------------------------------------------------------------------
sx128x:
	$(MAKE) example RADIO=sx128x $(MTHREAD_FLAG)

full_sx128x:
	$(MAKE) clean_sx128x
	$(MAKE) sx128x $(MTHREAD_FLAG)
ifneq ($(DRIVE),nc)
	$(MAKE) flash RADIO=sx128x
endif
