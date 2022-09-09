##############################################################################
# Main makefile for basic_modem
##############################################################################

-include makefiles/printing.mk

#-----------------------------------------------------------------------------
# Global configuration options
#-----------------------------------------------------------------------------
# Prefix for all build directories
BUILD_ROOT = build

# Prefix for all binaries names
TARGET_ROOT = basic_modem

BYPASS=no

PERF_TEST?=no

# Compile with coverage analysis support
COVERAGE ?= no

# Use multithreaded build (make -j)
MULTITHREAD ?= yes

# Print each object file size
SIZE ?= no

# Save memory usage to log file
LOG_MEM ?= yes

# Tranceiver
RADIO ?= nc

#MCU - Must be provided by user
MCU_FLAGS =? nc

#-----------------------------------------------------------------------------
# Internal LBM features management
#-----------------------------------------------------------------------------

# Middleware advanced access
MIDDLEWARE?= no

# Crypto management
CRYPTO ?= SOFT

# D2D feature
ADD_D2D ?= no

# Multicast feature
ADD_MULTICAST ?= yes

# Stream feature
ADD_SMTC_STREAM ?= yes

# File Upload feature
ADD_SMTC_FILE_UPLOAD ?= yes

# ALCSYNC feature
ADD_SMTC_ALC_SYNC ?= yes

# Trace prints
MODEM_TRACE ?= yes
MODEM_DEEP_TRACE ?= no

# GNSS
USE_GNSS ?= yes


#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	sx128x	lr1110	lr1120	sx1261	sx1262	sx1268")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------------------- Clean -------------------------------------")
	$(call echo_help, " * make clean_<TARGET>                     : clean basic_modem for a given target")
	$(call echo_help, " * make clean_all                          : clean all")
	$(call echo_help, "")
	$(call echo_help_b, "----------------------------- Compilation ----------------------------------")
	$(call echo_help, " * make basic_modem_<TARGET> MCU_FLAGS=xxx : build basic_modem on a given target with chosen mcu flags")
	$(call echo_help, " *                                           MCU_FLAGS are mandatory. Ex for stm32l4:")
	$(call echo_help, " *                                           MCU_FLAGS=\"-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard\"")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------- Optional build parameters ---------------------------")
	$(call echo_help, " * REGION=xxx                              : choose which region should be compiled (default: all)")
	$(call echo_help, " *                                           Combinations also work (i.e. REGION=EU_868,US_915 )")
	$(call echo_help, " *                                          - AS_923")
	$(call echo_help, " *                                          - AU_915")
	$(call echo_help, " *                                          - CN_470")
	$(call echo_help, " *                                          - CN_470_RP_1_0")
	$(call echo_help, " *                                          - EU_868")
	$(call echo_help, " *                                          - IN_865")
	$(call echo_help, " *                                          - KR_920")
	$(call echo_help, " *                                          - RU_864")
	$(call echo_help, " *                                          - US_915")
	$(call echo_help, " *                                          - WW_2G4 (to be used only for lr1120 and sx128x targets)")
	$(call echo_help, " * RP_VERSION=xxx                          : choose wich regional paramerter version should be compiled (default: RP2_101) ")
	$(call echo_help, " *                                          - RP2_101")
	$(call echo_help, " *                                          - RP2_103 (LR-FHSS support)")
	$(call echo_help, " * CRYPTO=xxx                              : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                          - SOFT")
	$(call echo_help, " *                                          - LR11XX (only for lr1110 and lr1120 targets)")
	$(call echo_help, " *                                          - LR11XX_WITH_CREDENTIALS (only for lr1110 and lr1120 targets)")
	$(call echo_help, " * MODEM_TRACE=yes/no                      : choose to enable or disable modem trace print (default: yes)")
	$(call echo_help, " * USE_GNSS=yes/no                         : only for lr1110 and lr1120 targets: choose to enable or disable use of gnss (default: yes)")
	$(call echo_help, " * MIDDLEWARE=yes/no                       : build target for middleware advanced access (default: no)")
	$(call echo_help_b, "-------------------- Optional makefile parameters --------------------------")
	$(call echo_help, " * DEBUG=yes/no                            : Change opt to O0 and add -g* options for debugging (default: no)")
	$(call echo_help, " * MULTITHREAD=yes/no                      : Multithreaded build (default: yes)")
	$(call echo_help, " * VERBOSE=yes/no                          : Increase build verbosity (default: no)")
	$(call echo_help, " * SIZE=yes/no                             : Display size for all objects (default: no)")



#-----------------------------------------------------------------------------
# Makefile include selection
#-----------------------------------------------------------------------------
ifeq ($(RADIO),lr1110)
-include makefiles/lr11xx.mk
endif

ifeq ($(RADIO),lr1120)
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

all: basic_modem_sx128x basic_modem_lr1110 basic_modem_lr1120 basic_modem_sx1261 basic_modem_sx1262

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_all:
	-rm -rf $(BUILD_ROOT)

clean_sx128x:
	$(MAKE) clean_target RADIO=sx128x

clean_lr1110:
	$(MAKE) clean_target RADIO=lr1110

clean_lr1120:
	$(MAKE) clean_target RADIO=lr1120

clean_sx1261:
	$(MAKE) clean_target RADIO=sx1261

clean_sx1262:
	$(MAKE) clean_target RADIO=sx1262

clean_sx1268:
	$(MAKE) clean_target RADIO=sx1268

#-----------------------------------------------------------------------------
# Compilation
#-----------------------------------------------------------------------------
basic_modem_sx128x:
	$(MAKE) basic_modem RADIO=sx128x $(MTHREAD_FLAG)

basic_modem_lr1110:
	$(MAKE) basic_modem RADIO=lr1110 $(MTHREAD_FLAG)

basic_modem_lr1120:
	$(MAKE) basic_modem RADIO=lr1120 $(MTHREAD_FLAG)

basic_modem_sx1261:
	$(MAKE) basic_modem RADIO=sx1261 $(MTHREAD_FLAG)

basic_modem_sx1262:
	$(MAKE) basic_modem RADIO=sx1262 $(MTHREAD_FLAG)

basic_modem_sx1268:
	$(MAKE) basic_modem RADIO=sx1268 $(MTHREAD_FLAG)
