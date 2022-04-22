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

PERF_TEST=no

# Crypto management
CRYPTO ?= SOFT

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

# Trace prints
MODEM_TRACE ?= yes
MODEM_DEEP_TRACE ?= no

# GNSS
USE_GNSS ?= yes

# TB bypass
BYPASS_JOIN_DUTY_CYCLE?= no

# Middleware advanced access
MIDDLEWARE?= no

ifeq ($(MODEM_APP),HW_MODEM)
override BYPASS_JOIN_DUTY_CYCLE = yes
endif

#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	sx128x	lr1110	lr1120	sx1261	sx1262")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------------------- Clean -------------------------------------")
	$(call echo_help, " * make clean_<TARGET>             : clean basic_modem for a given target")
	$(call echo_help, " * make clean_all                  : clean all")
	$(call echo_help, "")
	$(call echo_help_b, "----------------------------- Compilation ----------------------------------")
	$(call echo_help, " * make basic_modem_<TARGET>       : build basic_modem on a given target")
	$(call echo_help, " * make all                        : build all targets")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------- Optional build parameters ---------------------------")
	$(call echo_help, " * REGION=xxx                      : choose which region should be compiled (default: all)")
	$(call echo_help, " *                                   Combinations also work (i.e. REGION=EU_868,US_915 )")
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
	$(call echo_help, " * RP_VERSION=xxx                  : choose wich regional paramerter version should be compiled (default: RP2_101) ")
	$(call echo_help, " *                                  - RP2_101")
	$(call echo_help, " *                                  - RP2_103 (LR-FHSS support)")
	$(call echo_help, " * CRYPTO=xxx                      : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                  - SOFT")
	$(call echo_help, " *                                  - LR11XX (only for lr1110 and lr1120 targets)")
	$(call echo_help, " *                                  - LR11XX_WITH_CREDENTIALS (only for lr1110 and lr1120 targets)")
	$(call echo_help, " * MODEM_TRACE=yes/no              : choose to enable or disable modem trace print (default: trace is ON)")
	$(call echo_help, " * USE_GNSS=yes/no                 : only for lr1110 and lr1120 targets: choose to enable or disable use of gnss (default: gnss is ON)")
	$(call echo_help, " * MIDDLEWARE=yes                  : build target for middleware advanced access")
	$(call echo_help_b, "-------------------- Optional makefile parameters --------------------------")
	$(call echo_help, " * MULTITHREAD=no                  : Disable multithreaded build")
	$(call echo_help, " * COVERAGE=xxx                    : build target with coverage instrumentation for specific subsystems:")
	$(call echo_help, " *                                  - RADIO")
	$(call echo_help, " *                                  - MODEM")
	$(call echo_help, " * VERBOSE=yes                     : Increase build verbosity")
	$(call echo_help, " * SIZE=yes                        : Display size for all objects")



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

