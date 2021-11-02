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

# For LR1110 tranceiver
# - CHINA_DEMO -> Use Regional Parameters 1.0
# - HYBRID_CHINA -> RP 1.0, single channel
CHINA_DEMO ?= no
HYBRID_CHINA ?= no

ifeq ($(HYBRID_CHINA),yes)
CHINA_DEMO = yes
endif

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

# GNSS
USE_GNSS ?= yes



ifeq ($(MODEM_APP),EXAMPLE_LR1110_DEMO)
override USE_GNSS = yes
endif


#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	sx128x	lr1110	sx1261	sx1262")
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
	$(call echo_help, " * CRYPTO=xxx                      : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                  - SOFT")
	$(call echo_help, " *                                  - LR1110 (only for lr1110 target)")
	$(call echo_help, " *                                  - LR1110_WITH_CREDENTIALS (only for lr1110 target)")
	$(call echo_help, " * MODEM_TRACE=yes/no              : choose to enable or disable modem trace print (default: trace is ON)")
	$(call echo_help, " * HYBRID_CHINA=yes                : only for lr1110 target: build hybrid china with monochannel region")
	$(call echo_help, " * CHINA_DEMO=yes                  : only for lr1110 target: build with full China RP_1_0 region")
	$(call echo_help, " * USE_GNSS=yes/no                 : only for lr1110 target: choose to enable or disable use of gnss (default: gnss is ON)")
	$(call echo_help, " * BYPASS=yes                      : build target using lorawan bypass")
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
-include makefiles/lr1110.mk
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

all: basic_modem_sx128x basic_modem_lr1110 basic_modem_sx1261 basic_modem_sx1262

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_all:
	-rm -rf $(BUILD_ROOT)

clean_sx128x:
	$(MAKE) clean_target RADIO=sx128x

clean_lr1110:
	$(MAKE) clean_target RADIO=lr1110

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

basic_modem_sx1261:
	$(MAKE) basic_modem RADIO=sx1261 $(MTHREAD_FLAG)

basic_modem_sx1262:
	$(MAKE) basic_modem RADIO=sx1262 $(MTHREAD_FLAG)

