##############################################################################
# Main makefile for basic_modem
##############################################################################
-include app_makefiles/app_printing.mk
-include app_makefiles/app_options.mk


#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	lr1110	lr1120")
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
	$(call echo_help, " * make full_<TARGET>              : clean and build basic_modem on a given target")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------- Optional build parameters ---------------------------")
	$(call echo_help, " * MODEM_APP=xxx                   : choose which modem application to build:(default is EXAMPLE_GEOLOCATION)")
	$(call echo_help, " *                                  - EXAMPLE_GEOLOCATION")
	$(call echo_help, " *                                  - EXAMPLE_FULL_ALMANAC_UPDATE")
	$(call echo_help, " *                                  - EXAMPLE_LR11XX_FLASHER")
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
	$(call echo_help, " *                                  - WW_2G4 (to be used only for lr1120 target)")
	$(call echo_help, " * CRYPTO=xxx                      : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                  - SOFT")
	$(call echo_help, " *                                  - LR11XX (only for lr1110 and lr1120 targets)")
	$(call echo_help, " *                                  - LR11XX_WITH_CREDENTIALS (only for lr1110 and lr1120 targets)")
	$(call echo_help, " * LBM_TRACE=yes/no                : choose to enable or disable modem trace print (default: trace is ON)")
	$(call echo_help, " * APP_TRACE=yes/no                : choose to enable or disable application trace print (default: trace is ON)")
	$(call echo_help, " * USE_LR11XX_CRC_SPI=yes          : only for lr1110 and lr1120 targets: use crc over spi")
	$(call echo_help_b, "-------------------- Optional makefile parameters --------------------------")
	$(call echo_help, " * MULTITHREAD=no                  : Disable multithreaded build")
	$(call echo_help, " * VERBOSE=yes                     : Increase build verbosity")
	$(call echo_help, " * SIZE=yes                        : Display size for all objects")
	$(call echo_help, " * DEBUG=yes                       : Compile library and application with debug symbols")



#-----------------------------------------------------------------------------
# Makefile include selection
#-----------------------------------------------------------------------------
ifeq ($(TARGET_RADIO),lr1110)
-include app_makefiles/app_lr11xx.mk
endif

ifeq ($(TARGET_RADIO),lr1120)
-include app_makefiles/app_lr11xx.mk
endif

#-----------------------------------------------------------------------------
-include app_makefiles/app_common.mk

.PHONY: clean_all help

#-----------------------------------------------------------------------------
# Clean
#-----------------------------------------------------------------------------
clean_all: clean_app
	$(MAKE) -C $(LORA_BASICS_MODEM) clean_all $(MTHREAD_FLAG)

clean_lr1110:
	$(MAKE) clean_modem TARGET_RADIO=lr1110
	$(MAKE) clean_target TARGET_RADIO=lr1110

clean_lr1120:
	$(MAKE) clean_modem TARGET_RADIO=lr1120
	$(MAKE) clean_target TARGET_RADIO=lr1120

clean:
	$(MAKE) clean_modem
	$(MAKE) clean_target

clean_app:
	-rm -rf $(APPBUILD_ROOT)*


#-----------------------------------------------------------------------------
# Application Compilation
#-----------------------------------------------------------------------------

#-- Generic -------------------------------------------------------------------
app:
ifeq ($(TARGET_RADIO),nc)
	$(call echo_error,"No radio selected! Please specified the target radio using TARGET_RADIO=radio_name option")
else
ifneq ($(CRYPTO),SOFT)
ifneq ($(LBM_NB_OF_STACK),1)
	$(call echo_error, "----------------------------------------------------------")
	$(call echo_error, "More than one stack compiled: only soft crypto can be used")
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "----------------------------------------------------------")
	exit 1
endif
endif
	$(MAKE) app_build
endif

#-- LR1110 -------------------------------------------------------------------
lr1110:
	$(MAKE) app_build TARGET_RADIO=lr1110 $(MTHREAD_FLAG)

full_lr1110:
	$(MAKE) clean_modem TARGET_RADIO=lr1110
	$(MAKE) clean_target TARGET_RADIO=lr1110
	$(MAKE) app_build TARGET_RADIO=lr1110 $(MTHREAD_FLAG)

#-- LR1120 -------------------------------------------------------------------
lr1120:
	$(MAKE) app_build TARGET_RADIO=lr1120 $(MTHREAD_FLAG)

full_lr1120:
	$(MAKE) clean_modem TARGET_RADIO=lr1120
	$(MAKE) clean_target TARGET_RADIO=lr1120
	$(MAKE) app_build TARGET_RADIO=lr1120 $(MTHREAD_FLAG)
