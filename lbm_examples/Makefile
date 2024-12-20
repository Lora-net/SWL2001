##############################################################################
# Main makefile for basic_modem
##############################################################################
-include app_makefiles/app_printing.mk
-include app_makefiles/app_options.mk


#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	sx128x	lr1110	lr1120	lr1121	sx1261	sx1262	sx1268 sx1272 sx1276")
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
	$(call echo_help, " * BOARD=xxx                       : choose which mcu board will be used:(default is NUCLEO_L476)")
	$(call echo_help, " *                                  - NUCLEO_L476")
	$(call echo_help, " *                                  - NUCLEO_L073")
	$(call echo_help, " * MODEM_APP=xxx                   : choose which modem application to build:(default is PERIODICAL_UPLINK)")
	$(call echo_help, " *                                  - PERIODICAL_UPLINK")
	$(call echo_help, " *                                  - HW_MODEM")
	$(call echo_help, " *                                  - PORTING_TESTS")
	$(call echo_help, " *                                  - LCTT_CERTIF")
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
	$(call echo_help, " * CRYPTO=xxx                      : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                  - SOFT")
	$(call echo_help, " *                                  - LR11XX (only for lr1110 and lr1120 targets)")
	$(call echo_help, " *                                  - LR11XX_WITH_CREDENTIALS (only for lr1110 and lr1120 targets)")
	$(call echo_help, " * LBM_TRACE=yes/no                : choose to enable or disable modem trace print (default: trace is ON)")
	$(call echo_help, " * APP_TRACE=yes/no                : choose to enable or disable application trace print (default: trace is ON)")
	$(call echo_help, " * ALLOW_RELAY_TX=yes/no           : choose to enable or disable RelayTx (default: no)")
	$(call echo_help, " * ALLOW_RELAY_RX=yes/no           : choose to enable or disable RelayRx (default: no)")
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

ifeq ($(TARGET_RADIO),lr1121)
-include app_makefiles/app_lr11xx.mk
endif

ifeq ($(TARGET_RADIO),sx1261)
-include app_makefiles/app_sx126x.mk
endif

ifeq ($(TARGET_RADIO),sx1262)
-include app_makefiles/app_sx126x.mk
endif

ifeq ($(TARGET_RADIO),sx1268)
-include app_makefiles/app_sx126x.mk
endif

ifeq ($(TARGET_RADIO),sx128x)
-include app_makefiles/app_sx128x.mk
endif

ifeq ($(TARGET_RADIO),sx1272)
-include app_makefiles/app_sx127x.mk
endif

ifeq ($(TARGET_RADIO),sx1276)
-include app_makefiles/app_sx127x.mk
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

clean_lr1121:
	$(MAKE) clean_modem TARGET_RADIO=lr1121
	$(MAKE) clean_target TARGET_RADIO=lr1121

clean_sx1261:
	$(MAKE) clean_modem TARGET_RADIO=sx1261
	$(MAKE) clean_target TARGET_RADIO=sx1261

clean_sx1262:
	$(MAKE) clean_modem TARGET_RADIO=sx1262
	$(MAKE) clean_target TARGET_RADIO=sx1262

clean_sx1268:
	$(MAKE) clean_modem TARGET_RADIO=sx1268
	$(MAKE) clean_target TARGET_RADIO=sx1268

clean_sx128x:
	$(MAKE) clean_modem TARGET_RADIO=sx128x
	$(MAKE) clean_target TARGET_RADIO=sx128x

clean_sx1272:
	$(MAKE) clean_modem TARGET_RADIO=sx1272
	$(MAKE) clean_target TARGET_RADIO=sx1272

clean_sx1276:
	$(MAKE) clean_modem TARGET_RADIO=sx1276
	$(MAKE) clean_target TARGET_RADIO=sx1276

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
ifneq ($(TARGET_RADIO),lr1110)
ifneq ($(TARGET_RADIO),lr1120)
ifneq ($(TARGET_RADIO),lr1121)
	$(call echo_error, "------------------------------------------------------------")
	$(call echo_error, "sx126x, sx127x and sx128x radio tagets: only soft crypto can be used")
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "------------------------------------------------------------")
	exit 1
endif
endif
endif
endif
	$(MAKE) app_build
endif

#-- LR1110 -------------------------------------------------------------------
lr1110:
	$(MAKE) app TARGET_RADIO=lr1110 $(MTHREAD_FLAG)

full_lr1110:
	$(MAKE) clean_modem TARGET_RADIO=lr1110
	$(MAKE) clean_target TARGET_RADIO=lr1110
	$(MAKE) app TARGET_RADIO=lr1110 $(MTHREAD_FLAG)

#-- LR1120 -------------------------------------------------------------------
lr1120:
	$(MAKE) app TARGET_RADIO=lr1120 $(MTHREAD_FLAG)

full_lr1120:
	$(MAKE) clean_modem TARGET_RADIO=lr1120
	$(MAKE) clean_target TARGET_RADIO=lr1120
	$(MAKE) app TARGET_RADIO=lr1120 $(MTHREAD_FLAG)

#-- LR1121 -------------------------------------------------------------------
lr1121:
	$(MAKE) app TARGET_RADIO=lr1121 $(MTHREAD_FLAG)

full_lr1121:
	$(MAKE) clean_modem TARGET_RADIO=lr1121
	$(MAKE) clean_target TARGET_RADIO=lr1121
	$(MAKE) app TARGET_RADIO=lr1121 $(MTHREAD_FLAG)

#-- SX1261 -------------------------------------------------------------------
sx1261:
	$(MAKE) app TARGET_RADIO=sx1261 $(MTHREAD_FLAG)

full_sx1261:
	$(MAKE) clean_modem TARGET_RADIO=sx1261
	$(MAKE) clean_target TARGET_RADIO=sx1261
	$(MAKE) app TARGET_RADIO=sx1261 $(MTHREAD_FLAG)

#-- SX1262 -------------------------------------------------------------------
sx1262:
	$(MAKE) app TARGET_RADIO=sx1262 $(MTHREAD_FLAG)

full_sx1262:
	$(MAKE) clean_modem TARGET_RADIO=sx1262
	$(MAKE) clean_target TARGET_RADIO=sx1262
	$(MAKE) app TARGET_RADIO=sx1262 $(MTHREAD_FLAG)

#-- SX1268 -------------------------------------------------------------------
sx1268:
	$(MAKE) app TARGET_RADIO=sx1268 $(MTHREAD_FLAG)

full_sx1268:
	$(MAKE) clean_modem TARGET_RADIO=sx1268
	$(MAKE) clean_target TARGET_RADIO=sx1268
	$(MAKE) app TARGET_RADIO=sx1268 $(MTHREAD_FLAG)

#-- SX128x -------------------------------------------------------------------
sx128x:
	$(MAKE) app TARGET_RADIO=sx128x $(MTHREAD_FLAG)

full_sx128x:
	$(MAKE) clean_modem TARGET_RADIO=sx128x
	$(MAKE) clean_target TARGET_RADIO=sx128x
	$(MAKE) app TARGET_RADIO=sx128x $(MTHREAD_FLAG)

#-- SX1272 -------------------------------------------------------------------
sx1272:
	$(MAKE) app TARGET_RADIO=sx1272 $(MTHREAD_FLAG)

full_sx1272:
	$(MAKE) clean_modem TARGET_RADIO=sx1272
	$(MAKE) clean_target TARGET_RADIO=sx1272
	$(MAKE) app TARGET_RADIO=sx1272 $(MTHREAD_FLAG)

#-- SX1276 -------------------------------------------------------------------
sx1276:
	$(MAKE) app TARGET_RADIO=sx1276 $(MTHREAD_FLAG)

full_sx1276:
	$(MAKE) clean_modem TARGET_RADIO=sx1276
	$(MAKE) clean_target TARGET_RADIO=sx1276
	$(MAKE) app TARGET_RADIO=sx1276 $(MTHREAD_FLAG)
