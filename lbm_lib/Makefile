##############################################################################
# Main makefile for basic_modem
##############################################################################

-include makefiles/printing.mk
-include makefiles/options.mk


#-----------------------------------------------------------------------------
# default action: print help
#-----------------------------------------------------------------------------
help:
	$(call echo_help_b, "Available TARGETs:	sx128x	lr1110	lr1120	lr1121	sx1261	sx1262	sx1268 sx1272 sx1276")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------------------- Clean -------------------------------------")
	$(call echo_help, " * make clean_<TARGET>                     : clean basic_modem for a given target")
	$(call echo_help, " * make clean_all                          : clean all")
	$(call echo_help, "")
	$(call echo_help_b, "----------------------------- Compilation ----------------------------------")
	$(call echo_help, " * make basic_modem_<TARGET> MCU_FLAGS=xxx : build basic_modem on a given target with chosen mcu flags")
	$(call echo_help, " *                                           MCU_FLAGS are mandatory. Ex for stm32l4:")
	$(call echo_help, " *                                           MCU_FLAGS=\"-mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard\"")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------- Optional build parameters ---------------------------")
	$(call echo_help, " * REGION=xxx                              : choose which region should be compiled (default: ALL)")
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
	$(call echo_help, " *                                          - ALL (to build all possible regions according to the radio target) ")
	$(call echo_help, " * CRYPTO=xxx                              : choose which crypto should be compiled (default: SOFT)")
	$(call echo_help, " *                                          - SOFT")
	$(call echo_help, " *                                          - LR11XX (only for lr1110 and lr1120 targets)")
	$(call echo_help, " *                                          - LR11XX_WITH_CREDENTIALS (only for lr1110 and lr1120 targets)")
	$(call echo_help, " * MODEM_TRACE=yes/no                      : choose to enable or disable modem trace print (default: yes)")
	$(call echo_help, " * LBM_CLASS_B=yes/no                      : choose to build class B feature (default: no)")
	$(call echo_help, " * LBM_CLASS_C=yes/no                      : choose to build class C feature (default: no)")
	$(call echo_help, " * LBM_MULTICAST=yes/no                    : choose to build multicast stack feature (default: no)")
	$(call echo_help, " * LBM_CSMA=yes/no                         : choose to build CSMA Feature (default: yes)")
	$(call echo_help, " * USE_CSMA_BY_DEFAULT=yes/no              : in case CSMA is built choose to enable CSMA Feature at start (default: no)")
	$(call echo_help, " * LBM_ALC_SYNC=yes/no                     : choose to build ALCSync service (default: no)")
	$(call echo_help, " * LBM_ALC_SYNC_VERSION=x                  : choose which version of ALCSync package should be compiled (default: 1)")
	$(call echo_help, " * LBM_FUOTA=yes/no                        : choose to build LoRaWAN Packages for FUOTA (default: no)")
	$(call echo_help, " * LBM_FUOTA_VERSION=x                     : choose which version of FUOTA packageq should be compiled (default: 1)")
	$(call echo_help, " * LBM_FUOTA_ENABLE_FMP=yes/no             : in case FUOTA is enabled choose to build LoRaWAN Firmware Management Package (default: yes)")
	$(call echo_help, " * LBM_FUOTA_ENABLE_MPA=yes/no             : in case FUOTA is enabled choose to build LoRaWAN Multi-Package Access Package (default: no)")
	$(call echo_help, " * LBM_ALMANAC=yes/no                      : choose to build Cloud Almanac Update service (default: no)")
	$(call echo_help, " * LBM_STREAM=yes/no                       : choose to build Cloud Stream service (default: no)")
	$(call echo_help, " * LBM_LFU=yes/no                          : choose to build Cloud Large File Upload service (default: no)")
	$(call echo_help, " * LBM_DEVICE_MANAGEMENT=yes/no            : choose to build Cloud Device Management service (default: no)")
	$(call echo_help, " * LBM_GEOLOCATION=yes/no                  : choose to build Geolocation service (default: no)")
	$(call echo_help, " * LBM_STORE_AND_FORWARD=yes/no            : choose to build Store and Forward service (default: no)")
	$(call echo_help, " * LBM_RELAY_TX_ENABLE=yes/no              : choose to build Relay Tx service (default: no)")
	$(call echo_help, " * LBM_RELAY_RX_ENABLE=yes/no              : choose to build Relay Rx service (default: no)")
	$(call echo_help, "")
	$(call echo_help_b, "-------------------- Optional makefile parameters --------------------------")
	$(call echo_help, " * EXTRAFLAGS=xxx                          : Add specific compilation flag for LBM lib build")
	$(call echo_help, " * DEBUG=yes/no                            : -g options for debugging (default: no)")
	$(call echo_help, " * OPT=xxx                                 : Choose compilation optimization  (default: -Os)")
	$(call echo_help, " * MULTITHREAD=yes/no                      : Multithreaded build (default: yes)")
	$(call echo_help, " * VERBOSE=yes/no                          : Increase build verbosity (default: no)")
	$(call echo_help, " * SIZE=yes/no                             : Display size for all objects (default: no)")
	$(call echo_help, "")
	$(call echo_help_b, "---------------------------------------------------------------------------")
	$(call echo_help_b, "All options can also be directly updated in makefiles/options.mk file")



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

ifeq ($(RADIO),sx1272)
-include makefiles/sx127x.mk
endif

ifeq ($(RADIO),sx1276)
-include makefiles/sx127x.mk
endif

#-----------------------------------------------------------------------------
-include makefiles/common.mk

.PHONY: clean_all all help

all: basic_modem_sx128x basic_modem_lr1110 basic_modem_lr1120  basic_modem_lr1121 basic_modem_sx1261 basic_modem_sx1262 basic_modem_sx1272 basic_modem_sx1276

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

clean_lr1121:
	$(MAKE) clean_target RADIO=lr1121

clean_sx1261:
	$(MAKE) clean_target RADIO=sx1261

clean_sx1262:
	$(MAKE) clean_target RADIO=sx1262

clean_sx1268:
	$(MAKE) clean_target RADIO=sx1268

clean_sx1272:
	$(MAKE) clean_target RADIO=sx1272

clean_sx1276:
	$(MAKE) clean_target RADIO=sx1276

clean:
	$(MAKE) clean_target

#-----------------------------------------------------------------------------
# Compilation
#-----------------------------------------------------------------------------
basic_modem:
ifeq ($(RADIO),nc)
	$(call echo_error,"No radio selected! Please specified the target radio  using RADIO=radio_name option")
else
ifneq ($(CRYPTO),SOFT)
ifeq ($(LBM_RELAY_TX_ENABLE),yes)
	$(call echo_error, "------------------------------------------------------------")
	$(call echo_error, "When Relay Tx feature is enable: only soft crypto can be used")	
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "------------------------------------------------------------")
	exit 1
endif
ifeq ($(LBM_RELAY_RX_ENABLE),yes)
	$(call echo_error, "------------------------------------------------------------")
	$(call echo_error, "When Relay Rx feature is enable: only soft crypto can be used")	
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "------------------------------------------------------------")
	exit 1
endif
ifneq ($(NB_OF_STACK),1)
	$(call echo_error, "----------------------------------------------------------")
	$(call echo_error, "More than one stack compiled: only soft crypto can be used")
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "----------------------------------------------------------")
	exit 1
endif
ifneq ($(RADIO),lr1110)
ifneq ($(RADIO),lr1120)
ifneq ($(RADIO),lr1121)
	$(call echo_error, "------------------------------------------------------------")
	$(call echo_error, "sx126x sx127x and sx128x radio tagets: only soft crypto can be used")
	$(call echo_error, "Please use CRYPTO=SOFT option")
	$(call echo_error, "------------------------------------------------------------")
	exit 1
endif
endif
endif
endif
	$(MAKE) basic_modem_build
endif

basic_modem_sx128x:
	$(MAKE) basic_modem RADIO=sx128x $(MTHREAD_FLAG)

basic_modem_lr1110:
	$(MAKE) basic_modem RADIO=lr1110 $(MTHREAD_FLAG)

basic_modem_lr1120:
	$(MAKE) basic_modem RADIO=lr1120 $(MTHREAD_FLAG)

basic_modem_lr1121:
	$(MAKE) basic_modem RADIO=lr1121 $(MTHREAD_FLAG)

basic_modem_sx1261:
	$(MAKE) basic_modem RADIO=sx1261 $(MTHREAD_FLAG)

basic_modem_sx1262:
	$(MAKE) basic_modem RADIO=sx1262 $(MTHREAD_FLAG)

basic_modem_sx1268:
	$(MAKE) basic_modem RADIO=sx1268 $(MTHREAD_FLAG)

basic_modem_sx1272:
	$(MAKE) basic_modem RADIO=sx1272 $(MTHREAD_FLAG)

basic_modem_sx1276:
	$(MAKE) basic_modem RADIO=sx1276 $(MTHREAD_FLAG)
